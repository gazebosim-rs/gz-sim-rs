// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn a_public_receiver_wrench_accepts_the_stable_overhang_only_after_twenty_loaded_ticks() {
    let half = [0.1, 0.09, 0.06];
    let policy = FiniteDeckMargin::new(half, 0.002, glam::DQuat::IDENTITY.to_array()).unwrap();
    for (shift, expected) in [
        (80, Outcome::Resting),
        (95, Outcome::Held),
        (165, Outcome::Held),
    ] {
        let xml = world(16, true, true).unwrap();
        let mut r = Recording::new(&xml, "pressure.sdf", "default").unwrap();
        let (rows, samples) = measure(&mut r, shift, &mut None, Some(policy)).unwrap();
        let result = summarize(&rows, &samples, shift, 16, half[2], MS, false).unwrap();
        assert_eq!(result.outcome, expected);
        assert_eq!(r.costs_ns.len(), END);
        let mut confirmed_ms = 0;
        let mut previous_opening = 0;
        let mut vetoed_weight = 0;
        let mut last_center_y = 0.;
        for (row, s) in rows.iter().zip(&samples) {
            let advanced = row.opening_ms > previous_opening;
            if advanced {
                assert!(confirmed_ms >= 20);
            }
            previous_opening = row.opening_ms;
            let [jx, jy, jz] = s.deck_impulse_ns;
            let [kx, ky, _] = s.deck_moment_nms;
            // Independent explicit line/plane intersection. Subtract the actual
            // deck travel to express the same world point in the ending footprint.
            let interior = jz > 0. && {
                let x = (half[2] * jx - ky) / jz;
                let y = (half[2] * jy + kx) / jz;
                x.abs() <= half[0] - 0.002
                    && y.abs() <= (half[1] - 0.002)
                    && (y - s.deck_y_m + last_center_y).abs() <= (half[1] - 0.002)
            };
            assert_eq!(s.receiver_inside_margin, Some(interior));
            last_center_y = s.deck_y_m;
            if row.tick >= 2550 {
                let valid_load = (0.8 * WEIGHT..=1.5 * WEIGHT).contains(&row.table_n);
                if valid_load && !interior {
                    vetoed_weight += 1;
                }
                confirmed_ms = if valid_load && interior {
                    confirmed_ms + 1
                } else {
                    0
                };
            }
        }
        if shift == 80 {
            assert_eq!(previous_opening, 350);
            assert!((result.mean_final_table_n - WEIGHT).abs() < 1e-10);
        } else {
            assert_eq!(previous_opening, 0);
            assert!(rows.last().unwrap().grip_n.iter().all(|n| *n > 0.7));
            if shift == 95 {
                assert!(vetoed_weight > 1000);
                check_custody_after_withdrawal(&mut r, rows.last().unwrap());
            } else {
                assert_eq!(result.mean_final_table_n, 0.);
                assert!(rows.last().unwrap().search_exhausted);
            }
        }
    }
}

#[test]
fn an_invalid_requested_inset_cannot_publish_a_robot_episode() {
    for inset in [0., -1., 90., f64::NAN] {
        let tmp = tempfile::tempdir().unwrap();
        let out = tmp.path().join("run");
        assert!(
            write_new(&out, |path| run(path, 95, 16, true, false, Some(inset)))
                .unwrap_err()
                .to_string()
                .contains("release margin")
        );
        assert!(!out.exists());
    }
}

fn check_custody_after_withdrawal(r: &mut Recording, last: &Reading) {
    let deck = entity_id(&r.sim, "default::source::body").unwrap();
    let controller = Controller::with_jaw_damping(&r.sim, 1.5).unwrap();
    let command = SetdownCommand {
        lift_offset_m: last.lift_offset_m,
        ..Default::default()
    };
    gz_sim_scene::queue_link_world_velocity(&mut r.sim, deck, [0., 0.25, 0.], [0.; 3]).unwrap();
    let mut final_pad_ns = 0.;
    let mut final_deck_ns = 0.;
    for tick in 0..600 {
        if tick == 400 {
            gz_sim_scene::queue_link_world_velocity(&mut r.sim, deck, [0.; 3], [0.; 3]).unwrap();
        }
        drive(
            &controller,
            &mut r.sim,
            4.3 + f64::from(tick) * 0.001,
            Mode::Finite,
            command,
        )
        .unwrap();
        r.advance().unwrap();
        if tick >= 500 {
            let b = arm_sheet::cloth(&r.sim).unwrap().last_boundary_impulses();
            for pad in &b[..2] {
                assert!(pad.linear[0].hypot(pad.linear[1]) > 0.0005);
                final_pad_ns -= pad.linear[2];
            }
            final_deck_ns -= b[2].linear[2];
        }
    }
    let center = state(&r.sim, deck).unwrap().position;
    let points = arm_sheet::cloth(&r.sim).unwrap().particles();
    let clearance = center[1]
        - 0.09
        - points
            .iter()
            .map(|p| p.position[1] + arm_sheet::RADIUS)
            .fold(f64::NEG_INFINITY, f64::max);
    assert!(
        clearance > 0.02,
        "finite deck lateral clearance {clearance}"
    );
    assert_eq!(final_deck_ns, 0.);
    assert!(
        (final_pad_ns / 0.1 - WEIGHT).abs() < 0.02 * WEIGHT,
        "final finger force {}",
        final_pad_ns / 0.1
    );
    println!(
        "withdrawn deck clearance {clearance} m, final upward jaw load {} N, deck load {} N",
        final_pad_ns / 0.1,
        final_deck_ns / 0.1
    );
}

#[test]
fn the_sensor_reads_the_authored_collision_frame_and_requires_one_receiving_face() {
    let xml = world(16, true, true).unwrap();
    let start = xml.find("<model name=\"source\">").unwrap();
    let mut rotated = xml.clone();
    let col = start
        + xml[start..].find("<collision name=\"body\">").unwrap()
        + "<collision name=\"body\">".len();
    rotated.insert_str(col, "<pose>0.01 0.02 0 0 0 1.5707963267948966</pose>");
    let mut r = Recording::new(&rotated, "yawed.sdf", "default").unwrap();
    let half = arm_cable::half_size(&r.sdf, "source", "body").unwrap();
    let policy = receiver_margin(&r, half, 2.).unwrap();
    let j = DVec3::Z * 0.001;
    assert!(policy.accepts(
        j.to_array(),
        DVec3::new(0., 0.095, 0.06).cross(j).to_array(),
        [0.; 3]
    ));
    assert!(!policy.accepts(
        j.to_array(),
        DVec3::new(0.095, 0., 0.06).cross(j).to_array(),
        [0.; 3]
    ));
    let model = r.sdf.worlds[0]
        .models
        .iter_mut()
        .find(|m| m.name == "source")
        .unwrap();
    model.links[0].collisions.clear();
    assert!(receiver_margin(&r, half, 2.).is_err());
    let model = r.sdf.worlds[0]
        .models
        .iter_mut()
        .find(|m| m.name == "source")
        .unwrap();
    model.links.clear();
    assert!(receiver_margin(&r, half, 2.).is_err());
    r.sdf.worlds[0].models.retain(|m| m.name != "source");
    assert!(receiver_margin(&r, half, 2.).is_err());
}
