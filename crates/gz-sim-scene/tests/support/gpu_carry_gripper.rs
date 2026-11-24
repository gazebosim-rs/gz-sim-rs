// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::gpu_scene::parallel_gripper::{carry, observations, set_population_targets};
const CARRY: &str = include_str!("../../../../examples/gpu_carry_gripper.sdf");
#[test]
fn carry_rejects_incompatible_rails_shelves_and_release_rules() {
    for (a, b, want) in [
        (
            "track=\"track\" shelf",
            "track=\"unknown\" shelf",
            "unknown track joint",
        ),
        (
            "shelf=\"shelf::top\"",
            "shelf=\"unknown::top\"",
            "unknown static support",
        ),
        ("shelf=\"shelf::top\"", "shelf=\"top\"", "model::link"),
        (
            "shelf=\"shelf::top\"",
            "shelf=\"table::base\"",
            "distinct source",
        ),
        (
            "type=\"prismatic\"><parent>world</parent><child>rail",
            "type=\"revolute\"><parent>world</parent><child>rail",
            "world-rooted",
        ),
        (
            "<parent>rail</parent><child>carriage",
            "<parent>world</parent><child>carriage",
            "distinct moving fingertips",
        ),
        (
            "<box><size>0.088 0.12 0.014</size></box>",
            "<sphere><radius>0.012</radius></sphere>",
            "finite receiving box",
        ),
        ("0.088 0.12 0.014", "0.088 0.015 0.014", "transverse width"),
        (
            "0.18 0 0.007 0 0 0",
            "0.18 0 0.014 0 0 0",
            "travel must clear",
        ),
        (
            "<track-drive stiffness",
            "<unknown-drive stiffness",
            "unknown gripper drive element",
        ),
        (
            "after_release=\"true\"",
            "after_release=\"perhaps\"",
            "after_release flag",
        ),
        ("tick=\"571\"", "tick=\"201\"", "duplicate or out-of-range"),
        (
            "track=\"0.18\" lift",
            "track=\"0.31\" lift",
            "duplicate or out-of-range",
        ),
        (
            "weight_fraction=\"0.65\"",
            "weight_fraction=\"-1\"",
            "invalid shelf load",
        ),
        (
            "dwell=\"24\"",
            "dwell=\"0\"",
            "invalid carrying-gripper tick",
        ),
        ("max_gap=\"0.0008\"", "max_gap=\"-1\"", "invalid shelf load"),
        ("right=\"0\"/>", "right=\"-0.03\"/>", "invalid shelf load"),
    ] {
        let s = CARRY.replace(a, b);
        assert_ne!(s, CARRY);
        let error = load(&s, Some("unavailable")).err().unwrap().to_string();
        assert!(error.contains(want), "{a}: {error}");
    }
    for (source, want) in [
        (
            CARRY.replace(
                "      </environment>",
                "      </environment><environment profile=\"carrying-gripper\"/>",
            ),
            "one horizontal carrying gripper",
        ),
        (
            CARRY.replace("model=\"gripper\" object", "model=\"missing\" object"),
            "unknown carrier",
        ),
        (
            CARRY.replace(
                "        <track-drive stiffness=\"175\" damping=\"23\" target=\"0\"/>",
                "",
            ),
            "missing rail drive",
        ),
        (
            CARRY.replace("        <release after", "        <release/><release after"),
            "one shelf release rule",
        ),
        (
            CARRY
                .lines()
                .filter(|l| !l.contains("<release "))
                .collect::<Vec<_>>()
                .join("\n"),
            "missing shelf release rule",
        ),
        (
            CARRY.replace("max_weight_fraction=\"1.4\"", "max_weight_fraction=\"0.3\""),
            "invalid shelf load",
        ),
    ] {
        let e = load(&source, Some("unavailable"))
            .err()
            .unwrap()
            .to_string();
        assert!(e.contains(want), "{e}");
    }
}
#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn normal_carrier_places_on_the_named_shelf_and_repeats_after_reset() {
    let mut sim = load(CARRY, Some(&std::env::var("GZ_GPU_ADAPTER").unwrap())).unwrap();
    assert_eq!(execution_counts(&sim), Some((0, 1)));
    assert!(matches!(
        carry::observe(&sim, 55),
        Err(GpuError::UnknownEnvironment(55))
    ));
    let mut trace = Vec::new();
    for epoch in 0..2 {
        if epoch > 0 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        assert!(carry::set_targets(&mut sim, 12, [0.5, 0., 0., 0.]).is_err());
        carry::set_targets(&mut sim, 12, [0., 0., 0.028, -0.028]).unwrap();
        assert!(set_population_targets(&mut sim, &[]).is_err());
        set_population_targets(&mut sim, &[[0., 0.028, -0.028]]).unwrap();
        assert!(carry::observe(&sim, 12).unwrap().1.is_none());
        for t in 1..=2300 {
            sim.step_exact(1).unwrap();
            if epoch == 1 && t == 1400 {
                sim.set_paused(true);
                let time = sim.elapsed();
                for _ in 0..30 {
                    assert!(sim.tick().unwrap().paused);
                    assert!(carry::observe(&sim, 12).unwrap().1.is_none());
                }
                assert_eq!(sim.elapsed(), time);
            }
            let (l, receipt) = carry::observe(&sim, 12).unwrap();
            assert_eq!(
                observations(&sim).unwrap().next().unwrap().1.ball,
                l.hand.ball
            );
            let reading = sim
                .world()
                .iter_entities()
                .find_map(|e| e.get::<gz_sim_scene::JointStateSample>())
                .unwrap();
            assert_eq!(reading.joints.len(), 4);
            for (j, &(position, speed)) in [
                (l.track, l.track_speed),
                (l.hand.position[0] - 0.03, l.hand.speed[0]),
                (
                    l.hand.position[1] + 0.05 - l.track,
                    l.hand.speed[1] - l.track_speed,
                ),
                (
                    l.hand.position[2] - 0.05 - l.track,
                    l.hand.speed[2] - l.track_speed,
                ),
            ]
            .iter()
            .enumerate()
            {
                assert!((reading.joints[j].position - position).abs() < 1e-12);
                assert!((reading.joints[j].velocity - speed).abs() < 1e-12);
            }
            if (800..=1100).contains(&t) {
                assert!(
                    l.hand.ball[0] > 0.14
                        && l.hand.ball[1] - l.hand.radius > 0.09
                        && l.shelf_normal == 0.
                        && l.hand.normal_impulse[2] == 0.
                );
            }
            if t < 1418 {
                assert!(receipt.is_none());
            } else {
                let s = receipt.unwrap();
                assert_eq!((s.tick, s.sustained_ticks), (1418, 24));
                assert!(s.upward_force_n >= 0.65 * 0.08 * 9.81 && s.clearance_m.abs() < 0.0008);
            }
            if t == 2300 {
                assert!(
                    (l.hand.ball[0] - 0.18).abs() < 0.002
                        && (l.shelf_up / 0.001 - 0.08 * 9.81).abs() < 1e-9
                        && l.hand.normal_impulse == [0.; 3]
                );
            }
            let frame = (
                l.track,
                l.hand.position,
                l.hand.ball,
                l.hand.velocity,
                l.shelf_up,
                receipt,
            );
            if epoch == 0 {
                trace.push(frame);
            } else {
                assert_eq!(trace[t - 1], frame);
            }
        }
    }
    let shelf = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::shelf::top")
        })
        .unwrap()
        .id();
    sim.world_mut()
        .get_mut::<WorldPose>(shelf)
        .unwrap()
        .translation[0] += 0.02;
    assert!(sim.step_exact(1).is_err());
    assert!(carry::observe(&sim, 12).is_err());
}
#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn wrong_destination_and_frictionless_pickup_keep_jaws_closed() {
    let named = std::env::var("GZ_GPU_ADAPTER").unwrap();
    for (s, x) in [
        (CARRY.replace("0.18 0 0.007", "0.34 0 0.007"), 0.18),
        (
            CARRY.replace("<mu>0.6</mu><mu2>0.6", "<mu>0</mu><mu2>0"),
            0.,
        ),
    ] {
        let mut sim = load(&s, Some(&named)).unwrap();
        for _ in 0..2300 {
            sim.step_exact(1).unwrap();
        }
        let (l, r) = carry::observe(&sim, 12).unwrap();
        assert!(r.is_none());
        assert_eq!(l.shelf_normal, 0.);
        if x > 0. {
            assert!(l.hand.ball[1] - l.hand.radius > 0.005 && l.hand.normal_impulse[2] == 0.);
        } else {
            assert!(l.hand.normal_impulse[2] > 0.);
        }
        assert!((l.hand.ball[0] - x).abs() < 0.002);
        assert_eq!(l.hand.target[1], -0.05 + 0.028);
    }
}

const ECCENTRIC: &str = include_str!("../../../../examples/gpu_eccentric_gripper.sdf");
#[test]
fn eccentric_sdf_checks_symmetry_and_physical_mass_properties() {
    for (from, to) in [
        ("0.006 0 0 0 0 0", "0.006 0.001 0 0 0 0"),
        ("0.006 0 0 0 0 0", "0.026 0 0 0 0 0"),
        ("0 0 0.022 0 0 0", "0 0 0.022 0 0 0.2"),
        ("0.000015488", "0.0000245"),
        ("0.000015488</ixx>", "0.000015488</ixx><ixy>0.000001</ixy>"),
    ] {
        let changed = ECCENTRIC.replace(from, to);
        assert_ne!(changed, ECCENTRIC);
        let e = load(&changed, Some("unavailable"))
            .err()
            .unwrap()
            .to_string();
        assert!(e.contains("transverse principal axis"), "{to}: {e}");
    }
}
#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn eccentric_sdf_publishes_geometric_origin_and_measured_pad_loads_across_reset() {
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    for (mu, held) in [(0.6, true), (0.12, false)] {
        let source = ECCENTRIC
            .replace("0.6</mu>", &format!("{mu}</mu>"))
            .replace("0.6</mu2>", &format!("{mu}</mu2>"));
        let mut sim = load(&source, Some(&adapter)).unwrap();
        let mut previous = Vec::new();
        for epoch in 0..2 {
            if epoch == 1 {
                reset_loaded_world(&mut sim, |_| ()).unwrap();
            }
            let entity = sim
                .world()
                .iter_entities()
                .find(|e| {
                    e.get::<Name>()
                        .is_some_and(|n| n.0 == "default::parcel::ball")
                })
                .unwrap()
                .id();
            let mut average = [0.; 2];
            for t in 1..=1000 {
                sim.step_exact(1).unwrap();
                let h = carry::observe(&sim, 12).unwrap().0.hand;
                let pose = sim.world().get::<WorldPose>(entity).unwrap();
                let velocity = sim.world().get::<gz_sim_scene::Velocity>(entity).unwrap();
                let (s, c) = (-h.angle * 0.5).sin_cos();
                assert_eq!(pose.translation, [h.ball[0], 0., h.ball[1]]);
                assert!(
                    (pose.rotation[1] - s).abs() < 1e-12 && (pose.rotation[3] - c).abs() < 1e-12
                );
                assert_eq!(velocity.linear, [h.velocity[0], 0., h.velocity[1]]);
                assert_eq!(velocity.angular, [0., -h.spin, 0.]);
                if t >= 801 {
                    for (a, imp) in average.iter_mut().zip(h.support_impulse) {
                        *a += imp / 0.2;
                    }
                    if held {
                        assert_eq!(h.normal_impulse[2], 0.);
                        assert!(h.ball[1] > 0.117);
                    } else {
                        assert!(h.normal_impulse[2] > 0. && (h.ball[1] - h.radius).abs() < 1e-6);
                    }
                }
                let state = (
                    h.ball,
                    h.velocity,
                    h.angle,
                    h.spin,
                    h.support_impulse,
                    h.normal_impulse,
                );
                if epoch == 0 {
                    previous.push(state);
                } else {
                    assert_eq!(previous[t - 1], state);
                }
            }
            if held {
                assert!((average[0] - 0.2855).abs() < 0.003 && (average[1] - 0.4993).abs() < 0.003);
            } else {
                assert_eq!(average, [0.; 2]);
            }
        }
    }
}

#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn carrying_solver_resolves_the_rotated_authored_mass_frame() {
    let a = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let rotated = ECCENTRIC
        .replace("0 0 0.022 0 0 0", "0 0 0.022 0 1.5707963267948966 0")
        .replace("0.006 0 0 0 0 0", "0 0 0.006 0.37 0 0");
    let mut plain = load(ECCENTRIC, Some(&a)).unwrap();
    let mut world = load(&rotated, Some(&a)).unwrap();
    let (l, _) = carry::observe(&world, 12).unwrap();
    assert!((l.com_offset[0] - 0.006).abs() < 1e-15 && l.com_offset[1].abs() < 1e-15);
    assert!((l.angular_inertia - 0.000015488).abs() < 1e-15);
    for _ in 0..500 {
        plain.step_exact(1).unwrap();
        world.step_exact(1).unwrap();
        let reference = carry::observe(&plain, 12).unwrap().0.hand;
        let actual = carry::observe(&world, 12).unwrap().0.hand;
        for (x, y) in actual
            .ball
            .into_iter()
            .chain([actual.angle, actual.spin])
            .zip(
                reference
                    .ball
                    .into_iter()
                    .chain([reference.angle, reference.spin]),
            )
        {
            assert!((x - y).abs() < 1e-8);
        }
    }
}

const BOX_CARRY: &str = include_str!("../../../../examples/gpu_box_gripper.sdf");
#[test]
fn finite_box_requires_transverse_symmetry_and_inertia_inside_its_shape() {
    for (from, to, want) in [
        (
            "object=\"parcel\"",
            "object=\"table\"",
            "dynamic free payload",
        ),
        ("0 0 0.020 0 0 0", "0 0 0.020 0.1 0 0", "transverse axis"),
        ("0.048 0.024 0.040", "0.048 0.08 0.040", "covering"),
        ("0.0000192</izz>", "0.000037</izz>", "mass moments"),
        ("0 0 0.020 0 0 0", "0 0 0.015 0 0 0", "clear the shelf"),
        ("0 0 0.020 0 0 0", "0.18 0 0.02 0 0 0", "clear the shelf"),
    ] {
        let source = BOX_CARRY.replace(from, to);
        assert_ne!(source, BOX_CARRY);
        let err = load(&source, Some("unavailable"))
            .err()
            .unwrap()
            .to_string();
        assert!(err.contains(want), "{to}: {err}");
    }
}
#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn ordinary_box_carry_reports_both_face_impulses_and_resets() {
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut sim = load(BOX_CARRY, Some(&adapter)).unwrap();
    let mut first = Vec::new();
    for epoch in 0..2 {
        if epoch != 0 {
            reset_loaded_world(&mut sim, |_| ()).unwrap();
        }
        let mut paired = [0; 2];
        let mut support = 0.;
        for tick in 1..=1000 {
            sim.step_exact(1).unwrap();
            let (l, release) = carry::observe(&sim, 12).unwrap();
            assert!(release.is_none());
            assert_eq!(l.box_half, [0.024, 0.020]);
            assert_eq!(l.hand.radius, 0.);
            for s in 0..4 {
                let expected = [
                    l.hand.normal_impulse[0],
                    l.hand.normal_impulse[1],
                    l.hand.normal_impulse[2],
                    l.shelf_normal,
                ][s];
                assert!((l.point_impulse[s].iter().sum::<f64>() - expected).abs() < 1e-13);
            }
            for (n, imp) in paired.iter_mut().zip(l.point_impulse) {
                if imp[0].min(imp[1]) > 1e-9 {
                    *n += 1;
                }
            }
            if tick >= 801 {
                let center = l.hand.ball[0];
                let half = l.payload_reach()[0];
                let left = l.hand.position[1] + l.hand.half_size[0][0];
                let right = l.hand.position[2] - l.hand.half_size[1][0];
                assert!(
                    ((center - half) - left).abs() < 1e-7 && (right - (center + half)).abs() < 1e-7
                );
                support += l.hand.support_impulse.iter().sum::<f64>() / 0.2;
                assert!(
                    l.hand.ball[1] - l.payload_reach()[1] > 0.09 && l.hand.normal_impulse[2] == 0.
                );
            }
            let frame = (l.hand.ball, l.hand.angle, l.hand.spin, l.point_impulse);
            if epoch == 0 {
                first.push(frame)
            } else {
                assert_eq!(first[tick - 1], frame);
            }
        }
        assert!(paired.into_iter().all(|v| v > 850));
        assert!((support - 0.08 * 9.81).abs() < 0.0001);
    }
}

#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn a_carried_cuboid_settles_on_the_finite_named_shelf_before_opening() {
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut sim = load(BOX_CARRY, Some(&adapter)).unwrap();
    let mut initial = *carry::observe(&sim, 12).unwrap().0;
    initial.hand.spin = 2.;
    assert!((initial.settling_speed() - 0.048).abs() < 1e-14);
    let mut open = None;
    for t in 1..=2300 {
        sim.step_exact(1).unwrap();
        let (l, r) = carry::observe(&sim, 12).unwrap();
        if let Some(r) = r
            && open.is_none()
        {
            assert_eq!(r.tick, t);
            assert_eq!(r.sustained_ticks, 24);
            assert!(
                r.upward_force_n >= 0.65 * 0.08 * 9.81 && r.upward_force_n <= 1.4 * 0.08 * 9.81
            );
            assert!(r.clearance_m.abs() < 0.0008);
            open = Some(t);
        }
        if t >= 2101 {
            let [width, height] = l.payload_reach();
            assert!((l.hand.ball[0] - l.shelf_center[0]).abs() + width < l.shelf_half[0]);
            assert!((l.hand.ball[1] - height - 0.014).abs() < 1e-7);
            assert!((l.shelf_up / 0.001 - 0.08 * 9.81).abs() < 1e-5);
            assert_eq!(l.hand.normal_impulse, [0.; 3]);
        }
    }
    assert!(open.is_some_and(|t| (1300..1600).contains(&t)));
}
