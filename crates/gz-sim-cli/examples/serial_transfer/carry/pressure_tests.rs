// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn fixture(physics_us: u64, mu: f64) -> Result<String> {
    Ok(diagonal_fixture(0.4, mu, 10., 8.)?.replace(
        "<max_step_size>0.001<",
        &format!("<max_step_size>{}<", physics_us as f64 / 1e6),
    ))
}
#[test]
fn lower_friction_shelf_edge_and_three_clocks() -> Result<()> {
    for (us, control, when, duration) in
        [(500, 2, 6201, 64), (1000, 1, 6200, 65), (2000, 2, 6201, 64)]
    {
        let xml = fixture(us, 0.28)?;
        let pull = format!("{when},{duration},8").parse()?;
        let out = repeated_clock(&xml, Trial::GuardedCarry, 4000, control, Some(pull))?;
        let payload = serde_json::to_value(out.payload.as_ref().unwrap())?;
        for i in 0..2 {
            assert!(
                payload["mean_pad_normal_n"][i].as_f64().unwrap()
                    >= payload["min_pad_normal_n"][i].as_f64().unwrap()
            );
        }
        let carry = out.carry.unwrap();
        assert!(carry.supported_open_ms.is_some() && carry.sustained_at_open_ms == Some(30));
        assert_eq!(carry.return_station, Some("receiver"));
        assert!(out.final_pad_load_n == 0. && (out.final_shelf_load_n - 3.924).abs() < 0.001);
        assert!(carry.load_at_open_n.unwrap() >= 0.94 * 3.924);
        if us != 2000 {
            let lift = carry.shelf_edge_lifts.last().unwrap();
            assert!(lift.start_gaps_m[1] < 0. && lift.resume_gaps_m.unwrap()[1] > 0.);
            assert!(lift.clear_for_ms >= 30 && lift.peak_spindle_position_rad <= 2.);
        }
    }
    let mut coarse = world(&fixture(2000, 0.28)?)?;
    for bad in ["6200,64,8", "6201,65,8"] {
        assert!(
            bad.parse::<pull::Pull>()?
                .install(&mut coarse, Trial::GuardedCarry)
                .is_err()
        );
    }
    assert!(cycle::Cadence::new(Duration::from_micros(750), 2).is_err());
    assert!(cycle::Cadence::new(Duration::from_millis(1), 3).is_err());
    Ok(())
}
fn pressure_matrix(us: u64, control: u64, start: u64, duration: u64) -> Result<()> {
    let out = std::env::var_os("GZ_SERIAL_PRESSURE_OUTPUT").map(PathBuf::from);
    if let Some(p) = &out {
        std::fs::create_dir_all(p)?;
    }
    for (mu, n) in [(0.28, 0), (0.28, 6), (0.28, 8), (0.28, 12), (0.35, 8)] {
        if control == 1 && mu == 0.35 {
            continue;
        }
        let xml = fixture(us, mu)?;
        let cmd = format!("{start},{duration},{n}").parse::<pull::Pull>()?;
        for (mode, tag) in [
            (Trial::ObserveCarry, "observe"),
            (Trial::DirectCarry, "direct"),
            (Trial::GuardedCarry, "guarded"),
        ] {
            let o = repeated_clock(&xml, mode, 4000, control, Some(cmd))?;
            if let Some(p) = &out {
                let name = format!("p{us}-c{control}-u{mu}-n{n}-t{start}-d{duration}-{tag}");
                std::fs::write(p.join(format!("{name}.sdf")), &xml)?;
                std::fs::write(
                    p.join(format!("{name}.json")),
                    serde_json::to_vec_pretty(&o)?,
                )?;
            }
        }
    }
    Ok(())
}

#[test]
#[ignore = "twelve physical reaction, control and reset comparisons"]
fn serial_pressure_matrix_historical() -> Result<()> {
    pressure_matrix(1000, 1, 6200, 65)
}
#[test]
#[ignore = "fifteen physical reaction, control and reset comparisons"]
fn serial_pressure_matrix_half_ms() -> Result<()> {
    pressure_matrix(500, 2, 6201, 64)
}
#[test]
#[ignore = "fifteen physical reaction, control and reset comparisons"]
fn serial_pressure_matrix_one_ms() -> Result<()> {
    pressure_matrix(1000, 2, 6201, 64)
}
#[test]
#[ignore = "fifteen physical reaction, control and reset comparisons"]
fn serial_pressure_matrix_two_ms() -> Result<()> {
    pressure_matrix(2000, 2, 6201, 64)
}

#[test]
#[ignore = "independent native contact point forces, and oriented parcel bounds"]
fn serial_pressure_force_audit() -> Result<()> {
    use bevy_ecs::prelude::{Query, Res, ResMut};
    use gz_sim_core::{Phase, UpdateInfo};
    #[derive(Resource, Default)]
    struct Audit {
        rows: Vec<[f64; 6]>,
    }
    let mut cases = Vec::new();
    for (us, control, first, duration, n) in [
        (1000, 1, 6200, 65, 8),
        (500, 2, 6201, 64, 8),
        (1000, 2, 6201, 64, 8),
        (2000, 2, 6201, 64, 8),
        (1000, 1, 6200, 65, 12),
    ] {
        let xml = fixture(us, 0.28)?;
        let sdf = sdformat::load_str(&xml, "pressure.sdf", &sdformat::NoResources)?;
        let mut sim = world(&xml)?;
        configure_clock(&mut sim, Trial::GuardedCarry, 4000, control)?;
        format!("{first},{duration},{n}")
            .parse::<pull::Pull>()?
            .install(&mut sim, Trial::GuardedCarry)?;
        let parcel = part(&sim, "parcel::box")?;
        let body = parcel.body(&sim)?;
        let shelf = part(&sim, "receiver::shelf")?;
        let station = shelf.body(&sim)?;
        let jaws = [
            part(&sim, "gripper::left")?.body(&sim)?,
            part(&sim, "gripper::right")?.body(&sim)?,
        ];
        // Independent geometry uses the fixture's eight rectangular corners in the reported link pose.
        let half = glam::DVec3::new(0.030, 0.0275, 0.035);
        sim.world_mut().insert_resource(Audit::default());
        sim.add_systems(
            Phase::Finalize,
            move |physics: Res<PhysicsBackend>,
                  clock: Res<UpdateInfo>,
                  poses: Query<&WorldPose>,
                  mut audit: ResMut<Audit>| {
                let solver = physics.downcast_ref::<MultiFidelityBackend>().unwrap();
                let mut up = 0.;
                let mut total = 0.;
                let mut fingers = [0.; 2];
                for patch in solver.rigid_contact_patches() {
                    let (peer, sign) = if patch.a == body {
                        (patch.b, 1.)
                    } else if patch.b == body {
                        (patch.a, -1.)
                    } else {
                        continue;
                    };
                    let ns = patch.points.iter().map(|p| p.normal_impulse).sum::<f64>();
                    if peer == station {
                        up += sign * patch.normal_on_a[2] * ns;
                        total += ns;
                    }
                    if let Some(i) = jaws.iter().position(|j| *j == peer) {
                        fingers[i] += ns;
                    }
                }
                let parcel = poses.get(parcel.entity).unwrap();
                let shelf = poses.get(shelf.entity).unwrap();
                let q = glam::DQuat::from_array(parcel.rotation);
                let top = shelf.translation[2] + 0.006;
                let mut gap = f64::INFINITY;
                for x in [-half.x, half.x] {
                    for y in [-half.y, half.y] {
                        for z in [-half.z, half.z] {
                            gap = gap.min(
                                parcel.translation[2] + (q * glam::DVec3::new(x, y, z)).z - top,
                            );
                        }
                    }
                }
                let h = clock.dt.as_secs_f64();
                audit.rows.push([
                    up / h,
                    total / h,
                    fingers[0] / h,
                    fingers[1] / h,
                    gap,
                    parcel.translation[2] - half.length() - top,
                ]);
            },
        );
        let route = Passage::bind(&sim, &sdf)?;
        let o = drive(
            &mut sim,
            Trial::GuardedCarry,
            Some(route),
            payload::weight(&sdf)?,
            true,
            |_| Ok(()),
        )?;
        let c = o.carry.unwrap();
        let Audit { rows } = sim.world_mut().remove_resource::<Audit>().unwrap();
        let native_per_500 = 500000 / us as usize;
        assert_eq!(rows.len(), 18_000_000 / us as usize);
        let mean = |slice: &[[f64; 6]], col: usize| {
            slice.iter().map(|r| r[col]).sum::<f64>() / slice.len() as f64
        };
        let tail = &rows[rows.len() - native_per_500..];
        assert!((mean(tail, 0) - o.final_shelf_load_n).abs() < 1e-9);
        assert!((mean(tail, 2) + mean(tail, 3) - o.final_pad_load_n).abs() < 1e-9);
        let at_ms = |ms: usize| rows[ms * 1000 / us as usize - 1];
        let open = c.supported_open_ms.map(|ms| {
            let end = ms * 1000 / us as usize;
            let window = &rows[end - 30000 / us as usize..end];
            let normal = mean(window, 0);
            assert!((normal - c.load_at_open_n.unwrap()).abs() < 1e-9);
            assert!((0.94 * 3.924..=1.25 * 3.924).contains(&normal));
            assert!(
                window
                    .iter()
                    .all(|r| { r[1] > 0. && r[0] >= 0.999 * r[1] && r[0] >= 0.2 * 3.924 })
            );
            serde_json::json!({
                "ms": ms,
                "mean_upward_n": normal,
                "min_upward_n": window.iter().map(|r| r[0]).fold(f64::INFINITY, f64::min),
                "max_upward_n": window.iter().map(|r| r[0]).fold(0., f64::max),
                "native_solves": window.len()
            })
        });
        let mut lifts = Vec::new();
        for lift in &c.shelf_edge_lifts {
            let start = at_ms(lift.first_contact_ms);
            assert!((start[4] - lift.start_gaps_m[0]).abs() < 1e-12);
            assert!((start[5] - lift.start_gaps_m[1]).abs() < 1e-12);
            let end = lift.first_contact_ms * 1000 / us as usize;
            let incoming = &rows[end - control as usize * 1000 / us as usize..end];
            assert!((mean(incoming, 0) - lift.initial_mean_upward_force_n).abs() < 1e-8);
            assert!((mean(incoming, 1) - lift.initial_mean_total_normal_force_n).abs() < 1e-8);
            let clear = lift.resume_ms.map(|ms| {
                let end = ms * 1000 / us as usize;
                let window = &rows[end - 30000 / us as usize..end];
                let pose = at_ms(ms);
                let reported = lift.resume_gaps_m.unwrap();
                assert!((pose[4] - reported[0]).abs() < 1e-12);
                assert!((pose[5] - reported[1]).abs() < 1e-12);
                assert!(window.iter().all(|r| r[1] == 0. && r[2] > 0.1 && r[3] > 0.1));
                serde_json::json!({
                    "ms": ms, "bottom_gap_m": pose[4], "all_orientation_gap_m": pose[5],
                    "max_shelf_normal_n": window.iter().map(|r| r[1]).fold(0., f64::max),
                    "min_individual_jaw_n": window.iter().map(|r| r[2].min(r[3])).fold(f64::INFINITY, f64::min)
                })
            });
            lifts.push(serde_json::json!({
                "ms": lift.first_contact_ms,
                "start_sample": start,
                "mean_incoming_upward_n": mean(incoming, 0),
                "mean_incoming_total_normal_n": mean(incoming, 1),
                "clear": clear
            }));
        }
        cases.push(serde_json::json!({
            "physics_us": us, "control_ms": control, "force_n": n,
            "open_window": open, "lifts": lifts,
            "final_upward_n": mean(tail, 0), "final_jaw_n": [mean(tail, 2), mean(tail, 3)]
        }));
    }
    if let Some(file) = std::env::var_os("GZ_SERIAL_PRESSURE_AUDIT") {
        std::fs::write(file, serde_json::to_vec_pretty(&cases)?)?;
    }
    Ok(())
}

#[test]
#[ignore = "nine ordinary recorded trials with interleaved physics steps at fixed control frequency"]
fn serial_pressure_step_cost() -> Result<()> {
    let mut rows = Vec::new();
    for order in [[500, 1000, 2000], [2000, 500, 1000], [1000, 2000, 500]] {
        for us in order {
            let xml = fixture(us, 0.28)?;
            let mut block = cost_block(&xml, Trial::GuardedCarry, 2, 6201, 64, 8)?;
            block["physics_us"] = us.into();
            rows.push(block);
        }
    }
    if let Some(file) = std::env::var_os("GZ_SERIAL_PRESSURE_COST") {
        std::fs::write(file, serde_json::to_vec_pretty(&rows)?)?;
    }
    Ok(())
}
