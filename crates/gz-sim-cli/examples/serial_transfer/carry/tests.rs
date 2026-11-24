// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::EntityKind;

fn offset_fixture(kg: f64, mu: f64, offset_mm: f64) -> Result<String> {
    diagonal_fixture(kg, mu, offset_mm, 0.)
}
fn diagonal_fixture(kg: f64, mu: f64, x_mm: f64, y_mm: f64) -> Result<String> {
    let xml = crate::payload::tests::fixture(kg, mu)?;
    let (workcell, parcel) = xml.split_once("<model name=\"parcel\">").unwrap();
    let x = x_mm / 1000.;
    let y = y_mm / 1000.;
    let parcel = parcel.replacen(
        "<inertial>",
        &format!("<inertial><pose>{x} {y} 0 0 0 0</pose>"),
        1,
    );
    let xml = format!("{workcell}<model name=\"parcel\">{parcel}");
    let sdf = sdformat::load_str(&xml, "carry.sdf", &sdformat::NoResources)?;
    let link = &sdf.worlds[0]
        .models
        .iter()
        .find(|m| m.name == "parcel")
        .unwrap()
        .links[0];
    assert_eq!(link.inertial.pose.transform.translation, [x, y, 0.]);
    Ok(xml)
}
fn world(xml: &str) -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        xml,
        "carry.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn repeated(xml: &str, trial: Trial, ms: u64) -> Result<Outcome> {
    repeated_pull(xml, trial, ms, None)
}
fn repeated_pull(xml: &str, trial: Trial, ms: u64, pull: Option<pull::Pull>) -> Result<Outcome> {
    repeated_clock(xml, trial, ms, 1, pull)
}
fn repeated_clock(
    xml: &str,
    trial: Trial,
    ms: u64,
    control: u64,
    pull: Option<pull::Pull>,
) -> Result<Outcome> {
    let sdf = sdformat::load_str(xml, "carry.sdf", &sdformat::NoResources)?;
    let mut sim = world(xml)?;
    configure_clock(&mut sim, trial, ms, control)?;
    if let Some(p) = pull {
        p.install(&mut sim, trial)?;
    }
    let run = |sim: &mut Simulation| -> Result<_> {
        let route = Passage::bind(sim, &sdf)?;
        let mut frames = Vec::new();
        let outcome = drive(
            sim,
            trial,
            Some(route),
            payload::weight(&sdf)?,
            true,
            |sim| {
                frames.push((
                    sim.elapsed(),
                    sim.world()
                        .iter_entities()
                        .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Visual))
                        .map(|e| {
                            let pose = e.get::<WorldPose>().unwrap();
                            (
                                e.get::<EntityId>().unwrap().get(),
                                pose.translation.map(f64::to_bits),
                                pose.rotation.map(f64::to_bits),
                            )
                        })
                        .collect::<Vec<_>>(),
                ));
                Ok(())
            },
        )?;
        Ok((outcome, frames))
    };
    let cold = run(&mut sim)?;
    sim.reset()?;
    assert_eq!(cold, run(&mut sim)?);
    Ok(cold.0)
}

#[test]
fn offset_carry_recovers_a_light_parcel_and_returns_a_heavier_parcel_to_verified_support()
-> Result<()> {
    let light = offset_fixture(0.4, 0.28, 10.)?;
    let control = repeated(&light, Trial::ObserveCarry, 250)?;
    let caught = repeated(&light, Trial::GuardedCarry, 250)?;
    let a = control.carry.unwrap();
    let b = caught.carry.unwrap();
    assert_eq!(a.first_alarm_ms, b.first_alarm_ms);
    assert!(a.repeated_alarm_ms.is_some() && control.release_tick.is_none());
    assert!(b.extra_jaw_effort_n > 0. && b.repeated_alarm_ms.is_none());
    assert!(caught.release_tick.is_some() && (caught.final_shelf_load_n - 3.924).abs() < 0.001);
    let heavy = repeated(&offset_fixture(1., 0.70, 10.)?, Trial::GuardedCarry, 250)?;
    let h = heavy.carry.unwrap();
    assert_eq!(h.return_station, Some("pickup"));
    assert!(
        h.first_alarm_ms < h.repeated_alarm_ms
            && h.repeated_alarm_ms < h.lowering_ms
            && h.lowering_ms < h.first_recovery_support_ms
            && h.first_recovery_support_ms < h.supported_open_ms
    );
    assert_eq!(h.sustained_at_open_ms, Some(30));
    assert!(h.load_at_open_n.unwrap() >= 0.94 * 9.81);
    assert!(h.parcel_obstacle_impulse_ns == 0. && h.hold_on_blocked_return_m.is_none());
    assert!(
        heavy.final_pad_load_n == 0.
            && heavy.final_shelf_load_n == 0.
            && (heavy.final_tray_load_n - 9.81).abs() < 0.001
    );
    Ok(())
}
#[test]
fn carry_commands_require_the_expected_route_and_clock() -> Result<()> {
    assert!(Args::try_parse_from(["serial", "guarded-carry", "out", "--travel-ms", "0"]).is_err());
    assert!(
        Args::try_parse_from(["serial", "guarded-carry", "out", "--travel-ms", "4001"]).is_err()
    );
    let xml = Trial::GuardedCarry.xml();
    let mut slow = world(&xml.replace("<max_step_size>0.001", "<max_step_size>0.002"))?;
    assert!(configure(&mut slow, Trial::GuardedCarry, 250).is_err());
    let mut sim = world(xml)?;
    assert!(configure(&mut sim, Trial::Transfer, 250).is_err());
    assert!(configure(&mut sim, Trial::ObserveCarry, 0).is_err());
    assert!(
        drive(
            &mut sim,
            Trial::GuardedCarry,
            None,
            WEIGHT,
            true,
            |_| Ok(())
        )
        .is_err()
    );
    let sdf = sdformat::load_str(xml, "carry.sdf", &sdformat::NoResources)?;
    let route = Passage::bind(&sim, &sdf)?;
    assert!(
        drive(
            &mut sim,
            Trial::GuardedCarry,
            Some(route),
            WEIGHT,
            true,
            |_| Ok(())
        )
        .is_err()
    );
    assert_eq!(sim.elapsed(), Duration::ZERO);
    Ok(())
}

#[test]
fn blocked_return_holds_rail_and_spindle_even_if_support_recovers() -> Result<()> {
    let xml = Trial::GuardedCarry.xml();
    let mut sim = world(xml)?;
    configure(&mut sim, Trial::GuardedCarry, 250)?;
    sim.step_exact(1)?;
    let body = |name| part(&sim, name)?.body(&sim);
    let rail = part(&sim, "gripper::travel")?;
    let spindle = part(&sim, "gripper::spindle")?;
    let links = Links {
        body: body("parcel::box")?,
        pads: [body("gripper::left")?, body("gripper::right")?],
        stations: [
            body("stand::post")?,
            body("receiver::shelf")?,
            body("obstacle::bar")?,
        ],
        jaws: [
            part(&sim, "gripper::close_left")?.id,
            part(&sim, "gripper::close_right")?.id,
        ],
        spindle,
        rail,
    };
    let mut supervisor = Supervisor::new(&sim, Trial::GuardedCarry, WEIGHT, links)?;
    supervisor.route = Some(Return {
        tick: MOVE_START,
        start: 0.29,
        station: 1,
        demand: None,
    });
    assert_eq!(
        supervisor.remaining(MOVE_START, 0.29, DISTANCE),
        Some(DISTANCE - 0.29)
    );
    let ready = SupportReading {
        ready: true,
        face_loaded: true,
        ..Default::default()
    };
    let blocked_at = MOVE_START + RETURN_STEPS;
    supervisor.report.lowering_ms = Some(blocked_at - 1);
    supervisor.update(&mut sim, blocked_at, 0.44, [&ready; 2], true)?;
    supervisor.command_rail(&mut sim, rail, blocked_at + 1, DISTANCE, 0.001)?;
    sim.step_exact(1)?;
    supervisor.update(&mut sim, blocked_at + 1, DISTANCE, [&ready; 2], false)?;
    assert_eq!(supervisor.report.hold_on_blocked_return_m, Some(0.44));
    assert_eq!(
        supervisor.remaining(blocked_at + 1, DISTANCE, DISTANCE),
        None
    );
    assert!(supervisor.report.supported_open_ms.is_none());
    sim.world_mut()
        .entity_mut(spindle.entity)
        .remove::<JointSample>();
    assert!(spindle_position(&sim, spindle).is_err());
    sim.world_mut()
        .entity_mut(rail.entity)
        .remove::<JointSample>();
    assert!(
        supervisor
            .update(&mut sim, blocked_at + 2, DISTANCE, [&ready; 2], false)
            .is_err()
    );
    Ok(())
}

#[test]
#[ignore = "full paired offset/acceleration matrix with a reset and all sampled visuals per condition"]
fn serial_offset_matrix() -> Result<()> {
    let output = std::env::var_os("GZ_SERIAL_CARRY_OUTPUT").map(PathBuf::from);
    if let Some(dir) = &output {
        std::fs::create_dir(dir)?;
    }
    for (mass, mu, offsets, times) in [
        (0.4, 0.28, &[-10, 0, 10][..], &[250, 500, 1000, 4000][..]),
        (0.4, 0.85, &[-10, 0, 10][..], &[250, 4000][..]),
        (1., 0.7, &[-10, 10][..], &[250, 4000][..]),
    ] {
        for &offset in offsets {
            let xml = offset_fixture(mass, mu, offset as f64)?;
            for &ms in times {
                for (trial, tag) in [
                    (Trial::GuardedCarry, "guarded"),
                    (Trial::ObserveCarry, "observe"),
                ] {
                    let result = repeated(&xml, trial, ms)?;
                    if let Some(dir) = &output {
                        let name = format!("m{mass}-u{mu}-x{offset:+}-t{ms}-{tag}");
                        std::fs::write(dir.join(format!("{name}.sdf")), &xml)?;
                        std::fs::write(
                            dir.join(format!("{name}.json")),
                            serde_json::to_vec_pretty(&result)?,
                        )?;
                    }
                }
            }
        }
    }
    Ok(())
}

#[test]
fn repeated_slip_selects_the_first_reachable_station_or_holds_both_axes() -> Result<()> {
    let xml = Trial::GuardedCarry.xml();
    for (wall_y, expected) in [(0.1, Some("pickup")), (0., None)] {
        let xml = xml.replace("<pose>0 0.23 0.5535", &format!("<pose>0 {wall_y} 0.5535"));
        let mut sim = world(&xml)?;
        configure(&mut sim, Trial::GuardedCarry, 4000)?;
        sim.step_exact(1)?;
        let body = |name| part(&sim, name)?.body(&sim);
        let links = Links {
            body: body("parcel::box")?,
            pads: [body("gripper::left")?, body("gripper::right")?],
            stations: [
                body("stand::post")?,
                body("receiver::shelf")?,
                body("obstacle::bar")?,
            ],
            jaws: [
                part(&sim, "gripper::close_left")?.id,
                part(&sim, "gripper::close_right")?.id,
            ],
            spindle: part(&sim, "gripper::spindle")?,
            rail: part(&sim, "gripper::travel")?,
        };
        let mut supervisor = Supervisor::new(&sim, Trial::GuardedCarry, WEIGHT, links)?;
        let sdf = sdformat::load_str(&xml, "route.sdf", &sdformat::NoResources)?;
        let route = Passage::bind(&sim, &sdf)?;
        supervisor.route = Some(Return {
            tick: MOVE_START,
            station: 1,
            start: 0.26,
            demand: None,
        });
        supervisor.verify_return(&mut sim, MOVE_START, 0.26, &route)?;
        assert_eq!(supervisor.report.return_station, expected);
        assert_eq!(
            supervisor.report.hold_on_blocked_return_m,
            expected.is_none().then_some(0.26)
        );
    }
    Ok(())
}

#[test]
fn a_late_diagonal_parcel_lands_on_verified_receiver_support() -> Result<()> {
    let xml = diagonal_fixture(0.4, 0.35, 10., 8.)?;
    let command = "6200,65,8".parse::<pull::Pull>()?;
    let guarded = repeated_pull(&xml, Trial::GuardedCarry, 4000, Some(command))?;
    let control = repeated_pull(&xml, Trial::ObserveCarry, 4000, Some(command))?;
    let g = guarded.carry.unwrap();
    assert_eq!(g.first_alarm_ms, control.carry.unwrap().first_alarm_ms);
    assert_eq!(g.return_station, Some("receiver"));
    assert!(g.load_at_open_n.unwrap() >= 0.94 * 3.924 && g.sustained_at_open_ms == Some(30));
    assert!(g.recovery_no_pad_ticks == 0 && g.parcel_obstacle_impulse_ns == 0.);
    assert!(guarded.final_pad_load_n == 0. && (guarded.final_shelf_load_n - 3.924).abs() < 0.001);
    assert!(control.release_tick.is_none());
    let a = serde_json::to_value(guarded.pull.unwrap())?;
    let b = serde_json::to_value(control.pull.unwrap())?;
    assert_eq!(a["incoming"], b["incoming"]);
    assert_eq!(a["first_alarm"], b["first_alarm"]);
    assert_ne!(a["repeated_alarm"], b["repeated_alarm"]);
    Ok(())
}

#[test]
#[ignore = "32 ordinary diagonal offset, slip, and sham trials repeated after reset"]
fn serial_receiver_matrix() -> Result<()> {
    let output = std::env::var_os("GZ_SERIAL_RECEIVER_OUTPUT").map(PathBuf::from);
    if let Some(dir) = &output {
        std::fs::create_dir(dir)?;
    }
    for x in [-10, 10] {
        for y in [-8, 8] {
            let xml = diagonal_fixture(0.4, 0.35, x as f64, y as f64)?;
            for when in [4450, 6200] {
                for force in [0, 8] {
                    let pulse = format!("{when},65,{force}").parse::<pull::Pull>()?;
                    for (trial, tag) in [
                        (Trial::GuardedCarry, "guarded"),
                        (Trial::ObserveCarry, "observe"),
                    ] {
                        let result = repeated_pull(&xml, trial, 4000, Some(pulse))?;
                        if let Some(dir) = &output {
                            let tag = format!("x{x:+}-y{y:+}-t{when}-n{force}-{tag}");
                            std::fs::write(dir.join(format!("{tag}.sdf")), &xml)?;
                            std::fs::write(
                                dir.join(format!("{tag}.json")),
                                serde_json::to_vec_pretty(&result)?,
                            )?;
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

#[test]
#[ignore = "paired release step timings on the diagonal parcel, use fixed CPU affinity"]
fn serial_receiver_step_cost() -> Result<()> {
    let xml = diagonal_fixture(0.4, 0.35, 10., 8.)?;
    let policies = [
        (Trial::GuardedCarry, 0),
        (Trial::ObserveCarry, 8),
        (Trial::GuardedCarry, 8),
    ];
    let mut blocks = Vec::new();
    for order in [[0, 1, 2], [2, 0, 1], [1, 2, 0]] {
        for i in order {
            let (trial, force) = policies[i];
            blocks.push(cost_block(&xml, trial, 1, 6200, 65, force)?);
        }
    }
    if let Some(file) = std::env::var_os("GZ_SERIAL_RECEIVER_COST") {
        std::fs::write(file, serde_json::to_vec_pretty(&blocks)?)?;
    }
    Ok(())
}

fn cost_block(
    xml: &str,
    trial: Trial,
    control_ms: u64,
    first: u64,
    duration: u64,
    force: u8,
) -> Result<serde_json::Value> {
    use bevy_ecs::prelude::{Res, ResMut};
    use gz_sim_core::{Phase, UpdateInfo};
    use std::time::Instant;
    #[derive(Resource, Default)]
    struct Cost {
        prior: Option<Instant>,
        ns: Vec<u64>,
    }
    let sdf = sdformat::load_str(xml, "cost.sdf", &sdformat::NoResources)?;
    let mut sim = world(xml)?;
    configure_clock(&mut sim, trial, 4000, control_ms)?;
    format!("{first},{duration},{force}")
        .parse::<pull::Pull>()?
        .install(&mut sim, trial)?;
    sim.world_mut().init_resource::<Cost>();
    // Consecutive ingress times include the completed Bevy step and the
    // ordinary contact checks, control and capture before the next solve.
    sim.add_systems(
        Phase::Ingress,
        move |clock: Res<UpdateInfo>, mut c: ResMut<Cost>| {
            let ms = clock.start_time.as_millis() as u64;
            if !clock
                .start_time
                .as_micros()
                .is_multiple_of(control_ms as u128 * 1000)
            {
                return;
            }
            let now = Instant::now();
            if let Some(last) = c.prior.replace(now)
                && (6200 + control_ms..=10200 + control_ms).contains(&ms)
            {
                c.ns.push(now.duration_since(last).as_nanos() as u64);
            }
        },
    );
    let route = Passage::bind(&sim, &sdf)?;
    let mut frames = 0;
    let mut captured = Vec::new();
    let o = drive(
        &mut sim,
        trial,
        Some(route),
        payload::weight(&sdf)?,
        true,
        |s| {
            frames += 1;
            captured.push(gz_sim_cli::primitive_frame(s, &sdf, "default", frames)?);
            Ok(())
        },
    )?;
    let mut costs = sim.world_mut().remove_resource::<Cost>().unwrap().ns;
    assert_eq!(costs.len(), 4000 / control_ms as usize + 1);
    costs.sort_unstable();
    Ok(
        serde_json::json!({"mode":format!("{trial:?}"),"downward_n":force,
                "sampled_control_intervals":costs.len(), "p50_ns":costs[costs.len().div_ceil(2)-1],
                "p95_ns":costs[(95*costs.len()).div_ceil(100)-1],
                "p99_ns":costs[(99*costs.len()).div_ceil(100)-1],
                "frames": captured.len(), "shelf_release_ms":o.release_tick,
                "recovery_open_ms":o.carry.unwrap().supported_open_ms}),
    )
}
#[test]
#[ignore = "per-tick independent parcel motion against a matched no-pull trajectory"]
fn serial_receiver_slip_trace() -> Result<()> {
    use bevy_ecs::prelude::{Query, Res, ResMut};
    use gz_sim_core::{Phase, UpdateInfo};
    #[derive(Resource, Default)]
    struct Trace(Vec<glam::DVec3>);
    let mut cases = Vec::new();
    for x in [-10, 10] {
        for y in [-8, 8] {
            for first in [4450, 6200] {
                let xml = diagonal_fixture(0.4, 0.35, x as f64, y as f64)?;
                let sdf = sdformat::load_str(&xml, "motion.sdf", &sdformat::NoResources)?;
                let trace = |trial: Trial, force: u8| -> Result<_> {
                    let mut sim = world(&xml)?;
                    configure(&mut sim, trial, 4000)?;
                    format!("{first},65,{force}")
                        .parse::<pull::Pull>()?
                        .install(&mut sim, trial)?;
                    let hand = part(&sim, "gripper::palm")?.entity;
                    let object = part(&sim, "parcel::box")?.entity;
                    sim.world_mut().init_resource::<Trace>();
                    sim.add_systems(
                        Phase::Finalize,
                        move |clock: Res<UpdateInfo>,
                              poses: Query<&WorldPose>,
                              mut rows: ResMut<Trace>| {
                            if (first - 1..=first + 164).contains(&clock.iteration) {
                                let h = poses.get(hand).unwrap();
                                let p = poses.get(object).unwrap();
                                rows.0.push(
                                    glam::DQuat::from_array(h.rotation).inverse()
                                        * (glam::DVec3::from_array(p.translation)
                                            - glam::DVec3::from_array(h.translation)),
                                );
                            }
                        },
                    );
                    let route = Passage::bind(&sim, &sdf)?;
                    let state = drive(
                        &mut sim,
                        trial,
                        Some(route),
                        payload::weight(&sdf)?,
                        true,
                        |_| Ok(()),
                    )?;
                    let path = sim.world_mut().remove_resource::<Trace>().unwrap().0;
                    assert_eq!(path.len(), 166);
                    Ok((state.carry.unwrap().first_alarm_ms, path))
                };
                let (_, sham) = trace(Trial::GuardedCarry, 0)?;
                let (alarm, guarded) = trace(Trial::GuardedCarry, 8)?;
                let (control_alarm, control) = trace(Trial::ObserveCarry, 8)?;
                let index = alarm.unwrap() - (first - 1) as usize;
                assert_eq!(alarm, control_alarm);
                assert_eq!(sham[0], guarded[0]);
                assert_eq!(guarded[..=index], control[..=index]);
                let relative_mm = |path: &[glam::DVec3]| {
                    path.iter()
                        .zip(&sham)
                        .map(|(p, s)| 1000. * p.distance(*s))
                        .collect::<Vec<_>>()
                };
                let g = relative_mm(&guarded);
                let c = relative_mm(&control);
                assert!(g[index] > 0. && g[165] < c[165]);
                cases.push(serde_json::json!({"x_mm":x,"y_mm":y,"first_ms":first,"first_sample_ms":first-1,"samples":166,
            "alarm_ms":alarm,"alarm_position_difference_mm":g[index],
            "guarded_palm_relative_difference_mm":g,"fixed_palm_relative_difference_mm":c}));
            }
        }
    }
    if let Some(file) = std::env::var_os("GZ_SERIAL_RECEIVER_MOTION") {
        std::fs::write(file, serde_json::to_vec(&cases)?)?;
    }
    Ok(())
}

#[path = "pressure_tests.rs"]
mod pressure_tests;
