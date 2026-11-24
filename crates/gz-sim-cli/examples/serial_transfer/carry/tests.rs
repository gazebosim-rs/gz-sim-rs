// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::EntityKind;

fn offset_fixture(kg: f64, mu: f64, offset_mm: f64) -> Result<String> {
    let xml = crate::payload::tests::fixture(kg, mu)?;
    let (workcell, parcel) = xml.split_once("<model name=\"parcel\">").unwrap();
    let offset = offset_mm / 1000.;
    let parcel = parcel.replacen(
        "<inertial>",
        &format!("<inertial><pose>{offset} 0 0 0 0 0</pose>"),
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
    assert_eq!(link.inertial.pose.transform.translation, [offset, 0., 0.]);
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
    let sdf = sdformat::load_str(xml, "carry.sdf", &sdformat::NoResources)?;
    let mut sim = world(xml)?;
    configure(&mut sim, trial, ms)?;
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
