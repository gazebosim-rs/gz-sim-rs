// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

// The canonical workcell has precisely three 0.85 surfaces (parcel and both pads).
// Check this and all authored body/material values after the substitution.
pub(crate) fn fixture(kg: f64, mu: f64) -> Result<String> {
    let xml = Trial::ClearObstacle.xml();
    let material = "<mu>0.85</mu><mu2>0.85</mu2>";
    let inertia = "<mass>0.2</mass><inertia><ixx>0.00016</ixx><iyy>0.00014</iyy><izz>0.00013</izz>";
    assert_eq!(xml.matches(material).count(), 3);
    assert_eq!(xml.matches(inertia).count(), 1);
    let axes = [0.00016, 0.00014, 0.00013].map(|i| i * (kg / 0.2));
    let [x, y, z] = axes;
    let xml = xml
        .replace(material, &format!("<mu>{mu}</mu><mu2>{mu}</mu2>"))
        .replace(
            inertia,
            &format!("<mass>{kg}</mass><inertia><ixx>{x}</ixx><iyy>{y}</iyy><izz>{z}</izz>"),
        );
    let sdf = sdformat::load_str(&xml, "payload.sdf", &sdformat::NoResources)?;
    let world = &sdf.worlds[0];
    for (model, link) in [("parcel", "box"), ("gripper", "left"), ("gripper", "right")] {
        let body = world
            .models
            .iter()
            .find(|m| m.name == model)
            .unwrap()
            .links
            .iter()
            .find(|l| l.name == link)
            .unwrap();
        let f = &body.collisions[0].surface.friction.ode;
        assert_eq!((f.mu, f.mu2), (mu, mu));
        if model == "parcel" {
            assert_eq!(body.inertial.mass, kg);
            assert_eq!(body.inertial.inertia[..3], axes);
        }
    }
    Ok(xml)
}

fn replay_reset(xml: &str, trial: Trial) -> Result<Outcome> {
    let sdf = sdformat::load_str(xml, "payload.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        xml,
        "payload.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let run = |sim: &mut Simulation| -> Result<_> {
        let passage = Passage::bind(sim, &sdf)?;
        drive(sim, trial, Some(passage), weight(&sdf)?, true, |_| Ok(()))
    };
    let cold = run(&mut sim)?;
    sim.reset()?;
    assert_eq!(cold, run(&mut sim)?);
    Ok(cold)
}

#[test]
fn loaded_transfer_obstructed_retreat_and_empty_jaw_interlock_repeat_on_reset() -> Result<()> {
    let loaded = replay_reset(&fixture(1.245, 0.85)?, Trial::ClearObstacle)?;
    assert_eq!(
        loaded.parcel_carried_on_both_pads_ticks,
        loaded.traverse_ticks
    );
    assert!(loaded.release_tick.is_some());
    assert!((loaded.final_shelf_load_n - 1.245 * 9.81).abs() < 0.001);
    let route = loaded.obstacle.unwrap();
    assert!(
        route
            .parts
            .values()
            .all(|p| p.minimum_gap_m > 0. && p.normal_impulse_ns == 0.)
    );
    let heavy = replay_reset(&fixture(1.24, 0.85)?, Trial::CachedObstacle)?;
    let route = heavy.obstacle.unwrap();
    assert!(route.abort_on_measured_contact && route.first_obstacle_reaction_tick.is_some());
    assert!(heavy.release_tick.is_none());
    let slick = replay_reset(&fixture(0.2, 0.)?, Trial::ClearObstacle)?;
    let route = slick.obstacle.unwrap();
    assert_eq!(route.unsecured_dispatch_tick, Some(MOVE_START));
    assert_eq!(route.abort_tick, Some(MOVE_START));
    assert!(slick.lateral_carriage_at_arrival_mm.abs() < 0.1 && slick.release_tick.is_none());
    assert!((slick.final_tray_load_n - 0.2 * 9.81).abs() < 0.001);
    Ok(())
}

#[test]
fn missing_workcell_and_parcel_have_no_assumed_weight() -> Result<()> {
    for (needle, message) in [
        ("default", "missing serial workcell"),
        ("parcel", "missing parcel inertial"),
    ] {
        let xml = Trial::ClearObstacle
            .xml()
            .replace(&format!("name=\"{needle}\""), "name=\"unknown\"");
        let sdf = sdformat::load_str(&xml, "payload.sdf", &sdformat::NoResources)?;
        assert_eq!(weight(&sdf).unwrap_err().to_string(), message);
    }
    Ok(())
}

#[test]
#[ignore = "full 26-case payload study, each from fresh state and again after reset"]
fn payload_matrix() -> Result<()> {
    let dir = std::env::var_os("GZ_SERIAL_PAYLOAD_OUTPUT").map(PathBuf::from);
    if let Some(dir) = &dir {
        std::fs::create_dir(dir)?;
    }
    for (mass, coefficients) in [
        (0.2, &[0., 0.125, 0.15, 0.25, 0.30, 0.35, 0.85][..]),
        (0.4, &[0.25, 0.27, 0.2725, 0.275, 0.28, 0.85][..]),
        (1.0, &[0.65, 0.70, 0.85][..]),
        (1.15, &[0.85][..]),
        (1.20, &[0.85][..]),
        (1.21, &[0.85][..]),
        (1.22, &[0.85][..]),
        (1.23, &[0.85][..]),
        (1.24, &[0.85][..]),
        (1.245, &[0.85][..]),
        (1.2475, &[0.85][..]),
        (1.25, &[0.85][..]),
        (1.40, &[0.85][..]),
    ] {
        for &mu in coefficients {
            let xml = fixture(mass, mu)?;
            let measured = replay_reset(&xml, Trial::ClearObstacle)?;
            if let Some(dir) = &dir {
                let name = format!("m{mass:.4}-u{mu:.4}");
                std::fs::write(dir.join(format!("{name}.sdf")), xml)?;
                std::fs::write(
                    dir.join(format!("{name}.json")),
                    serde_json::to_vec_pretty(&measured)?,
                )?;
            }
        }
    }
    Ok(())
}
