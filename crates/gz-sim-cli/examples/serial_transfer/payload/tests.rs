// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

// The canonical workcell has precisely three 0.85 surfaces (parcel and both pads).
// Check this and all authored body/material values after the substitution.
pub(crate) fn fixture(kg: f64, mu: f64) -> Result<String> {
    let xml = Trial::ClearObstacle.xml();
    let xml = configure(xml, Some(kg), Some(mu))?.into_owned();
    let axes = [0.00016, 0.00014, 0.00013].map(|i| i * (kg / 0.2));
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

#[test]
fn mass_and_friction_overrides_preserve_other_surfaces_and_the_full_inertia_tensor() -> Result<()> {
    let source = Trial::ClearObstacle.xml().replace(
        "<izz>0.00013</izz>",
        "<izz>0.00013</izz><ixy>-0.00001</ixy><ixz>0</ixz><iyz>0.00002</iyz>",
    );
    assert!(matches!(configure(&source, None, None)?, Cow::Borrowed(_)));
    let changes = configure(&source, Some(0.4), Some(0.125))?;
    assert!(changes.contains("<ixy>-0.00002</ixy><ixz>0</ixz><iyz>0.00004</iyz>"));
    assert_eq!(changes.matches("<mu>0.125</mu><mu2>0.125</mu2>").count(), 3);
    assert_eq!(
        changes.matches("<mu>0.75</mu>").count(),
        source.matches("<mu>0.75</mu>").count()
    );
    for (mass, friction) in [
        (Some(0.), None),
        (Some(f64::NAN), None),
        (None, Some(-0.1)),
        (None, Some(f64::INFINITY)),
    ] {
        assert!(configure(&source, mass, friction).is_err());
    }
    // A custom SDF must bind the requested properties to the intended, fully specified faces.
    for broken in [
        "<sdf".to_owned(),
        source.replace("name=\"default\"", "name=\"unknown\""),
        source.replace("name=\"parcel\"", "name=\"unknown\""),
        source.replace("inertial>", "unknown_inertial>"),
        source.replace("mass>", "unknown_mass>"),
        source.replace("<mass>0.2</mass>", "<mass>not a number</mass>"),
        source.replace("inertia>", "unknown_inertia>"),
        source.replace("<ixx>0.00016</ixx>", "<ixx/>"),
        source.replace("name=\"right\"", "name=\"unknown\""),
        source.replace("collision name=\"box\"", "collision name=\"unknown\""),
        source.replace("ode>", "unknown_ode>"),
        source.replace("<mu2>0.85</mu2>", ""),
        source.replace("name=\"left\"", "name=\"unknown\""),
        source.replace("<iyy>0.00014</iyy>", ""),
        source.replace("<mass>0.2</mass>", "<mass>0</mass>"),
        source.replace("<ixx>0.00016</ixx>", "<ixx>1.0e308</ixx>"),
        source.replace("<mu2>0.85</mu2>", "<mu2/>"),
    ] {
        assert!(configure(&broken, Some(0.4), Some(0.2)).is_err());
    }
    Ok(())
}

#[test]
#[ignore = "validates the serial_transfer executable from a built local CPU preview"]
fn installed_workcell_places_a_loaded_parcel_and_stops_at_an_unsecured_grasp() -> Result<()> {
    let install = std::env::var_os("GZ_TEST_LOCAL_CPU_INSTALL")
        .ok_or("set GZ_TEST_LOCAL_CPU_INSTALL to the extracted preview")?;
    let binary = PathBuf::from(install).join("bin/serial_transfer");
    let dir = tempfile::tempdir()?;
    let execute = |label: &str, args: &[&str]| -> Result<serde_json::Value> {
        let archive = dir.path().join(label);
        let output = std::process::Command::new(&binary)
            .arg("clear-obstacle")
            .arg(&archive)
            .args(args)
            .output()?;
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        assert!(archive.join("frames.jsonl").is_file());
        Ok(serde_json::from_slice(&output.stdout)?)
    };
    let placed = execute("placed", &["--parcel-kg", "1"])?;
    assert_eq!(
        placed["parcel_carried_on_both_pads_ticks"],
        placed["traverse_ticks"]
    );
    assert!(placed["release_tick"].is_u64());
    assert!(placed["dwell_ms_at_release"].as_u64().unwrap() >= 30);
    assert!((placed["final_shelf_load_n"].as_f64().unwrap() - 9.81).abs() < 0.001);
    assert!(
        placed["obstacle"]["parts"]
            .as_object()
            .unwrap()
            .values()
            .all(|part| part["normal_impulse_ns"] == 0.
                && part["minimum_gap_m"].as_f64().unwrap() > 0.)
    );
    let rejected = execute("rejected", &["--grip-friction", "0"])?;
    assert_eq!(rejected["obstacle"]["unsecured_dispatch_tick"], MOVE_START);
    assert_eq!(rejected["obstacle"]["abort_tick"], MOVE_START);
    assert!(rejected["release_tick"].is_null());
    // The interlock holds the rail at pickup; the carriage can deflect while the jaws close.
    assert!(
        rejected["lateral_carriage_at_arrival_mm"]
            .as_f64()
            .unwrap()
            .abs()
            < 0.1
    );
    assert!((rejected["final_tray_load_n"].as_f64().unwrap() - 1.962).abs() < 0.001);
    let invalid = dir.path().join("bad");
    let output = std::process::Command::new(binary)
        .arg("clear-obstacle")
        .arg(&invalid)
        .args(["--parcel-kg", "0"])
        .output()?;
    assert!(!output.status.success() && !invalid.exists());
    assert!(
        String::from_utf8_lossy(&output.stderr).contains("parcel mass must be finite and positive")
    );
    Ok(())
}
