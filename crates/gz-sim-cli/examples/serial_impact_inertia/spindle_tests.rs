// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::primary_contact::last_refinement;

fn study(extra: &[&str]) -> Result<Run> {
    let args = [
        "study",
        "--automatic",
        "--stop-guard",
        "--substeps",
        "1000",
        "--divisor",
        "16",
    ]
    .into_iter()
    .chain(extra.iter().copied());
    run(Args::try_parse_from(args)?.trial)
}
#[test]
fn concentrated_relative_inertia_drives_the_clock_in_flight_and_through_both_stops() -> Result<()> {
    for (args, sign) in [
        (&[][..], 1.),
        (&["--right"][..], 1.),
        (&["--miss"][..], 1.),
        (&["--passive-spindle", "capped"][..], 1.),
        (
            &["--passive-spindle", "capped", "--reverse-torque"][..],
            -1.,
        ),
    ] {
        let r = study(args)?;
        let expected = if r.trial.miss { 147 } else { 1000 };
        assert_eq!(r.samples[0].native_steps, expected);
        assert!(r.samples[1..].iter().all(|s| s.native_steps == 147));
        for s in &r.samples {
            let v = sign * s.joint_velocity[1];
            assert!(v > 0. && v <= 0.802, "signed speed {v}");
            // Gravity and the world rail have no Y impulse. The parcel and
            // the complete five-link robot form a closed momentum balance on Y.
            let py = LINKS
                .iter()
                .zip(&s.bodies)
                .map(|((_, m, _), b)| m * b.v[1])
                .sum::<f64>()
                + 0.2 * s.bodies[5].v[1];
            assert!(py.abs() < 1e-8, "transverse momentum {py}");
        }
        if r.trial.passive_spindle.is_some() {
            assert!(sign * r.samples.last().unwrap().joint_velocity[1] > 0.79);
        }
    }
    Ok(())
}
#[test]
fn standalone_stop_budget_resets_and_drops_removed_bindings() -> Result<()> {
    let trial = Args::try_parse_from([
        "study",
        "--automatic",
        "--stop-guard",
        "--divisor",
        "16",
        "--substeps",
        "32",
    ])?
    .trial;
    let xml = fixture(&trial);
    let without_pairs = xml
        .lines()
        .filter(|s| !s.contains("<pair>"))
        .collect::<Vec<_>>()
        .join("\n");
    let load = |x: &str| {
        load_world_with_sdf_step(
            x,
            "spindle.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
    };
    for child in ["", "default::gripper::missing", "default::gripper::wrist"] {
        assert!(
            load(&without_pairs.replace(
                ">default::gripper::spindle</speed_stop_joint>",
                &format!(">{child}</speed_stop_joint>")
            ))
            .is_err()
        );
    }
    for invalid in [
        xml.replace("<incoming_face>-x</incoming_face>", ""),
        xml.replace("<velocity>0.8</velocity>", ""),
        xml.replace("<effort>3.0</effort>", "<effort>0</effort>"),
        xml.replace("<velocity>0.8</velocity>", "<velocity>0</velocity>"),
    ] {
        assert!(load(&invalid).is_err());
    }
    let tag = "<speed_stop_joint>default::gripper::spindle</speed_stop_joint>";
    assert!(load(&without_pairs.replace(tag, "")).is_err());
    let mut passive = trial.clone();
    passive.passive_spindle = Some(PassiveSpindle::Capped);
    let xml = fixture(&passive);
    for (field, zero) in [
        ("<velocity>0.8</velocity>", "<velocity>0</velocity>"),
        ("<effort>3.0</effort>", "<effort>0</effort>"),
    ] {
        let invalid = xml.replace(field, zero);
        let sim = load(&invalid.replace(tag, ""))?;
        let joint = sim
            .world()
            .get::<gz_sim_scene::JointHandle>(entity(&sim, "gripper::spindle"))
            .unwrap();
        let native = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        assert_eq!(native.screw_stop_resolution_rate(joint.id()), None);
        assert!(load(&invalid).is_err());
    }
    let mut sim = load(&without_pairs)?;
    let model = sim
        .world()
        .get::<EntityId>(entity(&sim, "gripper"))
        .unwrap()
        .get();
    let missing = gz_physics::JointId::from_raw(u64::MAX);
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .screw_stop_resolution_rate(missing),
        None
    );
    sim.step_exact(1)?;
    assert_eq!(last_refinement(&sim).unwrap().interval.native_steps, 32);
    sim.reset()?;
    assert!(last_refinement(&sim).is_none());
    sim.step_exact(1)?;
    assert_eq!(last_refinement(&sim).unwrap().interval.native_steps, 32);
    gz_sim_scene::remove_model(&mut sim, model)?;
    sim.step_exact(1)?;
    assert!(last_refinement(&sim).is_none_or(|r| r.sim_time < sim.elapsed()));
    Ok(())
}
#[test]
fn nominal_inertia_keeps_the_ordinary_clock_and_the_first_native_state_can_be_matched() -> Result<()>
{
    let miss = run(Args::try_parse_from([
        "study",
        "--divisor",
        "1",
        "--automatic",
        "--substeps",
        "1000",
        "--stop-guard",
        "--miss",
        "--ticks",
        "2",
    ])?
    .trial)?;
    assert!(miss.samples.iter().all(|s| s.native_steps == 1));
    let impact = study(&["--ticks", "2"])?;
    let fixed = run(Args::try_parse_from([
        "study",
        "--divisor",
        "16",
        "--first-native-steps",
        "1000",
        "--substeps",
        "1",
        "--ticks",
        "2",
    ])?
    .trial)?;
    assert_eq!(impact.samples[0].bodies, fixed.samples[0].bodies);
    assert_ne!(impact.samples[1].bodies, fixed.samples[1].bodies);
    let measured = compare::against_fine(&impact)?;
    assert!(measured.primary_vs_fine.max_translation_mm < 0.1);
    Ok(())
}

#[test]
fn stop_rate_projects_both_authored_inertias_onto_the_actual_screw_axis() -> Result<()> {
    let trial = Args::try_parse_from(["study"])?.trial;
    let text = fixture(&trial);
    for (rotation, axis) in [("0", 2), ("1.5707963267948966", 0)] {
        let xml = text.replace(
            "<inertial><mass>0.7</mass>",
            &format!("<inertial><pose>0 0 0 0 {rotation} 0</pose><mass>0.7</mass>"),
        );
        let sim = load_world_with_sdf_step(
            &xml,
            "tensor.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )?;
        let j = sim
            .world()
            .get::<gz_sim_scene::JointHandle>(entity(&sim, "gripper::spindle"))
            .unwrap()
            .id();
        let rate = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap()
            .screw_stop_resolution_rate(j)
            .unwrap();
        let parent = tensor(LINKS[0].1, LINKS[0].2, 1.);
        let rotor = tensor(LINKS[1].1, LINKS[1].2, 1.);
        let expected = (parent[2].recip() + rotor[axis].recip()) * 3. / 0.8 / 8.;
        assert!((rate - expected).abs() < 1e-9);
    }
    Ok(())
}
