// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn positive_tensors_have_a_realizable_mass_and_survive_native_loading() -> Result<()> {
    for input in ["0", "1025", "NaN", "junk"] {
        assert!(Args::try_parse_from(["study", "--divisor", input]).is_err());
    }
    for d in ["1", "1.25", "5"] {
        let args = Args::try_parse_from(["study", "--ticks", "1", "--divisor", d])?;
        let sdf = sdformat::load_str(
            &fixture(&args.trial),
            "serial-impact.sdf",
            &sdformat::NoResources,
        )?;
        let arm = sdf.worlds[0]
            .models
            .iter()
            .find(|m| m.name == "gripper")
            .unwrap();
        for (name, mass, dims) in LINKS {
            let link = arm.links.iter().find(|link| link.name == name).unwrap();
            assert_eq!(link.inertial.mass, mass);
            assert_eq!(link.inertial.pose.transform.translation, [0.; 3]);
            assert!(
                matches!(link.collisions[0].geometry, sdformat::Geometry::Box {size} if size==dims)
            );
        }
        let r = run(args.trial)?;
        assert!((r.native_robot_mass_kg - 2.46).abs() < 1e-12);
        for (i, row) in r.native_left_tensor_kg_m2.iter().enumerate() {
            assert!((row[i] - r.tensors_kg_m2[3][i]).abs() < 1e-15);
        }
        for t in r.tensors_kg_m2 {
            for i in 0..3 {
                assert!(t[i] > 0. && t[i] < t[(i + 1) % 3] + t[(i + 2) % 3]);
            }
        }
    }
    Ok(())
}

#[test]
fn touching_setup_matches_and_native_impulse_explains_parcel_momentum() -> Result<()> {
    let args = Args::try_parse_from(["study", "--ticks", "2"])?;
    let r = run(args.trial)?;
    let p = &r.incoming[5];
    assert!((p.xyz[0] + 0.03 - (r.incoming[3].xyz[0] - 0.009)).abs() < 1e-14);
    assert_eq!(p.v, [0.75, 0., 0.]);
    let compare = compare::against_fine(&r)?;
    assert!((0.04..0.05).contains(&compare.primary.left_ns));
    assert!(compare.primary_vs_fine.impulse_percent.unwrap() < 5.);
    assert!(compare.primary_vs_fine.max_translation_mm < 0.5);
    let m = 0.2 * (p.v[0] - r.samples[0].bodies[5].v[0]);
    assert!((m - r.samples[0].impact.left_ns).abs() < 1e-12);
    Ok(())
}

#[test]
fn missed_strike_leaves_a_ballistic_parcel() -> Result<()> {
    let r =
        run(Args::try_parse_from(["study", "--miss", "--ticks", "2", "--substeps", "8"])?.trial)?;
    let c = compare::against_fine(&r)?;
    assert_eq!(c.primary.left_ns, 0.);
    assert!(c.primary_vs_fine.impulse_percent.is_none());
    for s in r.samples {
        let p = &s.bodies[5];
        let dt = f64::from(s.ms) * 0.001;
        assert_eq!(p.v[0], 0.75);
        assert!((p.v[2] + 9.81 * dt).abs() < 1e-12);
        assert!((p.xyz[0] + 0.099 - 0.75 * dt).abs() < 1e-12);
    }
    Ok(())
}

#[test]
fn refined_native_impact_handles_the_concentrated_mass_control() -> Result<()> {
    let args = Args::try_parse_from([
        "study",
        "--divisor",
        "1.75",
        "--substeps",
        "1000",
        "--ticks",
        "2",
    ])?;
    let r = run(args.trial)?;
    let c = compare::against_fine(&r)?;
    assert!(c.primary_vs_fine.impulse_percent.unwrap() < 3.);
    assert!(c.primary_vs_fine.max_translation_mm < 0.1);
    assert!(c.primary_vs_fine.max_rotation_deg < 0.2);
    Ok(())
}

#[test]
fn refining_the_extreme_mass_distribution_keeps_the_driven_axes_below_their_speed_caps()
-> Result<()> {
    let r = run(Args::try_parse_from(["study", "--divisor", "16", "--substeps", "64"])?.trial)?;
    assert!(r.samples[0].impact.left_ns > 0.);
    for s in r.samples {
        assert!(s.joint_velocity[1].abs() <= 0.8);
        assert!(s.joint_velocity[5].abs() <= 0.6);
    }
    Ok(())
}

#[test]
fn both_approach_faces_select_only_the_strike_and_the_miss_spends_no_extra_steps() -> Result<()> {
    for option in ["--right", "--miss", "--automatic"] {
        let mut args = Args::try_parse_from([
            "study",
            "--divisor",
            "1.75",
            "--substeps",
            "1000",
            "--ticks",
            "4",
            option,
        ])?;
        args.trial.automatic = true;
        let r = run(args.trial)?;
        let selected = r.samples.iter().map(|s| s.native_steps).collect::<Vec<_>>();
        if option == "--miss" {
            assert_eq!(selected, [1; 4]);
        } else {
            assert_eq!(selected, [1000, 1, 1, 1]);
            let i = &r.samples[0].impact;
            assert!(if option == "--right" {
                i.right_ns > 0.045 && i.left_ns == 0.
            } else {
                i.left_ns > 0.045 && i.right_ns == 0.
            });
            let normal = 0.2 * (r.samples[0].bodies[5].v[0] - r.incoming[5].v[0]);
            assert!((normal - i.normal_on_parcel_ns[0]).abs() < 1e-9);
        }
    }
    Ok(())
}

#[test]
fn authored_faces_rebind_after_reset_and_reject_mixed_modes() -> Result<()> {
    let args = Args::try_parse_from(["study", "--automatic", "--substeps", "8"])?;
    let xml = fixture(&args.trial);
    let load = |text: &str| {
        load_world_with_sdf_step(
            text,
            "impact.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
    };
    for axis in ["+x", "-x", "+y", "-y", "+z", "-z"] {
        load(&xml.replace("<incoming_face>-x", &format!("<incoming_face>{axis}")))?;
    }
    for invalid in [
        "q</incoming_face>",
        "-x</incoming_face><project_planar>true</project_planar>",
    ] {
        assert!(load(&xml.replace("-x</incoming_face>", invalid)).is_err());
    }
    let mut sim = load(&xml)?;
    let mut impulses = Vec::new();
    for _ in 0..2 {
        sim.step_exact(1)?;
        let r = gz_sim_scene::primary_contact::last_refinement(&sim).unwrap();
        assert_eq!(r.requested_pairs.len(), 1);
        impulses.push(r.requested_pairs[0].normal_impulse);
        sim.reset()?;
        assert!(gz_sim_scene::primary_contact::last_refinement(&sim).is_none());
    }
    assert_eq!(impulses[0], impulses[1]);
    Ok(())
}

#[test]
fn removing_the_velocity_motor_still_leaves_a_physical_spindle_stop() -> Result<()> {
    let max_speed = |cap: &str| -> Result<f64> {
        let args = Args::try_parse_from([
            "study",
            "--passive-spindle",
            cap,
            "--automatic",
            "--substeps",
            "64",
        ])?;
        let r = run(args.trial)?;
        Ok(r.samples
            .iter()
            .map(|s| s.joint_velocity[1].abs())
            .fold(0., f64::max))
    };
    assert!(max_speed("capped")? < 0.81);
    assert!(max_speed("uncapped")? > 3.);
    Ok(())
}

#[test]
fn a_predicted_arrival_without_load_does_not_spend_the_next_strike() -> Result<()> {
    let r = run(Args::try_parse_from([
        "study",
        "--automatic",
        "--divisor",
        "1.75",
        "--substeps",
        "1000",
        "--gap-um",
        "1200",
        "--ticks",
        "3",
    ])?
    .trial)?;
    let steps = r.samples.iter().map(|s| s.native_steps).collect::<Vec<_>>();
    assert_eq!(steps, [1, 1000, 1000]);
    assert_eq!(r.samples[1].impact.left_ns, 0.);
    assert!(r.samples[2].impact.left_ns > 0.02);
    Ok(())
}

#[test]
fn timing_compares_every_native_body_without_changing_the_actuator_clock() -> Result<()> {
    for options in [
        &[][..],
        &["--automatic", "--substeps", "4"][..],
        &["--passive-spindle", "capped", "--substeps", "4"][..],
    ] {
        let a = Args::try_parse_from(
            ["impact", "--ticks", "2", "--timing-runs", "2"]
                .into_iter()
                .chain(options.iter().copied()),
        )?;
        let r = run(a.trial)?;
        let timed = serde_json::to_value(timing::measure(&r, 2)?)?;
        assert_eq!(
            timed["native_calls_per_tick"],
            serde_json::to_value(r.samples.iter().map(|s| s.native_steps).collect::<Vec<_>>())?
        );
        let rows = timed["step_ns_by_run"].as_array().unwrap();
        assert_eq!(rows.len(), 2);
        for row in rows {
            let ns = row.as_array().unwrap();
            assert_eq!(ns.len(), 2);
            assert!(ns.iter().all(|t| t.as_u64().unwrap() > 0));
        }
    }
    Ok(())
}

#[test]
fn failed_native_timing_never_returns_a_partial_latency_sample() -> Result<()> {
    let mut reference = run(Args::try_parse_from(["study", "--ticks", "1"])?.trial)?;
    // The native grid cannot represent more intervals than nanoseconds.
    reference.trial.substeps = u32::MAX;
    assert!(timing::measure(&reference, 1).is_err());
    // Admission must also fail before timing when the SDF policy is invalid.
    reference.trial.substeps = 0;
    reference.trial.automatic = true;
    assert!(timing::measure(&reference, 1).is_err());
    assert!(run(reference.trial).is_err());
    Ok(())
}
