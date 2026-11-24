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
