// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Real command-line loading of local contact geometry and deformable measurements.
use std::{
    fs,
    path::Path,
    process::{Command, Output},
};

const PRIMARY: &str = include_str!("../../../examples/multifidelity_finite_primary.sdf");
const PROJECTION: &str = "<project_finite_point>true</project_finite_point>";

fn run(file: &Path, ticks: u32) -> Output {
    Command::new(env!("CARGO_BIN_EXE_gz-sim-rs"))
        .arg(file)
        .args([
            "--physics-engine",
            "multifidelity",
            "--iterations",
            &ticks.to_string(),
        ])
        .output()
        .unwrap()
}

fn assert_refused(file: &Path, input: &str, message: &str) {
    fs::write(file, PRIMARY.replace(PROJECTION, input)).unwrap();
    let output = run(file, 1);
    assert_eq!(output.status.code(), Some(1), "accepted {input}");
    assert!(
        output.stdout.is_empty(),
        "published state after rejecting {input}"
    );
    assert!(
        String::from_utf8_lossy(&output.stderr).contains(message),
        "{input}: {}",
        String::from_utf8_lossy(&output.stderr)
    );
}

#[test]
fn a_contact_pair_requires_explicit_boolean_choices() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("rocker.sdf");
    for tag in [
        "certify_face_vertex",
        "certify_segment_friction_work",
        "certify_skew_edge",
        "compact_impact_budget",
        "compact_point_tail",
        "detect_finite_arrival",
        "detect_initial_point",
        "detect_separated_crossings",
        "follow_frictionless_deck_edge",
        "follow_frictionless_edge",
        "follow_frictionless_face",
        "follow_planar_edge",
        "follow_rigid_departure",
        "follow_skew_slip_launch",
        "follow_sliding_skew_edge",
        "follow_sliding_vertex",
        "follow_sticking_point",
        "guard_separated_flight",
        "integrate_ballistic_translation",
        "integrate_free_rotation",
        "integrate_released_rotation",
        "project_corner",
        "project_edge",
        "project_finite_point",
        "project_frictionless_sole",
        "project_mixed_edge",
        "project_opponent_corner",
        "project_opponent_return",
        "project_opposing_normals",
        "project_planar",
        "project_segment_endpoint",
        "project_sliding_edge",
        "prospective_timing",
        "refine_active_point",
        "refine_rotation_timing",
        "refresh_box_features",
        "refresh_returning_box_features",
        "resolve_fractional_points",
        "resolve_point_transfers",
        "resolve_rocking_limit",
        "retime_finite_transfer",
        "settle_comoving_supports",
        "share_coplanar_face",
        "share_uniform_sliding",
        "subdivide_point_bounds",
        "target_point_bounds",
    ] {
        assert_refused(
            &path,
            &format!("<{tag}>sometimes</{tag}>"),
            &format!("{tag} must be true or false"),
        );
    }
}

#[test]
fn contact_continuation_requires_the_geometry_and_timing_it_uses() {
    let directory = tempfile::tempdir().unwrap();
    let path = directory.path().join("rocker.sdf");
    for message in [
        "project_opposing_normals requires project_opponent_corner",
        "project_opponent_return requires project_opponent_corner",
        "project_opponent_corner requires guard_opponent",
        "follow_rigid_departure requires project_rigid_corner",
        "compact_impact_budget requires detect_separated_crossings and measured rotation_contact_pulse",
        "detect_separated_crossings requires refine_rotation_timing",
        "refine_rotation_timing requires integrate_free_rotation",
        "integrate_released_rotation requires integrate_free_rotation",
        "integrate_ballistic_translation requires integrate_free_rotation",
        "integrate_free_rotation requires guard_separated_flight",
        "guard_separated_flight requires prospective_timing",
        "resolve_fractional_points requires project_finite_point",
        "certify_skew_edge requires project_finite_point",
        "certify_face_vertex requires project_finite_point",
        "follow_skew_slip_launch requires project_finite_point or project_segment_endpoint",
        "follow_sliding_skew_edge requires project_finite_point or project_segment_endpoint",
        "target_point_bounds requires subdivide_point_bounds",
        "detect_initial_point requires resolve_point_transfers",
        "refine_active_point requires resolve_point_transfers",
        "compact_point_tail requires resolve_point_transfers",
        "subdivide_point_bounds requires resolve_point_transfers",
        "resolve_point_transfers requires project_finite_point or project_segment_endpoint",
        "follow_sticking_point requires project_finite_point or project_segment_endpoint",
        "follow_sliding_vertex requires project_finite_point or project_segment_endpoint",
        "retime_finite_transfer requires project_finite_point or project_segment_endpoint",
        "certify_segment_friction_work requires project_segment_endpoint",
        "settle_comoving_supports requires share_uniform_sliding",
        "share_uniform_sliding requires share_coplanar_face",
        "share_coplanar_face requires follow_frictionless_face, with no other projection, timing or deck-edge continuation",
        "follow_frictionless_deck_edge requires follow_frictionless_face",
        "resolve_rocking_limit requires project_frictionless_sole and follow_frictionless_edge",
        "follow_frictionless_edge requires project_edge or project_frictionless_sole",
        "follow_planar_edge requires project_planar",
    ] {
        let tag = message.split_once(' ').unwrap().0;
        assert_refused(&path, &format!("<{tag}>true</{tag}>"), message);
    }
    assert_refused(
        &path,
        &format!("{PROJECTION}<project_segment_endpoint>true</project_segment_endpoint>"),
        "choose one projected contact geometry per pair",
    );
}

#[test]
fn offline_local_contact_outputs_match_the_physical_reference() {
    let path =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples/finite_material_cable.sdf");
    // Before touching the roller, both cable particles travel downward at 1.58 m/s
    // in zero gravity and the guide's centre travels with its authored velocity.
    for ticks in [0, 4, 20] {
        let o = run(&path, ticks);
        assert!(o.status.success(), "{}", String::from_utf8_lossy(&o.stderr));
        let output = String::from_utf8(o.stdout).unwrap();
        let mut lines = output.lines();
        assert_eq!(
            lines.next().unwrap(),
            format!(
                "simulation_ns={} iterations={ticks}",
                u64::from(ticks) * 2_000_000
            )
        );
        let t = f64::from(ticks) * 0.002;
        let guide: Vec<f64> = lines
            .next()
            .unwrap()
            .strip_prefix("default::roller::guide ")
            .unwrap()
            .split_ascii_whitespace()
            .map(|n| n.parse().unwrap())
            .collect();
        assert_eq!(guide.len(), 3);
        let expected = [0.04 * t, -0.03 * t, 0.6 + 0.02 * t];
        for (a, b) in guide.iter().zip(expected) {
            assert!((a - b).abs() < 1e-12);
        }
        let height: f64 = lines
            .next()
            .unwrap()
            .strip_prefix("deformable material particles=2 min_z=")
            .unwrap()
            .parse()
            .unwrap();
        assert!((height - (0.92 - 1.58 * t)).abs() < 1e-12);
        assert_eq!(lines.next(), None);
    }
}

#[test]
fn finite_rocker_strike_publishes_the_solved_link_pose_at_the_outer_clock() {
    use gz_sim_scene::{Name, PhysicsEngine, WorldPose, load_world_with_sdf_step, primary_contact};
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../examples");
    let output = run(&root.join("multifidelity_finite_primary.sdf"), 1);
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8(output.stdout).unwrap();
    assert_eq!(
        stdout.lines().next(),
        Some("simulation_ns=500000 iterations=1")
    );

    let mut world = load_world_with_sdf_step(
        PRIMARY,
        "rocker.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    world.step_exact(1).unwrap();
    let hits = &primary_contact::last_refinement(&world)
        .unwrap()
        .interval
        .finite_point_impacts;
    assert_eq!(hits.len(), 1);
    assert_eq!(hits[0].resultant.elapsed.as_nanos(), 328543);
    assert!(hits[0].resultant.impulse_on_foot[2] > 0.8);
    let pose = world
        .world_mut()
        .query::<(&Name, &WorldPose)>()
        .iter(world.world())
        .find(|(name, _)| name.0 == "default::rocker::foot")
        .unwrap()
        .1
        .translation;
    let reported: Vec<f64> = stdout
        .lines()
        .find_map(|l| l.strip_prefix("default::rocker::foot "))
        .unwrap()
        .split_ascii_whitespace()
        .map(|value| value.parse().unwrap())
        .collect();
    assert_eq!(reported.as_slice(), pose);
}
