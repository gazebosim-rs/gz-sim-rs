// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::super::tests::sample;
use super::*;

fn held() -> Branch {
    Branch {
        outcome: Outcome {
            baseline: 0,
            weight: 0.08,
            anchor: json!({"q":1}),
            trace: vec![sample(); HORIZON + 1],
        },
        actions: Recovery::default(),
    }
}

#[test]
fn recovery_reports_the_original_slip_and_bench_independently() {
    let mut branch = held();
    assert_eq!(metrics(&branch)["outcome"], "retained");
    for t in &mut branch.outcome.trace[50..] {
        t.pad_frame_y_m = [-0.00026, 0.00024];
        t.jaw_up_n = [0.; 2];
    }
    assert_eq!(metrics(&branch)["first_250um_tick"], 50);
    assert_eq!(metrics(&branch)["outcome"], "unsettled");
    branch.actions.open = Some(790);
    for t in &mut branch.outcome.trace[800..] {
        t.bench_up_n = 0.08;
    }
    assert_eq!(metrics(&branch)["outcome"], "placed_on_bench");
    branch.actions.escaped = Some(99);
    assert_eq!(metrics(&branch)["outcome"], "lost_aloft");
}

#[test]
fn unguarded_control_detects_airborne_loss_without_driving_the_jaws() {
    let previous = sample();
    let mut lost = previous.clone();
    lost.jaw_squeeze_n = [0.; 2];
    let mut actions = Recovery::default();
    for k in 1..6 {
        actions.observe(k, &previous, &lost, 0.075, 0.08, false);
    }
    assert_eq!(actions.lost_aloft, Some(1));
    assert_eq!(actions.tighten, None);
    assert_eq!(actions.lower, None);
    assert_eq!(actions.open, None);
}

#[test]
fn comparisons_require_the_same_physical_anchor_and_tactile_history() {
    let guard = held();
    let mut control = held();
    let ab = compare(5, 1100, &guard, &control).unwrap();
    assert_eq!(ab["matching_postgrip_ticks"], HORIZON);
    control.outcome.trace[61].command_stroke_m = 0.079;
    assert_eq!(
        compare(5, 1100, &guard, &control).unwrap()["matching_postgrip_ticks"],
        60
    );
    control.outcome.trace[0].bench_up_n = 1.;
    assert!(compare(5, 1100, &guard, &control).is_err());
    control.outcome.trace[0] = sample();
    control.outcome.anchor = Value::Null;
    assert!(compare(5, 1100, &guard, &control).is_err());
    control.outcome.baseline = 1;
    assert!(compare(5, 1100, &guard, &control).is_err());
}

#[test]
fn escape_requires_continuous_clearance_of_both_jaws_and_the_bench() {
    let mut r = Recovery::default();
    let mut sample = sample();
    sample.jaw_squeeze_n = [0.; 2];
    for k in 1..4 {
        r.confirm_escape(k, &sample, [0.01, 0.01]);
    }
    r.confirm_escape(4, &sample, [0.01, 0.]);
    assert_eq!(r.clear_ticks, 0);
    r.confirm_escape(5, &sample, [0.01, 0.01]);
    sample.min_bench_clearance_m = 0.;
    r.confirm_escape(6, &sample, [0.01, 0.01]);
    assert_eq!(r.clear_ticks, 0);
    sample.min_bench_clearance_m = 0.05;
    for k in 7..12 {
        r.confirm_escape(k, &sample, [0.01, 0.01]);
    }
    assert_eq!(r.escaped, Some(10));
    sample.jaw_squeeze_n[1] = 1.;
    r.confirm_escape(12, &sample, [0.01, 0.01]);
    assert_eq!(r.clear_ticks, 0);
    let mut b = held();
    b.outcome.trace.truncate(65);
    assert!(metrics(&b)["patch_motion_at_400ms_m"].is_null());
    assert_eq!(metrics(&b)["observed_postgrip_ticks"], 64);
}

#[test]
fn admission_and_publication_errors_leave_no_partial_recovery_study() {
    let parent = tempfile::tempdir().unwrap();
    let out = parent.path().join("study");
    assert!(write_new(&out, |p| run(p, Some((9, 1100)), End::Escape)).is_err());
    assert!(!out.exists());
}

#[test]
fn a_confirmed_supported_release_opens_the_real_actuator_over_300_ticks() {
    let scene = closing_sheet_scene::scene(5).unwrap();
    let mut grasp = Grasp::pickup(&scene, 5).unwrap();
    let mut actions = Recovery::default();
    let last = sample();
    let mut now = last.clone();
    now.jaw_squeeze_n = [0.; 2];
    now.bench_up_n = 0.08;
    for k in 1..=75 {
        actions.observe(k, &last, &now, 0.07998, 0.08, true);
    }
    let open = actions
        .open
        .expect("sustained receiver support authorizes release");
    assert_eq!(open, actions.lower.unwrap() + 20);
    actions
        .drive(open + 150, &mut grasp, "recovery", 0.)
        .unwrap();
    assert!((grasp.stroke - 0.04).abs() < 1e-12);
    actions
        .drive(open + 300, &mut grasp, "recovery", 0.)
        .unwrap();
    assert_eq!(grasp.stroke, 0.);
}

#[test]
fn invalid_mesh_and_failed_archive_storage_propagate_through_the_pair() {
    let dir = tempfile::tempdir().unwrap();
    assert!(pair(4, 1100, dir.path(), End::Escape).is_err());
    let dest = dir.path().join("file");
    fs::write(&dest, b"keep").unwrap();
    assert!(run(&dest, Some((5, 1250)), End::Escape).is_err());
    assert_eq!(fs::read(&dest).unwrap(), b"keep");
}

#[test]
fn landing_balance_accounts_for_both_jaws_gravity_and_bench_friction() {
    let mut prev = sample();
    prev.momentum_y_kg_m_s = 0.01;
    let mut now = prev.clone();
    now.bench_up_n = 4.;
    now.jaw_up_n = [0.1, -0.2];
    now.force_y_n = 0.9;
    now.bench_side_reaction_n = 0.6;
    now.jaw_side_reaction_n = [0.1, 0.3];
    now.momentum_y_kg_m_s = 0.0099;
    let masses_before = [[0.01, -0.2], [0.02, -0.1]];
    let masses_after = [[0.01, -0.03], [0.02, -0.005]];
    let impact = landing::impact(27, &masses_before, &masses_after, &prev, &now, 0.3);
    assert!(impact["vertical_balance_error_ns"].as_f64().unwrap().abs() < 1e-14);
    assert!(impact["lateral_balance_error_ns"].as_f64().unwrap().abs() < 1e-14);
    assert_eq!(impact["incoming_mass_kg_vz_m_s"], json!(masses_before));
    assert_eq!(impact["outgoing_mass_kg_vz_m_s"], json!(masses_after));
}
