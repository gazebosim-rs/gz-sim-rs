// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact, reset_loaded_world};
use std::time::Duration;
const SDF: &str = include_str!("../../../examples/multifidelity_finite_transfer_endpoint.sdf");
fn load(xml: &str) -> gz_sim_core::Simulation {
    load_world_with_sdf_step(
        xml,
        "secondary.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
#[test]
fn a_registered_secondary_loads_only_the_closing_endpoint_and_resets() {
    for n in [2, 8, 64] {
        let mut s = load(&SDF.replace("<native_steps>8", &format!("<native_steps>{n}")));
        s.step_exact(1).unwrap();
        let record = primary_contact::last_refinement(&s).unwrap().clone();
        assert_eq!(record.interval.native_steps, n);
        let [h] = record.interval.finite_point_impacts.as_slice() else {
            panic!("missing endpoint")
        };
        let b = h.segment.unwrap();
        let a = h.resultant;
        assert_eq!(a.elapsed, Duration::ZERO);
        assert!(a.sticking);
        let impulse = DVec3::from(a.impulse_on_foot);
        assert!(
            impulse.distance(DVec3::new(
                0.12106378145052,
                0.11519480432165,
                0.37127796362431
            )) < 1e-12
        );
        assert!(DVec3::from(a.after_relative_velocity).length() < 1e-12);
        assert!(b.opening_lower_bound > 0.08);
        assert!(DVec3::from(a.normal).dot(b.opposite_after.into()) > b.opening_lower_bound);
        let [p] = record.requested_pairs.as_slice() else {
            panic!("missing public load")
        };
        assert_eq!(p.first_positive_end, Some(Duration::ZERO));
        assert!(p.normal_impulse >= impulse.dot(a.normal.into()));
        s.step_exact(0).unwrap();
        assert_eq!(primary_contact::last_refinement(&s), Some(&record));
        reset_loaded_world(&mut s, |_| ()).unwrap();
        s.step_exact(1).unwrap();
        assert_eq!(
            primary_contact::last_refinement(&s).unwrap().interval,
            record.interval
        );
    }
}
#[test]
fn a_disjoint_deck_or_second_coincident_support_keeps_the_ordinary_solver() {
    let (a, b) = SDF.split_once("<model name=\"lift\">").unwrap();
    let model = b.split_once("</model>").unwrap().0;
    let duplicate =
        format!("{a}<model name=\"other_deck\">{model}</model><model name=\"lift\">{b}");
    let (pre, pose) = b.split_once("<pose>").unwrap();
    let (_, rest) = pose.split_once("</pose>").unwrap();
    let separated = format!("{a}<model name=\"lift\">{pre}<pose>0 0 -1 0 0 0</pose>{rest}");
    for xml in [duplicate, separated] {
        let mut sim = load(&xml);
        sim.step_exact(1).unwrap();
        assert!(
            primary_contact::last_refinement(&sim)
                .is_none_or(|r| r.interval.finite_point_impacts.is_empty())
        );
    }
}

const WORK: &str = include_str!("../../../examples/multifidelity_finite_sliding_transfer.sdf");
#[test]
fn a_sliding_endpoint_requires_friction_work_to_exclude_an_entire_loaded_line() {
    let mut standard = load(&WORK.replace(
        "<certify_segment_friction_work>true",
        "<certify_segment_friction_work>false",
    ));
    standard.step_exact(1).unwrap();
    assert!(
        primary_contact::last_refinement(&standard)
            .unwrap()
            .interval
            .finite_point_impacts
            .is_empty()
    );
    for n in [2, 8, 64] {
        let mut sim = load(&WORK.replace("<native_steps>8", &format!("<native_steps>{n}")));
        sim.step_exact(1).unwrap();
        let r = primary_contact::last_refinement(&sim).unwrap().clone();
        let [h] = r.interval.finite_point_impacts.as_slice() else {
            panic!("missing friction-work proof")
        };
        let certificate = h.segment.unwrap();
        assert!((2..=128).contains(&certificate.work_boxes) && !h.resultant.sticking);
        assert_eq!(h.resultant.elapsed, Duration::ZERO);
        assert!(
            (DVec3::from(h.resultant.impulse_on_foot)
                - DVec3::new(-0.231260754369, 0.165535141807, 0.568800209861))
            .length()
                < 1e-11
        );
        assert!(certificate.opening_lower_bound > 0.116);
        assert_eq!(r.interval.native_steps, n);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(
            primary_contact::last_refinement(&sim).unwrap().interval,
            r.interval
        );
    }
}
#[test]
fn a_falling_line_with_two_required_endpoints_declines_the_one_point_law() {
    fn first_tag(xml: &str, tag: &str, new: &str) -> String {
        let left = format!("<{tag}>");
        let right = format!("</{tag}>");
        let (prefix, tail) = xml.split_once(&left).unwrap();
        let (_, suffix) = tail.split_once(&right).unwrap();
        format!("{prefix}{left}{new}{right}{suffix}")
    }
    let xml = first_tag(
        &first_tag(WORK, "linear_velocity", "0 0 -1"),
        "angular_velocity",
        "0 0 0",
    );
    let mut s = load(&xml);
    s.step_exact(1).unwrap();
    let rec = primary_contact::last_refinement(&s).unwrap();
    assert_eq!(rec.interval.native_steps, 8);
    assert!(rec.interval.finite_point_impacts.is_empty());
    assert!(rec.requested_pairs[0].normal_impulse > 0.);
}

#[test]
fn the_outgoing_vertex_moves_the_next_native_call_within_the_same_public_tick() {
    let src = include_str!("../../../examples/multifidelity_finite_sliding_transfer.sdf");
    let timed = src.replace(
        "</pair>",
        "<retime_finite_transfer>true</retime_finite_transfer></pair>",
    );
    for calls in [2, 8, 64] {
        let run = |source: &str| {
            let mut sim =
                load(&source.replace("<native_steps>8", &format!("<native_steps>{calls}")));
            sim.step_exact(1).unwrap();
            primary_contact::last_refinement(&sim)
                .unwrap()
                .interval
                .clone()
        };
        let regular = run(src);
        let changed = run(&timed);
        assert_eq!(changed.native_steps, calls);
        let ordinary = regular.finite_point_impacts[0];
        let mut h = changed.finite_point_impacts[0];
        let bound = h.next_feature_boundary.take();
        assert_eq!(h, ordinary);
        if calls == 64 {
            assert!(bound.is_none());
        } else {
            let (limit, dt) = bound.expect("earlier outgoing feature");
            assert_eq!(limit, gz_physics::FeatureLimit::MovingNeighbor(1));
            assert!(dt.abs_diff(Duration::from_nanos(17_383)) < Duration::from_nanos(16));
        }
    }
}
