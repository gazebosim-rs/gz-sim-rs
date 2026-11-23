// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::SeparatingFeature;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact, reset_loaded_world};
use std::time::Duration;
const SDF: &str = include_str!("../../../examples/multifidelity_finite_primary.sdf");
fn load(s: &str) -> gz_sim_core::Simulation {
    load_world_with_sdf_step(
        s,
        "outer.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
#[test]
fn first_skew_edge_contact_uses_the_measured_root_twist_and_resets_identically() {
    for n in [2, 8, 64] {
        let mut s = load(&SDF.replace("<native_steps>8", &format!("<native_steps>{n}")));
        s.step_exact(1).unwrap();
        let rec = primary_contact::last_refinement(&s).unwrap().clone();
        let [impact] = rec.interval.finite_point_impacts.as_slice() else {
            panic!("lost finite projection {:?}", rec.interval);
        };
        assert_eq!(rec.interval.native_steps, n);
        assert_eq!(impact.resultant.elapsed, Duration::from_nanos(328543));
        assert_eq!(impact.feature, SeparatingFeature::EdgePair(0, 1));
        let target = [0.3485768399227482, 0.2111929825539457, 0.815135677906691];
        for (got, want) in impact.resultant.impulse_on_foot.into_iter().zip(target) {
            assert!((got - want).abs() < 1e-10);
        }
        assert_eq!(
            rec.requested_pairs[0].first_positive_end,
            Some(impact.resultant.elapsed)
        );
        s.step_exact(0).unwrap();
        assert_eq!(primary_contact::last_refinement(&s), Some(&rec));
        reset_loaded_world(&mut s, |_| ()).unwrap();
        s.step_exact(1).unwrap();
        assert_eq!(
            primary_contact::last_refinement(&s).unwrap().interval,
            rec.interval
        );
    }
}
#[test]
fn duplicate_support_or_anisotropic_locked_tree_declines_fractional_projection() {
    let other = r#"<model name="neighbor"><static>true</static><pose>0.165 0 -0.077556 0 0 0</pose><link name="top"><collision name="c"><geometry><box><size>0.66 0.36 0.12</size></box></geometry></collision></link></model>"#;
    for xml in [
        SDF.replace("</world>", &format!("{other}</world>")),
        SDF.replace("<iyy>0.0201", "<iyy>0.0211"),
    ] {
        let mut sim = load(&xml);
        sim.step_exact(1).unwrap();
        assert!(
            primary_contact::last_refinement(&sim)
                .is_none_or(|r| r.interval.finite_point_impacts.is_empty())
        );
    }
}

#[test]
fn measured_outgoing_features_retime_an_ordinary_step_and_keep_the_public_clock() {
    for calls in [2, 8, 64] {
        let xml = SDF
            .replace("<native_steps>8", &format!("<native_steps>{calls}"))
            .replace(
                "</project_finite_point>",
                "</project_finite_point><retime_finite_transfer>true</retime_finite_transfer>",
            );
        let mut sim = load(&xml);
        sim.step_exact(1).unwrap();
        let rec = primary_contact::last_refinement(&sim).unwrap().clone();
        assert_eq!(rec.interval.native_steps, calls);
        let hit = rec.interval.finite_point_impacts[0];
        assert_eq!(hit.resultant.elapsed, Duration::from_nanos(328543));
        if calls == 8 || calls == 64 {
            let (edge, t) = hit.next_feature_boundary.expect("early second contact");
            assert_eq!(edge, gz_physics::FeatureLimit::SupportNeighbor(0));
            assert!((2550..=2580).contains(&t.as_nanos()));
        } else {
            assert!(hit.next_feature_boundary.is_none());
        }
        sim.step_exact(0).unwrap();
        assert_eq!(primary_contact::last_refinement(&sim), Some(&rec));
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(
            primary_contact::last_refinement(&sim).unwrap().interval,
            rec.interval
        );
    }
}
