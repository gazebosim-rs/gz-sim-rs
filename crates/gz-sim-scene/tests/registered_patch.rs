// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{Bearing, PlanarPatchImpact};
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact, reset_loaded_world};
use sdformat::NoResources;
use std::time::Duration;
const SDF: &str = include_str!("../../../examples/multifidelity_planar_edge.sdf");
fn load(s: &str) -> gz_sim_core::Simulation {
    load_world_with_sdf_step(
        s,
        "patch.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
#[test]
fn finite_offset_patch_delivers_the_pressure_resultant_at_the_last_nanosecond_and_resets() {
    let mut s = load(SDF);
    let target = PlanarPatchImpact {
        mass: 2.,
        pitch_inertia: 0.05,
        com_height: 0.13,
        supported_x: [-0.18, -0.09],
        tangent_speed: 1.3,
        closing_speed: 1.4,
        friction: 0.6,
    }
    .solve()
    .unwrap();
    assert_eq!(target.bearing, Bearing::UpperXEdge);
    s.step_exact(1).unwrap();
    let r = primary_contact::last_refinement(&s)
        .expect("loaded footprint")
        .clone();
    assert!(r.interval.corner_impacts.is_empty());
    assert_eq!(r.interval.planar_impacts.len(), 1);
    assert_eq!(r.interval.native_steps, 2);
    let p = r.interval.planar_impacts[0];
    assert_eq!(p.bearing, target.bearing);
    assert!((p.pressure_x - target.center_of_pressure).abs() < 2e-9);
    assert!(
        p.resultant
            .elapsed
            .abs_diff(Duration::from_nanos(499_999))
            .as_nanos()
            <= 1
    );
    assert_eq!(
        r.requested_pairs[0].first_positive_end,
        Some(p.resultant.elapsed)
    );
    let j = p.resultant.impulse_on_foot;
    assert!(
        (j[0] - target.impulse[0]).abs() < 1e-7
            && (j[2] - target.impulse[1]).abs() < 1e-7
            && j[1].abs() < 1e-12
    );
    s.step_exact(0).unwrap();
    assert_eq!(primary_contact::last_refinement(&s), Some(&r));
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(primary_contact::last_refinement(&s).is_none());
    s.step_exact(1).unwrap();
    assert_eq!(
        primary_contact::last_refinement(&s).unwrap().interval,
        r.interval
    );
}
#[test]
fn spatial_slip_free_hinge_or_a_second_support_require_the_coupled_solver() {
    let other = r#"<model name="extra"><static>true</static><pose>0 0 -0.06 0 0 0</pose><link name="deck"><collision name="c"><geometry><box><size>0.66 0.36 0.12</size></box></geometry></collision></link></model>"#;
    for xml in [
        SDF.replace("1.3 0 -1.1", "1.3 0.2 -1.1"),
        SDF.replace("<exact_revolute_locks>true", "<exact_revolute_locks>false")
            .replace("<limit><lower>0</lower><upper>0</upper></limit>", ""),
        SDF.replace("</world>", &format!("{other}</world>")),
    ] {
        let mut s = load(&xml);
        s.step_exact(2).unwrap();
        assert!(
            primary_contact::last_refinement(&s)
                .is_none_or(|r| r.interval.planar_impacts.is_empty())
        );
    }
}
const EDGE: &str = include_str!("../../../examples/multifidelity_loaded_deck_edge.sdf");
#[test]
fn continuous_deck_load_and_public_boundary_impulses_survive_pause_and_reset() {
    let mut s = load(EDGE);
    s.step_exact(1).unwrap();
    let first = primary_contact::last_refinement(&s).unwrap().clone();
    assert_eq!(first.interval.planar_impacts.len(), 1);
    assert_eq!(
        first.interval.sustained_edges.len(),
        2,
        "initial sustained reactions {:?}",
        first.interval
    );
    let normal: f64 = first
        .interval
        .sustained_edges
        .iter()
        .map(|r| r.normal_impulse_on_foot[2])
        .sum();
    assert!(
        (normal - 0.015768457212322888).abs() < 5e-11,
        "measured {normal}"
    );
    assert!(first.requested_pairs[0].active_at_end);
    s.step_exact(39).unwrap();
    let later = primary_contact::last_refinement(&s).unwrap().clone();
    assert_eq!(later.iteration, 40);
    assert_eq!(later.interval.native_steps, 1);
    assert!(later.interval.planar_impacts.is_empty());
    assert_eq!(later.interval.sustained_edges.len(), 1);
    assert_eq!(
        later.requested_pairs[0].first_positive_end,
        Some(Duration::ZERO)
    );
    s.step_exact(0).unwrap();
    assert_eq!(primary_contact::last_refinement(&s), Some(&later));
    reset_loaded_world(&mut s, |_| ()).unwrap();
    s.step_exact(1).unwrap();
    assert_eq!(
        primary_contact::last_refinement(&s).unwrap().interval,
        first.interval
    );
}
#[test]
fn new_drive_and_external_force_revalidate_the_edge_before_spending_any_impulse() {
    use gz_physics::{DynamicBackend, PhysicsBackend};
    use gz_physics_multifidelity::MultiFidelityBackend;
    use gz_sim_scene::{BodyHandle, Name};
    for drive in [true, false] {
        let mutate = |s: &mut gz_sim_core::Simulation| {
            let name = if drive {
                "default::lift::pad"
            } else {
                "default::rocker::foot"
            };
            #[allow(deprecated)]
            let id = s
                .world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap()
                .get::<BodyHandle>()
                .unwrap()
                .id();
            let mut res = s.world_mut().resource_mut::<PhysicsBackend>();
            let n = res.downcast_mut::<MultiFidelityBackend>().unwrap();
            if drive {
                n.set_world_velocity(id, [0., 0., -0.8], [0.; 3]).unwrap();
            } else {
                n.add_world_wrench(id, [1.4, -0.2, 0.7], [0.1, 0.3, 0.])
                    .unwrap();
            }
        };
        let step = |enabled: bool| {
            let mut s = load(EDGE);
            s.step_exact(6).unwrap();
            assert_eq!(primary_contact::last_refinement(&s).unwrap().iteration, 6);
            if !enabled {
                primary_contact::set_enabled(&mut s, false).unwrap();
            }
            mutate(&mut s);
            s.step_exact(0).unwrap();
            s.step_exact(1).unwrap();
            assert!(
                primary_contact::last_refinement(&s)
                    .is_none_or(|r| r.iteration != 7 || r.interval.sustained_edges.is_empty())
            );
            #[allow(deprecated)]
            s.world()
                .iter_entities()
                .filter_map(|e| e.get::<BodyHandle>())
                .map(|h| {
                    s.world()
                        .resource::<PhysicsBackend>()
                        .state(h.id())
                        .unwrap()
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(step(true), step(false));
    }
}

#[test]
fn gravity_loaded_edge_accounts_for_gravity_separately_from_the_contact_reaction() {
    for (gravity, expected) in [
        ("0 0 -9.81", [-0.00001583002006296, 0.0247439150897124]),
        ("4 0 -8.94", [-0.00001532521387219, 0.0239549627212523]),
    ] {
        let source = EDGE.replace("<gravity>0 0 0", &format!("<gravity>{gravity}"));
        let mut s = load(&source);
        s.step_exact(1).unwrap();
        let native = &primary_contact::last_refinement(&s).unwrap().interval;
        assert_eq!(native.planar_impacts.len(), 1);
        assert_eq!(native.sustained_edges.len(), 2);
        for (axis, value) in [(0, expected[0]), (2, expected[1])] {
            let measured: f64 = native
                .sustained_edges
                .iter()
                .map(|e| e.normal_impulse_on_foot[axis] + e.tangent_impulse_on_foot[axis])
                .sum();
            assert!((measured - value).abs() < 2e-12);
        }
        s.step_exact(39).unwrap();
        assert_eq!(primary_contact::last_refinement(&s).unwrap().iteration, 40);
    }
    let mut s = load(&EDGE.replace("<gravity>0 0 0", "<gravity>4 0.02 -8.94"));
    s.step_exact(1).unwrap();
    let r = primary_contact::last_refinement(&s).unwrap();
    assert!(r.interval.sustained_edges.is_empty());
}

#[test]
fn a_prescribed_neighbor_heading_into_the_swept_assembly_preempts_the_edge() {
    use gz_physics::{DynamicBackend, PhysicsBackend};
    use gz_physics_multifidelity::MultiFidelityBackend;
    use gz_sim_scene::{BodyHandle, Name};
    let obstacle = r#"<model name="slider"><pose>0.02 0.21 0.09 0 0 0</pose>
      <plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>0 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>
      <link name="side"><collision name="c"><geometry><box><size>0.10 0.08 0.10</size></box></geometry></collision></link></model>"#;
    let xml = EDGE.replace("</world>", &format!("{obstacle}</world>"));
    let step = |enabled: bool| {
        let mut s = load(&xml);
        s.step_exact(6).unwrap();
        assert_eq!(primary_contact::last_refinement(&s).unwrap().iteration, 6);
        if !enabled {
            primary_contact::set_enabled(&mut s, false).unwrap();
        }
        let mut res = s.world_mut().query::<(&Name, &BodyHandle)>();
        let id = res
            .iter(s.world())
            .find(|(n, _)| n.0 == "default::slider::side")
            .unwrap()
            .1
            .id();
        s.world_mut()
            .resource_mut::<PhysicsBackend>()
            .downcast_mut::<MultiFidelityBackend>()
            .unwrap()
            .set_world_velocity(id, [0., -240., 0.], [0.; 3])
            .unwrap();
        s.step_exact(1).unwrap();
        assert!(
            primary_contact::last_refinement(&s)
                .is_none_or(|r| r.iteration != 7 || r.interval.sustained_edges.is_empty())
        );
        let mut bodies = s.world_mut().query::<&BodyHandle>();
        bodies
            .iter(s.world())
            .map(|h| {
                s.world()
                    .resource::<PhysicsBackend>()
                    .state(h.id())
                    .unwrap()
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(step(true), step(false));
}

#[test]
fn gravity_holds_a_material_pivot_with_no_slip_or_frictional_work() {
    let mut s = load(include_str!(
        "../../../examples/multifidelity_sticking_deck_pivot.sdf"
    ));
    let mut total = [0.; 3];
    for tick in 1..=40 {
        s.step_exact(1).unwrap();
        let r = primary_contact::last_refinement(&s).unwrap();
        assert_eq!(r.iteration, tick);
        assert_eq!(r.interval.planar_impacts.len(), usize::from(tick == 1));
        assert_eq!(
            r.interval.sustained_edges.len(),
            if tick == 1 { 2 } else { 1 }
        );
        for e in &r.interval.sustained_edges {
            assert_eq!(e.motion, gz_physics::EdgeMotion::Sticking);
            assert_eq!(e.dissipated_energy, 0.);
            assert_eq!(e.start_x, e.end_x);
            for (i, x) in total.iter_mut().enumerate() {
                *x += e.normal_impulse_on_foot[i] + e.tangent_impulse_on_foot[i];
            }
        }
    }
    for (a, b) in total
        .into_iter()
        .zip([0.0728285193267406, 0., 0.3002977681310395])
    {
        assert!((a - b).abs() < 2e-13);
    }
}
