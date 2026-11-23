// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics::{BodyId, DynamicBackend, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step,
    primary_contact::{last_refinement, set_enabled},
    reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;
const INITIAL_Z: f64 = 0.128_176_984_407_140_3;
const SDF: &str = include_str!("../../../examples/multifidelity_registered_corner.sdf");
fn load(source: &str) -> Simulation {
    load_world_with_sdf_step(
        source,
        "corner.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn id(s: &Simulation, name: &str) -> BodyId {
    #[allow(deprecated)]
    s.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<BodyHandle>()
        .unwrap()
        .id()
}
#[test]
fn authored_free_hinge_corner_applies_once_with_signed_reaction_on_the_fixed_actuator_clock() {
    let mut s = load(SDF);
    let foot = id(&s, "default::rocker::foot");
    let top = id(&s, "default::rocker::upper");
    let pad = id(&s, "default::lift::pad");
    let state = |s: &Simulation, id| s.world().resource::<PhysicsBackend>().state(id).unwrap();
    s.step_exact(1).unwrap();
    let first = last_refinement(&s)
        .expect("qualified exact contact at start")
        .clone();
    let impulse = first.interval.corner_impacts[0];
    assert_eq!((impulse.foot, impulse.support), (foot, pad));
    assert_eq!(
        first.requested_pairs[0].first_positive_end,
        Some(Duration::ZERO)
    );
    assert_eq!(first.interval.corner_impacts.len(), 1);
    assert_eq!(s.elapsed(), Duration::from_micros(500));
    let v = DVec3::from(impulse.after_relative_velocity);
    let n = DVec3::from(impulse.normal);
    assert!(v.dot(n).abs() < 5e-12);
    let j = DVec3::from(impulse.impulse_on_foot);
    let normal = j.dot(n);
    let t = j - normal * n;
    assert!(normal > 0.5 && t.length() <= impulse.friction * normal + 1e-11);
    assert!(t.dot(v) <= 1e-11);
    assert!(
        (impulse.kinetic_energy_change + impulse.dissipated_energy
            - j.dot(DVec3::new(1.1, -0.7, 1.4)))
        .abs()
            < 1e-11
    );
    let p = &first.requested_pairs[0];
    let total =
        DVec3::from(p.normal_impulse_on_foot) + DVec3::from(p.tangent_impulse_on_foot.unwrap());
    let momentum = DVec3::from(state(&s, foot).velocity) + DVec3::from(state(&s, top).velocity);
    assert!(total.distance(momentum) < 2e-4);
    let final_states = [state(&s, foot), state(&s, top), state(&s, pad)];
    s.step_exact(0).unwrap();
    assert_eq!(last_refinement(&s), Some(&first));
    s.step_exact(10).unwrap();
    assert!(
        last_refinement(&s)
            .is_none_or(|r| r.iteration == 1 || r.interval.corner_impacts.is_empty())
    );
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_refinement(&s).is_none());
    s.step_exact(1).unwrap();
    assert_eq!(last_refinement(&s).unwrap().interval, first.interval);
    assert_eq!(
        [state(&s, foot), state(&s, top), state(&s, pad)],
        final_states
    );
}
#[test]
fn disabled_or_multicontact_never_projects_a_private_impulse() {
    let mut off = load(SDF);
    set_enabled(&mut off, false).unwrap();
    off.step_exact(1).unwrap();
    assert!(last_refinement(&off).is_none());
    let mut level = SDF.to_owned();
    let from = SDF.lines().find(|l| l.contains("-0.18 0.31 0.15")).unwrap();
    level = level.replace(from, "      <pose>0 0 0.06 0 0 0</pose>");
    let mut flat = load(&level);
    flat.step_exact(1).unwrap();
    assert!(last_refinement(&flat).is_none_or(|r| r.interval.corner_impacts.is_empty()));
    let mut contact = load(SDF);
    let f = id(&contact, "default::rocker::foot");
    let p = id(&contact, "default::lift::pad");
    let mut owner = contact.world_mut().resource_mut::<PhysicsBackend>();
    let a = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
    a.step(Duration::from_micros(1)).unwrap();
    let before = a.state(f);
    assert!(a.project_corner_impact(f, p).unwrap().is_none());
    assert_eq!(a.state(f), before);
}

fn arriving(source: &str, ns: f64) -> String {
    source.replace(
        "0.12817698440714029",
        &format!("{:.17}", INITIAL_Z + 1.4 * ns * 1e-9),
    )
}
#[test]
fn native_free_flight_delivers_one_oblique_impulse_at_the_fractional_arrival() {
    for ns in [1.25, 62_500.35, 138_197.65, 249_999.9, 499_997.1] {
        let mut s = load(&arriving(SDF, ns));
        let foot = id(&s, "default::rocker::foot");
        let pad = id(&s, "default::lift::pad");
        let dt = s.step_duration();
        let expected = Duration::from_nanos(ns.floor() as u64);
        let native = s
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        assert_eq!(
            native.corner_arrival(foot, pad, dt).unwrap(),
            Some(expected)
        );
        s.step_exact(1).unwrap();
        let r = last_refinement(&s).unwrap();
        assert_eq!(r.interval.native_steps, 8);
        assert_eq!(r.interval.corner_impacts.len(), 1);
        let h = r.interval.corner_impacts[0];
        assert_eq!(h.elapsed, expected);
        assert!(h.gap >= -1e-13 && h.gap <= 1.4e-9 + 1e-13);
        assert_eq!(r.requested_pairs[0].first_positive_end, Some(expected));
        assert!((h.impulse_on_foot[2] - 0.9214035185183282).abs() < 1e-11);
        assert_eq!(s.elapsed(), dt);
    }
}

#[test]
fn flight_with_another_body_or_external_force_reverts_to_the_coupled_solver() {
    let xml = arriving(SDF, 138_197.6);
    let mut forced = load(&xml);
    let foot = id(&forced, "default::rocker::foot");
    let pad = id(&forced, "default::lift::pad");
    let dt = forced.step_duration();
    let mut owner = forced.world_mut().resource_mut::<PhysicsBackend>();
    let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
    n.add_world_wrench(foot, [4., 0., 0.], [0.; 3]).unwrap();
    assert_eq!(n.corner_arrival(foot, pad, dt).unwrap(), None);
    let obstacle = "<model name=\"extra\"><link name=\"ball\"><collision name=\"c\"><geometry><sphere><radius>0.15</radius></sphere></geometry></collision></link></model>";
    let mut s = load(&xml.replace("</world>", &format!("{obstacle}</world>")));
    s.step_exact(1).unwrap();
    assert!(last_refinement(&s).is_none_or(|r| r.interval.corner_impacts.is_empty()));
}

#[test]
fn coming_from_more_than_one_public_tick_preserves_the_first_arrival() {
    for ns in [638_197.65, 2_138_197.65, 6_638_197.65] {
        let mut s = load(&arriving(SDF, ns));
        let ticks = (ns / 500_000.).ceil() as u64;
        s.step_exact(ticks).unwrap();
        let r = last_refinement(&s).expect("qualified arrival");
        assert_eq!(r.iteration, ticks);
        let h = r
            .interval
            .corner_impacts
            .first()
            .expect("one projected impulse after preceding flight");
        assert_eq!(h.elapsed, Duration::from_nanos(138_197));
    }
}

#[test]
fn distant_static_floor_and_box_allow_the_registered_flight() {
    let clutter = r#"<model name="floor"><static>true</static><pose>0 0 -2 0 0 0</pose><link name="p"><collision name="c"><geometry><plane><normal>0 0 1</normal></plane></geometry></collision></link></model>
      <model name="far"><static>true</static><pose>-3 2 0 0 0 0</pose><link name="b"><collision name="c"><geometry><box><size>0.4 0.6 0.8</size></box></geometry></collision></link></model>"#;
    let mut s = load(&arriving(
        &SDF.replace("</world>", &format!("{clutter}</world>")),
        2_138_197.65,
    ));
    s.step_exact(5).unwrap();
    assert_eq!(
        last_refinement(&s).unwrap().interval.corner_impacts[0].elapsed,
        Duration::from_nanos(138_197)
    );
}

#[test]
fn ballistic_impact_uses_the_matched_boundary_velocity_and_full_vector_of_gravity() {
    for calls in [2u32, 8, 32] {
        for (g, v, t_ns) in [
            ([2.1, -3.4, -9.81], [1.1, -0.7, 1.4], 138_197.65),
            ([0., 0., -9.81], [1.1, -0.7, -0.001], 490_000.6),
            ([1.2, 0.3, 9.81], [1.1, -0.7, 1.4], 499_997.85),
            ([-3., 1.2, 0.], [1.1, -0.7, 1.4], 1.25),
        ] {
            let g = DVec3::from(g);
            let v = DVec3::from(v);
            let t = t_ns * 1e-9;
            let z = INITIAL_Z + v.z * t - 0.5 * g.z * t * t;
            let src = SDF
                .replace(
                    "<gravity>0 0 0</gravity>",
                    &format!("<gravity>{} {} {}</gravity>", g.x, g.y, g.z),
                )
                .replace("0.12817698440714029", &format!("{z:.17}"))
                .replace("1.1 -0.7 1.4", &format!("{} {} {}", v.x, v.y, v.z))
                .replace("<native_steps>8", &format!("<native_steps>{calls}"));
            let mut s = load(&src);
            let foot = id(&s, "default::rocker::foot");
            let pad = id(&s, "default::lift::pad");
            let dt = s.step_duration();
            let native = s
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            let at = native
                .corner_arrival(foot, pad, dt)
                .unwrap()
                .expect("certified ballistic approach");
            assert!(at.as_nanos().abs_diff(t_ns.floor() as u128) <= 1);
            s.step_exact(1).unwrap();
            let r = last_refinement(&s).expect("one accrued ballistic reaction");
            assert_eq!(r.interval.native_steps, calls);
            assert_eq!(
                r.interval.corner_impacts.len(),
                1,
                "g={g}, pad={v}, time={at:?}"
            );
            let hit = r.interval.corner_impacts[0];
            assert_eq!(at, hit.elapsed);
            let t = at.as_secs_f64();
            assert!(DVec3::from(hit.before_relative_velocity).distance(g * t - v) < 1e-11);
            assert!((hit.gap - (z - INITIAL_Z + 0.5 * g.z * t * t - v.z * t)).abs() < 2e-13);
            let j = DVec3::from(hit.impulse_on_foot);
            assert!((j.dot(v) - hit.kinetic_energy_change - hit.dissipated_energy).abs() < 3e-12);
        }
    }
}
