// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics::{BodyId, BodyState, DynamicBackend, PhysicsBackend};
use gz_physics_multifidelity::{BoxContactActions, MultiFidelityBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions,
    load_world_with_runtime, primary_contact::last_refinement, queue_link_world_wrench,
    remove_model, reset_loaded_world,
};
use std::time::Duration;
const SDF: &str = include_str!("../../../examples/multifidelity_sliding_edge.sdf");
fn try_load(source: &str) -> Result<Simulation, Box<dyn std::error::Error>> {
    Ok(load_world_with_runtime(
        source,
        "edge.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            rapier_per_point_friction: true,
            ..Default::default()
        },
    )?)
}
fn handles(s: &Simulation, name: &str) -> (u64, Option<BodyId>) {
    #[allow(deprecated)]
    let e = s
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (
        e.get::<EntityId>().unwrap().get(),
        e.get::<BodyHandle>().map(|b| b.id()),
    )
}
fn state(s: &Simulation, id: BodyId) -> BodyState {
    s.world().resource::<PhysicsBackend>().state(id).unwrap()
}
fn source_at(seconds: f64, gravity: f64, mu: f64) -> String {
    // Uniform incoming free translation gives a rotationally fixed edge;
    // both pre-impact normal speeds equal 1.27 m/s plus gravity × time.
    let mut s = SDF.to_owned();
    let tag = "<linear_velocity>";
    let a = s.find(tag).unwrap();
    let b = a + s[a..].find("</angular_velocity>").unwrap() + "</angular_velocity>".len();
    s.replace_range(a..b,"<linear_velocity>3.6 -1.9 -1.05</linear_velocity><angular_velocity>0 0 0</angular_velocity>");
    s.replace(
        "0.07693357890986972",
        &format!(
            "{:.17}",
            0.07693357890986972 + 1.27 * seconds - 0.5 * gravity * seconds * seconds
        ),
    )
    .replace("0 0 -9.81", &format!("0 0 {gravity}"))
    .replace(
        "<mu>0.35</mu><mu2>0.35</mu2>",
        &format!("<mu>{mu}</mu><mu2>{mu}</mu2>"),
    )
}
#[test]
fn authored_locked_edge_delivers_both_signed_reactions_once_with_one_clock_and_replays() {
    let mut s = try_load(SDF).unwrap();
    let foot = handles(&s, "default::rocker::foot").1.unwrap();
    let upper = handles(&s, "default::rocker::upper").1.unwrap();
    let p0 = DVec3::from(state(&s, foot).velocity) + DVec3::from(state(&s, upper).velocity);
    s.step_exact(1).unwrap();
    let r = last_refinement(&s).unwrap().clone();
    assert_eq!(s.elapsed(), Duration::from_micros(500));
    assert_eq!(r.interval.native_steps, 8);
    assert_eq!(r.interval.sliding_edge_impacts.len(), 1);
    let e = r.interval.sliding_edge_impacts[0];
    let mut loads = e.sites.map(|p| p.impulse_on_foot[2]);
    loads.sort_by(f64::total_cmp);
    assert!((loads[0] - 0.4).abs() < 1e-12 && (loads[1] - 0.8).abs() < 1e-12);
    let a = DVec3::from(e.sites[0].after_relative_velocity);
    let b = DVec3::from(e.sites[1].after_relative_velocity);
    assert!(a.angle_between(b) > 0.2 && a.z.abs() < 1e-12 && b.z.abs() < 1e-12);
    let pair = &r.requested_pairs[0];
    assert_eq!(pair.first_positive_end, Some(Duration::ZERO));
    let total = DVec3::from(pair.normal_impulse_on_foot)
        + DVec3::from(pair.tangent_impulse_on_foot.unwrap());
    let p1 = DVec3::from(state(&s, foot).velocity) + DVec3::from(state(&s, upper).velocity);
    assert!(total.distance(p1 - p0 - DVec3::new(0., 0., -9.81 * 2. * 0.0005)) < 1e-4);
    assert!(r.interval.planar_impacts.is_empty() && r.interval.corner_impacts.is_empty());
    assert!(pair.last_native_normal_impulse < pair.normal_impulse);
    for t in 2..=10 {
        s.step_exact(1).unwrap();
        assert!(
            last_refinement(&s)
                .is_none_or(|e| e.iteration != t || e.interval.sliding_edge_impacts.is_empty())
        );
    }
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_refinement(&s).is_none());
    s.step_exact(1).unwrap();
    assert_eq!(last_refinement(&s).unwrap().interval, r.interval);
}
#[test]
fn free_translation_and_gravity_preserve_fractional_edge_arrival_and_one_actuator_clock() {
    for (ns, g) in [
        (62500.37, 0.),
        (499999.34, -9.81),
        (638197.61, -9.81),
        (2138197.72, 0.),
    ] {
        let mut s = try_load(&source_at(ns * 1e-9, g, 0.08)).unwrap();
        let ticks = (ns / 500000.).ceil() as u64;
        s.step_exact(ticks - 1).unwrap();
        let foot = handles(&s, "default::rocker::foot").1.unwrap();
        let pad = handles(&s, "default::lift::pad").1.unwrap();
        let n = s
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let plan = n
            .sliding_edge_arrival(foot, pad, s.step_duration())
            .unwrap();
        assert!(
            plan.is_some(),
            "unqualified flight {ns} ns gravity {g}, foot {:?} pad {:?}",
            n.state(foot),
            n.state(pad)
        );
        s.step_exact(1).unwrap();
        let r = last_refinement(&s).unwrap();
        assert_eq!(r.iteration, ticks);
        assert_eq!(
            r.interval.sliding_edge_impacts.len(),
            1,
            "missed arrival {ns} {g} with plan {plan:?}"
        );
        let hit = r.interval.sliding_edge_impacts[0];
        let stamp = Duration::from_nanos(ns.floor() as u64 % 500000);
        assert_eq!(r.requested_pairs[0].first_positive_end, Some(stamp));
        assert_eq!(s.elapsed(), Duration::from_micros(ticks * 500));
        for p in hit.sites {
            assert_eq!(p.elapsed, stamp);
            assert!(p.gap.abs() < 1.4e-9);
            assert!(
                (p.before_relative_velocity[2]
                    - (-1.27 + g * (r.start_time + stamp).as_secs_f64()))
                .abs()
                    < 2e-11
            );
            assert!(p.after_relative_velocity[2].abs() < 1e-12);
        }
    }
}
#[test]
fn removed_support_and_unrelated_queued_load_do_not_cross_pair_ownership() {
    let witness = r#"<model name="witness"><pose>3 0 2 0 0 0</pose><link name="b"><inertial><mass>2</mass></inertial><collision name="ball"><geometry><sphere><radius>0.08</radius></sphere></geometry></collision></link></model>"#;
    let mut s = try_load(&SDF.replace("</world>", &format!("{witness}</world>"))).unwrap();
    let (uid, w) = handles(&s, "default::witness::b");
    queue_link_world_wrench(&mut s, uid, [12., -6., 0.], [0.; 3]).unwrap();
    s.step_exact(1).unwrap();
    assert_eq!(
        last_refinement(&s)
            .unwrap()
            .interval
            .sliding_edge_impacts
            .len(),
        1
    );
    let v = DVec3::from(state(&s, w.unwrap()).velocity);
    assert!(v.distance(DVec3::new(0.003, -0.0015, -0.004905)) < 1e-9);
    let mut airborne = try_load(&source_at(0.0021381976, -9.81, 0.08)).unwrap();
    airborne.step_exact(2).unwrap();
    let pad = handles(&airborne, "default::lift").0;
    airborne.set_paused(true);
    assert!(airborne.tick().unwrap().dt.is_zero());
    remove_model(&mut airborne, pad).unwrap();
    airborne.step_exact(10).unwrap();
    assert!(last_refinement(&airborne).is_none());
}
#[test]
fn overlapping_rows_uncertified_friction_and_geometric_scope_decline_atomically() {
    let obstacle = r#"<model name="bar"><static>true</static><pose>0 0 0.08 0 0 0</pose><link name="block"><collision name="c"><geometry><sphere><radius>0.035</radius></sphere></geometry></collision></link></model>"#;
    for xml in [
        SDF.replace("</world>", &format!("{obstacle}</world>")),
        SDF.replace("0.82 0.82 0.12", "0.12 0.12 0.12"),
        SDF.replace(">0.35<", ">3.0<"),
        SDF.replace("<exact_revolute_locks>true", "<exact_revolute_locks>false"),
    ] {
        let mut s = try_load(&xml).unwrap();
        let foot = handles(&s, "default::rocker::foot").1.unwrap();
        let pad = handles(&s, "default::lift::pad").1.unwrap();
        let mut owner = s.world_mut().resource_mut::<PhysicsBackend>();
        let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
        let before = n.state(foot);
        assert!(n.project_sliding_edge_impact(foot, pad).unwrap().is_none());
        assert_eq!(before, n.state(foot));
    }
    for other in ["project_planar", "project_corner", "project_sliding_edge"] {
        assert!(
            try_load(&SDF.replace("</pair>", &format!("<{other}>true</{other}></pair>"))).is_err()
        );
    }
    let mut s = try_load(SDF).unwrap();
    let foot = handles(&s, "default::rocker::foot").1.unwrap();
    let pad = handles(&s, "default::lift::pad").1.unwrap();
    let mut owner = s.world_mut().resource_mut::<PhysicsBackend>();
    let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
    let before = n.state(foot);
    let pair = [(foot, pad)];
    let a = BoxContactActions {
        sliding_edge_impacts: &pair,
        corner_impacts: &pair,
        ..Default::default()
    };
    assert!(
        n.advance_box_contacts(Duration::from_micros(500), 8, 4.into(), a, None)
            .is_err()
    );
    assert_eq!(n.state(foot), before);
}

#[path = "support/registered_mixed_edge.rs"]
mod mixed;
