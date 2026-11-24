// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DMat3, DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine, load_world_with_sdf_step,
    primary_contact::{last_opposing_corner, set_enabled, set_refined_observer},
    remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

const WORLD: &str = include_str!("../../../examples/multifidelity_local_projected_ceiling.sdf");
const OPT: &str = "<project_opponent_corner>true</project_opponent_corner>";
fn load(s: &str) -> Simulation {
    load_world_with_sdf_step(
        s,
        "ceiling.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

#[test]
fn measured_upper_corner_closes_both_momenta_and_has_its_own_public_pair_receipt() {
    let mut sim = load(WORLD);
    let samples = Arc::new(Mutex::new(Vec::new()));
    let saved = samples.clone();
    set_refined_observer(&mut sim, move |t, s| {
        saved
            .lock()
            .unwrap()
            .push((t.iteration, s.elapsed, s.opposing_corner_impact));
    })
    .unwrap();
    sim.step_exact(2).unwrap();
    let r = last_opposing_corner(&sim).unwrap().clone();
    let [resolved] = r.interval.opposing_corner_impacts.as_slice() else {
        panic!("expected one isolated strike")
    };
    let h = resolved.point;
    let j = DVec3::from(h.impact.impulse_on_foot);
    let expected = DVec3::new(-3.448821633445934, -0.19038457673023723, -10.51765647424304);
    let n = DVec3::from(h.impact.normal);
    assert_eq!(r.iteration, 2);
    assert_eq!(r.sim_time, Duration::from_nanos(3960));
    assert_eq!(r.interval.native_steps, 8);
    assert_eq!(h.corner, 3);
    assert!(!h.impact.sticking);
    assert!(
        resolved.timing_error_s <= 1e-10
            && resolved.minimum_other_gap_m > 0.
            && resolved.minimum_footprint_margin_m > 0.
    );
    assert_eq!(h.impact.elapsed, Duration::from_nanos(1980));
    assert!(
        j.distance(expected) < 5e-8
            && (h.impact.dissipated_energy - 67.39774542199578).abs() < 1e-6
    );
    let edge = &r
        .interval
        .pairs
        .iter()
        .find(|p| {
            (p.a, p.b)
                == (
                    h.impact.foot.min(h.impact.support),
                    h.impact.foot.max(h.impact.support),
                )
        })
        .unwrap();
    let sign = if edge.a == h.impact.foot { 1. } else { -1. };
    assert!((edge.normal_impulse - j.dot(n)).abs() < 1e-11);
    assert_eq!(edge.first_positive_end, Some(h.impact.elapsed));
    let jpair = sign
        * (DVec3::from(edge.normal_impulse_on_a) + DVec3::from(edge.tangent_impulse_on_a.unwrap()));
    assert!(jpair.distance(j) < 1e-11);
    let (a, b) = (resolved.incoming, resolved.outgoing);
    assert_eq!(a.position, b.position);
    assert_eq!(a.orientation, b.orientation);
    assert!((2. * (DVec3::from(b.velocity) - DVec3::from(a.velocity))).distance(j) < 1e-12);
    let q = DQuat::from_array(a.orientation);
    let com = DVec3::from(a.position) + q * DVec3::new(0., 0., 0.07);
    let tensor = DMat3::from_cols_array(&[
        0.05183937212966956,
        -0.002034155695998175,
        0.0023284745769741964,
        -0.002034155695998175,
        0.05521533167239691,
        0.0028454090909881945,
        0.0023284745769741964,
        0.0028454090909881945,
        0.05054529619793354,
    ]);
    let dw = DVec3::from(b.angular_velocity) - DVec3::from(a.angular_velocity);
    assert!(
        (q * (tensor * (q.conjugate() * dw)))
            .distance((DVec3::from(h.impact.point) - com).cross(j))
            < 1e-12
    );
    let snapshots = samples.lock().unwrap().clone();
    assert_eq!(snapshots.len(), 8);
    assert_eq!(snapshots.iter().filter(|s| s.2.is_some()).count(), 1);
    assert_eq!(snapshots.last().unwrap().2, Some(*resolved));
    sim.set_paused(true);
    assert!(sim.tick().unwrap().dt.is_zero());
    #[allow(deprecated)]
    let gate = sim
        .world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<Name>()?.0 == "default::opponent").then(|| e.get::<EntityId>().unwrap().get())
        })
        .unwrap();
    remove_model(&mut sim, gate).unwrap();
    sim.step_exact(2).unwrap();
    assert_eq!(last_opposing_corner(&sim), Some(&r));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(last_opposing_corner(&sim).is_none());
    sim.step_exact(2).unwrap();
    let replay = last_opposing_corner(&sim).unwrap();
    assert_eq!(r.interval, replay.interval);
    assert_eq!(replay.epoch, r.epoch + 1);
}
#[test]
fn upper_projection_is_explicit_and_declines_an_additional_actor() {
    for (xml,disable) in [(WORLD.replace(OPT,""),false),(WORLD.to_string(),true),(WORLD.replace("</world>",r#"<model name="other"><static>true</static><pose>5 0 0 0 0 0</pose><link name="far"><collision name="box"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></world>"#),false)]{
        let mut s=load(&xml);if disable{set_enabled(&mut s,false).unwrap();} s.step_exact(8).unwrap();
        assert!(last_opposing_corner(&s).is_none_or(|r|r.interval.opposing_corner_impacts.is_empty()));
    }
    for bad in [
        WORLD.replace(OPT, &format!("{OPT}{OPT}")),
        WORLD.replace(
            OPT,
            "<project_opponent_corner>yes</project_opponent_corner>",
        ),
        WORLD.replace(
            "<guard_opponent>default::opponent::gate</guard_opponent>",
            "",
        ),
    ] {
        assert!(
            load_world_with_sdf_step(
                &bad,
                "bad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

const ORDERED: &str = include_str!("../../../examples/multifidelity_local_opposing_return.sdf");
const RETURN_OPT: &str = "<project_opponent_return>true</project_opponent_return>";
#[test]
fn spatial_return_uses_both_authored_pairs_at_their_separate_fractional_times() {
    for count in [1, 8] {
        let mut sim = load(&ORDERED.replace(
            "<native_steps>8</native_steps>",
            &format!("<native_steps>{count}</native_steps>"),
        ));
        let stream = Arc::new(Mutex::new(Vec::new()));
        let writer = stream.clone();
        set_refined_observer(&mut sim, move |_, s| {
            writer.lock().unwrap().push((
                s.elapsed,
                s.opposing_corner_impact,
                s.opposing_return_impact.copied(),
            ));
        })
        .unwrap();
        sim.step_exact(1).unwrap();
        let r = last_opposing_corner(&sim).unwrap().clone();
        assert_eq!(r.sim_time, Duration::from_nanos(161544));
        assert_eq!(r.interval.native_steps, count);
        let [upper] = r.interval.opposing_corner_impacts.as_slice() else {
            panic!("missing upper event ({count})")
        };
        let [lower] = r.interval.opposing_return_impacts.as_slice() else {
            panic!("missing lower return ({count})")
        };
        let physical = |h: gz_physics_multifidelity::RigidPointImpactReceipt| {
            h.pulse_start.as_secs_f64() + h.after_cut_s
        };
        assert!((physical(upper.point) * 1e9 - 10422.009445586242).abs() < 0.05);
        assert!((physical(lower.lower.point) * 1e9 - 140697.12751541147).abs() < 0.05);
        assert_eq!(lower.previous_upper_s, physical(upper.point));
        assert_eq!(upper.point.impact.foot, lower.lower.point.impact.foot);
        assert_ne!(upper.point.impact.support, lower.lower.point.impact.support);
        assert_eq!(upper.point.corner, lower.previous_upper_corner);
        assert!(lower.previous_upper_gaps_m[1] > 0.);
        for (point, j) in [
            (
                upper.point,
                DVec3::new(8.200609166505911, 5.931998450622482, -15.173246371413748),
            ),
            (
                lower.lower.point,
                DVec3::new(
                    -0.25282281045799526,
                    0.024654996782301285,
                    0.3893609642382788,
                ),
            ),
        ] {
            assert!(DVec3::from(point.impact.impulse_on_foot).distance(j) < 1e-7);
            let (f, s) = (point.impact.foot, point.impact.support);
            let pair = r
                .interval
                .pairs
                .iter()
                .find(|p| (p.a, p.b) == (f.min(s), f.max(s)))
                .unwrap();
            assert_eq!(pair.first_positive_end, Some(point.impact.elapsed));
            let sign = if pair.a == f { 1. } else { -1. };
            assert!(
                (sign
                    * (DVec3::from(pair.normal_impulse_on_a)
                        + DVec3::from(pair.tangent_impulse_on_a.unwrap())))
                .distance(j)
                    < 1e-7
            );
        }
        let samples = stream.lock().unwrap().clone();
        assert_eq!(samples.len(), count as usize);
        assert_eq!(samples.last().unwrap().0, r.sim_time);
        assert_eq!(samples.iter().filter(|s| s.1.is_some()).count(), 1);
        assert_eq!(samples.iter().filter(|s| s.2.is_some()).count(), 1);
        if count == 1 {
            assert!(samples[0].1.is_some() && samples[0].2.is_some());
            assert!(lower.lower.point.pulse_start > Duration::ZERO);
        }
        sim.set_paused(true);
        sim.tick().unwrap();
        assert_eq!(last_opposing_corner(&sim), Some(&r));
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert!(last_opposing_corner(&sim).is_none());
        sim.step_exact(1).unwrap();
        assert_eq!(last_opposing_corner(&sim).unwrap().interval, r.interval);
    }
}

#[test]
fn isolated_return_requires_a_measured_upper_and_explicit_enrollment() {
    for xml in [
        ORDERED.replace(RETURN_OPT, ""),
        ORDERED.replace(
            RETURN_OPT,
            "<project_opponent_return>false</project_opponent_return>",
        ),
    ] {
        let mut s = load(&xml);
        s.step_exact(1).unwrap();
        let r = last_opposing_corner(&s).unwrap();
        assert_eq!(r.interval.opposing_corner_impacts.len(), 1);
        assert!(r.interval.opposing_return_impacts.is_empty());
    }
    for bad in [
        ORDERED.replace(OPT, ""),
        ORDERED.replace(RETURN_OPT, &format!("{RETURN_OPT}{RETURN_OPT}")),
        ORDERED.replace(
            RETURN_OPT,
            "<project_opponent_return>yes</project_opponent_return>",
        ),
    ] {
        assert!(
            load_world_with_sdf_step(
                &bad,
                "bad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}
