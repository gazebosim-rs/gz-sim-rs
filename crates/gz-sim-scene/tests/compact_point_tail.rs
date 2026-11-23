// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact, reset_loaded_world};
use std::time::Duration;
const SCENE: &str = include_str!("../../../examples/multifidelity_compact_point_tail.sdf");
const TAG: &str = "<compact_point_tail>true</compact_point_tail>";
fn load(xml: &str) -> gz_sim_core::Simulation {
    load_world_with_sdf_step(
        xml,
        "primary-compact.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn first(xml: &str) -> primary_contact::Refinement {
    let mut sim = load(xml);
    sim.step_exact(1).unwrap();
    primary_contact::last_refinement(&sim).unwrap().clone()
}
#[test]
fn a_complete_swept_point_tail_spends_two_native_calls_on_the_original_clock() {
    let reference = first(
        &SCENE
            .replace(TAG, "")
            .replace("<native_steps>8", "<native_steps>2"),
    );
    for budget in [8, 64] {
        let mut sim = load(&SCENE.replace("<native_steps>8", &format!("<native_steps>{budget}")));
        sim.step_exact(1).unwrap();
        let r = primary_contact::last_refinement(&sim).unwrap().clone();
        let compressed = r
            .interval
            .compact_point_tail
            .expect("qualified original primary tail");
        assert_eq!(compressed.start, Duration::from_nanos(51784));
        assert_eq!(compressed.duration, Duration::from_nanos(448216));
        assert_eq!(r.interval.native_steps, 2);
        assert_eq!(compressed.omitted_native_steps, budget - 2);
        assert_eq!(r.sim_time - r.start_time, Duration::from_micros(500));
        assert_eq!(r.requested_pairs, reference.requested_pairs);
        assert_eq!(
            r.interval.point_transfers,
            reference.interval.point_transfers
        );
        let [p] = r.requested_pairs.as_slice() else {
            panic!("unexpected contacts")
        };
        let j =
            DVec3::from(p.normal_impulse_on_foot) + DVec3::from(p.tangent_impulse_on_foot.unwrap());
        assert!(
            j.distance(DVec3::new(
                0.27781231546053453,
                -0.12256766073352288,
                2.053663881929264
            )) < 5e-7,
            "{j:?}"
        );
        assert!((p.normal_impulse - 2.0536264544300478).abs() < 5e-7);
        sim.step_exact(0).unwrap();
        assert_eq!(primary_contact::last_refinement(&sim), Some(&r));
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        sim.step_exact(1).unwrap();
        assert_eq!(
            primary_contact::last_refinement(&sim).unwrap().interval,
            r.interval
        );
    }
}
#[test]
fn an_unrelated_prescribed_body_keeps_every_requested_native_interval() {
    let xml=SCENE.replace("</world>",r#"<model name="other"><pose>5 2 1 0 0 0</pose><plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>0.1 0.2 0.3</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin><link name="ball"><collision name="shape"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model></world>"#);
    let ordinary = first(&xml.replace(TAG, ""));
    let r = first(&xml);
    assert_eq!(r.interval.native_steps, 8);
    assert_eq!(r.interval.compact_point_tail, None);
    assert_eq!(r.interval, ordinary.interval);
}
#[test]
fn finite_arrival_detection_can_select_an_ordinary_grid_on_the_first_public_tick() {
    let (a, b) = SCENE.split_once("<project_finite_point>").unwrap();
    let (_, b) = b.split_once("</pair>").unwrap();
    let xml = format!("{a}<detect_finite_arrival>true</detect_finite_arrival></pair>{b}");
    let r = first(&xml);
    assert_eq!(r.interval.native_steps, 8);
    assert!(r.interval.finite_point_impacts.is_empty() && r.interval.point_transfers.is_empty());
    assert!(r.requested_pairs[0].normal_impulse > 0.);
    let mut sim = load(&xml.replace(
        "<detect_finite_arrival>true",
        "<detect_finite_arrival>false",
    ));
    sim.step_exact(1).unwrap();
    assert!(primary_contact::last_refinement(&sim).is_none());
}

#[test]
fn a_distant_fixed_obstacle_allows_the_certified_suffix() {
    let xml=SCENE.replace("</world>",r#"<model name="wall"><static>true</static><pose>4 5 0 0 0 0</pose><link name="wall"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision></link></model></world>"#);
    let r = first(&xml);
    assert_eq!(r.interval.native_steps, 2);
    assert_eq!(
        r.interval.compact_point_tail.unwrap().omitted_native_steps,
        6
    );
}

#[test]
fn a_bearing_can_spend_one_call_next_tick_and_a_new_deck_command_expires_its_pressure() {
    let xml=SCENE.replace("</pair>","<target_point_bounds>true</target_point_bounds><refine_active_point>true</refine_active_point></pair>");
    let mut sim = load(&xml);
    sim.step_exact(2).unwrap();
    let second = primary_contact::last_refinement(&sim).unwrap().clone();
    assert_eq!(second.sim_time, Duration::from_millis(1));
    let span = second
        .interval
        .compact_point_tail
        .expect("continued bearing over entire tick");
    assert_eq!(
        (span.start, span.duration, span.omitted_native_steps),
        (Duration::ZERO, Duration::from_micros(500), 7)
    );
    assert_eq!(second.interval.native_steps, 1);
    assert!(second.requested_pairs[0].normal_impulse > 0.);
    let id = sim
        .world()
        .iter_entities()
        .find_map(|e| {
            if e.get::<gz_sim_scene::Name>()?.0 != "default::lift::pad" {
                return None;
            }
            Some(e.get::<gz_sim_core::EntityId>()?.get())
        })
        .unwrap();
    gz_sim_scene::queue_link_world_velocity(&mut sim, id, [0., 0., -4.], [0.; 3]).unwrap();
    sim.step_exact(0).unwrap();
    assert_eq!(primary_contact::last_refinement(&sim), Some(&second));
    sim.step_exact(1).unwrap();
    let released = primary_contact::last_refinement(&sim).unwrap();
    assert_eq!(released.sim_time, Duration::from_micros(1500));
    assert!(released.interval.compact_point_tail.is_none());
    assert!(
        released.interval.point_transfers.is_empty()
            && released.interval.sticking_points.is_empty()
            && released.interval.sliding_skew_edges.is_empty()
    );
    assert_eq!(released.requested_pairs[0].normal_impulse, 0.);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(2).unwrap();
    assert_eq!(
        primary_contact::last_refinement(&sim).unwrap().interval,
        second.interval
    );
}

#[test]
fn live_point_loads_match_public_impulses_at_their_executed_boundaries() {
    use gz_physics_multifidelity::PointSpanLoad;
    use std::sync::{Arc, Mutex};
    let skew = include_str!("../../../examples/multifidelity_sliding_skew_edge.sdf");
    for xml in [
        SCENE.to_owned(),
        SCENE.replace(TAG, ""),
        SCENE.replace("<native_steps>8", "<native_steps>64"),
        skew.to_owned(),
    ] {
        let mut sim = load(&xml);
        let measured = Arc::new(Mutex::new(Vec::new()));
        let sink = Arc::clone(&measured);
        primary_contact::set_refined_observer(&mut sim, move |_, s| {
            let start = s.elapsed - s.duration;
            for h in s.finite_point_impacts {
                assert!(h.resultant.elapsed <= s.elapsed);
                assert!(
                    h.resultant.elapsed > start || start.is_zero() && h.resultant.elapsed.is_zero()
                );
                assert!(s.state(h.resultant.foot).is_some());
            }
            if let Some(p) = s.point_span {
                let (at, dt) = match p {
                    PointSpanLoad::Sliding(x) => (x.start, x.duration),
                    PointSpanLoad::Sticking(x) => (x.start, x.duration),
                    PointSpanLoad::Transfer(x) => (x.start, x.duration),
                };
                assert_eq!((at, dt), (start, s.duration));
            }
            sink.lock()
                .unwrap()
                .push((s.finite_point_impacts.to_vec(), s.point_span.cloned()));
        })
        .unwrap();
        sim.step_exact(0).unwrap();
        assert!(measured.lock().unwrap().is_empty());
        sim.step_exact(1).unwrap();
        let q = &primary_contact::last_refinement(&sim).unwrap().interval;
        let records = measured.lock().unwrap();
        assert_eq!(records.len(), q.native_steps as usize);
        let hits: Vec<_> = records.iter().flat_map(|(h, _)| h).copied().collect();
        assert_eq!(hits, q.finite_point_impacts);
        assert_eq!(hits.len(), 1);
        let spans: Vec<_> = records.iter().filter_map(|(_, s)| s.as_ref()).collect();
        assert!(!spans.is_empty());
        assert_eq!(
            spans.len(),
            q.sliding_vertices.len()
                + q.sliding_skew_edges.len()
                + q.sticking_points.len()
                + q.point_transfers.len()
        );
        let h = hits[0].resultant;
        let mut sum = DVec3::from(h.impulse_on_foot);
        let mut normal = sum.dot(h.normal.into());
        for s in spans {
            let (pair, n, j) = s.reaction();
            assert_eq!(pair, (h.foot, h.support));
            normal += n;
            sum += DVec3::from(j);
            match s {
                PointSpanLoad::Sliding(x) => {
                    assert!(q.sliding_vertices.contains(x) || q.sliding_skew_edges.contains(x))
                }
                PointSpanLoad::Sticking(x) => assert!(q.sticking_points.contains(x)),
                PointSpanLoad::Transfer(x) => assert!(q.point_transfers.contains(x)),
            }
        }
        let pair = q
            .pairs
            .iter()
            .find(|p| (p.a, p.b) == (h.foot.min(h.support), h.foot.max(h.support)))
            .unwrap();
        let signed = if pair.a == h.foot { 1. } else { -1. };
        assert!((normal - pair.normal_impulse).abs() < 1e-10);
        assert!(
            sum.distance(
                signed
                    * (DVec3::from(pair.normal_impulse_on_a)
                        + DVec3::from(pair.tangent_impulse_on_a.unwrap()))
            ) < 1e-10
        );
    }
}

#[test]
fn cold_supported_material_point_can_request_a_grid_before_the_first_solve() {
    check_cold_solve(
        include_str!("fixtures/cold_split_bearing.sdf"),
        8,
        DVec3::new(
            -0.002037917400387498,
            0.00045604119610154003,
            0.0042847432663452615,
        ),
    );
}

#[test]
fn the_previously_split_cold_bearing_now_preserves_its_load_in_one_call() {
    check_cold_solve(
        include_str!("../../../examples/multifidelity_cold_material_bearing.sdf"),
        1,
        DVec3::new(
            0.0022652901113727167,
            -0.0015676009105252426,
            0.00550981356697702,
        ),
    );
}

fn check_cold_solve(xml: &str, expected_calls: u32, expected_impulse: DVec3) {
    let option = "<detect_initial_point>true</detect_initial_point>";
    let mut control = load(&xml.replace(option, ""));
    control.step_exact(1).unwrap();
    let mut sim = load(xml);
    sim.step_exact(0).unwrap();
    assert!(primary_contact::last_refinement(&sim).is_none());
    sim.step_exact(1).unwrap();
    let observed = primary_contact::last_refinement(&sim).unwrap().clone();
    if expected_calls == 1 {
        let ordinary = primary_contact::last_refinement(&control).unwrap();
        assert_eq!(ordinary.interval.native_steps, 1);
        assert_eq!(ordinary.requested_pairs, observed.requested_pairs);
    } else {
        assert!(primary_contact::last_refinement(&control).is_none());
    }
    assert_eq!(observed.interval.native_steps, expected_calls);
    assert!(observed.interval.finite_point_impacts.is_empty());
    let mut spans = observed
        .interval
        .sliding_vertices
        .iter()
        .chain(&observed.interval.sliding_skew_edges)
        .map(|s| (s.start, s.duration))
        .chain(
            observed
                .interval
                .sticking_points
                .iter()
                .map(|s| (s.start, s.duration)),
        )
        .chain(
            observed
                .interval
                .point_transfers
                .iter()
                .map(|s| (s.start, s.duration)),
        )
        .collect::<Vec<_>>();
    spans.sort_unstable();
    let mut at = Duration::ZERO;
    for (t, d) in spans {
        assert_eq!(t, at);
        at += d;
    }
    assert_eq!(at, Duration::from_micros(500));
    let pair = &observed.requested_pairs[0];
    let j = DVec3::from(pair.normal_impulse_on_foot)
        + DVec3::from(pair.tangent_impulse_on_foot.unwrap());
    assert!(j.distance(expected_impulse) < 1e-11);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(1).unwrap();
    assert_eq!(
        primary_contact::last_refinement(&sim).unwrap().interval,
        observed.interval
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let pad = sim
        .world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<gz_sim_scene::Name>()?.0 == "default::lift::pad")
                .then(|| e.get::<gz_sim_core::EntityId>().unwrap().get())
        })
        .unwrap();
    gz_sim_scene::queue_link_world_velocity(&mut sim, pad, [0., 0., -10.], [0.; 3]).unwrap();
    sim.step_exact(0).unwrap();
    sim.step_exact(1).unwrap();
    if let Some(r) = primary_contact::last_refinement(&sim) {
        assert!(
            r.interval.point_transfers.is_empty()
                && r.interval.sliding_vertices.is_empty()
                && r.interval.sticking_points.is_empty()
                && r.interval.sliding_skew_edges.is_empty()
        );
        assert!(r.interval.pairs.iter().all(|p| p.normal_impulse == 0.));
    }
}

#[test]
fn enabling_cold_detection_preserves_an_already_complete_ordinary_bearing() {
    // This donor previously lost its bearing halfway through an eight-call
    // grid. Its cold first solve can carry the whole tick on a single span.
    let xml = include_str!("fixtures/cold_complete_bearing.sdf").replace(TAG, "");
    let baseline = first(&xml.replace("<detect_initial_point>true</detect_initial_point>", ""));
    assert_eq!(baseline.interval.native_steps, 1);
    let mut sim = load(&xml);
    sim.step_exact(1).unwrap();
    let receipt = &primary_contact::last_refinement(&sim).unwrap().interval;
    let decision = receipt
        .compact_point_tail
        .expect("ordinary whole-span trial");
    assert_eq!(
        (
            decision.start,
            decision.duration,
            decision.omitted_native_steps
        ),
        (Duration::ZERO, Duration::from_micros(500), 7)
    );
    let mut actual = receipt.clone();
    actual.compact_point_tail = None;
    assert_eq!(actual, baseline.interval);
}
