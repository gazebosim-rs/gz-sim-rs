// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine, load_world_with_sdf_step,
    primary_contact::{last_box_timing, last_refinement, set_enabled, set_refined_observer},
    remove_model, reset_loaded_world,
};
use sdformat::NoResources;
use std::{
    sync::{Arc, Mutex},
    time::Duration,
};
const SDF: &str = include_str!("../../../examples/multifidelity_local_prospective_timing.sdf");
const TAG: &str = "<prospective_timing>true</prospective_timing>";
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "rocker.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
#[test]
fn authored_retiming_observes_only_selected_ticks_and_replays_after_pause_remove_reset() {
    let mut s = load(SDF);
    let samples = Arc::new(Mutex::new(vec![]));
    let seen = samples.clone();
    set_refined_observer(&mut s, move |t, b| {
        seen.lock().unwrap().push((t.epoch, t.iteration, b.elapsed))
    })
    .unwrap();
    s.step_exact(8).unwrap();
    let first = last_refinement(&s).unwrap().clone();
    let last = last_box_timing(&s).unwrap().clone();
    assert_eq!(first.iteration, 5);
    assert_eq!(last.iteration, 8);
    assert_eq!(last.timing.decline, None);
    assert!(last.timing.retimed > 0);
    assert!(last.interval.pairs.iter().any(|p| p.normal_impulse > 0.1));
    let initial = samples.lock().unwrap().clone();
    assert_eq!(initial.len(), 16);
    assert!(initial.iter().all(|(_, t, _)| *t == 5 || *t == 8));
    assert_eq!(initial[7].2, Duration::from_micros(500));
    assert_eq!(initial[15].2, Duration::from_micros(500));
    s.set_paused(true);
    #[allow(deprecated)]
    let pad = s
        .world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<Name>()?.0 == "default::lift").then(|| e.get::<EntityId>().unwrap().get())
        })
        .unwrap();
    remove_model(&mut s, pad).unwrap();
    assert!(s.tick().unwrap().dt.is_zero());
    assert_eq!(last_box_timing(&s), Some(&last));
    s.step_exact(4).unwrap();
    assert_eq!(*samples.lock().unwrap(), initial);
    assert_eq!(last_box_timing(&s), Some(&last));
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_box_timing(&s).is_none());
    s.step_exact(8).unwrap();
    let again = last_box_timing(&s).unwrap();
    assert_eq!(again.epoch, last.epoch + 1);
    assert_eq!(again.timing, last.timing);
    assert_eq!(again.interval, last.interval);
    assert_eq!(samples.lock().unwrap().len(), 32);
}
#[test]
fn disabled_and_omitted_timing_use_no_prediction_witness_across_epochs() {
    let mut disabled = load(SDF);
    set_enabled(&mut disabled, false).unwrap();
    disabled.step_exact(8).unwrap();
    assert!(last_box_timing(&disabled).is_none() && last_refinement(&disabled).is_none());
    reset_loaded_world(&mut disabled, |_| ()).unwrap();
    disabled.step_exact(8).unwrap();
    assert!(last_box_timing(&disabled).is_some());
    let mut ordinary = load(&SDF.replace(TAG, ""));
    ordinary.step_exact(8).unwrap();
    assert!(last_box_timing(&ordinary).is_none());
    assert_eq!(last_refinement(&ordinary).unwrap().iteration, 5);
    for field in [
        "<prospective_timing>yes</prospective_timing>",
        "<prospective_timing>true</prospective_timing><prospective_timing>false</prospective_timing>",
    ] {
        assert!(
            load_world_with_sdf_step(
                &SDF.replace(TAG, field),
                "bad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn authored_guard_uses_ordinary_cadence_through_pause_removal_and_reset() {
    use gz_sim_scene::primary_contact::last_box_flight;
    const GUARD: &str = "<guard_separated_flight>true</guard_separated_flight>";
    let authored = include_str!("../../../examples/multifidelity_local_separated_flight.sdf");

    let mut s = load(authored);
    let samples = Arc::new(Mutex::new(Vec::new()));
    let seen = samples.clone();
    set_refined_observer(&mut s, move |t, b| {
        seen.lock().unwrap().push((t.iteration, b.elapsed));
    })
    .unwrap();
    s.step_exact(1).unwrap();
    let first = last_box_flight(&s).unwrap().clone();
    assert_eq!(first.flight.guarded, [true]);
    assert_eq!(first.flight.durations, [s.step_duration()]);
    assert!(last_box_timing(&s).is_none());
    assert!(samples.lock().unwrap().is_empty());
    s.set_paused(true);
    assert!(s.tick().unwrap().dt.is_zero());
    assert_eq!(last_box_flight(&s), Some(&first));
    set_enabled(&mut s, false).unwrap();
    s.step_exact(1).unwrap();
    assert_eq!(last_box_flight(&s), Some(&first));
    set_enabled(&mut s, true).unwrap();
    s.step_exact(1).unwrap();
    let before_remove = last_box_flight(&s).unwrap().clone();
    assert_eq!(before_remove.iteration, 3);
    #[allow(deprecated)]
    let pad = s
        .world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<Name>()?.0 == "default::lift").then(|| e.get::<EntityId>().unwrap().get())
        })
        .unwrap();
    remove_model(&mut s, pad).unwrap();
    s.step_exact(6).unwrap();
    assert_eq!(last_box_flight(&s), Some(&before_remove));
    assert!(samples.lock().unwrap().is_empty());
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_box_flight(&s).is_none());
    s.step_exact(1).unwrap();
    let again = last_box_flight(&s).unwrap();
    assert_eq!(again.epoch, first.epoch + 1);
    assert_eq!(again.flight, first.flight);
    s.step_exact(5).unwrap();
    assert_eq!(samples.lock().unwrap().len(), 8);
    let landing = last_box_flight(&s).unwrap();
    assert_eq!(landing.flight.guarded.len(), 8);
    assert!(landing.flight.guarded.contains(&false));

    let mut locked = load(&SDF.replace(TAG, &format!("{TAG}{GUARD}")));
    locked.step_exact(1).unwrap();
    let declined = last_box_flight(&locked).unwrap();
    assert_eq!(declined.flight.guarded, [false]);
    assert_eq!(
        declined.flight.decline,
        Some(gz_physics_multifidelity::BoxTimingDecline::GeneralizedContact)
    );

    let mut old = load(SDF);
    old.step_exact(8).unwrap();
    assert!(last_box_flight(&old).is_none());
    for policy in [
        GUARD.into(),
        format!("{TAG}{GUARD}{GUARD}"),
        format!("{TAG}<guard_separated_flight>yes</guard_separated_flight>"),
    ] {
        assert!(
            load_world_with_sdf_step(
                &SDF.replace(TAG, &policy),
                "invalid.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn torque_free_rotation_is_explicit_and_reconstructed_after_reset() {
    use gz_sim_scene::primary_contact::last_box_flight;
    const WORLD: &str = include_str!("../../../examples/multifidelity_local_gyroscopic_flight.sdf");
    const ROTATION: &str = "<integrate_free_rotation>true</integrate_free_rotation>";
    let mut sim = load(WORLD);
    let mut old = load(&WORLD.replace(ROTATION, ""));
    sim.step_exact(1).unwrap();
    old.step_exact(1).unwrap();
    let first = last_box_flight(&sim).unwrap().clone();
    assert!(first.flight.rotation_slices.as_ref().unwrap()[0] >= 2);
    assert!(
        last_box_flight(&old)
            .unwrap()
            .flight
            .rotation_slices
            .is_none()
    );
    sim.set_paused(true);
    let stamp = sim.tick().unwrap();
    assert!(stamp.dt.is_zero());
    assert_eq!(last_box_flight(&sim), Some(&first));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(last_box_flight(&sim).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(first.flight, last_box_flight(&sim).unwrap().flight);
    sim.step_exact(4).unwrap();
    let primary = last_refinement(&sim).unwrap();
    assert!(
        primary
            .requested_pairs
            .iter()
            .any(|p| p.normal_impulse > 0.)
    );
    let flight = last_box_flight(&sim).unwrap();
    for (&protected, &n) in flight
        .flight
        .guarded
        .iter()
        .zip(flight.flight.rotation_slices.as_ref().unwrap())
    {
        assert!(protected || n == 0);
    }
    for bad in [
        WORLD.replace("<guard_separated_flight>true</guard_separated_flight>", ""),
        WORLD.replace(
            ROTATION,
            "<integrate_free_rotation>ok</integrate_free_rotation>",
        ),
        WORLD.replace(ROTATION, &format!("{ROTATION}{ROTATION}")),
    ] {
        assert!(
            load_world_with_sdf_step(
                &bad,
                "invalid.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn cubic_corner_bracket_uses_the_pair_guard_and_restarts_from_measured_state() {
    const WORLD: &str = include_str!("../../../examples/multifidelity_local_cubic_rotation.sdf");
    const FLAG: &str = "<refine_rotation_timing>true</refine_rotation_timing>";
    let mut s = load(WORLD);
    s.step_exact(5).unwrap();
    let timing = last_box_timing(&s).unwrap().timing.clone();
    let flight = gz_sim_scene::primary_contact::last_box_flight(&s)
        .unwrap()
        .flight
        .clone();
    assert_eq!(timing.durations.len(), 8);
    assert_eq!(
        timing.durations.iter().sum::<Duration>(),
        Duration::from_micros(500)
    );
    let hit = timing.crossings[0];
    assert_eq!(hit.corner, 3);
    assert!(hit.earliest_ns <= 456_568 && hit.latest_ns >= 456_568);
    assert!(hit.latest_ns - hit.earliest_ns <= 32);
    let mut end = 0;
    for (&h, &guard) in timing.durations.iter().zip(&flight.guarded) {
        end += h.as_nanos() as u64;
        if end <= hit.earliest_ns {
            assert!(
                guard,
                "unprotected strictly separated interval ending at {end}"
            );
        }
    }
    s.set_paused(true);
    s.tick().unwrap();
    assert_eq!(last_box_timing(&s).unwrap().timing, timing);
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_box_timing(&s).is_none());
    s.step_exact(5).unwrap();
    assert_eq!(last_box_timing(&s).unwrap().timing, timing);
    let mut old = load(&WORLD.replace(FLAG, ""));
    old.step_exact(5).unwrap();
    assert!(last_box_timing(&old).unwrap().timing.crossings.is_empty());
    assert_eq!(last_box_timing(&old).unwrap().timing.cubic_queries, 0);
    for bad in [
        WORLD.replace(
            "<integrate_free_rotation>true</integrate_free_rotation>",
            "",
        ),
        WORLD.replace(FLAG, &format!("{FLAG}{FLAG}")),
        WORLD.replace(FLAG, "<refine_rotation_timing>yes</refine_rotation_timing>"),
    ] {
        assert!(
            load_world_with_sdf_step(
                &bad,
                "rocker.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn registered_impact_pulse_consumes_one_native_call_after_verified_separation() {
    const BASE: &str = include_str!("../../../examples/multifidelity_local_cubic_rotation.sdf");
    const END: &str = "</refine_rotation_timing>";
    let xml = |v: &str| {
        BASE.replace(
            END,
            &format!("{END}<rotation_contact_pulse>{v}</rotation_contact_pulse>"),
        )
    };
    for mode in ["measured", "64ns", "256ns"] {
        let mut sim = load(&xml(mode));
        sim.step_exact(5).unwrap();
        let t = &last_box_timing(&sim).unwrap().timing;
        let g = &gz_sim_scene::primary_contact::last_box_flight(&sim)
            .unwrap()
            .flight;
        assert_eq!(t.pulses.len(), 1);
        let pulse = t.pulses[0];
        assert_eq!(pulse.decline, None);
        let h = pulse.applied_ns.unwrap();
        assert!(pulse.arrival_by_ns.unwrap() < h);
        assert_eq!(
            h,
            match mode {
                "64ns" => 64,
                "256ns" => 256,
                _ => 4 * (pulse.arrival_by_ns.unwrap() + 1),
            }
        );
        let mut end = 0;
        let mut found = false;
        for (i, d) in t.durations.iter().enumerate() {
            if end == pulse.cut_ns {
                assert!(g.guarded[i - 1] && !g.guarded[i]);
                assert_eq!(d.as_nanos(), u128::from(h));
                found = true;
            }
            end += d.as_nanos() as u64;
        }
        assert!(found);
        assert_eq!(end, 500_000);
        assert_eq!(t.durations.len(), 8);
    }
    for bad in ["0ns", "4001ns", "1us", "32", ""] {
        assert!(
            load_world_with_sdf_step(
                &xml(bad),
                "rocker.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
    let missing =
        xml("measured").replace("<refine_rotation_timing>true</refine_rotation_timing>", "");
    assert!(
        load_world_with_sdf_step(
            &missing,
            "rocker.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity
        )
        .is_err()
    );
}

#[test]
fn observed_impact_conditions_followup_rotation_and_epoch_discards_the_witness() {
    use gz_sim_scene::primary_contact::last_box_flight;
    let base = include_str!("../../../examples/multifidelity_local_cubic_rotation.sdf").replace(
        "</refine_rotation_timing>",
        "</refine_rotation_timing><rotation_contact_pulse>measured</rotation_contact_pulse>",
    );
    let xml = base.replace(
        "</integrate_free_rotation>",
        "</integrate_free_rotation><integrate_released_rotation>true</integrate_released_rotation>",
    );
    let mut s = load(&xml);
    s.step_exact(4).unwrap();
    assert!(last_box_flight(&s).unwrap().flight.released.is_none());
    s.step_exact(1).unwrap();
    assert!(last_box_flight(&s).unwrap().flight.released.is_none());
    s.set_paused(true);
    s.tick().unwrap();
    s.step_exact(1).unwrap();
    let flight = last_box_flight(&s).unwrap();
    assert_eq!(flight.flight.guarded, [true]);
    let record = flight.flight.released.as_ref().unwrap()[0];
    assert!(record.integrated && record.minimum_gap_pm.unwrap() > 0);
    let expected = flight.flight.clone();
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_box_flight(&s).is_none());
    s.step_exact(6).unwrap();
    assert_eq!(last_box_flight(&s).unwrap().flight, expected);
    reset_loaded_world(&mut s, |_| ()).unwrap();
    s.step_exact(5).unwrap();
    set_enabled(&mut s, false).unwrap();
    set_enabled(&mut s, true).unwrap();
    s.step_exact(1).unwrap();
    assert!(
        last_box_flight(&s)
            .is_none_or(|f| f.sim_time != s.elapsed() || f.flight.released.is_none())
    );
    let bad = xml.replace(
        "<integrate_free_rotation>true</integrate_free_rotation>",
        "",
    );
    assert!(
        load_world_with_sdf_step(
            &bad,
            "rocker.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity
        )
        .is_err()
    );
}

#[test]
fn separated_sweep_enrolls_a_false_negative_and_never_reuses_a_paused_decision() {
    use gz_physics_multifidelity::RigidBoxSweep;
    use gz_sim_scene::primary_contact::last_box_sweep;
    const WORLD: &str =
        include_str!("../../../examples/multifidelity_local_separated_crossings.sdf");
    const TAG: &str = "<detect_separated_crossings>true</detect_separated_crossings>";
    let mut old = load(&WORLD.replace(TAG, ""));
    old.step_exact(4).unwrap();
    assert!(last_refinement(&old).is_none());
    let mut s = load(WORLD);
    s.step_exact(4).unwrap();
    let cut = last_box_sweep(&s).unwrap();
    assert_eq!(cut.sim_time, Duration::from_millis(2));
    assert!(matches!(cut.pairs[0].2,RigidBoxSweep::Possible{vertices} if vertices.count_ones()>0));
    let timing = last_box_timing(&s).unwrap().timing.clone();
    assert_eq!(
        timing.durations.iter().sum::<Duration>(),
        Duration::from_micros(500)
    );
    assert!(!timing.crossings.is_empty() && timing.pulses[0].applied_ns.is_some());
    let first = last_refinement(&s).unwrap().requested_pairs[0]
        .first_positive_end
        .unwrap();
    assert_eq!(first, Duration::from_nanos(174_533));
    s.set_paused(true);
    s.tick().unwrap();
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_box_sweep(&s).is_none());
    s.step_exact(4).unwrap();
    assert_eq!(last_box_timing(&s).unwrap().timing, timing);
    let bad = WORLD.replace("<refine_rotation_timing>true</refine_rotation_timing>", "");
    assert!(
        load_world_with_sdf_step(
            &bad,
            "rocker.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity
        )
        .is_err()
    );
}

#[test]
fn compact_landing_requires_both_measured_contact_and_a_certified_remaining_flight() {
    use gz_sim_scene::primary_contact::last_box_flight;
    const WORLD: &str = include_str!("../../../examples/multifidelity_local_compact_impact.sdf");
    const FLAG: &str = "<compact_impact_budget>true</compact_impact_budget>";
    let mut ordinary = load(&WORLD.replace(FLAG, ""));
    ordinary.step_exact(5).unwrap();
    let full = last_box_timing(&ordinary).unwrap();
    assert_eq!(full.interval.native_steps, 8);
    assert!(full.timing.compact.is_none());
    let mut sim = load(WORLD);
    sim.step_exact(5).unwrap();
    let actual = last_box_timing(&sim).unwrap().timing.clone();
    let compact = actual.compact.unwrap();
    assert!(
        compact.approach_planned
            && compact.pulse_planned
            && compact.complete
            && compact.decline.is_none()
    );
    assert!(compact.tail_gap_pm.is_some());
    assert_eq!(actual.durations.len(), 3);
    assert_eq!(
        actual.durations.iter().sum::<Duration>(),
        Duration::from_micros(500)
    );
    assert_eq!(
        last_box_flight(&sim).unwrap().flight.guarded,
        [true, false, true]
    );
    assert!(last_refinement(&sim).unwrap().requested_pairs[0].normal_impulse > 0.);
    sim.set_paused(true);
    sim.tick().unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(last_box_timing(&sim).is_none());
    sim.step_exact(5).unwrap();
    assert_eq!(last_box_timing(&sim).unwrap().timing, actual);
    for missing in [
        "<detect_separated_crossings>true</detect_separated_crossings>",
        "<rotation_contact_pulse>measured</rotation_contact_pulse>",
    ] {
        assert!(
            load_world_with_sdf_step(
                &WORLD.replace(missing, ""),
                "compact.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn gravity_advection_uses_actual_qualified_free_cuts_through_pause_and_reset() {
    use gz_sim_scene::primary_contact::last_box_flight;
    const WORLD: &str = include_str!("../../../examples/multifidelity_local_triaxial_gravity.sdf");
    const FLAG: &str = "<integrate_ballistic_translation>true</integrate_ballistic_translation>";
    let mut baseline = load(&WORLD.replace(FLAG, ""));
    let mut s = load(WORLD);
    for _ in 0..3 {
        s.step_exact(1).unwrap();
        baseline.step_exact(1).unwrap();
        assert_eq!(
            last_box_flight(&s).unwrap().flight.ballistic_steps,
            Some(vec![true])
        );
        assert!(
            last_box_flight(&baseline)
                .unwrap()
                .flight
                .ballistic_steps
                .is_none()
        );
    }
    s.set_paused(true);
    let before = last_box_flight(&s).unwrap().clone();
    s.tick().unwrap();
    assert_eq!(&before, last_box_flight(&s).unwrap());
    s.step_exact(1).unwrap();
    let flight = last_box_flight(&s).unwrap().flight.clone();
    let record = last_refinement(&s).unwrap();
    assert_eq!(
        record.requested_pairs[0].first_positive_end,
        Some(Duration::from_nanos(238020))
    );
    let first = flight
        .durations
        .iter()
        .scan(Duration::ZERO, |t, d| {
            *t += *d;
            Some(*t)
        })
        .position(|t| Some(t) == record.requested_pairs[0].first_positive_end)
        .unwrap();
    assert!(!flight.ballistic_steps.as_ref().unwrap()[first]);
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_box_flight(&s).is_none());
    s.step_exact(4).unwrap();
    assert_eq!(last_box_flight(&s).unwrap().flight, flight);
    let bad = WORLD.replace(
        "<integrate_free_rotation>true</integrate_free_rotation>",
        "",
    );
    assert!(
        load_world_with_sdf_step(
            &bad,
            "gravity.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity
        )
        .is_err()
    );
}

const TRIAXIAL_POINT_WORLD: &str =
    include_str!("../../../examples/multifidelity_local_triaxial_point.sdf");
const POINT_POLICY: &str = "<project_rigid_corner>true</project_rigid_corner>";

#[test]
fn measured_rigid_corner_reports_the_fractional_reaction_and_restarts_on_reset() {
    use glam::DVec3;
    let mut sim = load(TRIAXIAL_POINT_WORLD);
    let stream = Arc::new(Mutex::new(Vec::new()));
    let seen = stream.clone();
    set_refined_observer(&mut sim, move |_, b| {
        let native = b
            .contact_patches()
            .into_iter()
            .flat_map(|p| p.points)
            .map(|p| p.normal_impulse)
            .sum::<f64>();
        seen.lock()
            .unwrap()
            .push((b.elapsed, b.rigid_point_impact, native));
    })
    .unwrap();
    sim.step_exact(5).unwrap();
    let flight = last_box_timing(&sim).unwrap().clone();
    let [hit] = flight.interval.rigid_point_impacts.as_slice() else {
        panic!("isolated vertex should project once");
    };
    assert_eq!(hit.corner, 0);
    assert_eq!(hit.impact.elapsed, Duration::from_nanos(229_373));
    assert!(
        hit.pulse_start <= hit.impact.elapsed
            && hit.impact.elapsed < hit.pulse_start + hit.pulse_duration
    );
    assert!(hit.outward_acceleration > 0. && !hit.impact.sticking);
    assert!((hit.impact.friction - 0.8).abs() < 1e-7);
    let reference = DVec3::new(0.652377499665548, 1.4498326039628517, 1.7811493682503405);
    let impulse = DVec3::from(hit.impact.impulse_on_foot);
    assert!(impulse.distance(reference) < 5e-7);
    assert!((hit.impact.dissipated_energy - 3.684303784062843).abs() < 1e-6);
    let p = &last_refinement(&sim).unwrap().requested_pairs[0];
    assert_eq!(p.first_positive_end, Some(hit.impact.elapsed));
    let normal = DVec3::from(p.normal_impulse_on_foot);
    let n = DVec3::from(hit.impact.normal);
    assert!((normal.dot(n) - impulse.dot(n)).abs() < 4e-5);
    // This fixture has a second, smaller native grouped friction load after
    // the explicit impact. The integrated tangent must retain that uncertainty.
    assert!(p.tangent_impulse_on_foot.is_none());
    let mut resolved = gz_sim_scene::load_world_with_runtime(
        TRIAXIAL_POINT_WORLD,
        "point.sdf",
        "default",
        &NoResources,
        gz_sim_scene::WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        gz_sim_scene::WorldRuntimeOptions {
            rapier_per_point_friction: true,
            ..Default::default()
        },
    )
    .unwrap();
    resolved.step_exact(5).unwrap();
    let resolved = &last_refinement(&resolved).unwrap().requested_pairs[0];
    let tangent = DVec3::from(resolved.tangent_impulse_on_foot.unwrap());
    assert!(normal.dot(tangent).abs() < 1e-11);
    assert!((normal + tangent).distance(impulse) < 5e-5);
    let pulses = stream.lock().unwrap().clone();
    assert_eq!(pulses.len(), 8);
    let projected: Vec<_> = pulses.iter().filter(|(_, h, _)| h.is_some()).collect();
    assert_eq!(projected.len(), 1);
    assert_eq!(projected[0].1.as_ref(), Some(hit));
    assert_eq!(projected[0].2, 0.);
    assert_eq!(projected[0].0, hit.pulse_start + hit.pulse_duration);
    assert_eq!(pulses.last().unwrap().0, Duration::from_micros(500));
    sim.set_paused(true);
    sim.tick().unwrap();
    assert_eq!(last_box_timing(&sim), Some(&flight));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(last_box_timing(&sim).is_none());
    sim.step_exact(5).unwrap();
    let again = last_box_timing(&sim).unwrap();
    assert_eq!(again.epoch, flight.epoch + 1);
    assert_eq!(again.timing, flight.timing);
    assert_eq!(again.interval, flight.interval);
}

#[test]
fn rigid_corner_declines_extra_colliders_and_rejects_incomplete_registration() {
    use gz_physics_multifidelity::RigidPointDecline;
    let clutter = r#"<model name="neighbor"><static>true</static><pose>5 0 0 0 0 0</pose><link name="base"><collision name="part"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model>"#;
    let text = TRIAXIAL_POINT_WORLD.replace("</world>", &format!("{clutter}</world>"));
    let mut sim = load(&text);
    let mut base = load(&text.replace(POINT_POLICY, ""));
    sim.step_exact(5).unwrap();
    base.step_exact(5).unwrap();
    let w = last_box_timing(&sim).unwrap();
    let reference = last_box_timing(&base).unwrap();
    assert_eq!(w.interval, reference.interval);
    assert_eq!(w.timing.durations, reference.timing.durations);
    assert!(w.interval.rigid_point_impacts.is_empty());
    assert!(
        w.timing
            .point_attempts
            .as_ref()
            .unwrap()
            .iter()
            .any(|(_, why)| *why == Some(RigidPointDecline::Environment))
    );
    assert!(reference.timing.point_attempts.is_none());
    assert_eq!(
        last_refinement(&sim).unwrap().requested_pairs,
        last_refinement(&base).unwrap().requested_pairs
    );
    for invalid in [
        TRIAXIAL_POINT_WORLD.replace(
            "<detect_separated_crossings>true</detect_separated_crossings>",
            "",
        ),
        TRIAXIAL_POINT_WORLD.replace(
            "<rotation_contact_pulse>measured</rotation_contact_pulse>",
            "<rotation_contact_pulse>64ns</rotation_contact_pulse>",
        ),
        TRIAXIAL_POINT_WORLD.replace(POINT_POLICY, &format!("{POINT_POLICY}{POINT_POLICY}")),
    ] {
        assert!(
            load_world_with_sdf_step(
                &invalid,
                "bad.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn known_tangency_qualifies_only_the_immediate_outgoing_solve_and_preserves_its_reaction() {
    const DEPART: &str = "<follow_rigid_departure>true</follow_rigid_departure>";
    let text = TRIAXIAL_POINT_WORLD.replace(POINT_POLICY, &format!("{POINT_POLICY}{DEPART}"));
    let mut sim = load(&text);
    sim.step_exact(5).unwrap();
    let t = last_box_timing(&sim).unwrap().clone();
    let hit = t.interval.rigid_point_impacts[0];
    let [(start, h)] = t.timing.departure_calls.as_ref().unwrap().as_slice() else {
        panic!("expected one certified native outgoing solve");
    };
    assert_eq!(*start, hit.pulse_start + hit.pulse_duration);
    assert!(!h.is_zero() && *start + *h <= Duration::from_micros(500));
    let p = &last_refinement(&sim).unwrap().requested_pairs[0];
    let normal = glam::DVec3::from(p.normal_impulse_on_foot);
    let tangent = glam::DVec3::from(p.tangent_impulse_on_foot.unwrap());
    assert!((normal + tangent).distance(hit.impact.impulse_on_foot.into()) < 1e-11);
    let reactions = last_refinement(&sim).unwrap().requested_pairs.clone();
    sim.step_exact(3).unwrap();
    let last = last_box_timing(&sim).unwrap();
    if last.iteration > t.iteration {
        assert!(
            last.timing
                .departure_calls
                .as_ref()
                .is_none_or(Vec::is_empty)
        );
    }
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(5).unwrap();
    let again = last_box_timing(&sim).unwrap();
    assert_eq!(again.timing, t.timing);
    assert_eq!(again.interval, t.interval);
    assert_eq!(last_refinement(&sim).unwrap().requested_pairs, reactions);
    for invalid in [
        text.replace(POINT_POLICY, ""),
        text.replace(DEPART, &format!("{DEPART}{DEPART}")),
    ] {
        assert!(
            load_world_with_sdf_step(
                &invalid,
                "departure.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn explicitly_swept_third_obstacles_preserve_the_two_body_projection_and_dynamic_refusal() {
    let clear = "<project_rigid_corner>clear_neighbors</project_rigid_corner>";
    let obstacle = |kind: &str| {
        format!(
            r#"<model name="opponent"><pose>1.2 0 0 0 0 0</pose>{kind}<link name="bar"><collision name="contact"><geometry><box><size>.1 .22 .1</size></box></geometry></collision></link></model>"#
        )
    };
    let translated = r#"<plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>-1 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>"#;
    let flags = TRIAXIAL_POINT_WORLD.replace(POINT_POLICY, clear);
    let mut bare = load(&flags);
    bare.step_exact(5).unwrap();
    let hit = last_box_timing(&bare)
        .unwrap()
        .interval
        .rigid_point_impacts
        .clone();
    assert_eq!(hit.len(), 1);
    for kind in ["<static>true</static>", translated] {
        let mut sim = load(&flags.replace("</world>", &format!("{}</world>", obstacle(kind))));
        sim.step_exact(5).unwrap();
        let r = last_box_timing(&sim).unwrap();
        assert_eq!(r.interval.rigid_point_impacts, hit);
        assert_eq!(
            last_refinement(&sim).unwrap().requested_pairs,
            last_refinement(&bare).unwrap().requested_pairs
        );
    }
    let dynamic = flags.replace("</world>", &format!("{}</world>", obstacle("")));
    let mut legacy = load(&dynamic.replace(clear, POINT_POLICY));
    let mut requested = load(&dynamic);
    legacy.step_exact(5).unwrap();
    requested.step_exact(5).unwrap();
    let a = last_box_timing(&legacy).unwrap();
    let b = last_box_timing(&requested).unwrap();
    assert_eq!(a.interval, b.interval);
    assert_eq!(a.timing.durations, b.timing.durations);
    assert!(b.interval.rigid_point_impacts.is_empty());
}

#[test]
fn moving_third_body_receives_its_native_reaction_after_a_measured_primary_impulse() {
    use gz_physics::DynamicBackend;
    use gz_physics::PhysicsBackend;
    use gz_physics_multifidelity::MultiFidelityBackend;
    use gz_sim_scene::BodyHandle;
    const XML: &str = include_str!("../../../examples/multifidelity_local_rigid_neighbors.sdf");
    let mut sim = load(XML);
    #[allow(deprecated)]
    let id = |name: &str| {
        sim.world()
            .iter_entities()
            .find_map(|e| (e.get::<Name>()?.0 == name).then(|| e.get::<BodyHandle>().unwrap().id()))
            .unwrap()
    };
    let foot = id("default::rocker::foot");
    let gate = id("default::opponent::gate");
    sim.step_exact(6).unwrap();
    let hit = last_box_timing(&sim).unwrap().interval.rigid_point_impacts[0];
    assert_eq!(hit.impact.elapsed, Duration::from_nanos(155_600));
    assert_eq!(hit.impact.foot, foot);
    assert_ne!(hit.impact.support, gate);
    let first = last_refinement(&sim).unwrap().requested_pairs[0];
    assert_eq!(first.first_positive_end, Some(hit.impact.elapsed));
    sim.step_exact(1).unwrap();
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    let ids = (foot.min(gate), foot.max(gate));
    let measured = native
        .rigid_contact_observations()
        .into_iter()
        .filter(|c| (c.a, c.b) == ids)
        .map(|c| c.normal_impulse)
        .sum::<f64>();
    assert!(
        measured > 0.,
        "third body must remain in the ordinary solver"
    );
    assert!(
        native
            .state(gate)
            .unwrap()
            .velocity
            .iter()
            .any(|x| x.abs() > 1.)
    );
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    sim.step_exact(6).unwrap();
    assert_eq!(
        last_box_timing(&sim).unwrap().interval.rigid_point_impacts,
        [hit]
    );
}

const NAMED_OPPONENT: &str =
    include_str!("../../../examples/multifidelity_local_named_opponent.sdf");
const OPPONENT_FIELD: &str = "<guard_opponent>default::opponent::gate</guard_opponent>";

#[test]
fn named_opponent_guard_releases_real_ceiling_and_rebinds_after_paused_removal() {
    use gz_physics::PhysicsBackend;
    use gz_physics_multifidelity::MultiFidelityBackend;
    use gz_sim_scene::BodyHandle;
    use gz_sim_scene::primary_contact::last_box_flight;
    let mut s = load(NAMED_OPPONENT);
    #[allow(deprecated)]
    let id = |s: &Simulation, key: &str| {
        s.world()
            .iter_entities()
            .find_map(|e| (e.get::<Name>()?.0 == key).then(|| e.get::<BodyHandle>().unwrap().id()))
            .unwrap()
    };
    let gate = id(&s, "default::opponent::gate");
    let foot = id(&s, "default::rocker::foot");
    s.step_exact(5).unwrap();
    let hit = last_box_timing(&s)
        .unwrap()
        .interval
        .rigid_point_impacts
        .clone();
    assert_eq!(hit.len(), 1);
    assert_eq!(hit[0].impact.elapsed, Duration::from_nanos(229373));
    let guards = last_box_flight(&s).unwrap().flight.clone();
    assert!(
        guards
            .opposing
            .as_ref()
            .unwrap()
            .iter()
            .flatten()
            .any(|g| g.opponent == gate && g.minimum_gap_pm > 0)
    );
    s.step_exact(1).unwrap();
    let rows = s
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .rigid_contact_observations();
    assert!(
        rows.iter()
            .any(|r| (r.a, r.b) == (foot.min(gate), foot.max(gate)) && r.normal_impulse > 9.)
    );
    s.set_paused(true);
    let past = last_box_flight(&s).unwrap().clone();
    #[allow(deprecated)]
    let model = s
        .world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<Name>()?.0 == "default::opponent").then(|| e.get::<EntityId>().unwrap().get())
        })
        .unwrap();
    remove_model(&mut s, model).unwrap();
    assert!(s.tick().unwrap().dt.is_zero());
    assert_eq!(last_box_flight(&s), Some(&past));
    s.step_exact(2).unwrap();
    if let Some(current) = last_box_flight(&s).filter(|r| r.iteration > past.iteration) {
        assert!(current.flight.opposing.is_none());
    }
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_box_flight(&s).is_none());
    s.step_exact(5).unwrap();
    assert_eq!(last_box_flight(&s).unwrap().flight, guards);
    assert_eq!(
        last_box_timing(&s).unwrap().interval.rigid_point_impacts,
        hit
    );
}

#[test]
fn ambiguous_named_opponent_registration_is_rejected_before_stepping() {
    let xml = NAMED_OPPONENT;
    for bad in [
        xml.replace("clear_neighbors","true"),
        xml.replace(OPPONENT_FIELD,&format!("{OPPONENT_FIELD}{OPPONENT_FIELD}")),
        xml.replace(OPPONENT_FIELD,"<guard_opponent>default::lift::pad</guard_opponent>"),
        xml.replace(OPPONENT_FIELD,"<guard_opponent>default::rocker::foot</guard_opponent>"),
        xml.replace(OPPONENT_FIELD,"<guard_opponent>default::missing::gate</guard_opponent>"),
        xml.replace(OPPONENT_FIELD,"<guard_opponent>gate</guard_opponent>"),
        xml.replace("default::opponent::gate</guard_opponent>","default::extra::ball</guard_opponent>").replace("</world>",r#"<model name="extra"><link name="ball"><collision name="x"><geometry><sphere><radius>.1</radius></sphere></geometry></collision></link></model></world>"#),
        xml.replace("default::opponent::gate</guard_opponent>","default::extra::box</guard_opponent>").replace("</world>",r#"<model name="extra"><link name="box"><collision name="x"><geometry><box><size>.1 .1 .1</size></box></geometry></collision></link></model></world>"#),
    ]{
        assert!(load_world_with_sdf_step(&bad,"bad.sdf","default",&NoResources,PhysicsEngine::MultiFidelity).is_err());
    }
}

#[test]
fn unbounded_fourth_body_restores_both_contact_pairs_and_preserves_its_public_forces() {
    use gz_physics::{DynamicBackend, PhysicsBackend};
    use gz_physics_multifidelity::MultiFidelityBackend;
    use gz_sim_scene::BodyHandle;
    use gz_sim_scene::primary_contact::last_box_flight;
    let floor = r#"<model name="floor"><static>true</static><pose>2 0 -.05 0 0 0</pose><link name="pad"><collision name="c"><geometry><box><size>.8 .8 .1</size></box></geometry></collision></link></model>"#;
    let witness = r#"<model name="extra"><pose>2 0 .049 0 0 0</pose><link name="box"><collision name="c"><geometry><box><size>.1 .1 .1</size></box></geometry></collision></link></model>"#;
    let xml = NAMED_OPPONENT.replace("</world>", &format!("{floor}{witness}</world>"));
    let run = |xml: &str| {
        let mut s = load(xml);
        #[allow(deprecated)]
        let id = |key: &str| {
            s.world()
                .iter_entities()
                .find_map(|e| {
                    (e.get::<Name>()?.0 == key).then(|| e.get::<BodyHandle>().unwrap().id())
                })
                .unwrap()
        };
        let subject = id("default::rocker::foot");
        let gate = id("default::opponent::gate");
        let fourth = id("default::extra::box");
        let base = id("default::floor::pad");
        let mut ticks = vec![];
        for _ in 0..8 {
            s.world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .add_world_wrench(fourth, [1., 0., -20.], [0.; 3])
                .unwrap();
            s.step_exact(1).unwrap();
            let e = s
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            let timing = last_box_timing(&s).filter(|t| t.sim_time == s.elapsed());
            let log = last_box_flight(&s).filter(|t| t.sim_time == s.elapsed());
            if let Some(log) = timing {
                assert!(log.interval.rigid_point_impacts.is_empty());
            }
            if let Some(proofs) = log.and_then(|l| l.flight.opposing.as_ref()) {
                assert!(proofs.iter().all(Option::is_none));
            }
            let load = |a, b| {
                let pair = if a < b { (a, b) } else { (b, a) };
                if let Some(t) = timing {
                    t.interval
                        .pairs
                        .iter()
                        .filter(|r| (r.a, r.b) == pair)
                        .map(|r| r.normal_impulse)
                        .sum::<f64>()
                } else {
                    e.rigid_contact_observations()
                        .into_iter()
                        .filter(|r| (r.a, r.b) == pair)
                        .map(|r| r.normal_impulse)
                        .sum::<f64>()
                }
            };
            ticks.push((
                e.state(subject).unwrap(),
                e.state(fourth).unwrap(),
                load(subject, gate),
                load(fourth, base),
            ));
        }
        ticks
    };
    let base = run(&xml.replace(OPPONENT_FIELD, ""));
    let named = run(&xml);
    assert_eq!(base, named);
    assert!(named.iter().any(|t| t.2 > 0.));
    assert!(named.iter().any(|t| t.3 > 0.));
}
