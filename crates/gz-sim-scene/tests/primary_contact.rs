// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    ArticulationTier, BodyHandle, Name, PhysicsEngine, begin_model_free_coast,
    load_world_with_sdf_step, model_fidelity,
    primary_contact::{Refinement, last_refinement},
    queue_link_world_wrench, remove_model, reset_loaded_world, set_model_fidelity,
};
use sdformat::NoResources;

const SDF: &str = include_str!("../../../examples/multifidelity_local_primary.sdf");
const PAIR: &str =
    "<pair><foot>default::rocker::foot</foot><support>default::lift::pad</support></pair>";
fn load(sdf: &str) -> Simulation {
    load_world_with_sdf_step(
        sdf,
        "primary.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    #[allow(deprecated)]
    sim.world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == format!("default::{name}"))
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn state(sim: &Simulation, link: u64) -> BodyState {
    let body = sim
        .world()
        .get::<BodyHandle>(entity_by_id(sim.world(), link).unwrap())
        .unwrap()
        .id();
    sim.world()
        .resource::<PhysicsBackend>()
        .state(body)
        .unwrap()
}
fn first(sim: &mut Simulation) -> Refinement {
    for _ in 0..400 {
        sim.step_exact(1).unwrap();
        if let Some(r) = last_refinement(sim) {
            return r.clone();
        }
    }
    panic!("no primary interval")
}
fn plugin_free(sdf: &str) -> String {
    let start = sdf
        .find("    <plugin name=\"gz::sim::systems::PrimaryContactRefinement\"")
        .unwrap();
    let end = sdf[start..].find("</plugin>").unwrap() + start + "</plugin>".len();
    format!("{}{}", &sdf[..start], &sdf[end..])
}

#[test]
fn removed_binding_pauses_and_loaded_reset_replays_the_same_impulse() {
    let mut sim = load(SDF);
    let target = id(&sim, "rocker");
    assert!(
        begin_model_free_coast(&mut sim, target)
            .unwrap_err()
            .to_string()
            .contains("independent native clock")
    );
    let before = first(&mut sim);
    assert_eq!(before.requested_pairs.len(), 1);
    let q = before.requested_pairs[0];
    assert!(q.normal_impulse > 1. && q.first_positive_end.unwrap() <= sim.step_duration());
    assert!(q.normal_impulse > q.last_native_normal_impulse && q.active_at_end);
    assert_eq!(before.interval.native_steps, 8);
    assert_eq!(before.sim_time - before.start_time, sim.step_duration());
    sim.set_paused(true);
    let pad = id(&sim, "lift");
    remove_model(&mut sim, pad).unwrap();
    assert!(sim.tick().unwrap().dt.is_zero());
    assert_eq!(last_refinement(&sim), Some(&before));
    sim.step_exact(60).unwrap();
    assert_eq!(last_refinement(&sim), Some(&before));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(last_refinement(&sim).is_none());
    let again = first(&mut sim);
    assert_eq!(again.epoch, before.epoch + 1);
    assert_eq!(again.iteration, before.iteration);
    assert_eq!(again.requested_pairs, before.requested_pairs);
    assert_eq!(again.interval, before.interval);
}

#[test]
fn simultaneous_pairs_share_one_tick_and_hold_a_mixed_world_wrench_once() {
    let start = SDF.find("    <model name=\"rocker\">").unwrap();
    let end = SDF.find("  </world>").unwrap();
    let other = SDF[start..end]
        .replace("name=\"rocker\"", "name=\"other_rocker\"")
        .replace("name=\"lift\"", "name=\"other_lift\"")
        .replace("<pose>0 0 0.20 ", "<pose>0 4 0.20 ")
        .replace("<pose>0.18 0 -0.06 ", "<pose>0.18 4 -0.06 ");
    let witness = r#"<model name="witness"><pose>4 0 2 0 0 0</pose><link name="wheel"><inertial><mass>2</mass></inertial><collision name="c"><geometry><sphere><radius>0.1</radius></sphere></geometry></collision></link></model>"#;
    let sdf = SDF
        .replace("  </world>", &format!("{other}{witness}</world>"))
        .replace(
            PAIR,
            &format!(
                "{PAIR}{}",
                PAIR.replace("::rocker::", "::other_rocker::")
                    .replace("::lift::", "::other_lift::")
            ),
        );
    let mut calibration = load(&sdf);
    let first = first(&mut calibration);
    assert_eq!(first.requested_pairs.len(), 2);
    assert_eq!(first.interval.native_steps, 8);
    assert!(first.requested_pairs.iter().all(|r| r.normal_impulse > 1.));
    let mut sim = load(&sdf);
    let mut control = load(&plugin_free(&sdf));
    let foot = id(&sim, "rocker::foot");
    let witness = id(&sim, "witness::wheel");
    sim.step_exact(first.iteration - 1).unwrap();
    control.step_exact(first.iteration - 1).unwrap();
    assert_eq!(state(&sim, foot), state(&control, foot));
    assert_eq!(state(&sim, witness), state(&control, witness));
    let dt = sim.step_duration().as_secs_f64();
    for s in [&mut sim, &mut control] {
        queue_link_world_wrench(s, witness, [24., 0., 0.], [0.; 3]).unwrap();
        s.step_exact(1).unwrap();
        assert!((state(s, witness).velocity[0] - 12. * dt).abs() < 1e-12);
        s.step_exact(1).unwrap();
        assert!((state(s, witness).velocity[0] - 12. * dt).abs() < 1e-12);
    }
    assert!(last_refinement(&control).is_none());
    assert_eq!(last_refinement(&sim).unwrap().iteration, first.iteration);
}
#[test]
fn prescribed_foot_and_disconnected_foot_skip_selection() {
    let mut sim = load(
        &SDF.replace("<gravity>0 0 -9.81", "<gravity>0 0 0")
            .replace("<linear_velocity>0 0 0.08", "<linear_velocity>0 0 1.50"),
    );
    let rocker = id(&sim, "rocker");
    set_model_fidelity(&mut sim, rocker, ArticulationTier::Prescribed).unwrap();
    sim.step_exact(400).unwrap();
    assert_eq!(
        model_fidelity(&sim, rocker).unwrap(),
        ArticulationTier::Prescribed
    );
    assert!(last_refinement(&sim).is_none());
    let mut overhang = load(&SDF.replace("<pose>0.18 0 -0.06 ", "<pose>1.0 0 -0.06 "));
    overhang.step_exact(400).unwrap();
    assert!(last_refinement(&overhang).is_none());
}
#[test]
fn invalid_configuration_is_rejected_during_loading() {
    let invalid = [
        (SDF.replace("</pair>", "<returning_pad_face>+z</returning_pad_face></pair><early_face_step_ns>100</early_face_step_ns><early_face_max_steps>4</early_face_max_steps>"), "native_steps grid"),
        (
            SDF.replace(
                "</pair>",
                "<returning_pad_face>+z</returning_pad_face></pair>",
            ),
            "authored reduced pad stiffness",
        ),
        (SDF.replace(PAIR, &format!("{PAIR}{PAIR}")), "unique"),
        (
            SDF.replace("default::lift::pad", "default::missing"),
            "scoped link",
        ),
        (
            SDF.replace("<native_steps>8", "<native_steps>0"),
            "native_steps",
        ),
        (
            SDF.replace("<native_steps>8", "<native_steps>500001"),
            "invalid numeric",
        ),
        (
            SDF.replace("<solver_iterations>8", "<solver_iterations>33"),
            "solver_iterations",
        ),
        (
            SDF.replace("</pair>", "<weight>10</weight></pair>"),
            "one foot",
        ),
        (
            SDF.replace("<pose>0.18 0 -0.06 0 0 0", "<pose>0.18 0 -0.06 0 0.01 0"),
            "horizontal",
        ),
        (
            SDF.replace("default::rocker::foot", "default::lift::pad"),
            "distinct models",
        ),
        (
            SDF.replace("<angular_velocity>0 0 0", "<angular_velocity>0 0 0.01"),
            "horizontal",
        ),
    ];
    for (sdf, expected) in invalid {
        let error = load_world_with_sdf_step(
            &sdf,
            "invalid.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .err()
        .unwrap()
        .to_string();
        assert!(error.contains(expected), "{expected}: {error}");
    }
    let moving = include_str!("../../../examples/multifidelity_curved_shuttle.sdf");
    let from = moving.find("<trajectory ").unwrap();
    let to = from + moving[from..].find("</trajectory>").unwrap() + "</trajectory>".len();
    let incompatible = SDF.replace(
        "<linear_velocity>0 0 0.08</linear_velocity><angular_velocity>0 0 0</angular_velocity>",
        &moving[from..to],
    );
    let err = load_world_with_sdf_step(
        &incompatible,
        "curve.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .err()
    .unwrap();
    assert!(err.to_string().contains("independently clocked"), "{err}");
    let wrong = load_world_with_sdf_step(
        SDF,
        "invalid.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .err()
    .unwrap();
    assert!(wrong.to_string().contains("--physics-engine multifidelity"));
}

#[test]
fn authored_relaxation_matches_centered_zero_slip_level_impact() {
    const LEVEL: &str = include_str!("../../../examples/multifidelity_level_face.sdf");
    let mut errs = Vec::new();
    for r in [1, 4, 8, 16, 32] {
        let mut sim = load(&LEVEL.replace(
            "<relaxation_iterations>8",
            &format!("<relaxation_iterations>{r}"),
        ));
        let foot = id(&sim, "rocker::foot");
        let pad = id(&sim, "lift::pad");
        let event = first(&mut sim);
        assert_eq!(event.iteration, 5);
        assert_eq!(event.solver.relaxation, Some(r));
        assert_eq!(event.solver.pgs, Some(1));
        assert_eq!(event.solver.temporal, 4);
        let p = event.requested_pairs[0];
        let b = state(&sim, foot);
        let spin = glam::DVec3::from(b.angular_velocity).length();
        let excess = (p.normal_impulse - 2.8).abs();
        eprintln!(
            "relaxation={r} impulse={} spin={spin} normal_relative_velocity={}",
            p.normal_impulse,
            b.velocity[2] - state(&sim, pad).velocity[2]
        );
        errs.push((excess, spin));
    }
    assert!(errs[0].0 > 1e-3 && errs[0].1 > 1e-3);
    assert!(errs[2].0 < errs[1].0 && errs[1].0 < errs[0].0);
    assert!(errs[2].1 < errs[1].1 && errs[1].1 < errs[0].1);
    assert!(errs.windows(2).all(|w| w[1].0 < w[0].0 && w[1].1 < w[0].1));
    assert!(errs[4].0 < 1e-5 && errs[4].1 < 1e-5);
}

const BOX_FEATURES: &str = include_str!("../../../examples/multifidelity_local_box_features.sdf");
#[test]
fn authored_box_features_recompute_only_live_contacts_and_replay_after_reset() {
    use gz_sim_scene::primary_contact::last_box_feature_refresh;
    let mut sim = load(BOX_FEATURES);
    let first_r = first(&mut sim);
    let first_geometry = last_box_feature_refresh(&sim).unwrap().clone();
    assert_eq!(first_r.iteration, first_geometry.iteration);
    assert_eq!(first_geometry.native_steps, 8);
    assert_eq!(first_geometry.pairs.len(), 1);
    assert!(first_geometry.rebuilt_manifolds > 0);
    sim.step_exact(1).unwrap();
    let retained = last_box_feature_refresh(&sim).unwrap().clone();
    assert_eq!(retained.iteration, first_geometry.iteration + 1);
    assert_eq!(retained.native_steps, 1);
    assert_eq!(retained.pairs, first_geometry.pairs);
    assert!(retained.rebuilt_manifolds > 0);
    sim.set_paused(true);
    let lift = id(&sim, "lift");
    remove_model(&mut sim, lift).unwrap();
    assert!(sim.tick().unwrap().dt.is_zero());
    assert_eq!(last_box_feature_refresh(&sim), Some(&retained));
    sim.step_exact(8).unwrap();
    assert_eq!(last_box_feature_refresh(&sim), Some(&retained));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(last_box_feature_refresh(&sim).is_none());
    let again = first(&mut sim);
    let geometry = last_box_feature_refresh(&sim).unwrap();
    assert_eq!(again.iteration, first_r.iteration);
    assert_eq!(again.requested_pairs, first_r.requested_pairs);
    assert_eq!(geometry.epoch, first_geometry.epoch + 1);
    assert_eq!(geometry.pairs, first_geometry.pairs);
    assert_eq!(geometry.rebuilt_manifolds, first_geometry.rebuilt_manifolds);
}
#[test]
fn authored_box_feature_admission_accepts_inclined_translating_supports_and_requires_explicit_booleans()
 {
    let tilted = BOX_FEATURES.replace(
        "<pose>0 0 -0.06 0 0 0</pose>",
        "<pose>0 0 -0.06 0.03 0.02 0</pose>",
    );
    let mut sim = load(&tilted);
    let r = first(&mut sim);
    assert!(r.requested_pairs[0].normal_impulse > 0.);
    for bad in [tilted.replace("<refresh_box_features>true</refresh_box_features>","<refresh_box_features>false</refresh_box_features>"),
        BOX_FEATURES.replace("<refresh_box_features>true</refresh_box_features>","<refresh_box_features>yes</refresh_box_features>"),
        BOX_FEATURES.replace("<refresh_box_features>true</refresh_box_features>","<refresh_box_features>true</refresh_box_features><refresh_box_features>false</refresh_box_features>")]{
        assert!(load_world_with_sdf_step(&bad,"bad.sdf","default",&NoResources,PhysicsEngine::MultiFidelity).is_err());
    }
}
#[test]
fn simultaneous_authored_pairs_refresh_only_the_enrolled_foot() {
    use gz_sim_scene::primary_contact::last_box_feature_refresh;
    let split = BOX_FEATURES.find("    <model name=\"rocker\">").unwrap();
    let end = BOX_FEATURES.find("  </world>").unwrap();
    let other = BOX_FEATURES[split..end]
        .replace("name=\"rocker\"", "name=\"other_rocker\"")
        .replace("name=\"lift\"", "name=\"other_lift\"")
        .replace("0.01579394861948648", "4.01579394861948648")
        .replace("<pose>0 0 -0.06 ", "<pose>0 4 -0.06 ");
    let two=BOX_FEATURES.replacen("</pair>","</pair><pair><foot>default::other_rocker::foot</foot><support>default::other_lift::pad</support></pair>",1)
        .replace("  </world>",&format!("{other}  </world>"));
    let mut sim = load(&two);
    let r = first(&mut sim);
    assert_eq!(r.requested_pairs.len(), 2);
    let feature = last_box_feature_refresh(&sim).unwrap();
    assert_eq!(feature.pairs.len(), 1);
    let enrolled = r
        .requested_pairs
        .iter()
        .find(|p| feature.pairs[0] == (p.foot, p.support))
        .unwrap();
    let other = r
        .requested_pairs
        .iter()
        .find(|p| feature.pairs[0] != (p.foot, p.support))
        .unwrap();
    let mut on = load(BOX_FEATURES);
    let mut off = load(&two.replace("<refresh_box_features>true</refresh_box_features>", ""));
    assert!(
        (first(&mut on).requested_pairs[0].normal_impulse - enrolled.normal_impulse).abs() < 1e-9
    );
    let baseline = first(&mut off);
    let other_baseline = baseline
        .requested_pairs
        .iter()
        .find(|p| (p.foot, p.support) == (other.foot, other.support))
        .unwrap();
    assert_eq!(other, other_baseline);
}

#[test]
fn switching_one_foot_to_fresh_features_preserves_another_foot_on_the_same_pad() {
    use gz_sim_scene::primary_contact::last_box_feature_refresh;
    let source = BOX_FEATURES
        .replace("0.66 0.36 0.12", "0.66 1.6 0.12")
        .replace("0.01579394861948648", "-0.3042060513805135");
    let start = source.find("    <model name=\"rocker\">").unwrap();
    let next = source.find("    <model name=\"lift\">").unwrap();
    let other = source[start..next]
        .replace("name=\"rocker\"", "name=\"other_rocker\"")
        .replace("-0.3042060513805135", "0.3357939486194865");
    let paired=source.replacen("</pair>","</pair><pair><foot>default::other_rocker::foot</foot><support>default::lift::pad</support></pair>",1)
        .replace("  </world>",&format!("{other}  </world>"));
    let run = |sdf: &str| {
        let mut sim = load(sdf);
        let primary = first(&mut sim);
        let first_fresh = last_box_feature_refresh(&sim).cloned();
        let body = sim
            .world()
            .get::<BodyHandle>(entity_by_id(sim.world(), id(&sim, "other_rocker::foot")).unwrap())
            .unwrap()
            .id();
        sim.step_exact(3).unwrap();
        let endpoint = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(body)
            .unwrap();
        (primary, first_fresh, body, endpoint)
    };
    let (on, work, body, end) = run(&paired);
    let (off, _, other, baseline) =
        run(&paired.replace("<refresh_box_features>true</refresh_box_features>", ""));
    let selected = work.unwrap();
    assert_eq!(selected.pairs.len(), 1);
    assert_ne!(selected.pairs[0].0, body);
    assert_eq!(body, other);
    assert_eq!(
        on.requested_pairs.iter().find(|p| p.foot == body),
        off.requested_pairs.iter().find(|p| p.foot == body)
    );
    assert_eq!(end, baseline);
}

#[test]
fn turning_an_authored_policy_on_in_flight_preserves_the_ordinary_prefix() {
    use gz_sim_scene::primary_contact::{PolicyError, set_enabled};
    let mut sim = load(SDF);
    let mut ordinary = load(&plugin_free(SDF));
    assert!(matches!(
        set_enabled(&mut ordinary, true),
        Err(PolicyError::Unconfigured)
    ));
    let foot = id(&sim, "rocker::foot");
    set_enabled(&mut sim, false).unwrap();
    sim.step_exact(290).unwrap();
    ordinary.step_exact(290).unwrap();
    assert!(last_refinement(&sim).is_none());
    assert_eq!(state(&sim, foot), state(&ordinary, foot));
    sim.set_paused(true);
    set_enabled(&mut sim, true).unwrap();
    assert!(sim.tick().unwrap().dt.is_zero());
    let e = first(&mut sim);
    assert_eq!(e.iteration, 309);
    assert!(e.requested_pairs[0].normal_impulse > 2.);
    set_enabled(&mut sim, false).unwrap();
    sim.step_exact(60).unwrap();
    assert_eq!(last_refinement(&sim), Some(&e));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(first(&mut sim).iteration, 309);
}

#[test]
fn observing_selected_substeps_preserves_motion_and_the_public_receipt() {
    use gz_sim_scene::primary_contact::{clear_refined_observer, set_refined_observer};
    use std::sync::{Arc, Mutex};
    let mut observed = load(SDF);
    let mut control = load(SDF);
    let foot = id(&observed, "rocker::foot");
    let upper = id(&observed, "rocker::upper");
    let body = |id| {
        observed
            .world()
            .get::<BodyHandle>(entity_by_id(observed.world(), id).unwrap())
            .unwrap()
            .id()
    };
    let [foot_body, upper_body] = [foot, upper].map(body);
    let samples = Arc::new(Mutex::new(Vec::new()));
    let output = Arc::clone(&samples);
    set_refined_observer(&mut observed, move |time, s| {
        output.lock().unwrap().push((
            time.epoch,
            time.iteration,
            s.elapsed,
            s.duration,
            s.contact_patches().len(),
            [s.state(foot_body).unwrap(), s.state(upper_body).unwrap()],
        ));
    })
    .unwrap();
    let a = first(&mut observed);
    let b = first(&mut control);
    assert_eq!(a, b);
    assert_eq!(state(&observed, foot), state(&control, foot));
    let rows = samples.lock().unwrap().clone();
    assert_eq!(rows.len(), a.interval.native_steps as usize);
    assert!(rows.iter().all(|r| r.0 == a.epoch && r.1 == a.iteration));
    assert_eq!(rows.last().unwrap().2, observed.step_duration());
    assert_eq!(
        rows.last().unwrap().5,
        [state(&control, foot), state(&control, upper)]
    );
    assert_eq!(
        rows.iter().map(|r| r.3).sum::<std::time::Duration>(),
        observed.step_duration()
    );
    clear_refined_observer(&mut observed).unwrap();
    reset_loaded_world(&mut observed, |_| ()).unwrap();
    first(&mut observed);
    assert_eq!(*samples.lock().unwrap(), rows);
}

#[test]
fn returning_geometry_is_scoped_to_the_selected_pair_and_reset_epoch() {
    use gz_sim_scene::primary_contact::{last_box_feature_refresh, last_returning_box_features};
    let option = "<refresh_returning_box_features>true</refresh_returning_box_features>";
    let source = SDF.replace("</support></pair>",&format!("</support>{option}</pair>"));
    let mut sim=load(&source);
    let landed=first(&mut sim);
    let receipt=last_returning_box_features(&sim).unwrap().clone();
    let [pair]=receipt.pairs.as_slice() else{panic!("one authored pair");};
    assert_eq!((pair.foot,pair.support),(landed.requested_pairs[0].foot,landed.requested_pairs[0].support));
    assert!(pair.counts.requery<=1);
    assert_eq!(pair.counts.requery,pair.counts.stronger_plane);
    assert_eq!(pair.counts.first_witness.is_some(),pair.counts.requery==1);
    assert!(last_box_feature_refresh(&sim).is_none());
    sim.set_paused(true);
    let lift=id(&sim,"lift");
    remove_model(&mut sim,lift).unwrap();
    assert!(sim.tick().unwrap().dt.is_zero());
    sim.step_exact(2).unwrap();
    assert_eq!(last_returning_box_features(&sim),Some(&receipt));
    reset_loaded_world(&mut sim,|_|()).unwrap();
    assert!(last_returning_box_features(&sim).is_none());
    let replayed=first(&mut sim);
    let again=last_returning_box_features(&sim).unwrap();
    assert_eq!(replayed.interval,landed.interval);
    assert_eq!(again.pairs,receipt.pairs);
    assert_eq!(again.epoch,receipt.epoch+1);
}

#[test]
fn continuous_and_returning_refresh_on_the_same_pair_are_ambiguous() {
    let source=SDF.replace("</support></pair>","</support><refresh_box_features>true</refresh_box_features><refresh_returning_box_features>true</refresh_returning_box_features></pair>");
    let result=load_world_with_sdf_step(&source,"invalid.sdf","default",&NoResources,PhysicsEngine::MultiFidelity);
    assert!(result.err().unwrap().to_string().contains("either continuous or returning"));
}

#[test]
fn authored_returning_foot_requeries_a_disproved_plane_once_and_repeats_after_reset() {
    use gz_sim_scene::primary_contact::last_returning_box_features;
    let sdf = include_str!("../../../examples/multifidelity_returning_box_features.sdf");
    let mut sim = load(sdf);
    let mut baseline = load(&sdf.replace("<refresh_returning_box_features>true</refresh_returning_box_features>", ""));
    let links = ["rocker::foot", "rocker::upper", "lift::pad"].map(|n| id(&sim, n));
    sim.step_exact(241).unwrap();
    assert_eq!(last_refinement(&sim).unwrap().iteration, 241);
    assert_eq!(last_returning_box_features(&sim).unwrap().pairs[0].counts.requery, 0);
    sim.step_exact(55).unwrap();
    baseline.step_exact(296).unwrap();
    for link in links {
        assert_eq!(state(&sim, link), state(&baseline, link));
    }
    let check = |sim: &mut Simulation| {
        sim.step_exact(1).unwrap();
        let result = last_refinement(sim).unwrap();
        assert_eq!(result.iteration, 297);
        assert_eq!(result.interval.native_steps, 8);
        let work = last_returning_box_features(sim).unwrap();
        let [reported] = work.pairs.as_slice() else { panic!("one registered pair") };
        assert_eq!(reported.counts.requery, 1);
        let witness = reported.counts.first_witness.unwrap();
        assert!(witness.unloaded_points > 0);
        assert!(witness.gap_m[1] > witness.gap_m[0] + witness.tolerance_m);
        assert!(witness.gap_m[1] > 0.);
        assert_eq!(reported.foot, result.requested_pairs[0].foot);
        (result.requested_pairs[0], work.clone())
    };
    let (first, evidence) = check(&mut sim);
    baseline.step_exact(1).unwrap();
    let ordinary = last_refinement(&baseline).unwrap();
    assert_eq!(ordinary.iteration, 297);
    assert!(first.normal_impulse > ordinary.requested_pairs[0].normal_impulse + 0.001);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(last_returning_box_features(&sim).is_none());
    sim.step_exact(296).unwrap();
    let (again, repeated) = check(&mut sim);
    assert_eq!(again, first);
    assert_eq!(repeated.epoch, evidence.epoch + 1);
    assert_eq!(repeated.pairs, evidence.pairs);
}

#[test]
fn finite_face_certificate_resolves_the_shipped_nearly_flat_return() {
    let sdf = include_str!("../../../examples/multifidelity_certified_face_vertex.sdf");
    let flag = "<certify_face_vertex>true</certify_face_vertex>";
    let mut certified = load(sdf);
    let mut control = load(&sdf.replace(flag, ""));
    let regular = first(&mut control);
    let actual = first(&mut certified);
    assert_eq!(actual.interval.native_steps, 8);
    assert_eq!(regular.interval.native_steps, 8);
    assert_eq!(actual.iteration, 1);
    assert!(regular.interval.finite_point_impacts.is_empty());
    let [first] = actual.interval.finite_point_impacts.as_slice() else { panic!("one isolated initial impulse") };
    assert_eq!(first.resultant.elapsed.as_nanos(), 295339);
    assert_eq!(first.feature, gz_physics::SeparatingFeature::SupportFace(2));
    let physical = [-0.564080443, 0.309184478, 1.28651745];
    for (observed, reference) in first.resultant.impulse_on_foot.iter().zip(physical) {
        assert!((observed-reference).abs()<1e-7);
    }
    let invalid=sdf.replace("<project_finite_point>true</project_finite_point>", "");
    assert!(load_world_with_sdf_step(&invalid,"wrong.sdf","default",&NoResources,PhysicsEngine::MultiFidelity).is_err());
}

#[test]
fn directed_bounds_resolve_three_distinct_material_transfers_after_the_first_return() {
    let base = include_str!("../../../examples/multifidelity_certified_face_vertex.sdf");
    let after = "<subdivide_point_bounds>true</subdivide_point_bounds>";
    let aimed = "<target_point_bounds>true</target_point_bounds>";
    let s = base.replace("</pair>", &format!("{after}</pair>"));
    let mut midpoint = load(&s);
    let mut directed = load(&s.replace(after, &format!("{after}{aimed}")));
    let previous = first(&mut midpoint);
    let current = first(&mut directed);
    assert_eq!(current.interval.native_steps, previous.interval.native_steps);
    assert_eq!(current.interval.finite_point_impacts, previous.interval.finite_point_impacts);
    assert_eq!(current.interval.native_steps, 8);
    assert!(previous.interval.point_transfers.is_empty());
    let onset = current.interval.finite_point_impacts[0].resultant.elapsed.as_secs_f64();
    let jumps: Vec<_> = current.interval.point_transfers.iter().flat_map(|r|
        r.intervals.iter().filter_map(move |s| s.impact.map(|i|
            ((r.start.as_secs_f64() - onset + s.offset + s.duration) * 1e9, i.impulse_on_foot)))
    ).collect();
    let physical = [
        (6.57942168, [-0.158022772, -0.030501339, 0.321879033]),
        (7.71211511, [0.283305626, -0.379462010, 1.415126738]),
        (9.77157775, [-0.028868818, 0.033195231, 0.150488407]),
    ];
    assert_eq!(jumps.len(), physical.len());
    for ((when, signed), (expected, target)) in jumps.into_iter().zip(physical) {
        assert!((when - expected).abs() < 1e-5);
        assert!(signed.iter().zip(target).all(|(a, b)| (a-b).abs() < 2e-8));
    }
    let pair = &current.requested_pairs[0];
    let target = [-0.466965325, -0.06681168, 3.176620109];
    let tangent = pair.tangent_impulse_on_foot.unwrap();
    for k in 0..3 {
        assert!((pair.normal_impulse_on_foot[k] + tangent[k] - target[k]).abs() < 2e-8);
    }
}

#[test]
fn skew_certificate_recovers_a_single_site_and_the_rest_of_its_authored_return() {
    let sdf=include_str!("../../../examples/multifidelity_certified_skew_edge.sdf");
    let mut certified=load(sdf);
    let mut ordinary=load(&sdf.replace("<certify_skew_edge>true</certify_skew_edge>",""));
    let base=first(&mut ordinary);
    let measured=first(&mut certified);
    assert_eq!(base.interval.native_steps,8);
    assert_eq!(measured.interval.native_steps,8);
    assert!(base.interval.finite_point_impacts.is_empty());
    let [event]=measured.interval.finite_point_impacts.as_slice() else{panic!("one measured isolated edge impact")};
    assert_eq!(event.feature,gz_physics::SeparatingFeature::EdgePair(0,1));
    assert_eq!(event.resultant.elapsed.as_nanos(),383002);
    let physical=[0.547293053,0.351772979,1.301190106];
    for (a,b) in event.resultant.impulse_on_foot.into_iter().zip(physical) {assert!((a-b).abs()<6e-9);}
    let pair=&measured.requested_pairs[0];
    let tangent=pair.tangent_impulse_on_foot.unwrap();
    let net=[0.206261821,0.581502137,2.123571528];
    for k in 0..3 {assert!((pair.normal_impulse_on_foot[k]+tangent[k]-net[k]).abs()<1e-8);}
}

#[test]
fn two_material_strikes_inside_one_nanosecond_share_a_public_reaction_and_reset() {
    let source = include_str!("../../../examples/multifidelity_fractional_material_points.sdf");
    let mut sim = load(source);
    let check = |sim: &mut Simulation| {
        sim.step_exact(1).unwrap();
        let r = last_refinement(sim).unwrap().clone();
        assert_eq!(r.iteration, 1);
        assert_eq!(r.interval.native_steps, 8);
        let [path] = r.interval.fractional_point_paths.as_slice() else { panic!("expected one qualified bracket") };
        assert_eq!(path.native_boundary.as_nanos(), 224079);
        assert_eq!(path.events.len(), 2);
        let expected = [
            (224078.268588928, [0.607979663, 0.370310501, 1.423754387]),
            (224078.450460181, [-0.380236574, 0.239860127, 0.899138994]),
        ];
        for (e, (when, impulse)) in path.events.iter().zip(expected) {
            assert!((e.time_s * 1e9 - when).abs() < 0.0001);
            assert!(e.dissipated_energy >= 0.);
            assert!(e.impulse_on_foot.iter().zip(impulse).all(|(a,b)| (a-b).abs() < 2e-8));
        }
        let pair = &r.requested_pairs[0];
        let tangent = pair.tangent_impulse_on_foot.unwrap();
        let net = [0.2267484672969044, 0.610794716517816, 2.32524179736049];
        for k in 0..3 {
            assert!((pair.normal_impulse_on_foot[k] + tangent[k] - net[k]).abs() < 1e-9);
        }
        sim.step_exact(0).unwrap();
        assert_eq!(last_refinement(sim), Some(&r));
        r.interval
    };
    let original = check(&mut sim);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(check(&mut sim), original);
}

#[test]
fn fractional_paths_decline_unmodelled_wrenches_and_neighbor_contacts() {
    let sdf = include_str!("../../../examples/multifidelity_fractional_material_points.sdf");
    let third = r#"<model name="ceiling"><static>true</static><pose>0.175 0 -0.093264 0 0 0</pose><link name="pad"><collision name="c"><geometry><box><size>0.66 0.36 0.12</size></box></geometry></collision></link></model>"#;
    for (world, force) in [(sdf.replace("</world>", &format!("{third}</world>")), false), (sdf.to_owned(), true)] {
        let mut sim = load(&world);
        let mut twin = load(&world.replace("<resolve_fractional_points>true</resolve_fractional_points>", ""));
        for s in [&mut sim, &mut twin] {
            if force {
                let link = id(s, "rocker::upper");
                queue_link_world_wrench(s, link, [10., -2., 5.], [0.;3]).unwrap();
            }
            s.step_exact(1).unwrap();
            assert!(last_refinement(s).is_none_or(|r|r.interval.fractional_point_paths.is_empty()));
        }
        for name in ["rocker::foot","rocker::upper","lift::pad"] {
            assert_eq!(state(&sim, id(&sim,name)), state(&twin,id(&twin,name)));
        }
    }
}

#[test]
fn a_translating_parcel_hits_the_finite_face_between_two_disjoint_endpoint_footprints() {
    let xml = include_str!("../../../examples/worlds/local_sorter_crossing.sdf");
    let mut selected = load(xml);
    let raw = plugin_free(xml);
    let fine_xml = raw.replace("0.02</max_step_size>", "0.0002</max_step_size>");
    let mut fine = load(&fine_xml);
    let mut ordinary = load(&raw);
    let part = id(&selected, "parcel::body");
    let bypass = id(&selected, "bypass::body");
    // Without contact the two 8 mm parcels cross from x=-20 to +20 mm.
    // Both endpoint footprints are clear of the 12 mm pad. At 10 ms only
    // the orange parcel is over its pad, arriving at 4 m/s along the normal.
    for epoch in 0..2 {
        selected.step_exact(1).unwrap();
        fine.step_exact(100).unwrap();
        ordinary.step_exact(1).unwrap();
        let r = last_refinement(&selected).unwrap();
        assert_eq!(
            (
                r.epoch,
                r.iteration,
                r.requested_pairs.len(),
                r.interval.native_steps
            ),
            (epoch, 1, 1, 100)
        );
        let pair = r.requested_pairs[0];
        assert_eq!(pair.foot_link, part);
        let arriving = state(&selected, part);
        assert_eq!(arriving, state(&fine, part));
        assert!(
            (pair.normal_impulse_on_foot[2] - 0.01 * (arriving.velocity[2] + 4.)).abs() < 1e-10
        );
        assert!((pair.normal_impulse - 0.04).abs() < 0.0001);
        let first = pair.first_positive_end.unwrap().as_secs_f64();
        assert!((first - 0.010).abs() < 0.001);
        assert!((arriving.position[0] - 0.020).abs() < 1e-12);
        assert!((state(&ordinary, part).position[0] - arriving.position[0]).abs() > 0.020);
        let free = state(&selected, bypass);
        assert_eq!(free.velocity, [2., 0., -4.]);
        assert_eq!(free, state(&fine, bypass));
        eprintln!(
            "sorter epoch {epoch}, first positive end {first:.9} s, vertical impulse {:.12} N s, public body at {:?}",
            pair.normal_impulse_on_foot[2], arriving.position
        );
        selected.step_exact(3).unwrap();
        assert_eq!(last_refinement(&selected).unwrap().iteration, 1);
        for s in [&mut selected, &mut fine, &mut ordinary] {
            s.reset().unwrap();
        }
    }
}

#[test]
fn the_touching_solver_requires_one_centered_parallel_impact_and_a_valid_native_grid() {
    let raw = include_str!("../../../examples/worlds/local_sorter_crossing.sdf");
    let tag = "<touching_solver><native_steps>4</native_steps><solver_iterations>8</solver_iterations><pgs_iterations>16</pgs_iterations></touching_solver>";
    let xml = raw
        .replacen("<pair>", &format!("{tag}<pair>"), 1)
        .replace("-0.02 0 0.079", "0 0 0.039")
        .replace("2 0 -4</linear_velocity>", "0 0 -4</linear_velocity>");
    for (world, calls, pairs) in [
        (xml.clone(), 4, 1),
        (xml.replace("<pgs_iterations>16</pgs_iterations>", ""), 4, 1),
        (xml.replace("-0.02 0.11 0.079", "0 0.05 0.039"), 100, 2),
        (xml.replace("0 0 0.039", "0.006 0 0.039"), 100, 1),
        (
            xml.replace("0 0 -4</linear_velocity>", "0.01 0 -4</linear_velocity>"),
            100,
            1,
        ),
    ] {
        let mut sim = load(&world);
        for _ in 0..2 {
            sim.step_exact(1).unwrap();
            let r = last_refinement(&sim).unwrap();
            assert_eq!(
                (r.interval.native_steps, r.requested_pairs.len()),
                (calls, pairs)
            );
            assert_eq!(r.solver.temporal, if calls == 4 { 8 } else { 4 });
            sim.reset().unwrap();
        }
    }
    // The second lane is bearing a load when a paused edit places the first parcel at its pad.
    let mut loaded = load(
        &xml.replace("0 0 0.039", "0.1 0 0.039")
            .replace("-0.02 0.11 0.079", "0 0.05 0.039"),
    );
    loaded.step_exact(1).unwrap();
    let red = id(&loaded, "parcel");
    let mut pose = gz_sim_scene::model_state(&loaded, red).unwrap();
    pose.translation = [0., 0., 0.039];
    pose.linear_velocity = [0., 0., -4.];
    gz_sim_scene::set_model_state(&mut loaded, red, pose).unwrap();
    assert!(
        loaded
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
            .unwrap()
            .active_contact_pairs()
            > 0
    );
    loaded.step_exact(1).unwrap();
    let resumed = last_refinement(&loaded).unwrap();
    assert_eq!(resumed.requested_pairs.len(), 1);
    assert_eq!(resumed.interval.native_steps, 100);
    let try_load = |xml: &str| {
        load_world_with_sdf_step(
            xml,
            "test.sdf",
            "default",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
    };
    for bad in [
        "",
        "<pgs_iterations>1</pgs_iterations>",
        "<native_steps>4</native_steps>",
        "<native_steps>0</native_steps><solver_iterations>4</solver_iterations>",
        "<native_steps>4</native_steps><solver_iterations>0</solver_iterations>",
        "<native_steps>bad</native_steps><solver_iterations>4</solver_iterations>",
        "<native_steps>4</native_steps><solver_iterations>x</solver_iterations>",
        "<native_steps>4</native_steps><solver_iterations>4</solver_iterations><pgs_iterations>x</pgs_iterations>",
        "<native_steps>4</native_steps><solver_iterations>4</solver_iterations><pgs_iterations>4</pgs_iterations><pgs_iterations>4</pgs_iterations>",
        "<native_steps a='b'>4</native_steps>",
        "<native_steps/>",
        "<bad/>",
        "junk",
    ] {
        assert!(
            try_load(&xml.replace(tag, &format!("<touching_solver>{bad}</touching_solver>")))
                .is_err()
        );
    }
    assert!(try_load(&xml.replace(tag, &format!("{tag}{tag}"))).is_err());
    assert!(try_load(&xml.replace("<incoming_face>+z</incoming_face>", "")).is_err());
    assert!(
        try_load(
            &xml.lines()
                .filter(|l| !l.contains("<pair>"))
                .collect::<String>()
                .replace(
                    "<native_steps>100</native_steps>",
                    &format!("{tag}<native_steps>100</native_steps>")
                )
        )
        .is_err()
    );
}

#[test]
fn separated_face_budget_preserves_public_time_reset_and_fallbacks() {
    use gz_sim_scene::primary_contact::set_refined_observer;
    use std::{
        sync::{Arc, Mutex},
        time::Duration,
    };
    // Drop 4 mm at 4 m/s: the crossing is at 1 ms on the original 0.2 ms grid.
    let raw = include_str!("../../../examples/worlds/local_sorter_crossing.sdf");
    let tag = "<early_face_step_ns>10000</early_face_step_ns><early_face_max_steps>200</early_face_max_steps>";
    let world = raw
        .replacen("<pair>", &format!("{tag}<pair>"), 1)
        .replace("-0.02 0 0.079", "0 0 0.043")
        .replace("2 0 -4</linear_velocity>", "0 0 -4</linear_velocity>");
    let mut sim = load(&world);
    let control = first(&mut sim);
    let [fine, rest] = control.early_face_schedule.unwrap();
    assert_eq!(fine.duration, Duration::from_micros(1200));
    assert_eq!((fine.native_steps, rest.native_steps), (120, 94));
    assert!(control.requested_pairs[0].normal_impulse > 0.);
    sim.reset().unwrap();
    let samples = Arc::new(Mutex::new(Vec::new()));
    let output = samples.clone();
    set_refined_observer(&mut sim, move |clock, cut| {
        output
            .lock()
            .unwrap()
            .push((clock.iteration, cut.duration, cut.elapsed));
    })
    .unwrap();
    let repeat = first(&mut sim);
    assert_eq!(control.early_face_schedule, repeat.early_face_schedule);
    assert_eq!(control.interval, repeat.interval);
    let rows = samples.lock().unwrap();
    assert_eq!(rows.len(), 214);
    assert!(
        rows[..120]
            .iter()
            .all(|s| s.0 == repeat.iteration && s.1 == Duration::from_micros(10))
    );
    assert!(
        rows[120..]
            .iter()
            .all(|s| s.1 == Duration::from_micros(200))
    );
    assert_eq!(rows.last().unwrap().2, sim.step_duration());

    for xml in [
        world.replace("200</early_face_max", "119</early_face_max"),
        world.replace("10000</early_face_step", "200000</early_face_step"),
        world.replace("0 0 0.043", "0 0 0.1186"),
        world.replace("0 0 0.043", "0 0 0.039"),
        world.replace("0 0 -4</linear_velocity>", "0.01 0 -4</linear_velocity>"),
        world.replace("-0.02 0.11 0.079", "0 0.05 0.043"),
    ] {
        let before = first(&mut load(&xml.replace(tag, "")));
        let r = first(&mut load(&xml));
        assert_eq!(r.early_face_schedule, None);
        assert_eq!(r.interval, before.interval);
    }
    let mut active = load(
        &world
            .replace("0 0 0.043", "0.1 0 0.043")
            .replace("-0.02 0.11 0.079", "0 0.05 0.039"),
    );
    active.step_exact(1).unwrap();
    let parcel = id(&active, "parcel");
    let mut pose = gz_sim_scene::model_state(&active, parcel).unwrap();
    pose.translation = [0., 0., 0.043];
    pose.linear_velocity = [0., 0., -4.];
    gz_sim_scene::set_model_state(&mut active, parcel, pose).unwrap();
    active.step_exact(1).unwrap();
    let r = last_refinement(&active).unwrap();
    assert_eq!(r.sim_time, active.elapsed());
    assert_eq!(r.early_face_schedule, None);
}

#[test]
fn early_face_fields_require_positive_budgets_and_named_incoming_pairs() {
    let raw = include_str!("../../../examples/worlds/local_sorter_crossing.sdf");
    let step = "<early_face_step_ns>25</early_face_step_ns>";
    let cap = "<early_face_max_steps>4096</early_face_max_steps>";
    for (sdf, tags) in [
        (raw.to_owned(), step.to_owned()),
        (raw.to_owned(), cap.to_owned()),
        (raw.to_owned(), format!("{step}{step}{cap}")),
        (raw.to_owned(), format!("{step}{cap}{cap}")),
        (raw.to_owned(), format!("{}{cap}", step.replace("25", "0"))),
        (
            raw.to_owned(),
            format!("{step}{}", cap.replace("4096", "x")),
        ),
        (
            raw.replace("<incoming_face>+z</incoming_face>", ""),
            format!("{step}{cap}"),
        ),
    ] {
        let xml = sdf.replacen("<pair>", &format!("{tags}<pair>"), 1);
        assert!(
            load_world_with_sdf_step(
                &xml,
                "early.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn release_tail_authoring_requires_a_complete_positive_coarser_window() {
    let raw = include_str!("../../../examples/worlds/local_sorter_crossing.sdf");
    let fine = "<early_face_step_ns>25</early_face_step_ns><early_face_max_steps>4000</early_face_max_steps>";
    let tail = "<early_face_tail_step_ns>125</early_face_tail_step_ns>";
    let guard = "<early_face_guard_ns>2000</early_face_guard_ns>";
    for tags in [
        format!("{fine}{tail}"),
        format!("{fine}{guard}"),
        format!("{tail}{guard}"),
        format!("{fine}{tail}{tail}{guard}"),
        format!("{fine}{guard}{guard}{tail}"),
        format!("{fine}{guard}{}", tail.replace("125", "25")),
        format!("{fine}{guard}{}", tail.replace("125", "0")),
        format!("{fine}{tail}{}", guard.replace("2000", "x")),
    ] {
        let xml = raw.replacen("<pair>", &format!("{tags}<pair>"), 1);
        assert!(
            load_world_with_sdf_step(
                &xml,
                "tail.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn returning_pad_requires_the_resident_slider_and_rebinds_after_reset() {
    let mut sim = load(include_str!(
        "../../../examples/worlds/local_refined_rocking_pad_grasp.sdf"
    ));
    let jaw = id(&sim, "gripper::left_slide");
    let entity = entity_by_id(sim.world(), jaw).unwrap();
    let joint = sim
        .world()
        .get::<gz_sim_scene::JointHandle>(entity)
        .unwrap()
        .id();
    let child = sim
        .world()
        .resource::<PhysicsBackend>()
        .joint_child(joint)
        .unwrap();
    assert!(
        sim.world_mut()
            .resource_mut::<PhysicsBackend>()
            .remove_joint(joint)
    );
    sim.world_mut().despawn(entity);
    let resumed = sim.step_exact(1);
    assert!(resumed.is_ok(), "{:?}", sim.fault());
    assert!(last_refinement(&sim).is_none());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let backend = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
        .unwrap();
    assert_eq!(backend.pad_joint(child), Some(joint));
    let right = id(&sim, "gripper::right_slide");
    gz_sim_scene::set_joint_force(&mut sim, jaw, 3.).unwrap();
    gz_sim_scene::set_joint_force(&mut sim, right, 3.).unwrap();
    sim.step_exact(1000).unwrap();
    let parcel = id(&sim, "parcel::body");
    queue_link_world_wrench(&mut sim, parcel, [0.; 3], [0., 6., 0.]).unwrap();
    let selected = first(&mut sim);
    assert_eq!(selected.epoch, 1);
    assert!(
        selected
            .pad_impacts
            .iter()
            .all(|p| p.peak_normal_force_n > 100.)
    );
    assert_eq!(selected.pad_impacts.len(), 2);
}

#[test]
fn rocking_grid_requires_an_enrolled_face_and_a_valid_intermediate_budget() {
    let sdf = include_str!("../../../examples/worlds/local_rocking_pad_approach.sdf");
    let field = "<pad_rocking_steps>8</pad_rocking_steps>";
    for xml in [
        sdf.replace(field, "<pad_rocking_steps>x</pad_rocking_steps>"),
        sdf.replace(field, "<pad_rocking_steps>1</pad_rocking_steps>"),
        sdf.replace(field, "<pad_rocking_steps>200</pad_rocking_steps>"),
        sdf.replace(field, &field.repeat(2)),
        sdf.replace("returning_pad_face", "incoming_face"),
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml,
                "invalid.sdf",
                "default",
                &NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
    // Seven uneven native slices must still consume the exact 200 µs public clock.
    let mut sim = load(&sdf.replace(field, "<pad_rocking_steps>7</pad_rocking_steps>"));
    for name in ["gripper::left_slide", "gripper::right_slide"] {
        let jaw = id(&sim, name);
        gz_sim_scene::set_joint_force(&mut sim, jaw, 3.).unwrap();
    }
    sim.step_exact(1000).unwrap();
    let parcel = id(&sim, "parcel::body");
    queue_link_world_wrench(&mut sim, parcel, [0.; 3], [0., 6., 0.]).unwrap();
    let forced = first(&mut sim);
    assert_eq!(forced.interval.native_steps, 200);
    let mut saw_rocking = false;
    for _ in 0..5 {
        sim.step_exact(1).unwrap();
        if let Some(r) = last_refinement(&sim)
            && r.sim_time == sim.elapsed()
            && r.interval.native_steps == 7
        {
            assert_eq!(
                r.sim_time - r.start_time,
                std::time::Duration::from_micros(200)
            );
            saw_rocking = true;
        }
    }
    assert!(saw_rocking);
}
