// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, DynamicBackend, PhysicsBackend};
use gz_physics_multifidelity::{
    ArticulationTier::{Dynamic, Prescribed},
    MultiFidelityBackend,
};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions,
    load_world_with_runtime, model_fidelity,
    primary_contact::last_refinement,
    queue_link_world_wrench, remove_model, reset_loaded_world, set_model_fidelity,
    support_loss_promotion::{self, SupportLossReason},
};
use std::time::Duration;
const SCENE: &str = include_str!("../../../examples/multifidelity_parked_shared_tier.sdf");
fn scene() -> Simulation {
    load_world_with_runtime(
        SCENE,
        "parked.sdf",
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
    )
    .unwrap()
}
fn id(s: &Simulation, name: &str) -> (u64, Option<BodyId>) {
    #[allow(deprecated)]
    let e = s
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|v| v.0 == format!("default::{name}"))
        })
        .unwrap();
    (
        e.get::<EntityId>().unwrap().get(),
        e.get::<BodyHandle>().map(BodyHandle::id),
    )
}
#[test]
fn measured_stationary_reactions_can_resume_and_each_removed_pad_wakes_before_solve() {
    for name in ["lift", "toe"] {
        let mut s = scene();
        let model = id(&s, "rocker").0;
        let foot = id(&s, "rocker::foot").1.unwrap();
        let pad = id(&s, &format!("{name}::pad")).1.unwrap();
        assert!(set_model_fidelity(&mut s, model, Prescribed).is_err());
        s.step_exact(1).unwrap();
        let reaction = last_refinement(&s)
            .unwrap()
            .requested_pairs
            .iter()
            .map(|p| p.normal_impulse)
            .collect::<Vec<_>>();
        assert_eq!(reaction.len(), 2);
        assert!(reaction.iter().all(|j| *j > 0.004));
        assert!((reaction.iter().sum::<f64>() - 0.00981).abs() < 1e-14);
        set_model_fidelity(&mut s, model, Prescribed).unwrap();
        s.step_exact(7).unwrap();
        set_model_fidelity(&mut s, model, Dynamic).unwrap();
        s.step_exact(1).unwrap();
        let next = last_refinement(&s).unwrap();
        assert_eq!(next.iteration, 9);
        for (previous, current) in reaction.iter().zip(&next.requested_pairs) {
            assert!((current.normal_impulse - previous).abs() < 1e-14);
        }
        set_model_fidelity(&mut s, model, Prescribed).unwrap();
        s.set_paused(true);
        let start = s.elapsed();
        let pad_model = id(&s, name).0;
        remove_model(&mut s, pad_model).unwrap();
        s.tick().unwrap();
        assert_eq!(s.elapsed(), start);
        assert_eq!(model_fidelity(&s, model).unwrap(), Prescribed);
        s.step_exact(1).unwrap();
        let wake = support_loss_promotion::last_promotion(&s, model).unwrap();
        assert_eq!(
            (wake.support, wake.link, wake.start_time),
            (pad, foot, start)
        );
        assert_eq!(wake.reason, SupportLossReason::Retired);
        assert_eq!(model_fidelity(&s, model).unwrap(), Dynamic);
    }
}
#[test]
fn solve_provenance_expires_on_motion_and_reset_and_pending_wrench_defers() {
    let mut s = scene();
    let model = id(&s, "rocker").0;
    let pad = id(&s, "toe::pad").1.unwrap();
    s.step_exact(2).unwrap();
    // Even a command restored before a solve must not resurrect the two-pad receipt.
    {
        let mut resource = s.world_mut().resource_mut::<PhysicsBackend>();
        let b = resource.downcast_mut::<MultiFidelityBackend>().unwrap();
        b.set_world_velocity(pad, [0.03, 0.02, 0.04], [0.; 3])
            .unwrap();
        b.set_world_velocity(pad, [0.; 3], [0.; 3]).unwrap();
    }
    assert!(set_model_fidelity(&mut s, model, Prescribed).is_err());
    s.step_exact(1).unwrap();
    set_model_fidelity(&mut s, model, Prescribed).unwrap();
    set_model_fidelity(&mut s, model, Dynamic).unwrap();
    s.step_exact(1).unwrap();
    let upper = id(&s, "rocker::upper").0;
    queue_link_world_wrench(&mut s, upper, [0., 0., 2.], [0.; 3]).unwrap();
    assert!(set_model_fidelity(&mut s, model, Prescribed).is_err());
    reset_loaded_world(&mut s, |_| ()).unwrap();
    assert!(last_refinement(&s).is_none());
    assert!(set_model_fidelity(&mut s, model, Prescribed).is_err());
    s.step_exact(1).unwrap();
    set_model_fidelity(&mut s, model, Prescribed).unwrap();
}
#[test]
fn eligibility_dwell_restarts_if_a_support_moves_on_the_admission_boundary() {
    use gz_sim_scene::{
        fidelity_zone::{self, FidelityZone},
        queue_link_world_velocity,
    };
    let mut s = scene();
    let model = id(&s, "rocker").0;
    fidelity_zone::configure_with_eligibility_dwell(
        &mut s,
        model,
        vec![FidelityZone {
            center: [100., 0., 0.],
            promote_radius: 0.1,
            demote_radius: 1.,
        }],
        Duration::ZERO,
        Duration::from_millis(2),
    )
    .unwrap();
    s.step_exact(5).unwrap();
    assert_eq!(model_fidelity(&s, model).unwrap(), Dynamic);
    let toe = id(&s, "toe::pad").0;
    queue_link_world_velocity(&mut s, toe, [0., 0., -0.75], [0.; 3]).unwrap();
    s.step_exact(1).unwrap();
    assert_eq!(model_fidelity(&s, model).unwrap(), Dynamic);
    assert!(support_loss_promotion::last_promotion(&s, model).is_none());
}

#[test]
fn each_changed_belt_retires_only_its_material_pressure_preference() {
    use glam::DVec3;
    for changed in [[true, false], [false, true], [true, true]] {
        let source = include_str!("../../../examples/multifidelity_uniform_shared_slide.sdf");
        let mut s = load_world_with_runtime(
            source,
            "belts.sdf",
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
        )
        .unwrap();
        s.step_exact(70).unwrap();
        let before = last_refinement(&s)
            .unwrap()
            .interval
            .shared_faces
            .last()
            .unwrap()
            .selected_fraction_b;
        let foot = id(&s, "rocker::foot").1.unwrap();
        let shoe = DVec3::from(
            s.world()
                .resource::<PhysicsBackend>()
                .state(foot)
                .unwrap()
                .velocity,
        );
        for (k, name) in ["lift::pad", "toe::pad"].into_iter().enumerate() {
            if changed[k] {
                let (link, pad) = id(&s, name);
                let v = DVec3::from(
                    s.world()
                        .resource::<PhysicsBackend>()
                        .state(pad.unwrap())
                        .unwrap()
                        .velocity,
                );
                let command = v - (shoe - v).normalize() * 0.007;
                gz_sim_scene::queue_link_world_velocity(&mut s, link, command.to_array(), [0.; 3])
                    .unwrap();
            }
        }
        s.step_exact(1).unwrap();
        let r = last_refinement(&s).unwrap();
        assert_eq!(r.iteration, 71);
        let shared = r.interval.shared_faces.last().unwrap();
        let [lo, hi] = shared.feasible_constant_fraction_b;
        let midpoint = 0.5 * lo + 0.5 * hi;
        let old = before.clamp(lo, hi);
        assert!((old - midpoint).abs() > 1e-4);
        let expected = if changed == [true, true] {
            midpoint
        } else {
            old
        };
        assert!((shared.selected_fraction_b - expected).abs() < 1e-14);
    }
}

const BATCH: &str = include_str!("../../../examples/multifidelity_shared_batch.sdf");
fn batched(source: &str) -> Simulation {
    gz_sim_scene::load_world_with_sdf_step(
        source,
        "batch.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}
#[test]
fn independent_trees_share_one_native_step_and_a_parked_neighbor_keeps_its_own_tier() {
    let mut s = batched(BATCH);
    s.step_exact(260).unwrap();
    let root = id(&s, "rocker_2").0;
    let foot = id(&s, "rocker_2::foot").1.unwrap();
    let r = &last_refinement(&s).unwrap().interval;
    assert_eq!((r.native_steps, r.shared_faces.len()), (1, 4));
    set_model_fidelity(&mut s, root, Prescribed).unwrap();
    s.step_exact(1).unwrap();
    let r = &last_refinement(&s).unwrap().interval;
    assert_eq!((r.native_steps, r.shared_faces.len()), (1, 3));
    assert!(r.shared_faces.iter().all(|g| g.foot != foot));
    assert_eq!(model_fidelity(&s, root).unwrap(), Prescribed);
    set_model_fidelity(&mut s, root, Dynamic).unwrap();
    s.step_exact(1).unwrap();
    let after = &last_refinement(&s).unwrap().interval;
    assert_eq!(after.shared_faces.len(), 4);
    let expected = after
        .shared_faces
        .iter()
        .find(|s| s.foot == foot)
        .unwrap()
        .loads;
    reset_loaded_world(&mut s, |_| ()).unwrap();
    s.step_exact(262).unwrap();
    let uninterrupted = last_refinement(&s)
        .unwrap()
        .interval
        .shared_faces
        .iter()
        .find(|s| s.foot == foot)
        .unwrap();
    for (x, y) in expected.iter().zip(uninterrupted.loads) {
        assert!((x.selected_normal_impulse - y.selected_normal_impulse).abs() < 1e-14);
    }
}
#[test]
fn batched_clearance_covers_fast_external_sweeps_and_unknown_dynamic_neighbors() {
    let body = |name: &str, pose: &str, kind: &str| {
        format!(
            r#"<model name="{name}"><pose>{pose} 0 0 0</pose>{kind}<link name="body"><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name="c"><geometry><box><size>0.08 0.08 0.08</size></box></geometry></collision></link></model>"#
        )
    };
    let drive = r#"<plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>-360 0 0</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>"#;
    for (extra, accept) in [
        (body("fast", "1.775 -1.902 0.63", drive), false),
        (body("unbounded", "99 -2 0.63", ""), false),
        (body("wall", "99 -2 0.63", "<static>true</static>"), true),
    ] {
        let src = BATCH.replace("</world>", &(extra + "</world>"));
        let mut s = batched(&src);
        let mut raw = batched(&src);
        gz_sim_scene::primary_contact::set_enabled(&mut raw, false).unwrap();
        s.step_exact(1).unwrap();
        raw.step_exact(1).unwrap();
        let count = last_refinement(&s).map_or(0, |r| r.interval.shared_faces.len());
        assert_eq!(count, if accept { 4 } else { 0 });
        if !accept {
            for i in 0..4 {
                let get = |s: &Simulation| {
                    s.world()
                        .resource::<PhysicsBackend>()
                        .state(id(s, &format!("rocker_{i}::foot")).1.unwrap())
                        .unwrap()
                };
                assert_eq!(get(&s), get(&raw));
            }
        }
    }
}

#[test]
fn batched_demotion_validates_every_requested_root_before_applying_any() {
    let mut s = batched(BATCH);
    s.step_exact(260).unwrap();
    let roots = [0, 1, 2, 3].map(|i| id(&s, &format!("rocker_{i}::foot")).1.unwrap());
    let mut backend = s.world_mut().resource_mut::<PhysicsBackend>();
    let engine = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
    let dt = Duration::from_micros(500);
    assert_eq!(engine.measured_parked_trees(), 4);
    assert!(
        engine
            .apply_parked_demotions(&[(roots[0], true), (roots[0], false)], dt)
            .is_err()
    );
    for root in roots {
        assert_eq!(engine.articulation_tier(root).unwrap(), Dynamic);
    }
    assert!(
        engine
            .apply_parked_demotions(
                &[(roots[2], false), (roots[1], true), (roots[0], false)],
                dt
            )
            .unwrap()
    );
    assert_eq!(engine.articulation_tier(roots[1]).unwrap(), Prescribed);
    assert!(
        !engine
            .apply_parked_demotions(&[(roots[0], true), (roots[1], true)], dt)
            .unwrap()
    );
    assert_eq!(engine.articulation_tier(roots[0]).unwrap(), Dynamic);
    assert!(
        engine
            .apply_parked_demotions(&[(roots[0], true), (roots[3], true)], dt)
            .unwrap()
    );
    assert_eq!(engine.articulation_tier(roots[2]).unwrap(), Dynamic);
}

#[test]
fn an_early_station_can_park_while_the_other_three_keep_sliding() {
    let mut s = batched(BATCH);
    s.step_exact(40).unwrap();
    let early = id(&s, "rocker_0").0;
    assert_eq!(
        last_refinement(&s)
            .unwrap()
            .interval
            .shared_faces
            .iter()
            .filter(|p| p.comoving_rest)
            .count(),
        1
    );
    set_model_fidelity(&mut s, early, Prescribed).unwrap();
    s.step_exact(1).unwrap();
    let load = &last_refinement(&s).unwrap().interval.shared_faces;
    assert_eq!(load.len(), 3);
    assert!(load.iter().all(|p| !p.comoving_rest));
    let moving = id(&s, "rocker_1").0;
    assert!(set_model_fidelity(&mut s, moving, Prescribed).is_err());
    assert_eq!(model_fidelity(&s, early).unwrap(), Prescribed);
}
