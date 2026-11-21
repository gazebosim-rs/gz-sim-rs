// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Centered runtime collisions reuse four-node ownership and admitted visuals.
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{EntityId, Simulation, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, CollisionDensity, EntityKind, LocalPose, LogicalCameraSample, ModelState, Name,
    PendingResetRemoval, PhysicsEngine, VisualGeometry, WorldEditError, WorldPose,
    load_world_with_sdf_step, model_state, prepare_model_template, queue_link_world_velocity,
    queue_link_world_wrench, remove_model, reset_loaded_world, set_model_state,
    spawn_model_template,
};
use sdformat::NoResources;
use std::{collections::BTreeMap, sync::Arc, time::Duration};

const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial>";
const VISUAL: &str = "<visual name='view'><pose>0.25 -0.5 0.75 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual>";
const CONTROL: &str =
    "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/>";
fn geometry(shape: &str) -> String {
    format!("<{shape}><radius>0.5</radius><length>2</length></{shape}>")
}
fn document(shape: &str, fixed: bool, gravity: &str, extra: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='w'><gravity>{gravity}</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='source'><pose>-100 0 10 0 0 0</pose><static>{fixed}</static><link name='body'>{INERTIA}<collision name='solid'><density>37</density><geometry>{}</geometry></collision>{VISUAL}</link></model><model name='survivor'><static>true</static><pose>100 100 10 0 0 0</pose><link name='body'><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision>{VISUAL}</link></model>{extra}</world></sdf>",
        geometry(shape)
    )
}
fn load(text: &str) -> Simulation {
    load_world_with_sdf_step(
        text,
        "primitive-runtime",
        "w",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn entity(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .get::<EntityId>(entity(sim, name))
        .unwrap()
        .get()
}
fn body(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .get::<BodyHandle>(entity(sim, name))
        .unwrap()
        .id()
}
fn native(sim: &Simulation, handle: BodyId) -> BodyState {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(handle)
        .unwrap()
}
fn identities(sim: &Simulation) -> BTreeMap<u64, Entity> {
    sim.world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(|id| (id.get(), e.id())))
        .collect()
}
fn states(sim: &Simulation) -> BTreeMap<BodyId, BodyState> {
    sim.world()
        .iter_entities()
        .filter_map(|e| e.get::<BodyHandle>().map(|h| (h.id(), native(sim, h.id()))))
        .collect()
}
fn near(actual: f64, expected: f64) {
    assert!(
        actual.is_finite() && (actual - expected).abs() < 1e-11,
        "{actual} != {expected}"
    );
}
fn vector(actual: [f64; 3], expected: [f64; 3]) {
    for (a, b) in actual.into_iter().zip(expected) {
        near(a, b);
    }
}
fn ray(
    sim: &Simulation,
    origin: [f64; 3],
    direction: [f64; 3],
    max: f64,
    exclude: Option<BodyId>,
) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(origin, direction, max, exclude)
        .unwrap()
}
fn pose(translation: [f64; 3], rotation: [f64; 4]) -> WorldPose {
    WorldPose {
        translation,
        rotation,
    }
}
fn spawned_body(sim: &Simulation, model: u64) -> BodyId {
    sim.world()
        .get::<BodyHandle>(entity_by_id(sim.world(), model + 1).unwrap())
        .unwrap()
        .id()
}
fn extent(shape: &str) -> f64 {
    if shape == "cylinder" { 1. } else { 1.5 }
}

#[test]
fn centered_runtime_primitives_keep_shape_frames_aliases_and_four_ids() {
    for shape in ["cylinder", "capsule"] {
        for fixed in [false, true] {
            let text = document(shape, fixed, "0 0 0", "");
            let parsed = sdformat::load_str(&text, "primitive-runtime", &NoResources).unwrap();
            let mut sim = load(&text);
            let template =
                prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
            let source_visual = entity(&sim, "w::source::body::view");
            assert_eq!(
                template.source_visual_id(),
                id(&sim, "w::source::body::view")
            );
            let alias = sim
                .world()
                .get::<VisualGeometry>(source_visual)
                .unwrap()
                .0
                .clone();
            let old_ids = identities(&sim);
            let old_states = states(&sim);
            let clock = (sim.epoch(), sim.elapsed());
            let first = next_entity_id(sim.world()).unwrap();
            let requested = pose([10., 20., 30.], [0.5; 4]);
            let model = spawn_model_template(&mut sim, &template, requested).unwrap();
            assert_eq!(model, first);
            assert_eq!(next_entity_id(sim.world()).unwrap(), first + 4);
            for (offset, kind) in [
                EntityKind::Model,
                EntityKind::Link,
                EntityKind::Collision,
                EntityKind::Visual,
            ]
            .into_iter()
            .enumerate()
            {
                assert_eq!(
                    sim.world().get::<EntityKind>(
                        entity_by_id(sim.world(), first + offset as u64).unwrap()
                    ),
                    Some(&kind)
                );
            }
            for (id, e) in old_ids {
                assert_eq!(entity_by_id(sim.world(), id), Some(e));
            }
            for (h, s) in old_states {
                assert_eq!(native(&sim, h), s);
            }
            assert_eq!((sim.epoch(), sim.elapsed()), clock);
            let visual = entity_by_id(sim.world(), model + 3).unwrap();
            assert!(Arc::ptr_eq(
                &alias,
                &sim.world().get::<VisualGeometry>(visual).unwrap().0
            ));
            let visual_pose = sim.world().get::<WorldPose>(visual).unwrap();
            vector(visual_pose.translation, [10.75, 20.25, 29.5]);
            let collision = entity_by_id(sim.world(), model + 2).unwrap();
            assert_eq!(
                sim.world().get::<CollisionDensity>(collision).unwrap().0,
                37.
            );
            let local = sim.world().get::<LocalPose>(collision).unwrap().0;
            assert_eq!(
                (local.translation, local.rotation),
                ([0.; 3], [0., 0., 0., 1.])
            );
            let collision_pose = sim.world().get::<WorldPose>(collision).unwrap();
            assert_eq!(
                (collision_pose.translation, collision_pose.rotation),
                (requested.translation, requested.rotation)
            );
            assert_eq!(
                model_state(&sim, model).unwrap().translation,
                requested.translation
            );
            let handle = spawned_body(&sim, model);
            assert_eq!(
                sim.world().resource::<PhysicsBackend>().is_dynamic(handle),
                Some(!fixed)
            );
            // Only the runtime body is within five metres of these queries: the
            // source is at(-100,0,10), the survivor at(100,100,10). Q cycles Z->X.
            near(
                ray(&sim, [14., 20., 30.], [-1., 0., 0.], 5., None).unwrap(),
                4. - extent(shape),
            );
            near(
                ray(&sim, [10., 20.3, 34.], [0., 0., -1.], 5., None).unwrap(),
                3.6,
            );
            assert_eq!(
                ray(&sim, [10., 20., 30.], [1., 0., 0.], 0.125, None),
                Some(0.)
            );
            assert!(ray(&sim, [14., 20., 30.], [-1., 0., 0.], 5., Some(handle)).is_none());
            assert!(ray(&sim, [10., 20.501, 34.], [0., 0., -1.], 5., None).is_none());
            if fixed {
                let mut changed = model_state(&sim, model).unwrap();
                changed.translation[0] += 2.;
                assert_eq!(set_model_state(&mut sim, model, changed).unwrap(), changed);
                near(
                    ray(&sim, [16., 20., 30.], [-1., 0., 0.], 5., None).unwrap(),
                    4. - extent(shape),
                );
                let before = native(&sim, handle);
                let mut invalid = changed;
                invalid.linear_velocity[0] = 1.;
                assert!(matches!(
                    set_model_state(&mut sim, model, invalid),
                    Err(WorldEditError::Physics(
                        gz_physics::PhysicsError::InvalidValue
                    ))
                ));
                assert_eq!(native(&sim, handle), before);
                assert_eq!(model_state(&sim, model).unwrap(), changed);
                sim.reset().unwrap();
                assert_eq!(
                    model_state(&sim, model).unwrap().translation,
                    requested.translation
                );
                near(
                    ray(&sim, [14., 20., 30.], [-1., 0., 0.], 5., None).unwrap(),
                    4. - extent(shape),
                );
            }
        }
    }
}

#[test]
fn runtime_primitives_keep_binary_bounds_and_global_nearest_exclusion() {
    for shape in ["cylinder", "capsule"] {
        let text = document(shape, false, "0 0 0", "");
        let parsed = sdformat::load_str(&text, "bounds", &NoResources).unwrap();
        let mut sim = load(&text);
        let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
        let model =
            spawn_model_template(&mut sim, &template, pose([10., 20., 30.], [0., 0., 0., 1.]))
                .unwrap();
        let handle = spawned_body(&sim, model);
        near(
            ray(
                &sim,
                [10., 20., 31. + extent(shape)],
                [0., 0., -1.],
                5.,
                None,
            )
            .unwrap(),
            1.,
        );
        // X is unchanged by the centered cylinder's Y-to-Z axis correction.
        // This radial side endpoint therefore uses only binary coordinates.
        let origin = [11.5, 20., 30.];
        assert_eq!(ray(&sim, origin, [-1., 0., 0.], 1., None), Some(1.));
        assert!(ray(&sim, origin, [-1., 0., 0.], 0.999, None).is_none());
        assert_eq!(
            ray(&sim, [10., 20., 30.], [1., 0., 0.], 0.125, None),
            Some(0.)
        );
        assert!(ray(&sim, origin, [-1., 0., 0.], 1., Some(handle)).is_none());
        // This independent sphere has a .25 gap above even the capsule tip.
        let nearer = "<model name='nearer'><static>true</static><pose>10 20 32 0 0 0</pose><link name='body'><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";
        let text = document(shape, false, "0 0 0", nearer);
        let parsed = sdformat::load_str(&text, "nearest", &NoResources).unwrap();
        let mut sim = load(&text);
        let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
        let model =
            spawn_model_template(&mut sim, &template, pose([10., 20., 30.], [0., 0., 0., 1.]))
                .unwrap();
        let handle = spawned_body(&sim, model);
        let nearer = body(&sim, "w::nearer::body");
        near(
            ray(&sim, [10., 20., 34.], [0., 0., -1.], 5., None).unwrap(),
            1.75,
        );
        near(
            ray(&sim, [10., 20., 34.], [0., 0., -1.], 5., Some(nearer)).unwrap(),
            4. - extent(shape),
        );
        near(
            ray(&sim, [10., 20., 34.], [0., 0., -1.], 5., Some(handle)).unwrap(),
            1.75,
        );
    }
}

#[test]
fn runtime_primitive_explicit_inertia_and_pending_inputs_survive_invalid_requests() {
    for shape in ["cylinder", "capsule"] {
        let text = document(shape, false, "0 0 0", "");
        let parsed = sdformat::load_str(&text, "inertia", &NoResources).unwrap();
        let mut sim = load(&text);
        let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
        let model =
            spawn_model_template(&mut sim, &template, pose([10., 20., 30.], [0.5; 4])).unwrap();
        let handle = spawned_body(&sim, model);
        // Q cycles explicit diag(2,3,4) to world diag(4,2,3). One torque
        // axis at a time avoids inventing a coupled gyroscopic oracle.
        for (axis, torque, moment) in [(0, 8., 4.), (1, 4., 2.), (2, 12., 3.)] {
            sim.reset().unwrap();
            sim.set_paused(true);
            let before = native(&sim, handle);
            let old_ids = identities(&sim);
            let mut t = [0.; 3];
            t[axis] = torque;
            queue_link_world_wrench(&mut sim, model + 1, [4., -6., 8.], t).unwrap();
            assert!(
                queue_link_world_wrench(&mut sim, model + 1, [f64::NAN, 0., 0.], [0.; 3]).is_err()
            );
            assert!(
                spawn_model_template(
                    &mut sim,
                    &template,
                    pose([f64::MAX, 0., 0.], [0., 0., 0., 1.])
                )
                .is_err()
            );
            assert_eq!(identities(&sim), old_ids);
            assert_eq!(native(&sim, handle), before);
            sim.tick().unwrap();
            assert_eq!(native(&sim, handle), before);
            assert_eq!(sim.elapsed(), Duration::ZERO);
            sim.set_paused(false);
            sim.step_exact(1).unwrap();
            vector(native(&sim, handle).velocity, [0.002, -0.003, 0.004]);
            let mut expected = [0.; 3];
            expected[axis] = torque / moment * 0.001;
            vector(native(&sim, handle).angular_velocity, expected);
        }
    }
}

#[test]
fn runtime_primitive_contacts_use_existing_geometry_and_separated_countercontrols() {
    let ball = "<model name='ball'><pose>10 0 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";
    for shape in ["cylinder", "capsule"] {
        for support in [false, true] {
            let text = document(shape, true, "0 0 -9.8", ball);
            let parsed = sdformat::load_str(&text, "contact", &NoResources).unwrap();
            let mut sim = load(&text);
            let template =
                prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
            if support {
                spawn_model_template(&mut sim, &template, pose([10., 0., 1.], [0., 0., 0., 1.]))
                    .unwrap();
            }
            // Retained contact-settling window, selected before verification. The
            // original source and survivor are far outside the ball's vertical line.
            sim.step_exact(1500).unwrap();
            let height = native(&sim, body(&sim, "w::ball::body")).position[2];
            if support {
                assert!(
                    (height - (1. + extent(shape) + 0.25)).abs() < 0.01,
                    "{shape}: {height}"
                );
            } else {
                assert!(height < 0., "separated {shape}: {height}");
            }
        }
    }
}

#[test]
fn runtime_primitive_edit_reset_removal_and_loaded_reset_keep_owned_ids() {
    for shape in ["cylinder", "capsule"] {
        let text = document(shape, false, "0 0 0", "");
        let parsed = sdformat::load_str(&text, "lifecycle", &NoResources).unwrap();
        let mut sim = load(&text);
        let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
        let original_ids = identities(&sim);
        let original_states = states(&sim);
        let origin = pose([10., 20., 30.], [0.5; 4]);
        let model = spawn_model_template(&mut sim, &template, origin).unwrap();
        let handle = spawned_body(&sim, model);
        let requested = ModelState {
            translation: [12., 20., 30.],
            rotation: [0.5; 4],
            linear_velocity: [0.125, -0.25, 0.5],
            angular_velocity: [0.25, 0., 0.],
        };
        assert_eq!(
            set_model_state(&mut sim, model, requested).unwrap(),
            requested
        );
        assert_eq!(model_state(&sim, model).unwrap(), requested);
        near(
            ray(&sim, [12., 20.3, 34.], [0., 0., -1.], 5., None).unwrap(),
            3.6,
        );
        sim.step_exact(2).unwrap();
        assert_ne!(
            model_state(&sim, model).unwrap().translation,
            requested.translation
        );
        sim.reset().unwrap();
        let reset = model_state(&sim, model).unwrap();
        assert_eq!(reset.translation, origin.translation);
        assert_eq!(reset.rotation, origin.rotation);
        assert_eq!(reset.linear_velocity, [0.; 3]);
        assert_eq!(spawned_body(&sim, model), handle);
        near(
            ray(&sim, [14., 20., 30.], [-1., 0., 0.], 5., None).unwrap(),
            4. - extent(shape),
        );
        assert_eq!(
            remove_model(&mut sim, model).unwrap(),
            (model..model + 4).collect::<Vec<_>>()
        );
        assert!(
            sim.world()
                .resource::<PhysicsBackend>()
                .state(handle)
                .is_none()
        );
        assert!(model_state(&sim, model).is_err());
        assert!(ray(&sim, [14., 20., 30.], [-1., 0., 0.], 5., None).is_none());
        sim.reset().unwrap();
        assert!(model_state(&sim, model).is_err());
        let high = next_entity_id(sim.world()).unwrap();
        assert!(high > model + 3);
        let next = spawn_model_template(&mut sim, &template, origin).unwrap();
        assert_eq!(next, high);
        let high = next_entity_id(sim.world()).unwrap();
        reset_loaded_world(&mut sim, |restored| {
            for offset in 0..4 {
                let e = entity_by_id(restored.world(), next + offset).unwrap();
                assert!(restored.world().get::<PendingResetRemoval>(e).is_some());
                assert!(restored.world().get::<BodyHandle>(e).is_none());
            }
        })
        .unwrap();
        assert_eq!(next_entity_id(sim.world()).unwrap(), high);
        assert!(model_state(&sim, next).is_err());
        for id in original_ids.keys() {
            assert!(entity_by_id(sim.world(), *id).is_some());
        }
        assert_eq!(states(&sim), original_states);
        // Prepared aliases stay usable after reconstruction in the same World.
        assert_eq!(
            spawn_model_template(&mut sim, &template, origin).unwrap(),
            high
        );
    }
}

#[test]
fn runtime_primitive_template_refusals_and_foreign_worlds_stay_atomic() {
    for shape in ["cylinder", "capsule"] {
        let base = document(shape, false, "0 0 0", "");
        let parsed = sdformat::load_str(&base, "refusals", &NoResources).unwrap();
        let mut sim = load(&base);
        let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
        let old_ids = identities(&sim);
        let old_states = states(&sim);
        let first = next_entity_id(sim.world()).unwrap();
        let clock = (sim.epoch(), sim.elapsed());
        for invalid in [
            pose([f64::NAN, 0., 0.], [0., 0., 0., 1.]),
            pose([0.; 3], [0.; 4]),
            pose([f64::MAX, 0., 0.], [0., 0., 0., 1.]),
        ] {
            assert!(spawn_model_template(&mut sim, &template, invalid).is_err());
            assert_eq!(identities(&sim), old_ids);
            assert_eq!(states(&sim), old_states);
            assert_eq!(next_entity_id(sim.world()).unwrap(), first);
            assert_eq!((sim.epoch(), sim.elapsed()), clock);
        }
        let mut foreign = load(&base);
        let foreign_ids = identities(&foreign);
        let foreign_states = states(&foreign);
        let foreign_first = next_entity_id(foreign.world()).unwrap();
        assert!(matches!(
            spawn_model_template(
                &mut foreign,
                &template,
                pose([10., 20., 30.], [0., 0., 0., 1.])
            ),
            Err(WorldEditError::InvalidScene)
        ));
        assert_eq!(identities(&foreign), foreign_ids);
        assert_eq!(states(&foreign), foreign_states);
        assert_eq!(next_entity_id(foreign.world()).unwrap(), foreign_first);
        let extra_collision = "<collision name='extra'><pose>3 0 0 0 0 0</pose><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>";
        let imu = "<sensor name='imu' type='imu'><always_on>true</always_on><update_rate>100</update_rate></sensor>";
        let hinge = "<joint name='hinge' type='revolute'><parent>world</parent><child>body</child><axis><xyz>0 0 1</xyz></axis></joint>";
        // Preserve the old extra-collision input as a positive compound witness.
        let compound = base.replacen(VISUAL, &format!("{extra_collision}{VISUAL}"), 1);
        assert_ne!(compound, base);
        let compound_sdf = sdformat::load_str(&compound, "compound", &NoResources).unwrap();
        let mut scene = load(&compound);
        let source_ids = identities(&scene);
        let source_states = states(&scene);
        let first = next_entity_id(scene.world()).unwrap();
        let compound_template =
            prepare_model_template(&scene, &compound_sdf, &compound_sdf.worlds[0].models[0])
                .unwrap();
        let clone = spawn_model_template(
            &mut scene,
            &compound_template,
            pose([0., 0., 30.], [0., 0., 0., 1.]),
        )
        .unwrap();
        assert_eq!(clone, first);
        assert_eq!(next_entity_id(scene.world()).unwrap(), first + 5);
        assert_eq!(identities(&scene).len(), source_ids.len() + 5);
        let handle = spawned_body(&scene, clone);
        // The original cylinder/capsule is centered at x=0; the sphere at x=3
        // alone intersects this segment. Loaded source/survivor are far away.
        near(
            ray(&scene, [3., 0., 32.], [0., 0., -1.], 2.1, None).unwrap(),
            1.5,
        );
        assert!(ray(&scene, [3., 0., 32.], [0., 0., -1.], 2.1, Some(handle)).is_none());
        remove_model(&mut scene, clone).unwrap();
        assert_eq!(identities(&scene), source_ids);
        assert_eq!(states(&scene), source_states);
        assert_eq!(scene.elapsed(), Duration::ZERO);
        for closed in [
            base.replacen("<collision name='solid'>","<collision name='solid'><pose>0.25 0 0 0 0 0</pose>",1),
            base.replacen("<collision name='solid'>","<collision name='solid'><pose>0 0 0 0 0 0.3</pose>",1),
            base.replacen(VISUAL,&format!("{VISUAL}{imu}"),1),
            base.replacen("<link name='body'>",&format!("{CONTROL}<link name='body'>"),1),
            base.replacen("</link></model>",&format!("</link>{hinge}</model>"),1),
            base.replacen("<visual name='view'><pose>0.25 -0.5 0.75 0 0 0</pose><geometry><box><size>1 1 1</size></box>","<visual name='view'><pose>0.25 -0.5 0.75 0 0 0</pose><geometry><cylinder><radius>0.5</radius><length>2</length></cylinder>",1),
        ] {
            assert_ne!(closed,base);let parsed=sdformat::load_str(&closed,"closed",&NoResources).unwrap();let scene=load(&closed);let ids=identities(&scene);let before_states=states(&scene);
            assert!(matches!(prepare_model_template(&scene,&parsed,&parsed.worlds[0].models[0]),Err(WorldEditError::UnsupportedModel)));
            assert_eq!(identities(&scene),ids);assert_eq!(states(&scene),before_states);assert_eq!(scene.elapsed(),Duration::ZERO);
        }
        // Unsupported nonzero COM remains a loader refusal, not new template admission.
        let com = base.replacen("<inertial>", "<inertial><pose>0.25 0 0 0 0 0</pose>", 1);
        assert!(
            load_world_with_sdf_step(&com, "closed-com", "w", &NoResources, PhysicsEngine::Rapier)
                .is_err()
        );
        let box_xml = base.replacen(&geometry(shape), "<box><size>1 1 1</size></box>", 1);
        let box_parsed = sdformat::load_str(&box_xml, "tpe", &NoResources).unwrap();
        let tpe = load_world_with_sdf_step(&box_xml, "tpe", "w", &NoResources, PhysicsEngine::Tpe)
            .unwrap();
        let tpe_ids = identities(&tpe);
        let tpe_next = next_entity_id(tpe.world()).unwrap();
        let tpe_clock = (tpe.epoch(), tpe.elapsed());
        // TPE owns model Motion, not Rapier BodyHandle state. Snapshot both models.
        let motions = |sim: &Simulation| {
            sim.world()
                .iter_entities()
                .filter_map(|entity| {
                    entity
                        .get::<gz_physics_tpe::Motion>()
                        .map(|motion| (entity.get::<EntityId>().unwrap().get(), *motion.state()))
                })
                .collect::<BTreeMap<_, _>>()
        };
        let tpe_motions = motions(&tpe);
        assert_eq!(tpe_motions.len(), 2);
        assert!(tpe_motions.contains_key(&id(&tpe, "w::source")));
        assert!(tpe_motions.contains_key(&id(&tpe, "w::survivor")));
        // The loaded TPE link has no BodyHandle, so binding refuses before backend validation.
        assert!(matches!(
            prepare_model_template(&tpe, &box_parsed, &box_parsed.worlds[0].models[0]),
            Err(WorldEditError::InvalidScene)
        ));
        assert_eq!(identities(&tpe), tpe_ids);
        assert_eq!(next_entity_id(tpe.world()).unwrap(), tpe_next);
        assert_eq!((tpe.epoch(), tpe.elapsed()), tpe_clock);
        assert_eq!(motions(&tpe), tpe_motions);
    }
}

#[test]
fn runtime_primitive_camera_capacity_refuses_without_consuming_ids_and_recovers() {
    let observer = "<model name='observer'><static>true</static><pose>-200 0 10 0 0 0</pose><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><sensor name='logical' type='logical_camera'><update_rate>0</update_rate><logical_camera><near>0.1</near><far>1000</far><horizontal_fov>1.5</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor></link></model>";
    for shape in ["cylinder", "capsule"] {
        let text = document(shape, false, "0 0 0", observer);
        let parsed = sdformat::load_str(&text, "capacity", &NoResources).unwrap();
        let mut sim = load(&text);
        let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
        let mut last = 0;
        // Three originals plus125 additions hit the real128-model demand bound.
        for index in 0..125 {
            last = spawn_model_template(
                &mut sim,
                &template,
                pose([10. + 4. * index as f64, 20., 30.], [0., 0., 0., 1.]),
            )
            .unwrap();
        }
        let source = id(&sim, "w::source::body");
        queue_link_world_velocity(&mut sim, source, [1., 0., 0.], [0.; 3]).unwrap();
        let old_ids = identities(&sim);
        let old_states = states(&sim);
        let first = next_entity_id(sim.world()).unwrap();
        let clock = (sim.epoch(), sim.elapsed());
        assert!(matches!(
            spawn_model_template(
                &mut sim,
                &template,
                pose([600., 20., 30.], [0., 0., 0., 1.])
            ),
            Err(WorldEditError::UnsupportedModel)
        ));
        assert_eq!(identities(&sim), old_ids);
        assert_eq!(states(&sim), old_states);
        assert_eq!(next_entity_id(sim.world()).unwrap(), first);
        assert_eq!((sim.epoch(), sim.elapsed()), clock);
        remove_model(&mut sim, last).unwrap();
        assert_eq!(
            spawn_model_template(
                &mut sim,
                &template,
                pose([600., 20., 30.], [0., 0., 0., 1.])
            )
            .unwrap(),
            first
        );
        sim.step_exact(1).unwrap();
        vector(
            native(&sim, body(&sim, "w::source::body")).velocity,
            [1., 0., 0.],
        );
        assert!(
            sim.world()
                .iter_entities()
                .find_map(|e| e.get::<LogicalCameraSample>())
                .unwrap()
                .reading
                .is_ok()
        );
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(
            sim.world()
                .iter_entities()
                .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Model))
                .count(),
            3
        );
    }
}
