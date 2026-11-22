// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Posed singleton clones retain the link mass frame and the four-node lifecycle.
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, BodyState, PhysicsBackend, PhysicsError};
use gz_sim_core::{EntityId, Simulation, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, CollisionDensity, EntityKind, LocalPose, ModelState, Name, PendingResetRemoval,
    PhysicsEngine, VisualGeometry, WorldEditError, WorldPose, load_world_with_sdf_step,
    model_state, prepare_model_template, queue_link_world_velocity, queue_link_world_wrench,
    remove_model, reset_loaded_world, set_model_state, set_velocity_control_command,
    spawn_model_template, velocity_control_binding_matches,
};
use sdformat::{Geometry, NoResources};
use std::{collections::BTreeMap, sync::Arc, time::Duration};

const SHAPES: [&str; 4] = [
    "<box><size>1.5 0.5 1</size></box>",
    "<sphere><radius>0.5</radius></sphere>",
    "<cylinder><radius>0.5</radius><length>2</length></cylinder>",
    "<capsule><radius>0.5</radius><length>2</length></capsule>",
];
const LOCAL: &str = "2 0 0 0 1.5707963267948966 0";
const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial>";
const VISUAL: &str = "<visual name='view'><pose>0.25 -0.5 0.75 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual>";
const CONTROL: &str =
    "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/>";

fn document(shape: &str, fixed: bool, local: &str, extra: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='source'><pose>-100 0 10 0 0 0</pose><static>{fixed}</static><link name='body'>{INERTIA}<collision name='solid'><pose>{local}</pose><density>37</density><geometry>{shape}</geometry></collision>{VISUAL}</link></model><model name='survivor'><pose>200 200 10 0 0 0</pose><static>true</static><link name='body'><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision>{VISUAL}</link></model>{extra}</world></sdf>"
    )
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "posed-runtime",
        "w",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn entity(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|v| v.0 == name))
        .unwrap()
        .id()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .get::<EntityId>(entity(sim, name))
        .unwrap()
        .get()
}
fn named_body(sim: &Simulation, name: &str) -> BodyId {
    sim.world()
        .get::<BodyHandle>(entity(sim, name))
        .unwrap()
        .id()
}
fn body(sim: &Simulation, model: u64) -> BodyId {
    sim.world()
        .get::<BodyHandle>(entity_by_id(sim.world(), model + 1).unwrap())
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
        .filter_map(|e| e.get::<EntityId>().map(|v| (v.get(), e.id())))
        .collect()
}
fn states(sim: &Simulation) -> BTreeMap<BodyId, BodyState> {
    sim.world()
        .iter_entities()
        .filter_map(|e| e.get::<BodyHandle>().map(|v| (v.id(), native(sim, v.id()))))
        .collect()
}
#[derive(Debug, PartialEq)]
struct Atomic {
    ids: BTreeMap<u64, Entity>,
    native: BTreeMap<BodyId, BodyState>,
    entities: usize,
    next: u64,
    clock: (u64, Duration),
}
fn snapshot(sim: &Simulation) -> Atomic {
    Atomic {
        ids: identities(sim),
        native: states(sim),
        entities: sim.world().iter_entities().count(),
        next: next_entity_id(sim.world()).unwrap(),
        clock: (sim.epoch(), sim.elapsed()),
    }
}
fn near(a: f64, b: f64) {
    assert!(a.is_finite() && (a - b).abs() < 1e-11, "{a} != {b}");
}
fn vector(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        near(a, b);
    }
}
fn pose(translation: [f64; 3], rotation: [f64; 4]) -> WorldPose {
    WorldPose {
        translation,
        rotation,
    }
}
fn cycle(translation: [f64; 3]) -> WorldPose {
    pose(translation, [0.5; 4])
}
fn ray(
    sim: &Simulation,
    origin: [f64; 3],
    direction: [f64; 3],
    maximum: f64,
    exclude: Option<BodyId>,
) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(origin, direction, maximum, exclude)
        .unwrap()
}

#[test]
fn posed_runtime_four_shapes_keep_noncommuting_frames_and_four_id_visual_aliases() {
    for (index, shape) in SHAPES.into_iter().enumerate() {
        for fixed in [false, true] {
            let xml = document(shape, fixed, LOCAL, "");
            let sdf = sdformat::load_str(&xml, "frames", &NoResources).unwrap();
            let mut sim = load(&xml);
            let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
            let original = snapshot(&sim);
            let alias = sim
                .world()
                .get::<VisualGeometry>(entity(&sim, "w::source::body::view"))
                .unwrap()
                .0
                .clone();
            assert_eq!(
                template.source_visual_id(),
                id(&sim, "w::source::body::view")
            );
            let model = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
            assert_eq!(model, original.next);
            assert_eq!(next_entity_id(sim.world()).unwrap(), model + 4);
            for (offset, kind) in [
                EntityKind::Model,
                EntityKind::Link,
                EntityKind::Collision,
                EntityKind::Visual,
            ]
            .into_iter()
            .enumerate()
            {
                let node = entity_by_id(sim.world(), model + offset as u64).unwrap();
                assert_eq!(sim.world().get::<EntityKind>(node), Some(&kind));
                if offset > 0 {
                    assert_eq!(
                        sim.world().get::<ChildOf>(node).unwrap().parent(),
                        entity_by_id(sim.world(), if offset == 1 { model } else { model + 1 })
                            .unwrap()
                    );
                }
            }
            let link = entity_by_id(sim.world(), model + 1).unwrap();
            let collision = entity_by_id(sim.world(), model + 2).unwrap();
            let visual = entity_by_id(sim.world(), model + 3).unwrap();
            assert!(sim.world().get::<BodyHandle>(collision).is_none());
            assert!(sim.world().get::<BodyHandle>(visual).is_none());
            assert!(Arc::ptr_eq(
                &alias,
                &sim.world().get::<VisualGeometry>(visual).unwrap().0
            ));
            assert_eq!(
                sim.world().get::<LocalPose>(link).unwrap().0.translation,
                [0.; 3]
            );
            assert_eq!(
                sim.world().get::<LocalPose>(link).unwrap().0.rotation,
                [0., 0., 0., 1.]
            );
            vector(native(&sim, body(&sim, model)).position, [10., 20., 30.]);
            vector(
                sim.world()
                    .get::<LocalPose>(collision)
                    .unwrap()
                    .0
                    .translation,
                [2., 0., 0.],
            );
            vector(
                sim.world().get::<WorldPose>(collision).unwrap().translation,
                [10., 22., 30.],
            );
            // q_cycle * q_Y90 sends child axes to (-X,+Z,+Y). The child
            // rotation does not rotate the local translation a second time.
            let q = std::f64::consts::FRAC_1_SQRT_2;
            for (a, b) in sim
                .world()
                .get::<WorldPose>(collision)
                .unwrap()
                .rotation
                .into_iter()
                .zip([0., q, q, 0.])
            {
                near(a, b);
            }
            vector(
                sim.world().get::<WorldPose>(visual).unwrap().translation,
                [10.75, 20.25, 29.5],
            );
            near(
                sim.world().get::<CollisionDensity>(collision).unwrap().0,
                37.,
            );
            let handle = body(&sim, model);
            assert_eq!(
                sim.world().resource::<PhysicsBackend>().is_dynamic(handle),
                Some(!fixed)
            );
            let radial = if index == 0 { 3.75 } else { 3.5 };
            let axial = match index {
                0 | 1 => 3.5,
                2 => 3.,
                _ => 2.5,
            };
            near(
                ray(&sim, [10., 22., 34.], [0., 0., -1.], 5., None).unwrap(),
                radial,
            );
            near(
                ray(&sim, [10., 26., 30.], [0., -1., 0.], 5., None).unwrap(),
                axial,
            );
            // The cylinder's exact 3-4-5 curved chord is also a sphere and
            // capsule-band oracle. Every other body is far outside this segment.
            near(
                ray(&sim, [10.3, 22., 34.], [0., 0., -1.], 5., None).unwrap(),
                if index == 0 { 3.75 } else { 3.6 },
            );
            assert_eq!(
                ray(&sim, [10.3, 22., 34.], [0., 0., -1.], 5., Some(handle)),
                None
            );
            assert_eq!(ray(&sim, [10., 20., 34.], [0., 0., -1.], 5., None), None);
            for (id, e) in original.ids {
                assert_eq!(entity_by_id(sim.world(), id), Some(e));
            }
            for (id, state) in original.native {
                assert_eq!(native(&sim, id), state);
            }
            assert_eq!((sim.epoch(), sim.elapsed()), original.clock);
        }
    }
}

#[test]
fn posed_runtime_binary_endpoints_and_all_body_nearest_queries_are_separate() {
    let xml = document(SHAPES[1], true, LOCAL, "");
    let mut sdf = sdformat::load_str(&xml, "binary", &NoResources).unwrap();
    let mut sim = load(&xml);
    // Only this caller DOM's sphere orientation is identity. The original
    // rotated template remains a separate wide-bound symmetry witness.
    let rotated = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let rotated_model = spawn_model_template(&mut sim, &rotated, cycle([40., 20., 10.])).unwrap();
    near(
        ray(&sim, [41.5, 22., 10.], [-1., 0., 0.], 5., None).unwrap(),
        1.,
    );
    let local = &mut sdf.worlds[0].models[0].links[0].collisions[0].pose;
    local.transform.rotation = [0., 0., 0., 1.];
    let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let first =
        spawn_model_template(&mut sim, &template, pose([10., 0., 10.], [0., 0., 0., 1.])).unwrap();
    let first_body = body(&sim, first);
    // Integer centers, binary radius and identity body/child rotations make
    // these endpoint preconditions exactly representable, unlike generic Qs.
    assert_eq!(
        ray(&sim, [13.5, 0., 10.], [-1., 0., 0.], 1., None),
        Some(1.)
    );
    assert_eq!(ray(&sim, [13.5, 0., 10.], [-1., 0., 0.], 0.999, None), None);
    assert_eq!(
        ray(&sim, [12., 0., 10.], [1., 0., 0.], 0.125, None),
        Some(0.)
    );
    assert_eq!(
        ray(&sim, [13.5, 0., 10.], [-1., 0., 0.], 1., Some(first_body)),
        None
    );
    // This ray traverses the sphere's AABB but misses its curved surface.
    assert_eq!(ray(&sim, [12.45, 0.45, 14.], [0., 0., -1.], 5., None), None);
    near(
        ray(&sim, [12.5, 4., 10.], [0., -1., 0.], 5., None).unwrap(),
        4.,
    );
    let second =
        spawn_model_template(&mut sim, &template, pose([15., 0., 10.], [0., 0., 0., 1.])).unwrap();
    near(
        ray(&sim, [20., 0., 10.], [-1., 0., 0.], 10., None).unwrap(),
        2.5,
    );
    near(
        ray(
            &sim,
            [20., 0., 10.],
            [-1., 0., 0.],
            10.,
            Some(body(&sim, second)),
        )
        .unwrap(),
        7.5,
    );
    near(
        ray(&sim, [20., 0., 10.], [-1., 0., 0.], 10., Some(first_body)).unwrap(),
        2.5,
    );
    assert!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(body(&sim, rotated_model))
            .is_some()
    );
    let before = snapshot(&sim);
    assert!(matches!(
        sim.world()
            .resource::<PhysicsBackend>()
            .cast_ray([12., 0., 10.], [1., 0., 0.], 0., None),
        Err(PhysicsError::InvalidValue)
    ));
    assert_eq!(snapshot(&sim), before);
}

#[test]
fn posed_runtime_six_axis_edits_and_explicit_inertia_ignore_collision_density() {
    for shape in SHAPES {
        let xml = document(shape, false, LOCAL, "");
        let mut low = load(&xml);
        let mut high = load(&xml.replace("<density>37</density>", "<density>9999</density>"));
        let low_sdf = sdformat::load_str(&xml, "low", &NoResources).unwrap();
        let high_sdf = sdformat::load_str(
            &xml.replace("<density>37</density>", "<density>9999</density>"),
            "high",
            &NoResources,
        )
        .unwrap();
        let a = prepare_model_template(&low, &low_sdf, &low_sdf.worlds[0].models[0]).unwrap();
        let b = prepare_model_template(&high, &high_sdf, &high_sdf.worlds[0].models[0]).unwrap();
        let ma = spawn_model_template(&mut low, &a, cycle([10., 20., 30.])).unwrap();
        let mb = spawn_model_template(&mut high, &b, cycle([10., 20., 30.])).unwrap();
        let target = ModelState {
            translation: [40., 50., 60.],
            rotation: [0.5; 4],
            linear_velocity: [0.125, -0.25, 0.5],
            angular_velocity: [0.25, -0.5, 0.75],
        };
        assert_eq!(model_state(&low, ma).unwrap().translation, [10., 20., 30.]);
        assert_eq!(set_model_state(&mut low, ma, target).unwrap(), target);
        assert_eq!(model_state(&low, ma).unwrap(), target);
        assert_eq!(
            native(&low, body(&low, ma)).velocity,
            target.linear_velocity
        );
        vector(
            low.world()
                .get::<WorldPose>(entity_by_id(low.world(), ma + 2).unwrap())
                .unwrap()
                .translation,
            [40., 52., 60.],
        );
        near(
            ray(&low, [40.3, 52., 64.], [0., 0., -1.], 5., None).unwrap(),
            if shape == SHAPES[0] { 3.75 } else { 3.6 },
        );
        assert_eq!(low.elapsed(), Duration::ZERO);
        let stationary = ModelState {
            translation: [10., 20., 30.],
            rotation: [0.5; 4],
            linear_velocity: [0.; 3],
            angular_velocity: [0.; 3],
        };
        set_model_state(&mut low, ma, stationary).unwrap();
        for (sim, model) in [(&mut low, ma), (&mut high, mb)] {
            queue_link_world_wrench(sim, model + 1, [2., -4., 6.], [6., 8., 4.]).unwrap();
            sim.set_paused(true);
            assert!(sim.tick().unwrap().dt.is_zero());
            assert_eq!(native(sim, body(sim, model)).velocity, [0.; 3]);
            sim.set_paused(false);
            sim.step_exact(1).unwrap();
            vector(
                native(sim, body(sim, model)).velocity,
                [0.001, -0.002, 0.003],
            );
            // A finite step under this nonprincipal torque includes gyroscopic
            // coupling. Keep the original input and require positive work;
            // complete native-state equality below proves density independence.
            let angular = native(sim, body(sim, model)).angular_velocity;
            assert!(angular.into_iter().all(f64::is_finite));
            assert!(
                angular
                    .into_iter()
                    .zip([6., 8., 4.])
                    .map(|(w, torque)| w * torque)
                    .sum::<f64>()
                    > 0.
            );
        }
        assert_eq!(native(&low, body(&low, ma)), native(&high, body(&high, mb)));
        // q_cycle maps local diagonal I=(2,3,4) to world (4,2,3).
        // Each torque is along one world principal axis: w cross Iw stays zero,
        // so these independent inertia oracles retain the strict 1e-11 bound.
        for (torque, expected) in [
            ([6., 0., 0.], [0.0015, 0., 0.]),
            ([0., 8., 0.], [0., 0.004, 0.]),
            ([0., 0., 4.], [0., 0., 4. / 3. * 0.001]),
        ] {
            for (sim, model) in [(&mut low, ma), (&mut high, mb)] {
                set_model_state(sim, model, stationary).unwrap();
                queue_link_world_wrench(sim, model + 1, [2., -4., 6.], torque).unwrap();
                sim.step_exact(1).unwrap();
                vector(
                    native(sim, body(sim, model)).velocity,
                    [0.001, -0.002, 0.003],
                );
                vector(native(sim, body(sim, model)).angular_velocity, expected);
            }
            assert_eq!(native(&low, body(&low, ma)), native(&high, body(&high, mb)));
        }
    }
    // Missing XML inertial fields are the parser's resolved centered defaults,
    // not a request to infer a shifted collider mass or a new auto-inertia pass.
    let xml = document(SHAPES[1], false, LOCAL, "").replacen(INERTIA, "", 1);
    let sdf = sdformat::load_str(&xml, "defaults", &NoResources).unwrap();
    let mut sim = load(&xml);
    assert_eq!(sdf.worlds[0].models[0].links[0].inertial.mass, 1.);
    assert_eq!(
        sdf.worlds[0].models[0].links[0].inertial.inertia,
        [1., 1., 1., 0., 0., 0.]
    );
    let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let model = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
    queue_link_world_wrench(&mut sim, model + 1, [1., 2., 3.], [2., 3., 4.]).unwrap();
    sim.step_exact(1).unwrap();
    vector(
        native(&sim, body(&sim, model)).velocity,
        [0.001, 0.002, 0.003],
    );
    vector(
        native(&sim, body(&sim, model)).angular_velocity,
        [0.002, 0.003, 0.004],
    );
}

#[test]
fn posed_runtime_source_retirement_both_resets_and_aliases_keep_high_water_ids() {
    for shape in SHAPES {
        for fixed in [false, true] {
            let xml = document(shape, fixed, LOCAL, "");
            let sdf = sdformat::load_str(&xml, "reset", &NoResources).unwrap();
            let mut sim = load(&xml);
            let original_ids = identities(&sim);
            let original_states = states(&sim);
            let source = id(&sim, "w::source");
            let survivor = named_body(&sim, "w::survivor::body");
            let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
            let origin = cycle([10., 20., 30.]);
            let model = spawn_model_template(&mut sim, &template, origin).unwrap();
            let handle = body(&sim, model);
            let target = ModelState {
                translation: [40., 50., 60.],
                rotation: [0.5; 4],
                linear_velocity: if fixed { [0.; 3] } else { [0.5, -0.25, 0.125] },
                angular_velocity: if fixed { [0.; 3] } else { [0.25, 0.5, -0.125] },
            };
            assert_eq!(set_model_state(&mut sim, model, target).unwrap(), target);
            sim.step_exact(2).unwrap();
            sim.reset().unwrap();
            let reset = model_state(&sim, model).unwrap();
            assert_eq!(reset.translation, origin.translation);
            assert_eq!(reset.rotation, origin.rotation);
            assert_eq!(reset.linear_velocity, [0.; 3]);
            assert_eq!(reset.angular_velocity, [0.; 3]);
            near(
                ray(&sim, [10.3, 22., 34.], [0., 0., -1.], 5., None).unwrap(),
                if shape == SHAPES[0] { 3.75 } else { 3.6 },
            );
            assert_eq!(body(&sim, model), handle);
            remove_model(&mut sim, source).unwrap();
            assert!(
                sim.world()
                    .resource::<PhysicsBackend>()
                    .state(survivor)
                    .is_some()
            );
            let alias = sim
                .world()
                .get::<VisualGeometry>(entity_by_id(sim.world(), model + 3).unwrap())
                .unwrap()
                .0
                .clone();
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
            assert_eq!(ray(&sim, [10.3, 22., 34.], [0., 0., -1.], 5., None), None);
            let high = next_entity_id(sim.world()).unwrap();
            assert!(high > model + 3);
            let next = spawn_model_template(&mut sim, &template, origin).unwrap();
            assert_eq!(next, high);
            assert_eq!(body(&sim, next).raw(), handle.raw() + 1);
            let next_body = body(&sim, next);
            assert!(Arc::ptr_eq(
                &alias,
                &sim.world()
                    .get::<VisualGeometry>(entity_by_id(sim.world(), next + 3).unwrap())
                    .unwrap()
                    .0
            ));
            let after = next_entity_id(sim.world()).unwrap();
            reset_loaded_world(&mut sim, |restored| {
                for offset in 0..4 {
                    let node = entity_by_id(restored.world(), next + offset).unwrap();
                    assert!(restored.world().get::<PendingResetRemoval>(node).is_some());
                    assert!(restored.world().get::<BodyHandle>(node).is_none());
                }
            })
            .unwrap();
            assert_eq!(next_entity_id(sim.world()).unwrap(), after);
            for key in original_ids.keys() {
                assert!(entity_by_id(sim.world(), *key).is_some());
            }
            assert_eq!(states(&sim), original_states);
            let rebuilt = spawn_model_template(&mut sim, &template, origin).unwrap();
            assert_eq!(rebuilt, after);
            assert_eq!(body(&sim, rebuilt).raw(), next_body.raw() + 1);
            assert!(Arc::ptr_eq(
                &alias,
                &sim.world()
                    .get::<VisualGeometry>(entity_by_id(sim.world(), rebuilt + 3).unwrap())
                    .unwrap()
                    .0
            ));
            near(
                ray(&sim, [10.3, 22., 34.], [0., 0., -1.], 5., None).unwrap(),
                if shape == SHAPES[0] { 3.75 } else { 3.6 },
            );
        }
    }
}

#[test]
fn posed_runtime_real_contacts_keep_link_origins_and_separated_controls() {
    let ball = "<model name='ball'><pose>10 0 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";
    for (index, shape) in SHAPES.into_iter().enumerate() {
        for support in [false, true] {
            let xml = document(shape, true, LOCAL, ball)
                .replace("<gravity>0 0 0</gravity>", "<gravity>0 0 -9.8</gravity>");
            let sdf = sdformat::load_str(&xml, "contact", &NoResources).unwrap();
            let mut sim = load(&xml);
            let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
            let origin = pose([if support { 8. } else { 20. }, 0., 1.], [0., 0., 0., 1.]);
            let model = spawn_model_template(&mut sim, &template, origin).unwrap();
            let extent = if index == 0 { 0.75 } else { 0.5 };
            assert!(4. - 0.25 > 1. + extent); // A genuine initial vertical gap.
            if support {
                near(
                    ray(&sim, [10., 0., 3.], [0., 0., -1.], 2., None).unwrap(),
                    2. - extent,
                );
            } else {
                assert_eq!(ray(&sim, [10., 0., 3.], [0., 0., -1.], 2., None), None);
            }
            // Same 1.5-second settling window as retained primitive/compound controls.
            sim.step_exact(1500).unwrap();
            let height = native(&sim, named_body(&sim, "w::ball::body")).position[2];
            if support {
                assert!(
                    (height - (1. + extent + 0.25)).abs() < 0.01,
                    "{index}: {height}"
                );
            } else {
                assert!(height < 0., "clear {index}: {height}");
            }
            assert_eq!(native(&sim, body(&sim, model)).position, origin.translation);
        }
    }
}

#[test]
fn posed_runtime_actual_passive_sources_preserve_identity_and_runtime_routes() {
    for shape in SHAPES {
        let xml = document(shape, false, LOCAL, "");
        let controlled = xml.replacen(
            "<link name='body'>",
            &format!("{CONTROL}<link name='body'>"),
            1,
        );
        let sdf = sdformat::load_str(&controlled, "controlled", &NoResources).unwrap();
        let mut sim = load(&controlled);
        let owner = id(&sim, "w::source");
        assert!(velocity_control_binding_matches(&sim, owner));
        let mut stripped = sdf.worlds[0].models[0].clone();
        stripped.plugins.clear();
        set_velocity_control_command(&mut sim, owner, [1., 2., 3.], [0.; 3]).unwrap();
        let before = snapshot(&sim);
        // The getter is admitted; the new posed branch then vetoes the ACTUAL
        // controller owner even though supplied DOM has no plugin.
        model_state(&sim, owner).unwrap();
        assert!(matches!(
            prepare_model_template(&sim, &sdf, &stripped),
            Err(WorldEditError::UnsupportedModel)
        ));
        assert_eq!(snapshot(&sim), before);
        assert!(velocity_control_binding_matches(&sim, owner));
        // Old identity caller-parameter semantics remain accepted for that same
        // actual controlled source. Do not move the new veto to the common path.
        stripped.links[0].collisions[0].pose.transform.translation = [0.; 3];
        stripped.links[0].collisions[0].pose.transform.rotation = [0., 0., 0., 1.];
        let identity = prepare_model_template(&sim, &sdf, &stripped).unwrap();
        let passive =
            spawn_model_template(&mut sim, &identity, pose([20., 0., 10.], [0., 0., 0., 1.]))
                .unwrap();
        assert!(!velocity_control_binding_matches(&sim, passive));
        sim.step_exact(2).unwrap();
        vector(
            native(&sim, named_body(&sim, "w::source::body")).velocity,
            [1., 2., 3.],
        );

        let sdf = sdformat::load_str(&xml, "passive", &NoResources).unwrap();
        let mut sim = load(&xml);
        let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
        let first = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
        // A real generated scope with a fresh resolving graph, not a renamed
        // model backed by the old source's graph, proves runtime-singleton reuse.
        let runtime_xml = format!(
            "<sdf version='1.12'><world name='w'><model name='runtime_{first}'><link name='body'>{INERTIA}<collision name='collision'><pose>{LOCAL}</pose><geometry>{shape}</geometry></collision><visual name='visual'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>"
        );
        let runtime = sdformat::load_str(&runtime_xml, "runtime-source", &NoResources).unwrap();
        let source = &runtime.worlds[0].models[0];
        let resolved = runtime
            .resolve_pose(
                &source.links[0].collisions[0].pose,
                &source.scoped_name,
                &source.links[0].scoped_name,
                &source.links[0].scoped_name,
            )
            .unwrap();
        assert_eq!(resolved.translation, [2., 0., 0.]);
        let second_template = prepare_model_template(&sim, &runtime, source).unwrap();
        assert_eq!(second_template.source_visual_id(), first + 3);
        let second =
            spawn_model_template(&mut sim, &second_template, cycle([40., 20., 30.])).unwrap();
        near(
            ray(&sim, [40.3, 22., 34.], [0., 0., -1.], 5., None).unwrap(),
            if shape == SHAPES[0] { 3.75 } else { 3.6 },
        );
        assert_ne!(body(&sim, first), body(&sim, second));
    }
    let extra = "<collision name='extra'><pose>0 4 0 0 0 0</pose><geometry><sphere><radius>0.25</radius></sphere></geometry></collision>";
    let xml =
        document(SHAPES[1], false, LOCAL, "").replacen(VISUAL, &format!("{extra}{VISUAL}"), 1);
    let sdf = sdformat::load_str(&xml, "compound-source", &NoResources).unwrap();
    let mut sim = load(&xml);
    let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let compound =
        spawn_model_template(&mut sim, &template, pose([80., 20., 30.], [0., 0., 0., 1.])).unwrap();
    model_state(&sim, compound).unwrap();
    let runtime_xml = format!(
        "<sdf version='1.12'><world name='w'><model name='runtime_{compound}'><link name='body'>{INERTIA}<collision name='collision'><pose>{LOCAL}</pose><geometry>{}</geometry></collision><visual name='visual'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>",
        SHAPES[1]
    );
    let runtime = sdformat::load_str(&runtime_xml, "compound-as-singleton", &NoResources).unwrap();
    let requested = &runtime.worlds[0].models[0];
    assert_eq!(
        runtime
            .resolve_pose(
                &requested.links[0].collisions[0].pose,
                &requested.scoped_name,
                &requested.links[0].scoped_name,
                &requested.links[0].scoped_name
            )
            .unwrap()
            .translation,
        [2., 0., 0.]
    );
    let before = snapshot(&sim);
    let singleton = prepare_model_template(&sim, &runtime, requested).unwrap();
    assert_eq!(singleton.source_visual_id(), compound + 3);
    assert_eq!(snapshot(&sim), before);
    let next = next_entity_id(sim.world()).unwrap();
    let copy = spawn_model_template(&mut sim, &singleton, cycle([400., 20., 30.])).unwrap();
    assert_eq!(copy, next);
    assert_eq!(next_entity_id(sim.world()).unwrap(), next + 4);
    assert_ne!(body(&sim, compound), body(&sim, copy));
    vector(native(&sim, body(&sim, copy)).position, [400., 20., 30.]);
    let collision = entity_by_id(sim.world(), copy + 2).unwrap();
    let local = sim.world().get::<LocalPose>(collision).unwrap().0;
    vector(local.translation, [2., 0., 0.]);
    vector(
        sim.world().get::<WorldPose>(collision).unwrap().translation,
        [400., 22., 30.],
    );
    assert_eq!(
        sim.world().get::<EntityKind>(collision),
        Some(&EntityKind::Collision)
    );
    assert_eq!(
        sim.world()
            .get::<Children>(entity_by_id(sim.world(), copy + 1).unwrap())
            .unwrap()
            .len(),
        2
    );
    assert!(Arc::ptr_eq(
        &sim.world()
            .get::<VisualGeometry>(entity_by_id(sim.world(), copy + 3).unwrap())
            .unwrap()
            .0,
        &sim.world()
            .get::<VisualGeometry>(entity_by_id(sim.world(), compound + 3).unwrap())
            .unwrap()
            .0,
    ));
    near(
        ray(&sim, [400.3, 22., 34.], [0., 0., -1.], 5., None).unwrap(),
        3.6,
    );
    assert_eq!(
        ray(
            &sim,
            [400.3, 22., 34.],
            [0., 0., -1.],
            5.,
            Some(body(&sim, copy))
        ),
        None
    );
}

#[test]
fn posed_runtime_parameter_and_first_guard_refusals_preserve_pending_inputs() {
    for shape in SHAPES {
        let xml = document(shape, false, LOCAL, "");
        let sdf = sdformat::load_str(&xml, "refusals", &NoResources).unwrap();
        let mut sim = load(&xml);
        let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
        let source_link = id(&sim, "w::source::body");
        queue_link_world_velocity(&mut sim, source_link, [1., 2., 3.], [0.; 3]).unwrap();
        let before = snapshot(&sim);
        let native_next = before.native.keys().map(|v| v.raw()).max().unwrap() + 1;
        for request in [
            pose([f64::NAN, 0., 0.], [0., 0., 0., 1.]),
            pose([0.; 3], [0.; 4]),
            pose([f64::MAX, 0., 0.], [0., 0., 0., 1.]),
        ] {
            assert!(matches!(
                spawn_model_template(&mut sim, &template, request),
                Err(WorldEditError::InvalidState)
            ));
            assert_eq!(snapshot(&sim), before);
        }
        let mut foreign = load(&xml);
        let foreign_before = snapshot(&foreign);
        assert!(matches!(
            spawn_model_template(&mut foreign, &template, cycle([10., 20., 30.])),
            Err(WorldEditError::InvalidScene)
        ));
        assert_eq!(snapshot(&foreign), foreign_before);
        // These fields are resolved caller-DOM parameters. Valid loaded state
        // reaches each precise guard before any native/public allocator commits.
        for row in 0..5 {
            let mut requested = sdf.worlds[0].models[0].clone();
            match row {
                0 => requested.links[0].collisions[0].density = Some(f64::NAN),
                1 => requested.links[0].collisions[0].pose.transform.translation = [1e100, 0., 0.],
                2 => requested.links[0].inertial.pose.transform.translation = [1e-13, 0., 0.],
                3 => requested.links[0].collisions[0].pose.relative_to = "missing_m92_frame".into(),
                _ => requested.links[0].collisions[0].pose.transform.rotation = [0.; 4],
            }
            let error = prepare_model_template(&sim, &sdf, &requested);
            match row {
                0 | 1 => assert!(matches!(error, Err(WorldEditError::InvalidState))),
                2 => assert!(error.is_ok(), "exact former tiny-COM input is admitted"),
                _ => assert!(matches!(error, Err(WorldEditError::InvalidScene))),
            }
            assert_eq!(snapshot(&sim), before);
        }
        for row in 0..3 {
            let mut requested = sdf.worlds[0].models[0].clone();
            match row {
                0 => requested.links[0].inertial.mass = f64::NAN,
                1 => requested.links[0].inertial.inertia[0] = -1.,
                _ => requested.links[0].collisions[0].geometry = Geometry::Sphere { radius: 1e200 },
            }
            let invalid = prepare_model_template(&sim, &sdf, &requested).unwrap();
            assert!(matches!(
                spawn_model_template(&mut sim, &invalid, cycle([10., 20., 30.])),
                Err(WorldEditError::Physics(PhysicsError::InvalidValue))
            ));
            assert_eq!(snapshot(&sim), before);
        }
        let accepted = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
        assert_eq!(accepted, before.next);
        assert_eq!(body(&sim, accepted).raw(), native_next);
        sim.step_exact(1).unwrap();
        vector(
            native(&sim, named_body(&sim, "w::source::body")).velocity,
            [1., 2., 3.],
        );

        // Both selected posed and mature identity routes preserve the same
        // tiny COM exactly; the velocity cross term must not be hidden by the
        // geometry helper's much wider 1e-11 tolerance.
        let mut identity = sdf.worlds[0].models[0].clone();
        identity.links[0].collisions[0].pose.transform.translation = [0.; 3];
        identity.links[0].collisions[0].pose.transform.rotation = [0., 0., 0., 1.];
        identity.links[0].inertial.pose.transform.translation = [1e-13, 0., 0.];
        let identity_template = prepare_model_template(&sim, &sdf, &identity).unwrap();
        let centered = spawn_model_template(
            &mut sim,
            &identity_template,
            pose([40., 0., 10.], [0., 0., 0., 1.]),
        )
        .unwrap();
        let target = ModelState {
            translation: [40., 0., 10.],
            rotation: [0., 0., 0., 1.],
            linear_velocity: [1., 0., 3.],
            angular_velocity: [0., 0., 2.],
        };
        assert_eq!(set_model_state(&mut sim, centered, target).unwrap(), target);
        let velocity = native(&sim, body(&sim, centered)).velocity;
        assert_eq!(velocity, [1., 2e-13, 3.]);
        assert_ne!(velocity, target.linear_velocity);

        // Spawn the exact former posed-COM refusal input, not an identity copy.
        let mut tiny_posed = sdf.worlds[0].models[0].clone();
        tiny_posed.links[0].inertial.pose.transform.translation = [1e-13, 0., 0.];
        let tiny_template = prepare_model_template(&sim, &sdf, &tiny_posed).unwrap();
        let posed_copy = spawn_model_template(
            &mut sim,
            &tiny_template,
            pose([80., 0., 10.], [0., 0., 0., 1.]),
        )
        .unwrap();
        let posed_target = ModelState {
            translation: [80., 0., 10.],
            ..target
        };
        assert_eq!(
            set_model_state(&mut sim, posed_copy, posed_target).unwrap(),
            posed_target
        );
        let posed_velocity = native(&sim, body(&sim, posed_copy)).velocity;
        assert_eq!(posed_velocity, [1., 2e-13, 3.]);
        assert_ne!(posed_velocity, posed_target.linear_velocity);

        // Valid caller changes copy mass/tensor/shape/translation, not original
        // loaded geometry. Visual resources remain the actual source Arc.
        let mut changed = sdf.worlds[0].models[0].clone();
        changed.links[0].inertial.mass = 4.;
        changed.links[0].inertial.inertia = [4., 6., 8., 0., 0., 0.];
        changed.links[0].collisions[0].geometry = Geometry::Sphere { radius: 0.75 };
        changed.links[0].collisions[0].pose.transform.translation = [3., 0., 0.];
        changed.links[0].collisions[0].density = Some(49.);
        let changed_template = prepare_model_template(&sim, &sdf, &changed).unwrap();
        let copied = spawn_model_template(
            &mut sim,
            &changed_template,
            pose([60., 0., 10.], [0., 0., 0., 1.]),
        )
        .unwrap();
        near(
            ray(&sim, [65., 0., 10.], [-1., 0., 0.], 5., None).unwrap(),
            1.25,
        );
        queue_link_world_wrench(&mut sim, copied + 1, [4., 0., 0.], [0., 6., 0.]).unwrap();
        sim.step_exact(1).unwrap();
        vector(native(&sim, body(&sim, copied)).velocity, [0.001, 0., 0.]);
        vector(
            native(&sim, body(&sim, copied)).angular_velocity,
            [0., 0.001, 0.],
        );
    }
    // TPE's actual first guard is the absent Rapier BodyHandle, before any
    // engine-specific constructor or supplied posed-parameter checks.
    let xml = document(SHAPES[0], false, "0 0 0 0 0 0", "");
    let mut sdf = sdformat::load_str(&xml, "tpe", &NoResources).unwrap();
    let tpe = load_world_with_sdf_step(&xml, "tpe", "w", &NoResources, PhysicsEngine::Tpe).unwrap();
    let motions = |sim: &Simulation| {
        sim.world()
            .iter_entities()
            .filter_map(|e| {
                e.get::<gz_physics_tpe::Motion>()
                    .map(|v| (e.get::<EntityId>().unwrap().get(), *v.state()))
            })
            .collect::<BTreeMap<_, _>>()
    };
    let old_motions = motions(&tpe);
    assert_eq!(old_motions.len(), 2);
    let ids = identities(&tpe);
    let next = next_entity_id(tpe.world()).unwrap();
    let clock = (tpe.epoch(), tpe.elapsed());
    sdf.worlds[0].models[0].links[0].collisions[0]
        .pose
        .transform
        .translation = [2., 0., 0.];
    assert!(matches!(
        prepare_model_template(&tpe, &sdf, &sdf.worlds[0].models[0]),
        Err(WorldEditError::InvalidScene)
    ));
    assert_eq!(motions(&tpe), old_motions);
    assert_eq!(identities(&tpe), ids);
    assert_eq!(next_entity_id(tpe.world()).unwrap(), next);
    assert_eq!((tpe.epoch(), tpe.elapsed()), clock);
    // The actual unsupported visual (not merely caller DOM metadata) refuses at
    // the existing world_edit whitelist; no Cylinder/Capsule visual is added.
    for geometry in [
        "<cylinder><radius>0.5</radius><length>1</length></cylinder>",
        "<capsule><radius>0.5</radius><length>1</length></capsule>",
    ] {
        let xml = document(SHAPES[1], false, LOCAL, "").replacen(
            VISUAL,
            &format!("<visual name='view'><geometry>{geometry}</geometry></visual>"),
            1,
        );
        let sdf = sdformat::load_str(&xml, "visual", &NoResources).unwrap();
        let sim = load(&xml);
        let before = snapshot(&sim);
        assert!(matches!(
            prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]),
            Err(WorldEditError::UnsupportedModel)
        ));
        assert_eq!(snapshot(&sim), before);
    }
}

#[test]
fn posed_runtime_camera_capacity_preserves_both_allocators_and_pending_velocity() {
    let observer = "<model name='observer'><static>true</static><pose>-200 0 10 0 0 0</pose><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><sensor name='logical' type='logical_camera'><update_rate>0</update_rate><logical_camera><near>0.1</near><far>1000</far><horizontal_fov>1.5</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor></link></model>";
    let xml = document(SHAPES[2], false, LOCAL, observer);
    let sdf = sdformat::load_str(&xml, "capacity", &NoResources).unwrap();
    let mut sim = load(&xml);
    let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let mut last = 0;
    // Three genuine originals plus125 additions reach camera capacity128.
    for index in 0..125 {
        last = spawn_model_template(
            &mut sim,
            &template,
            pose([10. + 4. * index as f64, 20., 30.], [0., 0., 0., 1.]),
        )
        .unwrap();
    }
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|e| e.get::<EntityKind>() == Some(&EntityKind::Model))
            .count(),
        128
    );
    assert_eq!(states(&sim).len(), 128);
    let last_body = body(&sim, last);
    let source_link = id(&sim, "w::source::body");
    queue_link_world_velocity(&mut sim, source_link, [1., 0., 0.], [0.; 3]).unwrap();
    let before = snapshot(&sim);
    assert!(matches!(
        spawn_model_template(
            &mut sim,
            &template,
            pose([600., 20., 30.], [0., 0., 0., 1.])
        ),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim), before);
    remove_model(&mut sim, last).unwrap();
    let recovered = spawn_model_template(
        &mut sim,
        &template,
        pose([600., 20., 30.], [0., 0., 0., 1.]),
    )
    .unwrap();
    assert_eq!(recovered, before.next);
    assert_eq!(body(&sim, recovered).raw(), last_body.raw() + 1);
    assert_eq!(next_entity_id(sim.world()).unwrap(), recovered + 4);
    sim.step_exact(1).unwrap();
    vector(
        native(&sim, named_body(&sim, "w::source::body")).velocity,
        [1., 0., 0.],
    );
}
