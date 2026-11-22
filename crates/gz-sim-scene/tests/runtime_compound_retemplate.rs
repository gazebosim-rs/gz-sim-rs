// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Owned runtime compounds retain their admission and lifetime when retemplated.
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, BodyState, PhysicsBackend, PhysicsError};
use gz_sim_core::{
    EntityId, Simulation, assign_id, entity_by_id, next_entity_id, reserve_id, restore_id,
};
use gz_sim_scene::{
    BodyHandle, CollisionDensity, EntityKind, LocalName, LocalPose, ModelState, Name,
    PendingResetRemoval, PhysicsEngine, SceneResetError, VisualGeometry, WorldEditError, WorldPose,
    load_world_with_sdf_step, model_state, prepare_model_template, queue_link_world_velocity,
    queue_link_world_wrench, remove_model, reset_loaded_world, set_model_state,
    spawn_model_template,
};
use sdformat::{Geometry, NoResources, Sdf};
use std::{collections::BTreeMap, sync::Arc, time::Duration};

const CONTROL: &str =
    "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/>";
const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial>";
fn model_xml(name: &str, count: usize, fixed: bool, visual: &str, density: f64) -> String {
    let collisions = (0..count).map(|i| {
        let geometry = match i % 4 {
            0 => "<sphere><radius>0.5</radius></sphere>",
            1 => "<cylinder><radius>0.5</radius><length>2</length></cylinder>",
            2 => "<box><size>1.5 0.5 1</size></box>",
            _ => "<capsule><radius>0.5</radius><length>2</length></capsule>",
        };
        format!("<collision name='c{i}'><pose>{} 0 0 0 1.5707963267948966 0</pose><density>{density}</density><geometry>{geometry}</geometry></collision>", 4 * i)
    }).collect::<String>();
    format!(
        "<model name='{name}'><static>{fixed}</static><link name='body'>{INERTIA}{collisions}<visual name='{visual}'><pose>0.25 -0.5 0.75 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual></link></model>"
    )
}
fn document(count: usize, fixed: bool, extra: &str) -> String {
    let source = model_xml("source", count, fixed, "view", 17.).replacen(
        "<static>",
        "<pose>-100 0 10 0 0 0</pose><static>",
        1,
    );
    format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics>{source}<model name='survivor'><pose>-300 0 10 0 0 0</pose><static>true</static><link name='body'><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision><visual name='view'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model>{extra}</world></sdf>"
    )
}
fn parsed(xml: &str) -> Sdf {
    sdformat::load_str(xml, "retemplate", &NoResources).unwrap()
}
fn requested(id: u64, count: usize, fixed: bool, density: f64) -> Sdf {
    parsed(&format!(
        "<sdf version='1.12'><world name='w'>{}</world></sdf>",
        model_xml(&format!("runtime_{id}"), count, fixed, "visual", density)
    ))
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(xml, "retemplate", "w", &NoResources, PhysicsEngine::Rapier).unwrap()
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
#[derive(Debug, PartialEq)]
struct Atomic {
    ids: BTreeMap<u64, Entity>,
    entities: usize,
    next: u64,
    clock: (u64, Duration),
    bodies: BTreeMap<u64, BodyState>,
}
fn snapshot(sim: &Simulation) -> Atomic {
    Atomic {
        ids: identities(sim),
        entities: sim.world().iter_entities().count(),
        next: next_entity_id(sim.world()).unwrap(),
        clock: (sim.epoch(), sim.elapsed()),
        bodies: sim
            .world()
            .iter_entities()
            .filter_map(|e| {
                e.get::<BodyHandle>()
                    .map(|v| (v.id().raw(), native(sim, v.id())))
            })
            .collect(),
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
fn child_id(model: u64, i: usize) -> u64 {
    model + if i == 0 { 2 } else { i as u64 + 3 }
}
fn geometry(sim: &Simulation, model: u64, count: usize, p: [f64; 3], density: f64) {
    let link = entity_by_id(sim.world(), model + 1).unwrap();
    assert_eq!(sim.world().get::<Children>(link).unwrap().len(), count + 1);
    assert_eq!(
        sim.world()
            .get::<EntityKind>(entity_by_id(sim.world(), model).unwrap()),
        Some(&EntityKind::Model)
    );
    assert_eq!(sim.world().get::<EntityKind>(link), Some(&EntityKind::Link));
    assert_eq!(
        sim.world().get::<ChildOf>(link).unwrap().parent(),
        entity_by_id(sim.world(), model).unwrap()
    );
    let handle = body(sim, model);
    assert_eq!(
        sim.world()
            .iter_entities()
            .filter(|e| e.get::<BodyHandle>().is_some_and(|v| v.id() == handle))
            .count(),
        1
    );
    let h = std::f64::consts::FRAC_1_SQRT_2;
    for i in 0..count {
        let child = entity_by_id(sim.world(), child_id(model, i)).unwrap();
        assert_eq!(
            sim.world().get::<EntityKind>(child),
            Some(&EntityKind::Collision)
        );
        assert_eq!(sim.world().get::<ChildOf>(child).unwrap().parent(), link);
        assert!(sim.world().get::<BodyHandle>(child).is_none());
        assert_eq!(
            sim.world().get::<LocalName>(child).unwrap().0,
            if i == 0 {
                "collision".into()
            } else {
                format!("collision_{i}")
            }
        );
        vector(
            sim.world().get::<LocalPose>(child).unwrap().0.translation,
            [4. * i as f64, 0., 0.],
        );
        near(
            sim.world().get::<CollisionDensity>(child).unwrap().0,
            density,
        );
        let frame = sim.world().get::<WorldPose>(child).unwrap();
        let center = [p[0], p[1] + 4. * i as f64, p[2]];
        vector(frame.translation, center);
        // q_cycle * q_Y90 maps local (X,Y,Z) to world (-X,+Z,+Y).
        for (actual, expected) in frame.rotation.into_iter().zip([0., h, h, 0.]) {
            near(actual, expected);
        }
        let origin = [center[0] + 0.3, center[1], center[2] + 4.];
        let expected = if i % 4 == 2 { 3.75 } else { 3.6 };
        near(ray(sim, origin, [0., 0., -1.], 5., None).unwrap(), expected);
        assert_eq!(ray(sim, origin, [0., 0., -1.], 5., Some(handle)), None);
        assert_eq!(ray(sim, center, [0., 0., 1.], 0.125, None), Some(0.));
    }
    // This crosses the aggregate's Z interior, halfway between disjoint Y lanes.
    assert_eq!(
        ray(sim, [p[0], p[1] + 2., p[2] + 4.], [0., 0., -1.], 5., None),
        None
    );
    let visual = entity_by_id(sim.world(), model + 3).unwrap();
    assert_eq!(
        sim.world().get::<EntityKind>(visual),
        Some(&EntityKind::Visual)
    );
    assert_eq!(sim.world().get::<ChildOf>(visual).unwrap().parent(), link);
    vector(
        sim.world().get::<WorldPose>(visual).unwrap().translation,
        [p[0] + 0.75, p[1] + 0.25, p[2] - 0.5],
    );
}

#[test]
fn runtime_compound_generations_resolve_real_names_and_preserve_complete_geometry() {
    for count in [2, 4, 16] {
        for fixed in [false, true] {
            let xml = document(count, fixed, "");
            let sdf = parsed(&xml);
            let mut sim = load(&xml);
            let source = id(&sim, "w::source");
            let source_state = model_state(&sim, source).unwrap();
            let source_native = native(&sim, body(&sim, source));
            let visual = sim
                .world()
                .get::<VisualGeometry>(entity(&sim, "w::source::body::view"))
                .unwrap()
                .0
                .clone();
            let initial = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
            let mut template = initial;
            for generation in 1..=3 {
                let p = [generation as f64 * 100., 20., 30.];
                let first = next_entity_id(sim.world()).unwrap();
                let before = snapshot(&sim);
                let model = spawn_model_template(&mut sim, &template, cycle(p)).unwrap();
                assert_eq!(model, first);
                assert_eq!(
                    next_entity_id(sim.world()).unwrap(),
                    first + count as u64 + 3
                );
                assert_eq!(snapshot(&sim).bodies.len(), before.bodies.len() + 1);
                assert_eq!(
                    body(&sim, model).raw(),
                    before.bodies.keys().copied().max().unwrap() + 1
                );
                for (id, state) in before.bodies {
                    assert_eq!(snapshot(&sim).bodies[&id], state);
                }
                let alias = entity_by_id(sim.world(), model + 3).unwrap();
                assert!(Arc::ptr_eq(
                    &visual,
                    &sim.world().get::<VisualGeometry>(alias).unwrap().0
                ));
                geometry(&sim, model, count, p, 17.);
                let supplied = requested(model, count, fixed, 17.);
                let requested_model = &supplied.worlds[0].models[0];
                assert_eq!(requested_model.scoped_name, format!("w::runtime_{model}"));
                for child in &requested_model.links[0].collisions {
                    supplied
                        .resolve_pose(
                            &child.pose,
                            &requested_model.scoped_name,
                            &requested_model.links[0].scoped_name,
                            &requested_model.links[0].scoped_name,
                        )
                        .unwrap();
                }
                let observed = snapshot(&sim);
                template = prepare_model_template(&sim, &supplied, requested_model).unwrap();
                assert_eq!(template.source_visual_id(), model + 3);
                assert_eq!(snapshot(&sim), observed);
            }
            assert_eq!(model_state(&sim, source).unwrap(), source_state);
            assert_eq!(native(&sim, body(&sim, source)), source_native);
        }
    }
}

#[test]
fn runtime_retemplate_copies_requested_parameters_and_keeps_binary_endpoints_separate() {
    let xml = document(2, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let first = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let source = spawn_model_template(&mut sim, &first, cycle([100., 20., 30.])).unwrap();
    let source_state = native(&sim, body(&sim, source));
    // Supplied parameters deliberately differ from the immutable two-child source.
    let supplied = requested(source, 16, true, 1234.);
    let copied = prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
    let second = spawn_model_template(&mut sim, &copied, cycle([200., 20., 30.])).unwrap();
    geometry(&sim, second, 16, [200., 20., 30.], 1234.);
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .is_dynamic(body(&sim, second)),
        Some(false)
    );
    assert_eq!(native(&sim, body(&sim, source)), source_state);
    let mut exact = requested(second, 2, false, 9.);
    // Identity Sphere0, binary center/radius/origin and axis isolate exact max=1.
    // The mixed noncommuting template above keeps all of its original rotations.
    exact.worlds[0].models[0].links[0].collisions[0]
        .pose
        .transform
        .rotation = [0., 0., 0., 1.];
    exact.worlds[0].models[0].links[0].collisions[1]
        .pose
        .transform
        .translation = [0., 8., 0.];
    let binary = prepare_model_template(&sim, &exact, &exact.worlds[0].models[0]).unwrap();
    let third =
        spawn_model_template(&mut sim, &binary, pose([400., 20., 30.], [0., 0., 0., 1.])).unwrap();
    let handle = body(&sim, third);
    assert_eq!(
        ray(&sim, [401.5, 20., 30.], [-1., 0., 0.], 1., None),
        Some(1.)
    );
    assert_eq!(
        ray(&sim, [401.5, 20., 30.], [-1., 0., 0.], 0.999, None),
        None
    );
    assert_eq!(
        ray(&sim, [400., 20., 30.], [1., 0., 0.], 0.125, None),
        Some(0.)
    );
    assert_eq!(
        ray(&sim, [401.5, 20., 30.], [-1., 0., 0.], 1., Some(handle)),
        None
    );
    // A separate body lies closer along the same axis; exclusion chooses the next owner.
    let closer =
        spawn_model_template(&mut sim, &binary, pose([402., 20., 30.], [0., 0., 0., 1.])).unwrap();
    near(
        ray(&sim, [404., 20., 30.], [-1., 0., 0.], 5., None).unwrap(),
        1.5,
    );
    near(
        ray(
            &sim,
            [404., 20., 30.],
            [-1., 0., 0.],
            5.,
            Some(body(&sim, closer)),
        )
        .unwrap(),
        3.5,
    );
}

#[test]
fn retemplated_explicit_inertia_and_six_axis_edits_ignore_child_density() {
    let xml = document(4, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let original = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let source = spawn_model_template(&mut sim, &original, cycle([100., 20., 30.])).unwrap();
    let source_before = native(&sim, body(&sim, source));
    let light = requested(source, 4, false, 1.);
    let dense = requested(source, 4, false, 1e6);
    let a = prepare_model_template(&sim, &light, &light.worlds[0].models[0]).unwrap();
    let b = prepare_model_template(&sim, &dense, &dense.worlds[0].models[0]).unwrap();
    let x = spawn_model_template(&mut sim, &a, cycle([200., 20., 30.])).unwrap();
    let y = spawn_model_template(&mut sim, &b, cycle([300., 20., 30.])).unwrap();
    let target = ModelState {
        translation: [400., 50., 60.],
        rotation: [0.5; 4],
        linear_velocity: [1., -2., 3.],
        angular_velocity: [-0.25, 0.5, 0.75],
    };
    assert_eq!(set_model_state(&mut sim, x, target).unwrap(), target);
    assert_eq!(model_state(&sim, x).unwrap(), target);
    geometry(&sim, x, 4, target.translation, 1.);
    let stopped = ModelState {
        linear_velocity: [0.; 3],
        angular_velocity: [0.; 3],
        ..target
    };
    let stationary = [
        stopped,
        ModelState {
            translation: [500., 50., 60.],
            ..stopped
        },
    ];
    for (axis, magnitude) in [6., 8., 4.].into_iter().enumerate() {
        for (model, state) in [x, y].into_iter().zip(stationary) {
            set_model_state(&mut sim, model, state).unwrap();
        }
        let mut torque = [0.; 3];
        torque[axis] = magnitude;
        for model in [x, y] {
            queue_link_world_wrench(&mut sim, model + 1, [2., -4., 6.], torque).unwrap();
        }
        sim.set_paused(true);
        assert!(sim.tick().unwrap().dt.is_zero());
        assert_eq!(model_state(&sim, x).unwrap(), stationary[0]);
        sim.set_paused(false);
        sim.step_exact(1).unwrap();
        let left = native(&sim, body(&sim, x));
        let right = native(&sim, body(&sim, y));
        vector(left.velocity, [0.001, -0.002, 0.003]);
        vector(left.velocity, right.velocity);
        let mut expected = [0.; 3];
        // q_cycle maps link I=(2,3,4) to world diagonal (4,2,3).
        expected[axis] = torque[axis] * 0.001 / [4., 2., 3.][axis];
        vector(left.angular_velocity, expected);
        vector(left.angular_velocity, right.angular_velocity);
        assert_eq!(left.velocity, right.velocity);
        assert_eq!(left.angular_velocity, right.angular_velocity);
        assert_eq!(left.orientation, right.orientation);
        vector(
            left.position,
            [
                right.position[0] - 100.,
                right.position[1],
                right.position[2],
            ],
        );
    }
    assert_eq!(native(&sim, body(&sim, source)), source_before);
}

#[test]
fn retemplate_lifetime_survives_source_retirement_and_both_reset_boundaries() {
    for fixed in [false, true] {
        let xml = document(4, fixed, "");
        let sdf = parsed(&xml);
        let mut sim = load(&xml);
        let loaded_ids = identities(&sim);
        let loaded_bodies = snapshot(&sim).bodies;
        let loaded_source = id(&sim, "w::source");
        let loaded = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
        let first = spawn_model_template(&mut sim, &loaded, cycle([100., 20., 30.])).unwrap();
        let supplied = requested(first, 4, fixed, 17.);
        let second_template =
            prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
        let second =
            spawn_model_template(&mut sim, &second_template, cycle([200., 20., 30.])).unwrap();
        let second_dom = requested(second, 4, fixed, 17.);
        let detached =
            prepare_model_template(&sim, &second_dom, &second_dom.worlds[0].models[0]).unwrap();
        let alias = sim
            .world()
            .get::<VisualGeometry>(entity_by_id(sim.world(), second + 3).unwrap())
            .unwrap()
            .0
            .clone();
        let original = model_state(&sim, second).unwrap();
        let second_body = body(&sim, second);
        let edited = ModelState {
            translation: [200., 40., 60.],
            rotation: [0., 0., 1., 0.],
            linear_velocity: [0.; 3],
            angular_velocity: [0.; 3],
        };
        set_model_state(&mut sim, second, edited).unwrap();
        sim.reset().unwrap();
        assert_eq!(model_state(&sim, second).unwrap(), original);
        assert_eq!(body(&sim, second), second_body);
        geometry(&sim, second, 4, [200., 20., 30.], 17.);
        assert_eq!(
            remove_model(&mut sim, first).unwrap(),
            (first..first + 7).collect::<Vec<_>>()
        );
        remove_model(&mut sim, loaded_source).unwrap();
        let next = next_entity_id(sim.world()).unwrap();
        let third = spawn_model_template(&mut sim, &detached, cycle([300., 20., 30.])).unwrap();
        assert_eq!(third, next);
        assert!(body(&sim, third).raw() > second_body.raw());
        assert!(Arc::ptr_eq(
            &alias,
            &sim.world()
                .get::<VisualGeometry>(entity_by_id(sim.world(), third + 3).unwrap())
                .unwrap()
                .0
        ));
        geometry(&sim, third, 4, [300., 20., 30.], 17.);
        let runtime = [second, third];
        let high = next_entity_id(sim.world()).unwrap();
        reset_loaded_world(&mut sim, |observed| {
            for model in runtime {
                for node in model..model + 7 {
                    let node = entity_by_id(observed.world(), node).unwrap();
                    assert!(observed.world().get::<PendingResetRemoval>(node).is_some());
                    assert!(observed.world().get::<BodyHandle>(node).is_none());
                    assert!(observed.world().get::<WorldPose>(node).is_some());
                }
            }
        })
        .unwrap();
        assert_eq!(next_entity_id(sim.world()).unwrap(), high);
        assert_eq!(
            identities(&sim).keys().copied().collect::<Vec<_>>(),
            loaded_ids.keys().copied().collect::<Vec<_>>()
        );
        for (raw, state) in loaded_bodies {
            assert_eq!(snapshot(&sim).bodies[&raw], state);
        }
        assert!(
            runtime
                .into_iter()
                .all(|model| entity_by_id(sim.world(), model).is_none())
        );
        let fourth = spawn_model_template(&mut sim, &detached, cycle([400., 20., 30.])).unwrap();
        assert_eq!(fourth, high);
        geometry(&sim, fourth, 4, [400., 20., 30.], 17.);
    }
}

#[test]
fn runtime_retemplate_closed_domains_and_late_errors_preserve_both_allocators() {
    let xml = document(2, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let initial = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let source = spawn_model_template(&mut sim, &initial, cycle([100., 20., 30.])).unwrap();
    let supplied = requested(source, 2, false, 17.);
    let valid = prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
    let before = snapshot(&sim);
    for count in [0, 17] {
        let other = requested(source, count, false, 17.);
        assert!(matches!(
            prepare_model_template(&sim, &other, &other.worlds[0].models[0]),
            Err(WorldEditError::UnsupportedModel)
        ));
        assert_eq!(snapshot(&sim), before);
    }
    let singleton = requested(source, 1, false, 17.);
    // The same formerly refused one-child DOM now prepares without touching
    // the source. Exercise its four-node publication in a separate world so
    // every later refusal still compares against the original allocator state.
    let single = prepare_model_template(&sim, &singleton, &singleton.worlds[0].models[0]).unwrap();
    assert_eq!(single.source_visual_id(), source + 3);
    assert_eq!(snapshot(&sim), before);
    let mut positive = load(&xml);
    let initial = prepare_model_template(&positive, &sdf, &sdf.worlds[0].models[0]).unwrap();
    assert_eq!(
        spawn_model_template(&mut positive, &initial, cycle([100., 20., 30.])).unwrap(),
        source
    );
    let single =
        prepare_model_template(&positive, &singleton, &singleton.worlds[0].models[0]).unwrap();
    let next = next_entity_id(positive.world()).unwrap();
    let copy = spawn_model_template(&mut positive, &single, cycle([600., 20., 30.])).unwrap();
    assert_eq!(copy, next);
    assert_eq!(next_entity_id(positive.world()).unwrap(), next + 4);
    assert_ne!(body(&positive, copy), body(&positive, source));
    geometry(&positive, copy, 1, [600., 20., 30.], 17.);
    let mut mesh = singleton.worlds[0].models[0].clone();
    let mesh_source = mesh.links[0].collisions[0].source.clone();
    mesh.links[0].collisions[0].geometry = Geometry::Mesh(sdformat::Mesh {
        uri: "must-not-be-decoded.glb".into(),
        scale: [1.; 3],
        submesh: String::new(),
        center_submesh: false,
        optimization: Default::default(),
        convex_decomposition: None,
        source: mesh_source,
    });
    assert!(matches!(
        prepare_model_template(&sim, &singleton, &mesh),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim), before);
    for invalid_pose in [
        pose([f64::NAN, 0., 0.], [0., 0., 0., 1.]),
        pose([200., 20., 30.], [0.; 4]),
    ] {
        assert!(matches!(
            spawn_model_template(&mut sim, &valid, invalid_pose),
            Err(WorldEditError::InvalidState)
        ));
        assert_eq!(snapshot(&sim), before);
    }
    for row in 0..7 {
        let mut model = supplied.worlds[0].models[0].clone();
        match row {
            0 => {
                model.links[0].collisions[1].geometry = Geometry::Plane {
                    normal: [0., 0., 1.],
                    size: [1., 1.],
                }
            }
            1 => model.links[0].collisions[1].pose.relative_to = "absent_last_child".into(),
            2 => model.links[0].collisions[1].density = Some(f64::NAN),
            3 => model.links[0].collisions[1].geometry = Geometry::Sphere { radius: -1. },
            4 => model.links[0].inertial.pose.transform.translation = [0.25, 0., 0.],
            5 => model.links[0].pose.transform.translation = [1., 0., 0.],
            _ => {
                model.plugins = parsed(&document(2, false, "").replacen(
                    "<link name='body'>",
                    &format!("{CONTROL}<link name='body'>"),
                    1,
                ))
                .worlds[0]
                    .models[0]
                    .plugins
                    .clone()
            }
        }
        let result = prepare_model_template(&sim, &supplied, &model);
        match row {
            1 => assert!(matches!(result, Err(WorldEditError::InvalidScene))),
            2 | 3 => assert!(matches!(result, Err(WorldEditError::InvalidState))),
            _ => assert!(matches!(result, Err(WorldEditError::UnsupportedModel))),
        }
        assert_eq!(snapshot(&sim), before);
    }
    for row in 0..3 {
        let mut model = supplied.worlds[0].models[0].clone();
        match row {
            0 => model.links[0].inertial.mass = f64::NAN,
            1 => model.links[0].inertial.inertia[0] = -1.,
            _ => model.links[0].collisions[1].geometry = Geometry::Sphere { radius: 1e200 },
        }
        let invalid = prepare_model_template(&sim, &supplied, &model).unwrap();
        assert!(matches!(
            spawn_model_template(&mut sim, &invalid, cycle([200., 20., 30.])),
            Err(WorldEditError::Physics(PhysicsError::InvalidValue))
        ));
        assert_eq!(snapshot(&sim), before);
    }
    let mut foreign = load(&xml);
    let foreign_before = snapshot(&foreign);
    assert!(matches!(
        spawn_model_template(&mut foreign, &valid, cycle([200., 20., 30.])),
        Err(WorldEditError::InvalidScene)
    ));
    assert_eq!(snapshot(&foreign), foreign_before);
    let model = spawn_model_template(&mut sim, &valid, cycle([200., 20., 30.])).unwrap();
    assert_eq!(model, before.next);
    assert_eq!(
        body(&sim, model).raw(),
        before.bodies.keys().copied().max().unwrap() + 1
    );
    let controlled_xml = xml.replacen(
        "<link name='body'>",
        &format!("{CONTROL}<link name='body'>"),
        1,
    );
    let owner_sdf = parsed(&controlled_xml);
    let owner = load(&controlled_xml);
    let mut stripped = owner_sdf.worlds[0].models[0].clone();
    stripped.plugins.clear();
    let observed = snapshot(&owner);
    assert!(model_state(&owner, id(&owner, "w::source")).is_ok());
    assert!(matches!(
        prepare_model_template(&owner, &owner_sdf, &stripped),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&owner), observed);
}

#[test]
fn runtime_source_membership_drift_cannot_downgrade_and_repair_preserves_pending_wrench() {
    for row in 0..8 {
        let xml = document(2, false, "");
        let sdf = parsed(&xml);
        let mut sim = load(&xml);
        let initial = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
        let model = spawn_model_template(&mut sim, &initial, cycle([100., 20., 30.])).unwrap();
        let supplied = requested(model, 2, false, 17.);
        prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
        let root = entity_by_id(sim.world(), model).unwrap();
        let link = entity_by_id(sim.world(), model + 1).unwrap();
        let child = entity_by_id(sim.world(), model + 4).unwrap();
        let local = *sim.world().get::<LocalPose>(child).unwrap();
        let name = sim.world().get::<LocalName>(child).unwrap().0.clone();
        let kind = *sim.world().get::<EntityKind>(child).unwrap();
        let reserved = reserve_id(sim.world(), child).unwrap();
        queue_link_world_wrench(&mut sim, model + 1, [4., 0., 0.], [0.; 3]).unwrap();
        let mut foreign_handle = None;
        let mut extra = None;
        match row {
            0 => {
                sim.world_mut().entity_mut(child).insert(ChildOf(root));
            }
            1 => {
                sim.world_mut()
                    .entity_mut(child)
                    .get_mut::<LocalPose>()
                    .unwrap()
                    .0
                    .translation[0] += 0.25;
            }
            2 => {
                sim.world_mut()
                    .entity_mut(child)
                    .insert(LocalName("forged".into()));
            }
            3 => {
                sim.world_mut().entity_mut(child).insert(EntityKind::Sensor);
            }
            4 => {
                sim.world_mut().entity_mut(child).remove::<EntityId>();
            }
            5 => {
                let source_link = entity(&sim, "w::source::body");
                let handle = sim
                    .world_mut()
                    .entity_mut(source_link)
                    .take::<BodyHandle>()
                    .unwrap();
                sim.world_mut().entity_mut(child).insert(handle);
                foreign_handle = Some(source_link);
            }
            6 => {
                let fake = sim
                    .world_mut()
                    .spawn((EntityKind::Collision, ChildOf(link), Name("forged".into())))
                    .id();
                assign_id(sim.world_mut(), fake).unwrap();
                extra = Some(fake);
            }
            _ => {
                sim.world_mut()
                    .entity_mut(link)
                    .get_mut::<LocalPose>()
                    .unwrap()
                    .0
                    .translation[0] = 0.25;
            }
        }
        let before = snapshot(&sim);
        assert!(
            matches!(
                prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]),
                Err(WorldEditError::UnsupportedModel)
            ),
            "drift row {row}"
        );
        assert_eq!(snapshot(&sim), before);
        assert!(matches!(
            reset_loaded_world(&mut sim, |_| ()),
            Err(SceneResetError::RuntimePhysics)
        ));
        assert_eq!(snapshot(&sim), before);
        match row {
            0 => {
                sim.world_mut().entity_mut(child).insert(ChildOf(link));
            }
            1 => {
                sim.world_mut().entity_mut(child).insert(local);
            }
            2 => {
                sim.world_mut().entity_mut(child).insert(LocalName(name));
            }
            3 => {
                sim.world_mut().entity_mut(child).insert(kind);
            }
            4 => {
                restore_id(sim.world_mut(), child, &reserved).unwrap();
            }
            5 => {
                let handle = sim
                    .world_mut()
                    .entity_mut(child)
                    .take::<BodyHandle>()
                    .unwrap();
                sim.world_mut()
                    .entity_mut(foreign_handle.unwrap())
                    .insert(handle);
            }
            6 => {
                sim.world_mut().entity_mut(extra.unwrap()).despawn();
            }
            _ => {
                sim.world_mut()
                    .entity_mut(link)
                    .get_mut::<LocalPose>()
                    .unwrap()
                    .0
                    .translation = [0.; 3];
            }
        }
        prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
        sim.step_exact(1).unwrap();
        vector(native(&sim, body(&sim, model)).velocity, [0.002, 0., 0.]);
    }
}

#[test]
fn retemplated_camera_capacity_and_pending_inputs_recover_without_consuming_ids() {
    let camera = "<model name='observer'><static>true</static><pose>-400 0 10 0 0 0</pose><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><sensor name='logical' type='logical_camera'><update_rate>0</update_rate><logical_camera><near>0.1</near><far>1000</far><horizontal_fov>1.5</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor></link></model>";
    let xml = document(2, false, camera);
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let initial = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let first = spawn_model_template(&mut sim, &initial, cycle([100., 20., 30.])).unwrap();
    let supplied = requested(first, 2, false, 17.);
    let template = prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
    queue_link_world_velocity(&mut sim, first + 1, [1., 0., 0.], [0.; 3]).unwrap();
    queue_link_world_wrench(&mut sim, first + 1, [4., 0., 0.], [0.; 3]).unwrap();
    let mut clones = Vec::new();
    // Three loaded models, one runtime source, then124 descendants reach128.
    for i in 0..124 {
        clones.push(
            spawn_model_template(
                &mut sim,
                &template,
                cycle([1000. + i as f64 * 8., 20., 30.]),
            )
            .unwrap(),
        );
    }
    let last_body = body(&sim, *clones.last().unwrap());
    let observed = snapshot(&sim);
    assert!(matches!(
        spawn_model_template(&mut sim, &template, cycle([200., 20., 30.])),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim), observed);
    remove_model(&mut sim, clones[0]).unwrap();
    let next = next_entity_id(sim.world()).unwrap();
    let recovered = spawn_model_template(&mut sim, &template, cycle([200., 20., 30.])).unwrap();
    assert_eq!(recovered, next);
    assert_eq!(body(&sim, recovered).raw(), last_body.raw() + 1);
    sim.step_exact(1).unwrap();
    near(native(&sim, body(&sim, first)).velocity[0], 1.002);
}

#[test]
fn retemplated_static_geometry_has_real_positive_gap_contact_and_a_miss_control() {
    let falling = "<model name='ball'><pose>0 0 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy><izz>0.025</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model><model name='miss'><pose>30 0 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy><izz>0.025</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";
    let xml = document(2, true, falling).replacen(
        "<gravity>0 0 0</gravity>",
        "<gravity>0 0 -9.8</gravity>",
        1,
    );
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let initial = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let source = spawn_model_template(&mut sim, &initial, cycle([100., 20., 30.])).unwrap();
    let mut supplied = requested(source, 2, true, 17.);
    let model = &mut supplied.worlds[0].models[0];
    model.links[0].collisions[0].geometry = Geometry::Box { size: [2., 2., 1.] };
    model.links[0].collisions[0].pose.transform.rotation = [0., 0., 0., 1.];
    model.links[0].collisions[1].pose.transform.translation = [10., 0., 0.];
    let copied = prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
    let floor =
        spawn_model_template(&mut sim, &copied, pose([0., 0., 1.], [0., 0., 0., 1.])).unwrap();
    let floor_state = native(&sim, body(&sim, floor));
    let ball = sim
        .world()
        .get::<BodyHandle>(entity(&sim, "w::ball::body"))
        .unwrap()
        .id();
    let miss = sim
        .world()
        .get::<BodyHandle>(entity(&sim, "w::miss::body"))
        .unwrap()
        .id();
    // Reuse the retained upfront1.5s positive-gap contact horizon and bounds.
    for _ in 0..1500 {
        sim.step_exact(1).unwrap();
    }
    assert!((native(&sim, ball).position[2] - 1.75).abs() < 0.01);
    assert!(native(&sim, miss).position[2] < 0.);
    assert_eq!(native(&sim, body(&sim, floor)), floor_state);
    sim.reset().unwrap();
    for _ in 0..1500 {
        sim.step_exact(1).unwrap();
    }
    assert!((native(&sim, ball).position[2] - 1.75).abs() < 0.01);
    assert!(native(&sim, miss).position[2] < 0.);
}
