// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Runtime compound templates retain one native body and exact bounded ownership.
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
use sdformat::{Geometry, NoResources};
use std::{collections::BTreeMap, sync::Arc, time::Duration};

const CONTROL: &str =
    "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/>";
const I: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial>";
const VISUAL: &str = "<visual name='view'><pose>0.25 -0.5 0.75 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual>";
fn document(count: usize, fixed: bool, extra: &str) -> String {
    let children = (0..count).map(|index| {
        let (shape, rotation) = match index % 4 {
            0 => ("<sphere><radius>0.5</radius></sphere>", "0 0 0.3"),
            1 => ("<cylinder><radius>0.5</radius><length>2</length></cylinder>", "0 1.5707963267948966 0"),
            2 => ("<box><size>1.5 0.5 1</size></box>", "0 0 1.5707963267948966"),
            _ => ("<capsule><radius>0.5</radius><length>2</length></capsule>", "0 1.5707963267948966 0"),
        };
        format!("<collision name='c{index}'><pose>2 0 {} {rotation}</pose><density>{}</density><geometry>{shape}</geometry></collision>", 4 * index, 17 + index)
    }).collect::<String>();
    format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='source'><pose>-100 0 10 0 0 0</pose><static>{fixed}</static><link name='body'>{I}{children}{VISUAL}</link></model><model name='survivor'><pose>200 200 10 0 0 0</pose><static>true</static><link name='body'><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision>{VISUAL}</link></model>{extra}</world></sdf>"
    )
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "runtime-compound",
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
    native: Vec<(BodyId, Option<BodyState>)>,
}
fn snapshot(sim: &Simulation, bodies: &[BodyId]) -> Atomic {
    Atomic {
        ids: identities(sim),
        entities: sim.world().iter_entities().count(),
        next: next_entity_id(sim.world()).unwrap(),
        clock: (sim.epoch(), sim.elapsed()),
        native: bodies
            .iter()
            .map(|id| (*id, sim.world().resource::<PhysicsBackend>().state(*id)))
            .collect(),
    }
}
fn source_bodies(sim: &Simulation) -> [BodyId; 2] {
    [
        sim.world()
            .get::<BodyHandle>(entity(sim, "w::source::body"))
            .unwrap()
            .id(),
        sim.world()
            .get::<BodyHandle>(entity(sim, "w::survivor::body"))
            .unwrap()
            .id(),
    ]
}

#[test]
fn runtime_compounds_keep_mixed_child_frames_prefix_ids_and_isolated_global_queries() {
    for count in [2, 4, 16] {
        for fixed in [false, true] {
            let xml = document(count, fixed, "");
            let sdf = sdformat::load_str(&xml, "compound", &NoResources).unwrap();
            let mut sim = load(&xml);
            let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
            let original_visual = sim
                .world()
                .get::<VisualGeometry>(entity(&sim, "w::source::body::view"))
                .unwrap()
                .0
                .clone();
            assert_eq!(
                template.source_visual_id(),
                id(&sim, "w::source::body::view")
            );
            let first = next_entity_id(sim.world()).unwrap();
            let model = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
            assert_eq!(model, first);
            assert_eq!(
                next_entity_id(sim.world()).unwrap(),
                model + count as u64 + 3
            );
            let link = entity_by_id(sim.world(), model + 1).unwrap();
            let handle = body(&sim, model);
            assert_eq!(sim.world().get::<Children>(link).unwrap().len(), count + 1);
            assert_eq!(
                sim.world()
                    .iter_entities()
                    .filter(|e| e.get::<BodyHandle>().is_some_and(|v| v.id() == handle))
                    .count(),
                1
            );
            let visual = entity_by_id(sim.world(), model + 3).unwrap();
            assert!(Arc::ptr_eq(
                &original_visual,
                &sim.world().get::<VisualGeometry>(visual).unwrap().0
            ));
            vector(
                sim.world().get::<WorldPose>(visual).unwrap().translation,
                [10.75, 20.25, 29.5],
            );
            for index in 0..count {
                let child_id = if index == 0 {
                    model + 2
                } else {
                    model + index as u64 + 3
                };
                let child = entity_by_id(sim.world(), child_id).unwrap();
                assert_eq!(
                    sim.world().get::<EntityKind>(child),
                    Some(&EntityKind::Collision)
                );
                assert_eq!(sim.world().get::<ChildOf>(child).unwrap().parent(), link);
                assert!(sim.world().get::<BodyHandle>(child).is_none());
                assert_eq!(
                    sim.world().get::<LocalName>(child).unwrap().0,
                    if index == 0 {
                        "collision".to_owned()
                    } else {
                        format!("collision_{index}")
                    }
                );
                near(
                    sim.world().get::<CollisionDensity>(child).unwrap().0,
                    (17 + index) as f64,
                );
                let center = [10. + 4. * index as f64, 22., 30.];
                vector(
                    sim.world().get::<WorldPose>(child).unwrap().translation,
                    center,
                );
                vector(
                    sim.world().get::<LocalPose>(child).unwrap().0.translation,
                    [2., 0., 4. * index as f64],
                );
                // The cyclic model rotation maps the child lane Z to world X.
                // A child's local Y quarter turn maps its axial Z to world Y.
                let expected = if index % 4 == 2 { 3.25 } else { 3.5 };
                near(
                    ray(
                        &sim,
                        [center[0], center[1], center[2] + 4.],
                        [0., 0., -1.],
                        5.,
                        None,
                    )
                    .unwrap(),
                    expected,
                );
            }
            // Strict 3-4-5 cylinder chord: other children differ in X by at least4.
            near(
                ray(&sim, [14.3, 22., 34.], [0., 0., -1.], 5., None).unwrap(),
                3.6,
            );
            near(
                ray(&sim, [14., 25., 30.], [0., -1., 0.], 5., None).unwrap(),
                2.,
            );
            if count >= 4 {
                near(
                    ray(&sim, [22.3, 27., 30.], [0., -1., 0.], 5., None).unwrap(),
                    3.6,
                );
            }
            // This passes through the aggregate interior between distinct X lanes.
            assert_eq!(ray(&sim, [12., 22., 34.], [0., 0., -1.], 5., None), None);
            assert_eq!(
                ray(&sim, [10., 22., 30.], [0., 0., 1.], 0.125, None),
                Some(0.)
            );
            assert_eq!(
                ray(&sim, [14.3, 22., 34.], [0., 0., -1.], 5., Some(handle)),
                None
            );
            let closer = spawn_model_template(&mut sim, &template, cycle([10., 20., 32.])).unwrap();
            near(
                ray(&sim, [10., 22., 34.], [0., 0., -1.], 5., None).unwrap(),
                1.5,
            );
            near(
                ray(
                    &sim,
                    [10., 22., 34.],
                    [0., 0., -1.],
                    5.,
                    Some(body(&sim, closer)),
                )
                .unwrap(),
                3.5,
            );
            near(
                ray(&sim, [10., 22., 34.], [0., 0., -1.], 5., Some(handle)).unwrap(),
                1.5,
            );
            // Exact max/short-bound witness: model and sphere rotations are identity.
            // Radius .5, center [52,0,10] and origin [53.5,0,10] then leave only
            // binary coordinates in the local quadratic. The mixed template above
            // retains its original sphere yaw and every other member pose.
            let mut binary_requested = sdf.worlds[0].models[0].clone();
            assert_ne!(
                binary_requested.links[0].collisions[0]
                    .pose
                    .transform
                    .rotation,
                [0., 0., 0., 1.]
            );
            assert_eq!(
                binary_requested.links[0].collisions[0]
                    .pose
                    .transform
                    .translation,
                [2., 0., 0.]
            );
            binary_requested.links[0].collisions[0]
                .pose
                .transform
                .rotation = [0., 0., 0., 1.];
            let binary_template = prepare_model_template(&sim, &sdf, &binary_requested).unwrap();
            let binary = spawn_model_template(
                &mut sim,
                &binary_template,
                pose([50., 0., 10.], [0., 0., 0., 1.]),
            )
            .unwrap();
            assert_eq!(
                ray(&sim, [53.5, 0., 10.], [-1., 0., 0.], 1., None),
                Some(1.)
            );
            assert_eq!(ray(&sim, [53.5, 0., 10.], [-1., 0., 0.], 0.999, None), None);
            assert_eq!(
                ray(&sim, [52., 0., 10.], [1., 0., 0.], 0.125, None),
                Some(0.)
            );
            assert_eq!(
                ray(
                    &sim,
                    [53.5, 0., 10.],
                    [-1., 0., 0.],
                    1.,
                    Some(body(&sim, binary))
                ),
                None
            );
            assert_eq!(sim.elapsed(), Duration::ZERO);
        }
    }
}

#[test]
fn runtime_compound_six_axis_edits_and_caller_parameters_keep_explicit_physics() {
    let xml = document(4, false, "");
    let sdf = sdformat::load_str(&xml, "compound", &NoResources).unwrap();
    let mut sim = load(&xml);
    let source_state = native(&sim, source_bodies(&sim)[0]);
    let mut requested = sdf.worlds[0].models[0].clone();
    requested.links[0].inertial.mass = 4.;
    requested.links[0].inertial.inertia = [8., 10., 12., 0., 0., 0.];
    requested.links[0].collisions[1].geometry = Geometry::Cylinder {
        radius: 0.75,
        length: 2.,
    };
    let template = prepare_model_template(&sim, &sdf, &requested).unwrap();
    // Subsequent caller-DOM edits cannot change the already copied parameters.
    requested.links[0].inertial.mass = 100.;
    requested.links[0].collisions[1].geometry = Geometry::Sphere { radius: 9. };
    let model = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
    let handle = body(&sim, model);
    vector(
        model_state(&sim, model).unwrap().translation,
        [10., 20., 30.],
    );
    let target = ModelState {
        translation: [30., 40., 50.],
        rotation: [0.5; 4],
        linear_velocity: [1., -2., 3.],
        angular_velocity: [0.2, -0.3, 0.4],
    };
    let before_ids = identities(&sim);
    let clock = (sim.epoch(), sim.elapsed());
    assert_eq!(set_model_state(&mut sim, model, target).unwrap(), target);
    assert_eq!(model_state(&sim, model).unwrap(), target);
    let state = native(&sim, handle);
    vector(state.position, target.translation);
    vector(state.velocity, target.linear_velocity);
    vector(state.angular_velocity, target.angular_velocity);
    // The copied requested radius .75 is intentionally different from the loaded .5.
    near(
        ray(&sim, [34., 42., 54.], [0., 0., -1.], 5., None).unwrap(),
        3.25,
    );
    vector(
        sim.world()
            .get::<WorldPose>(entity_by_id(sim.world(), model + 4).unwrap())
            .unwrap()
            .translation,
        [34., 42., 50.],
    );
    assert_eq!(identities(&sim), before_ids);
    assert_eq!((sim.epoch(), sim.elapsed()), clock);
    let stopped = ModelState {
        linear_velocity: [0.; 3],
        angular_velocity: [0.; 3],
        ..target
    };
    set_model_state(&mut sim, model, stopped).unwrap();
    sim.set_paused(true);
    queue_link_world_wrench(&mut sim, model + 1, [8., 0., 0.], [0., 8., 0.]).unwrap();
    assert!(sim.tick().unwrap().dt.is_zero());
    assert_eq!(model_state(&sim, model).unwrap(), stopped);
    let before = native(&sim, handle);
    assert!(matches!(
        set_model_state(
            &mut sim,
            model,
            ModelState {
                rotation: [0.; 4],
                ..target
            }
        ),
        Err(WorldEditError::InvalidState)
    ));
    assert_eq!(native(&sim, handle), before);
    sim.set_paused(false);
    sim.step_exact(1).unwrap();
    // Model Q cyclically maps link I=(8,10,12) to world I=(12,8,10).
    near(native(&sim, handle).velocity[0], 0.002);
    near(native(&sim, handle).angular_velocity[1], 0.001);
    assert_eq!(native(&sim, source_bodies(&sim)[0]), source_state);
}

#[test]
fn runtime_compound_source_removal_both_resets_and_retirement_preserve_aliases_and_high_water() {
    for fixed in [false, true] {
        let xml = document(4, fixed, "");
        let sdf = sdformat::load_str(&xml, "compound", &NoResources).unwrap();
        let mut sim = load(&xml);
        let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
        let initial_ids = identities(&sim);
        let initial_states = source_bodies(&sim).map(|id| (id, native(&sim, id)));
        let first = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
        let first_body = body(&sim, first);
        let original = model_state(&sim, first).unwrap();
        let mut target = original;
        target.translation = [20., 30., 40.];
        target.rotation = [0., 0., 1., 0.];
        set_model_state(&mut sim, first, target).unwrap();
        sim.reset().unwrap();
        assert_eq!(model_state(&sim, first).unwrap(), original);
        assert_eq!(body(&sim, first), first_body);
        near(
            ray(&sim, [14.3, 22., 34.], [0., 0., -1.], 5., None).unwrap(),
            3.6,
        );
        let removed = remove_model(&mut sim, first).unwrap();
        assert_eq!(removed, (first..first + 7).collect::<Vec<_>>());
        assert!(
            sim.world()
                .resource::<PhysicsBackend>()
                .state(first_body)
                .is_none()
        );
        let source = id(&sim, "w::source");
        remove_model(&mut sim, source).unwrap();
        let next = next_entity_id(sim.world()).unwrap();
        let second = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
        assert_eq!(second, next);
        assert!(second > first + 6);
        assert!(body(&sim, second).raw() > first_body.raw());
        let visual = entity_by_id(sim.world(), second + 3).unwrap();
        assert_eq!(
            sim.world().get::<EntityKind>(visual),
            Some(&EntityKind::Visual)
        );
        assert!(model_state(&sim, second).is_ok());
        let high = next_entity_id(sim.world()).unwrap();
        reset_loaded_world(&mut sim, |observed| {
            for offset in 0..7 {
                let node = entity_by_id(observed.world(), second + offset).unwrap();
                assert!(observed.world().get::<PendingResetRemoval>(node).is_some());
                assert!(observed.world().get::<BodyHandle>(node).is_none());
                assert!(observed.world().get::<WorldPose>(node).is_some());
            }
        })
        .unwrap();
        assert_eq!(next_entity_id(sim.world()).unwrap(), high);
        assert!(entity_by_id(sim.world(), second).is_none());
        assert_eq!(
            identities(&sim).keys().copied().collect::<Vec<_>>(),
            initial_ids.keys().copied().collect::<Vec<_>>()
        );
        for (body, state) in initial_states {
            assert_eq!(native(&sim, body), state);
        }
        let third = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
        assert_eq!(third, high);
        near(
            ray(&sim, [14.3, 22., 34.], [0., 0., -1.], 5., None).unwrap(),
            3.6,
        );
    }
}

#[test]
fn runtime_compound_contacts_use_copied_geometry_and_the_retained_settling_window() {
    let balls = "<model name='ball'><pose>0 0 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy><izz>0.025</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model><model name='miss'><pose>30 0 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy><izz>0.025</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";
    let xml = document(2, true, balls).replacen(
        "<gravity>0 0 0</gravity>",
        "<gravity>0 0 -9.8</gravity>",
        1,
    );
    let sdf = sdformat::load_str(&xml, "contact", &NoResources).unwrap();
    let mut sim = load(&xml);
    let mut requested = sdf.worlds[0].models[0].clone();
    requested.links[0].collisions[0].geometry = Geometry::Box { size: [2., 2., 1.] };
    requested.links[0].collisions[0].pose.transform.translation = [0.; 3];
    requested.links[0].collisions[1].pose.transform.translation = [10., 0., 0.];
    let template = prepare_model_template(&sim, &sdf, &requested).unwrap();
    let model =
        spawn_model_template(&mut sim, &template, pose([0., 0., 1.], [0., 0., 0., 1.])).unwrap();
    let fixed = native(&sim, body(&sim, model));
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
    // Same upfront1.5s contact horizon as retained compound controls, no solver trial.
    for _ in 0..1500 {
        sim.step_exact(1).unwrap();
    }
    assert!((native(&sim, ball).position[2] - 1.75).abs() < 0.01);
    assert!(native(&sim, miss).position[2] < 0.);
    assert_eq!(native(&sim, body(&sim, model)), fixed);
    sim.reset().unwrap();
    for _ in 0..1500 {
        sim.step_exact(1).unwrap();
    }
    assert!((native(&sim, ball).position[2] - 1.75).abs() < 0.01);
    assert!(native(&sim, miss).position[2] < 0.);
}

#[test]
fn runtime_compound_preparation_keeps_original_domains_and_late_input_errors_closed() {
    let xml = document(2, false, "");
    let sdf = sdformat::load_str(&xml, "compound", &NoResources).unwrap();
    let mut sim = load(&xml);
    let bodies = source_bodies(&sim);
    let before = snapshot(&sim, &bodies);
    prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    for row in 0..10 {
        let mut requested = sdf.worlds[0].models[0].clone();
        match row {
            0 => requested.links[0].collisions.clear(),
            1 => {
                let child = requested.links[0].collisions[0].clone();
                requested.links[0].collisions = vec![child; 17];
            }
            2 => {
                requested.links[0].collisions[1].geometry = Geometry::Plane {
                    normal: [0., 0., 1.],
                    size: [1., 1.],
                }
            }
            3 => {
                let child = &mut requested.links[0].collisions[1];
                child.geometry = Geometry::Mesh(sdformat::Mesh {
                    uri: "not-read.glb".into(),
                    scale: [1.; 3],
                    submesh: String::new(),
                    center_submesh: false,
                    optimization: Default::default(),
                    convex_decomposition: None,
                    source: child.source.clone(),
                });
            }
            4 => requested.links[0].collisions[1].pose.relative_to = "missing_last_child".into(),
            5 => requested.links[0].collisions[1].density = Some(f64::NAN),
            6 => requested.links[0].collisions[1].geometry = Geometry::Sphere { radius: -1. },
            7 => requested.links[0].inertial.pose.transform.translation = [0.25, 0., 0.],
            8 => requested.links[0].pose.transform.translation = [1., 0., 0.],
            _ => {
                let mut link = requested.links[0].clone();
                link.name = "other".into();
                requested.links.push(link);
            }
        }
        let result = prepare_model_template(&sim, &sdf, &requested);
        match row {
            4 => assert!(matches!(result, Err(WorldEditError::InvalidScene))),
            5 | 6 => assert!(matches!(result, Err(WorldEditError::InvalidState))),
            _ => assert!(matches!(result, Err(WorldEditError::UnsupportedModel))),
        }
        assert_eq!(snapshot(&sim, &bodies), before, "input row{row}");
    }
    // Native mass/inertia rejection remains before either allocator commits.
    for row in 0..3 {
        let mut requested = sdf.worlds[0].models[0].clone();
        match row {
            0 => requested.links[0].inertial.mass = f64::NAN,
            1 => requested.links[0].inertial.inertia[0] = -1.,
            _ => requested.links[0].collisions[1].geometry = Geometry::Sphere { radius: 1e200 },
        }
        let template = prepare_model_template(&sim, &sdf, &requested).unwrap();
        assert!(matches!(
            spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])),
            Err(WorldEditError::Physics(PhysicsError::InvalidValue))
        ));
        assert_eq!(snapshot(&sim, &bodies), before);
    }
    let valid = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let logical_next = next_entity_id(sim.world()).unwrap();
    let native_next = bodies.iter().map(|body| body.raw()).max().unwrap() + 1;
    let accepted = spawn_model_template(&mut sim, &valid, cycle([10., 20., 30.])).unwrap();
    assert_eq!(accepted, logical_next);
    assert_eq!(body(&sim, accepted).raw(), native_next);
    model_state(&sim, accepted).unwrap();
    let runtime_xml = format!(
        "<sdf version='1.12'><world name='w'><model name='runtime_{accepted}'><link name='body'><collision name='collision'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><visual name='visual'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>"
    );
    let runtime_sdf = sdformat::load_str(&runtime_xml, "runtime-source", &NoResources).unwrap();
    let requested = &runtime_sdf.worlds[0].models[0];
    assert_eq!(requested.scoped_name, format!("w::runtime_{accepted}"));
    assert_eq!(
        requested.links[0].scoped_name,
        format!("w::runtime_{accepted}::body")
    );
    assert_eq!(requested.links[0].visuals[0].name, "visual");
    // This graph really resolves every supplied pose: without the actual-source
    // veto, the old centered singleton preparation route would accept this DOM.
    for value in [
        &requested.links[0].collisions[0].pose,
        &requested.links[0].visuals[0].pose,
        &requested.links[0].inertial.pose,
    ] {
        let resolved = runtime_sdf
            .resolve_pose(
                value,
                &requested.scoped_name,
                &requested.links[0].scoped_name,
                &requested.links[0].scoped_name,
            )
            .unwrap();
        assert_eq!(resolved.translation, [0.; 3]);
        assert_eq!(resolved.rotation, [0., 0., 0., 1.]);
    }
    let current = snapshot(&sim, &[bodies[0], bodies[1], body(&sim, accepted)]);
    assert!(matches!(
        prepare_model_template(&sim, &runtime_sdf, requested),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(
        snapshot(&sim, &[bodies[0], bodies[1], body(&sim, accepted)]),
        current
    );
    let controlled = xml.replacen(
        "<link name='body'>",
        &format!("{CONTROL}<link name='body'>"),
        1,
    );
    let controlled_sdf = sdformat::load_str(&controlled, "owner", &NoResources).unwrap();
    let owner = load(&controlled);
    let mut stripped = controlled_sdf.worlds[0].models[0].clone();
    stripped.plugins.clear();
    assert!(matches!(
        prepare_model_template(&owner, &controlled_sdf, &stripped),
        Err(WorldEditError::UnsupportedModel)
    ));
    let sensor = xml.replacen(VISUAL, &format!("{VISUAL}<sensor name='imu' type='imu'><always_on>true</always_on><update_rate>100</update_rate></sensor>"), 1);
    let sensor_sdf = sdformat::load_str(&sensor, "sensor", &NoResources).unwrap();
    let sensors = load(&sensor);
    let mut stripped = sensor_sdf.worlds[0].models[0].clone();
    stripped.links[0].sensors.clear();
    assert!(matches!(
        prepare_model_template(&sensors, &sensor_sdf, &stripped),
        Err(WorldEditError::UnsupportedModel)
    ));
    let jointed = xml.replacen("</link></model>", "</link><joint name='hinge' type='revolute'><parent>world</parent><child>body</child><axis><xyz>0 0 1</xyz></axis></joint></model>", 1);
    let joint_sdf = sdformat::load_str(&jointed, "joint", &NoResources).unwrap();
    let joints = load(&jointed);
    let mut stripped = joint_sdf.worlds[0].models[0].clone();
    stripped.joints.clear();
    assert!(matches!(
        prepare_model_template(&joints, &joint_sdf, &stripped),
        Err(WorldEditError::UnsupportedModel)
    ));
    let singleton = "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='source'><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='view'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>";
    let one = sdformat::load_str(singleton, "one", &NoResources).unwrap();
    let single = load(singleton);
    let mut forged = one.worlds[0].models[0].clone();
    let child = forged.links[0].collisions[0].clone();
    forged.links[0].collisions.push(child);
    assert!(matches!(
        prepare_model_template(&single, &one, &forged),
        Err(WorldEditError::UnsupportedModel)
    ));
    let tpe =
        load_world_with_sdf_step(singleton, "tpe", "w", &NoResources, PhysicsEngine::Tpe).unwrap();
    let tpe_ids = identities(&tpe);
    let clock = (tpe.epoch(), tpe.elapsed());
    assert!(matches!(
        prepare_model_template(&tpe, &one, &one.worlds[0].models[0]),
        Err(WorldEditError::InvalidScene)
    ));
    assert_eq!(identities(&tpe), tpe_ids);
    assert_eq!((tpe.epoch(), tpe.elapsed()), clock);
}

#[test]
fn runtime_compound_membership_and_parent_metadata_drift_refuse_without_native_mutation() {
    for row in 0..11 {
        let xml = document(2, false, "");
        let sdf = sdformat::load_str(&xml, "compound", &NoResources).unwrap();
        let mut sim = load(&xml);
        let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
        let model = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
        model_state(&sim, model).unwrap();
        let handle = body(&sim, model);
        let source = source_bodies(&sim);
        let tracked = [source[0], source[1], handle];
        queue_link_world_wrench(&mut sim, model + 1, [4., 0., 0.], [0.; 3]).unwrap();
        let root = entity_by_id(sim.world(), model).unwrap();
        let link = entity_by_id(sim.world(), model + 1).unwrap();
        let last = entity_by_id(sim.world(), model + 4).unwrap();
        match row {
            0 => {
                sim.world_mut().entity_mut(last).despawn();
            }
            1 => {
                let foreign = entity(&sim, "w::source::body");
                sim.world_mut().entity_mut(last).insert(ChildOf(foreign));
            }
            2 => {
                sim.world_mut().entity_mut(last).insert(EntityKind::Sensor);
            }
            3 => {
                sim.world_mut()
                    .entity_mut(last)
                    .get_mut::<LocalPose>()
                    .unwrap()
                    .0
                    .translation[0] += 0.25;
            }
            4 => {
                sim.world_mut().entity_mut(last).remove::<WorldPose>();
            }
            5 => {
                sim.world_mut()
                    .entity_mut(last)
                    .get_mut::<LocalName>()
                    .unwrap()
                    .0
                    .push_str("_forged");
            }
            6 => {
                let visual = entity_by_id(sim.world(), model + 3).unwrap();
                sim.world_mut()
                    .entity_mut(visual)
                    .insert(VisualGeometry(Arc::new(Geometry::Sphere { radius: 0.5 })));
            }
            7 => {
                let foreign = entity(&sim, "w::survivor::body");
                let body = sim
                    .world_mut()
                    .entity_mut(foreign)
                    .take::<BodyHandle>()
                    .unwrap();
                sim.world_mut().entity_mut(last).insert(body);
            }
            8 => {
                let fake = sim
                    .world_mut()
                    .spawn((
                        EntityKind::Collision,
                        Name("forged".into()),
                        ChildOf(link),
                        LocalName("forged".into()),
                        LocalPose(pose([0.; 3], [0., 0., 0., 1.])),
                        pose([0.; 3], [0., 0., 0., 1.]),
                    ))
                    .id();
                assign_id(sim.world_mut(), fake).unwrap();
            }
            9 => {
                let reserved = reserve_id(sim.world(), root).unwrap();
                sim.world_mut().entity_mut(root).remove::<EntityId>();
                let world = entity(&sim, "w");
                let fake = sim
                    .world_mut()
                    .spawn((
                        EntityKind::Model,
                        Name(format!("w::runtime_{model}")),
                        LocalName(format!("runtime_{model}")),
                        ChildOf(world),
                    ))
                    .id();
                restore_id(sim.world_mut(), fake, &reserved).unwrap();
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
        let before = snapshot(&sim, &tracked);
        assert!(
            matches!(
                model_state(&sim, model),
                Err(WorldEditError::UnsupportedModel)
            ),
            "drift row{row}"
        );
        assert!(matches!(
            set_model_state(
                &mut sim,
                model,
                ModelState {
                    translation: [50., 60., 70.],
                    rotation: [0., 0., 0., 1.],
                    linear_velocity: [0.; 3],
                    angular_velocity: [0.; 3]
                }
            ),
            Err(WorldEditError::UnsupportedModel)
        ));
        assert_eq!(snapshot(&sim, &tracked), before);
        assert!(matches!(
            reset_loaded_world(&mut sim, |_| ()),
            Err(SceneResetError::RuntimePhysics)
        ));
        assert_eq!(snapshot(&sim, &tracked), before);
    }
    let xml = document(16, false, "");
    let sdf = sdformat::load_str(&xml, "compound", &NoResources).unwrap();
    let mut sim = load(&xml);
    let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let bodies = source_bodies(&sim);
    let world = entity(&sim, "w");
    let missing = sim
        .world_mut()
        .entity_mut(world)
        .take::<LocalName>()
        .unwrap();
    let before = snapshot(&sim, &bodies);
    assert!(matches!(
        spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])),
        Err(WorldEditError::InvalidScene)
    ));
    assert_eq!(snapshot(&sim, &bodies), before);
    sim.world_mut().entity_mut(world).insert(missing);
    let next = next_entity_id(sim.world()).unwrap();
    let trap = sim
        .world_mut()
        .spawn(Name(format!("w::runtime_{next}::body::collision_15")))
        .id();
    let before = snapshot(&sim, &bodies);
    assert!(matches!(
        spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim, &bodies), before);
    sim.world_mut().entity_mut(trap).despawn();
    let mut foreign = load(&xml);
    let foreign_bodies = source_bodies(&foreign);
    let before = snapshot(&foreign, &foreign_bodies);
    assert!(matches!(
        spawn_model_template(&mut foreign, &template, cycle([10., 20., 30.])),
        Err(WorldEditError::InvalidScene)
    ));
    assert_eq!(snapshot(&foreign, &foreign_bodies), before);
    assert_eq!(
        spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap(),
        next
    );
}

#[test]
fn runtime_compound_camera_capacity_and_pending_inputs_keep_both_allocators_atomic() {
    let camera = "<model name='observer'><static>true</static><pose>-200 0 10 0 0 0</pose><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><sensor name='logical' type='logical_camera'><update_rate>0</update_rate><logical_camera><near>0.1</near><far>1000</far><horizontal_fov>1.5</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor></link></model>";
    let xml = document(2, false, camera);
    let sdf = sdformat::load_str(&xml, "capacity", &NoResources).unwrap();
    let mut sim = load(&xml);
    let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let source = source_bodies(&sim);
    let source_link = id(&sim, "w::source::body");
    queue_link_world_velocity(&mut sim, source_link, [1., 0., 0.], [0.; 3]).unwrap();
    queue_link_world_wrench(&mut sim, source_link, [4., 0., 0.], [0.; 3]).unwrap();
    let mut first = None;
    let mut last = None;
    // Three loaded models +125 clones exercise the existing real128-model cap.
    for index in 0..125 {
        let model = spawn_model_template(
            &mut sim,
            &template,
            pose([1000. + index as f64 * 8., 0., 10.], [0., 0., 0., 1.]),
        )
        .unwrap();
        first.get_or_insert(model);
        last = Some(model);
    }
    let last = last.unwrap();
    let last_body = body(&sim, last);
    let before = snapshot(&sim, &source);
    assert!(matches!(
        spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim, &source), before);
    remove_model(&mut sim, first.unwrap()).unwrap();
    let next = next_entity_id(sim.world()).unwrap();
    let recovered = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
    assert_eq!(recovered, next);
    assert_eq!(body(&sim, recovered).raw(), last_body.raw() + 1);
    sim.step_exact(1).unwrap();
    near(native(&sim, source[0]).velocity[0], 1.002);
}
