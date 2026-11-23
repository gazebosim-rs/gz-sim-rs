// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicit compound COM belongs to the mass frame, not child geometry or ownership.
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, BodyState, PhysicsBackend, PhysicsError};
use gz_sim_core::{EntityId, Simulation, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, CollisionDensity, EntityKind, LocalName, LocalPose, ModelState, Name,
    PhysicsEngine, SceneResetError, VisualGeometry, WorldEditError, WorldPose,
    load_world_with_sdf_step, model_state, prepare_model_template, queue_link_world_velocity,
    queue_link_world_wrench, remove_model, reset_loaded_world, set_model_state,
    spawn_model_template,
};
use sdformat::{Geometry, NoResources, Sdf};
use std::{collections::BTreeMap, sync::Arc, time::Duration};

const CONTROL: &str =
    "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/>";
const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>2</izz></inertia></inertial>";
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
    {
        let mut sim = load_world_with_sdf_step(
            xml,
            "compound-com",
            "w",
            &NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        // Initialize Bevy's resource-backed schedules without running callbacks.
        sim.initialize_schedules().unwrap();
        sim
    }
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
struct LoadedBinding {
    name: String,
    kind: EntityKind,
    parent: Option<u64>,
    body: Option<u64>,
    local: Option<([f64; 3], [f64; 4])>,
}
fn loaded_bindings(sim: &Simulation) -> BTreeMap<u64, LoadedBinding> {
    identities(sim)
        .into_iter()
        .map(|(id, entity)| {
            let world = sim.world();
            (
                id,
                LoadedBinding {
                    name: world.get::<Name>(entity).unwrap().0.clone(),
                    kind: *world.get::<EntityKind>(entity).unwrap(),
                    parent: world
                        .get::<ChildOf>(entity)
                        .map(|parent| world.get::<EntityId>(parent.parent()).unwrap().get()),
                    body: world
                        .get::<BodyHandle>(entity)
                        .map(|handle| handle.id().raw()),
                    local: world
                        .get::<LocalPose>(entity)
                        .map(|local| (local.0.translation, local.0.rotation)),
                },
            )
        })
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

const COM: [f64; 3] = [0.25, -0.5, 0.75];
fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] + b[i])
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let u = [q[0], q[1], q[2]];
    let uv = cross(u, v);
    let uuv = cross(u, uv);
    std::array::from_fn(|i| v[i] + 2. * (q[3] * uv[i] + uuv[i]))
}
fn offset(sdf: &Sdf, center: [f64; 3], density: f64) -> sdformat::Model {
    let mut model = sdf.worlds[0].models[0].clone();
    model.links[0].inertial.pose.transform.translation = center;
    for child in &mut model.links[0].collisions {
        child.density = Some(density);
    }
    model
}
fn center(state: BodyState, c: [f64; 3]) -> [f64; 3] {
    add(state.position, rotate(state.orientation, c))
}

#[test]
fn offset_compound_loaded_and_runtime_generations_keep_every_child_and_mass_frame() {
    for count in [2, 4, 16] {
        for fixed in [false, true] {
            let xml = document(count, fixed, "");
            let sdf = parsed(&xml);
            let mut sim = load(&xml);
            let source = id(&sim, "w::source");
            let original = native(&sim, body(&sim, source));
            let alias = sim
                .world()
                .get::<VisualGeometry>(entity(&sim, "w::source::body::view"))
                .unwrap()
                .0
                .clone();
            let template = prepare_model_template(&sim, &sdf, &offset(&sdf, COM, 37.)).unwrap();
            let mut next_template = template;
            for generation in 0..3 {
                let p = [100. + generation as f64 * 100., 20., 30.];
                let next = next_entity_id(sim.world()).unwrap();
                let spawned = spawn_model_template(&mut sim, &next_template, cycle(p)).unwrap();
                assert_eq!(spawned, next);
                assert_eq!(
                    next_entity_id(sim.world()).unwrap(),
                    next + count as u64 + 3
                );
                geometry(&sim, spawned, count, p, 37.);
                assert_eq!(
                    sim.world()
                        .resource::<PhysicsBackend>()
                        .is_dynamic(body(&sim, spawned)),
                    Some(!fixed)
                );
                assert!(Arc::ptr_eq(
                    &alias,
                    &sim.world()
                        .get::<VisualGeometry>(entity_by_id(sim.world(), spawned + 3).unwrap())
                        .unwrap()
                        .0
                ));
                let observed = model_state(&sim, spawned).unwrap();
                vector(observed.translation, p);
                vector(native(&sim, body(&sim, spawned)).position, p);
                let supplied = requested(spawned, count, fixed, 37.);
                // A fresh original may copy a different COM, including outside the union.
                let c = if generation == 0 { [80., -3., 2.] } else { COM };
                next_template =
                    prepare_model_template(&sim, &supplied, &offset(&supplied, c, 37.)).unwrap();
            }
            assert_eq!(native(&sim, body(&sim, source)), original);
        }
    }
}

#[test]
fn offset_compound_binary_bounds_nearest_and_visual_origin_ignore_interior_or_exterior_com() {
    let xml = document(2, true, "");
    let mut sdf = parsed(&xml);
    let mut sim = load(&xml);
    // Exact sphere arithmetic is kept separate from mixed rotated curved surfaces.
    sdf.worlds[0].models[0].links[0].collisions[0]
        .pose
        .transform
        .rotation = [0., 0., 0., 1.];
    let template = prepare_model_template(&sim, &sdf, &offset(&sdf, [80., -3., 2.], 1.)).unwrap();
    let a =
        spawn_model_template(&mut sim, &template, pose([20., 0., 10.], [0., 0., 0., 1.])).unwrap();
    let b =
        spawn_model_template(&mut sim, &template, pose([20., 0., 12.], [0., 0., 0., 1.])).unwrap();
    assert_eq!(
        ray(&sim, [20., 0., 13.5], [0., 0., -1.], 1., None),
        Some(1.)
    );
    assert_eq!(ray(&sim, [20., 0., 13.5], [0., 0., -1.], 0.999, None), None);
    assert_eq!(
        ray(
            &sim,
            [20., 0., 13.5],
            [0., 0., -1.],
            5.,
            Some(body(&sim, b))
        ),
        Some(3.)
    );
    assert_eq!(ray(&sim, [20., 0., 12.], [1., 0., 0.], 0.1, None), Some(0.));
    let visual = sim
        .world()
        .get::<WorldPose>(entity_by_id(sim.world(), a + 3).unwrap())
        .unwrap();
    vector(visual.translation, [20.25, -0.5, 10.75]);
    let (visual_id, projected) =
        gz_sim_scene::model_visual_pose(&sim, a, cycle([30., 40., 50.])).unwrap();
    assert_eq!(visual_id, a + 3);
    vector(projected.translation, [30.75, 40.25, 49.5]);
}

#[test]
fn offset_compound_six_axis_edits_and_paused_inputs_round_trip_origin_and_native_com() {
    let xml = document(4, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let template = prepare_model_template(&sim, &sdf, &offset(&sdf, COM, 1.)).unwrap();
    let model = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
    let target = ModelState {
        translation: [20., 30., 40.],
        rotation: [0.5; 4],
        linear_velocity: [1., -2., 3.],
        angular_velocity: [-0.25, 0.5, 0.75],
    };
    assert_eq!(set_model_state(&mut sim, model, target).unwrap(), target);
    let arm = [0.75, 0.25, -0.5];
    vector(
        native(&sim, body(&sim, model)).velocity,
        add(target.linear_velocity, cross(target.angular_velocity, arm)),
    );
    assert_eq!(model_state(&sim, model).unwrap(), target);
    queue_link_world_velocity(&mut sim, model + 1, [-1., 2., -3.], [0., 0., 2.]).unwrap();
    sim.set_paused(true);
    let before = snapshot(&sim);
    assert!(sim.tick().unwrap().dt.is_zero());
    assert_eq!(snapshot(&sim), before);
    sim.set_paused(false);
    sim.step_exact(1).unwrap();
    let actual = native(&sim, body(&sim, model));
    vector(
        actual.velocity,
        add([-1., 2., -3.], cross([0., 0., 2.], arm)),
    );
    vector(actual.angular_velocity, [0., 0., 2.]);
    let state = model_state(&sim, model).unwrap();
    vector(
        add(
            state.linear_velocity,
            cross(state.angular_velocity, rotate(state.rotation, COM)),
        ),
        actual.velocity,
    );
    for i in 0..4 {
        let local = sim
            .world()
            .get::<LocalPose>(entity_by_id(sim.world(), child_id(model, i)).unwrap())
            .unwrap()
            .0;
        let world = sim
            .world()
            .get::<WorldPose>(entity_by_id(sim.world(), child_id(model, i)).unwrap())
            .unwrap();
        vector(
            world.translation,
            add(state.translation, rotate(state.rotation, local.translation)),
        );
        let origin = add(world.translation, rotate(world.rotation, [0.3, 4., 0.]));
        let direction = rotate(world.rotation, [0., -1., 0.]);
        // Off-axis sphere/cylinder/capsule chord .4 and box Y face .25.
        near(
            ray(&sim, origin, direction, 5., None).unwrap(),
            if i == 2 { 3.75 } else { 3.6 },
        );
        assert_eq!(
            ray(&sim, origin, direction, 5., Some(body(&sim, model))),
            None
        );
    }
}

#[test]
fn offset_compound_explicit_inertia_density_and_centered_twins_use_measured_com_displacement() {
    let xml = document(4, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let mut models = Vec::new();
    for (i, c, d) in [(0, COM, 1.), (1, COM, 1e6), (2, [0.; 3], 91.)] {
        let t = prepare_model_template(&sim, &sdf, &offset(&sdf, c, d)).unwrap();
        models.push(
            spawn_model_template(&mut sim, &t, cycle([10. + 100. * i as f64, 20., 30.])).unwrap(),
        );
    }
    for axis in 0..4 {
        let mut start = Vec::new();
        for (i, model) in models.iter().copied().enumerate() {
            set_model_state(
                &mut sim,
                model,
                ModelState {
                    translation: [10. + 100. * i as f64, 20., 30.],
                    rotation: [0.5; 4],
                    linear_velocity: [0.; 3],
                    angular_velocity: [0.; 3],
                },
            )
            .unwrap();
            start.push(center(
                native(&sim, body(&sim, model)),
                if i == 2 { [0.; 3] } else { COM },
            ));
        }
        let force = if axis == 3 { [2., -4., 6.] } else { [0.; 3] };
        let mut torque = [0.; 3];
        if axis < 3 {
            torque[axis] = [6., 8., 4.][axis];
        }
        for model in &models {
            queue_link_world_wrench(&mut sim, *model + 1, force, torque).unwrap();
        }
        sim.set_paused(true);
        let before = snapshot(&sim);
        assert!(sim.tick().unwrap().dt.is_zero());
        assert_eq!(snapshot(&sim), before);
        sim.set_paused(false);
        sim.step_exact(1).unwrap();
        let reference = native(&sim, body(&sim, models[2]));
        let ref_delta = std::array::from_fn(|i| reference.position[i] - start[2][i]);
        for (i, model) in models.iter().copied().enumerate() {
            let n = native(&sim, body(&sim, model));
            vector(n.velocity, force.map(|v| v * 0.001 / 2.));
            vector(n.angular_velocity, torque.map(|v| v * 0.001 / 2.));
            vector(n.velocity, reference.velocity);
            vector(n.angular_velocity, reference.angular_velocity);
            let actual = center(n, if i == 2 { [0.; 3] } else { COM });
            vector(std::array::from_fn(|j| actual[j] - start[i][j]), ref_delta);
            let logical = model_state(&sim, model).unwrap();
            vector(
                add(
                    logical.linear_velocity,
                    cross(
                        logical.angular_velocity,
                        rotate(logical.rotation, if i == 2 { [0.; 3] } else { COM }),
                    ),
                ),
                n.velocity,
            );
        }
        let v = reference.velocity;
        sim.step_exact(1).unwrap();
        vector(native(&sim, body(&sim, models[2])).velocity, v);
    }
}

#[test]
fn offset_compound_lifetimes_retemplate_after_retirement_and_restore_both_reset_boundaries() {
    for fixed in [false, true] {
        let xml = document(4, fixed, "");
        let sdf = parsed(&xml);
        let mut sim = load(&xml);
        let loaded = identities(&sim);
        let bindings = loaded_bindings(&sim);
        let original = snapshot(&sim).bodies;
        let t = prepare_model_template(&sim, &sdf, &offset(&sdf, COM, 17.)).unwrap();
        let first = spawn_model_template(&mut sim, &t, cycle([100., 20., 30.])).unwrap();
        let request = requested(first, 16, fixed, 17.);
        let next_template =
            prepare_model_template(&sim, &request, &offset(&request, [80., -3., 2.], 17.)).unwrap();
        let second =
            spawn_model_template(&mut sim, &next_template, cycle([200., 20., 30.])).unwrap();
        let request = requested(second, 2, fixed, 17.);
        let detached = prepare_model_template(&sim, &request, &offset(&request, COM, 17.)).unwrap();
        let initial = model_state(&sim, second).unwrap();
        let handle = body(&sim, second);
        let alias = sim
            .world()
            .get::<VisualGeometry>(entity_by_id(sim.world(), second + 3).unwrap())
            .unwrap()
            .0
            .clone();
        set_model_state(
            &mut sim,
            second,
            ModelState {
                translation: [200., 40., 60.],
                rotation: [0., 0., 1., 0.],
                linear_velocity: [0.; 3],
                angular_velocity: [0.; 3],
            },
        )
        .unwrap();
        sim.reset().unwrap();
        assert_eq!(model_state(&sim, second).unwrap(), initial);
        assert_eq!(body(&sim, second), handle);
        geometry(&sim, second, 16, [200., 20., 30.], 17.);
        remove_model(&mut sim, first).unwrap();
        let source = id(&sim, "w::source");
        remove_model(&mut sim, source).unwrap();
        let next = next_entity_id(sim.world()).unwrap();
        let third = spawn_model_template(&mut sim, &detached, cycle([300., 20., 30.])).unwrap();
        assert_eq!(third, next);
        assert!(body(&sim, third).raw() > handle.raw());
        assert!(Arc::ptr_eq(
            &alias,
            &sim.world()
                .get::<VisualGeometry>(entity_by_id(sim.world(), third + 3).unwrap())
                .unwrap()
                .0
        ));
        let third_body = body(&sim, third);
        let high = next_entity_id(sim.world()).unwrap();
        let epoch = sim.epoch();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        // Loaded reset reconstructs Bevy entities. Public identities, hierarchy,
        // local frames and native bindings are restored onto fresh handles.
        assert_eq!(loaded_bindings(&sim), bindings);
        assert_eq!(
            identities(&sim).keys().collect::<Vec<_>>(),
            loaded.keys().collect::<Vec<_>>()
        );
        for entity in loaded.values() {
            assert!(sim.world().get_entity(*entity).is_err());
        }
        for runtime in [first, second, third] {
            assert!(entity_by_id(sim.world(), runtime).is_none());
        }
        for retired in [handle, third_body] {
            assert!(
                sim.world()
                    .resource::<PhysicsBackend>()
                    .state(retired)
                    .is_none()
            );
        }
        assert_eq!((sim.epoch(), sim.elapsed()), (epoch + 1, Duration::ZERO));
        assert!(next_entity_id(sim.world()).unwrap() >= high);
        for (raw, state) in original {
            assert_eq!(native(&sim, BodyId::from_raw(raw)), state);
        }
        let again = spawn_model_template(&mut sim, &detached, cycle([400., 20., 30.])).unwrap();
        assert!(again >= high);
        assert!(body(&sim, again).raw() > third_body.raw());
    }
}

#[test]
fn offset_compound_first_errors_native_boundaries_and_pending_repair_do_not_consume_ids() {
    use gz_physics::{
        CollisionSpec, CompoundBodySpec, DynamicBackend, Inertia, RotationSpec, Shape,
    };
    use gz_physics_rapier::RapierBackend;
    // Public native API is part of this Scene harness in BOTH profiles. Ordinary
    // geometry/body/inertia pass earlier guards before c^2 overflows.
    let children = [CollisionSpec {
        shape: Shape::Sphere { radius: 0.5 },
        position: [0.; 3],
        orientation: [0., 0., 0., 1.],
    }; 2];
    for mass in [None, Some(2.)] {
        let mut b = RapierBackend::new([0.; 3]).unwrap();
        let spec = CompoundBodySpec {
            collisions: &children,
            position: [10., 20., 30.],
            velocity: [0.; 3],
            mass,
        };
        let inert = Inertia {
            center: COM,
            orientation: [0., 0., 0., 1.],
            tensor: [2., 2., 2., 0., 0., 0.],
        };
        let base = b
            .spawn_compound_link(spec, RotationSpec::default(), Some(inert))
            .unwrap();
        let state = b.state(base).unwrap();
        if mass.is_some() {
            b.add_world_wrench(base, [2., 0., 0.], [0.; 3]).unwrap();
        }
        assert_eq!(
            b.spawn_compound_link(
                spec,
                RotationSpec::default(),
                Some(Inertia {
                    orientation: [0.5; 4],
                    ..inert
                })
            ),
            Err(PhysicsError::Unsupported {
                engine: "rapier",
                capability: "primitive compound offset inertia orientation"
            })
        );
        assert_eq!(b.state(base).unwrap(), state);
        for c in [[1e200, 0., 0.], [f64::NAN, 0., 0.]] {
            assert_eq!(
                b.spawn_compound_link(
                    spec,
                    RotationSpec::default(),
                    Some(Inertia { center: c, ..inert })
                ),
                Err(PhysicsError::InvalidValue)
            );
            assert_eq!(b.state(base).unwrap(), state);
        }
        let next = b
            .spawn_compound_link(
                CompoundBodySpec {
                    position: [100., 20., 30.],
                    ..spec
                },
                RotationSpec::default(),
                Some(inert),
            )
            .unwrap();
        assert_eq!(next.raw(), base.raw() + 1);
        let centered = b
            .spawn_compound_link(
                CompoundBodySpec {
                    position: [200., 20., 30.],
                    ..spec
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [0.; 3],
                    orientation: [0.5; 4],
                    ..inert
                }),
            )
            .unwrap();
        assert_eq!(centered.raw(), next.raw() + 1);
        vector(b.state(centered).unwrap().position, [200., 20., 30.]);
        b.step(Duration::from_millis(1)).unwrap();
        near(
            b.state(base).unwrap().velocity[0],
            if mass.is_some() { 0.001 } else { 0. },
        );
    }
    let xml = document(4, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let t = prepare_model_template(&sim, &sdf, &offset(&sdf, COM, 1.)).unwrap();
    let pending = spawn_model_template(&mut sim, &t, cycle([100., 20., 30.])).unwrap();
    queue_link_world_wrench(&mut sim, pending + 1, [2., 0., 0.], [0.; 3]).unwrap();
    let before = snapshot(&sim);
    for row in 0..9 {
        let mut m = offset(&sdf, COM, 1.);
        match row {
            0 => m.links[0].inertial.pose.transform.rotation = [0.5; 4],
            1 => m.links[0].pose.transform.translation = [0.1, 0., 0.],
            2 => m.links[0].collisions[0].density = Some(f64::NAN),
            3 => m.links[0].inertial.pose.transform.translation = [f64::NAN, 0., 0.],
            4 => m.links[0].collisions[0].pose.transform.translation = [f64::NAN, 0., 0.],
            5 => m.links[0].inertial.pose.transform.translation = [1e100, 0., 0.],
            6 => m.links[0].collisions.truncate(1),
            7 => {
                let source = m.links[0].collisions[0].source.clone();
                m.links[0].collisions[0].geometry = Geometry::Mesh(sdformat::Mesh {
                    uri: "must-not-be-decoded.glb".into(),
                    scale: [1.; 3],
                    submesh: String::new(),
                    center_submesh: false,
                    optimization: Default::default(),
                    convex_decomposition: None,
                    source,
                });
            }
            _ => {
                let visual = m.links[0].visuals[0].clone();
                m.links[0].visuals.push(visual);
            }
        }
        let result = prepare_model_template(&sim, &sdf, &m);
        match row {
            2 => assert!(
                matches!(&result, Err(WorldEditError::InvalidState)),
                "row={row} {:?}",
                result.as_ref().err()
            ),
            3 | 4 => assert!(
                matches!(&result, Err(WorldEditError::InvalidScene)),
                "row={row} {:?}",
                result.as_ref().err()
            ),
            _ => assert!(
                matches!(&result, Err(WorldEditError::UnsupportedModel)),
                "row={row} {:?}",
                result.as_ref().err()
            ),
        }
        assert_eq!(snapshot(&sim), before);
    }
    for row in 0..2 {
        let mut m = offset(&sdf, COM, 1.);
        if row == 0 {
            m.links[0].inertial.mass = 0.
        } else {
            m.links[0].inertial.inertia[0] = -1.;
        }
        let invalid = prepare_model_template(&sim, &sdf, &m).unwrap();
        assert!(matches!(
            spawn_model_template(&mut sim, &invalid, cycle([200., 20., 30.])),
            Err(WorldEditError::Physics(PhysicsError::InvalidValue))
        ));
        assert_eq!(snapshot(&sim), before);
    }
    let mut foreign = load(&xml);
    let old = snapshot(&foreign);
    assert!(matches!(
        spawn_model_template(&mut foreign, &t, cycle([1., 2., 3.])),
        Err(WorldEditError::InvalidScene)
    ));
    assert_eq!(snapshot(&foreign), old);
    let singleton_xml = document(1, false, "");
    let singleton_sdf = parsed(&singleton_xml);
    let singleton = load(&singleton_xml);
    let mut widened = offset(&singleton_sdf, COM, 1.);
    let child = widened.links[0].collisions[0].clone();
    widened.links[0].collisions.push(child);
    assert!(matches!(
        prepare_model_template(&singleton, &singleton_sdf, &widened),
        Err(WorldEditError::UnsupportedModel)
    ));
    let loaded_offset = xml.replacen("<inertial>", "<inertial><pose>0.25 0 0 0 0 0</pose>", 1);
    assert_ne!(loaded_offset, xml);
    // M104 admits this exact loaded passive compound. Geometry is unchanged;
    // its COM response is compared with the mature centered constructor.
    let mut loaded = load(&loaded_offset);
    let mut centered = load(&xml);
    assert_eq!(loaded_bindings(&loaded), loaded_bindings(&centered));
    for (id, entity) in identities(&loaded) {
        let peer = entity_by_id(centered.world(), id).unwrap();
        let frame = |sim: &Simulation, entity| {
            sim.world()
                .get::<WorldPose>(entity)
                .map(|pose| (pose.translation, pose.rotation))
        };
        assert_eq!(frame(&loaded, entity), frame(&centered, peer));
    }
    let loaded_id = id(&loaded, "w::source");
    let centered_id = id(&centered, "w::source");
    let loaded_body = body(&loaded, loaded_id);
    let centered_body = body(&centered, centered_id);
    let initial_native = native(&loaded, loaded_body);
    let initial_state = model_state(&loaded, loaded_id).unwrap();
    let initial_bindings = loaded_bindings(&loaded);
    let alias = loaded
        .world()
        .get::<VisualGeometry>(entity(&loaded, "w::source::body::view"))
        .unwrap()
        .0
        .clone();
    let initial_com = center(initial_native, [0.25, 0., 0.]);
    vector(initial_com, [-99.75, 0., 10.]);
    let centered_start = native(&centered, centered_body).position;
    for (sim, model) in [(&mut loaded, loaded_id), (&mut centered, centered_id)] {
        queue_link_world_wrench(sim, model + 1, [2., 0., 0.], [0., 0., 2.]).unwrap();
        sim.step_exact(1).unwrap();
    }
    let moving = native(&loaded, loaded_body);
    let reference = native(&centered, centered_body);
    vector(moving.velocity, [0.001, 0., 0.]);
    vector(moving.angular_velocity, [0., 0., 0.001]);
    vector(moving.velocity, reference.velocity);
    vector(moving.angular_velocity, reference.angular_velocity);
    let moved_com = center(moving, [0.25, 0., 0.]);
    vector(
        std::array::from_fn(|i| moved_com[i] - initial_com[i]),
        std::array::from_fn(|i| reference.position[i] - centered_start[i]),
    );
    let logical = model_state(&loaded, loaded_id).unwrap();
    vector(
        add(
            logical.linear_velocity,
            cross(
                logical.angular_velocity,
                rotate(logical.rotation, [0.25, 0., 0.]),
            ),
        ),
        moving.velocity,
    );
    for &child in loaded
        .world()
        .get::<Children>(entity_by_id(loaded.world(), loaded_id + 1).unwrap())
        .unwrap()
    {
        let local = loaded.world().get::<LocalPose>(child).unwrap().0;
        let world = loaded.world().get::<WorldPose>(child).unwrap();
        vector(
            world.translation,
            add(
                moving.position,
                rotate(moving.orientation, local.translation),
            ),
        );
        for axis in [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]] {
            vector(
                rotate(world.rotation, axis),
                rotate(moving.orientation, rotate(local.rotation, axis)),
            );
        }
    }
    for full in [false, true] {
        if full {
            reset_loaded_world(&mut loaded, |_| ()).unwrap();
        } else {
            loaded.reset().unwrap();
        }
        assert_eq!(model_state(&loaded, loaded_id).unwrap(), initial_state);
        assert_eq!(native(&loaded, body(&loaded, loaded_id)), initial_native);
        assert_eq!(loaded_bindings(&loaded), initial_bindings);
        assert!(Arc::ptr_eq(
            &alias,
            &loaded
                .world()
                .get::<VisualGeometry>(entity(&loaded, "w::source::body::view"))
                .unwrap()
                .0
        ));
    }
    let tpe_xml = "<sdf version='1.12'><world name='w'><model name='source'><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='view'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>";
    let tpe_sdf = parsed(tpe_xml);
    let tpe =
        load_world_with_sdf_step(tpe_xml, "tpe", "w", &NoResources, PhysicsEngine::Tpe).unwrap();
    let tpe_before = (
        identities(&tpe),
        next_entity_id(tpe.world()).unwrap(),
        tpe.epoch(),
        tpe.elapsed(),
    );
    // TPE has no native BodyHandle: model_state refuses before caller COM validation.
    let tpe_preparation = prepare_model_template(&tpe, &tpe_sdf, &offset(&tpe_sdf, COM, 1.));
    assert!(
        matches!(&tpe_preparation, Err(WorldEditError::InvalidScene)),
        "TPE native binding refusal: {:?}",
        tpe_preparation.as_ref().err()
    );
    assert_eq!(
        (
            identities(&tpe),
            next_entity_id(tpe.world()).unwrap(),
            tpe.epoch(),
            tpe.elapsed()
        ),
        tpe_before
    );
    let repaired = spawn_model_template(&mut sim, &t, cycle([200., 20., 30.])).unwrap();
    assert_eq!(repaired, before.next);
    assert_eq!(body(&sim, repaired).raw(), body(&sim, pending).raw() + 1);
    sim.step_exact(1).unwrap();
    vector(native(&sim, body(&sim, pending)).velocity, [0.001, 0., 0.]);
}

#[test]
fn offset_compound_owner_and_membership_corruption_stays_upstream_of_caller_parameters() {
    let xml = document(4, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let t = prepare_model_template(&sim, &sdf, &offset(&sdf, COM, 1.)).unwrap();
    let model = spawn_model_template(&mut sim, &t, cycle([100., 20., 30.])).unwrap();
    let link = entity_by_id(sim.world(), model + 1).unwrap();
    let child = entity_by_id(sim.world(), model + 4).unwrap();
    let model_entity = entity_by_id(sim.world(), model).unwrap();
    let supplied = requested(model, 4, false, 1.);
    let mut malformed = offset(&supplied, COM, 1.);
    malformed.links[0].inertial.pose.transform.rotation = [0.; 4];
    for row in 0..3 {
        queue_link_world_wrench(&mut sim, model + 1, [2., 0., 0.], [0.; 3]).unwrap();
        let local = *sim.world().get::<LocalPose>(child).unwrap();
        let handle = if row == 0 {
            sim.world_mut().entity_mut(link).take::<BodyHandle>()
        } else {
            None
        };
        if row == 1 {
            sim.world_mut()
                .entity_mut(child)
                .insert(ChildOf(model_entity));
        } else if row == 2 {
            sim.world_mut()
                .entity_mut(child)
                .get_mut::<LocalPose>()
                .unwrap()
                .0
                .translation[0] += 1.;
        }
        let before = snapshot(&sim);
        let result = prepare_model_template(&sim, &supplied, &malformed);
        assert!(
            matches!(result, Err(WorldEditError::UnsupportedModel)),
            "row={row}"
        );
        assert_eq!(snapshot(&sim), before);
        assert!(matches!(
            reset_loaded_world(&mut sim, |_| ()),
            Err(SceneResetError::RuntimePhysics)
        ));
        assert_eq!(snapshot(&sim), before);
        if let Some(handle) = handle {
            sim.world_mut().entity_mut(link).insert(handle);
        }
        if row == 1 {
            sim.world_mut().entity_mut(child).insert(ChildOf(link));
        }
        if row == 2 {
            sim.world_mut().entity_mut(child).insert(local);
        }
        prepare_model_template(&sim, &supplied, &offset(&supplied, COM, 1.)).unwrap();
        sim.step_exact(1).unwrap();
        near(
            native(&sim, body(&sim, model)).velocity[0],
            0.001 * (row + 1) as f64,
        );
    }
    // A controller on the ACTUAL source cannot be stripped in copied parameters.
    let controlled = xml.replacen(
        "<link name='body'>",
        &format!("{CONTROL}<link name='body'>"),
        1,
    );
    let controlled_sdf = parsed(&controlled);
    let owned = load(&controlled);
    let mut m = offset(&controlled_sdf, COM, 1.);
    m.plugins.clear();
    assert!(matches!(
        prepare_model_template(&owned, &controlled_sdf, &m),
        Err(WorldEditError::UnsupportedModel)
    ));
}

#[test]
fn offset_compound_contacts_and_camera_capacity_keep_real_geometry_and_retained_pending_inputs() {
    let falling = "<model name='ball'><pose>0 0 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy><izz>0.025</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model><model name='miss'><pose>30 0 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy><izz>0.025</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";
    let xml = document(2, true, falling).replacen(
        "<gravity>0 0 0</gravity>",
        "<gravity>0 0 -9.8</gravity>",
        1,
    );
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let mut request = offset(&sdf, [80., -3., 2.], 17.);
    request.links[0].collisions[0].geometry = Geometry::Box { size: [2., 2., 1.] };
    request.links[0].collisions[0].pose.transform.rotation = [0., 0., 0., 1.];
    request.links[0].collisions[1].pose.transform.translation = [10., 0., 0.];
    let t = prepare_model_template(&sim, &sdf, &request).unwrap();
    let support = spawn_model_template(&mut sim, &t, pose([0., 0., 1.], [0., 0., 0., 1.])).unwrap();
    let still = model_state(&sim, support).unwrap();
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
    near(
        ray(&sim, [0., 0., 4.], [0., 0., -1.], 5., Some(ball)).unwrap(),
        2.5,
    );
    for reset in [false, true] {
        if reset {
            sim.reset().unwrap();
        }
        sim.step_exact(1500).unwrap();
        assert!((native(&sim, ball).position[2] - 1.75).abs() < 0.01);
        assert!(native(&sim, miss).position[2] < 0.);
        assert_eq!(model_state(&sim, support).unwrap(), still);
    }
    let camera = "<model name='observer'><static>true</static><pose>-400 0 10 0 0 0</pose><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><sensor name='logical' type='logical_camera'><update_rate>0</update_rate><logical_camera><near>0.1</near><far>1000</far><horizontal_fov>1.5</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor></link></model>";
    let xml = document(2, false, camera);
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let t = prepare_model_template(&sim, &sdf, &offset(&sdf, COM, 1.)).unwrap();
    let first = spawn_model_template(&mut sim, &t, cycle([100., 20., 30.])).unwrap();
    queue_link_world_velocity(&mut sim, first + 1, [1., 0., 0.], [0.; 3]).unwrap();
    queue_link_world_wrench(&mut sim, first + 1, [4., 0., 0.], [0.; 3]).unwrap();
    let mut clones = Vec::new();
    for i in 0..124 {
        clones.push(
            spawn_model_template(&mut sim, &t, cycle([1000. + i as f64 * 8., 20., 30.])).unwrap(),
        );
    }
    let before = snapshot(&sim);
    assert!(matches!(
        spawn_model_template(&mut sim, &t, cycle([200., 20., 30.])),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim), before);
    let last = body(&sim, *clones.last().unwrap());
    remove_model(&mut sim, clones[0]).unwrap();
    let next = next_entity_id(sim.world()).unwrap();
    let replacement = spawn_model_template(&mut sim, &t, cycle([200., 20., 30.])).unwrap();
    assert_eq!(replacement, next);
    assert_eq!(body(&sim, replacement).raw(), last.raw() + 1);
    sim.step_exact(1).unwrap();
    near(native(&sim, body(&sim, first)).velocity[0], 1.002);
}
