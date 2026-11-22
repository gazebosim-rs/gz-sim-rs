// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! File-backed triangle Mesh copies retain the validated runtime source and four-node lifetime.
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, BodyState, PhysicsBackend, PhysicsError};
use gz_sim_core::{EntityId, Simulation, entity_by_id, next_entity_id};
use gz_sim_scene::{
    BodyHandle, CollisionDensity, EntityKind, LocalName, LocalPose, ModelState, Name,
    PendingResetRemoval, PhysicsEngine, VisualGeometry, WorldEditError, WorldPose,
    load_world_with_sdf_step, model_state, prepare_model_template, queue_link_world_velocity,
    queue_link_world_wrench, remove_model, reset_loaded_world, set_model_state,
    spawn_model_template,
};
use sdformat::{Geometry, NoResources, Sdf};
use std::{collections::BTreeMap, path::PathBuf, sync::Arc, time::Duration};

const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial>";
const SHAPES: [&str; 4] = [
    "<sphere><radius>0.5</radius></sphere>",
    "<cylinder><radius>0.5</radius><length>2</length></cylinder>",
    "<box><size>1.5 0.5 1</size></box>",
    "<capsule><radius>0.5</radius><length>2</length></capsule>",
];
const LOCAL: &str = "2 0 0 0 1.5707963267948966 0";
const CONTROL: &str =
    "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/>";
fn document(count: usize, fixed: bool, extra: &str) -> String {
    let collisions = (0..count).map(|i| format!("<collision name='c{i}'><pose>{} 0 0 0 1.5707963267948966 0</pose><density>17</density><geometry>{}</geometry></collision>", 4*i, SHAPES[i%4])).collect::<String>();
    format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='source'><pose>-100 0 10 0 0 0</pose><static>{fixed}</static><link name='body'>{INERTIA}{collisions}<visual name='view'><pose>0.25 -0.5 0.75 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual></link></model><model name='survivor'><pose>-300 0 10 0 0 0</pose><static>true</static><link name='body'><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision><visual name='view'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model>{extra}</world></sdf>"
    )
}
fn parsed(xml: &str) -> Sdf {
    sdformat::load_str(xml, "compound-to-mesh", &NoResources).unwrap()
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "compound-to-mesh",
        "w",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn pose(translation: [f64; 3], rotation: [f64; 4]) -> WorldPose {
    WorldPose {
        translation,
        rotation,
    }
}
fn cycle(p: [f64; 3]) -> WorldPose {
    pose(p, [0.5; 4])
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
fn identities(sim: &Simulation) -> Vec<(u64, Entity)> {
    let mut pairs: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(|v| (v.get(), e.id())))
        .collect();
    pairs.sort_unstable();
    pairs
}
#[derive(Debug, PartialEq)]
struct Atomic {
    ids: Vec<(u64, Entity)>,
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
fn ray(
    sim: &Simulation,
    o: [f64; 3],
    d: [f64; 3],
    maximum: f64,
    exclude: Option<BodyId>,
) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(o, d, maximum, exclude)
        .unwrap()
}
fn runtime_source(sim: &mut Simulation, sdf: &Sdf) -> u64 {
    let template = prepare_model_template(sim, sdf, &sdf.worlds[0].models[0]).unwrap();
    spawn_model_template(sim, &template, cycle([100., 20., 30.])).unwrap()
}

fn asset_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../examples/meshes")
        .canonicalize()
        .unwrap()
}
fn mesh_request(source: u64, local: &str, fixed: bool, density: f64, uri: &str) -> Sdf {
    let xml = format!(
        "<sdf version='1.12'><world name='w'><model name='runtime_{source}'><static>{fixed}</static><link name='body'>{INERTIA}<collision name='collision'><pose>{local}</pose><density>{density}</density><geometry><mesh><uri>{uri}</uri><scale>2 1 0.5</scale></mesh></geometry></collision><visual name='visual'><pose>0.25 -0.5 0.75 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>"
    );
    // Both the DOM graph's generated scope and Mesh source URI are genuine.
    sdformat::load_str(
        &xml,
        &format!("file://{}", asset_root().join("request.sdf").display()),
        &NoResources,
    )
    .unwrap()
}
fn check_mesh(sim: &Simulation, model: u64, p: [f64; 3], fixed: bool, density: f64) {
    let root = entity_by_id(sim.world(), model).unwrap();
    let link = entity_by_id(sim.world(), model + 1).unwrap();
    let collision = entity_by_id(sim.world(), model + 2).unwrap();
    let visual = entity_by_id(sim.world(), model + 3).unwrap();
    assert_eq!(sim.world().get::<Children>(root).unwrap().len(), 1);
    assert_eq!(sim.world().get::<Children>(link).unwrap().len(), 2);
    for (node, kind, parent, name) in [
        (link, EntityKind::Link, root, "body"),
        (collision, EntityKind::Collision, link, "collision"),
        (visual, EntityKind::Visual, link, "visual"),
    ] {
        assert_eq!(sim.world().get::<EntityKind>(node), Some(&kind));
        assert_eq!(sim.world().get::<ChildOf>(node).unwrap().parent(), parent);
        assert_eq!(sim.world().get::<LocalName>(node).unwrap().0, name);
    }
    assert!(sim.world().get::<BodyHandle>(collision).is_none());
    assert!(sim.world().get::<BodyHandle>(visual).is_none());
    assert_eq!(
        sim.world().get::<CollisionDensity>(collision).unwrap().0,
        density
    );
    vector(
        sim.world()
            .get::<LocalPose>(collision)
            .unwrap()
            .0
            .translation,
        [2., 0., 0.],
    );
    vector(sim.world().get::<WorldPose>(link).unwrap().translation, p);
    vector(native(sim, body(sim, model)).position, p);
    let center = [p[0], p[1] + 2., p[2]];
    let frame = sim.world().get::<WorldPose>(collision).unwrap();
    vector(frame.translation, center);
    let h = std::f64::consts::FRAC_1_SQRT_2;
    for (a, b) in frame.rotation.into_iter().zip([0., h, h, 0.]) {
        near(a, b);
    }
    vector(
        sim.world().get::<WorldPose>(visual).unwrap().translation,
        [p[0] + 0.75, p[1] + 0.25, p[2] - 0.5],
    );
    // Existing OBJ/STL tetra vertices after scale: (0,0,0),(0,2,0),(2,0,0),(0,0,1.5).
    // cycle*Ry90 maps (x,y,z) to(-x,+z,+y). At x=0.25,y=0.5,
    // its sloping face has z=1.5*(1-0.25/2-0.5/2)=0.9375.
    // All loaded/source bodies are at X=-300/-100/100, whereas query clones
    // use separated X>=200 lanes. The entire ray segment is isolated globally.
    let origin = [center[0] - 0.25, center[1] + 4., center[2] + 0.5];
    near(ray(sim, origin, [0., -1., 0.], 5., None).unwrap(), 3.0625);
    assert_eq!(ray(sim, origin, [0., -1., 0.], 3., None), None);
    assert_eq!(
        ray(sim, origin, [0., -1., 0.], 5., Some(body(sim, model))),
        None
    );
    // This crosses the tetra's AABB at x=y=1.5 but no triangle: x/2+y/2>1.
    assert_eq!(
        ray(
            sim,
            [center[0] - 1.5, center[1] + 4., center[2] + 1.5],
            [0., -1., 0.],
            5.,
            None
        ),
        None
    );
    // Static Mesh is a triangle surface: an interior ray exits at a triangle.
    // The unchanged dynamic path constructs a closed convex solid instead.
    let inside = ray(
        sim,
        [center[0] - 0.25, center[1] + 0.1, center[2] + 0.5],
        [0., 1., 0.],
        2.,
        None,
    )
    .unwrap();
    if fixed {
        near(inside, 0.8375);
    } else {
        assert_eq!(inside, 0.);
    }
}

#[test]
fn runtime_mesh_copies_preserve_surface_frames_four_ids_and_visual_aliases() {
    for count in [2, 4, 16] {
        for fixed in [false, true] {
            for uri in ["m16-asymmetric.obj", "m14-asymmetric-ascii.stl"] {
                let xml = document(count, fixed, "");
                let sdf = parsed(&xml);
                let mut sim = load(&xml);
                let source = runtime_source(&mut sim, &sdf);
                let supplied = mesh_request(source, LOCAL, fixed, 23., uri);
                let before = snapshot(&sim);
                let template =
                    prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
                assert_eq!(snapshot(&sim), before);
                assert_eq!(template.source_visual_id(), source + 3);
                let alias = sim
                    .world()
                    .get::<VisualGeometry>(entity_by_id(sim.world(), source + 3).unwrap())
                    .unwrap()
                    .0
                    .clone();
                let clone =
                    spawn_model_template(&mut sim, &template, cycle([200., 20., 30.])).unwrap();
                assert_eq!(clone, before.next);
                assert_eq!(next_entity_id(sim.world()).unwrap(), clone + 4);
                assert_eq!(identities(&sim).len(), before.ids.len() + 4);
                assert_eq!(snapshot(&sim).bodies.len(), before.bodies.len() + 1);
                assert_eq!(body(&sim, clone).raw(), body(&sim, source).raw() + 1);
                for (raw, state) in before.bodies {
                    assert_eq!(snapshot(&sim).bodies[&raw], state);
                }
                assert!(Arc::ptr_eq(
                    &alias,
                    &sim.world()
                        .get::<VisualGeometry>(entity_by_id(sim.world(), clone + 3).unwrap())
                        .unwrap()
                        .0
                ));
                check_mesh(&sim, clone, [200., 20., 30.], fixed, 23.);
            }
        }
    }
}

#[test]
fn mesh_queries_choose_global_nearest_body_and_keep_triangle_surface_domains() {
    let xml = document(16, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let source = runtime_source(&mut sim, &sdf);
    let supplied = mesh_request(source, "0 0 0 0 0 0", true, 1., "m16-asymmetric.obj");
    let template = prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
    let a =
        spawn_model_template(&mut sim, &template, pose([0., 0., 10.], [0., 0., 0., 1.])).unwrap();
    let b =
        spawn_model_template(&mut sim, &template, pose([0., 0., 14.], [0., 0., 0., 1.])).unwrap();
    let origin = [0.25, 0.5, 20.];
    near(ray(&sim, origin, [0., 0., -1.], 12., None).unwrap(), 5.0625);
    near(
        ray(&sim, origin, [0., 0., -1.], 12., Some(body(&sim, b))).unwrap(),
        9.0625,
    );
    assert_eq!(ray(&sim, origin, [0., 0., -1.], 5., None), None);
    // All other bodies are in disjoint X lanes, including the16-child source.
    assert_eq!(
        ray(
            &sim,
            [0.25, 0.5, 12.],
            [0., 0., -1.],
            4.,
            Some(body(&sim, a))
        ),
        None
    );
    near(
        ray(&sim, [0.25, 0.5, 10.1], [0., 0., 1.], 2., None).unwrap(),
        0.8375,
    );
    assert_eq!(ray(&sim, [1.5, 1.5, 20.], [0., 0., -1.], 12., None), None);
    assert!(matches!(
        sim.world()
            .resource::<PhysicsBackend>()
            .cast_ray([0.; 3], [0.; 3], 1., None),
        Err(PhysicsError::InvalidValue)
    ));
    assert!(matches!(
        sim.world().resource::<PhysicsBackend>().cast_ray(
            [f64::NAN, 0., 0.],
            [0., 0., -1.],
            1.,
            None
        ),
        Err(PhysicsError::InvalidValue)
    ));
}

#[test]
fn mesh_singletons_keep_six_axis_edits_centered_explicit_inertia_and_density_twins() {
    let xml = document(4, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let source = runtime_source(&mut sim, &sdf);
    let original = native(&sim, body(&sim, source));
    let left = mesh_request(source, LOCAL, false, 1., "m16-asymmetric.obj");
    let right = mesh_request(source, LOCAL, false, 1e6, "m16-asymmetric.obj");
    let a = prepare_model_template(&sim, &left, &left.worlds[0].models[0]).unwrap();
    let b = prepare_model_template(&sim, &right, &right.worlds[0].models[0]).unwrap();
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
    check_mesh(&sim, x, target.translation, false, 1.);
    let states = [
        ModelState {
            linear_velocity: [0.; 3],
            angular_velocity: [0.; 3],
            ..target
        },
        ModelState {
            translation: [500., 50., 60.],
            linear_velocity: [0.; 3],
            angular_velocity: [0.; 3],
            ..target
        },
    ];
    for axis in 0..3 {
        for (model, state) in [x, y].into_iter().zip(states) {
            set_model_state(&mut sim, model, state).unwrap();
        }
        let mut torque = [0.; 3];
        torque[axis] = [6., 8., 4.][axis];
        for model in [x, y] {
            queue_link_world_wrench(&mut sim, model + 1, [2., -4., 6.], torque).unwrap();
        }
        sim.set_paused(true);
        assert!(sim.tick().unwrap().dt.is_zero());
        assert_eq!(model_state(&sim, x).unwrap(), states[0]);
        sim.set_paused(false);
        sim.step_exact(1).unwrap();
        let l = native(&sim, body(&sim, x));
        let r = native(&sim, body(&sim, y));
        vector(l.velocity, [0.001, -0.002, 0.003]);
        // World-principal axes avoid nonprincipal finite-step gyroscopic coupling.
        let mut angular = [0.; 3];
        angular[axis] = torque[axis] * 0.001 / [4., 2., 3.][axis];
        vector(l.angular_velocity, angular);
        assert_eq!(l.velocity, r.velocity);
        assert_eq!(l.angular_velocity, r.angular_velocity);
        assert_eq!(l.orientation, r.orientation);
        vector(
            l.position,
            [r.position[0] - 100., r.position[1], r.position[2]],
        );
    }
    assert_eq!(native(&sim, body(&sim, source)), original);
}

#[test]
fn decoded_mesh_template_survives_source_retirement_and_both_reset_lifetimes() {
    for fixed in [false, true] {
        let xml = document(4, fixed, "");
        let sdf = parsed(&xml);
        let mut sim = load(&xml);
        let loaded = snapshot(&sim);
        let loaded_source = id(&sim, "w::source");
        let source = runtime_source(&mut sim, &sdf);
        let supplied = mesh_request(source, LOCAL, fixed, 7., "m16-asymmetric.obj");
        let template =
            prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
        let clone = spawn_model_template(&mut sim, &template, cycle([200., 20., 30.])).unwrap();
        let initial = model_state(&sim, clone).unwrap();
        let original_body = body(&sim, clone);
        let detached_dom = mesh_request(clone, LOCAL, fixed, 9., "m14-asymmetric-ascii.stl");
        let detached =
            prepare_model_template(&sim, &detached_dom, &detached_dom.worlds[0].models[0]).unwrap();
        let alias = sim
            .world()
            .get::<VisualGeometry>(entity_by_id(sim.world(), clone + 3).unwrap())
            .unwrap()
            .0
            .clone();
        set_model_state(
            &mut sim,
            clone,
            ModelState {
                translation: [200., 40., 60.],
                rotation: [0., 0., 1., 0.],
                linear_velocity: [0.; 3],
                angular_velocity: [0.; 3],
            },
        )
        .unwrap();
        sim.reset().unwrap();
        assert_eq!(model_state(&sim, clone).unwrap(), initial);
        assert_eq!(body(&sim, clone), original_body);
        check_mesh(&sim, clone, [200., 20., 30.], fixed, 7.);
        assert_eq!(
            remove_model(&mut sim, source).unwrap(),
            (source..source + 7).collect::<Vec<_>>()
        );
        remove_model(&mut sim, loaded_source).unwrap();
        let next = next_entity_id(sim.world()).unwrap();
        let another = spawn_model_template(&mut sim, &detached, cycle([300., 20., 30.])).unwrap();
        assert_eq!(another, next);
        assert_eq!(body(&sim, another).raw(), original_body.raw() + 1);
        assert!(Arc::ptr_eq(
            &alias,
            &sim.world()
                .get::<VisualGeometry>(entity_by_id(sim.world(), another + 3).unwrap())
                .unwrap()
                .0
        ));
        check_mesh(&sim, another, [300., 20., 30.], fixed, 9.);
        assert_eq!(
            remove_model(&mut sim, another).unwrap(),
            (another..another + 4).collect::<Vec<_>>()
        );
        let high = next_entity_id(sim.world()).unwrap();
        let native_high = body(&sim, clone).raw() + 1;
        reset_loaded_world(&mut sim, |observed| {
            for node in clone..clone + 4 {
                let e = entity_by_id(observed.world(), node).unwrap();
                assert!(observed.world().get::<PendingResetRemoval>(e).is_some());
                assert!(observed.world().get::<BodyHandle>(e).is_none());
            }
        })
        .unwrap();
        assert_eq!(next_entity_id(sim.world()).unwrap(), high);
        assert_eq!(
            identities(&sim).iter().map(|p| p.0).collect::<Vec<_>>(),
            loaded.ids.iter().map(|p| p.0).collect::<Vec<_>>()
        );
        assert_eq!(snapshot(&sim).bodies, loaded.bodies);
        assert!(entity_by_id(sim.world(), clone).is_none());
        let final_clone =
            spawn_model_template(&mut sim, &template, cycle([400., 20., 30.])).unwrap();
        assert_eq!(final_clone, high);
        assert!(body(&sim, final_clone).raw() > native_high);
        check_mesh(&sim, final_clone, [400., 20., 30.], fixed, 7.);
    }
}

#[test]
fn mesh_source_membership_errors_precede_caller_resolution_and_keep_pending_wrenches() {
    for row in 0..8 {
        let xml = document(4, false, "");
        let sdf = parsed(&xml);
        let mut sim = load(&xml);
        let source = runtime_source(&mut sim, &sdf);
        let supplied = mesh_request(source, LOCAL, false, 1., "m16-asymmetric.obj");
        prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
        let link = entity_by_id(sim.world(), source + 1).unwrap();
        let child = entity_by_id(sim.world(), source + 4).unwrap();
        queue_link_world_wrench(&mut sim, source + 1, [4., 0., 0.], [0.; 3]).unwrap();
        let local = *sim.world().get::<LocalPose>(child).unwrap();
        let world_pose = *sim.world().get::<WorldPose>(child).unwrap();
        let name = sim.world().get::<LocalName>(child).unwrap().0.clone();
        let density = *sim.world().get::<CollisionDensity>(child).unwrap();
        let foreign = entity(&sim, "w::survivor::body");
        match row {
            0 => {
                sim.world_mut().entity_mut(child).despawn();
            }
            1 => {
                sim.world_mut().entity_mut(child).insert(ChildOf(foreign));
            }
            2 => {
                sim.world_mut()
                    .entity_mut(child)
                    .insert(LocalName("forged".into()));
            }
            3 => {
                sim.world_mut()
                    .entity_mut(child)
                    .insert(LocalPose(pose([0.; 3], [0., 0., 0., 1.])));
            }
            4 => {
                sim.world_mut().entity_mut(child).insert(EntityKind::Sensor);
            }
            5 => {
                sim.world_mut().entity_mut(child).remove::<WorldPose>();
            }
            6 => {
                sim.world_mut()
                    .entity_mut(child)
                    .insert(CollisionDensity(42.));
            }
            _ => {
                let handle = sim
                    .world_mut()
                    .entity_mut(foreign)
                    .take::<BodyHandle>()
                    .unwrap();
                sim.world_mut().entity_mut(child).insert(handle);
            }
        }
        let before = snapshot(&sim);
        // Poisoned caller pose would independently be InvalidScene AFTER a valid
        // source. UnsupportedModel proves actual immutable membership wins first.
        let mut poisoned = supplied.worlds[0].models[0].clone();
        poisoned.links[0].collisions[0].pose.relative_to = "missing_caller_frame".into();
        assert!(
            matches!(
                prepare_model_template(&sim, &supplied, &poisoned),
                Err(WorldEditError::UnsupportedModel)
            ),
            "row{row}"
        );
        assert_eq!(snapshot(&sim), before);
        if row == 0 {
            continue;
        }
        sim.world_mut().entity_mut(child).insert((
            ChildOf(link),
            LocalName(name),
            local,
            world_pose,
            density,
            EntityKind::Collision,
        ));
        if row == 7 {
            let handle = sim
                .world_mut()
                .entity_mut(child)
                .take::<BodyHandle>()
                .unwrap();
            sim.world_mut().entity_mut(foreign).insert(handle);
        }
        assert!(matches!(
            prepare_model_template(&sim, &supplied, &poisoned),
            Err(WorldEditError::InvalidScene)
        ));
        prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
        sim.step_exact(1).unwrap();
        near(native(&sim, body(&sim, source)).velocity[0], 0.002);
    }
}

#[test]
fn mesh_request_first_errors_and_native_refusals_keep_both_allocators_and_pending_inputs() {
    let xml = document(2, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let source = runtime_source(&mut sim, &sdf);
    let supplied = mesh_request(source, LOCAL, false, 1., "m16-asymmetric.obj");
    let valid = prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
    queue_link_world_velocity(&mut sim, source + 1, [1., 2., 3.], [0.; 3]).unwrap();
    queue_link_world_wrench(&mut sim, source + 1, [4., 0., 0.], [0.; 3]).unwrap();
    let before = snapshot(&sim);
    for row in 0..20 {
        let mut model = supplied.worlds[0].models[0].clone();
        match row {
            0 => {
                if let Geometry::Mesh(mesh) = &mut model.links[0].collisions[0].geometry {
                    mesh.uri = "missing.glb".into();
                }
            }
            1 => {
                if let Geometry::Mesh(mesh) = &mut model.links[0].collisions[0].geometry {
                    mesh.scale = [0., 1., 1.];
                }
            }
            2 => {
                if let Geometry::Mesh(mesh) = &mut model.links[0].collisions[0].geometry {
                    mesh.submesh = "forbidden".into();
                }
            }
            3 => {
                if let Geometry::Mesh(mesh) = &mut model.links[0].collisions[0].geometry {
                    mesh.uri = "https://example.invalid/not-fetched.glb".into();
                }
            }
            4 => model.links[0].pose.transform.translation = [1., 0., 0.],
            5 => model.links[0].pose.relative_to = "missing_link_frame".into(),
            6 => model.links[0].inertial.pose.transform.translation = [1e-13, 0., 0.],
            7 => model.links[0].collisions[0].pose.relative_to = "missing_collision_frame".into(),
            8 => model.links[0].collisions[0].density = Some(f64::NAN),
            9 => model.links[0].collisions[0].pose.transform.translation = [f64::NAN, 0., 0.],
            10 => {
                let s = model.links[0].collisions[0].source.clone();
                model.links[0].collisions[0].surface.source = Some(s);
            }
            11 => model.links[0].visuals.clear(),
            12 => model.links[0].inertial.fluid_added_mass = Some(Box::new([[0.; 6]; 6])),
            13 => model.links[0].inertial.pose.transform.rotation = [0.; 4],
            14 => model.links[0].visuals[0].pose.relative_to = "missing_visual_frame".into(),
            15 => model.links[0].pose.transform.rotation = [0., 0., 1., 0.],
            16 => {
                model.links[0].collisions[0].geometry = Geometry::Plane {
                    normal: [0., 0., 1.],
                    size: [1.; 2],
                }
            }
            17 => {
                if let Geometry::Mesh(mesh) = &mut model.links[0].collisions[0].geometry {
                    mesh.source.uri = "memory-source".into();
                }
            }
            18 => {
                let child = model.links[0].collisions[0].clone();
                model.links[0].collisions = vec![child; 17];
            }
            _ => {
                model.plugins = parsed(&xml.replacen(
                    "<link name='body'>",
                    &format!("{CONTROL}<link name='body'>"),
                    1,
                ))
                .worlds[0]
                    .models[0]
                    .plugins
                    .clone();
            }
        }
        let result = prepare_model_template(&sim, &supplied, &model);
        let error = result.as_ref().err();
        match row {
            5 | 7 | 9 | 13 | 14 => assert!(
                matches!(&result, Err(WorldEditError::InvalidScene)),
                "row{row}: {error:?}"
            ),
            8 => assert!(
                matches!(&result, Err(WorldEditError::InvalidState)),
                "row{row}: {error:?}"
            ),
            _ => assert!(
                matches!(&result, Err(WorldEditError::UnsupportedModel)),
                "row{row}: {error:?}"
            ),
        }
        assert_eq!(snapshot(&sim), before, "row{row}");
    }
    for row in 0..2 {
        let mut model = supplied.worlds[0].models[0].clone();
        if row == 0 {
            model.links[0].inertial.mass = f64::NAN;
        } else {
            model.links[0].inertial.inertia[0] = -1.;
        }
        let bad = prepare_model_template(&sim, &supplied, &model).unwrap();
        assert!(matches!(
            spawn_model_template(&mut sim, &bad, cycle([200., 20., 30.])),
            Err(WorldEditError::Physics(PhysicsError::InvalidValue))
        ));
        assert_eq!(snapshot(&sim), before);
    }
    for bad in [
        pose([f64::NAN, 0., 0.], [0., 0., 0., 1.]),
        pose([0.; 3], [0.; 4]),
    ] {
        assert!(matches!(
            spawn_model_template(&mut sim, &valid, bad),
            Err(WorldEditError::InvalidState)
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
    let accepted = spawn_model_template(&mut sim, &valid, cycle([200., 20., 30.])).unwrap();
    assert_eq!(accepted, before.next);
    assert_eq!(body(&sim, accepted).raw(), body(&sim, source).raw() + 1);
    sim.step_exact(1).unwrap();
    vector(native(&sim, body(&sim, source)).velocity, [1.002, 2., 3.]);
    let tpe_xml = "<sdf version='1.12'><world name='w'><model name='source'><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='view'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>";
    let tpe_sdf = parsed(tpe_xml);
    let tpe =
        load_world_with_sdf_step(tpe_xml, "tpe", "w", &NoResources, PhysicsEngine::Tpe).unwrap();
    let mut requested = tpe_sdf.worlds[0].models[0].clone();
    requested.links[0].collisions[0].geometry = supplied.worlds[0].models[0].links[0].collisions[0]
        .geometry
        .clone();
    let ids = identities(&tpe);
    let next = next_entity_id(tpe.world()).unwrap();
    let clock = (tpe.epoch(), tpe.elapsed());
    let motions = || {
        tpe.world()
            .iter_entities()
            .filter_map(|e| {
                e.get::<gz_physics_tpe::Motion>()
                    .map(|v| (e.id(), *v.state()))
            })
            .collect::<BTreeMap<_, _>>()
    };
    let before_motion = motions();
    assert_eq!(before_motion.len(), 1);
    // TPE's actual link lacks BodyHandle: InvalidScene precedes Mesh IO and
    // engine/native checks. Observe real Motion records, not empty Rapier state.
    assert!(matches!(
        prepare_model_template(&tpe, &tpe_sdf, &requested),
        Err(WorldEditError::InvalidScene)
    ));
    assert_eq!(motions(), before_motion);
    assert_eq!(identities(&tpe), ids);
    assert_eq!(next_entity_id(tpe.world()).unwrap(), next);
    assert_eq!((tpe.epoch(), tpe.elapsed()), clock);
}

#[test]
fn mesh_copy_camera_capacity_keeps_native_ids_and_recovers_without_losing_inputs() {
    let camera = "<model name='observer'><static>true</static><pose>-400 0 10 0 0 0</pose><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><sensor name='logical' type='logical_camera'><update_rate>0</update_rate><logical_camera><near>0.1</near><far>1000</far><horizontal_fov>1.5</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor></link></model>";
    let xml = document(2, false, camera);
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let source = runtime_source(&mut sim, &sdf);
    let supplied = mesh_request(source, "0 0 0 0 0 0", false, 1., "m16-asymmetric.obj");
    let template = prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
    queue_link_world_velocity(&mut sim, source + 1, [1., 0., 0.], [0.; 3]).unwrap();
    queue_link_world_wrench(&mut sim, source + 1, [4., 0., 0.], [0.; 3]).unwrap();
    let mut clones = Vec::new();
    // Three loaded models plus runtime source plus124 singleton meshes =128.
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
    let last = body(&sim, *clones.last().unwrap());
    let before = snapshot(&sim);
    assert!(matches!(
        spawn_model_template(&mut sim, &template, cycle([200., 20., 30.])),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim), before);
    remove_model(&mut sim, clones[0]).unwrap();
    let next = next_entity_id(sim.world()).unwrap();
    let recovered = spawn_model_template(&mut sim, &template, cycle([200., 20., 30.])).unwrap();
    assert_eq!(recovered, next);
    assert_eq!(body(&sim, recovered).raw(), last.raw() + 1);
    assert_eq!(next_entity_id(sim.world()).unwrap(), recovered + 4);
    sim.step_exact(1).unwrap();
    near(native(&sim, body(&sim, source)).velocity[0], 1.002);
}

#[test]
fn static_mesh_contact_and_dynamic_fall_use_existing_geometry_with_clear_controls() {
    let extra = "<model name='ball'><pose>0.5 -0.5 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy><izz>0.025</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model><model name='miss'><pose>20 -0.5 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy><izz>0.025</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";
    let xml = document(2, true, extra).replacen(
        "<gravity>0 0 0</gravity>",
        "<gravity>0 0 -9.8</gravity>",
        1,
    );
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let source = runtime_source(&mut sim, &sdf);
    // Ry-independent exact X180 turns the tetra's bottom triangle into an upward
    // horizontal support at Z1; all other faces are below it. Footprint intercepts
    // are X4 and Y-4, so (0.5,-0.5) lies strictly inside, with positive initial gap.
    let supplied = mesh_request(
        source,
        "0 0 0 3.141592653589793 0 0",
        true,
        1.,
        "m16-asymmetric.obj",
    );
    let mut model = supplied.worlds[0].models[0].clone();
    model.links[0].collisions[0].pose.transform.rotation = [1., 0., 0., 0.];
    if let Geometry::Mesh(mesh) = &mut model.links[0].collisions[0].geometry {
        mesh.scale = [4., 2., 1.];
    }
    let template = prepare_model_template(&sim, &supplied, &model).unwrap();
    let floor =
        spawn_model_template(&mut sim, &template, pose([0., 0., 1.], [0., 0., 0., 1.])).unwrap();
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
    // A separate dynamic mesh is isolated far from every floor and source. Its
    // native mass exists and gravity must move it; no hull or density inference.
    let falling = mesh_request(source, "0 0 0 0 0 0", false, 1., "m16-asymmetric.obj");
    let fall_template =
        prepare_model_template(&sim, &falling, &falling.worlds[0].models[0]).unwrap();
    let free = spawn_model_template(
        &mut sim,
        &fall_template,
        pose([60., 0., 4.], [0., 0., 0., 1.]),
    )
    .unwrap();
    for reset in [false, true] {
        if reset {
            sim.reset().unwrap();
        }
        assert_eq!(native(&sim, ball).position, [0.5, -0.5, 4.]);
        sim.step_exact(100).unwrap();
        assert!(native(&sim, body(&sim, free)).position[2] < 3.99);
        sim.step_exact(1400).unwrap();
        assert!((native(&sim, ball).position[2] - 1.25).abs() < 0.01);
        assert!(native(&sim, miss).position[2] < 0.);
        assert_eq!(native(&sim, body(&sim, floor)), floor_state);
    }
}
