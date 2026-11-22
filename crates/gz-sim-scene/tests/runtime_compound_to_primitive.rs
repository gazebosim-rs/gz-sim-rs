// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Primitive copies from an intact owned runtime compound use the mature four-node path.
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
use std::{collections::BTreeMap, sync::Arc, time::Duration};

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
    sdformat::load_str(xml, "compound-to-primitive", &NoResources).unwrap()
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "compound-to-primitive",
        "w",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn requested(source: u64, shape: usize, local: &str, fixed: bool, density: f64) -> Sdf {
    parsed(&format!(
        "<sdf version='1.12'><world name='w'><model name='runtime_{source}'><static>{fixed}</static><link name='body'>{INERTIA}<collision name='collision'><pose>{local}</pose><density>{density}</density><geometry>{}</geometry></collision><visual name='visual'><pose>0.25 -0.5 0.75 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>",
        SHAPES[shape]
    ))
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
fn check_posed(sim: &Simulation, model: u64, shape: usize, p: [f64; 3], density: f64) {
    let root = entity_by_id(sim.world(), model).unwrap();
    let link = entity_by_id(sim.world(), model + 1).unwrap();
    let collision = entity_by_id(sim.world(), model + 2).unwrap();
    let visual = entity_by_id(sim.world(), model + 3).unwrap();
    assert_eq!(sim.world().get::<Children>(root).unwrap().len(), 1);
    assert_eq!(sim.world().get::<Children>(link).unwrap().len(), 2);
    for (node, kind, parent) in [
        (link, EntityKind::Link, root),
        (collision, EntityKind::Collision, link),
        (visual, EntityKind::Visual, link),
    ] {
        assert_eq!(sim.world().get::<EntityKind>(node), Some(&kind));
        assert_eq!(sim.world().get::<ChildOf>(node).unwrap().parent(), parent);
    }
    assert_eq!(
        sim.world().get::<LocalName>(collision).unwrap().0,
        "collision"
    );
    assert!(sim.world().get::<BodyHandle>(collision).is_none());
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
    // Independent quaternion product: cycle * Ry90 maps (X,Y,Z) to (-X,+Z,+Y).
    for (a, b) in frame.rotation.into_iter().zip([0., h, h, 0.]) {
        near(a, b);
    }
    vector(
        sim.world().get::<WorldPose>(visual).unwrap().translation,
        [p[0] + 0.75, p[1] + 0.25, p[2] - 0.5],
    );
    let origin = [center[0] + 0.3, center[1], center[2] + 4.];
    // Other bodies occupy X=-300/-100/100 or distinct clone X lanes. The entire
    // five-unit segment is isolated, not merely its desired child's surface.
    near(
        ray(sim, origin, [0., 0., -1.], 5., None).unwrap(),
        if shape == 2 { 3.75 } else { 3.6 },
    );
    assert_eq!(
        ray(sim, origin, [0., 0., -1.], 5., Some(body(sim, model))),
        None
    );
    assert_eq!(ray(sim, center, [0., 0., 1.], 0.125, None), Some(0.));
}

#[test]
fn runtime_compound_sources_copy_all_four_posed_primitives_with_four_node_aliases() {
    for count in [2, 4, 16] {
        for fixed in [false, true] {
            for shape in 0..4 {
                let xml = document(count, fixed, "");
                let sdf = parsed(&xml);
                let mut sim = load(&xml);
                let source = runtime_source(&mut sim, &sdf);
                let supplied = requested(source, shape, LOCAL, fixed, 23.);
                let before = snapshot(&sim);
                let template =
                    prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
                assert_eq!(snapshot(&sim), before);
                assert_eq!(template.source_visual_id(), source + 3);
                let source_visual = sim
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
                    &source_visual,
                    &sim.world()
                        .get::<VisualGeometry>(entity_by_id(sim.world(), clone + 3).unwrap())
                        .unwrap()
                        .0
                ));
                check_posed(&sim, clone, shape, [200., 20., 30.], 23.);
            }
        }
    }
}

#[test]
fn primitive_copies_keep_exact_identity_endpoints_and_global_nearest_owner_exclusion() {
    let xml = document(16, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let source = runtime_source(&mut sim, &sdf);
    // The caller's Sphere0 rotation is exactly identity; the mixed source retains
    // its own Ry90 children. Binary endpoint1 does not depend on transformed math.
    let supplied = requested(source, 0, "0 0 0 0 0 0", false, 1.);
    let template = prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
    let a =
        spawn_model_template(&mut sim, &template, pose([0., 0., 10.], [0., 0., 0., 1.])).unwrap();
    let b =
        spawn_model_template(&mut sim, &template, pose([0., 0., 12.], [0., 0., 0., 1.])).unwrap();
    assert_eq!(ray(&sim, [1.5, 0., 10.], [-1., 0., 0.], 1., None), Some(1.));
    assert_eq!(ray(&sim, [1.5, 0., 10.], [-1., 0., 0.], 0.999, None), None);
    assert_eq!(
        ray(&sim, [0., 0., 10.], [1., 0., 0.], 0.125, None),
        Some(0.)
    );
    assert_eq!(
        ray(&sim, [1.5, 0., 10.], [-1., 0., 0.], 1., Some(body(&sim, a))),
        None
    );
    assert_eq!(ray(&sim, [0., 0., 14.], [0., 0., -1.], 5., None), Some(1.5));
    assert_eq!(
        ray(&sim, [0., 0., 14.], [0., 0., -1.], 5., Some(body(&sim, b))),
        Some(3.5)
    );
    assert_eq!(ray(&sim, [0., 0., 14.], [0., 0., -1.], 1.49, None), None);
    let backend = sim.world().resource::<PhysicsBackend>();
    assert!(matches!(
        backend.cast_ray([f64::NAN, 0., 0.], [1., 0., 0.], 1., None),
        Err(PhysicsError::InvalidValue)
    ));
    assert!(matches!(
        backend.cast_ray([0.; 3], [0.; 3], 1., None),
        Err(PhysicsError::InvalidValue)
    ));
}

#[test]
fn copied_singleton_mass_frame_six_axis_edits_and_principal_wrenches_ignore_density() {
    for shape in 0..4 {
        let xml = document(4, false, "");
        let sdf = parsed(&xml);
        let mut sim = load(&xml);
        let source = runtime_source(&mut sim, &sdf);
        let original_source = native(&sim, body(&sim, source));
        let a_dom = requested(source, shape, LOCAL, false, 1.);
        let b_dom = requested(source, shape, LOCAL, false, 1e6);
        let a = prepare_model_template(&sim, &a_dom, &a_dom.worlds[0].models[0]).unwrap();
        let b = prepare_model_template(&sim, &b_dom, &b_dom.worlds[0].models[0]).unwrap();
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
        check_posed(&sim, x, shape, target.translation, 1.);
        let stationary = [
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
        for axis in 0..4 {
            for (model, state) in [x, y].into_iter().zip(stationary) {
                set_model_state(&mut sim, model, state).unwrap();
            }
            let mut torque = [0.; 3];
            if axis < 3 {
                torque[axis] = [6., 8., 4.][axis];
            } else {
                torque = [6., 8., 4.];
            }
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
            if axis < 3 {
                // Stationary world-principal axes have w x Iw=0; cycle maps I to(4,2,3).
                let mut expected = [0.; 3];
                expected[axis] = torque[axis] * 0.001 / [4., 2., 3.][axis];
                vector(left.angular_velocity, expected);
            } else {
                assert!(left.angular_velocity.into_iter().all(f64::is_finite));
                assert!(
                    left.angular_velocity
                        .into_iter()
                        .zip(torque)
                        .map(|(w, t)| w * t)
                        .sum::<f64>()
                        > 0.
                );
            }
        }
        assert_eq!(native(&sim, body(&sim, source)), original_source);
    }
}

#[test]
fn primitive_template_lifetime_survives_compound_retirement_and_both_resets() {
    for fixed in [false, true] {
        let xml = document(4, fixed, "");
        let sdf = parsed(&xml);
        let mut sim = load(&xml);
        let loaded = snapshot(&sim);
        let loaded_source = id(&sim, "w::source");
        let source = runtime_source(&mut sim, &sdf);
        let supplied = requested(source, 3, LOCAL, fixed, 7.);
        let template =
            prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
        let clone = spawn_model_template(&mut sim, &template, cycle([200., 20., 30.])).unwrap();
        let body_before = body(&sim, clone);
        let initial = model_state(&sim, clone).unwrap();
        let new_graph = requested(clone, 1, LOCAL, fixed, 9.);
        // The resulting ordinary singleton can itself seed the mature posed route.
        let detached =
            prepare_model_template(&sim, &new_graph, &new_graph.worlds[0].models[0]).unwrap();
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
        assert_eq!(body(&sim, clone), body_before);
        check_posed(&sim, clone, 3, [200., 20., 30.], 7.);
        assert_eq!(
            remove_model(&mut sim, source).unwrap(),
            (source..source + 7).collect::<Vec<_>>()
        );
        remove_model(&mut sim, loaded_source).unwrap();
        let next = next_entity_id(sim.world()).unwrap();
        let survivor_body = native(
            &sim,
            sim.world()
                .get::<BodyHandle>(entity(&sim, "w::survivor::body"))
                .unwrap()
                .id(),
        );
        let another = spawn_model_template(&mut sim, &detached, cycle([300., 20., 30.])).unwrap();
        assert_eq!(another, next);
        assert!(body(&sim, another).raw() > body_before.raw());
        check_posed(&sim, another, 1, [300., 20., 30.], 9.);
        assert_eq!(
            remove_model(&mut sim, another).unwrap(),
            (another..another + 4).collect::<Vec<_>>()
        );
        let high = next_entity_id(sim.world()).unwrap();
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
        assert_eq!(
            native(
                &sim,
                sim.world()
                    .get::<BodyHandle>(entity(&sim, "w::survivor::body"))
                    .unwrap()
                    .id()
            ),
            survivor_body
        );
        let final_clone =
            spawn_model_template(&mut sim, &template, cycle([400., 20., 30.])).unwrap();
        assert_eq!(final_clone, high);
        assert!(body(&sim, final_clone).raw() > body_before.raw());
        check_posed(&sim, final_clone, 3, [400., 20., 30.], 7.);
    }
}

#[test]
fn new_primitive_route_first_errors_and_late_native_refusals_preserve_pending_inputs() {
    let xml = document(2, false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let source = runtime_source(&mut sim, &sdf);
    let supplied = requested(source, 0, "0 0 0 0 0 0", false, 1.);
    let valid = prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
    queue_link_world_velocity(&mut sim, source + 1, [1., 2., 3.], [0.; 3]).unwrap();
    queue_link_world_wrench(&mut sim, source + 1, [4., 0., 0.], [0.; 3]).unwrap();
    let before = snapshot(&sim);
    for row in 0..17 {
        let mut model = supplied.worlds[0].models[0].clone();
        match row {
            0 => model.links[0].collisions.clear(),
            1 => {
                let child = model.links[0].collisions[0].clone();
                model.links[0].collisions = (0..17).map(|_| child.clone()).collect();
            }
            2 => {
                model.links[0].collisions[0].geometry = Geometry::Plane {
                    normal: [0., 0., 1.],
                    size: [1.; 2],
                }
            }
            3 => {
                // The singleton Mesh route is eligible, but this unchanged
                // invalid URI must still refuse without consuming inputs/IDs.
                let source = model.links[0].collisions[0].source.clone();
                model.links[0].collisions[0].geometry = Geometry::Mesh(sdformat::Mesh {
                    uri: "must-not-be-decoded.glb".into(),
                    scale: [1.; 3],
                    submesh: String::new(),
                    center_submesh: false,
                    optimization: Default::default(),
                    convex_decomposition: None,
                    source,
                });
            }
            4 => model.links[0].pose.transform.translation = [1., 0., 0.],
            5 => model.links[0].pose.relative_to = "absent_link_frame".into(),
            6 => model.links[0].inertial.pose.transform.translation = [1e-13, 0., 0.],
            7 => model.links[0].collisions[0].pose.relative_to = "absent_collision_frame".into(),
            8 => model.links[0].collisions[0].density = Some(f64::NAN),
            9 => model.links[0].collisions[0].pose.transform.translation = [f64::NAN, 0., 0.],
            10 => {
                let surface_source = model.links[0].collisions[0].source.clone();
                model.links[0].collisions[0].surface.source = Some(surface_source);
            }
            11 => {
                model.plugins = parsed(&xml.replacen(
                    "<link name='body'>",
                    &format!("{CONTROL}<link name='body'>"),
                    1,
                ))
                .worlds[0]
                    .models[0]
                    .plugins
                    .clone()
            }
            12 => model.links[0].visuals.clear(),
            13 => model.links[0].inertial.fluid_added_mass = Some(Box::new([[0.; 6]; 6])),
            14 => model.links[0].inertial.pose.transform.rotation = [0.; 4],
            15 => model.links[0].visuals[0].pose.relative_to = "absent_visual_frame".into(),
            _ => model.links[0].pose.transform.rotation = [0., 0., 1., 0.],
        }
        let result = prepare_model_template(&sim, &supplied, &model);
        let error = result.as_ref().err();
        match row {
            5 | 7 | 9 | 14 | 15 => assert!(
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
    for row in 0..3 {
        let mut model = supplied.worlds[0].models[0].clone();
        match row {
            0 => model.links[0].inertial.mass = f64::NAN,
            1 => model.links[0].inertial.inertia[0] = -1.,
            _ => model.links[0].collisions[0].geometry = Geometry::Sphere { radius: 1e200 },
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
    // Keep the mature loaded-controlled identity caller-parameter policy, even
    // for tiny nonzero COM. The actual controller still owns its source setter.
    let controlled_xml = xml.replacen(
        "<link name='body'>",
        &format!("{CONTROL}<link name='body'>"),
        1,
    );
    let controlled_sdf = parsed(&controlled_xml);
    let mut controlled = load(&controlled_xml);
    let mut stripped = controlled_sdf.worlds[0].models[0].clone();
    stripped.plugins.clear();
    stripped.links[0].collisions.truncate(1);
    stripped.links[0].collisions[0].pose.transform.translation = [0.; 3];
    stripped.links[0].collisions[0].pose.transform.rotation = [0., 0., 0., 1.];
    stripped.links[0].inertial.pose.transform.translation = [1e-13, 0., 0.];
    let old_identity = prepare_model_template(&controlled, &controlled_sdf, &stripped).unwrap();
    let controlled_id = id(&controlled, "w::source");
    let observed = model_state(&controlled, controlled_id).unwrap();
    assert!(matches!(
        set_model_state(&mut controlled, controlled_id, observed),
        Err(WorldEditError::UnsupportedModel)
    ));
    spawn_model_template(
        &mut controlled,
        &old_identity,
        pose([500., 0., 10.], [0., 0., 0., 1.]),
    )
    .unwrap();
    stripped.links[0].inertial.pose.transform.translation = [0.; 3];
    stripped.links[0].collisions[0].pose.transform.translation = [2., 0., 0.];
    assert!(matches!(
        prepare_model_template(&controlled, &controlled_sdf, &stripped),
        Err(WorldEditError::UnsupportedModel)
    ));
    let tpe_xml = "<sdf version='1.12'><world name='w'><model name='source'><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='view'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>";
    let tpe_sdf = parsed(tpe_xml);
    let tpe =
        load_world_with_sdf_step(tpe_xml, "tpe", "w", &NoResources, PhysicsEngine::Tpe).unwrap();
    let tpe_ids = identities(&tpe);
    let tpe_next = next_entity_id(tpe.world()).unwrap();
    let clock = (tpe.epoch(), tpe.elapsed());
    let motions = || {
        tpe.world()
            .iter_entities()
            .filter_map(|e| {
                e.get::<gz_physics_tpe::Motion>()
                    .map(|v| (e.id(), *v.state()))
            })
            .collect::<Vec<_>>()
    };
    let before_motion = motions();
    assert_eq!(before_motion.len(), 1);
    assert!(matches!(
        prepare_model_template(&tpe, &tpe_sdf, &tpe_sdf.worlds[0].models[0]),
        Err(WorldEditError::InvalidScene)
    ));
    assert_eq!(motions(), before_motion);
    assert_eq!(identities(&tpe), tpe_ids);
    assert_eq!(next_entity_id(tpe.world()).unwrap(), tpe_next);
    assert_eq!((tpe.epoch(), tpe.elapsed()), clock);
}

#[test]
fn actual_compound_membership_corruption_cannot_authorize_singleton_and_repair_keeps_wrench() {
    for row in 0..8 {
        let xml = document(4, false, "");
        let sdf = parsed(&xml);
        let mut sim = load(&xml);
        let source = runtime_source(&mut sim, &sdf);
        let supplied = requested(source, 1, LOCAL, false, 1.);
        prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
        let link = entity_by_id(sim.world(), source + 1).unwrap();
        let child = entity_by_id(sim.world(), source + 4).unwrap();
        queue_link_world_wrench(&mut sim, source + 1, [4., 0., 0.], [0.; 3]).unwrap();
        if row == 0 {
            sim.world_mut().entity_mut(child).despawn();
            let before = snapshot(&sim);
            assert!(matches!(
                prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]),
                Err(WorldEditError::UnsupportedModel)
            ));
            assert_eq!(snapshot(&sim), before);
            continue;
        }
        let original_local = *sim.world().get::<LocalPose>(child).unwrap();
        let original_pose = *sim.world().get::<WorldPose>(child).unwrap();
        let original_name = sim.world().get::<LocalName>(child).unwrap().0.clone();
        let original_density = *sim.world().get::<CollisionDensity>(child).unwrap();
        let foreign = entity(&sim, "w::survivor::body");
        match row {
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
        assert!(matches!(
            prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]),
            Err(WorldEditError::UnsupportedModel)
        ));
        assert_eq!(snapshot(&sim), before);
        sim.world_mut().entity_mut(child).insert((
            ChildOf(link),
            LocalName(original_name),
            original_local,
            original_pose,
            original_density,
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
        prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
        sim.step_exact(1).unwrap();
        near(native(&sim, body(&sim, source)).velocity[0], 0.002);
    }
}

#[test]
fn singleton_copy_capacity_uses_real_camera_bound_and_keeps_native_and_logical_ids() {
    let camera = "<model name='observer'><static>true</static><pose>-400 0 10 0 0 0</pose><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><sensor name='logical' type='logical_camera'><update_rate>0</update_rate><logical_camera><near>0.1</near><far>1000</far><horizontal_fov>1.5</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor></link></model>";
    let xml = document(2, false, camera);
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let source = runtime_source(&mut sim, &sdf);
    let supplied = requested(source, 0, "0 0 0 0 0 0", false, 1.);
    let template = prepare_model_template(&sim, &supplied, &supplied.worlds[0].models[0]).unwrap();
    queue_link_world_velocity(&mut sim, source + 1, [1., 0., 0.], [0.; 3]).unwrap();
    queue_link_world_wrench(&mut sim, source + 1, [4., 0., 0.], [0.; 3]).unwrap();
    let mut clones = Vec::new();
    // Three loaded models plus one runtime source, then124 ordinary clones =128.
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
fn static_primitive_copy_contact_and_clear_control_survive_ordinary_reset() {
    let falling = "<model name='ball'><pose>0 0 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy><izz>0.025</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model><model name='miss'><pose>30 0 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.025</ixx><iyy>0.025</iyy><izz>0.025</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";
    let xml = document(2, true, falling).replacen(
        "<gravity>0 0 0</gravity>",
        "<gravity>0 0 -9.8</gravity>",
        1,
    );
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let source = runtime_source(&mut sim, &sdf);
    let supplied = requested(source, 2, "0 0 0 0 0 0", true, 17.);
    let mut model = supplied.worlds[0].models[0].clone();
    model.links[0].collisions[0].geometry = Geometry::Box { size: [2., 2., 1.] };
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
    for reset in [false, true] {
        if reset {
            sim.reset().unwrap();
        }
        // Retained upfront1.5s horizon: ball bottom3.75 starts2.25 above floor top1.5.
        for _ in 0..1500 {
            sim.step_exact(1).unwrap();
        }
        assert!((native(&sim, ball).position[2] - 1.75).abs() < 0.01);
        assert!(native(&sim, miss).position[2] < 0.);
        assert_eq!(native(&sim, body(&sim, floor)), floor_state);
    }
}
