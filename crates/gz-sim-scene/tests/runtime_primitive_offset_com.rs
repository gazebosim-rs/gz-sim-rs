// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Offset COM moves the mass frame, never the model/collider/visual origin.
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
use sdformat::NoResources;
use std::{collections::BTreeMap, sync::Arc, time::Duration};

const SHAPES: [&str; 4] = [
    "<box><size>1.5 0.5 1</size></box>",
    "<sphere><radius>0.5</radius></sphere>",
    "<cylinder><radius>0.5</radius><length>2</length></cylinder>",
    "<capsule><radius>0.5</radius><length>2</length></capsule>",
];
const LOCAL: &str = "2 0 0 0 1.5707963267948966 0";
const COM: [f64; 3] = [0.25, -0.5, 0.75];
const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>2</izz></inertia></inertial>";
const VISUAL: &str = "<visual name='view'><pose>0.25 -0.5 0.75 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual>";
const CONTROL: &str =
    "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'/>";
fn document(shape: &str, fixed: bool, extra: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='source'><pose>-100 0 10 0 0 0</pose><static>{fixed}</static><link name='body'>{INERTIA}<collision name='solid'><pose>{LOCAL}</pose><density>37</density><geometry>{shape}</geometry></collision>{VISUAL}</link></model><model name='survivor'><pose>200 200 10 0 0 0</pose><static>true</static><link name='body'><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision>{VISUAL}</link></model>{extra}</world></sdf>"
    )
}
fn parsed(xml: &str) -> sdformat::Sdf {
    sdformat::load_str(xml, "offset-com", &NoResources).unwrap()
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(xml, "offset-com", "w", &NoResources, PhysicsEngine::Rapier).unwrap()
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
    assert!(
        a.is_finite() && b.is_finite() && (a - b).abs() < 1e-11,
        "{a} != {b}"
    );
}
fn vector(a: [f64; 3], b: [f64; 3]) {
    for (a, b) in a.into_iter().zip(b) {
        near(a, b);
    }
}
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
// Independent unit-quaternion vector identity; no Scene transform helper.
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let u = [q[0], q[1], q[2]];
    let uv = cross(u, v);
    let uuv = cross(u, uv);
    std::array::from_fn(|i| v[i] + 2. * (q[3] * uv[i] + uuv[i]))
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
    max: f64,
    exclude: Option<BodyId>,
) -> Option<f64> {
    sim.world()
        .resource::<PhysicsBackend>()
        .cast_ray(origin, direction, max, exclude)
        .unwrap()
}
fn request(sdf: &sdformat::Sdf, center: [f64; 3], density: f64) -> sdformat::Model {
    let mut model = sdf.worlds[0].models[0].clone();
    model.links[0].inertial.pose.transform.translation = center;
    model.links[0].collisions[0].density = Some(density);
    model
}
fn runtime_document(model: u64, shape: &str, center: [f64; 3]) -> sdformat::Sdf {
    parsed(&format!(
        "<sdf version='1.12'><world name='w'><model name='runtime_{model}'><link name='body'><inertial><pose>{} {} {} 0 0 0</pose><mass>2</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>2</izz></inertia></inertial><collision name='collision'><pose>{LOCAL}</pose><geometry>{shape}</geometry></collision><visual name='visual'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>",
        center[0], center[1], center[2]
    ))
}
fn check_frames(sim: &Simulation, model: u64, index: usize, p: [f64; 3]) {
    let link = entity_by_id(sim.world(), model + 1).unwrap();
    let collision = entity_by_id(sim.world(), model + 2).unwrap();
    let visual = entity_by_id(sim.world(), model + 3).unwrap();
    vector(native(sim, body(sim, model)).position, p); // Native position is BODY origin, not COM.
    vector(sim.world().get::<WorldPose>(link).unwrap().translation, p);
    vector(
        sim.world().get::<LocalPose>(link).unwrap().0.translation,
        [0.; 3],
    );
    assert_eq!(
        sim.world().get::<LocalPose>(link).unwrap().0.rotation,
        [0., 0., 0., 1.]
    );
    vector(
        sim.world().get::<WorldPose>(collision).unwrap().translation,
        add(p, [0., 2., 0.]),
    );
    vector(
        sim.world().get::<WorldPose>(visual).unwrap().translation,
        add(p, [0.75, 0.25, -0.5]),
    );
    let frame = sim.world().get::<WorldPose>(collision).unwrap();
    for (axis, expected) in [
        ([1., 0., 0.], [-1., 0., 0.]),
        ([0., 1., 0.], [0., 0., 1.]),
        ([0., 0., 1.], [0., 1., 0.]),
    ] {
        vector(rotate(frame.rotation, axis), expected);
    }
    // Every other body is in another X/Y lane. COM is deliberately different
    // from both collision center and visual offset; none may shift the rays.
    let center = add(p, [0., 2., 0.]);
    let origin = add(center, [0.3, 0., 4.]);
    near(
        ray(sim, origin, [0., 0., -1.], 5., None).unwrap(),
        if index == 0 { 3.75 } else { 3.6 },
    );
    assert_eq!(ray(sim, origin, [0., 0., -1.], 3., None), None);
    assert_eq!(
        ray(sim, origin, [0., 0., -1.], 5., Some(body(sim, model))),
        None
    );
    let axial = [3.5, 3.5, 3., 2.5][index];
    near(
        ray(sim, add(center, [0., 4., 0.]), [0., -1., 0.], 5., None).unwrap(),
        axial,
    );
    assert_eq!(ray(sim, center, [0., 0., 1.], 0.125, None), Some(0.));
}

#[test]
fn offset_four_primitives_keep_model_child_frames_four_ids_and_visual_aliases() {
    for (index, shape) in SHAPES.into_iter().enumerate() {
        for fixed in [false, true] {
            let xml = document(shape, fixed, "");
            let sdf = parsed(&xml);
            let mut sim = load(&xml);
            let before = snapshot(&sim);
            let source_visual = entity(&sim, "w::source::body::view");
            let alias = sim
                .world()
                .get::<VisualGeometry>(source_visual)
                .unwrap()
                .0
                .clone();
            let model = request(&sdf, COM, 37.);
            let template = prepare_model_template(&sim, &sdf, &model).unwrap();
            assert_eq!(snapshot(&sim), before);
            assert_eq!(
                template.source_visual_id(),
                id(&sim, "w::source::body::view")
            );
            for p in [[10., 20., 30.], [50., 20., 30.]] {
                let next = next_entity_id(sim.world()).unwrap();
                let spawned = spawn_model_template(&mut sim, &template, cycle(p)).unwrap();
                assert_eq!(spawned, next);
                assert_eq!(next_entity_id(sim.world()).unwrap(), spawned + 4);
                for (offset, kind) in [
                    EntityKind::Model,
                    EntityKind::Link,
                    EntityKind::Collision,
                    EntityKind::Visual,
                ]
                .into_iter()
                .enumerate()
                {
                    let e = entity_by_id(sim.world(), spawned + offset as u64).unwrap();
                    assert_eq!(sim.world().get::<EntityKind>(e), Some(&kind));
                    if offset > 0 {
                        assert_eq!(
                            sim.world().get::<ChildOf>(e).unwrap().parent(),
                            entity_by_id(
                                sim.world(),
                                if offset == 1 { spawned } else { spawned + 1 }
                            )
                            .unwrap()
                        );
                    }
                }
                let visual = entity_by_id(sim.world(), spawned + 3).unwrap();
                assert!(Arc::ptr_eq(
                    &alias,
                    &sim.world().get::<VisualGeometry>(visual).unwrap().0
                ));
                assert_eq!(
                    sim.world()
                        .resource::<PhysicsBackend>()
                        .is_dynamic(body(&sim, spawned)),
                    Some(!fixed)
                );
                near(
                    sim.world()
                        .get::<CollisionDensity>(entity_by_id(sim.world(), spawned + 2).unwrap())
                        .unwrap()
                        .0,
                    37.,
                );
                check_frames(&sim, spawned, index, p);
                let (visual_id, projected) =
                    gz_sim_scene::model_visual_pose(&sim, spawned, cycle(p)).unwrap();
                assert_eq!(visual_id, spawned + 3);
                vector(projected.translation, add(p, [0.75, 0.25, -0.5]));
            }
            for (handle, state) in before.native {
                assert_eq!(native(&sim, handle), state);
            }
        }
    }
}

#[test]
fn offset_geometry_keeps_binary_endpoints_and_global_nearest_exclusion() {
    let xml = document(SHAPES[1], true, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let mut model = request(&sdf, COM, 1.);
    model.links[0].collisions[0].pose.transform.rotation = [0., 0., 0., 1.];
    let template = prepare_model_template(&sim, &sdf, &model).unwrap();
    let a =
        spawn_model_template(&mut sim, &template, pose([0., 0., 10.], [0., 0., 0., 1.])).unwrap();
    // Exact binary geometry/identity rotations isolate inclusive max=1; the COM
    // is unrelated to the collider, and the later closer body does not yet exist.
    assert_eq!(ray(&sim, [2., 0., 11.5], [0., 0., -1.], 1., None), Some(1.));
    assert_eq!(ray(&sim, [2., 0., 11.5], [0., 0., -1.], 0.999, None), None);
    assert_eq!(
        ray(&sim, [2., 0., 10.], [0., 0., 1.], 0.125, None),
        Some(0.)
    );
    assert_eq!(
        ray(&sim, [2., 0., 11.5], [0., 0., -1.], 1., Some(body(&sim, a))),
        None
    );
    let b =
        spawn_model_template(&mut sim, &template, pose([0., 0., 12.], [0., 0., 0., 1.])).unwrap();
    assert_eq!(ray(&sim, [2., 0., 14.], [0., 0., -1.], 5., None), Some(1.5));
    assert_eq!(
        ray(&sim, [2., 0., 14.], [0., 0., -1.], 5., Some(body(&sim, b))),
        Some(3.5)
    );
    assert_eq!(ray(&sim, [2., 0., 14.], [0., 0., -1.], 1.49, None), None);
}

#[test]
fn offset_six_axis_state_and_velocity_inputs_round_trip_origin_and_com_twists() {
    for shape in SHAPES {
        let xml = document(shape, false, "");
        let sdf = parsed(&xml);
        let mut sim = load(&xml);
        // Schedule initialization may create Bevy's Schedules resource entity;
        // it runs no tick/reset callbacks or pending physical inputs.
        sim.initialize_schedules().unwrap();
        let model = request(&sdf, COM, 1.);
        let template = prepare_model_template(&sim, &sdf, &model).unwrap();
        let spawned = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
        let target = ModelState {
            translation: [20., 30., 40.],
            rotation: [0.5; 4],
            linear_velocity: [1., -2., 3.],
            angular_velocity: [-0.25, 0.5, 0.75],
        };
        assert_eq!(set_model_state(&mut sim, spawned, target).unwrap(), target);
        let arm = [0.75, 0.25, -0.5];
        let expected = add(target.linear_velocity, cross(target.angular_velocity, arm));
        vector(native(&sim, body(&sim, spawned)).velocity, expected);
        assert_ne!(expected, target.linear_velocity);
        let observed = model_state(&sim, spawned).unwrap();
        vector(observed.linear_velocity, target.linear_velocity);
        vector(observed.angular_velocity, target.angular_velocity);
        let link = spawned + 1;
        queue_link_world_velocity(&mut sim, link, [-1., 2., -3.], [0., 0., 2.]).unwrap();
        sim.set_paused(true);
        let before = snapshot(&sim);
        assert!(sim.tick().unwrap().dt.is_zero());
        assert_eq!(snapshot(&sim), before);
        sim.set_paused(false);
        sim.step_exact(1).unwrap();
        let state = native(&sim, body(&sim, spawned));
        vector(
            state.velocity,
            add([-1., 2., -3.], cross([0., 0., 2.], arm)),
        );
        vector(state.angular_velocity, [0., 0., 2.]);
        let logical = model_state(&sim, spawned).unwrap();
        vector(
            add(
                logical.linear_velocity,
                cross(logical.angular_velocity, rotate(logical.rotation, COM)),
            ),
            state.velocity,
        );
        let stationary = ModelState {
            linear_velocity: [0.; 3],
            angular_velocity: [0.; 3],
            ..target
        };
        set_model_state(&mut sim, spawned, stationary).unwrap();
        queue_link_world_velocity(&mut sim, link, [1., 2., 3.], [0.; 3]).unwrap();
        sim.step_exact(1).unwrap();
        vector(native(&sim, body(&sim, spawned)).velocity, [1., 2., 3.]);
        vector(
            model_state(&sim, spawned).unwrap().linear_velocity,
            [1., 2., 3.],
        );
    }
}

#[test]
fn offset_explicit_mass_inertia_and_com_wrenches_ignore_collision_density() {
    for shape in SHAPES {
        let xml = document(shape, false, "");
        let sdf = parsed(&xml);
        let mut sim = load(&xml);
        let source = states(&sim);
        let a = prepare_model_template(&sim, &sdf, &request(&sdf, COM, 1.)).unwrap();
        let b = prepare_model_template(&sim, &sdf, &request(&sdf, COM, 1e6)).unwrap();
        let x = spawn_model_template(&mut sim, &a, cycle([10., 20., 30.])).unwrap();
        let y = spawn_model_template(&mut sim, &b, cycle([60., 20., 30.])).unwrap();
        for axis in 0..4 {
            for (model, p) in [(x, [10., 20., 30.]), (y, [60., 20., 30.])] {
                set_model_state(
                    &mut sim,
                    model,
                    ModelState {
                        translation: p,
                        rotation: [0.5; 4],
                        linear_velocity: [0.; 3],
                        angular_velocity: [0.; 3],
                    },
                )
                .unwrap();
            }
            let force = if axis == 3 { [2., -4., 6.] } else { [0.; 3] };
            let mut torque = [0.; 3];
            if axis < 3 {
                torque[axis] = [6., 8., 4.][axis];
            }
            for model in [x, y] {
                queue_link_world_wrench(&mut sim, model + 1, force, torque).unwrap();
            }
            sim.set_paused(true);
            let before = states(&sim);
            assert!(sim.tick().unwrap().dt.is_zero());
            assert_eq!(states(&sim), before);
            sim.set_paused(false);
            sim.step_exact(1).unwrap();
            let nx = native(&sim, body(&sim, x));
            let ny = native(&sim, body(&sim, y));
            vector(nx.velocity, ny.velocity);
            vector(nx.angular_velocity, ny.angular_velocity);
            vector(nx.velocity, force.map(|v| v * 0.001 / 2.));
            vector(nx.angular_velocity, torque.map(|v| v * 0.001 / 2.));
            if axis < 3 {
                // Isotropic COM inertia and a single world-axis torque have no
                // gyro term. Pure torque holds the actual COM fixed while the
                // body origin moves around it; no finite-step tau/I position law.
                vector(
                    add(nx.position, rotate(nx.orientation, COM)),
                    [10.75, 20.25, 29.5],
                );
                let logical = model_state(&sim, x).unwrap();
                vector(
                    logical.linear_velocity,
                    cross(logical.angular_velocity, rotate(logical.rotation, COM)).map(|v| -v),
                );
                assert!(logical.linear_velocity.into_iter().any(|v| v.abs() > 1e-6));
            } else {
                vector(nx.angular_velocity, [0.; 3]);
                assert!(nx.position[0] > 10. && nx.position[1] < 20. && nx.position[2] > 30.);
            }
            let velocity = nx.velocity;
            sim.step_exact(1).unwrap();
            vector(native(&sim, body(&sim, x)).velocity, velocity);
        }
        for (handle, state) in source {
            assert_eq!(native(&sim, handle), state);
        }
    }
}

#[test]
fn offset_templates_reuse_passive_runtime_sources_retire_and_keep_reset_high_water() {
    for shape in SHAPES {
        for fixed in [false, true] {
            let xml = document(shape, fixed, "");
            let sdf = parsed(&xml);
            let mut sim = load(&xml);
            let original_ids = identities(&sim);
            let original = states(&sim);
            let template = prepare_model_template(&sim, &sdf, &request(&sdf, COM, 1.)).unwrap();
            let source = id(&sim, "w::source");
            let origin = cycle([10., 20., 30.]);
            let x = spawn_model_template(&mut sim, &template, origin).unwrap();
            let native_x = body(&sim, x);
            let arc = sim
                .world()
                .get::<VisualGeometry>(entity_by_id(sim.world(), x + 3).unwrap())
                .unwrap()
                .0
                .clone();
            let fresh = runtime_document(x, shape, [-0.25, 0.5, -0.75]);
            let mut requested = fresh.worlds[0].models[0].clone();
            requested.is_static = fixed;
            let copied = prepare_model_template(&sim, &fresh, &requested).unwrap();
            let y = spawn_model_template(&mut sim, &copied, cycle([50., 20., 30.])).unwrap();
            assert!(Arc::ptr_eq(
                &arc,
                &sim.world()
                    .get::<VisualGeometry>(entity_by_id(sim.world(), y + 3).unwrap())
                    .unwrap()
                    .0
            ));
            set_model_state(
                &mut sim,
                x,
                ModelState {
                    translation: [20., 30., 40.],
                    rotation: [0.5; 4],
                    linear_velocity: if fixed { [0.; 3] } else { [1., 2., 3.] },
                    angular_velocity: if fixed { [0.; 3] } else { [0., 0., 1.] },
                },
            )
            .unwrap();
            sim.step_exact(2).unwrap();
            sim.reset().unwrap();
            assert_eq!(body(&sim, x), native_x);
            let reset = model_state(&sim, x).unwrap();
            vector(reset.translation, origin.translation);
            vector(reset.linear_velocity, [0.; 3]);
            vector(reset.angular_velocity, [0.; 3]);
            remove_model(&mut sim, source).unwrap();
            assert_eq!(
                remove_model(&mut sim, x).unwrap(),
                (x..x + 4).collect::<Vec<_>>()
            );
            assert!(
                sim.world()
                    .resource::<PhysicsBackend>()
                    .state(native_x)
                    .is_none()
            );
            let high = next_entity_id(sim.world()).unwrap();
            let z = spawn_model_template(&mut sim, &template, cycle([90., 20., 30.])).unwrap();
            assert_eq!(z, high);
            assert!(body(&sim, z).raw() > body(&sim, y).raw());
            let before_reset = next_entity_id(sim.world()).unwrap();
            let native_z = body(&sim, z);
            reset_loaded_world(&mut sim, |restored| {
                for model in [y, z] {
                    for offset in 0..4 {
                        let e = entity_by_id(restored.world(), model + offset).unwrap();
                        assert!(restored.world().get::<PendingResetRemoval>(e).is_some());
                        assert!(restored.world().get::<BodyHandle>(e).is_none());
                    }
                }
            })
            .unwrap();
            assert_eq!(next_entity_id(sim.world()).unwrap(), before_reset);
            assert_eq!(states(&sim), original);
            for key in original_ids.keys() {
                assert!(entity_by_id(sim.world(), *key).is_some());
            }
            let rebuilt = spawn_model_template(&mut sim, &copied, cycle([130., 20., 30.])).unwrap();
            assert_eq!(rebuilt, before_reset);
            assert_eq!(body(&sim, rebuilt).raw(), native_z.raw() + 1);
            assert!(Arc::ptr_eq(
                &arc,
                &sim.world()
                    .get::<VisualGeometry>(entity_by_id(sim.world(), rebuilt + 3).unwrap())
                    .unwrap()
                    .0
            ));
        }
    }
}

#[test]
fn offset_request_errors_and_source_corruption_preserve_allocators_and_pending_work() {
    // This Scene harness runs in both maintained profiles. The deeper private
    // Rapier matrix is headless-only because renderer Cargo selects Scene but
    // does not select gz-physics-rapier's own libtest target.
    {
        use gz_physics::{
            BodySpec, DynamicBackend, Inertia, PosedBoxBodySpec, PosedCapsuleBodySpec,
            PosedCylinderBodySpec, PosedSphereBodySpec, RotationSpec, Shape,
        };
        use gz_physics_rapier::RapierBackend;
        fn spawn(
            backend: &mut RapierBackend,
            kind: usize,
            center: [f64; 3],
            position: [f64; 3],
        ) -> Result<BodyId, PhysicsError> {
            let q = std::f64::consts::FRAC_1_SQRT_2;
            let rotation = RotationSpec {
                orientation: [0.5; 4],
                angular_velocity: [0.; 3],
            };
            let inertia = Some(Inertia {
                center,
                orientation: [0., 0., 0., 1.],
                tensor: [2., 2., 2., 0., 0., 0.],
            });
            match kind {
                0 => backend.spawn_posed_box_link(
                    PosedBoxBodySpec {
                        half_extents: [0.75, 0.25, 0.5],
                        collision_position: [2., 0., 0.],
                        collision_orientation: [0., q, 0., q],
                        position,
                        velocity: [0.; 3],
                        mass: Some(2.),
                    },
                    rotation,
                    inertia,
                ),
                1 => backend.spawn_posed_sphere_link(
                    PosedSphereBodySpec {
                        radius: 0.5,
                        collision_position: [2., 0., 0.],
                        collision_orientation: [0., q, 0., q],
                        position,
                        velocity: [0.; 3],
                        mass: Some(2.),
                    },
                    rotation,
                    inertia,
                ),
                2 => backend.spawn_posed_cylinder_link(
                    PosedCylinderBodySpec {
                        radius: 0.5,
                        half_height: 1.,
                        collision_position: [2., 0., 0.],
                        collision_orientation: [0., q, 0., q],
                        position,
                        velocity: [0.; 3],
                        mass: Some(2.),
                    },
                    rotation,
                    inertia,
                ),
                _ => backend.spawn_posed_capsule_link(
                    PosedCapsuleBodySpec {
                        radius: 0.5,
                        half_length: 1.,
                        collision_position: [2., 0., 0.],
                        collision_orientation: [0., q, 0., q],
                        position,
                        velocity: [0.; 3],
                        mass: Some(2.),
                    },
                    rotation,
                    inertia,
                ),
            }
        }
        for kind in 0..4 {
            let mut backend = RapierBackend::new([0.; 3]).unwrap();
            let mut twin = RapierBackend::new([0.; 3]).unwrap();
            let existing = BodySpec {
                shape: Shape::Sphere { radius: 0.25 },
                position: [-20., 0., 0.],
                velocity: [0.; 3],
                mass: Some(2.),
            };
            let pending = backend.spawn(existing).unwrap();
            assert_eq!(twin.spawn(existing).unwrap(), pending);
            // Ordinary model/collider geometry and I2 pass before the new COM
            // check. This accepted control prevents an unconditional refusal.
            let control = spawn(&mut backend, kind, COM, [40., 0., 10.]).unwrap();
            assert_eq!(
                spawn(&mut twin, kind, COM, [40., 0., 10.]).unwrap(),
                control
            );
            for b in [&mut backend, &mut twin] {
                b.add_world_wrench(pending, [2., 4., 6.], [0., 0., 0.1])
                    .unwrap();
            }
            let before = [
                backend.state(pending).unwrap(),
                backend.state(control).unwrap(),
            ];
            let overflow = [1e200, 0., 0.];
            assert!(overflow.into_iter().all(f64::is_finite));
            // Bypass Scene's f32 pose guard through the existing PUBLIC native
            // API: finite COM components, overflowing squared norm. The valid
            // shape/model/inertia above cannot mask this native COM predicate.
            assert_eq!(
                spawn(&mut backend, kind, overflow, [80., 0., 10.]),
                Err(PhysicsError::InvalidValue),
                "kind={kind}"
            );
            assert_eq!(
                [
                    backend.state(pending).unwrap(),
                    backend.state(control).unwrap()
                ],
                before
            );
            let repaired = spawn(&mut backend, kind, COM.map(|v| -v), [80., 0., 10.]).unwrap();
            assert_eq!(repaired.raw(), control.raw() + 1);
            assert_eq!(
                spawn(&mut twin, kind, COM.map(|v| -v), [80., 0., 10.]).unwrap(),
                repaired
            );
            backend.step(Duration::ZERO).unwrap();
            assert_eq!(backend.state(pending), Some(before[0]));
            backend.step(Duration::from_millis(1)).unwrap();
            twin.step(Duration::from_millis(1)).unwrap();
            for handle in [pending, control, repaired] {
                assert_eq!(backend.state(handle), twin.state(handle));
            }
            vector(
                backend.state(pending).unwrap().velocity,
                [0.001, 0.002, 0.003],
            );
            assert!(backend.state(pending).unwrap().angular_velocity[2] > 0.);
        }
    }
    let xml = document(SHAPES[1], false, "");
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let requested = request(&sdf, COM, 1.);
    let template = prepare_model_template(&sim, &sdf, &requested).unwrap();
    let pending = spawn_model_template(&mut sim, &template, cycle([10., 20., 30.])).unwrap();
    queue_link_world_wrench(&mut sim, pending + 1, [2., 4., 6.], [0.; 3]).unwrap();
    let before = snapshot(&sim);
    let native_next = before.native.keys().map(|v| v.raw()).max().unwrap() + 1;
    for row in 0..11 {
        let mut model = requested.clone();
        match row {
            0 => model.links[0].pose.transform.translation = [0.25, 0., 0.],
            1 => model.links[0].pose.transform.rotation = [0.; 4],
            2 => model.links[0].inertial.fluid_added_mass = Some(Box::new([[0.; 6]; 6])),
            3 => model.links[0].inertial.pose.transform.translation = [f64::NAN, 0., 0.],
            4 => model.links[0].inertial.pose.transform.translation = [1e100, 0., 0.],
            5 => model.links[0].inertial.pose.transform.rotation = [0.; 4],
            6 => model.links[0].collisions[0].pose.transform.translation = [f64::NAN, 0., 0.],
            7 => model.links[0].collisions[0].density = Some(f64::NAN),
            8 => model.links[0].collisions[0].pose.transform.translation = [1e100, 0., 0.],
            9 => {
                let child = model.links[0].collisions[0].clone();
                model.links[0].collisions.push(child);
            }
            _ => {
                let visual = model.links[0].visuals[0].clone();
                model.links[0].visuals.push(visual);
            }
        }
        let result = prepare_model_template(&sim, &sdf, &model);
        match row {
            1 | 3 | 5 | 6 => assert!(
                matches!(&result, Err(WorldEditError::InvalidScene)),
                "row={row}, error={:?}",
                result.as_ref().err()
            ),
            7 | 8 => assert!(
                matches!(&result, Err(WorldEditError::InvalidState)),
                "row={row}, error={:?}",
                result.as_ref().err()
            ),
            _ => assert!(
                matches!(&result, Err(WorldEditError::UnsupportedModel)),
                "row={row}, error={:?}",
                result.as_ref().err()
            ),
        }
        assert_eq!(snapshot(&sim), before);
    }
    for row in 0..3 {
        let mut model = requested.clone();
        match row {
            0 => model.links[0].inertial.mass = f64::NAN,
            1 => model.links[0].inertial.inertia[0] = -1.,
            _ => {
                model.links[0].collisions[0].geometry = sdformat::Geometry::Sphere { radius: 1e200 }
            }
        }
        let invalid = prepare_model_template(&sim, &sdf, &model).unwrap();
        assert!(matches!(
            spawn_model_template(&mut sim, &invalid, cycle([60., 20., 30.])),
            Err(WorldEditError::Physics(PhysicsError::InvalidValue))
        ));
        assert_eq!(snapshot(&sim), before);
    }
    for position in [
        pose([f64::NAN, 0., 0.], [0., 0., 0., 1.]),
        pose([1., 2., 3.], [0.; 4]),
        pose([f64::MAX, 0., 0.], [0., 0., 0., 1.]),
    ] {
        assert!(matches!(
            spawn_model_template(&mut sim, &template, position),
            Err(WorldEditError::InvalidState)
        ));
        assert_eq!(snapshot(&sim), before);
    }
    let mut foreign = load(&xml);
    let old = snapshot(&foreign);
    assert!(matches!(
        spawn_model_template(&mut foreign, &template, cycle([1., 2., 3.])),
        Err(WorldEditError::InvalidScene)
    ));
    assert_eq!(snapshot(&foreign), old);
    let source_link = entity(&sim, "w::source::body");
    let source_handle = sim
        .world_mut()
        .entity_mut(source_link)
        .take::<BodyHandle>()
        .unwrap();
    let old = snapshot(&sim);
    let mut malformed = requested.clone();
    malformed.links[0].inertial.pose.transform.rotation = [0.; 4];
    // Actual source ownership fails before the caller's malformed inertial frame.
    assert!(matches!(
        prepare_model_template(&sim, &sdf, &malformed),
        Err(WorldEditError::InvalidScene)
    ));
    assert_eq!(snapshot(&sim), old);
    sim.world_mut()
        .entity_mut(source_link)
        .insert(source_handle);
    let collision = entity(&sim, "w::source::body::solid");
    sim.world_mut().entity_mut(collision).remove::<ChildOf>();
    let old = snapshot(&sim);
    assert!(matches!(
        prepare_model_template(&sim, &sdf, &malformed),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim), old);
    sim.world_mut()
        .entity_mut(collision)
        .insert(ChildOf(source_link));
    assert_eq!(snapshot(&sim), before);
    let accepted = spawn_model_template(&mut sim, &template, cycle([60., 20., 30.])).unwrap();
    assert_eq!(accepted, before.next);
    assert_eq!(body(&sim, accepted).raw(), native_next);
    sim.step_exact(1).unwrap();
    vector(
        native(&sim, body(&sim, pending)).velocity,
        [0.001, 0.002, 0.003],
    );
    vector(native(&sim, body(&sim, pending)).angular_velocity, [0.; 3]);
}

#[test]
fn offset_new_route_keeps_actual_compounds_controllers_and_mature_identity_distinct() {
    let xml = document(SHAPES[0], false, "");
    let controlled = xml.replacen(
        "<link name='body'>",
        &format!("{CONTROL}<link name='body'>"),
        1,
    );
    let sdf = parsed(&controlled);
    let mut sim = load(&controlled);
    let owner = id(&sim, "w::source");
    assert!(velocity_control_binding_matches(&sim, owner));
    let mut model = request(&sdf, COM, 1.);
    model.plugins.clear();
    set_velocity_control_command(&mut sim, owner, [1., 2., 3.], [0.; 3]).unwrap();
    let before = snapshot(&sim);
    assert!(matches!(
        prepare_model_template(&sim, &sdf, &model),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim), before);
    // Exact legacy identity/tiny-COM route remains distinct from new posed COM.
    model.links[0].collisions[0].pose.transform.translation = [0.; 3];
    model.links[0].collisions[0].pose.transform.rotation = [0., 0., 0., 1.];
    model.links[0].inertial.pose.transform.translation = [1e-13, 0., 0.];
    let old = prepare_model_template(&sim, &sdf, &model).unwrap();
    let clone =
        spawn_model_template(&mut sim, &old, pose([30., 0., 10.], [0., 0., 0., 1.])).unwrap();
    let target = ModelState {
        translation: [30., 0., 10.],
        rotation: [0., 0., 0., 1.],
        linear_velocity: [1., 0., 3.],
        angular_velocity: [0., 0., 2.],
    };
    set_model_state(&mut sim, clone, target).unwrap();
    assert_eq!(native(&sim, body(&sim, clone)).velocity, [1., 2e-13, 3.]);
    // VelocityControl applies the old committed target, then latches pending
    // input in PostStep. The following advancing step applies the new target.
    sim.step_exact(1).unwrap();
    let first = native(
        &sim,
        sim.world()
            .get::<BodyHandle>(entity(&sim, "w::source::body"))
            .unwrap()
            .id(),
    );
    vector(first.velocity, [0.; 3]);
    vector(first.angular_velocity, [0.; 3]);
    sim.step_exact(1).unwrap();
    vector(
        native(
            &sim,
            sim.world()
                .get::<BodyHandle>(entity(&sim, "w::source::body"))
                .unwrap()
                .id(),
        )
        .velocity,
        [1., 2., 3.],
    );
    let second = "<collision name='other'><pose>0 0 -4 0 0 0</pose><geometry><sphere><radius>0.25</radius></sphere></geometry></collision>";
    let xml = document(SHAPES[1], false, "").replacen(VISUAL, &format!("{second}{VISUAL}"), 1);
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let mut one = request(&sdf, COM, 1.);
    one.links[0].collisions.truncate(1);
    let before = snapshot(&sim);
    assert!(matches!(
        prepare_model_template(&sim, &sdf, &one),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim), before);
    let compound = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    let runtime = spawn_model_template(&mut sim, &compound, cycle([60., 20., 30.])).unwrap();
    let fresh = runtime_document(runtime, SHAPES[1], COM);
    let before = snapshot(&sim);
    assert!(matches!(
        prepare_model_template(&sim, &fresh, &fresh.worlds[0].models[0]),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim), before);
    let mut centered = fresh.worlds[0].models[0].clone();
    centered.links[0].inertial.pose.transform.translation = [0.; 3];
    let retained = prepare_model_template(&sim, &fresh, &centered).unwrap();
    spawn_model_template(&mut sim, &retained, cycle([100., 20., 30.])).unwrap();
    // TPE's actual missing BodyHandle remains earlier than all caller COM rules.
    let xml = document(SHAPES[0], false, "").replace(LOCAL, "0 0 0 0 0 0");
    let sdf = parsed(&xml);
    let tpe = load_world_with_sdf_step(&xml, "offset-tpe", "w", &NoResources, PhysicsEngine::Tpe)
        .unwrap();
    let motions = |s: &Simulation| {
        s.world()
            .iter_entities()
            .filter_map(|e| {
                e.get::<gz_physics_tpe::Motion>()
                    .map(|v| (e.get::<EntityId>().unwrap().get(), *v.state()))
            })
            .collect::<BTreeMap<_, _>>()
    };
    let old = motions(&tpe);
    assert_eq!(old.len(), 2);
    let mut model = request(&sdf, COM, 1.);
    model.links[0].collisions[0].pose.transform.translation = [2., 0., 0.];
    assert!(matches!(
        prepare_model_template(&tpe, &sdf, &model),
        Err(WorldEditError::InvalidScene)
    ));
    assert_eq!(motions(&tpe), old);
}

#[test]
fn offset_static_contacts_dynamic_fall_and_camera_capacity_keep_owned_inputs() {
    let ball = "<model name='ball'><pose>10 0 4 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>0.1</ixx><iyy>0.1</iyy><izz>0.1</izz></inertia></inertial><collision name='solid'><geometry><sphere><radius>0.25</radius></sphere></geometry></collision></link></model>";
    for (index, shape) in SHAPES.into_iter().enumerate() {
        for support in [false, true] {
            let xml = document(shape, true, ball)
                .replace("<gravity>0 0 0</gravity>", "<gravity>0 0 -9.8</gravity>");
            let sdf = parsed(&xml);
            let mut sim = load(&xml);
            let template = prepare_model_template(&sim, &sdf, &request(&sdf, COM, 1.)).unwrap();
            let p = pose([if support { 8. } else { 20. }, 0., 1.], [0., 0., 0., 1.]);
            let model = spawn_model_template(&mut sim, &template, p).unwrap();
            let extent = if index == 0 { 0.75 } else { 0.5 };
            assert!(4. - 0.25 > 1. + extent);
            if support {
                near(
                    ray(&sim, [10., 0., 3.], [0., 0., -1.], 2., None).unwrap(),
                    2. - extent,
                );
            } else {
                assert_eq!(ray(&sim, [10., 0., 3.], [0., 0., -1.], 2., None), None);
            }
            sim.step_exact(1500).unwrap();
            let h = native(
                &sim,
                sim.world()
                    .get::<BodyHandle>(entity(&sim, "w::ball::body"))
                    .unwrap()
                    .id(),
            )
            .position[2];
            if support {
                assert!(
                    (h - (1. + extent + 0.25)).abs() < 0.01,
                    "shape={index}, height={h}"
                );
            } else {
                assert!(h < 0.);
            }
            assert_eq!(native(&sim, body(&sim, model)).position, p.translation);
        }
        let xml = document(shape, true, "")
            .replace("<gravity>0 0 0</gravity>", "<gravity>0 0 -9.8</gravity>");
        let sdf = parsed(&xml);
        let mut sim = load(&xml);
        let mut dynamic = request(&sdf, COM, 1.);
        dynamic.is_static = false;
        let template = prepare_model_template(&sim, &sdf, &dynamic).unwrap();
        let model = spawn_model_template(&mut sim, &template, cycle([10., 20., 10.])).unwrap();
        sim.step_exact(1500).unwrap();
        let state = native(&sim, body(&sim, model));
        assert!(state.position[2] < 0.);
        vector(state.angular_velocity, [0.; 3]);
    }
    let observer = "<model name='observer'><static>true</static><pose>-200 0 10 0 0 0</pose><link name='body'><collision name='solid'><geometry><box><size>1 1 1</size></box></geometry></collision><sensor name='logical' type='logical_camera'><update_rate>0</update_rate><logical_camera><near>0.1</near><far>1000</far><horizontal_fov>1.5</horizontal_fov><aspect_ratio>1</aspect_ratio></logical_camera></sensor></link></model>";
    let xml = document(SHAPES[2], false, observer);
    let sdf = parsed(&xml);
    let mut sim = load(&xml);
    let template = prepare_model_template(&sim, &sdf, &request(&sdf, COM, 1.)).unwrap();
    let mut last = 0;
    for i in 0..125 {
        last = spawn_model_template(&mut sim, &template, cycle([10. + 4. * i as f64, 20., 30.]))
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
    let old_body = body(&sim, last);
    let source_link = id(&sim, "w::source::body");
    queue_link_world_velocity(&mut sim, source_link, [1., 0., 0.], [0.; 3]).unwrap();
    let before = snapshot(&sim);
    assert!(matches!(
        spawn_model_template(&mut sim, &template, cycle([600., 20., 30.])),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(snapshot(&sim), before);
    remove_model(&mut sim, last).unwrap();
    let recovered = spawn_model_template(&mut sim, &template, cycle([600., 20., 30.])).unwrap();
    assert_eq!(recovered, before.next);
    assert_eq!(body(&sim, recovered).raw(), old_body.raw() + 1);
    sim.step_exact(1).unwrap();
    vector(
        native(
            &sim,
            sim.world()
                .get::<BodyHandle>(entity(&sim, "w::source::body"))
                .unwrap()
                .id(),
        )
        .velocity,
        [1., 0., 0.],
    );
}
