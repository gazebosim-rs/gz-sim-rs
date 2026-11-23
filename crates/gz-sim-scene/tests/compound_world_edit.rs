// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicit compound edits retain loaded construction and native query geometry.
use bevy_ecs::prelude::*;
use gz_physics::{BodyId, BodyState, PhysicsBackend, PhysicsError};
use gz_sim_core::{EntityId, Simulation, assign_id, entity_by_id, reserve_id, restore_id};
use gz_sim_scene::{
    BodyHandle, EntityKind, LocalName, LocalPose, ModelState, Name, PhysicsEngine,
    VelocityControlConfiguration, VisualGeometry, WorldEditError, WorldPose,
    load_world_with_sdf_step, model_state, model_visual_pose, prepare_model_template,
    queue_link_world_velocity, remove_model, reset_loaded_world, set_model_state,
    set_velocity_control_command, spawn_model_template, velocity_control_binding_matches,
};
use sdformat::NoResources;
use std::{collections::BTreeMap, sync::Arc, time::Duration};

const INERTIA: &str =
    "<inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial>";
const VISUAL: &str = "<visual name='view'><pose>0 1 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual>";
const CONTROL: &str = "<plugin filename='gz-sim-velocity-control-system' name='gz::sim::systems::VelocityControl'><initial_linear>1 0 0</initial_linear></plugin>";
fn collisions(count: usize) -> String {
    (0..count).map(|i| {
        let shape = match i % 4 {
            0 => "<sphere><radius>0.5</radius></sphere>",
            1 => "<cylinder><radius>0.5</radius><length>3</length></cylinder>",
            2 => "<box><size>1 0.5 1.5</size></box>",
            _ => "<capsule><radius>0.5</radius><length>1.5</length></capsule>",
        };
        let yaw = if i % 4 == 2 { std::f64::consts::FRAC_PI_2 } else { 0.3 };
        format!("<collision name='c{i}'><pose>{} 0 0 0 0 {yaw}</pose><geometry>{shape}</geometry></collision>", i as f64 * 4. - 6.)
    }).collect()
}
fn xml(count: usize, fixed: bool, controlled: bool, extra: &str) -> String {
    format!(
        "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='rig'><pose>1 2 10 0.2 0.3 0.4</pose><static>{fixed}</static>{}<link name='body'>{INERTIA}{}{VISUAL}{extra}</link></model><model name='survivor'><pose>100 0 10 0 0 0</pose><link name='body'>{INERTIA}<collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision>{VISUAL}</link></model></world></sdf>",
        if controlled { CONTROL } else { "" },
        collisions(count)
    )
}
fn load(text: &str) -> Simulation {
    load_world_with_sdf_step(
        text,
        "compound-edit",
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
fn native(sim: &Simulation, body: BodyId) -> BodyState {
    sim.world()
        .resource::<PhysicsBackend>()
        .state(body)
        .unwrap()
}
fn ids(sim: &Simulation) -> BTreeMap<u64, Entity> {
    sim.world()
        .iter_entities()
        .filter_map(|e| e.get::<EntityId>().map(|id| (id.get(), e.id())))
        .collect()
}
type Frames = BTreeMap<u64, ([f64; 3], [f64; 4], [f64; 3], [f64; 4])>;
fn frames(sim: &Simulation) -> Frames {
    sim.world()
        .iter_entities()
        .filter_map(|e| {
            let id = e.get::<EntityId>()?.get();
            let world = e.get::<WorldPose>()?;
            let local = e.get::<LocalPose>()?.0;
            Some((
                id,
                (
                    world.translation,
                    world.rotation,
                    local.translation,
                    local.rotation,
                ),
            ))
        })
        .collect()
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
fn target() -> ModelState {
    // This exact quaternion cycles local X->world Y, Y->Z, Z->X.
    ModelState {
        translation: [4., 5., 8.],
        rotation: [0.5; 4],
        linear_velocity: [1., -2., 3.],
        angular_velocity: [-0.5, 0.75, -1.25],
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
fn quaternion([x, y, z, w]: [f64; 4], [a, b, c, d]: [f64; 4]) -> [f64; 4] {
    [
        w * a + x * d + y * c - z * b,
        w * b - x * c + y * d + z * a,
        w * c + x * b - y * a + z * d,
        w * d - x * a - y * b - z * c,
    ]
}

#[test]
fn compound_state_edits_keep_six_axes_and_immediate_independent_child_queries() {
    for count in [2, 4, 16] {
        let mut sim = load(&xml(count, false, false, ""));
        let rig = id(&sim, "w::rig");
        let handle = body(&sim, "w::rig::body");
        let survivor = body(&sim, "w::survivor::body");
        let saved_survivor = native(&sim, survivor);
        let initial = model_state(&sim, rig).unwrap();
        let native_initial = native(&sim, handle);
        vector(initial.translation, native_initial.position);
        let saved_ids = ids(&sim);
        let clock = (sim.elapsed(), sim.epoch(), sim.is_paused());
        let locals: Vec<_> = (0..count)
            .map(|i| {
                sim.world()
                    .get::<LocalPose>(entity(&sim, &format!("w::rig::body::c{i}")))
                    .unwrap()
                    .0
            })
            .collect();
        assert_eq!(set_model_state(&mut sim, rig, target()).unwrap(), target());
        assert_eq!(model_state(&sim, rig).unwrap(), target());
        assert_eq!(native(&sim, survivor), saved_survivor);
        assert_eq!(ids(&sim), saved_ids);
        assert_eq!((sim.elapsed(), sim.epoch(), sim.is_paused()), clock);
        for (i, local) in locals.into_iter().enumerate() {
            let e = entity(&sim, &format!("w::rig::body::c{i}"));
            let world = sim.world().get::<WorldPose>(e).unwrap();
            let center = [4., 5. + i as f64 * 4. - 6., 8.];
            vector(world.translation, center);
            for (a, b) in world
                .rotation
                .into_iter()
                .zip(quaternion(target().rotation, local.rotation))
            {
                near(a, b);
            }
            let now = sim.world().get::<LocalPose>(e).unwrap().0;
            assert_eq!(
                (now.translation, now.rotation),
                (local.translation, local.rotation)
            );
            let hit = ray(
                &sim,
                [center[0] + 4., center[1], center[2]],
                [-1., 0., 0.],
                5.,
                None,
            )
            .unwrap();
            near(hit, [3.5, 2.5, 3.25, 2.75][i % 4]);
            assert!(
                ray(
                    &sim,
                    [center[0] + 4., center[1], center[2]],
                    [-1., 0., 0.],
                    5.,
                    Some(handle)
                )
                .is_none()
            );
        }
        // Cylinder radial section: 0.3^2+0.4^2=0.5^2, independent of its yaw.
        // Cast along Z so the adjacent box cannot contain the chord's origin.
        let center = [4., 3., 8.];
        near(
            ray(
                &sim,
                [center[0], center[1] + 0.3, center[2] + 4.],
                [0., 0., -1.],
                5.,
                None,
            )
            .unwrap(),
            3.6,
        );
        let original_ray = ray(
            &sim,
            [center[0], center[1] + 4., center[2] + 0.3],
            [0., -1., 0.],
            5.,
            None,
        );
        if count >= 4 {
            // The original origin is inside c2: the union's nearest hit is zero.
            assert_eq!(original_ray, Some(0.));
        } else {
            near(original_ray.unwrap(), 3.6);
        }
        assert!(ray(&sim, [8., 1., 8.], [-1., 0., 0.], 5., None).is_none());
        // A positive maximum is required even for an inside-solid query.
        near(ray(&sim, center, [1., 0., 0.], 0.125, None).unwrap(), 0.);
        near(
            ray(&sim, [6.5, 3., 8.], [-1., 0., 0.], 5., None).unwrap(),
            1.,
        );

        let (visual, projected) = model_visual_pose(
            &sim,
            rig,
            WorldPose {
                translation: [4., 5., 8.],
                rotation: [0.5; 4],
            },
        )
        .unwrap();
        vector(projected.translation, [4., 5., 9.]);
        vector(
            sim.world()
                .get::<WorldPose>(entity_by_id(sim.world(), visual).unwrap())
                .unwrap()
                .translation,
            projected.translation,
        );
        let before = model_state(&sim, rig).unwrap();
        let saved_frames = frames(&sim);
        for bad in 0..4 {
            let mut request = target();
            match bad {
                0 => request.translation[0] = f64::NAN,
                1 => request.rotation = [0.; 4],
                2 => request.angular_velocity[1] = f64::INFINITY,
                _ => request.translation[2] = f64::MAX,
            }
            assert!(set_model_state(&mut sim, rig, request).is_err());
            assert_eq!(model_state(&sim, rig).unwrap(), before);
            assert_eq!(native(&sim, survivor), saved_survivor);
            assert_eq!(ids(&sim), saved_ids);
            assert_eq!(frames(&sim), saved_frames);
        }
    }
    // Exact inclusive/short bounds use binary geometry without composed rotations.
    let endpoint_xml = xml(2, false, false, "")
        .replacen(
            "<pose>1 2 10 0.2 0.3 0.4</pose>",
            "<pose>0 0 0 0 0 0</pose>",
            1,
        )
        .replacen("<pose>-6 0 0 0 0 0.3</pose>", "<pose>0 0 0 0 0 0</pose>", 1);
    let endpoint = load(&endpoint_xml);
    let owner = body(&endpoint, "w::rig::body");
    assert_eq!(
        ray(&endpoint, [1.5, 0., 0.], [-1., 0., 0.], 1., None),
        Some(1.)
    );
    assert!(ray(&endpoint, [1.5, 0., 0.], [-1., 0., 0.], 0.999, None).is_none());
    assert_eq!(ray(&endpoint, [0.; 3], [1., 0., 0.], 0.125, None), Some(0.));
    assert!(ray(&endpoint, [1.5, 0., 0.], [-1., 0., 0.], 1., Some(owner)).is_none());
    // A zero from the first solid does not hide a later child's numeric refusal.
    let text = xml(2, false, false, "")
        .replacen(
            "<pose>1 2 10 0.2 0.3 0.4</pose>",
            "<pose>0 0 0 0 0 0</pose>",
            1,
        )
        .replacen("<pose>-6 0 0 0 0 0.3</pose>", "<pose>0 0 0 0 0 0</pose>", 1)
        .replacen("<pose>-2 0 0 0 0 0.3</pose>", "<pose>0 0 0 0 0 0</pose>", 1);
    let sim = load(&text);
    let handle = body(&sim, "w::rig::body");
    let old = native(&sim, handle);
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .cast_ray([1e-200, 0., 0.], [1., 0., 0.], 1., None),
        Err(PhysicsError::InvalidValue)
    );
    assert_eq!(native(&sim, handle), old);
}

#[test]
fn compound_state_static_anchor_both_resets_removal_and_spawn_boundary_remain_owned() {
    for fixed in [false, true] {
        let text = xml(4, fixed, false, "");
        let mut sim = load(&text);
        let rig = id(&sim, "w::rig");
        let original = model_state(&sim, rig).unwrap();
        let saved_ids: Vec<_> = ids(&sim).keys().copied().collect();
        let parsed = sdformat::load_str(&text, "compound-template", &NoResources).unwrap();
        // The same loaded static/dynamic compound now supplies a passive template.
        let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
        let clone = spawn_model_template(
            &mut sim,
            &template,
            WorldPose {
                translation: [200., 0., 20.],
                rotation: [0., 0., 0., 1.],
            },
        )
        .unwrap();
        assert_eq!(ids(&sim).len(), saved_ids.len() + 7);
        let clone_link = entity_by_id(sim.world(), clone + 1).unwrap();
        let clone_body = sim.world().get::<BodyHandle>(clone_link).unwrap().id();
        vector(native(&sim, clone_body).position, [200., 0., 20.]);
        // C0 is a sphere at local x=-6. Every other child and loaded body is
        // outside this vertical segment, including the cylinder at local x=-2.
        near(
            ray(&sim, [194., 0., 22.], [0., 0., -1.], 2.1, None).unwrap(),
            1.5,
        );
        assert!(ray(&sim, [194., 0., 22.], [0., 0., -1.], 2.1, Some(clone_body)).is_none());
        let source_visual = sim
            .world()
            .get::<VisualGeometry>(entity(&sim, "w::rig::body::view"))
            .unwrap();
        let clone_visual = sim
            .world()
            .get::<VisualGeometry>(entity_by_id(sim.world(), clone + 3).unwrap())
            .unwrap();
        assert!(Arc::ptr_eq(&source_visual.0, &clone_visual.0));
        remove_model(&mut sim, clone).unwrap();
        assert_eq!(ids(&sim).keys().copied().collect::<Vec<_>>(), saved_ids);
        assert_eq!(model_state(&sim, rig).unwrap(), original);
        let mut request = target();
        if fixed {
            request.linear_velocity = [0.; 3];
            request.angular_velocity = [0.; 3];
        }
        for _ in 0..2 {
            assert_eq!(set_model_state(&mut sim, rig, request).unwrap(), request);
        }
        if !fixed {
            let local_before: Vec<_> = (0..4)
                .map(|i| {
                    let local = sim
                        .world()
                        .get::<LocalPose>(entity(&sim, &format!("w::rig::body::c{i}")))
                        .unwrap()
                        .0;
                    (local.translation, local.rotation)
                })
                .collect();
            sim.step_exact(2).unwrap();
            let moved = model_state(&sim, rig).unwrap();
            assert_ne!(moved.translation, request.translation);
            vector(
                moved.translation,
                native(&sim, body(&sim, "w::rig::body")).position,
            );
            for (i, original_local) in local_before.into_iter().enumerate() {
                let local = sim
                    .world()
                    .get::<LocalPose>(entity(&sim, &format!("w::rig::body::c{i}")))
                    .unwrap()
                    .0;
                assert_eq!((local.translation, local.rotation), original_local);
            }
        }
        sim.reset().unwrap();
        assert_eq!(model_state(&sim, rig).unwrap(), original);
        set_model_state(&mut sim, rig, request).unwrap();
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(model_state(&sim, rig).unwrap(), original);
        assert_eq!(ids(&sim).keys().copied().collect::<Vec<_>>(), saved_ids);
        // Loaded reset uses fresh Bevy entities, retaining original logical IDs.
        let survivor = id(&sim, "w::survivor");
        let survivor_state = model_state(&sim, survivor).unwrap();
        remove_model(&mut sim, rig).unwrap();
        assert!(model_state(&sim, rig).is_err());
        assert!(set_model_state(&mut sim, rig, request).is_err());
        assert_eq!(model_state(&sim, survivor).unwrap(), survivor_state);
        let fresh = sim.world_mut().spawn_empty().id();
        let high = assign_id(sim.world_mut(), fresh).unwrap();
        assert!(high > *saved_ids.last().unwrap());
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(id(&sim, "w::rig"), rig);
        assert_eq!(model_state(&sim, rig).unwrap(), original);
    }
    // Existing singleton templates and runtime cleanup remain the four-node path.
    let text = xml(1, false, false, "").replacen(
        "<pose>-6 0 0 0 0 0.3</pose>",
        "<pose>0 0 0 0 0 0</pose>",
        1,
    );
    let mut sim = load(&text);
    let parsed = sdformat::load_str(&text, "singleton-template", &NoResources).unwrap();
    let template = prepare_model_template(&sim, &parsed, &parsed.worlds[0].models[0]).unwrap();
    let spawned = spawn_model_template(
        &mut sim,
        &template,
        WorldPose {
            translation: [20., 0., 5.],
            rotation: [0., 0., 0., 1.],
        },
    )
    .unwrap();
    assert!(model_state(&sim, spawned).is_ok());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(model_state(&sim, spawned).is_err());
}

#[test]
fn sensorless_compound_controller_getters_never_release_immutable_setter_ownership() {
    let mut sim = load(&xml(4, false, true, ""));
    let rig = id(&sim, "w::rig");
    let handle = body(&sim, "w::rig::body");
    let baseline = model_state(&sim, rig).unwrap();
    assert!(velocity_control_binding_matches(&sim, rig));
    set_velocity_control_command(&mut sim, rig, [2., 3., 4.], [0.25, -0.5, 0.75]).unwrap();
    let old = native(&sim, handle);
    let saved_ids = ids(&sim);
    let clock = (sim.elapsed(), sim.epoch());
    assert!(matches!(
        set_model_state(&mut sim, rig, target()),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(native(&sim, handle), old);
    assert_eq!(model_state(&sim, rig).unwrap(), baseline);
    let e = entity_by_id(sim.world(), rig).unwrap();
    let mut drift = sim
        .world()
        .get::<VelocityControlConfiguration>(e)
        .unwrap()
        .clone();
    drift.model_name = "w::other".into();
    sim.world_mut().entity_mut(e).insert(drift);
    assert!(!velocity_control_binding_matches(&sim, rig));
    assert!(model_state(&sim, rig).is_ok());
    assert!(matches!(
        set_model_state(&mut sim, rig, target()),
        Err(WorldEditError::UnsupportedModel)
    ));
    sim.world_mut()
        .entity_mut(e)
        .remove::<VelocityControlConfiguration>();
    assert!(matches!(
        set_model_state(&mut sim, rig, target()),
        Err(WorldEditError::UnsupportedModel)
    ));
    assert_eq!(native(&sim, handle), old);
    assert_eq!(ids(&sim), saved_ids);
    assert_eq!((sim.elapsed(), sim.epoch()), clock);
    let link = id(&sim, "w::rig::body");
    assert!(queue_link_world_velocity(&mut sim, link, [0.; 3], [0.; 3]).is_err());
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(velocity_control_binding_matches(&sim, rig));
    sim.step_exact(1).unwrap();
    vector(
        native(&sim, handle).velocity,
        [
            0.4_f64.cos() * 0.3_f64.cos(),
            0.4_f64.sin() * 0.3_f64.cos(),
            -0.3_f64.sin(),
        ],
    );
}

#[test]
fn original_compound_membership_and_metadata_drift_refuse_without_native_mutation() {
    for fault in 0..13 {
        let mut sim = load(&xml(2, false, false, ""));
        let rig = id(&sim, "w::rig");
        let handle = body(&sim, "w::rig::body");
        let survivor = body(&sim, "w::survivor::body");
        let link = entity(&sim, "w::rig::body");
        let child = entity(&sim, "w::rig::body::c1");
        let visual = entity(&sim, "w::rig::body::view");
        let old = native(&sim, handle);
        let old_survivor = native(&sim, survivor);
        match fault {
            0 => {
                sim.world_mut().entity_mut(child).despawn();
            } // downgrade to collision+visual
            1 => {
                sim.world_mut().entity_mut(child).insert(EntityKind::Sensor);
            }
            2 => {
                sim.world_mut()
                    .entity_mut(child)
                    .get_mut::<Name>()
                    .unwrap()
                    .0
                    .push_str("_forged");
            }
            3 => {
                sim.world_mut()
                    .entity_mut(child)
                    .get_mut::<LocalName>()
                    .unwrap()
                    .0
                    .push_str("_forged");
            }
            4 => {
                let other = entity(&sim, "w::survivor::body");
                sim.world_mut().entity_mut(child).insert(ChildOf(other));
            }
            5 => {
                sim.world_mut()
                    .entity_mut(child)
                    .get_mut::<LocalPose>()
                    .unwrap()
                    .0
                    .translation[0] += 0.25;
            }
            6 => {
                sim.world_mut().entity_mut(child).remove::<EntityId>();
            }
            7 => {
                let geometry = sim
                    .world()
                    .get::<VisualGeometry>(visual)
                    .unwrap()
                    .0
                    .as_ref()
                    .clone();
                sim.world_mut()
                    .entity_mut(visual)
                    .insert(VisualGeometry(Arc::new(geometry)));
            }
            8 => {
                let extra = sim
                    .world_mut()
                    .spawn((ChildOf(child), Name("leaf-extra".into())))
                    .id();
                assign_id(sim.world_mut(), extra).unwrap();
            }
            9 => {
                let other = entity(&sim, "w::survivor::body");
                let a = sim
                    .world_mut()
                    .entity_mut(link)
                    .take::<BodyHandle>()
                    .unwrap();
                let b = sim
                    .world_mut()
                    .entity_mut(other)
                    .take::<BodyHandle>()
                    .unwrap();
                sim.world_mut().entity_mut(link).insert(b);
                sim.world_mut().entity_mut(other).insert(a);
            }
            10 => {
                let reserved = reserve_id(sim.world(), child).unwrap();
                let local = *sim.world().get::<LocalPose>(child).unwrap();
                let world = *sim.world().get::<WorldPose>(child).unwrap();
                sim.world_mut().entity_mut(child).despawn();
                let forged = sim
                    .world_mut()
                    .spawn((
                        ChildOf(link),
                        Name("w::rig::body::c1".into()),
                        LocalName("c1".into()),
                        EntityKind::Collision,
                        local,
                        world,
                    ))
                    .id();
                restore_id(sim.world_mut(), forged, &reserved).unwrap();
            }
            _ => {
                let other = entity(&sim, "w::survivor::body");
                let foreign = sim
                    .world_mut()
                    .entity_mut(other)
                    .take::<BodyHandle>()
                    .unwrap();
                let target = if fault == 11 {
                    child
                } else {
                    entity_by_id(sim.world(), rig).unwrap()
                };
                sim.world_mut().entity_mut(target).insert(foreign);
            }
        }
        let after_tamper = ids(&sim);
        let saved_frames = frames(&sim);
        let clock = (sim.elapsed(), sim.epoch());
        assert!(model_state(&sim, rig).is_err(), "fault {fault}");
        assert!(
            set_model_state(&mut sim, rig, target()).is_err(),
            "fault {fault}"
        );
        assert_eq!(native(&sim, handle), old);
        assert_eq!(native(&sim, survivor), old_survivor);
        assert_eq!(ids(&sim), after_tamper);
        assert_eq!(frames(&sim), saved_frames);
        assert_eq!((sim.elapsed(), sim.epoch()), clock);
    }
}

#[test]
fn original_sensor_singletons_cannot_be_upgraded_and_closed_compound_domains_stay_closed() {
    let imu = "<sensor name='imu' type='imu'><always_on>true</always_on><update_rate>100</update_rate></sensor>";
    let mut sim = load(&xml(1, false, false, imu));
    let rig = id(&sim, "w::rig");
    let handle = body(&sim, "w::rig::body");
    let original = native(&sim, handle);
    let sensor = entity(&sim, "w::rig::body::imu");
    sim.world_mut()
        .entity_mut(sensor)
        .insert(EntityKind::Collision);
    assert!(model_state(&sim, rig).is_err());
    assert!(set_model_state(&mut sim, rig, target()).is_err());
    assert_eq!(native(&sim, handle), original);
    for text in [xml(2,false,false,imu), xml(2,false,false,"").replacen("<link name='body'>","<link name='body'><pose>0.25 0 0 0 0 0</pose>",1), xml(2,false,false,"").replacen(VISUAL,"",1), xml(2,false,false,"").replacen(VISUAL,&format!("{VISUAL}<visual name='extra'><geometry><sphere><radius>1</radius></sphere></geometry></visual>"),1)] {
        let mut sim=load(&text);let rig=id(&sim,"w::rig");let handle=body(&sim,"w::rig::body");let old=native(&sim,handle);
        assert!(model_state(&sim,rig).is_err());assert!(set_model_state(&mut sim,rig,target()).is_err());assert_eq!(native(&sim,handle),old);
    }
    // Loaded translated COM is physical; runtime/edit ownership boundaries below remain closed.
    let text = xml(1, false, false, "")
        .replacen("<pose>-6 0 0 0 0 0.3</pose>", "<pose>0 0 0 0 0 0</pose>", 1)
        .replacen("<inertial>", "<inertial><pose>0.25 0 0 0 0 0</pose>", 1);
    m104_mass_response(
        load_world_with_sdf_step(
            &text,
            "offset-com-refusal",
            "w",
            &NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap(),
        &text,
        "w::rig::body",
    );
    let admitted = text.replacen(
        "<pose>0.25 0 0 0 0 0</pose>",
        "<pose>0.0000000000001 0 0 0 0 0</pose>",
        1,
    );
    let mut sim = load(&admitted);
    let rig = id(&sim, "w::rig");
    set_model_state(&mut sim, rig, target()).unwrap();
    assert_eq!(model_state(&sim, rig).unwrap(), target());
    let native = native(&sim, body(&sim, "w::rig::body"));
    // Q cycles the COM onto +Y: omega cross COM = [1.25e-13, 0, -0.5e-13].
    assert_eq!(native.velocity, [1. + 1.25e-13, -2., 3. - 0.5e-13]);
    assert_ne!(native.velocity, target().linear_velocity);
    assert_eq!(sim.elapsed(), Duration::ZERO);
}

// M104: the exact formerly refused XML must produce a real mass-frame response.
fn m104_mass_response(mut sim: Simulation, text: &str, link_name: &str) {
    sim.initialize_schedules().unwrap();
    let dom = sdformat::load_str(text, "retained-resolved-com", &sdformat::NoResources).unwrap();
    let link = dom
        .worlds
        .iter()
        .flat_map(|w| &w.models)
        .flat_map(|m| &m.links)
        .find(|l| l.scoped_name == link_name)
        .unwrap();
    let mass = link.inertial.mass;
    assert!(mass.is_finite() && mass > 0.);
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<gz_sim_scene::Name>()
                .is_some_and(|n| n.0 == link_name)
        })
        .unwrap();
    let public = entity.get::<gz_sim_core::EntityId>().unwrap().get();
    let native = entity.get::<gz_sim_scene::BodyHandle>().unwrap().id();
    assert_eq!(
        sim.world()
            .resource::<gz_physics::PhysicsBackend>()
            .is_dynamic(native),
        Some(true)
    );
    let before = sim
        .world()
        .resource::<gz_physics::PhysicsBackend>()
        .state(native)
        .unwrap();
    gz_sim_scene::queue_link_world_wrench(&mut sim, public, [mass, 0., 0.], [0.; 3]).unwrap();
    sim.step_exact(1).unwrap();
    let after = sim
        .world()
        .resource::<gz_physics::PhysicsBackend>()
        .state(native)
        .unwrap();
    assert!(
        (after.velocity[0] - before.velocity[0] - sim.step_duration().as_secs_f64()).abs() < 1e-11
    );
    assert!(
        after
            .position
            .into_iter()
            .chain(after.velocity)
            .chain(after.angular_velocity)
            .all(f64::is_finite)
    );
    for (a, b) in after
        .angular_velocity
        .into_iter()
        .zip(before.angular_velocity)
    {
        assert!((a - b).abs() < 1e-11);
    }
}
