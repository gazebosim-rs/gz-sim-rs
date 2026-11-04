// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_sim_core::{StepFault, next_entity_id};
use gz_sim_proto::msgs::WorldReset;
use gz_sim_scene::{
    BodyHandle, EntityKind, JointHandle, LocalPose, ModelState, ModelTemplate, Name, PhysicsEngine,
    RemoveError, Velocity, WorldEditError, WorldPose, load_world_with_sdf_step, model_state,
    prepare_model_template,
};
use std::collections::BTreeMap;

fn loaded(with_joint: bool) -> (Server, ModelTemplate, u64, u64) {
    loaded_variant(with_joint, false)
}
fn loaded_variant(with_joint: bool, camera: bool) -> (Server, ModelTemplate, u64, u64) {
    let standalone = |name: &str, x: u8, fixed: bool| {
        format!(
            "<model name='{name}'><static>{fixed}</static><pose>{x} 0 0 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='c'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='v'><geometry><box><size>1 1 1</size></box></geometry><material><diffuse>0.2 0.4 0.6 1</diffuse></material></visual></link></model>"
        )
    };
    let joint = if with_joint {
        "<model name='jointed'><pose>20 0 0 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='c'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision></link><joint name='hinge' type='revolute'><parent>world</parent><child>body</child><axis><xyz>0 0 1</xyz></axis></joint></model>"
    } else {
        ""
    };
    let camera_fixture = include_str!("../../../gz-sim-scene/tests/fixtures/camera-binding.sdf");
    let camera_model = if camera {
        &camera_fixture[camera_fixture.find("<model ").unwrap()
            ..camera_fixture.rfind("</model>").unwrap() + "</model>".len()]
    } else {
        ""
    };
    let sensors = if camera {
        "<plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>"
    } else {
        ""
    };
    let xml = format!(
        "<sdf version='1.12'><world name='default'>{sensors}<gravity>0 0 0</gravity><physics name='p' type='ignored'><max_step_size>0.001</max_step_size></physics>{}{}{joint}{camera_model}</world></sdf>",
        standalone("a", 0, false),
        standalone("b", 10, true)
    );
    let source = sdformat::load_str(&xml, "server-edit.sdf", &sdformat::NoResources).unwrap();
    let simulation = load_world_with_sdf_step(
        &xml,
        "server-edit.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let template =
        prepare_model_template(&simulation, &source, &source.worlds[0].models[0]).unwrap();
    let lookup = |name: &str| {
        simulation
            .world()
            .iter_entities()
            .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get()
    };
    let a = lookup("default::a");
    let b = lookup("default::b");
    let server = if camera {
        let id = simulation
            .world()
            .iter_entities()
            .find(|entity| entity.contains::<gz_sim_scene::CameraSensor>())
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        Server::with_external_camera_output(
            simulation,
            "default",
            config(),
            isolated_zenoh_config(),
            NonZeroUsize::new(4).unwrap(),
            id,
        )
    } else {
        isolated_server(
            simulation,
            "default",
            config(),
            NonZeroUsize::new(4).unwrap(),
        )
    }
    .unwrap();
    (server, template, a, b)
}

fn pose() -> WorldPose {
    WorldPose {
        translation: [5.0, 6.0, 7.0],
        rotation: [0.0, 0.0, 1.0, 0.0],
    }
}
fn replacement() -> ModelState {
    ModelState {
        translation: [4.0, 5.0, 6.0],
        rotation: [0.0, 0.0, 1.0, 0.0],
        linear_velocity: [0.5, 0.0, 0.0],
        angular_velocity: [0.0, 0.0, 0.2],
    }
}

#[derive(Debug, PartialEq)]
struct Snapshot {
    clock: (u64, Duration, bool),
    admission: (bool, bool, bool, Option<UpdateInfo>, bool),
    next: u64,
    entities: BTreeMap<u64, (Entity, String)>,
    native: BTreeMap<BodyId, Option<BodyState>>,
}
fn snapshot(server: &Server) -> Snapshot {
    let world = server.simulation().world();
    Snapshot {
        clock: (
            server.simulation().epoch(),
            server.simulation().elapsed(),
            server.simulation().is_paused(),
        ),
        admission: (
            server.is_shutdown,
            server.reset_failed,
            server.pending_reset,
            server.prepared_reset,
            server.simulation().fault().is_some(),
        ),
        next: next_entity_id(world).unwrap(),
        entities: world
            .iter_entities()
            .filter_map(|entity| {
                entity.get::<EntityId>().map(|id| {
                    (
                        id.get(),
                        (
                            entity.id(),
                            format!(
                                "{:?}/{:?}/{:?}/{:?}/{:?}/{:?}/{:?}",
                                entity.get::<Name>(),
                                entity.get::<EntityKind>(),
                                entity
                                    .get::<bevy_ecs::hierarchy::ChildOf>()
                                    .map(bevy_ecs::hierarchy::ChildOf::parent),
                                entity.get::<LocalPose>(),
                                entity.get::<WorldPose>(),
                                entity.get::<Velocity>(),
                                entity
                                    .get::<JointHandle>()
                                    .map(|handle| (handle.id(), handle.child()))
                            ),
                        ),
                    )
                })
            })
            .collect(),
        native: world
            .iter_entities()
            .filter_map(|entity| {
                entity.get::<BodyHandle>().map(|handle| {
                    (
                        handle.id(),
                        world.resource::<PhysicsBackend>().state(handle.id()),
                    )
                })
            })
            .collect(),
    }
}

#[derive(Clone, Copy, Debug)]
enum Guard {
    ResetBoundary,
    Closed,
    ResetFailed,
    Faulted,
    Camera,
}
fn guarded(server: &mut Server, template: &ModelTemplate, model: u64, guard: Guard) {
    let before = snapshot(server);
    for operation in 0..3 {
        let result = match operation {
            0 => server.spawn_model_template(template, pose()).map(|_| ()),
            1 => server.set_model_state(model, replacement()).map(|_| ()),
            _ => server.remove_editable_model(model).map(|_| ()),
        };
        let error = result.unwrap_err();
        assert!(
            matches!(
                (guard, &error),
                (Guard::ResetBoundary, Error::IoPreparationRequired)
                    | (Guard::Closed, Error::WorldClosed)
                    | (Guard::ResetFailed, Error::ResetStreams)
                    | (Guard::Camera, Error::CameraWorldEdit)
                    | (Guard::Faulted, Error::Step(StepError::Faulted))
            ),
            "{guard:?}: {error:?}"
        );
        assert_eq!(snapshot(server), before, "{guard:?}, operation {operation}");
    }
}

#[test]
fn focused_edits_preserve_unrelated_bodies_ids_and_clock_and_remove_the_created_model() {
    let (mut server, template, a, b) = loaded(false);
    server.update().unwrap();
    server.update().unwrap();
    let before = snapshot(&server);
    let other = model_state(server.simulation(), b).unwrap();
    let unrelated: Vec<_> = server
        .simulation()
        .world()
        .iter_entities()
        .filter_map(|entity| {
            entity
                .get::<Name>()
                .filter(|name| name.0 == "default::b" || name.0.starts_with("default::b::"))
                .map(|_| entity.get::<EntityId>().unwrap().get())
        })
        .collect();
    let clone = server.spawn_model_template(&template, pose()).unwrap();
    assert_eq!(clone, before.next);
    let high_water = next_entity_id(server.simulation().world()).unwrap();
    assert_eq!(high_water, clone + 4);
    assert_eq!(
        model_state(server.simulation(), clone).unwrap().translation,
        pose().translation
    );
    let clone_link = entity_by_id(server.simulation().world(), clone + 1).unwrap();
    // Two initial native bodies precede the first actual runtime allocation.
    assert_eq!(
        server
            .simulation()
            .world()
            .get::<BodyHandle>(clone_link)
            .unwrap()
            .id()
            .raw(),
        2
    );
    assert_eq!(
        server.set_model_state(a, replacement()).unwrap(),
        replacement()
    );
    assert_eq!(model_state(server.simulation(), a).unwrap(), replacement());
    assert_eq!(model_state(server.simulation(), b).unwrap(), other);
    let after = snapshot(&server);
    assert_eq!(after.clock, before.clock);
    for (id, (entity, _)) in &before.entities {
        assert_eq!(
            entity_by_id(server.simulation().world(), *id),
            Some(*entity)
        );
    }
    for id in unrelated {
        assert_eq!(after.entities[&id], before.entities[&id]);
    }
    let b_link = server
        .simulation()
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == "default::b::body")
        })
        .unwrap();
    let other_body = b_link.get::<BodyHandle>().unwrap().id();
    assert_eq!(after.native[&other_body], before.native[&other_body]);
    assert_eq!(
        server.remove_editable_model(clone).unwrap(),
        (clone..clone + 4).collect::<Vec<_>>()
    );
    for id in clone..clone + 4 {
        assert!(entity_by_id(server.simulation().world(), id).is_none());
    }
    assert_eq!(
        next_entity_id(server.simulation().world()).unwrap(),
        high_water
    );
    assert_eq!(model_state(server.simulation(), b).unwrap(), other);
    assert_eq!(snapshot(&server).clock, before.clock);
}

#[test]
fn pending_and_prepared_reset_refuse_every_edit_before_loaded_reset_tick_reopens_admission() {
    let (mut server, template, a, _) = loaded(false);
    server.update().unwrap();
    let clone = server.spawn_model_template(&template, pose()).unwrap();
    let high_water = next_entity_id(server.simulation().world()).unwrap();
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    assert!(server.pending_reset && server.prepared_reset.is_none());
    guarded(&mut server, &template, a, Guard::ResetBoundary);
    server.prepare_io().unwrap();
    assert!(!server.pending_reset && server.prepared_reset.is_some());
    for id in clone..clone + 4 {
        assert!(entity_by_id(server.simulation().world(), id).is_none());
    }
    assert_eq!(
        next_entity_id(server.simulation().world()).unwrap(),
        high_water
    );
    guarded(&mut server, &template, a, Guard::ResetBoundary);
    let info = server.tick().unwrap();
    assert_eq!(
        (info.epoch, info.sim_time, info.dt, info.paused),
        (1, Duration::ZERO, Duration::ZERO, true)
    );
    assert!(server.prepared_reset.is_none());
    let restored = server.spawn_model_template(&template, pose()).unwrap();
    assert_eq!(restored, high_water);
    assert!(restored > clone);
    server.remove_editable_model(restored).unwrap();
    server.set_model_state(a, replacement()).unwrap();
}

#[test]
fn shutdown_reset_failure_and_actual_step_fault_keep_all_focused_edits_inert() {
    for guard in [Guard::Closed, Guard::ResetFailed, Guard::Faulted] {
        let (mut server, template, a, _) = loaded(false);
        server.update().unwrap();
        match guard {
            Guard::Closed => server.shutdown(),
            // Exercise this private terminal state directly; its cause belongs
            // to the existing real reset-stream failure regressions.
            Guard::ResetFailed => server.reset_failed = true,
            Guard::Faulted => server
                .simulation
                .world()
                .resource::<StepFault>()
                .report(std::io::Error::other("world-edit admission fault")),
            Guard::ResetBoundary => unreachable!("not a terminal-state case"),
            Guard::Camera => unreachable!("not a terminal-state case"),
        }
        guarded(&mut server, &template, a, guard);
    }
}

#[test]
fn externally_owned_camera_refuses_all_edits_without_mutating_the_world() {
    // Like camera_ownership's trust-boundary control, this installs the explicit
    // external owner without starting a renderer or claiming image execution.
    let (mut server, template, a, _) = loaded_variant(false, true);
    assert_eq!(
        server
            .simulation()
            .world()
            .iter_entities()
            .filter(|entity| entity.contains::<gz_sim_scene::CameraSensor>())
            .count(),
        1
    );
    guarded(&mut server, &template, a, Guard::Camera);
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    guarded(&mut server, &template, a, Guard::ResetBoundary);
    server.shutdown();
    guarded(&mut server, &template, a, Guard::Closed);
}

#[test]
fn typed_scene_and_global_removal_errors_preserve_the_last_committed_world() {
    let (mut server, _, a, _) = loaded(true);
    let before = snapshot(&server);
    assert!(matches!(
        server.set_model_state(u64::MAX, replacement()),
        Err(Error::WorldEdit(WorldEditError::UnsupportedModel))
    ));
    let mut invalid = replacement();
    invalid.rotation = [0.0; 4];
    assert!(matches!(
        server.set_model_state(a, invalid),
        Err(Error::WorldEdit(WorldEditError::InvalidState))
    ));
    assert_eq!(snapshot(&server), before);
    let joint = server
        .simulation
        .world()
        .iter_entities()
        .find(|entity| entity.contains::<JointHandle>())
        .unwrap()
        .id();
    server
        .simulation
        .world_mut()
        .entity_mut(joint)
        .remove::<JointHandle>();
    let damaged = snapshot(&server);
    assert!(matches!(
        server.remove_editable_model(a),
        Err(Error::WorldRemove(RemoveError::InvalidPhysics))
    ));
    assert_eq!(snapshot(&server), damaged);
}
