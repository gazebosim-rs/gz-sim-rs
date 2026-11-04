// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Authored M10 controls; run with the completed delivery's consolidated gate.
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, ModelState, Name, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
    model_state, model_visual_pose, remove_model, reset_loaded_world, set_model_state,
};
use sdformat::NoResources;

fn scene(extra: &str, engine: PhysicsEngine) -> Simulation {
    let model = |name: &str, extra: &str| {
        format!(
            "<model name='{name}'><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='c'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='v'><pose>0.25 0 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></visual>{extra}</link></model>"
        )
    };
    load_world_with_sdf_step(
        &format!(
            "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity>{}{}</world></sdf>",
            model("a", extra),
            model("b", "")
        ),
        "world-edit.sdf",
        "w",
        &NoResources,
        engine,
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|entity| entity.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn replacement() -> ModelState {
    ModelState {
        translation: [4.0, 5.0, 6.0],
        rotation: [0.0, 0.0, 1.0, 0.0],
        linear_velocity: [1.0, 2.0, 3.0],
        angular_velocity: [0.0, 0.0, 2.0],
    }
}

#[test]
fn edits_publish_origin_and_attached_poses_without_advancing_or_touching_other_bodies() {
    let mut sim = scene("", PhysicsEngine::Rapier);
    sim.step_exact(2).unwrap();
    let a = id(&sim, "w::a");
    let b = id(&sim, "w::b");
    let before = model_state(&sim, b).unwrap();
    let clock = (sim.epoch(), sim.elapsed(), sim.is_paused());
    let ids: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<EntityId>().map(|id| (id.get(), entity.id())))
        .collect();
    assert_eq!(
        set_model_state(&mut sim, a, replacement()).unwrap(),
        replacement()
    );
    assert_eq!(model_state(&sim, a).unwrap(), replacement());
    assert_eq!(model_state(&sim, b).unwrap(), before);
    assert_eq!((sim.epoch(), sim.elapsed(), sim.is_paused()), clock);
    for (id, entity) in ids {
        assert_eq!(entity_by_id(sim.world(), id), Some(entity));
    }
    let visual = entity_by_id(sim.world(), id(&sim, "w::a::body::v")).unwrap();
    assert_eq!(
        sim.world().get::<WorldPose>(visual).unwrap().translation,
        [3.75, 5.0, 6.0]
    );
    let link = entity_by_id(sim.world(), id(&sim, "w::a::body")).unwrap();
    assert_eq!(
        sim.world().get::<Velocity>(link).unwrap().linear,
        [1.0, 2.0, 3.0]
    );
    let handle = sim.world().get::<BodyHandle>(link).unwrap().id();
    assert_eq!(
        sim.world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .unwrap()
            .position,
        [4.0, 5.0, 6.0]
    );
}

#[test]
fn invalid_requests_leave_native_and_ecs_state_exactly_unchanged() {
    let mut sim = scene("", PhysicsEngine::Rapier);
    let a = id(&sim, "w::a");
    let before = model_state(&sim, a).unwrap();
    for change in 0..5 {
        let mut state = replacement();
        match change {
            0 => state.translation[0] = f64::NAN,
            1 => state.rotation = [0.0; 4],
            2 => state.linear_velocity[1] = f64::INFINITY,
            3 => state.angular_velocity[2] = f64::NAN,
            _ => state.translation[0] = f64::MAX,
        }
        assert!(set_model_state(&mut sim, a, state).is_err());
        assert_eq!(model_state(&sim, a).unwrap(), before);
        let model = entity_by_id(sim.world(), a).unwrap();
        assert_eq!(
            sim.world().get::<WorldPose>(model).unwrap().translation,
            before.translation
        );
    }
    assert!(set_model_state(&mut sim, u64::MAX, replacement()).is_err());
    let visual = id(&sim, "w::a::body::v");
    assert!(set_model_state(&mut sim, visual, replacement()).is_err());
    assert_eq!(model_state(&sim, a).unwrap(), before);
}

#[test]
fn sensor_models_and_other_engines_are_not_silently_admitted() {
    let mut sim = scene(
        "<sensor name='imu' type='imu'><always_on>true</always_on><update_rate>100</update_rate></sensor>",
        PhysicsEngine::Rapier,
    );
    let a = id(&sim, "w::a");
    assert!(model_state(&sim, a).is_err());
    assert!(set_model_state(&mut sim, a, replacement()).is_err());
    let mut tpe = scene("", PhysicsEngine::Tpe);
    let a = id(&tpe, "w::a");
    assert!(model_state(&tpe, a).is_err());
    assert!(set_model_state(&mut tpe, a, replacement()).is_err());
}

#[test]
fn loaded_reset_and_removal_retain_the_existing_identity_contract() {
    let mut sim = scene("", PhysicsEngine::Rapier);
    let a = id(&sim, "w::a");
    let before = model_state(&sim, a).unwrap();
    set_model_state(&mut sim, a, replacement()).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(id(&sim, "w::a"), a);
    assert_eq!(model_state(&sim, a).unwrap(), before);
    remove_model(&mut sim, a).unwrap();
    assert!(model_state(&sim, a).is_err());
    assert!(set_model_state(&mut sim, a, replacement()).is_err());
    let b = id(&sim, "w::b");
    assert!(model_state(&sim, b).is_ok());
}

#[test]
fn explicit_fixed_pose_edit_survives_both_reset_contracts_without_desynchronization() {
    let mut sim = load_world_with_sdf_step("<sdf version='1.12'><world name='w'><model name='a'><static>true</static><link name='body'><collision name='c'><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='v'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>", "fixed-edit.sdf", "w", &NoResources, PhysicsEngine::Rapier).unwrap();
    let a = id(&sim, "w::a");
    let before = model_state(&sim, a).unwrap();
    let mut moved = replacement();
    moved.linear_velocity = [0.0; 3];
    moved.angular_velocity = [0.0; 3];
    set_model_state(&mut sim, a, moved).unwrap();
    sim.reset().unwrap();
    assert_eq!(model_state(&sim, a).unwrap(), before);
    let model = entity_by_id(sim.world(), a).unwrap();
    assert_eq!(
        sim.world().get::<WorldPose>(model).unwrap().translation,
        before.translation
    );
    set_model_state(&mut sim, a, moved).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(model_state(&sim, a).unwrap(), before);
}

#[test]
fn visual_preflight_is_read_only_and_matches_the_real_committed_offset_visual() {
    let mut sim = scene("", PhysicsEngine::Rapier);
    let model = id(&sim, "w::a");
    let before = model_state(&sim, model).unwrap();
    let clock = (sim.epoch(), sim.elapsed());
    let requested = replacement();
    let (visual, projected) = model_visual_pose(
        &sim,
        model,
        WorldPose {
            translation: requested.translation,
            rotation: requested.rotation,
        },
    )
    .unwrap();
    assert_eq!(projected.translation, [3.75, 5.0, 6.0]);
    assert_eq!(projected.rotation, requested.rotation);
    assert_eq!(model_state(&sim, model).unwrap(), before);
    assert_eq!((sim.epoch(), sim.elapsed()), clock);
    for pose in [
        WorldPose {
            translation: [f64::MAX, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        },
        WorldPose {
            translation: [0.0; 3],
            rotation: [0.0; 4],
        },
    ] {
        assert!(model_visual_pose(&sim, model, pose).is_err());
        assert_eq!(model_state(&sim, model).unwrap(), before);
    }
    assert!(
        model_visual_pose(
            &sim,
            u64::MAX,
            WorldPose {
                translation: [0.0; 3],
                rotation: [0.0, 0.0, 0.0, 1.0]
            }
        )
        .is_err()
    );
    set_model_state(&mut sim, model, requested).unwrap();
    let visual = entity_by_id(sim.world(), visual).unwrap();
    let actual = sim.world().get::<WorldPose>(visual).unwrap();
    assert_eq!(actual.translation, projected.translation);
    assert_eq!(actual.rotation, projected.rotation);
    assert_eq!((sim.epoch(), sim.elapsed()), clock);
}
