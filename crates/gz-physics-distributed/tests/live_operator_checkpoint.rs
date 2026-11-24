// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Durability at the live server cut, including an acknowledged asynchronous removal.
#[path = "private/passive_pose_proof.rs"]
mod passive_pose_proof;

use gz_sim_core::{Simulation, entity_by_id};
use gz_sim_proto::{
    msgs::{Boolean, Entity, WorldControl, WorldReset, entity::Type},
    prost::Message,
};
use gz_sim_scene::{
    ModelState, PhysicsEngine, WorldLoadOptions, WorldPose,
    passive_checkpoint::{PassivePoolScene, RestoredTemplates, save_paused_runtime},
};
use gz_sim_server::{Server, ServerOptions};
use std::{
    num::NonZeroUsize,
    path::Path,
    time::{Duration, Instant},
};
use zenoh::Wait;

const SDF: &str = include_str!("../../../examples/distributed_stack.sdf");
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

fn options() -> WorldLoadOptions<'static> {
    WorldLoadOptions {
        engine: PhysicsEngine::DistributedIslands(4),
        distributed_worker: Some(Path::new(env!("CARGO_BIN_EXE_physics_worker"))),
        step: None,
        noise_seed: [0; 32],
    }
}
fn make_scene() -> Result<PassivePoolScene> {
    Ok(PassivePoolScene::load(
        SDF,
        "stack.sdf",
        "default",
        options(),
        Default::default(),
    )?)
}
fn resume(path: &Path) -> Result<PassivePoolScene> {
    Ok(PassivePoolScene::resume(
        SDF,
        "stack.sdf",
        "default",
        options(),
        Default::default(),
        path,
    )?)
}
fn local_peer() -> Result<(zenoh::Session, String)> {
    let config = zenoh::Config::from_json5(r#"{mode:"peer",listen:{endpoints:["tcp/127.0.0.1:0"]},connect:{endpoints:[]},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}"#).unwrap();
    let peer = zenoh::open(config).wait().map_err(|e| e.to_string())?;
    let endpoints: Vec<_> = peer
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|l| l.to_string())
        .collect();
    let client = serde_json::json!({"mode":"client","connect":{"endpoints":endpoints},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}}});
    Ok((peer, client.to_string()))
}

fn network(sim: Simulation) -> Result<(Server, zenoh::Session, ServerOptions)> {
    let (peer, client) = local_peer()?;
    let opts = ServerOptions {
        partition: format!("live-cut-{}", uuid::Uuid::new_v4()),
        ..Default::default()
    };
    let server = Server::with_zenoh_config(
        sim,
        "default",
        opts.clone(),
        zenoh::Config::from_json5(&client).unwrap(),
        NonZeroUsize::new(8).unwrap(),
    )?;
    Ok((server, peer, opts))
}

#[test]
fn queued_control_and_acknowledged_async_retirement_must_commit_before_sealing() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let path = dir.path().join("cut");
    let mut loaded = make_scene()?;
    loaded.set_paused(true);
    let (mut server, peer, opts) = network(loaded.into_checkpointable_runtime())?;
    server.tick()?;
    let initial_source = server
        .simulation()
        .world()
        .resource::<RestoredTemplates>()
        .by_name("default::first")
        .unwrap();
    let model = server.spawn_model_template(
        &initial_source,
        WorldPose {
            translation: [2., 0., 4.],
            rotation: [0., 0., 0., 1.],
        },
    )?;
    assert_eq!(model, 14);
    let q = 0.175_f64;
    let state = ModelState {
        translation: [2.25, 1., 3.5],
        rotation: [0., 0., q.sin(), q.cos()],
        linear_velocity: [0.25, 0., 0.125],
        angular_velocity: [0., 0.125, 0.],
    };
    server.set_model_state(model, state)?;
    server.tick()?;
    let newborn = dir.path().join("newborn");
    server.save_paused_pool_checkpoint(&newborn)?;
    assert_eq!(
        passive_pose_proof::proof(server.simulation()),
        passive_pose_proof::proof(resume(&newborn)?.simulation())
    );

    assert!(server.apply_local_control(WorldControl {
        pause: true,
        multi_step: 2,
        ..Default::default()
    }));
    assert!(server.save_paused_pool_checkpoint(&path).is_err());
    server.tick()?;
    assert!(server.save_paused_pool_checkpoint(&path).is_err());
    server.tick()?;
    assert!(server.simulation().is_paused());
    assert!(!path.exists());

    let timeout = Duration::from_secs(8);
    let key = opts.topic_key("/world/default/remove")?;
    let querier = peer
        .declare_querier(key)
        .timeout(timeout)
        .wait()
        .map_err(|e| e.to_string())?;
    let end = Instant::now() + timeout;
    while !querier
        .matching_status()
        .wait()
        .map_err(|e| e.to_string())?
        .matching()
    {
        assert!(
            Instant::now() < end,
            "actual asynchronous removal endpoint did not match"
        );
        std::thread::sleep(Duration::from_millis(2));
    }
    let query = querier
        .get()
        .payload(
            Entity {
                id: 6,
                r#type: Type::Model as i32,
                ..Default::default()
            }
            .encode_to_vec(),
        )
        .wait()
        .map_err(|e| e.to_string())?;
    while server.counters().removals_queued == 0 {
        assert!(Instant::now() < end, "removal ingress deadline");
        server.tick()?;
        std::thread::sleep(Duration::from_millis(1));
    }
    let reply = query
        .recv_timeout(timeout)
        .map_err(|e| e.to_string())?
        .expect("actual reply");
    let answer = reply.result().map_err(|_| "removal service error")?;
    assert!(Boolean::decode(answer.payload().to_bytes().as_ref())?.data);
    // The actual peer has received success, but the mutation awaits the next frame.
    assert!(entity_by_id(server.simulation().world(), 6).is_some());
    assert!(matches!(
        server.save_paused_pool_checkpoint(&path),
        Err(gz_sim_server::Error::CheckpointBoundary)
    ));
    assert!(!path.exists());
    server.tick()?;
    assert_eq!(server.counters().removals_completed, 1);
    assert!(entity_by_id(server.simulation().world(), 6).is_none());
    server.save_paused_pool_checkpoint(&path)?;
    let sealed = std::fs::read(&path)?;
    assert!(server.save_paused_pool_checkpoint(&path).is_err());
    assert_eq!(std::fs::read(&path)?, sealed);
    let mut restored = resume(&path)?;
    assert_eq!(
        passive_pose_proof::proof(server.simulation()),
        passive_pose_proof::proof(restored.simulation())
    );

    // A subsequent live command belongs to the later timeline. Both independently
    // running worlds can still clone the retired original at their respective cuts.
    let next = server.spawn_model_template(
        &initial_source,
        WorldPose {
            translation: [0., 3., 4.],
            rotation: [0., 0., 0., 1.],
        },
    )?;
    assert_eq!(next, 18);
    assert!(entity_by_id(restored.simulation().world(), next).is_none());
    assert_eq!(
        restored.spawn(
            6,
            WorldPose {
                translation: [0., 3., 4.],
                rotation: [0., 0., 0., 1.]
            }
        )?,
        next
    );
    assert!(server.apply_local_control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    }));
    let reset = dir.path().join("reset");
    assert!(server.save_paused_pool_checkpoint(&reset).is_err());
    server.prepare_io()?;
    assert!(server.save_paused_pool_checkpoint(&reset).is_err());
    server.tick()?;
    server.save_paused_pool_checkpoint(&reset)?;
    let after = resume(&reset)?;
    assert_eq!(after.simulation().epoch(), 1);
    assert!(entity_by_id(after.simulation().world(), next).is_none());
    assert!(entity_by_id(after.simulation().world(), 6).is_some());
    assert_eq!(
        passive_pose_proof::proof(server.simulation()),
        passive_pose_proof::proof(after.simulation())
    );
    Ok(())
}

#[test]
fn unarmed_unsynchronized_or_extended_live_schedules_cannot_publish() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let path = dir.path().join("invalid");
    let mut unarmed = make_scene()?.into_simulation();
    unarmed.set_paused(true);
    unarmed.tick()?;
    assert!(save_paused_runtime(&mut unarmed, &path).is_err());
    drop(unarmed);
    let mut s = make_scene()?.into_checkpointable_runtime();
    s.set_paused(true);
    s.tick()?;
    let model = entity_by_id(s.world(), 6).unwrap();
    let saved = *s.world().get::<WorldPose>(model).unwrap();
    s.world_mut()
        .get_mut::<WorldPose>(model)
        .unwrap()
        .translation[0] += 0.75;
    assert!(save_paused_runtime(&mut s, &path).is_err());
    *s.world_mut().get_mut::<WorldPose>(model).unwrap() = saved;
    // Replacing one original phase with the same number of foreign callbacks
    // preserves its count. Its actual schedule lineage still changed.
    #[derive(bevy_ecs::prelude::Component)]
    struct OtherApplication;
    let addition = s.world_mut().spawn(OtherApplication).id();
    assert!(save_paused_runtime(&mut s, &path).is_err());
    s.world_mut().despawn(addition);
    s.world_mut().entity_mut(model).insert(OtherApplication);
    assert!(save_paused_runtime(&mut s, &path).is_err());
    s.world_mut().entity_mut(model).remove::<OtherApplication>();
    let clock = *s.world().resource::<gz_sim_core::UpdateInfo>();
    s.world_mut().resource_mut::<gz_sim_core::UpdateInfo>().dt = Duration::from_secs(1);
    assert!(save_paused_runtime(&mut s, &path).is_err());
    *s.world_mut().resource_mut::<gz_sim_core::UpdateInfo>() = clock;
    let totals = s.scheduled_system_counts();
    let removed = s.remove_systems_in_set(
        gz_sim_core::Phase::PhysicsStep,
        gz_sim_core::Phase::PhysicsStep,
    )?;
    assert!(removed > 0);
    for _ in 0..removed {
        s.add_systems(gz_sim_core::Phase::PhysicsStep, || {});
    }
    assert_eq!(s.scheduled_system_counts(), totals);
    assert!(save_paused_runtime(&mut s, &path).is_err());
    assert!(!path.exists());
    Ok(())
}

#[test]
fn externally_changed_visual_or_world_construction_metadata_rejects_the_cut() -> Result<()> {
    use gz_sim_scene::{VisualGeometry, VisualMaterial, WorldPhysics, spawn_model_template};
    let dir = tempfile::tempdir()?;
    let mut sim = make_scene()?.into_checkpointable_runtime();
    sim.set_paused(true);
    let source = sim
        .world()
        .resource::<RestoredTemplates>()
        .by_name("default::first")
        .unwrap();
    let clone = spawn_model_template(
        &mut sim,
        &source,
        WorldPose {
            translation: [3., 2., 4.],
            rotation: [0., 0., 0., 1.],
        },
    )?;
    sim.tick()?;
    for visual_id in [9, clone + 3] {
        let visual = entity_by_id(sim.world(), visual_id).unwrap();
        let geometry = sim.world().get::<VisualGeometry>(visual).unwrap().clone();
        sim.world_mut()
            .entity_mut(visual)
            .insert(VisualGeometry(std::sync::Arc::new(
                sdformat::Geometry::Sphere { radius: 0.75 },
            )));
        let path = dir.path().join(format!("geometry-{visual_id}"));
        assert!(
            save_paused_runtime(&mut sim, &path).is_err(),
            "changed visual geometry {visual_id} escaped the passive archive"
        );
        assert!(!path.exists());
        sim.world_mut().entity_mut(visual).insert(geometry);
        let material = sim
            .world_mut()
            .entity_mut(visual)
            .take::<VisualMaterial>()
            .unwrap();
        let path = dir.path().join(format!("material-{visual_id}"));
        assert!(
            save_paused_runtime(&mut sim, &path).is_err(),
            "missing visual material {visual_id} escaped the passive archive"
        );
        assert!(!path.exists());
        sim.world_mut().entity_mut(visual).insert(material);
    }
    let profiles = std::mem::take(&mut sim.world_mut().resource_mut::<WorldPhysics>().profiles);
    let path = dir.path().join("world");
    assert!(save_paused_runtime(&mut sim, &path).is_err());
    assert!(!path.exists());
    sim.world_mut().resource_mut::<WorldPhysics>().profiles = profiles;
    save_paused_runtime(&mut sim, &path)?;
    assert_eq!(
        passive_pose_proof::proof(&sim),
        passive_pose_proof::proof(resume(&path)?.simulation())
    );
    Ok(())
}

#[cfg(target_os = "linux")]
#[path = "private/operator_signal.rs"]
mod operator_signal;
