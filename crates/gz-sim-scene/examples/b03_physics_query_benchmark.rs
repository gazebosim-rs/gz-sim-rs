// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded physics/query actor; no Server, transport, renderer or timing ratio.

use bevy_ecs::{entity::Entity, hierarchy::ChildOf};
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, JointId, PhysicsBackend};
use gz_sim_core::EntityId;
use gz_sim_scene::{
    BodyHandle, EntityKind, JointHandle, Name, PhysicsEngine, Velocity, WorldPose,
    load_world_with_sdf_step,
};
use sdformat::NoResources;
use serde_json::json;
use std::{collections::BTreeMap, fs, path::PathBuf, time::Instant};
const XML: &str = include_str!("../tests/fixtures/b03_double_pendulum.sdf");
const URI: &str = "b03.sdf";
const XML_SHA256: &str = "24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa";
const BODY_NAMES: [&str; 7] = [
    "default::ground_plane::link",
    "default::double_pendulum_with_base0::base",
    "default::double_pendulum_with_base0::upper_link",
    "default::double_pendulum_with_base0::lower_link",
    "default::double_pendulum_with_base1::base",
    "default::double_pendulum_with_base1::upper_link",
    "default::double_pendulum_with_base1::lower_link",
];
const JOINT_NAMES: [&str; 4] = [
    "default::double_pendulum_with_base0::upper_joint",
    "default::double_pendulum_with_base0::lower_joint",
    "default::double_pendulum_with_base1::upper_joint",
    "default::double_pendulum_with_base1::lower_joint",
];
const BODY_FIELDS: [&str; 22] = [
    "px",
    "py",
    "pz",
    "qx",
    "qy",
    "qz",
    "qw",
    "origin_vx",
    "origin_vy",
    "origin_vz",
    "com_vx",
    "com_vy",
    "com_vz",
    "wx",
    "wy",
    "wz",
    "com_px",
    "com_py",
    "com_pz",
    "local_com_x",
    "local_com_y",
    "local_com_z",
];

const CONTRACT: &str = "b03-physics-unlimited-pair-v1";

struct BodyBinding {
    entity: Entity,
    body: BodyId,
    id: u64,
    center: [f64; 3],
}

struct JointBinding {
    joint: JointId,
    id: u64,
}

#[derive(Clone, Default)]
struct Frame {
    bodies: [[f64; 22]; 7],
    joints: [[f64; 2]; 4],
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(
        args.len(),
        4,
        "b03_physics_query_benchmark <exact XML path> <new output prefix> <correctness|measure>"
    );
    assert!(matches!(args[3].as_str(), "correctness" | "measure"));
    assert_eq!(fs::read(&args[1]).unwrap(), XML.as_bytes());
    let prefix = PathBuf::from(&args[2]);
    let output = |suffix: &str| PathBuf::from(format!("{}{}", prefix.display(), suffix));
    for suffix in [".state.json", ".metrics.json", ".maps", ".input.sdf"] {
        assert!(!output(suffix).exists());
    }

    let parsed = sdformat::load_str(XML, URI, &NoResources).unwrap();
    let gravity = parsed.worlds[0].gravity;
    let mut sim =
        load_world_with_sdf_step(XML, URI, "default", &NoResources, PhysicsEngine::Dart).unwrap();
    assert_eq!(sim.elapsed().as_nanos(), 0);
    let dt_ns = u64::try_from(sim.step_duration().as_nanos()).unwrap();
    assert_eq!(dt_ns, 1_000_000);
    let entities: Vec<_> = sim.world().iter_entities().filter_map(|entity| {
        let family = match entity.get::<EntityKind>()? {
            EntityKind::World => "WORLD",
            EntityKind::Model => "MODEL",
            EntityKind::Link => "BODY",
            EntityKind::Joint => "JOINT",
            EntityKind::Collision => "SHAPE",
            _ => return None,
        };
        Some(json!({"family": family, "name": entity.get::<Name>().unwrap().0,
            "id": entity.get::<EntityId>().unwrap().get(),
            "parent_id": entity.get::<ChildOf>().map(|parent| sim.world().get::<EntityId>(parent.parent()).unwrap().get())}))
    }).collect();
    assert_eq!(entities.len(), 30);
    let mut centers = BTreeMap::new();
    for model in &parsed.worlds[0].models {
        for link in &model.links {
            let center = parsed
                .resolve_pose(
                    &link.inertial.pose,
                    &model.scoped_name,
                    &link.scoped_name,
                    &link.scoped_name,
                )
                .unwrap()
                .translation;
            assert!(centers.insert(link.scoped_name.clone(), center).is_none());
        }
    }
    let named = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
            .unwrap()
            .id()
    };
    let bodies = BODY_NAMES.map(|name| {
        let entity = named(name);
        BodyBinding {
            entity,
            body: sim.world().get::<BodyHandle>(entity).unwrap().id(),
            id: sim.world().get::<EntityId>(entity).unwrap().get(),
            center: centers[name],
        }
    });
    let joints = JOINT_NAMES.map(|name| {
        let entity = named(name);
        JointBinding {
            joint: sim.world().get::<JointHandle>(entity).unwrap().id(),
            id: sim.world().get::<EntityId>(entity).unwrap().get(),
        }
    });

    // Both actors allocate their complete observation buffers before timing.
    let mut frames = vec![Frame::default(); 100];
    let mut steps = 0_u64;
    let mut body_queries = 0_u64;
    let mut joint_queries = 0_u64;
    let start = Instant::now();
    for frame in &mut frames {
        sim.step_exact(1).unwrap();
        steps += 1;
        let world = sim.world();
        let backend = world.resource::<PhysicsBackend>();
        for (row, body) in frame.bodies.iter_mut().zip(&bodies) {
            let state = backend.state(body.body).unwrap();
            let pose = world.get::<WorldPose>(body.entity).unwrap();
            let origin = world.get::<Velocity>(body.entity).unwrap();
            let center = DVec3::from_array(pose.translation)
                + DQuat::from_array(pose.rotation) * DVec3::from_array(body.center);
            let values = pose
                .translation
                .into_iter()
                .chain(pose.rotation)
                .chain(origin.linear)
                .chain(state.velocity)
                .chain(origin.angular)
                .chain(center.to_array())
                .chain(body.center);
            for (slot, value) in row.iter_mut().zip(values) {
                *slot = value;
            }
            body_queries += 1;
        }
        for (row, joint) in frame.joints.iter_mut().zip(&joints) {
            let state = backend.joint_state(joint.joint).unwrap();
            *row = [state.position, state.velocity];
            joint_queries += 1;
        }
    }
    let wall = start.elapsed();

    assert_eq!((steps, body_queries, joint_queries), (100, 700, 400));
    assert_eq!(sim.elapsed().as_nanos(), 100_000_000);
    assert_eq!(sim.epoch(), 0);
    for frame in &frames {
        assert!(frame.bodies.iter().flatten().all(|value| value.is_finite()));
        assert!(frame.joints.iter().flatten().all(|value| value.is_finite()));
    }
    // Keep all queried values observable without serialization in the window.
    let state = json!({
        "schema_version": 1,
        "contract": CONTRACT,
        "role": "rust-scene-physics",
        "declared_uri": URI,
        "xml_sha256": XML_SHA256,
        "body_fields": BODY_FIELDS,
        "settings": {
            "gravity": {"value": gravity, "evidence": "sdf-input"},
            "detector": {"value": "ode", "evidence": "source-established"},
            "solver": {"value": "DantzigBoxedLcpSolver", "evidence": "source-established"},
            "global_max_contacts": {"value": 10000, "evidence": "source-established"},
            "pair_max_contacts": {"value": "unlimited", "evidence": "source-established"},
            "dt_ns": {"value": dt_ns, "evidence": "scene-readback"},
            "solver_iterations": {"value": null, "evidence": "unsupported"},
        },
        "entities": entities,
        "frames": frames.iter().enumerate().map(|(index, frame)| json!({
            "tick": index + 1,
            "time_ns": (index + 1) * 1_000_000,
            "bodies": frame.bodies.iter().enumerate().map(|(i, values)| json!({
                "name": BODY_NAMES[i], "id": bodies[i].id, "values": values,
            })).collect::<Vec<_>>(),
            "joints": frame.joints.iter().enumerate().map(|(i, values)| json!({
                "name": JOINT_NAMES[i], "id": joints[i].id, "values": values,
            })).collect::<Vec<_>>(),
        })).collect::<Vec<_>>(),
    });
    let metrics = json!({
        "schema_version": 1,
        "contract": CONTRACT,
        "role": "rust-scene-physics",
        "mode": args[3],
        "steps": steps,
        "dt_ns": 1_000_000,
        "body_queries": body_queries,
        "joint_queries": joint_queries,
        "window_wall_ns": u64::try_from(wall.as_nanos()).unwrap(),
        "worlds": 1, "bodies": bodies.len(), "joints": joints.len(),
        "colliders": 15, "sensors": 0, "rendering": false,
        "network": false,
    });
    fs::write(output(".input.sdf"), XML.as_bytes()).unwrap();
    fs::write(output(".maps"), fs::read("/proc/self/maps").unwrap()).unwrap();
    fs::write(
        output(".state.json"),
        serde_json::to_vec_pretty(&state).unwrap(),
    )
    .unwrap();
    fs::write(
        output(".metrics.json"),
        serde_json::to_vec_pretty(&metrics).unwrap(),
    )
    .unwrap();
    println!("{metrics}");
}
