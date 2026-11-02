// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Concrete B03 query workload shared by its regression and contextual example.
use bevy_ecs::hierarchy::ChildOf;
use glam::{DQuat, DVec3};
use gz_physics::PhysicsBackend;
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{
    BodyHandle, EntityKind, JointHandle, JointSample, Name, PhysicsEngine, Velocity, WorldLight,
    WorldPose, load_world_with_sdf_step,
};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    time::{Duration, Instant},
};

pub const XML: &str = include_str!("../fixtures/b03_double_pendulum.sdf");
pub const URI: &str = "b03.sdf";
pub const XML_SHA256: &str = "24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa";
pub const BODY_NAMES: [&str; 7] = [
    "default::ground_plane::link",
    "default::double_pendulum_with_base0::base",
    "default::double_pendulum_with_base0::upper_link",
    "default::double_pendulum_with_base0::lower_link",
    "default::double_pendulum_with_base1::base",
    "default::double_pendulum_with_base1::upper_link",
    "default::double_pendulum_with_base1::lower_link",
];
pub const JOINT_NAMES: [&str; 4] = [
    "default::double_pendulum_with_base0::upper_joint",
    "default::double_pendulum_with_base0::lower_joint",
    "default::double_pendulum_with_base1::upper_joint",
    "default::double_pendulum_with_base1::lower_joint",
];
pub const TICKS: [u64; 3] = [1, 10, 100];
pub const BODY_FIELDS: [&str; 22] = [
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

pub struct BodyRow {
    pub name: String,
    pub id: u64,
    pub values: [f64; 22],
}
pub struct JointRow {
    pub name: String,
    pub id: u64,
    pub values: [f64; 2],
}
pub struct Frame {
    pub tick: u64,
    pub time_ns: u64,
    pub bodies: Vec<BodyRow>,
    pub joints: Vec<JointRow>,
}
impl Frame {
    pub fn json(&self) -> Value {
        json!({"tick": self.tick, "time_ns": self.time_ns,
            "bodies": self.bodies.iter().map(|b| json!({"name": b.name, "id": b.id, "values": b.values})).collect::<Vec<_>>(),
            "joints": self.joints.iter().map(|j| json!({"name": j.name, "id": j.id, "values": j.values})).collect::<Vec<_>>()})
    }
}

pub struct Workload {
    pub sim: Simulation,
    pub entities: Vec<Value>,
    pub gravity: [f64; 3],
    bodies: Vec<(String, u64, [f64; 3])>,
    joints: Vec<(String, u64)>,
}
impl Workload {
    pub fn new() -> Self {
        let parsed = sdformat::load_str(XML, URI, &NoResources).unwrap();
        assert_eq!(parsed.worlds.len(), 1);
        let world = &parsed.worlds[0];
        assert_eq!(world.name, "default");
        assert_eq!(world.models.len(), 3);
        assert_eq!(world.lights.len(), 1);
        assert_eq!(world.lights[0].name, "sun");
        assert_eq!(world.physics[0].max_step_size, 0.001);
        assert_eq!(world.physics[0].real_time_factor, 0.0);
        let sim = load_world_with_sdf_step(XML, URI, "default", &NoResources, PhysicsEngine::Dart)
            .unwrap();
        assert_eq!(sim.elapsed(), Duration::ZERO);
        assert_eq!(sim.step_duration(), Duration::from_millis(1));
        let mut identities = BTreeMap::new();
        let mut entities = Vec::new();
        for entity in sim.world().iter_entities() {
            let Some(kind) = entity.get::<EntityKind>() else {
                continue;
            };
            let family = match kind {
                EntityKind::World => "WORLD",
                EntityKind::Model => "MODEL",
                EntityKind::Link => "BODY",
                EntityKind::Joint => "JOINT",
                EntityKind::Collision => "COLLISION",
                EntityKind::Light => "LIGHT",
                _ => continue,
            };
            let name = &entity.get::<Name>().unwrap().0;
            let id = entity.get::<EntityId>().unwrap().get();
            assert!(identities.insert(name.clone(), id).is_none());
            let parent = entity
                .get::<ChildOf>()
                .map(|p| sim.world().get::<EntityId>(p.parent()).unwrap().get());
            entities.push(json!({"family": family, "name": name, "id": id, "parent_id": parent}));
        }
        for (family, expected) in [
            ("WORLD", 1),
            ("MODEL", 3),
            ("BODY", 7),
            ("JOINT", 4),
            ("COLLISION", 15),
            ("LIGHT", 1),
        ] {
            assert_eq!(
                entities.iter().filter(|e| e["family"] == family).count(),
                expected
            );
        }
        let sun_entity = entity_by_id(sim.world(), identities["default::sun"]).unwrap();
        assert_eq!(
            sim.world()
                .get::<WorldLight>(sun_entity)
                .unwrap()
                .0
                .as_ref(),
            &world.lights[0]
        );
        let mut centers = BTreeMap::new();
        for model in &world.models {
            for link in &model.links {
                let name = format!("default::{}::{}", model.name, link.name);
                let resolved = parsed
                    .resolve_pose(
                        &link.inertial.pose,
                        &model.scoped_name,
                        &link.scoped_name,
                        &link.scoped_name,
                    )
                    .unwrap();
                assert!(centers.insert(name, resolved.translation).is_none());
            }
        }
        let bodies = BODY_NAMES
            .map(|name| (name.to_owned(), identities[name], centers[name]))
            .to_vec();
        let joints = JOINT_NAMES
            .map(|name| (name.to_owned(), identities[name]))
            .to_vec();
        for (_, id, _) in &bodies {
            let entity = entity_by_id(sim.world(), *id).unwrap();
            assert!(sim.world().get::<BodyHandle>(entity).is_some());
        }
        for (_, id) in &joints {
            let entity = entity_by_id(sim.world(), *id).unwrap();
            assert!(sim.world().get::<JointHandle>(entity).is_some());
        }
        Self {
            sim,
            entities,
            gravity: world.gravity,
            bodies,
            joints,
        }
    }
    fn observe(&self, tick: u64) -> Frame {
        assert_eq!(self.sim.elapsed(), Duration::from_millis(tick));
        let backend = self.sim.world().resource::<PhysicsBackend>();
        let bodies = self
            .bodies
            .iter()
            .map(|(name, id, center)| {
                let entity = entity_by_id(self.sim.world(), *id).unwrap();
                assert_eq!(self.sim.world().get::<Name>(entity).unwrap().0, *name);
                let handle = self.sim.world().get::<BodyHandle>(entity).unwrap();
                let state = backend.state(handle.id()).unwrap();
                let pose = self.sim.world().get::<WorldPose>(entity).unwrap();
                let origin = self.sim.world().get::<Velocity>(entity).unwrap();
                let com = DVec3::from_array(pose.translation)
                    + DQuat::from_array(pose.rotation) * DVec3::from_array(*center);
                let mut values = [0.; 22];
                let observed = pose
                    .translation
                    .into_iter()
                    .chain(pose.rotation)
                    .chain(origin.linear)
                    .chain(state.velocity)
                    .chain(origin.angular)
                    .chain(com.to_array())
                    .chain(*center);
                for (slot, value) in values.iter_mut().zip(observed) {
                    assert!(value.is_finite());
                    *slot = value;
                }
                // Keep backend COM velocity distinct from the public Scene origin value.
                for (a, b) in pose
                    .translation
                    .into_iter()
                    .chain(pose.rotation)
                    .chain(origin.angular)
                    .zip(
                        state
                            .position
                            .into_iter()
                            .chain(state.orientation)
                            .chain(state.angular_velocity),
                    )
                {
                    assert!((a - b).abs() <= 1e-12);
                }
                BodyRow {
                    name: name.clone(),
                    id: *id,
                    values,
                }
            })
            .collect();
        let joints = self
            .joints
            .iter()
            .map(|(name, id)| {
                let entity = entity_by_id(self.sim.world(), *id).unwrap();
                assert_eq!(self.sim.world().get::<Name>(entity).unwrap().0, *name);
                assert!(self.sim.world().get::<JointHandle>(entity).is_some());
                let sample = self.sim.world().get::<JointSample>(entity).unwrap();
                assert_eq!(sample.time, self.sim.elapsed());
                assert_eq!(sample.epoch, 0);
                let values = [sample.state.position, sample.state.velocity];
                assert!(values.iter().all(|v| v.is_finite()));
                JointRow {
                    name: name.clone(),
                    id: *id,
                    values,
                }
            })
            .collect();
        Frame {
            tick,
            time_ns: tick * 1_000_000,
            bodies,
            joints,
        }
    }
    pub fn run(&mut self) -> (Vec<Frame>, Duration) {
        assert_eq!(self.sim.elapsed(), Duration::ZERO);
        let mut selected = Vec::with_capacity(3);
        let begin = Instant::now();
        for tick in 1..=100 {
            self.sim.step_exact(1).unwrap();
            let frame = self.observe(tick);
            if TICKS.contains(&tick) {
                selected.push(frame);
            }
        }
        let wall = begin.elapsed();
        assert_eq!(selected.len(), 3);
        (selected, wall)
    }
}
