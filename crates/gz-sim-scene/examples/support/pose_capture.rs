// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Capture immutable SDF visual geometry and committed world poses for replay.
use bevy_ecs::entity::Entity;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{Name, VisualGeometry, WorldPose};
use sdformat::Geometry;
use serde_json::{Value, json};

pub struct Capture {
    visuals: Vec<Entity>,
    geometry: Vec<Value>,
    models: Vec<Value>,
    frames: Vec<Value>,
}
impl Capture {
    pub fn new(sim: &Simulation, model_names: &[&str], paint: impl Fn(&str) -> [f64; 3]) -> Self {
        let mut objects = Vec::new();
        for entity in sim.world().iter_entities() {
            let Some(geometry) = entity.get::<VisualGeometry>() else {
                continue;
            };
            let shape = match geometry.0.as_ref() {
                Geometry::Box { size } => json!({"kind":"box","size":size}),
                Geometry::Sphere { radius } => json!({"kind":"sphere","radius":radius}),
                _ => unreachable!("these fixtures contain primitive boxes and spheres"),
            };
            let name = &entity.get::<Name>().unwrap().0;
            let [r, g, b] = paint(name);
            let id = entity.get::<EntityId>().unwrap().get();
            let visual = json!({"id":id,"name":name,"shape":shape,"color":[r,g,b,1.]});
            objects.push((id, entity.id(), visual));
        }
        objects.sort_by_key(|v| v.0);
        let mut models = Vec::new();
        for &name in model_names {
            let model = sim
                .world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .unwrap();
            let model_id = model.get::<EntityId>().unwrap().get();
            let prefix = format!("{name}::");
            let visual_ids: Vec<_> = objects
                .iter()
                .filter(|o| o.2["name"].as_str().unwrap().starts_with(&prefix))
                .map(|o| o.0)
                .collect();
            models.push(json!({"name":name,"model_id":model_id,"visual_ids":visual_ids,"can_remove":false,"can_set_state":false}));
        }
        Self {
            models,
            visuals: objects.iter().map(|v| v.1).collect(),
            geometry: objects.into_iter().map(|v| v.2).collect(),
            frames: Vec::new(),
        }
    }
    pub fn sample(&mut self, sim: &Simulation) {
        self.sample_with_offset(sim, None);
    }
    /// Move only the specified presentation indicator in its local frame.
    /// The same SDF identity, geometry and material persist across all frames.
    pub fn sample_with_offset(&mut self, sim: &Simulation, offset: Option<(&str, [f64; 3])>) {
        let mut poses = Vec::with_capacity(self.visuals.len());
        for (&entity, definition) in self.visuals.iter().zip(&self.geometry) {
            let pose = sim.world().get::<WorldPose>(entity).unwrap();
            let mut p = glam::DVec3::from_array(pose.translation);
            if let Some((name, delta)) = offset
                && definition["name"] == name
            {
                p += glam::DQuat::from_array(pose.rotation) * glam::DVec3::from_array(delta);
            }
            poses.push(json!([p.to_array(), pose.rotation]));
        }
        self.frames
            .push(json!({"time_ns":sim.elapsed().as_nanos() as u64,"poses":poses}));
    }
    pub fn finish(self) -> Value {
        json!({"geometry":self.geometry,"models":self.models,"frames":self.frames})
    }
}
