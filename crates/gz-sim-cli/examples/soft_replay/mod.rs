// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Primitive scene and lifecycle frames for measured deformable trajectories.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::time::Duration;

#[derive(Default)]
pub struct Frame {
    pub visuals: Vec<Value>,
    pub models: Vec<Value>,
}
impl Frame {
    pub fn add(&mut self, name: &str, p: DVec3, q: DQuat, shape: Value, color: [f64; 4]) {
        let id = 2 * self.visuals.len() as u64 + 2;
        let name = format!("default::{name}");
        self.visuals.push(json!({"id":id+1,"name":format!("{name}::visual"),"translation":p.to_array(),"rotation":q.to_array(),"shape":shape,"color":color}));
        self.models.push(json!({"model_id":id,"name":name,"visual_ids":[id+1],"can_remove":false,"can_set_state":false}));
    }
    pub fn edge(
        &mut self,
        name: &str,
        a: DVec3,
        b: DVec3,
        length: f64,
        width: f64,
        color: [f64; 4],
    ) {
        let delta = b - a;
        let q = DQuat::from_rotation_arc(DVec3::Z, delta.normalize());
        self.add(
            name,
            (a + b) * 0.5,
            q,
            json!({"kind":"box","size":[width,width,length]}),
            color,
        );
    }
}
impl Frame {
    pub fn finish(self, t: Duration, sequence: usize) -> (String, String) {
        (json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":sequence,"seconds":t.as_secs(),"nanoseconds":t.subsec_nanos(),"paused":false,"visuals":self.visuals}).to_string(),
    json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,"models":self.models,"templates":[]}).to_string())
    }
}
