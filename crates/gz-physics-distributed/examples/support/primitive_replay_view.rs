// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};

#[derive(Default)]
pub struct View {
    visuals: Vec<Value>,
    models: Vec<Value>,
}
impl View {
    pub fn add(
        &mut self,
        id: u64,
        name: &str,
        position: [f64; 3],
        rotation: [f64; 4],
        shape: Value,
        color: [f64; 4],
    ) {
        self.visuals.push(json!({"id":id+1,"name":format!("default::{name}::visual"),"translation":position,"rotation":rotation,"shape":shape,"color":color}));
        self.models.push(json!({"model_id":id,"name":format!("default::{name}"),"visual_ids":[id+1],"can_remove":false,"can_set_state":false}));
    }
    pub fn cuboid(&mut self, id: u64, name: &str, pos: [f64; 3], size: [f64; 3], color: [f64; 4]) {
        self.add(
            id,
            name,
            pos,
            [0., 0., 0., 1.],
            json!({"kind":"box","size":size}),
            color,
        );
    }
    pub fn finish(self, tick: u64) -> (String, String) {
        self.finish_at(tick + 1, tick * 10_000_000)
    }
    pub fn finish_at(self, seq: u64, ns: u64) -> (String, String) {
        self.finish_stamp(seq, 0, std::time::Duration::from_nanos(ns), false)
    }
    pub fn finish_stamp(
        self,
        seq: u64,
        epoch: u64,
        time: std::time::Duration,
        paused: bool,
    ) -> (String, String) {
        (json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":epoch,"seconds":time.as_secs(),"nanoseconds":time.subsec_nanos(),
            "sequence":seq,"paused":paused,"visuals":self.visuals}).to_string(),
         json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":epoch,"scene_sequence":seq,"models":self.models,"templates":[]}).to_string())
    }
}
