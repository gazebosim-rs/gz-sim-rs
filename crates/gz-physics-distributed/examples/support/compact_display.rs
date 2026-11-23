// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two display-only gauges share the same zero-to-limit journal-entry scale.
use super::{LIMIT, replay};
use gz_physics_distributed::coordinator::Frame;
use serde::Serialize;
use serde_json::{Value, json};

#[derive(Serialize, PartialEq)]
pub struct JournalState {
    pub tick: u64,
    pub retained: [usize; 2],
    pub full: [usize; 2],
    pub baselines: usize,
    pub recoveries: usize,
}
#[derive(PartialEq)]
pub struct Sample {
    pub frame: Frame,
    pub journals: JournalState,
}
impl replay::Sample for Sample {
    fn committed(&self) -> &Frame {
        &self.frame
    }
    fn guides(&self, visuals: &mut Vec<Value>, models: &mut Vec<Value>) {
        for (worker, entries) in self.journals.retained.into_iter().enumerate() {
            let id = 1000 + worker as u64 * 5;
            let z = -1.65 - 0.65 * worker as f64;
            let name = format!("default::worker_{worker}_retained_entries_0_to_{LIMIT}");
            let rgb = if worker == 0 {
                [0.04, 0.80, 0.91]
            } else {
                [1., 0.34, 0.04]
            };
            let mut ids = Vec::new();
            let mut box_at = |part: &str, y: f64, height: f64, size: [f64; 3], color: [f64; 3]| {
                let vid = id + 1 + ids.len() as u64;
                ids.push(vid);
                visuals.push(json!({"id":vid,"name":format!("{name}::{part}"),"translation":[0.,y,height],"rotation":[0.,0.,0.,1.],
                    "shape":{"kind":"box","size":size},"color":[color[0],color[1],color[2],1.]}));
            };
            box_at("scale", 0., z, [0.08, 5.0, 0.036], [0.31, 0.36, 0.43]);
            box_at("zero", 2.5, z, [0.09, 0.035, 0.14], [0.48, 0.54, 0.58]);
            box_at("capacity", -2.5, z, [0.09, 0.035, 0.14], [0.48, 0.54, 0.58]);
            box_at(
                "retained",
                2.5 - 5. * entries as f64 / LIMIT as f64,
                z + 0.055,
                [0.16, 0.18, 0.17],
                rgb,
            );
            models.push(json!({"model_id":id,"name":name,"visual_ids":ids,"can_remove":false,"can_set_state":false}));
        }
    }
}
