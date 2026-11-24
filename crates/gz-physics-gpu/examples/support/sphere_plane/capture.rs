// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::reference::{Q, V};
use gz_physics_gpu::SpherePlaneLane as Lane;
use serde_json::{Value, json};
use std::{io, path::Path};

#[derive(Default)]
pub struct Capture {
    frames: Vec<(String, String)>,
}
impl Capture {
    pub fn sample(&mut self, tick: usize, lanes: &[Lane]) {
        let mut visuals = Vec::new();
        let mut models = Vec::new();
        for (i, s) in lanes.iter().take(6).enumerate() {
            let row = i / 3;
            let col = i % 3;
            // Each tile is a finite display of its independent infinite plane.
            let origin = V::new((row as f64 - 0.5) * 1.8, (1. - col as f64) * 2.35, 0.);
            let n = V::from(s.plane_normal);
            let q = Q::from_array(s.orientation);
            let p = V::from(s.position) + origin;
            let name = format!("default::oblique_{i}");
            let model = 2 + 8 * i as u64;
            let mut ids = Vec::new();
            let mut add = |label: &str, center: V, rotation: Q, shape: Value, color: [f64; 3]| {
                let id = model + 1 + ids.len() as u64;
                visuals.push(json!({"id":id,"name":format!("{name}::{label}"),"translation":center.to_array(),"rotation":rotation.to_array(),"shape":shape,"color":[color[0],color[1],color[2],1.]}));
                ids.push(id);
            };
            let stage_q = Q::from_rotation_arc(V::Z, n);
            add(
                "plane",
                origin + n * (s.plane_offset - 0.014),
                stage_q,
                json!({"kind":"box","size":[1.62,1.78,0.028]}),
                [0.32, 0.36, 0.40],
            );
            let color = match i % 4 {
                0 => [0.09, 0.66, 0.91],
                1 => [0.1, 0.81, 0.58],
                2 => [0.91, 0.61, 0.16],
                _ => [0.87, 0.30, 0.14],
            };
            add(
                "sphere",
                p,
                q,
                json!({"kind":"sphere","radius":s.radius}),
                color,
            );
            // Three body-fixed material markers expose the measured world quaternion.
            for (label, axis) in [("x_mark", V::X), ("y_mark", V::Y), ("z_mark", V::Z)] {
                add(
                    label,
                    p + q * axis * (s.radius * 0.99),
                    q,
                    json!({"kind":"sphere","radius":s.radius*0.18}),
                    [0.95, 0.95, 0.90],
                );
            }
            models.push(json!({"model_id":model,"name":name,"visual_ids":ids,"can_remove":false,"can_set_state":false}));
        }
        let sequence = self.frames.len() + 1;
        let ns = tick as u64 * 1_000_000;
        let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":sequence,"seconds":ns/1_000_000_000,"nanoseconds":ns%1_000_000_000,"paused":false,"visuals":visuals});
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,"models":models,"templates":[]});
        self.frames.push((scene.to_string(), catalog.to_string()));
    }
    pub fn write(self, path: &Path) -> io::Result<()> {
        gz_sim_cli::export_archive(path, Default::default(), self.frames)
    }
}
