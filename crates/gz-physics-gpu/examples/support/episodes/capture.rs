// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{runner::Runner, task::sphere};
use crate::reference::{Q, V};
use serde_json::json;
use std::{io, path::Path};
#[derive(Default)]
pub struct Capture {
    frames: Vec<(String, String)>,
}
impl Capture {
    pub fn sample(&mut self, k: u64, trained: &Runner, coast: &Runner) {
        if !k.is_multiple_of(8) {
            return;
        }
        // Same held-out sphere/plane environment in both worlds. A green
        // target moves from the episode's measured initial x at target velocity.
        let i = 6;
        let turn = Q::from_rotation_z(-std::f64::consts::FRAC_PI_2);
        let mut visuals = vec![];
        let mut models = vec![];
        for (row, run) in [trained, coast].into_iter().enumerate() {
            let env = run.states()[i];
            let e = &run.agent.episodes[i];
            let b = sphere(&env.state);
            let origin = V::new(0., 0., (0.5 - row as f64) * 0.92);
            let model = 2 + row as u64 * 12;
            let name = format!(
                "default::{}_env_{}",
                if row == 0 { "trained" } else { "coast" },
                env.id
            );
            let mut labels = vec![];
            let mut add = |label: &str, p: V, q: Q, shape: serde_json::Value, color: [f64; 3]| {
                let id = model + 1 + labels.len() as u64;
                labels.push(id);
                visuals.push(json!({"id":id,"name":format!("{name}::{label}"),"translation":(origin+turn*p).to_array(),"rotation":(turn*q).to_array(),"shape":shape,"color":[color[0],color[1],color[2],1.]}));
            };
            let target = e.initial_x + e.target * (k - e.start) as f64 * 0.001;
            add(
                "support",
                V::new(-0.12, 0., -0.0125),
                Q::IDENTITY,
                json!({"kind":"box","size":[1.25,0.39,0.025]}),
                [0.24, 0.30, 0.34],
            );
            add(
                "episode_velocity_target",
                V::new(target, -0.16, 0.145),
                Q::IDENTITY,
                json!({"kind":"box","size":[0.021,0.038,0.29]}),
                [0.24, 0.89, 0.40],
            );
            add(
                "target_tip",
                V::new(target, -0.16, 0.292),
                Q::IDENTITY,
                json!({"kind":"sphere","radius":0.029}),
                [0.39, 0.98, 0.37],
            );
            let p = V::from(b.position);
            let q = Q::from_array(b.orientation);
            let color = if row == 0 {
                [0.13, 0.75, 0.98]
            } else {
                [0.99, 0.55, 0.13]
            };
            add(
                "sphere",
                p,
                q,
                json!({"kind":"sphere","radius":b.radius}),
                color,
            );
            for (n, v) in [V::X, V::Y, V::Z].into_iter().enumerate() {
                add(
                    &format!("spin_{n}"),
                    p + q * (v * 0.99 * b.radius),
                    q,
                    json!({"kind":"sphere","radius":0.13*b.radius}),
                    [0.96, 0.96, 0.95],
                );
            }
            models.push(json!({"model_id":model,"name":name,"visual_ids":labels,"can_remove":false,"can_set_state":false}));
        }
        let seq = self.frames.len() + 1;
        let ns = k * 1_000_000;
        let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":seq,"seconds":ns/1_000_000_000,"nanoseconds":ns%1_000_000_000,"paused":false,"visuals":visuals});
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":seq,"models":models,"templates":[]});
        self.frames.push((scene.to_string(), catalog.to_string()));
    }
    pub fn write(self, p: &Path) -> io::Result<()> {
        gz_sim_cli::export_archive(p, Default::default(), self.frames)
    }
}
