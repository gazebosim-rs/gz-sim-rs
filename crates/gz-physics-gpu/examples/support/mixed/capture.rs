// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::reference::{Q, V};
use gz_physics_gpu::{Environment, EnvironmentState as State, SolidSphere};
use serde_json::{Value, json};
use std::{io, path::Path};
#[derive(Default)]
pub struct Capture {
    frames: Vec<(String, String)>,
}
impl Capture {
    pub fn sample(&mut self, tick: usize, envs: &[Environment]) {
        if !tick.is_multiple_of(8) {
            return;
        }
        let mut visuals = Vec::new();
        let mut models = Vec::new();
        for (row, i) in [3, 1, 14].into_iter().enumerate() {
            let e = envs[i];
            let name = format!(
                "default::env_{}_{}",
                e.id,
                match e.state {
                    State::SpherePlane(_) => "sphere_plane",
                    State::SpherePair(_) => "free_pair",
                    State::SupportedSpherePair(_) => "supported_pair",
                }
            );
            let model = 2 + row as u64 * 16;
            let origin = V::new(0., 0., (1. - row as f64) * 0.87);
            let turn = Q::from_rotation_z(-std::f64::consts::FRAC_PI_2);
            let mut ids = Vec::new();
            let mut add = |label: &str, p: V, q: Q, shape: Value, c: [f64; 3]| {
                let id = model + 1 + ids.len() as u64;
                ids.push(id);
                visuals.push(json!({"id":id,"name":format!("{name}::{label}"),"translation":(origin+turn*p).to_array(),"rotation":(turn*q).to_array(),"shape":shape,"color":[c[0],c[1],c[2],1.]}));
            };
            let mut bodies = Vec::new();
            let mut plane = None;
            match e.state {
                State::SpherePlane(s) => {
                    bodies.push(SolidSphere {
                        position: s.position,
                        velocity: s.velocity,
                        angular_velocity: s.angular_velocity,
                        orientation: s.orientation,
                        radius: s.radius,
                        inverse_mass: s.inverse_mass,
                    });
                    plane = Some((s.plane_normal, s.plane_offset, [0.24, 0.36, 0.40]));
                }
                State::SpherePair(s) => bodies.extend([s.a, s.b]),
                State::SupportedSpherePair(s) => {
                    bodies.extend([s.pair.a, s.pair.b]);
                    plane = Some((s.plane_normal, s.plane_offset, [0.32, 0.33, 0.44]));
                }
            }
            if let Some((normal, offset, color)) = plane {
                let n = V::from(normal);
                add(
                    "mixed_actual_plane",
                    n * (offset - 0.014),
                    Q::from_rotation_arc(V::Z, n),
                    json!({"kind":"box","size":[2.16,0.85,0.028]}),
                    color,
                );
            }
            for (slot, b) in bodies.into_iter().enumerate() {
                let p = V::from(b.position);
                let q = Q::from_array(b.orientation);
                let color = if row == 0 {
                    [0.33, 0.88, 0.57]
                } else if slot == 0 {
                    [0.10, 0.76, 0.99]
                } else {
                    [0.99, 0.56, 0.10]
                };
                add(
                    &format!("sphere_{slot}"),
                    p,
                    q,
                    json!({"kind":"sphere","radius":b.radius}),
                    color,
                );
                for (n, axis) in [V::X, V::Y, V::Z].into_iter().enumerate() {
                    add(
                        &format!("spin_{slot}_{n}"),
                        p + q * (axis * (0.99 * b.radius)),
                        q,
                        json!({"kind":"sphere","radius":0.16*b.radius}),
                        [0.98, 0.98, 0.95],
                    );
                }
            }
            models.push(json!({"model_id":model,"name":name,"visual_ids":ids,"can_remove":false,"can_set_state":false}));
        }
        let seq = self.frames.len() + 1;
        let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":seq,"seconds":0,"nanoseconds":tick as u64*1_000_000,"paused":false,"visuals":visuals});
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":seq,"models":models,"templates":[]});
        self.frames.push((scene.to_string(), catalog.to_string()));
    }
    pub fn write(self, path: &Path) -> io::Result<()> {
        gz_sim_cli::export_archive(path, Default::default(), self.frames)
    }
}
