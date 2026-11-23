// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::reference::{Q, V};
use gz_physics_gpu::SpherePairLane as Lane;
use serde_json::{Value, json};
use std::{io, path::Path};
#[derive(Default)]
pub struct Capture {
    frames: Vec<(String, String)>,
    last: Option<usize>,
    hits: [Option<(usize, V)>; 3],
}
impl Capture {
    pub fn observe(&mut self, tick: usize, batch: &[Lane]) {
        self.frame(tick, batch, None);
    }
    pub fn supported(&mut self, tick: usize, batch: &[gz_physics_gpu::SupportedSpherePairLane]) {
        // Equal-mass elastic control and unequal-mass sliding/rebounding control.
        let picks = [batch[0], batch[4]];
        let poses = picks.map(|s| (V::from(s.plane_normal), s.plane_offset));
        let states = picks.map(|s| {
            let mut p = s.pair;
            let n = V::from(s.plane_normal);
            let g = V::from(p.gravity);
            p.gravity = (g - n.dot(g) * n).to_array();
            p
        });
        self.frame(tick, &states, Some(&poses));
    }
    fn frame(&mut self, tick: usize, batch: &[Lane], planes: Option<&[(V, f64)]>) {
        let mut impact = false;
        for (i, s) in batch.iter().take(3).enumerate() {
            if s.normal_impulse > 1e-8 {
                // The endpoint is <= 1 ms after contact; publish the physical contact
                // point reconstructed at time of impact from the endpoint trajectory.
                let rest = 0.001 - s.impact_time;
                let t = s.impact_time;
                let p = V::from(s.a.position) - V::from(s.a.velocity) * rest
                    + 0.5 * V::from(s.gravity) * rest * rest;
                self.hits[i] = Some((tick, p + V::from(s.contact_normal) * s.a.radius));
                debug_assert!((0.0..=0.001).contains(&t));
                impact = true;
            }
        }
        if !tick.is_multiple_of(8) && !impact {
            return;
        }
        if self.last == Some(tick) {
            return;
        }
        self.last = Some(tick);
        let mut visuals = Vec::new();
        let mut models = Vec::new();
        let turn = Q::from_rotation_z(-std::f64::consts::FRAC_PI_2);
        for (i, s) in batch.iter().take(3).enumerate() {
            let origin = if planes.is_some() {
                V::new(0., (0.5 - i as f64) * 2.65, 0.)
            } else {
                V::new(0., 0., (1. - i as f64) * 0.64)
            };
            let name = format!("default::pair_{i}");
            let model = 2 + 16 * i as u64;
            let mut ids = Vec::new();
            let mut add = |label: &str, center: V, rotation: Q, shape: Value, color: [f64; 3]| {
                let id = model + 1 + ids.len() as u64;
                visuals.push(json!({"id":id,"name":format!("{name}::{label}"),"translation":(origin+turn*center).to_array(),"rotation":(turn*rotation).to_array(),"shape":shape,"color":[color[0],color[1],color[2],1.]}));
                ids.push(id);
            };
            if let Some(p) = planes {
                let (n, offset) = p[i];
                add(
                    "frictionless_support",
                    n * (offset - 0.015),
                    Q::from_rotation_arc(V::Z, n),
                    json!({"kind":"box","size":[2.36,1.18,0.03]}),
                    [0.30, 0.36, 0.41],
                );
            }
            for (label, body, color) in
                [("A", s.a, [0.1, 0.8, 0.98]), ("B", s.b, [0.99, 0.54, 0.10])]
            {
                let p = V::from(body.position);
                let q = Q::from_array(body.orientation);
                let r = body.radius;
                add(label, p, q, json!({"kind":"sphere","radius":r}), color);
                for (tag, axis) in [("x", V::X), ("y", V::Y), ("z", V::Z)] {
                    add(
                        &format!("{label}_mark_{tag}"),
                        p + q * (0.99 * r * axis),
                        q,
                        json!({"kind":"sphere","radius":0.16*r}),
                        [0.98, 0.98, 0.95],
                    );
                }
            }
            // Emphasize the measured event for 40 ms, using a fixed primitive
            // identity. Outside the interval it is occluded inside body A.
            let spark = self.hits[i]
                .filter(|(t, _)| tick - t <= 40)
                .map(|(_, p)| p)
                .unwrap_or(V::from(s.a.position));
            add(
                "contact_witness",
                spark,
                Q::IDENTITY,
                json!({"kind":"sphere","radius":0.033}),
                [1., 0.15, 0.12],
            );
            models.push(json!({"model_id":model,"name":name,"visual_ids":ids,"can_remove":false,"can_set_state":false}));
        }
        let seq = self.frames.len() + 1;
        let ns = tick as u64 * 1_000_000;
        let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":seq,"seconds":ns/1_000_000_000,"nanoseconds":ns%1_000_000_000,"paused":false,"visuals":visuals});
        let cat = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":seq,"models":models,"templates":[]});
        self.frames.push((scene.to_string(), cat.to_string()));
    }
    pub fn write(self, path: &Path) -> io::Result<()> {
        gz_sim_cli::export_archive(path, Default::default(), self.frames)
    }
}
