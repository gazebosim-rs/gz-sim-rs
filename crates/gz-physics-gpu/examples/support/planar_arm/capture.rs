// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::PlanarArmLane;
use serde_json::{Value, json};
use std::{fs, io, path::Path};

#[derive(Default)]
pub struct Capture {
    frames: Vec<(String, String)>,
    points: Vec<Value>,
}
impl Capture {
    pub fn sample(&mut self, time_ns: u64, lanes: &[PlanarArmLane]) {
        let mut visuals = vec![];
        let mut models = vec![];
        let mut arms = vec![];
        for (i, l) in lanes[..9].iter().enumerate() {
            let base = [(i % 3) as f64 * 1.6 - 1.6, (i / 3) as f64 * 1.2 - 1.2, 0.8];
            let [elbow, tip] = l.points().map(|[x, z]| [base[0] + x, base[1], base[2] + z]);
            let [a, b] = l.angle.map(|p| p[1].atan2(p[0]));
            let name = format!("default::arm_{i}");
            let model_id = 2 + i * 5;
            let mut ids = vec![];
            let mut add = |part: &str,
                           translation: [f64; 3],
                           angle: f64,
                           shape: Value,
                           color: [f64; 4]| {
                let id = model_id + 1 + ids.len();
                ids.push(id);
                visuals.push(json!({"id":id,"name":format!("{name}::{part}"),"translation":translation,
                    "rotation":[0.,(-angle/2.).sin(),0.,(angle/2.).cos()],"shape":shape,"color":color}));
            };
            let mid = |u: [f64; 3], v: [f64; 3]| std::array::from_fn(|i| (u[i] + v[i]) / 2.);
            add(
                "base",
                base,
                0.,
                json!({"kind":"box","size":[0.085,0.095,0.085]}),
                [0.21, 0.28, 0.34, 1.],
            );
            add(
                "upper",
                mid(base, elbow),
                a,
                json!({"kind":"box","size":[l.length[0],0.045,0.045]}),
                [0.09, 0.61, 0.70, 1.],
            );
            add(
                "forearm",
                mid(elbow, tip),
                a + b,
                json!({"kind":"box","size":[l.length[1],0.035,0.035]}),
                [0.28, 0.74, 0.48, 1.],
            );
            add(
                "tool",
                tip,
                0.,
                json!({"kind":"sphere","radius":0.024+l.tip_mass*0.080}),
                [0.94, 0.46, 0.10, 1.],
            );
            models.push(json!({"model_id":model_id,"name":name,"visual_ids":ids,"can_remove":false,"can_set_state":false}));
            arms.push(json!({"lane_id":i,"base":base,"elbow":elbow,"tip":tip,"angles":l.angle,"length":l.length,"tip_mass_kg":l.tip_mass}));
        }
        let seq = self.frames.len() + 1;
        self.frames.push((
            json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,
            "sequence":seq,"seconds":time_ns/1_000_000_000,"nanoseconds":time_ns%1_000_000_000,
            "paused":false,"visuals":visuals})
            .to_string(),
            json!({"version":1,"kind":"lifecycle_catalog",
            "world_id":1,"epoch":0,"scene_sequence":seq,"models":models,"templates":[]})
            .to_string(),
        ));
        self.points.push(json!({"time_ns":time_ns,"arms":arms}));
    }
    pub fn write(self, path: &Path) -> io::Result<()> {
        gz_sim_cli::export_archive(&path.join("replay"), Default::default(), self.frames)?;
        fs::write(path.join("frames.json"), serde_json::to_vec(&self.points)?)
    }
}
