// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::{PlanarArmLane, PlanarToolLane};
pub trait DisplayLane {
    fn display(&self) -> (&PlanarArmLane, Option<&PlanarToolLane>);
}
impl DisplayLane for PlanarArmLane {
    fn display(&self) -> (&PlanarArmLane, Option<&PlanarToolLane>) {
        (self, None)
    }
}
impl DisplayLane for PlanarToolLane {
    fn display(&self) -> (&PlanarArmLane, Option<&PlanarToolLane>) {
        (&self.arm, Some(self))
    }
}
use serde_json::{Value, json};
use std::{fs, io, path::Path};

#[derive(Default)]
pub struct Capture {
    frames: Vec<(String, String)>,
    points: Vec<Value>,
}
impl Capture {
    pub fn sample(&mut self, time_ns: u64, lanes: &[impl DisplayLane]) {
        let mut visuals = vec![];
        let mut models = vec![];
        let mut arms = vec![];
        for (i, lane) in lanes[..9].iter().enumerate() {
            let (l, contact) = lane.display();
            let base = [(i % 3) as f64 * 1.6 - 1.6, (i / 3) as f64 * 1.2 - 1.2, 0.8];
            let [elbow, tip] = l.points().map(|[x, z]| [base[0] + x, base[1], base[2] + z]);
            let [a, b] = l.angle.map(|p| p[1].atan2(p[0]));
            let name = format!("default::arm_{i}");
            let model_id = 2 + i * if contact.is_some() { 7 } else { 5 };
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
                json!({"kind":"sphere","radius":contact.map_or(0.024+l.tip_mass*0.080, |c|c.radius)}),
                [0.94, 0.46, 0.10, 1.],
            );
            if let Some(c) = contact {
                let height = if c.normal_impulse > 0. {
                    tip[2] + c.radius + 0.013
                } else {
                    base[2] + c.ground_z - 0.020
                };
                add(
                    "contact_marker",
                    [tip[0], tip[1], height],
                    0.,
                    json!({"kind":"sphere","radius":0.008}),
                    [1., 0.16, 0.08, 1.],
                );
                add(
                    "ground",
                    [base[0], base[1], base[2] + c.ground_z - 0.020],
                    0.,
                    json!({"kind":"box","size":[1.5,0.8,0.04]}),
                    [0.30, 0.36, 0.42, 1.],
                );
            }
            models.push(json!({"model_id":model_id,"name":name,"visual_ids":ids,"can_remove":false,"can_set_state":false}));
            let mut observation = json!({"lane_id":i,"base":base,"elbow":elbow,"tip":tip,"angles":l.angle,"length":l.length,"tip_mass_kg":l.tip_mass});
            if let Some(c) = contact {
                observation["ground_z_m"] = json!(c.ground_z);
                observation["tool_radius_m"] = json!(c.radius);
                observation["friction_mu"] = json!(c.friction);
                observation["normal_impulse_ns"] = json!(c.normal_impulse);
                observation["tangent_impulse_ns"] = json!(c.tangent_impulse);
                observation["surface_gap_m"] = json!(c.gap());
                observation["first_contact_offset_s"] = json!(c.first_contact_offset);
            }
            arms.push(observation);
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
