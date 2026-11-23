// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! gzweb replay of nine observed environments. Layout offsets are display-only.
use gz_physics_gpu::Lane;
use gz_sim_cli::export_archive;
use serde_json::{Value, json};
use std::{io, path::Path};

const COUNT: usize = 9;
const BOB_RADIUS: f64 = 0.03;
const RIGID_RADIUS: f64 = 0.08;

#[derive(Default)]
pub struct Capture {
    frames: Vec<(String, String)>,
}

fn visual(id: u64, name: &str, xyz: [f64; 3], shape: Value, rgb: [f64; 3]) -> Value {
    json!({
        "id": id, "name": name, "translation": xyz, "rotation": [0, 0, 0, 1],
        "shape": shape, "color": [rgb[0], rgb[1], rgb[2], 1]
    })
}

impl Capture {
    /// Record positions only after the batch has committed the tick.
    pub fn sample(&mut self, time_ns: u64, lanes: &[Lane]) -> io::Result<()> {
        let cells = lanes.get(..COUNT).ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::InvalidInput,
                "replay requires nine environment lanes",
            )
        })?;
        let mut visuals = Vec::new();
        let mut models = Vec::new();
        for (i, lane) in cells.iter().enumerate() {
            if (lane.combined_radius - (BOB_RADIUS + RIGID_RADIUS)).abs() > 1e-12 {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidInput,
                    "unexpected batch fixture radii",
                ));
            }
            let x = ((i / 3) as f64 - 1.0) * 0.94;
            let y = ((i % 3) as f64 - 1.0) * 0.65;
            let name = format!("default::cell_{i:02}");
            let model_id = 2 + 12 * i as u64;
            let mut ids = Vec::new();
            let mut add = |part: &str, z: f64, shape, color| {
                let id = model_id + 1 + ids.len() as u64;
                ids.push(id);
                visuals.push(visual(
                    id,
                    &format!("{name}::{part}"),
                    [x, y, z],
                    shape,
                    color,
                ));
            };
            add(
                "datum",
                -0.015,
                json!({"kind":"box","size":[0.43,0.43,0.03]}),
                [0.29, 0.34, 0.40],
            );
            // The two material families have disjoint compliance ranges.
            let anchor_color = if lane.compliance > 1e-4 {
                [0.94, 0.36, 0.11]
            } else {
                [0.05, 0.70, 0.56]
            };
            add(
                "anchor",
                lane.anchor_z,
                json!({"kind":"box","size":[0.16,0.16,0.035]}),
                anchor_color,
            );
            // These guide marks interpolate the two endpoints of the distance
            // constraint. They are display geometry and carry no simulation mass.
            for marker in 1..=5 {
                let z = lane.anchor_z + (lane.bob_z - lane.anchor_z) * f64::from(marker) / 6.0;
                add(
                    &format!("strut_{marker}"),
                    z,
                    json!({"kind":"sphere","radius":0.014}),
                    [0.68, 0.77, 0.85],
                );
            }
            add(
                "bob",
                lane.bob_z,
                json!({"kind":"sphere","radius":BOB_RADIUS}),
                [0.96, 0.51, 0.08],
            );
            add(
                "rigid",
                lane.rigid_z,
                json!({"kind":"sphere","radius":RIGID_RADIUS}),
                [0.10, 0.47, 0.97],
            );
            models.push(json!({"model_id": model_id, "name":name,"visual_ids": ids,
                "can_remove":false, "can_set_state":false}));
        }
        let sequence = self.frames.len() as u64 + 1;
        let scene = json!({
            "version":1, "kind":"scene", "world":"default","world_id":1,"epoch":0,
            "sequence":sequence,"seconds":time_ns/1_000_000_000,"nanoseconds":time_ns%1_000_000_000,
            "paused":false,"visuals":visuals
        });
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,
            "scene_sequence":sequence,"models":models,"templates":[]});
        self.frames.push((scene.to_string(), catalog.to_string()));
        Ok(())
    }

    pub fn write(self, path: &Path) -> io::Result<()> {
        export_archive(path, Default::default(), self.frames)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::reference::fixture;

    #[test]
    fn native_archive_admits_the_display_and_preserves_rigid_heights() {
        let mut capture = Capture::default();
        let mut lanes = fixture(COUNT);
        capture.sample(0, &lanes).unwrap();
        lanes[8].rigid_z -= 0.012;
        capture.sample(20_000_000, &lanes).unwrap();
        let scene: Value = serde_json::from_str(&capture.frames[1].0).unwrap();
        let displayed = scene["visuals"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|v| v["name"].as_str().unwrap().ends_with("::rigid"));
        assert_eq!(displayed.clone().count(), COUNT);
        for (visual, lane) in displayed.zip(&lanes) {
            assert_eq!(visual["translation"][2].as_f64().unwrap(), lane.rigid_z);
        }
        let temp = tempfile::tempdir().unwrap();
        let path = temp.path().join("replay");
        capture.write(&path).unwrap();
        assert!(path.join("manifest.json").is_file());
    }
}
