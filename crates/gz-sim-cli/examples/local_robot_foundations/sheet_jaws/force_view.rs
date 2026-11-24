// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Show the signed force on each panel alongside the unmodified physical playback.
use super::super::*;
use serde::Serialize;

#[derive(Clone, Copy, Default, Serialize)]
struct Forces {
    jaw: [f64; 2],
    bending: [f64; 2],
    self_friction: [f64; 2],
}

#[derive(Default)]
pub(crate) struct ForceView {
    window: Forces,
    frames: Vec<Forces>,
}

impl ForceView {
    pub(crate) fn add(&mut self, jaw: [f64; 2], bending: [f64; 2], friction: [f64; 2]) {
        for i in 0..2 {
            self.window.jaw[i] += jaw[i];
            self.window.bending[i] += bending[i];
            self.window.self_friction[i] += friction[i];
        }
    }

    pub(crate) fn sample(&mut self, period_s: f64) {
        let mut frame = std::mem::take(&mut self.window);
        for pair in [&mut frame.jaw, &mut frame.bending, &mut frame.self_friction] {
            for f in pair {
                *f /= period_s;
            }
        }
        self.frames.push(frame);
    }

    pub(crate) fn save(&self, recording: &Recording, out: &Path, weight: f64) -> Result<()> {
        if self.frames.len() != recording.frames.len() || !(weight.is_finite() && weight > 0.) {
            return Err("inconsistent panel force recording".into());
        }
        let mut frames = Vec::with_capacity(self.frames.len());
        for ((scene, catalog), forces) in recording.frames.iter().zip(&self.frames) {
            let mut scene: Value = serde_json::from_str(scene)?;
            let mut catalog: Value = serde_json::from_str(catalog)?;
            let visuals = scene["visuals"]
                .as_array_mut()
                .ok_or("missing scene visuals")?;
            let models = catalog["models"]
                .as_array_mut()
                .ok_or("missing model catalog")?;
            let highest_id = visuals
                .iter()
                .map(|v| &v["id"])
                .chain(models.iter().map(|m| &m["model_id"]))
                .try_fold(0_u64, |highest, id| id.as_u64().map(|v| highest.max(v)))
                .ok_or("missing visual or model identity")?;
            let model_id = highest_id.checked_add(10).ok_or("no available gauge IDs")? - 9;
            let mut ids = Vec::new();
            let mut add = |label: &str, position: [f64; 3], shape: Value, color: [f64; 4]| {
                let id = model_id + 1 + ids.len() as u64;
                ids.push(id);
                visuals.push(json!({"id":id,"name":format!("default::panel_forces::{label}"),"translation":position,"rotation":[0.,0.,0.,1.],"shape":shape,"color":color}));
            };
            let z0 = 0.39;
            for (index, (label, force)) in [
                ("minus_half_weight", -weight * 0.5),
                ("zero", 0.),
                ("plus_half_weight", weight * 0.5),
            ]
            .into_iter()
            .enumerate()
            {
                let width = if index == 1 { 0.225 } else { 0.204 };
                add(
                    label,
                    [0.625, 0.003, z0 + height(force, weight)],
                    json!({"kind":"box","size":[width,0.003,0.002]}),
                    [0.63, 0.68, 0.74, 1.],
                );
            }
            for (panel, side) in ["left", "right"].into_iter().enumerate() {
                for (axis, (name, values, color)) in [
                    ("jaw", forces.jaw, [1., 0.63, 0.10, 1.]),
                    ("fold", forces.bending, [0.12, 0.84, 0.52, 1.]),
                    ("friction", forces.self_friction, [0.70, 0.42, 0.98, 1.]),
                ]
                .into_iter()
                .enumerate()
                {
                    let force = values[panel];
                    if !force.is_finite() {
                        return Err("nonfinite panel force".into());
                    }
                    add(
                        &format!("{side}_{name}"),
                        [
                            0.711 - 0.112 * panel as f64 - 0.03 * axis as f64,
                            -0.004,
                            z0 + height(force, weight),
                        ],
                        json!({"kind":"sphere","radius":0.008}),
                        color,
                    );
                }
            }
            models.push(json!({"model_id":model_id,"name":"default::panel_forces","visual_ids":ids,"can_remove":false,"can_set_state":false}));
            frames.push((scene.to_string(), catalog.to_string()));
        }
        gz_sim_cli::export_archive(&out.join("force-replay"), Default::default(), frames)?;
        write_json(
            &out.join("panel-forces.json"),
            &json!({"frame_interval_ns":25_000_000,"sheet_weight_n":weight,"samples":self.frames}),
        )
    }
}
fn height(force: f64, weight: f64) -> f64 {
    0.055 * (force / (0.5 * weight)).asinh()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_uses_signed_window_averages_and_checks_every_native_frame() -> Result<()> {
        let mut recording = Recording::new(crate::arm::SCENE, "arm.sdf", "default")?;
        recording.capture()?;
        let mut v = ForceView::default();
        let out = tempfile::tempdir()?;
        assert!(v.save(&recording, out.path(), 0.1).is_err());
        v.add([0.001, -0.0005], [0.0002, -0.0002], [0.; 2]);
        v.sample(0.025);
        assert_eq!(v.frames[0].jaw, [0.04, -0.02]);
        assert!(height(-0.2, 0.1) < height(-0.1, 0.1) && height(-0.1, 0.1) == -height(0.1, 0.1));
        assert!(v.save(&recording, out.path(), 0.).is_err());
        let original = recording.frames[0].clone();
        for (scene, catalog) in [
            ("bad", "{}"),
            ("{}", "bad"),
            ("{}", "{}"),
            (r#"{"visuals":[]}"#, "{}"),
            (r#"{"visuals":[{}]}"#, r#"{"models":[]}"#),
            (
                r#"{"visuals":[{"id":18446744073709551615}]}"#,
                r#"{"models":[]}"#,
            ),
        ] {
            recording.frames[0] = (scene.into(), catalog.into());
            assert!(v.save(&recording, out.path(), 0.1).is_err());
        }
        recording.frames[0] = original;
        let mut corrupt = v.frames[0];
        corrupt.bending[0] = f64::INFINITY;
        v.frames[0] = corrupt;
        assert!(v.save(&recording, out.path(), 0.1).is_err());
        v.frames[0].bending[0] = 0.008;
        v.save(&recording, out.path(), 0.1)?;
        assert!(v.save(&recording, out.path(), 0.1).is_err());
        let archive = fs::read_to_string(out.path().join("force-replay/frames.jsonl"))?;
        let scene: Value = archive
            .lines()
            .map(serde_json::from_str::<Value>)
            .collect::<std::result::Result<Vec<_>, _>>()?
            .into_iter()
            .find(|row| row["kind"] == "frame")
            .ok_or("missing frame")?;
        let scene: Value = serde_json::from_str(scene["scene"].as_str().unwrap())?;
        for (side, force) in [("left", 0.04), ("right", -0.02)] {
            let name = format!("default::panel_forces::{side}_jaw");
            let dot = scene["visuals"]
                .as_array()
                .unwrap()
                .iter()
                .find(|v| v["name"] == name)
                .unwrap();
            assert!(
                (dot["translation"][2].as_f64().unwrap() - 0.39 - height(force, 0.1)).abs() < 1e-14
            );
        }
        Ok(())
    }
}
