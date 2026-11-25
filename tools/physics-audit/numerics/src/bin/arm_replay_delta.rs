// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the six moving link centers displayed at shared clocks of two serial-arm recordings.
use glam::DVec3;
use serde_json::{Value, json};
use std::collections::BTreeMap;
use std::io::{BufRead, BufReader};
use std::path::Path;

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
type Centers = [DVec3; 6];
type Frames = BTreeMap<u64, Centers>;
// Each visual's SDF origin coincides with its measured link center.
const VISUALS: [&str; 6] = [
    "default::gripper::palm::palm",
    "default::parcel::box::box",
    "default::gripper::carriage::truck",
    "default::gripper::nut::nut",
    "default::gripper::left::left",
    "default::gripper::right::right",
];

fn frames(dir: &Path) -> Result<Frames> {
    let manifest: Value = serde_json::from_reader(std::fs::File::open(dir.join("manifest.json"))?)?;
    let records = std::fs::File::open(dir.join("frames.jsonl"))?;
    decode(&manifest, BufReader::new(records))
}

fn decode(manifest: &Value, records: impl BufRead) -> Result<Frames> {
    let mut frames = BTreeMap::new();
    let mut count = 0;
    for line in records.lines() {
        let record: Value = serde_json::from_str(&line?)?;
        if record["kind"] != "frame" {
            continue;
        }
        let scene: Value = serde_json::from_str(record["scene"].as_str().ok_or("missing scene")?)?;
        let ns = scene["nanoseconds"].as_u64().ok_or("missing nanoseconds")?;
        let t = scene["seconds"]
            .as_u64()
            .and_then(|s| s.checked_mul(1_000_000_000))
            .and_then(|s| s.checked_add(ns))
            .filter(|_| ns < 1_000_000_000)
            .ok_or("invalid clock")?;
        let visuals = scene["visuals"].as_array().ok_or("missing visuals")?;
        let mut centers = [DVec3::ZERO; 6];
        for (center, name) in centers.iter_mut().zip(VISUALS) {
            let mut matches = visuals.iter().filter(|v| v["name"] == name);
            let pose = matches.next().ok_or_else(|| format!("missing {name}"))?;
            let p: [f64; 3] = serde_json::from_value(pose["translation"].clone())?;
            *center = DVec3::from(p);
            if !center.is_finite() || matches.next().is_some() {
                return Err(format!("invalid or duplicate {name} at {t} ns").into());
            }
        }
        // A paused edit can publish a second view at the same instant.
        // A clock compares unambiguously when the tracked centers agree.
        if frames
            .insert(t, centers)
            .is_some_and(|prior| prior != centers)
        {
            return Err(format!("ambiguous centers at {t} ns").into());
        }
        count += 1;
    }
    if manifest["complete"] != true || manifest["frames"] != count || frames.is_empty() {
        return Err("incomplete recording".into());
    }
    Ok(frames)
}

fn compare(adaptive: &Frames, dynamic: &Frames) -> Result<Value> {
    if adaptive.keys().next() != dynamic.keys().next()
        || adaptive.keys().last() != dynamic.keys().last()
    {
        return Err("recordings cover different time intervals".into());
    }
    let samples: Vec<_> = adaptive
        .iter()
        .filter_map(|(ns, a)| {
            let d = dynamic.get(ns)?;
            let distance: [f64; 6] = std::array::from_fn(|i| 1e3 * a[i].distance(d[i]));
            let (index, maximum) = distance
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.total_cmp(b.1))?;
            Some(json!([
                ns,
                distance[0],
                distance[1],
                maximum,
                VISUALS[index]
            ]))
        })
        .collect();
    Ok(
        json!({"columns":["time_ns","palm_center_delta_mm","parcel_center_delta_mm","maximum_link_center_delta_mm","maximum_visual_name"],
        "adaptive_only_ns":adaptive.keys().filter(|t| !dynamic.contains_key(t)).collect::<Vec<_>>(),
        "dynamic_only_ns":dynamic.keys().filter(|t| !adaptive.contains_key(t)).collect::<Vec<_>>(),
        "samples":samples}),
    )
}

fn main() -> Result<()> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    let [adaptive, dynamic] = args.as_slice() else {
        return Err("usage: arm_replay_delta ADAPTIVE_ARCHIVE DYNAMIC_ARCHIVE".into());
    };
    let data = compare(&frames(Path::new(adaptive))?, &frames(Path::new(dynamic))?)?;
    serde_json::to_writer(std::io::stdout().lock(), &data)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn paused_edits_preserve_comparison_only_when_named_centers_agree() {
        let scene = json!({"seconds":0,"nanoseconds":40,
            "visuals":VISUALS.map(|name| json!({"name":name,"translation":[0,0,0]}))});
        let frame = |scene: &Value| json!({"kind":"frame","scene":scene.to_string()}).to_string();
        let a = frame(&scene);
        let manifest = json!({"complete":true,"frames":2});
        let check = |other: &Value| decode(&manifest, format!("{a}\n{}", frame(other)).as_bytes());
        assert_eq!(check(&scene).unwrap().len(), 1);
        assert!(decode(&manifest, a.as_bytes()).is_err());
        let mut moved = scene.clone();
        moved["visuals"][0]["translation"] = json!([0.1, 0., 0.]);
        assert!(check(&moved).is_err());
        let mut renamed = scene.clone();
        renamed["visuals"][0]["name"] = json!(VISUALS[1]);
        assert!(check(&renamed).is_err());
        let mut bad_clock = scene;
        bad_clock["nanoseconds"] = json!(1_000_000_000u64);
        assert!(check(&bad_clock).is_err());
    }

    #[test]
    fn different_event_clocks_are_reported_and_distances_use_matching_times() {
        let zero = [DVec3::ZERO; 6];
        let mut a = Frames::from([(0, zero), (30, zero)]);
        let mut d = a.clone();
        a.insert(15, zero);
        d.insert(25, zero);
        let mut moved = zero;
        moved[0] = DVec3::Y * 0.003;
        moved[1] = DVec3::Z * 0.007;
        moved[4] = DVec3::X * 0.016;
        d.insert(30, moved);
        let paired = compare(&a, &d).unwrap();
        assert_eq!(paired["adaptive_only_ns"], json!([15]));
        assert_eq!(paired["dynamic_only_ns"], json!([25]));
        assert_eq!(paired["samples"][0][3], 0.);
        assert_eq!(paired["samples"][1], json!([30, 3., 7., 16., VISUALS[4]]));
        d.remove(&30);
        assert!(compare(&a, &d).is_err());
    }
}
