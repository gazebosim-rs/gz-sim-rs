// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check a recording produced through the ordinary simulator/recorder path by ordinal.
use super::{Result, same, vector};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    fs::{self, File},
    io::{BufRead, BufReader},
    path::Path,
};

pub(super) fn read(source: &Path, index: &Value) -> Result<Value> {
    let manifest: Value = serde_json::from_slice(&fs::read(source.join("manifest.json"))?)?;
    if manifest["complete"] != true {
        return Err("recording has no complete manifest".into());
    }
    let mut poses = BTreeMap::new();
    let mapped = index["aligned_visual_links"]
        .as_object()
        .or_else(|| index["source_visual_poses"].as_object());
    if let Some(links) = mapped {
        if links.is_empty() {
            return Err("empty authored visual to link mapping".into());
        }
        let path = index["source_poses"]
            .as_str()
            .ok_or("missing independent authored pose file")?;
        let snapshots: Vec<Value> =
            serde_json::from_reader(BufReader::new(File::open(source.join(path))?))?;
        for entry in snapshots {
            let t = entry["time_ns"].as_u64().ok_or("independent pose clock")?;
            if poses.insert(t, entry).is_some() {
                return Err("duplicate independent pose clock".into());
            }
        }
    }
    let mut frames = vec![];
    for line in BufReader::new(File::open(source.join("frames.jsonl"))?).lines() {
        let record: Value = serde_json::from_str(&line?)?;
        if record["kind"] != "frame" {
            continue;
        }
        let ordinal = record["ordinal"].as_u64().ok_or("recording ordinal")?;
        if ordinal != frames.len() as u64 + 1 {
            return Err("discontinuous recording ordinal".into());
        }
        let scene: Value = serde_json::from_str(record["scene"].as_str().ok_or("recorded scene")?)?;
        let mut catalog: Value =
            serde_json::from_str(record["catalog"].as_str().ok_or("recorded catalog")?)?;
        // Replay preserves model identities and visual membership but suppresses live writes.
        for model in catalog["models"].as_array_mut().ok_or("recorded models")? {
            model["can_remove"] = json!(false);
            model["can_set_state"] = json!(false);
        }
        let ns = clock(&scene)?;
        let visuals = scene["visuals"].as_array().ok_or("recorded visuals")?;
        if let Some(expected) = index["visual_names"].as_array()
            && (expected.len() != visuals.len()
                || expected
                    .iter()
                    .any(|name| !visuals.iter().any(|v| v["name"] == *name)))
        {
            return Err(format!("unexpected authored visual list at ordinal {ordinal}").into());
        }
        if let Some(links) = mapped {
            let sample = poses
                .get(&ns)
                .ok_or("missing independent authored state at recorded clock")?;
            if visuals.len() != links.len() {
                return Err("aligned mapping differs from authored visual count".into());
            }
            for v in visuals {
                let name = v["name"].as_str().ok_or("missing authored visual name")?;
                let link = links
                    .get(name)
                    .and_then(Value::as_str)
                    .ok_or("missing aligned authored link")?;
                let pose = &sample["poses"][link];
                if !same(&v["translation"], vector::<3>(&pose["translation"])?)?
                    || !same(&v["rotation"], vector::<4>(&pose["rotation"])?)?
                {
                    return Err(format!(
                        "authored visual {name} differs from independent link {link} at {ns} ns"
                    )
                    .into());
                }
            }
        }
        frames.push(json!({"ns":ns,"scene":scene,"catalog":catalog}));
    }
    if frames.is_empty() || manifest["frames"].as_u64() != Some(frames.len() as u64) {
        return Err("manifest and recorded frame count differ".into());
    }
    if let Some(names) = index.get("particle_visuals") {
        let samples = File::open(source.join("../particle-frames.json"))?;
        let samples: Vec<Value> = serde_json::from_reader(BufReader::new(samples))?;
        super::particle_visuals::verify(&frames, &samples, serde_json::from_value(names.clone())?)?;
    }
    if let Some(path) = index["gpu_arm_frames"].as_str() {
        let file = File::open(source.join(path))?;
        let samples: Vec<Value> = serde_json::from_reader(BufReader::new(file))?;
        super::gpu_arm_visuals::verify(&frames, &samples)?;
        if index["gpu_tool_contact"] == true {
            let audit = super::gpu_tool_visuals::verify(&frames, &samples)?;
            frames[0]["gpu_tool_audit"] = audit;
        }
    }
    if let Some(trace) = index["aligned_journal_trace"].as_str() {
        super::journal_visuals::verify(BufReader::new(File::open(source.join(trace))?), &frames)?;
    }
    Ok(Value::Array(frames))
}
fn clock(scene: &Value) -> Result<u64> {
    let n = scene["nanoseconds"].as_u64().ok_or("scene nanoseconds")?;
    scene["seconds"]
        .as_u64()
        .and_then(|s| s.checked_mul(1_000_000_000))
        .and_then(|s| s.checked_add(n))
        .ok_or_else(|| "invalid scene clock".into())
}
pub(super) fn verify(scene: &Value, catalog: &Value, status: &Value, raw: &Value) -> Result<u64> {
    let i = status["frame"]
        .as_u64()
        .and_then(|n| n.checked_sub(1))
        .ok_or("invalid replay ordinal")?;
    let r = raw.get(i as usize).ok_or("missing archived ordinal")?;
    if clock(scene)? != r["ns"].as_u64().ok_or("expected recorded clock")?
        || scene["sequence"] != status["scene_sequence"]
        || scene["sequence"] != catalog["scene_sequence"]
        || scene["epoch"] != catalog["epoch"]
        || scene["world_id"] != r["scene"]["world_id"]
        || scene["world"] != r["scene"]["world"]
        || catalog["models"] != r["catalog"]["models"]
        || catalog["templates"]
            .as_array()
            .is_none_or(|templates| !templates.is_empty())
    {
        return Err(format!("authored recording metadata differs on ordinal {}", i + 1).into());
    }
    let visuals = scene["visuals"].as_array().ok_or("viewer visual list")?;
    let expected = r["scene"]["visuals"]
        .as_array()
        .ok_or("source visual list")?;
    if visuals.len() != expected.len() {
        return Err("viewer visual count differs from recording".into());
    }
    for source in expected {
        let v = visuals
            .iter()
            .find(|v| v["id"] == source["id"])
            .ok_or("missing visual id")?;
        if v["name"] != source["name"]
            || !same_shape(&v["shape"], &source["shape"])
            || v["color"] != source["color"]
            || !same(&v["translation"], vector::<3>(&source["translation"])?)?
            || !same(&v["rotation"], vector::<4>(&source["rotation"])?)?
        {
            return Err(format!(
                "{} pose or shape differs from source ordinal {}",
                source["name"],
                i + 1
            )
            .into());
        }
    }
    Ok(i)
}

/// Replay rehosts archived mesh bytes on a new loopback HTTP origin.
fn same_shape(live: &Value, source: &Value) -> bool {
    if source["kind"] != "mesh" {
        return live == source;
    }
    fn route(shape: &Value) -> Option<String> {
        let uri = shape["uri"]
            .as_str()?
            .parse::<tungstenite::http::Uri>()
            .ok()?;
        let authority = uri.authority()?;
        (uri.scheme_str() == Some("http")
            && authority.host() == "127.0.0.1"
            && !authority.as_str().contains('@')
            && authority.port_u16().is_some_and(|port| port > 0)
            && uri.query().is_none()
            && uri.path().starts_with("/assets/"))
        .then(|| uri.path().to_owned())
    }
    let Some(path) = route(source) else {
        return false;
    };
    if route(live).as_ref() != Some(&path) {
        return false;
    }
    let mut expected = source.clone();
    expected["uri"] = live["uri"].clone();
    live == &expected
}

#[cfg(test)]
mod mesh_tests {
    use super::*;

    #[test]
    fn a_recording_can_rebind_only_the_port_of_a_matching_loopback_asset() {
        let original =
            json!({"kind":"mesh","uri":"http://127.0.0.1:12/assets/4/payload.glb","scale":[1,2,3]});
        let set = |address: &str| {
            let mut live = original.clone();
            live["uri"] = json!(address);
            live
        };
        let valid = set("http://127.0.0.1:34/assets/4/payload.glb");
        assert!(same_shape(&valid, &original));
        for address in [
            "http://localhost:34/assets/4/payload.glb",
            "http://user@127.0.0.1:34/assets/4/payload.glb",
            "https://127.0.0.1:34/assets/4/payload.glb",
            "http://127.0.0.1:34/assets/5/payload.glb",
            "http://127.0.0.1:34/assets/4/payload.glb?q=1",
            "http://127.0.0.1/assets/4/payload.glb",
            "not a URI",
            "/assets/4/payload.glb",
        ] {
            assert!(!same_shape(&set(address), &original));
        }
        let mut wrong_scale = valid;
        wrong_scale["scale"] = json!([1, 1, 1]);
        assert!(!same_shape(&wrong_scale, &original));
        assert!(same_shape(
            &json!({"kind":"sphere","radius":4}),
            &json!({"kind":"sphere","radius":4})
        ));
        assert!(!same_shape(&original, &json!({"kind":"mesh"})));
    }
}
