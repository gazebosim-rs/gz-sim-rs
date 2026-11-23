// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Cut, Result, launch, primitive_replay_view::View, read, reference, save};
use gz_physics_gpu::{SpherePairBlockBatch, SpherePairBlockObservation, SphereRotation};
use reference::{Q, V};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{fs, path::Path};

pub(super) fn row(
    view: &mut View,
    name: &str,
    id: u64,
    z: f64,
    obs: &SpherePairBlockObservation,
) -> Value {
    let bodies = [obs.state.a, obs.state.b];
    spheres(view, name, id, z, &bodies);
    json!({"name":name,"bodies":bodies,"offset_z":z,"contact_ticks":obs.contact_ticks(),"contact_times":obs.contact_times(),"impulse_on_b":obs.impulse_on_b})
}

fn spheres(
    view: &mut View,
    name: &str,
    id: u64,
    z: f64,
    bodies: &[gz_physics_gpu::SolidSphere; 2],
) {
    let turn = Q::from_rotation_z(-std::f64::consts::FRAC_PI_2);
    for (i, b) in bodies.iter().enumerate() {
        let pos = V::Z * z + turn * V::from(b.position);
        let rot = turn * Q::from_array(b.orientation);
        let color = if i == 0 {
            [0.08, 0.69, 0.85, 1.]
        } else {
            [0.96, 0.58, 0.17, 1.]
        };
        let tag = format!("{name}_{}", ["a", "b"][i]);
        view.add(
            id + i as u64 * 4,
            &tag,
            pos.to_array(),
            rot.to_array(),
            json!({"kind":"sphere","radius":b.radius}),
            color,
        );
        view.add(
            id + i as u64 * 4 + 2,
            &format!("{tag}_spin"),
            (pos + rot * V::Y * b.radius).to_array(),
            rot.to_array(),
            json!({"kind":"sphere","radius":0.015}),
            [0.97, 0.97, 0.97, 1.],
        );
    }
}

pub fn record(adapter: &str, out: &Path) -> Result<()> {
    fs::create_dir(out)?;
    let scratch = tempfile::tempdir_in(out)?;
    let dir = scratch.path();
    let mut control = SpherePairBlockBatch::with_rotation(
        adapter,
        reference::fixture(127),
        SphereRotation::FourthOrder,
    )?;
    save(dir, &Cut::capture(&control)?, "none")?;
    let mut frames = Vec::new();
    let mut index = Vec::new();
    let mut loaded = false;
    for k in 0..=56 {
        let t = control.tick();
        let (before, _) = read(dir)?;
        let mut at = "cold";
        let mut after_kill_tick = 0;
        if k != 0 {
            at = ["restored", "solved", "prepared", "committed"][(k - 1) % 4];
            control.step_block(8)?;
            launch(dir, adapter, t, 8, at)?;
            let (found, seen) = read(dir)?;
            let expected = if at == "committed" {
                Cut::capture(&control)?.to_bytes()?
            } else {
                before
            };
            if found != expected {
                return Err(format!("durable cut differs after {at} at {t}").into());
            }
            after_kill_tick = seen.tick();
            if at != "committed" {
                launch(dir, adapter, t, 8, "none")?;
            }
        }
        let (raw, reopened) = read(dir)?;
        let reference = Cut::capture(&control)?.to_bytes()?;
        if raw != reference {
            return Err("replayed GPU cut differs from uninterrupted".into());
        }
        let mut v = View::default();
        let a = row(&mut v, "uninterrupted", 100, 0.26, &control.lanes()[0]);
        let b = row(&mut v, "recovered", 200, -0.26, &reopened.observations()[0]);
        if !loaded && reopened.observations()[0].contact_ticks() != 0 {
            fs::write(out.join("first-impact.gzpair"), &raw)?;
            loaded = true;
        }
        // Four stations identify the boundary where the worker was killed.
        // The green marker starts beside the scale and moves above the selected station.
        let stops = ["restored", "solved", "prepared", "committed"];
        for (i, tag) in stops.iter().enumerate() {
            let y = 0.27 - 0.18 * i as f64;
            v.cuboid(
                300 + 4 * i as u64,
                tag,
                [0., y, -0.51],
                [0.06, 0.13, 0.035],
                [0.28, 0.32, 0.39, 1.],
            );
        }
        let y = stops
            .iter()
            .position(|tag| *tag == at)
            .map_or(0.45, |i| 0.27 - 0.18 * i as f64);
        v.add(
            330,
            "last_kill",
            [0., y, -0.464],
            [0., 0., 0., 1.],
            json!({"kind":"sphere","radius":0.032}),
            [0.19, 0.88, 0.51, 1.],
        );
        frames.push(v.finish_at(k as u64 + 1, reopened.tick() * 1_000_000));
        index.push(
            json!({"tick":reopened.tick(),"phase":at,"durable_tick_after_kill":after_kill_tick,
            "sha256":format!("{:x}",Sha256::digest(&raw)), "rows":[a,b]}),
        );
    }
    if !loaded {
        return Err("witness does not show a real contact".into());
    }
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
    fs::write(
        out.join("index.json"),
        serde_json::to_vec(
            &json!({"device":control.device_info().name,"lanes":127,"rendered_lane":0,"real_kills":56,"frames":index}),
        )?,
    )?;
    Ok(())
}

/// Compare two completed fault campaigns at identical committed physical states. Only
/// acknowledged bytes enter the two cumulative gauges, including after a lost acknowledgement.
pub fn compare(full: &Path, compact: &Path, out: &Path) -> Result<()> {
    use std::collections::BTreeMap;
    let load = |p: &Path| -> Result<Value> {
        Ok(serde_json::from_reader(std::fs::File::open(
            p.join("index.json"),
        )?)?)
    };
    let runs = [load(full)?, load(compact)?];
    if runs[0]["lanes"] != runs[1]["lanes"] || runs[0]["seed"] != runs[1]["seed"] {
        return Err("source populations differ".into());
    }
    let frames_a = runs[0]["frames"].as_array().ok_or("missing full frames")?;
    let frames_b = runs[1]["frames"]
        .as_array()
        .ok_or("missing optimized frames")?;
    if frames_a.len() != frames_b.len() || frames_a.is_empty() {
        return Err("different or empty witness lengths".into());
    }
    let measured = |run: &Value| -> Result<BTreeMap<u64, u64>> {
        let mut m = BTreeMap::new();
        for ack in run["measurements"]
            .as_array()
            .ok_or("missing measurements")?
        {
            let tick = ack["tick"].as_u64().ok_or("missing measurement tick")?;
            if m.insert(
                tick,
                ack["wire_bytes"].as_u64().ok_or("missing payload bytes")?,
            )
            .is_some()
            {
                return Err("duplicate measurement".into());
            }
        }
        Ok(m)
    };
    let amounts = [measured(&runs[0])?, measured(&runs[1])?];
    if !amounts[0].keys().eq(amounts[1].keys()) {
        return Err("different acknowledgement cuts".into());
    }
    let scale = amounts
        .iter()
        .map(|m| m.values().sum::<u64>())
        .max()
        .unwrap();
    if scale == 0 {
        return Err("no measured payloads".into());
    }
    let mut frames = Vec::new();
    let mut rows = Vec::new();
    let mut total = [0u64; 2];
    for (i, (a, b)) in frames_a.iter().zip(frames_b).enumerate() {
        if a != b || a["cut_sha256"].as_str().is_none() {
            return Err(format!("physical state or ownership differs at frame {i}").into());
        }
        let tick = a["tick"].as_u64().ok_or("missing physical tick")?;
        let mut v = View::default();
        let mut physical = Vec::new();
        for (k, tag) in ["full", "continuation"].iter().enumerate() {
            let mut p = a["rows"][1].clone();
            let z = [0.28, -0.15][k];
            let bodies = serde_json::from_value(p["bodies"].clone())?;
            spheres(&mut v, tag, 100 + k as u64 * 100, z, &bodies);
            p["name"] = json!(tag);
            p["offset_z"] = json!(z);
            physical.push(p);
            total[k] += amounts[k].get(&tick).copied().unwrap_or(0);
            let height = -0.44 - 0.12 * k as f64;
            v.cuboid(
                300 + 8 * k as u64,
                &format!("{tag}_payload_scale"),
                [0., 0., height],
                [0.022, 0.72, 0.015],
                [0.27, 0.32, 0.4, 1.],
            );
            let y = 0.36 - 0.72 * total[k] as f64 / scale as f64;
            v.add(
                304 + 8 * k as u64,
                &format!("{tag}_payload_bytes"),
                [0., y, height + 0.021],
                [0., 0., 0., 1.],
                json!({"kind":"sphere","radius":0.032}),
                [[0.98, 0.57, 0.2, 1.], [0.18, 0.86, 0.55, 1.]][k],
            );
        }
        frames.push(v.finish_at(i as u64 + 1, tick * 1_000_000));
        rows.push(json!({"tick":tick,"cut_sha256":a["cut_sha256"],"rows":physical,"acknowledged":amounts[0].contains_key(&tick),"cumulative_payload_bytes":total}));
    }
    for (run, bytes) in runs.iter().zip(total) {
        if run["totals"]["wire_bytes"].as_u64() != Some(bytes) {
            return Err("campaign totals disagree".into());
        }
    }
    fs::create_dir(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
    fs::write(
        out.join("index.json"),
        serde_json::to_vec(
            &json!({"seed":runs[0]["seed"],"lanes":runs[0]["lanes"],"scale_bytes":scale,"acknowledged_ticks":amounts[0].len(),"frames":rows}),
        )?,
    )?;
    Ok(())
}
