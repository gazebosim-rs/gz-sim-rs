// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare archived compact-8 solvers at a matched restart and carry a
//! measured within-tick impact into the first committed scene after the event.
use super::*;
#[path = "edge_roll.rs"]
pub(super) mod roll;

fn save(dir: &Path, file: &str, value: &Value) -> Result<()> {
    fs::write(dir.join(file), serde_json::to_vec(value)?)?;
    Ok(())
}

fn mark(elapsed_s: f64, point_m: DVec3, impulse: &Value) -> Result<Value> {
    let ns = Duration::try_from_secs_f64(elapsed_s)?.as_nanos();
    Ok(json!({"elapsed_ns":u64::try_from(ns)?,"point_m":point_m.to_array(),"impulse_ns":impulse}))
}

fn vec3(value: &Value) -> Result<DVec3> {
    Ok(DVec3::from(serde_json::from_value::<[f64; 3]>(
        value.clone(),
    )?))
}

fn bundle(
    out: &Path,
    lanes: [&Value; 3],
    labels: [(&str, &str); 3],
    horizon: usize,
    mechanics: bool,
    title: &str,
    capture_ticks: Value,
) -> Result<()> {
    let path = out.join("traces");
    fs::create_dir_all(&path)?;
    let mut manifest = vec![];
    for (i, (trace, (label, name))) in lanes.into_iter().zip(labels).enumerate() {
        let frames = trace["frames"]
            .as_array()
            .and_then(|f| f.get(..horizon))
            .ok_or("insufficient source history for replay horizon")?;
        let file = format!("traces/{label}.json");
        save(
            out,
            &file,
            &json!({"geometry":trace["geometry"], "frames":frames}),
        )?;
        manifest
            .push(json!({"file":file,"label":label,"name":name,"analytical":mechanics && i==2}));
    }
    save(
        out,
        "comparison.json",
        &json!({"title":title,"pair_totals_only":true,"lanes":manifest,"capture_ticks":capture_ticks}),
    )?;
    rebound::weak::export(out, &out.join("view"), rebound::weak::Study::Native)
}

pub(super) fn export(
    previous: &Path,
    current: &Path,
    controls: &Path,
    receipts: &Path,
    timing: &Path,
    out: &Path,
) -> Result<()> {
    if out.try_exists()? {
        return Err(format!("replay output already exists at {}", out.display()).into());
    }
    let mut old: Value = read(&previous.join("traces/lane-3.json"))?;
    let mut repaired: Value = read(&current.join("traces/lane-3.json"))?;
    let fine: Value = read(&current.join("traces/lane-4.json"))?;
    let mut mechanics: Value = read(controls)?;
    let observed: Value = read(receipts)?;
    for trace in [&old, &repaired, &fine, &mechanics] {
        if trace["frames"]
            .as_array()
            .is_none_or(|frames| frames.len() < 2)
        {
            return Err("missing incoming state and first resumed step".into());
        }
    }
    if mechanics["case"] != 80 || mechanics["stroke"] != 2 {
        return Err("edge slip replay requires fixture 80 stroke 2".into());
    }
    let m = &mechanics["references"];
    let from = m[0]["fine"]["elapsed"]
        .as_f64()
        .ok_or("missing primary boundary")?;
    let hits: Vec<_> = m[1]["fine"]["events"]
        .as_array()
        .ok_or("missing reference impacts")?
        .iter()
        .filter(|e| e["mode"] == "IncipientSlip")
        .collect();
    let [edge] = hits.as_slice() else {
        return Err("no unique incipient slipping edge".into());
    };
    let at = edge["at_s"]
        .as_f64()
        .ok_or("missing reference impact time")?;
    let offset = at - from;
    let lab = vec3(&edge["point"])? + vec3(&mechanics["drive"])? * at;
    let ref_mark = mark(offset, lab, &edge["j"])?;
    let events = observed["events"]
        .as_array()
        .ok_or("missing native impacts")?;
    let mut matching = events.iter().filter_map(|e| {
        let b = &e["bracket_s"];
        let lo = b[0].as_f64()?;
        let hi = b[1].as_f64()?;
        // Comparison allowance is one nanosecond on this authored clock.
        (lo <= hi && lo - 1e-9 <= offset && offset <= hi + 1e-9).then_some((e, 0.5 * (lo + hi)))
    });
    let (hit, at_native) = matching
        .next()
        .ok_or("no native impact near the independent edge event")?;
    if matching.next().is_some() {
        return Err("ambiguous native edge witness".into());
    }
    let actual = mark(at_native, vec3(&hit["point_m"])?, &hit["j_ns"])?;
    repaired["frames"][1]["interval_impacts"] = json!([actual]);
    mechanics["frames"][1]["interval_impacts"] = json!([ref_mark]);

    if repaired["frames"][1]["j"] != observed["j_ns"]
        || repaired["frames"][1]["native_steps"] != observed["native_calls"]
        || fs::read(previous.join("world.sdf"))? != fs::read(current.join("world.sdf"))?
        || read::<Value>(&previous.join("traces/lane-4.json"))? != fine
    {
        return Err(
            "impact receipt, incoming fixture or continuous native control disagrees".into(),
        );
    }
    let costs: Value = read(timing)?;
    let tick = mechanics["frames"][1]["tick"]
        .as_u64()
        .ok_or("missing measured tick")?;
    if costs["case"] != mechanics["case"]
        || costs["stroke"] != mechanics["stroke"]
        || costs["public_tick"] != tick
    {
        return Err("cost sample uses a different matched boundary".into());
    }
    let control_j = vec3(&mechanics["frames"][1]["j"])?;
    for (trace, revision) in [(&mut old, "before"), (&mut repaired, "after")] {
        let row = &mut trace["frames"][1];
        let wall = costs[revision]["wall_ns"]["p95"]
            .as_u64()
            .ok_or("missing measured p95")?;
        if wall == 0 {
            return Err("zero step cost".into());
        }
        let error = vec3(&row["j"])?.distance(control_j);
        row["matched_step"] = json!({"impulse_error_ns":error,"wall_p95_ns":wall});
    }
    let horizon = mechanics["frames"]
        .as_array()
        .ok_or("missing mechanics frames")?
        .len();
    if horizon < 2 || (0..horizon).any(|i| mechanics["references"][i]["qualified"] != true) {
        return Err("unqualified mechanical replay boundary".into());
    }
    // Publish both validated archives together after the native and independent
    // histories have passed the same renderer checks.
    let parent = out
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let staged = tempfile::Builder::new()
        .prefix(".edge-slip-")
        .tempdir_in(parent)?;
    let basic = [
        ("previous", "Previous compact-8"),
        ("repaired", "Repaired compact-8"),
    ];
    bundle(
        staged.path(),
        [&old, &repaired, &mechanics],
        [basic[0], basic[1], ("mechanics", "Independent mechanics")],
        horizon,
        true,
        "Fixture 80 · Edge slip against independent contact mechanics",
        json!({}),
    )?;
    let native_frames = fine["frames"]
        .as_array()
        .ok_or("missing native frames")?
        .len();
    bundle(
        &staged.path().join("continuous-native"),
        [&old, &repaired, &fine],
        [basic[0], basic[1], ("native8192", "8192 native calls")],
        native_frames,
        false,
        "Fixture 80 · Edge slip against the 8192-call native run",
        json!({}),
    )?;
    fs::copy(current.join("world.sdf"), staged.path().join("world.sdf"))?;
    fs::rename(staged.path(), out)?;
    Ok(())
}
