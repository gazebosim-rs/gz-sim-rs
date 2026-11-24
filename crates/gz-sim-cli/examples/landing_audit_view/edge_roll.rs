// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay the late change of support in fixture 80 at its measured public boundaries.
use super::*;
#[path = "edge_pressures.rs"]
mod pressures;

const FOCUS_START_US: u64 = 9000;

fn integer(row: &Value, key: &str) -> Result<u64> {
    row[key]
        .as_u64()
        .ok_or_else(|| format!("missing {key}").into())
}

fn number(row: &Value, key: &str) -> Result<f64> {
    row[key]
        .as_f64()
        .filter(|v| v.is_finite())
        .ok_or_else(|| format!("missing finite {key}").into())
}

// Source runs use an absolute actuator clock. Receipts use durations from
// the resumed public boundary. Keep their origins explicit for every event.
fn resume_us(frame: &Value, cut: u64) -> Result<u64> {
    integer(frame, "time_us")?
        .checked_sub(cut)
        .ok_or_else(|| "frame predates restart".into())
}

fn native_marks(trace: &mut Value, path: &Path) -> Result<u64> {
    let recording: Value = read(path)?;
    let receipts = recording["steps"]
        .as_array()
        .ok_or("missing measured steps")?;
    let frames = trace["frames"]
        .as_array_mut()
        .ok_or("missing measured trace")?;
    if frames.len() != 27 || receipts.len() != frames.len() - 1 {
        return Err("expected 26 measured actuator steps".into());
    }
    let cut = integer(&frames[0], "time_us")?;
    let mut capture_tick = None;
    for (i, step) in receipts.iter().enumerate() {
        let start = resume_us(&frames[i], cut)?;
        let end = resume_us(&frames[i + 1], cut)?;
        let frame = &mut frames[i + 1];
        if integer(step, "step")? != i as u64 + 1
            || (number(step, "tick_start_s")? - start as f64 * 1e-6).abs() > 1e-14
        {
            return Err("native receipt clock differs from frame".into());
        }
        if step["native_calls"] != frame["native_steps"]
            || (step["refined"] == true
                && vec3(&step["j_ns"])?.distance(vec3(&frame["j"])?) > 1e-11)
        {
            return Err("receipt effort or impulse disagrees with the public trajectory".into());
        }
        if start == FOCUS_START_US + 500 && step["native_intervals"].is_array() {
            frame["contact_intervals"] = step["native_intervals"].clone();
        }
        if start < FOCUS_START_US {
            continue;
        }
        let mut marks = vec![];
        for hit in step["events"]
            .as_array()
            .ok_or("missing native event list")?
        {
            let [lo, hi]: [f64; 2] = serde_json::from_value(hit["bracket_s"].clone())?;
            if !(0. <= lo && lo <= hi && hi <= (end - start) as f64 * 1e-6 + 1e-14) {
                return Err("native impact bracket outside the displayed interval".into());
            }
            marks.push(mark(0.5 * (lo + hi), vec3(&hit["point_m"])?, &hit["j_ns"])?);
        }
        if let Some(events) = step["finite_events"].as_array() {
            for hit in events {
                let at = number(hit, "elapsed_s")?;
                // A zero-time receipt describes the contact already present at the
                // previous sample. Mark impacts formed inside the displayed interval.
                if at == 0. {
                    continue;
                }
                marks.push(mark(at, vec3(&hit["point_m"])?, &hit["j_ns"])?);
            }
        }
        frame["interval_impacts"] = json!(marks);
        if start == FOCUS_START_US {
            capture_tick = Some(integer(frame, "tick")?);
        }
    }
    capture_tick.ok_or_else(|| "missing late contact window".into())
}

fn mechanics_marks(mechanics: &mut Value) -> Result<(usize, Value)> {
    if mechanics["case"] != 80 || mechanics["stroke"] != 2 {
        return Err("wrong physical reference for edge replay".into());
    }
    let drive = vec3(&mechanics["drive"])?;
    let refs = mechanics["references"]
        .as_array()
        .ok_or("missing independently integrated states")?;
    let from = number(&refs.first().ok_or("empty reference")?["fine"], "elapsed")?;
    let mut frames = mechanics["frames"]
        .as_array()
        .ok_or("missing physical trajectory")?
        .clone();
    if !(2..=27).contains(&frames.len()) || refs.len() < frames.len() {
        return Err("invalid reference horizon".into());
    }
    let cut = integer(&frames[0], "time_us")?;
    let mut captures = json!({});
    for i in 0..frames.len() {
        let end = resume_us(&frames[i], cut)?;
        if refs[i]["qualified"] != true
            || (number(&refs[i]["fine"], "elapsed")? - from - end as f64 * 1e-6).abs() > 1e-12
        {
            return Err("unqualified or misdated mechanical reference".into());
        }
        if i == 0 || resume_us(&frames[i - 1], cut)? < FOCUS_START_US {
            continue;
        }
        let start_us = resume_us(&frames[i - 1], cut)?;
        let start = from + start_us as f64 * 1e-6;
        let mut marks = vec![];
        for event in refs[i]["fine"]["events"]
            .as_array()
            .ok_or("missing reference contacts")?
        {
            let at = number(event, "at_s")?;
            if at > start && at <= from + end as f64 * 1e-6 {
                // The reference integrates in the translating deck's frame.
                marks.push(mark(
                    at - start,
                    vec3(&event["point"])? + at * drive,
                    &event["j"],
                )?);
            }
        }
        frames[i]["interval_impacts"] = json!(marks);
        if start_us == FOCUS_START_US {
            captures["change_of_support"] = frames[i]["tick"].clone();
        }
    }
    let len = frames.len();
    mechanics["frames"] = json!(frames);
    Ok((len, captures))
}

pub(in super::super) fn export(
    previous: &Path,
    current: &Path,
    reference: &Path,
    previous_receipts: &Path,
    current_receipts: &Path,
    out: &Path,
) -> Result<()> {
    if out.try_exists()? {
        return Err(format!("replay already exists at {}", out.display()).into());
    }
    let mut old: Value = read(&previous.join("traces/lane-3.json"))?;
    let mut new: Value = read(&current.join("traces/lane-3.json"))?;
    let fine: Value = read(&current.join("traces/lane-4.json"))?;
    // This comparison may opt into fractional timing for the named pair.
    // The donor, contacts, drive, geometry and the continuous control must match.
    let read_world = |dir: &Path| -> Result<String> {
        let xml = fs::read_to_string(dir.join("world.sdf"))?;
        let option = "<retime_finite_transfer>true</retime_finite_transfer>";
        Ok(xml.replacen(option, "", 1))
    };
    if read_world(previous)? != read_world(current)?
        || read::<Value>(&previous.join("traces/lane-4.json"))? != fine
    {
        return Err("matched scene or continuous native comparison changed".into());
    }
    let tick = native_marks(&mut new, current_receipts)?;
    if native_marks(&mut old, previous_receipts)? != tick {
        return Err("native capture clocks disagree".into());
    }
    let mut control: Value = read(reference)?;
    let (horizon, captures) = mechanics_marks(&mut control)?;
    let parent = out
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = tempfile::Builder::new()
        .prefix(".edge-roll-")
        .tempdir_in(parent)?;
    let before = ("previous", "Previous solver");
    let after = ("revised", "Revised solver");
    bundle(
        stage.path(),
        [&old, &new, &control],
        [before, after, ("mechanics", "Independent mechanics")],
        horizon,
        true,
        "Fixture 80 · Late support change against contact mechanics",
        captures,
    )?;
    let mut numerical_captures = json!({"change_of_support":tick});
    if let Some(frame) = fine["frames"].get(horizon) {
        numerical_captures["beyond_reference"] = frame["tick"].clone();
    }
    bundle(
        &stage.path().join("continuous-native"),
        [&old, &new, &fine],
        [before, after, ("native8192", "8192-call native comparison")],
        27,
        false,
        "Fixture 80 · Late support change against the 8192-call native run",
        numerical_captures,
    )?;
    pressures::draw(&stage.path().join("pressure-alternatives.svg"), &control)?;
    fs::copy(current.join("world.sdf"), stage.path().join("world.sdf"))?;
    fs::rename(stage.path(), out)?;
    Ok(())
}
