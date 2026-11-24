// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Audit a completed SSH operator qualification and plot times from its observed WebSocket events.
use plotters::prelude::*;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs,
    path::{Path, PathBuf},
};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
const RUNS: [(&str, &str, &str); 5] = [
    (
        "before-negative-retry",
        "Before transfer · link still down",
        "before-both-automatic-reopen-3-negative",
    ),
    (
        "before-positive-direct",
        "Before transfer · direct",
        "before-up-automatic-reopen-4-positive",
    ),
    (
        "after-positive-writer",
        "After transfer · writer held",
        "after-down-automatic-reopen-4-positive",
    ),
    (
        "after-negative-direct",
        "After transfer · direct",
        "after-both-exhaust-reopen-1-negative",
    ),
    (
        "after-positive-changed",
        "After transfer · archive changed",
        "",
    ),
];
#[derive(Clone, Copy, PartialEq)]
enum State {
    Waiting,
    Automatic,
    Parked,
    Installing,
    Paused,
}
impl State {
    fn color(self) -> RGBColor {
        match self {
            Self::Waiting => RGBColor(137, 147, 157),
            Self::Automatic => RGBColor(45, 99, 148),
            Self::Parked => RGBColor(221, 147, 49),
            Self::Installing => RGBColor(51, 141, 142),
            Self::Paused => RGBColor(49, 109, 69),
        }
    }
}
fn load(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn check_poses(file: &Path, tick: u64) -> Result<()> {
    let scene = &load(file)?["warm"];
    let poses = scene["visuals"].as_array().ok_or("missing poses")?;
    if scene["tick"] != tick
        || scene["paused"] != true
        || poses.len() != 4
        || !poses
            .iter()
            .all(|v| v["max_quaternion_error"] == 0.0 && v["max_position_error_m"] == 0.0)
    {
        return Err(format!("pose or cut mismatch at {}", file.display()).into());
    }
    Ok(())
}
struct Row {
    label: &'static str,
    end: String,
    events: Vec<(f64, State)>,
    duration: f64,
}
fn main() -> Result<()> {
    let dir = PathBuf::from(
        std::env::args()
            .nth(1)
            .ok_or("expected SSH evidence directory")?,
    );
    let mut rows = Vec::new();
    let mut data = Vec::new();
    for (name, label, baseline) in RUNS {
        let d = dir.join(name);
        let cut = load(&d.join("fault-ready.json"))?;
        let tick = cut["tick"].as_u64().ok_or("missing initial tick")?;
        check_poses(&d.join("warm.json"), tick)?;
        let packet = load(&d.join("packet-loss.json"))?;
        assert_eq!(packet["packets_dropped"], true);
        assert!(
            packet["counters"]
                .as_array()
                .ok_or("missing counters")?
                .iter()
                .all(|c| c["drops"].as_u64().unwrap_or(0) > 0)
        );
        let report = load(&d.join("operator.json"))?;
        let mut events = vec![(0., State::Waiting)];
        let mut last = 0.;
        let mut play_time = None;
        let mut first_fence = None;
        let mut tokens = Vec::new();
        let mut last_request = None;
        let mut admitted_token = None;
        let mut restored = None;
        let mut expect_scene = false;
        for line in fs::read_to_string(d.join("wire.jsonl"))?.lines() {
            let msg: Value = serde_json::from_str(line)?;
            last = msg["at_seconds"].as_f64().ok_or("missing elapsed time")?;
            let f = &msg["frame"];
            if play_time.is_none()
                && msg["direction"] == "sent"
                && f["kind"] == "control"
                && f["action"] == "resume"
            {
                play_time = Some(last);
            }
            let Some(origin) = play_time else {
                continue;
            };
            last -= origin;
            if expect_scene && (f["kind"] != "scene" || f["paused"] != true) {
                return Err("restored identity was not followed by the paused scene".into());
            }
            expect_scene = false;
            let state = match f["kind"].as_str() {
                Some("recovery_status") if f["phase"] == "restoring" => State::Automatic,
                Some("operator_fence") => {
                    first_fence.get_or_insert(last);
                    if f["epoch"] != 0 || f["tick"] != tick {
                        return Err("parked clock drifted".into());
                    }
                    let token = f["token"].as_str().ok_or("missing park token")?.to_owned();
                    if tokens.last() != Some(&token) {
                        if tokens.contains(&token) {
                            return Err("reused conditional offer".into());
                        }
                        tokens.push(token);
                    }
                    State::Parked
                }
                Some("recover_workers") if msg["direction"] == "sent" => {
                    last_request = Some(f.clone());
                    continue;
                }
                Some("operator_reply") if f["accepted"] == true => {
                    let request = last_request.take().ok_or("accepted request missing")?;
                    if f["request_id"] != request["request_id"]
                        || request["epoch"] != 0
                        || request["tick"] != tick
                    {
                        return Err("accepted request differs from fenced cut".into());
                    }
                    admitted_token = Some(request["token"].clone());
                    State::Installing
                }
                Some("operator_restored") => {
                    if restored.is_some()
                        || f["version"] != 1
                        || f["epoch"] != 0
                        || f["tick"] != tick
                        || Some(&f["token"]) != admitted_token.as_ref()
                        || f != &report["operator_restored"]
                    {
                        return Err(
                            "restored identity differs from the admitted offer or report".into(),
                        );
                    }
                    restored = Some(f.clone());
                    expect_scene = true;
                    continue;
                }
                Some("scene")
                    if events.last().is_some_and(|(_, s)| *s == State::Installing)
                        && f["paused"] == true =>
                {
                    State::Paused
                }
                _ => continue,
            };
            if events.last().unwrap().1 != state {
                events.push((last, state));
            }
        }
        let sha = fs::read_to_string(d.join("final-archive.sha256"))?;
        let sha = sha.trim();
        let mut audit = json!({"trial":name,"initial_tick":tick,"receipt_sha256":cut["sha256"],"final_sha256":sha,"packets":packet["counters"],
           "packet_cut_seconds":packet["full_cut_seconds"],"operator":report,"distinct_fence_offers":tokens.len(),"seconds_from_play_to_park":first_fence.ok_or("missing first fence")?});
        let end = if baseline.is_empty() {
            assert!(restored.is_none());
            assert_eq!(report["ineligible_refused"], true);
            assert_eq!(report["conflicting_bytes_preserved"], true);
            assert_eq!(report["final_status"]["eligible"], false);
            assert_eq!(
                sha,
                load(&dir.join("after-negative-direct/fault-ready.json"))?["sha256"]
                    .as_str()
                    .unwrap()
            );
            "Inspection only".into()
        } else {
            if restored.is_none() || expect_scene {
                return Err("missing paused handoff after restored identity".into());
            }
            let prior = dir
                .join("../../multifidelity-convoy-auto-reopen")
                .join(baseline)
                .join("recovered/barrier.gzmultisphere");
            if sha != format!("{:x}", Sha256::digest(fs::read(&prior)?)) {
                return Err(format!("independent archive mismatch for {name}").into());
            }
            check_poses(&d.join("paused.json"), tick + 1)?;
            assert_eq!(report["first_resumed_archive_exact"], true);
            if tick == 149 {
                let transfer = load(&d.join("transfer.json"))?;
                assert_eq!(transfer["before_owners"], json!([0, 1, 1]));
                assert_eq!(transfer["after_owners"], json!([1, 0, 0]));
                assert_eq!(transfer["observations_unchanged"], true);
                audit["loaded_transfer"] = transfer;
            }
            let processes = load(&d.join("processes.json"))?;
            let a = processes["original_transports"]
                .as_array()
                .ok_or("missing original workers")?;
            let b = processes["new_transports"]
                .as_array()
                .ok_or("missing replacement workers")?;
            assert!(a.len() == 3 && b.len() == 3 && a.iter().all(|x| !b.contains(x)));
            audit["reference_archive"] = json!(baseline);
            audit["equal_pose_count"] = json!(4);
            audit["resumed_tick"] = json!(tick + 1);
            format!("Paused at {}", tick + 1)
        };
        rows.push(Row {
            label,
            end,
            events,
            duration: last,
        });
        data.push(audit);
    }
    fs::write(
        dir.join("qualification.json"),
        serde_json::to_vec_pretty(&json!({"step_milliseconds":1,"trials":data}))?,
    )?;
    render(&dir.join("timeline.svg"), &rows)
}
fn render(path: &Path, rows: &[Row]) -> Result<()> {
    let area = SVGBackend::new(path, (1160, 374)).into_drawing_area();
    area.fill(&WHITE)?;
    let ink = RGBColor(37, 50, 58);
    let heading = ("sans-serif", 23)
        .into_font()
        .style(FontStyle::Bold)
        .color(&ink);
    let label = ("sans-serif", 14).into_font().color(&ink);
    area.draw(&Text::new(
        "Operator recovery under bidirectional worker packet loss",
        (16, 12),
        heading,
    ))?;
    area.draw(&Text::new(
        "Elapsed wall time since Play, seconds",
        (16, 48),
        label.clone(),
    ))?;
    let end = (rows.iter().map(|r| r.duration).fold(0., f64::max) / 5.).ceil() as i32 * 5;
    let x = |s: f64| 266 + (s / end as f64 * 714.).round() as i32;
    for t in (0..=end).step_by(5) {
        area.draw(&PathElement::new(
            [(x(t as f64), 95), (x(t as f64), 310)],
            RGBColor(216, 224, 228),
        ))?;
        area.draw(&Text::new(
            t.to_string(),
            (x(t as f64) - 4, 75),
            label.clone(),
        ))?;
    }
    for (i, row) in rows.iter().enumerate() {
        let y = 108 + i as i32 * 41;
        area.draw(&Text::new(row.label, (16, y + 3), label.clone()))?;
        for (j, (s, state)) in row.events.iter().enumerate() {
            let stop = row.events.get(j + 1).map_or(row.duration, |next| next.0);
            area.draw(&Rectangle::new(
                [(x(*s), y), (x(stop).max(x(*s) + 1), y + 19)],
                state.color().filled(),
            ))?;
        }
        area.draw(&Text::new(
            row.end.as_str(),
            (x(row.duration) + 8, y + 3),
            label.clone(),
        ))?;
    }
    for (left, state, text) in [
        (16, State::Waiting, "Awaiting fault detection"),
        (213, State::Automatic, "Automatic repair"),
        (387, State::Parked, "Parked at cut"),
        (538, State::Installing, "Guarded reload"),
        (708, State::Paused, "Restored session"),
    ] {
        area.draw(&Rectangle::new(
            [(left, 333), (left + 13, 346)],
            state.color().filled(),
        ))?;
        area.draw(&Text::new(text, (left + 18, 332), label.clone()))?;
    }
    area.present()?;
    Ok(())
}
