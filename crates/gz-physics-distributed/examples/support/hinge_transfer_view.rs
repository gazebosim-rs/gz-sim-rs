// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Add real private worker identities, durable native hashes and process loss to the articulated replay.
use super::{Result, State, View};
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    fs::{self, File},
    path::Path,
};
fn read(path: &Path) -> Result<Value> {
    Ok(serde_json::from_reader(File::open(path)?)?)
}
fn verify_durable_interval(
    dir: &Path,
    tick: u64,
    session: &[u8],
    cut: &str,
    native_state: &str,
) -> Result<()> {
    for (suffix, committed) in [("intent", false), ("receipt", true)] {
        let bytes = fs::read(dir.join(format!("decision.{tick}.{suffix}")))?;
        if bytes.len() < 100
            || bytes.get(..8) != Some(b"GZHNG001")
            || u16::from_le_bytes(bytes[8..10].try_into()?) as usize + 12 != bytes.len()
            || bytes[10..12] != [0; 2]
        {
            return Err("invalid durable interval framing".into());
        }
        let meta = &bytes[12..bytes.len() - 32];
        let sum = Sha256::digest(meta);
        let hex = |s: &[u8]| s.iter().map(|n| format!("{n:02x}")).collect::<String>();
        if sum.as_slice() != &bytes[bytes.len() - 32..]
            || &meta[..16] != session
            || hex(&meta[16..48]) != cut
            || u32::from_le_bytes(meta[48..52].try_into()?) != 2
            || u64::from(u16::from_le_bytes(meta[52..54].try_into()?)) != tick
        {
            return Err("durable run/tick/checksum mismatch".into());
        }
        let receipt_pos = if tick == 220 {
            if meta[54] != 1 || u64::from_le_bytes(meta[55..63].try_into()?) != 0.375f64.to_bits() {
                return Err("missing durable later effort".into());
            }
            63
        } else {
            if meta[54] != 0 {
                return Err("spurious durable motor effort".into());
            }
            55
        };
        if committed {
            if meta.len() != receipt_pos + 33
                || meta[receipt_pos] != 1
                || hex(&meta[receipt_pos + 1..]) != native_state
            {
                return Err("durable receipt failed independent native state".into());
            }
        } else if meta.len() != receipt_pos + 1 || meta[receipt_pos] != 0 {
            return Err("intent already contains a completed receipt".into());
        }
    }
    Ok(())
}
pub struct Transfer {
    pub label: &'static str,
    pub summary: Value,
    frames: Vec<Value>,
    original: Value,
    moved: bool,
}
impl Transfer {
    pub fn read(dir: &Path, mode: &str) -> Result<Self> {
        let summary = read(&dir.join("summary.json"))?;
        let source = read(&dir.join("source-cut.json"))?;
        let ref_frames: Vec<Value> =
            serde_json::from_reader(File::open(dir.join("reference.json"))?)?;
        let owned_frames: Vec<Value> =
            serde_json::from_reader(File::open(dir.join("handoff.json"))?)?;
        if summary["native_profile"] != "GZART001"
            || summary["physical_origin_tick"] != 120
            || summary["native_cut_bytes"] != 377
            || summary["exact_frames"].as_u64().is_some_and(|n| n != 257)
            || source["pid"] != summary["source_pid"]
            || source["slot"] != summary["source_slot"]
            || source["owner_epoch"] != 1
            || source["native_sha256"] != summary["native_cut_sha256"]
            || source["state"] != ref_frames[0]["state"]
            || owned_frames.len() != 257
            || ref_frames.len() != 257
        {
            return Err("unexpected owner-transfer cut".into());
        }
        let decision = fs::read(dir.join("decision.bin"))?;
        let native = decision
            .windows(8)
            .position(|w| w == b"GZART001")
            .and_then(|offset| decision.get(offset..offset + 377))
            .ok_or("durable decision did not contain the complete native cut")?;
        if decision.len() != summary["durable_decision_bytes"].as_u64().unwrap() as usize
            || format!("{:x}", Sha256::digest(native)) != summary["native_cut_sha256"]
            || !decision.starts_with(b"GZHNG001")
        {
            return Err("durable native construction or digest failed".into());
        }
        let session = if summary["exclusive_source_and_target_duplicate_refusals"] == true {
            let session = fs::read(dir.join("decision.run"))?;
            if session.len() != 16
                || decision.get(12..28) != Some(&session)
                || summary["committed_journal_entries"] != 256
            {
                return Err("private run or confirmed journal length differs".into());
            }
            Some(session)
        } else {
            None
        };
        let mut previous = summary["initial_target_pid"]
            .as_u64()
            .ok_or("no installed recipient")?;
        let mut observed = vec![];
        for (j, (a, b)) in ref_frames.iter().zip(&owned_frames).enumerate() {
            let t = j as u64 + 120;
            let pid = b["pid"].as_u64().ok_or("missing worker PID")?;
            if a["pid"] != ref_frames[0]["pid"]
                || !a["slot"].is_null()
                || a["owner_epoch"] != 0
                || a["state"] != b["state"]
                || a["native_sha256"] != b["native_sha256"]
                || a["logical_tick"] != t
                || b["logical_tick"] != t
                || a["offset"] != j
                || b["offset"] != j
                || a["time_ns"] != t * 1_000_000
                || a["time_ns"] != b["time_ns"]
                || a["worker_local_clock_ns"] != a["time_ns"]
                || b["worker_local_clock_ns"] != (j as u64) * 1_000_000
                || b["owner_epoch"] != 2
                || b["slot"] != summary["target_slot"]
            {
                return Err(format!("native process or public coordinate mismatch at {t}").into());
            }
            if j > 0
                && let Some(id) = &session
            {
                verify_durable_interval(
                    dir,
                    t,
                    id,
                    source["native_sha256"].as_str().unwrap(),
                    b["native_sha256"].as_str().unwrap(),
                )?;
            }
            if pid != previous {
                let h = &summary["hard_losses"][observed.len()];
                if h["killed_pid"] != previous
                    || h["fresh_pid"] != pid
                    || h["lost_reply_at_tick"] != t
                    || h["committed_steps_replayed"] != j - 1
                    || h["uncommitted_native_receipt_observed"] != b["native_sha256"]
                {
                    return Err("fault receipt did not identify the recovered state".into());
                }
                observed.push(t);
            }
            previous = pid;
        }
        if observed != [121, 257]
            || summary["last_target_pid"] != previous
            || ref_frames[0]["native_sha256"] != source["native_sha256"]
        {
            return Err("missing hard process recovery".into());
        }
        let moved = mode == "restored";
        Ok(Self {
            label: if moved {
                "transferred_worker"
            } else {
                "uninterrupted_worker"
            },
            summary,
            frames: if moved { owned_frames } else { ref_frames },
            original: source,
            moved,
        })
    }
    pub fn clock(&self, at: usize) -> u64 {
        self.frames[at]["worker_local_clock_ns"].as_u64().unwrap()
    }
    pub fn draw(&self, view: &mut View, at: usize, state: &State) -> Result<Value> {
        let row = &self.frames[at];
        let raw = &row["state"];
        let same = |v: &Value, arr: &[f64]| {
            v.as_array().is_some_and(|values| {
                values.len() == arr.len()
                    && values
                        .iter()
                        .zip(arr)
                        .all(|(v, f)| v.as_f64().map(f64::to_bits) == Some(f.to_bits()))
            })
        };
        if raw["public_joint"] != 9
            || raw["native_joint"] != 0
            || raw["q"].as_f64().map(f64::to_bits) != Some(state.q.to_bits())
            || raw["qdot"].as_f64().map(f64::to_bits) != Some(state.qdot.to_bits())
        {
            return Err("local physical control differs from process hinge".into());
        }
        for (j, l) in state.links.iter().enumerate() {
            let v = &raw["links"][j];
            if v["public_link"] != l.public_link
                || v["public_visual"] != l.public_visual
                || v["native_body"] != l.native_body
                || !same(&v["position"], &l.position)
                || !same(&v["orientation"], &l.orientation_xyzw)
                || !same(&v["velocity"], &l.com_velocity)
                || !same(&v["angular_velocity"], &l.angular_velocity)
            {
                return Err("local physical control differs from process links".into());
            }
        }
        let t = row["logical_tick"].as_u64().unwrap();
        let pid = row["pid"].as_u64().unwrap();
        let epoch = row["owner_epoch"].as_u64().unwrap();
        let slot = row["slot"]
            .as_u64()
            .map_or_else(|| "control".to_owned(), |w| format!("worker_{w}"));
        let base = 4_000_000 + at as u64 * 20;
        view.cuboid(
            base,
            &format!(
                "{}_logical_tick_{t}_{slot}_epoch_{epoch}_pid_{pid}",
                self.label
            ),
            [-2.55, -2.10, 0.25],
            [0.28, 0.24, 0.35],
            if self.moved {
                [0.97, 0.52, 0.17, 1.]
            } else {
                [0.21, 0.61, 0.77, 1.]
            },
        );
        let mut markers = vec![];
        if self.moved {
            let source_pid = self.original["pid"].as_u64().unwrap();
            let label =
                format!("retired_source_worker_0_epoch_1_pid_{source_pid}_native_cut_at_tick_120");
            view.cuboid(
                base + 2,
                &label,
                [-2.92, -2.10, 0.25],
                [0.23, 0.24, 0.30],
                [0.44, 0.51, 0.57, 1.],
            );
            markers.push(label);
            for (j, loss) in self.summary["hard_losses"]
                .as_array()
                .unwrap()
                .iter()
                .enumerate()
                .filter(|(_, h)| h["lost_reply_at_tick"].as_u64().unwrap() <= t)
            {
                let label = format!(
                    "lost_step_at_{}_killed_pid_{}_fresh_pid_{}_prior_committed_steps_replayed_{}",
                    loss["lost_reply_at_tick"],
                    loss["killed_pid"],
                    loss["fresh_pid"],
                    loss["committed_steps_replayed"]
                );
                view.cuboid(
                    base + 4 + 2 * j as u64,
                    &label,
                    [-2.55 + j as f64 * 0.38, -2.48, 0.25],
                    [0.25, 0.24, 0.33],
                    [0.91, 0.36, 0.16, 1.],
                );
                markers.push(label);
            }
        }
        if self.moved && self.summary["exclusive_source_and_target_duplicate_refusals"] == true {
            let width = (1.45 * at as f64 / 256.).max(0.035);
            let label = format!("leased_owner_2_durably_confirmed_intervals_{at}_at_tick_{t}");
            view.cuboid(
                base + 10,
                &label,
                [-2.6 + width / 2., -2.83, 0.22],
                [width, 0.14, 0.25],
                [0.35, 0.75, 0.56, 1.],
            );
            markers.push(label);
        }
        Ok(
            json!({"logical_tick":t,"absolute_ns":row["time_ns"],"local_ns":row["worker_local_clock_ns"],"slot":row["slot"],"epoch":epoch,"pid":pid,"native_sha256":row["native_sha256"],"markers":markers}),
        )
    }
}
