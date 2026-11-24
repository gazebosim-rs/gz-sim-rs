// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bind the final recurrent-contact window to the ordinary measured placement journal.
use super::{Frame, OWNER, Result, View};
use serde_json::{Value, json};
use std::{
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
};
pub struct Placement {
    events: Vec<Value>,
    control: Vec<Frame>,
    prior: Option<Vec<usize>>,
}
impl Placement {
    pub fn read(dir: &Path, control: &Path) -> Result<Self> {
        let summary: Value =
            serde_json::from_reader(File::open(dir.join("placement-summary.json"))?)?;
        let events = BufReader::new(File::open(dir.join("placement.jsonl"))?)
            .lines()
            .map(|l| Ok(serde_json::from_str::<Value>(&l?)?))
            .collect::<Result<Vec<_>>>()?
            .into_iter()
            .filter(|r| r["operation"] == "automatic whole-component placement")
            .collect::<Vec<_>>();
        let native = BufReader::new(File::open(control)?)
            .lines()
            .map(|l| Ok(serde_json::from_str(&l?)?))
            .collect::<Result<Vec<Frame>>>()?;
        if events.len() != summary["measured_transfers"].as_u64().unwrap() as usize
            || native.len() != 201
            || !events
                .windows(2)
                .all(|p| p[0]["tick"].as_u64() < p[1]["tick"].as_u64())
        {
            return Err("incomplete automatic placement log".into());
        }
        Ok(Self {
            events,
            control: native,
            prior: None,
        })
    }
    pub fn draw(&mut self, view: &mut View, base: u64, i: usize, f: &Frame) -> Result<Value> {
        let original = &self.control[i];
        let physical =
            |s: &Frame| bincode::serialize(&(s.tick, &s.bodies, &s.pairs, &s.floors, &s.wrenches));
        if physical(f)? != physical(original)?
            || f.trajectory_sha256_sample != original.trajectory_sha256_sample
        {
            return Err("ordinary automatic/native physical frames differ".into());
        }
        let n = self
            .events
            .partition_point(|r| r["tick"].as_u64().unwrap() <= f.tick);
        let last = self
            .events
            .get(n.checked_sub(1).ok_or("no recorded source move")?)
            .unwrap();
        let d = &last["details"];
        let at = last["tick"].as_u64().unwrap();
        let src = d["source"].as_u64().unwrap() as usize;
        let dst = d["destination"].as_u64().unwrap() as usize;
        let members: Vec<u64> = serde_json::from_value(d["members"].clone())?;
        if at == f.tick {
            let prev = self.prior.as_ref().ok_or("missing previous owner sample")?;
            for &id in &members {
                let k = id as usize - 1;
                if prev[k] != src
                    || f.owners[k] != dst
                    || d["commit_owners_and_generations"][k]
                        != json!([dst as u64, f.generations[k]])
                {
                    return Err("automatic source or destination generation differs".into());
                }
            }
        }
        self.prior = Some(f.owners.clone());
        let ids = members
            .iter()
            .map(u64::to_string)
            .collect::<Vec<_>>()
            .join("_");
        let label = format!("automatic_placement_{n}_at_{at}_worker_{src}_to_{dst}_members_{ids}");
        view.cuboid(
            base + 24,
            &label,
            [-2.1, 2.74, 0.25],
            [0.26 * members.len() as f64, 0.22, 0.34],
            OWNER[dst],
        );
        Ok(
            json!({"moves_through_frame":n,"latest":last,"label":label,"native_control_physical_sha":original.trajectory_sha256_sample}),
        )
    }
}
