// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare each rendered particle center with the independently sampled XPBD state.
use super::{Result, same, vector};
use serde_json::Value;
use std::collections::BTreeSet;

pub(super) fn verify(frames: &[Value], samples: &[Value], names: Vec<String>) -> Result<()> {
    if names.is_empty() || names.iter().collect::<BTreeSet<_>>().len() != names.len() {
        return Err("particle visuals require a nonempty list of distinct visual names".into());
    }
    if frames.is_empty() || frames.len() != samples.len() {
        return Err(
            "particle observations and recorded frames have different or empty counts".into(),
        );
    }
    for (frame, sample) in frames.iter().zip(samples) {
        let time = sample["time_ns"]
            .as_u64()
            .ok_or("missing particle observation clock")?;
        if frame["ns"].as_u64() != Some(time) {
            return Err(format!(
                "particle observation clock differs from recorded frame at {time} ns"
            )
            .into());
        }
        let positions = sample["positions"]
            .as_array()
            .ok_or("missing particle positions")?;
        if positions.len() != names.len() {
            return Err(format!("particle count differs from visual bindings at {time} ns").into());
        }
        let visuals = frame["scene"]["visuals"]
            .as_array()
            .ok_or("missing recorded visuals")?;
        for (name, position) in names.iter().zip(positions) {
            let mut matches = visuals
                .iter()
                .filter(|item| item["name"].as_str() == Some(name));
            let visual = matches
                .next()
                .ok_or_else(|| format!("missing particle visual {name}"))?;
            if matches.next().is_some() || visual["shape"]["kind"] != "sphere" {
                return Err(format!("particle visual {name} must identify one sphere").into());
            }
            if !same(&visual["translation"], vector::<3>(position)?)? {
                return Err(format!(
                    "visual {name} differs from its measured particle at {time} ns"
                )
                .into());
            }
        }
    }
    Ok(())
}

#[cfg(test)]
#[path = "particle_visuals_tests.rs"]
mod tests;
