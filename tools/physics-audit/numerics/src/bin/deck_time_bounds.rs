// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf};
fn main() -> Result<(), Box<dyn Error>> {
    let p = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply bounds directory")?,
    );
    let before: Vec<Value> = serde_json::from_slice(&fs::read(
        p.join("../multifidelity-native-deck-bearing/tips.json"),
    )?)?;
    let after: Vec<Value> = serde_json::from_slice(&fs::read(p.join("tips.json"))?)?;
    let mut early = [0; 3];
    let mut domains = 0;
    let mut split = 0;
    let mut cases = 0;
    assert_eq!(before.len(), after.len());
    for (a, b) in before.iter().zip(&after) {
        for label in ["duration_s", "support_edge", "end", "normal_ns", "event_s"] {
            assert_eq!(a[label], b[label]);
        }
        let ac = a["registered"].as_array().unwrap();
        let bc = b["registered"].as_array().unwrap();
        assert_eq!(ac.len(), bc.len());
        for (c, d) in ac.iter().zip(bc) {
            cases += 1;
            for label in [
                "tilted",
                "fraction_s",
                "pressure_end_s",
                "first_possible_ordinary_tick",
            ] {
                assert_eq!(c[label], d[label]);
            }
            let expected = d["first_possible_ordinary_tick"].as_u64().unwrap();
            assert_eq!(expected, d["first_ordinary_tick"].as_u64().unwrap());
            let h = expected
                .checked_sub(c["first_ordinary_tick"].as_u64().unwrap())
                .unwrap();
            early[h as usize] += 1;
            domains = domains.max(d["max_certified_domains"].as_u64().unwrap());
            split += d["timing_subdivided_spans"].as_u64().unwrap();
        }
    }
    assert_eq!(cases, 168);
    assert!(domains > 1 && split > 0);
    let summary = json!({"matched_public_trials":cases,"old_early_by_0_1_2_ticks":early,"current_on_exact_tick":cases,"max_time_domains":domains,"spans_requiring_subdivision":split});
    fs::write(
        p.join("checks/before-after.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    println!("{summary}");
    Ok(())
}
