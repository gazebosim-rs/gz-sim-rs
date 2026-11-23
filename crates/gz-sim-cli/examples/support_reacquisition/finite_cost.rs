// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time six finite-primary policies in balanced order on 30 released donors.
use super::*;
fn stats(mut v: Vec<u64>) -> Value {
    v.sort_unstable();
    let q = |p: usize| v[(v.len() * p).div_ceil(100) - 1];
    json!({"n":v.len(),"median_ns":q(50),"p95_ns":q(95),"p99_ns":q(99),"max_ns":v.last()})
}
pub(crate) fn write(source: &Path, out: &Path, transfer: bool) -> Result<()> {
    let profiles = if transfer {
        &TRANSFERS[..]
    } else {
        &PROFILES[..]
    };
    fs::create_dir_all(out)?;
    let mut selections = std::collections::BTreeMap::new();
    for k in 0..1250 {
        let f = survey::fixture(k);
        let offset = (f.offset_m * 1000.).round() as u32;
        if [175, 200, 225, 250, 275].contains(&offset) {
            selections.entry((offset, f.switched)).or_insert(k);
        }
    }
    let mut rows = vec![];
    let mut first = vec![Vec::<u64>::new(); profiles.len()];
    let mut suffix = vec![Vec::<u64>::new(); profiles.len()];
    let mut window = vec![Vec::<u64>::new(); profiles.len()];
    for (idx, k) in selections.into_values().enumerate() {
        let file = source.join("cases").join(format!("case-{k:04}.json"));
        let archive: Value = serde_json::from_reader(fs::File::open(file)?)?;
        let f = survey::fixture(k);
        for (stroke, cut) in survey::cuts(f)?.0.into_iter().enumerate() {
            let Some(cut) = cut else {
                continue;
            };
            let donor = archive["result"]["trials"]
                .as_array()
                .unwrap()
                .iter()
                .find(|r| r["stroke"] == stroke + 1)
                .ok_or("missing archived donor")?;
            for repeat in 0..14 {
                for offset in 0..profiles.len() {
                    let i = (idx + stroke + repeat + offset) % profiles.len();
                    let mode = profiles[i];
                    let mut costs = Vec::with_capacity(HORIZON as usize);
                    let (r, _) = measured_path(f, &cut, mode, false, Some(&mut costs))?;
                    let reread: Value = serde_json::from_slice(&serde_json::to_vec(&r)?)?;
                    if reread != donor["paths"][i] || costs.len() != HORIZON as usize {
                        return Err(format!(
                            "timing changed matched history {k}/{} {} ",
                            stroke + 1,
                            mode.id
                        )
                        .into());
                    }
                    if repeat >= 2 {
                        first[i].push(costs[0]);
                        suffix[i].extend_from_slice(&costs[1..]);
                        window[i].push(costs.iter().sum());
                        rows.push(json!({"case":k,"stroke":stroke+1,"profile":mode.id,"repeat":repeat-2,"ns":costs}));
                    }
                }
            }
        }
    }
    let mut summary = vec![];
    for (((f, s), w), p) in first.into_iter().zip(suffix).zip(window).zip(profiles) {
        summary.push(json!({"profile":p.id,"first":stats(f),"suffix":stats(s),"total":stats(w)}));
    }
    write_json(
        &out.join("cost.json"),
        &json!({"cpu_scope":"native interval including planning and projection","summary":summary,"rows":rows}),
    )
}
