// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Intervene only through the first physical landing. Then change the native
//! resolution on three identical cold graphs for all remaining actuator clocks.
use super::super::secondary::physical::thaw_with_grid;
use super::*;
const CASES: [usize; 8] = [0, 7, 14, 21, 28, 35, 42, 47];
const GRIDS: [u32; 4] = [512, 8192, 16384, 32768];
pub(crate) fn write(dir: &Path, k: usize) -> Result<()> {
    if !CASES.contains(&k) {
        return Err("choose one of nominal cases 0, 7, 14, 21, 28, 35, 42, 47".into());
    }
    let f = fixture(source(k, true));
    let c = survey::cuts(f)?.0[0].ok_or("missing primary donor")?;
    let (mechanics, initial) = super::whole_long::mechanical(f, &c)?;
    let folder = dir.join(format!("case-{k:03}"));
    fs::create_dir_all(folder.join("traces"))?;
    let mut runs = vec![];
    let mut lanes = vec![];
    let mut reports = vec![];
    for n in GRIDS {
        let p = thaw_with_grid(f, initial, n)?;
        let file = format!("traces/native{n}.json");
        let all = p.iter().map(Tick::as_json).collect::<Vec<_>>();
        write_json(
            &folder.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":all}),
        )?;
        if n != 512 {
            lanes.push(json!({"label":format!("native{n}"),"name":format!("Physical first tick, then {n} native calls"),"file":file,"analytical_ticks":[initial.tick]}));
        }
        reports.push(json!({"calls_per_later_tick":n,"first":p[0].as_json(),"end":p.last().unwrap().as_json(),"outcomes":events(&p),"native_calls":p.iter().map(|t|u64::from(t.calls)).sum::<u64>()}));
        eprintln!("completed shared-boundary donor {k}, native {n}");
        runs.push(p);
    }
    let differences = (0..4)
        .map(|i| compare(&runs[i], &runs[3]))
        .collect::<Result<Vec<_>>>()?;
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":"Native convergence after a common physical landing","pair_totals_only":true,"lanes":lanes}),
    )?;
    fs::create_dir_all(dir.join("cases"))?;
    write_json(
        &dir.join("cases").join(format!("case-{k:03}.json")),
        &json!({"format":"matched-fine-suffix-v1","case":k,"source":source(k,true),"fixture":f,"cut":snapshot(&c),"cut_tick":c.tick,"end_tick":DURATION,"mechanics":mechanics,"paths":reports,"against_finest":differences}),
    )
}
