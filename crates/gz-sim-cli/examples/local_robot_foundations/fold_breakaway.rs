// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently repeat each load from a matched grip and bracket persistent finger motion.
use super::*;
use std::collections::BTreeMap;
use std::collections::btree_map::Entry;

// Same displacement criterion as the independent rigid-arm slip measurement.
const SLIP_M: f64 = 0.00025;
const RESOLUTION_MN: u32 = 25;
const LADDER_MN: [u32; 7] = [300, 900, 1050, 1150, 1250, 1500, 2000];

#[derive(Serialize)]
struct Case {
    archive: String,
    load_mn: u32,
    final_min_pad_m: [f64; 2],
    residual_pad_m: [f64; 2],
    lasting_slip: bool,
    recovered_support_ticks: usize,
    recovered_bench_ticks: usize,
    min_bench_clearance_m: f64,
    recovery_layer_force_n: f64,
    mean_pull_squeeze_n: [f64; 2],
    mean_pull_side_reaction_n: [f64; 2],
}

fn compare(archive: String, load_mn: u32, case: &Outcome, sham: &Outcome) -> Result<Case> {
    let i = sham.baseline;
    if case.baseline != i || case.anchor != sham.anchor || case.trace[..=i] != sham.trace[..=i] {
        return Err("load trial did not reproduce the unloaded grasp".into());
    }
    let end = &case.trace[i + END + 1 - WINDOW..];
    let control = &sham.trace[i + END + 1 - WINDOW..];
    let mut min = [f64::INFINITY; 2];
    let mut residual = [0.; 2];
    for (a, b) in end.iter().zip(control) {
        for j in 0..2 {
            let shift = a.pad_frame_y_m[j] - b.pad_frame_y_m[j];
            min[j] = min[j].min(shift);
            residual[j] += shift / WINDOW as f64;
        }
    }
    let plateau = &case.trace[i + RAMP + HOLD + 1 - WINDOW..=i + RAMP + HOLD];
    let mean = |f: fn(&Tick) -> [f64; 2]| {
        std::array::from_fn(|j| plateau.iter().map(|s| f(s)[j]).sum::<f64>() / WINDOW as f64)
    };
    Ok(Case {
        archive,
        load_mn,
        final_min_pad_m: min,
        residual_pad_m: residual,
        lasting_slip: min.iter().any(|x| *x > SLIP_M),
        recovered_support_ticks: end.iter().filter(|s| supported(s, case.weight)).count(),
        recovered_bench_ticks: end.iter().filter(|s| s.bench_up_n.abs() >= 1e-6).count(),
        min_bench_clearance_m: case.trace[i + 1..]
            .iter()
            .map(|s| s.min_bench_clearance_m)
            .fold(f64::INFINITY, f64::min),
        recovery_layer_force_n: end.iter().map(|s| s.cross_panel_n).sum::<f64>() / WINDOW as f64,
        mean_pull_squeeze_n: mean(|s| s.jaw_squeeze_n),
        mean_pull_side_reaction_n: mean(|s| s.jaw_side_reaction_n),
    })
}

// Search only tested loads. Always test the previous clock's actual endpoints before
// filling an open side of its bracket. Preserve a censored result if no onset is reached.
fn search(
    previous: Option<[u32; 2]>,
    mut slipped: impl FnMut(u32) -> Result<bool>,
) -> Result<Option<[u32; 2]>> {
    let mut seen = BTreeMap::from([(0, false)]);
    if let Some(ends) = previous {
        for f in ends {
            if f != 0 {
                seen.insert(f, slipped(f)?);
            }
        }
    }
    let mut high = seen.iter().find_map(|(&f, &yes)| yes.then_some(f));
    if high.is_none() {
        for f in LADDER_MN {
            if let Entry::Vacant(e) = seen.entry(f) {
                let yes = slipped(f)?;
                e.insert(yes);
                if yes {
                    high = Some(f);
                    break;
                }
            }
        }
    }
    let Some(mut hi) = high else { return Ok(None) };
    let mut low = *seen.range(..hi).next_back().unwrap().0;
    while hi - low > RESOLUTION_MN {
        let mid = low + (hi - low) / 2;
        if slipped(mid)? { hi = mid } else { low = mid }
    }
    Ok(Some([low, hi]))
}

type Trial = fn(usize, f64, u32, u32, &Path) -> Result<Outcome>;
fn sweep(columns: usize, out: &Path, trial: Trial) -> Result<Value> {
    let mut baseline = None;
    let mut reports = vec![];
    let mut previous = None;
    let mut baseline_ns = 0;
    for (clock, steps, iters) in [("primary", 32, 24), ("fine", 64, 48)] {
        let name = |f| format!("mesh-{columns}-{clock}-{f}");
        let sham = trial(columns, 0., steps, iters, &out.join(name(0)))?;
        if let Some((state, prefix)) = &baseline {
            if state != &sham.anchor || prefix != &sham.trace[..=sham.baseline] {
                return Err("material clocks did not start with the same incoming state".into());
            }
        } else {
            baseline_ns = sham.trace[sham.baseline].time_ns;
            baseline = Some((sham.anchor.clone(), sham.trace[..=sham.baseline].to_vec()));
        }
        let mut cases = vec![compare(name(0), 0, &sham, &sham)?];
        let bracket = search(previous, |f| {
            let archive = name(f);
            let result = trial(
                columns,
                f64::from(f) * 0.001,
                steps,
                iters,
                &out.join(&archive),
            )?;
            let measured = compare(archive, f, &result, &sham)?;
            let yes = measured.lasting_slip;
            eprintln!(
                "{columns} {clock} {f} mN: slip={yes} min {:?} m; supported {}/20",
                measured.final_min_pad_m, measured.recovered_support_ticks
            );
            cases.push(measured);
            Ok(yes)
        })?;
        cases.sort_by_key(|r| r.load_mn);
        reports
            .push(json!({"substeps":steps,"iterations":iters,"cases":cases,"bracket_mn":bracket}));
        previous = bracket;
    }
    Ok(
        json!({"columns":columns,"baseline_time_ns":baseline_ns,"primary":reports[0],"fine":reports[1]}),
    )
}

pub(crate) fn run(out: &Path) -> Result<()> {
    run_with(out, trial_at)
}

fn run_with(out: &Path, trial: Trial) -> Result<()> {
    let [a, b] = std::thread::scope(|scope| {
        let jobs =
            [5, 7].map(|c| scope.spawn(move || sweep(c, out, trial).map_err(|e| e.to_string())));
        jobs.map(|h| {
            h.join()
                .map_err(|_| "breakaway trial panicked".to_string())
                .and_then(|r| r)
        })
    });
    write_json(
        &out.join("comparison.json"),
        &json!({"trials":[a?,b?],"slip_m":SLIP_M,"load_resolution_mn":RESOLUTION_MN,"dt_ns":1_000_000,"window_ticks":WINDOW,"ramp_ticks":RAMP,"hold_ticks":HOLD,"recovery_ticks":RECOVER}),
    )
}

#[cfg(test)]
#[path = "fold_breakaway_tests.rs"]
mod tests;
