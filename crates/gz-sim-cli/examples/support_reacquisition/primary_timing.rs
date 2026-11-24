// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Counterbalanced pinned-process timing of full Bevy steps and causally selected primary steps.
use super::*;

#[derive(Default, Serialize)]
struct Timings {
    trigger_ticks: Vec<usize>,
    elapsed_ns: Vec<Vec<u64>>,
}
fn timed(f: Fixture, mode: Mode) -> Result<(Vec<usize>, Vec<u64>)> {
    let mut r = start(f, mode)?;
    let mut times = Vec::with_capacity(DURATION as usize);
    let mut triggers = vec![];
    while r.tick < DURATION {
        times.push(r.advance_at(command(f, r.tick))?);
        if r.case
            .lane
            .sim
            .world()
            .resource::<Step>()
            .candidate
            .primary_started()
        {
            triggers.push(r.tick as usize);
        }
    }
    Ok((triggers, times))
}
fn stats(mut v: Vec<u64>) -> Value {
    v.sort_unstable();
    let q = |n: usize| (!v.is_empty()).then(|| v[(v.len() * n).div_ceil(100) - 1]);
    json!({"n":v.len(),"p50_ns":q(50),"p95_ns":q(95)})
}
pub(crate) fn timing(out: &Path) -> Result<()> {
    fs::create_dir_all(out)?;
    let mut records = vec![];
    let n = LIVE.len();
    // Each policy occupies every position in both forward and reverse cyclic order.
    for (index, (f, held)) in fixtures().enumerate() {
        let mut measurements: Vec<Timings> = LIVE.iter().map(|_| Timings::default()).collect();
        for (dst, &(_, mode)) in measurements.iter_mut().zip(&LIVE) {
            dst.trigger_ticks = timed(f, mode)?.0;
        }
        let mut orders = vec![];
        for rep in 0..2 * n {
            let order = (0..n)
                .map(|j| (index + rep % n + if rep < n { j } else { n - j }) % n)
                .collect::<Vec<_>>();
            for &p in &order {
                let (ticks, times) = timed(f, LIVE[p].1)?;
                if ticks != measurements[p].trigger_ticks {
                    return Err("timed primary admission varied within a process".into());
                }
                measurements[p].elapsed_ns.push(times);
            }
            orders.push(order);
        }
        let summaries=measurements.iter().map(|m|{
            let selected=m.elapsed_ns.iter().flat_map(|times|m.trigger_ticks.iter().map(move|i|times[i-1])).collect::<Vec<_>>();
            json!({"all":stats(m.elapsed_ns.iter().flatten().copied().collect()),"own_primary":stats(selected)})
        }).collect::<Vec<_>>();
        records.push(json!({"fixture":f,"held_out":held,"orders":orders,"summary":summaries,"timings":measurements}));
    }
    write_json(
        &out.join("timing.json"),
        &json!({"outer_us":STEP_US,"timed":"Bevy step_exact(1)","quantile":"nearest rank",
        "policies":LIVE.map(|(n,_)|n),"repetitions":2*n,"fixtures":records}),
    )
}
