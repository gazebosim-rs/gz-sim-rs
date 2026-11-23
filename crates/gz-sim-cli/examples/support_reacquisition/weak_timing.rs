// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Full Bevy step time across three lift strokes, including parked intervals.
use super::*;

const ORDERS: [[usize; 3]; 6] = [
    [0, 1, 2],
    [1, 2, 0],
    [2, 0, 1],
    [2, 1, 0],
    [1, 0, 2],
    [0, 2, 1],
];
const POLICIES: [(&str, Mode); 3] = [
    ("current", Mode::Budget(BUDGET)),
    ("weak", Mode::Weak(BUDGET)),
    ("fine", Mode::Uniform(Effort::Split(4))),
];
const LOADED: u8 = 1;
const WEAK_START: u8 = 2;
const SELECTED: u8 = 4;

#[derive(Default, Serialize)]
struct Samples {
    // One mask per completed outer tick; verified identical across repetitions.
    flags: Vec<u8>,
    elapsed_ns: Vec<Vec<u64>>,
}

fn run(fixture: Fixture, mode: Mode) -> Result<(Vec<u8>, Vec<u64>)> {
    let mut rig = start(fixture, mode)?;
    let ticks = (sweep::END_US / STEP_US) as usize;
    let mut flags = Vec::with_capacity(ticks);
    let mut elapsed = Vec::with_capacity(ticks);
    for _ in 0..ticks {
        elapsed.push(rig.advance_at(command(fixture, rig.tick))?);
        // Read the already committed receipt outside Rig's step_exact timer.
        let step = rig.case.lane.sim.world().resource::<Step>();
        let bits = (u8::from(step.normal > 0.) * LOADED)
            | (u8::from(step.weak_start) * WEAK_START)
            | (u8::from(step.selected) * SELECTED);
        flags.push(bits);
    }
    Ok((flags, elapsed))
}

#[derive(Default)]
struct Groups {
    all: Vec<u64>,
    loaded: Vec<u64>,
    weak_clock: Vec<u64>,
}
impl Groups {
    fn add(&mut self, samples: &Samples, weak_flags: &[u8]) {
        for repetition in &samples.elapsed_ns {
            self.all.extend(repetition);
            for ((&ns, &flags), &candidate) in repetition.iter().zip(&samples.flags).zip(weak_flags)
            {
                if flags & LOADED != 0 {
                    self.loaded.push(ns);
                }
                if candidate & WEAK_START != 0 {
                    self.weak_clock.push(ns);
                }
            }
        }
    }

    fn summary(mut self) -> Value {
        let quantile = |v: &mut Vec<u64>| {
            v.sort_unstable();
            let p95 = (!v.is_empty()).then(|| v[(95 * v.len()).div_ceil(100) - 1]);
            json!({"n":v.len(),"p95_ns":p95})
        };
        json!({"all":quantile(&mut self.all),"positive_reaction":quantile(&mut self.loaded),
            "weak_trigger_clock":quantile(&mut self.weak_clock)})
    }
}

pub(super) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out)?;
    let mut totals: [Groups; 3] = Default::default();
    let mut fixtures = Vec::new();
    for (i, fixture) in CASES.into_iter().enumerate() {
        let mut samples: [Samples; 3] = Default::default();
        for (s, (_, mode)) in samples.iter_mut().zip(POLICIES) {
            s.flags = run(fixture, mode)?.0;
        }
        let mut orders = Vec::new();
        for order in ORDERS
            .iter()
            .cycle()
            .skip(i % ORDERS.len())
            .take(ORDERS.len())
        {
            orders.push(order.map(|p| POLICIES[p].0));
            for &p in order {
                let (flags, elapsed) = run(fixture, POLICIES[p].1)?;
                if flags != samples[p].flags {
                    return Err(format!(
                        "nonreproducible timed physics in {} {}",
                        fixture.name, POLICIES[p].0
                    )
                    .into());
                }
                samples[p].elapsed_ns.push(elapsed);
            }
        }

        let mut rows = Vec::new();
        for (p, sample) in samples.iter().enumerate() {
            let weak_flags = &samples[1].flags;
            let mut groups = Groups::default();
            groups.add(sample, weak_flags);
            totals[p].add(sample, weak_flags);
            rows.push(json!({"policy":POLICIES[p].0,"summary":groups.summary(),"raw":sample}));
        }
        fixtures.push(json!({"fixture":fixture.name,"measured_order":orders,"rows":rows}));
    }
    let pooled: Vec<_> = POLICIES
        .iter()
        .zip(totals)
        .map(|((label, _), group)| json!({"policy":label,"summary":group.summary()}))
        .collect();
    write_json(
        &out.join("timing.json"),
        &json!({
            "outer_us":STEP_US,"ticks_per_run":sweep::END_US/STEP_US,"warmup_runs_per_policy":1,
            "repetitions_per_policy":ORDERS.len(),"timed_scope":"Bevy step_exact(1), including pre-solve policy and native reactions",
            "raw_clock":"index zero is completed outer tick 1","flag_bits":{"positive_reaction":LOADED,"weak_start":WEAK_START,"increased_effort":SELECTED},
            "weak_trigger_clock":"same physical outer ticks as the weak policy's own live admissions for this fixture",
            "quantile":"nearest rank from the pooled raw samples","pooled":pooled,"fixtures":fixtures
        }),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sparse_weak_cost_uses_candidate_ticks_even_for_a_control_without_admissions() {
        let samples = Samples {
            flags: vec![LOADED, LOADED, 0, 0],
            elapsed_ns: vec![vec![1, 2, 3, 4], vec![10, 20, 30, 40]],
        };
        let mut groups = Groups::default();
        groups.add(&samples, &[0, 0, WEAK_START, 0]);
        let report = groups.summary();
        assert_eq!(report["all"], json!({"n":8,"p95_ns":40}));
        assert_eq!(report["positive_reaction"], json!({"n":4,"p95_ns":20}));
        assert_eq!(report["weak_trigger_clock"], json!({"n":2,"p95_ns":30}));

        let mut empty = Groups::default();
        empty.add(&samples, &[0; 4]);
        assert_eq!(
            empty.summary()["weak_trigger_clock"],
            json!({"n":0,"p95_ns":null})
        );
    }
}
