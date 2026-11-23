// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare native solver effort, contact refresh cadence and causal touchdown triggering.
use super::*;
#[path = "control_figures.rs"]
mod figures;
#[path = "contact_gate.rs"]
mod gate;
#[path = "rebound.rs"]
pub(crate) mod rebound;
#[path = "rebound_gate.rs"]
mod rebound_gate;
#[path = "contact_gate_timing.rs"]
mod runtime;
use gate::Gate;

#[derive(Copy, Clone)]
struct Policy {
    label: &'static str,
    substeps: u32,
    start_offset: i64,
    ticks: u64,
    iterations: Option<usize>,
    cold: bool,
    online: bool,
}

impl Policy {
    const fn fixed(label: &'static str, substeps: u32, start_offset: i64, ticks: u64) -> Self {
        Self {
            label,
            substeps,
            start_offset,
            ticks,
            iterations: None,
            cold: false,
            online: false,
        }
    }
    fn active(self, tick: u64, first: u64) -> bool {
        let start = first.checked_add_signed(self.start_offset).unwrap();
        (start..start + self.ticks).contains(&tick)
    }
}
const COARSE: Policy = Policy::fixed("coarse", 1, 0, 0);
const TEN: Policy = Policy::fixed("fixed_10", 4, 0, 10);
const GATE: Policy = Policy {
    label: "gate_10",
    online: true,
    ..TEN
};
const GATE_ITER: Policy = Policy {
    label: "gate_2_iter16",
    substeps: 1,
    ticks: 2,
    iterations: Some(16),
    ..GATE
};
const POLICIES: [Policy; 17] = [
    COARSE,
    GATE_ITER,
    Policy::fixed("fixed_2x", 2, 0, 10),
    TEN,
    Policy::fixed("fixed_8x", 8, 0, 10),
    Policy::fixed("fixed_16x", 16, 0, 10),
    Policy {
        label: "coarse_16iter",
        iterations: Some(16),
        ..Policy::fixed("", 1, 0, 10)
    },
    Policy {
        label: "fine_1iter",
        iterations: Some(1),
        ..TEN
    },
    Policy {
        label: "coarse_no_seed",
        cold: true,
        ..Policy::fixed("", 1, 0, 10)
    },
    Policy {
        label: "fine_no_seed",
        cold: true,
        ..TEN
    },
    Policy::fixed("first_only", 4, 0, 1),
    Policy::fixed("first_two", 4, 0, 2),
    Policy::fixed("late_9", 4, 1, 9),
    Policy::fixed("early_10", 4, -1, 10),
    Policy::fixed("long_20", 4, 0, 20),
    GATE,
    Policy {
        label: "gate_2",
        ticks: 2,
        ..GATE
    },
];

struct Flight {
    case: matched::Case,
    cycle: usize,
    history: usize,
    cut: u64,
    stop: u64,
    baseline_onset: u64,
}

fn flight(cycle: usize, history: usize, shift: i64) -> Result<Flight> {
    let depart = sweep::DEPART_US[cycle] / STEP_US;
    let stroke = &STROKES[cycle];
    let rise = (depart + stroke.down + 24)
        .checked_add_signed(shift / STEP_US as i64)
        .ok_or("invalid rise")?;
    let cut = rise + 1;
    let stop = (depart + stroke.start_park())
        .checked_add_signed(shift / STEP_US as i64)
        .ok_or("invalid stop")?;
    let mut case = matched::Case::new(if history == 0 { 0 } else { 7 }, cut as usize + 1)?;
    let mut last = 0.;
    for t in 0..=cut {
        // Perturb only this descent's bottom dwell. All preceding history is nominal.
        let us = t.saturating_sub(1) * STEP_US;
        let v = sweep::command(us, if us >= depart * STEP_US { shift } else { 0 }).1;
        case.advance(t as usize, (last != v).then_some(v))?;
        last = v;
    }
    let lane = &case.lane;
    let mut witness =
        case.backend()
            .landing_fork(None, lane.foot, lane.pad, Duration::from_micros(STEP_US))?;
    let mut onset = None;
    for tick in cut + 1..=stop {
        let v = sweep::command((tick - 1) * STEP_US, shift).1;
        witness.step_outer(1, (last != v).then_some(v), |_| {})?;
        if witness.normal_impulse() > 0. {
            onset = Some(tick);
            break;
        }
        last = v;
    }
    Ok(Flight {
        case,
        cycle,
        history,
        cut,
        stop,
        baseline_onset: onset.ok_or("coarse run missed lift")?,
    })
}

fn run(flight: &Flight, policy: Policy, shift: i64, replay_path: Option<&Path>) -> Result<Value> {
    let mut recorder = None;
    let lane = &flight.case.lane;
    let mut fork = flight.case.backend().landing_fork(
        None,
        lane.foot,
        lane.pad,
        Duration::from_micros(STEP_US),
    )?;
    let baseline = fork.contact_settings();
    let mut gate = Gate::default();
    let mut last = sweep::command((flight.cut - 1) * STEP_US, shift).1;
    let onset = flight.baseline_onset;
    let mut first_loaded = None;
    let mut window = 0.;
    let mut common_first = 0.;
    let mut total = 0.;
    let depart = (sweep::DEPART_US[flight.cycle] / STEP_US) as usize;
    total += flight.case.samples[depart + 1..]
        .iter()
        .map(|s| s.normal_impulse_ns)
        .sum::<f64>();
    let mut refined = Vec::new();
    let mut selected = Vec::new();
    let mut sum_budget = 0;
    let mut max_gap = 0.0_f64;
    let mut first_after_window = 0.;
    let mut settings = baseline;
    for tick in flight.cut + 1..=flight.stop {
        if replay_path.is_some() && tick == CUT_TICK + 1 {
            recorder = Some(energy::Recorder::new(&fork));
        }
        let speed = sweep::command((tick - 1) * STEP_US, shift).1;
        let changed = (last != speed).then_some(speed);
        last = speed;
        let active = if policy.online {
            let [foot, _, mut pad] = fork.states();
            pad.velocity[2] = speed;
            gate.advance(foot, pad, policy.ticks, STEP_US as f64 * 1e-6)
        } else {
            policy.active(tick, onset)
        };
        let n = if active { policy.substeps } else { 1 };
        let new_settings = if active {
            (
                policy.iterations.unwrap_or(baseline.0),
                if policy.cold { 0. } else { baseline.1 },
            )
        } else {
            baseline
        };
        if new_settings != settings {
            fork.set_contact_settings(new_settings.0, new_settings.1)?;
            settings = new_settings;
        }
        if active {
            selected.push(tick);
        }
        if n > 1 {
            refined.push(tick);
        }
        let mut j = 0.;
        let mut k = 0u64;
        fork.step_outer(n, changed, |f| {
            if let Some(r) = &mut recorder {
                r.observe(f, lane, STEP_US / u64::from(n));
            }
            k += 1;
            let impulse = f.normal_impulse();
            if impulse > 0. {
                first_loaded.get_or_insert((
                    tick,
                    (tick - 1) * STEP_US * 1000 + k * STEP_US * 1000 / u64::from(n),
                    impulse,
                ));
            }
            j += impulse;
        })?;
        if tick == onset {
            common_first = j;
        }
        if tick >= onset && tick < onset + 10 {
            window += j;
        }
        if tick == onset + 10 {
            first_after_window = j;
        }
        total += j;
        if tick >= onset && tick < onset + 10 {
            sum_budget += n as usize * new_settings.0;
        }
        if tick > onset {
            let [foot, _, pad] = fork.states().map(Body::from);
            max_gap = max_gap.max(sole_gap(foot, pad));
        }
    }
    if let Some(path) = replay_path {
        recorder
            .ok_or("missing replay cut")?
            .write(path, window, total)?;
    }
    let (loaded, ns, j) = first_loaded.ok_or("selected treatment missed landing")?;
    Ok(
        json!({"history":LABELS[flight.history],"cycle":flight.cycle+1,"bottom_dwell_shift_us":shift,"policy":policy.label,
        "cut_tick":flight.cut,"stop_tick":flight.stop,"coarse_onset_tick":onset,"first_loaded_tick":loaded,"first_native_end_ns":ns,
        "first_native_ns":j,"first_outer_ns":common_first,"window_ns":window,"stroke_ns":total,"first_post_window_ns":first_after_window,
        "max_post_onset_outer_gap_m":max_gap,"default_solver_iterations":baseline.0,"native_iteration_budget_in_window":sum_budget,
        "selected_ticks":selected,"refined_ticks":refined,"trigger_ticks":gate.triggers.iter().map(|t|flight.cut+t).collect::<Vec<_>>()}),
    )
}

fn study(out: &Path) -> Result<Value> {
    let mut rows = Vec::new();
    for cycle in 0..3 {
        for history in 0..2 {
            for shift in if cycle == 2 {
                vec![0, -500, 500]
            } else {
                vec![0]
            } {
                let flight = flight(cycle, history, shift)?;
                for policy in POLICIES {
                    if cycle == 2 && shift == 0
                        || matches!(
                            policy.label,
                            "coarse" | "fixed_10" | "gate_10" | "gate_2" | "gate_2_iter16"
                        )
                    {
                        let replay = (cycle == 2
                            && shift == 0
                            && history == 0
                            && policy.label == GATE_ITER.label)
                            .then(|| out.join("gate_2_iter16.json"));
                        rows.push(run(&flight, policy, shift, replay.as_deref())?);
                    }
                }
            }
        }
    }
    Ok(json!({"outer_us":STEP_US,"runs":rows}))
}

pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out)?;
    let cases = study(out)?;
    fs::write(out.join("summary.json"), serde_json::to_vec_pretty(&cases)?)?;
    let timing = runtime::write(out)?;
    figures::draw(out, &cases, &timing)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn matched_iteration_budget_separates_contact_refresh_from_internal_solver_steps() -> Result<()>
    {
        let f = flight(2, 0, 0)?;
        let get = |name| {
            run(
                &f,
                *POLICIES.iter().find(|p| p.label == name).unwrap(),
                0,
                None,
            )
        };
        let a = get("fixed_10")?;
        let b = get("coarse_16iter")?;
        let c = get("fine_1iter")?;
        let base = get("coarse")?;
        assert_eq!(
            a["native_iteration_budget_in_window"],
            b["native_iteration_budget_in_window"]
        );
        assert_eq!(
            c["native_iteration_budget_in_window"],
            base["native_iteration_budget_in_window"]
        );
        assert!((base["window_ns"].as_f64().unwrap() - 3.15231315558155).abs() < 1e-11);
        assert!((a["window_ns"].as_f64().unwrap() - 3.5045077).abs() < 1e-6);
        Ok(())
    }
    #[test]
    fn two_tick_gate_tracks_the_matched_window_across_strokes_and_dwell_shifts() -> Result<()> {
        for (cycle, shift) in [(0, 0), (1, 0), (2, 0), (2, -500), (2, 500)] {
            for history in 0..2 {
                let f = flight(cycle, history, shift)?;
                let fixed = run(&f, TEN, shift, None)?;
                let gated = run(&f, GATE_ITER, shift, None)?;
                let onset = f.baseline_onset;
                assert_eq!(gated["trigger_ticks"], json!([onset]));
                assert_eq!(gated["selected_ticks"], json!([onset, onset + 1]));
                assert_eq!(gated["refined_ticks"], json!([]));
                for (key, bound) in [
                    ("first_outer_ns", 0.0012),
                    ("window_ns", 0.0006),
                    ("stroke_ns", 0.0004),
                ] {
                    let difference = gated[key].as_f64().unwrap() - fixed[key].as_f64().unwrap();
                    assert!(
                        difference.abs() < bound,
                        "cycle {cycle}, shift {shift}, history {history}: {key} {difference}"
                    );
                }
            }
        }
        Ok(())
    }
}
