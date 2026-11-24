// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Calibrate the effort after a secondary touchdown, then apply the frozen setting to other histories.
use super::*;

const RELATIVE_LIMIT: f64 = 0.25;

#[derive(Clone, Copy)]
struct Metrics {
    first: f64,
    window: f64,
    com: f64,
    iterations: usize,
    native_steps: u32,
}
impl Metrics {
    fn goal_ratio(self, base: Self) -> f64 {
        (self.window / base.window).max(self.com / base.com)
    }
    fn qualifies(self, base: Self) -> bool {
        self.first <= base.first + 1e-12 && self.goal_ratio(base) <= RELATIVE_LIMIT
    }
}

struct Measured {
    row: Value,
    metrics: Metrics,
}
fn effort_label(e: Effort) -> String {
    match e {
        Effort::Standard => "ordinary".into(),
        Effort::Iterations(n) => format!("iter{n}"),
        Effort::Split(n) => format!("split{n}"),
        Effort::Grid {
            calls,
            iterations,
            phase,
        } => format!("grid{calls}x{iterations}-{phase:?}"),
    }
}
fn budget_label(b: Budget) -> String {
    if b.follow_ticks == 0 {
        format!("{}_onset_only", effort_label(b.onset))
    } else {
        format!(
            "{}_{}_{}",
            effort_label(b.onset),
            effort_label(b.follow),
            b.follow_ticks
        )
    }
}
fn loaded_ticks(frames: &[ReboundFrame], onset: u64) -> Vec<u64> {
    frames
        .iter()
        .filter(|f| f.tick >= onset && f.normal_impulse_ns > 0.)
        .map(|f| f.tick)
        .collect()
}
fn measure(
    t: Target,
    frames: &[ReboundFrame],
    fine: &[ReboundFrame],
    label: &str,
) -> Result<Measured> {
    let (position_error, speed_error) = actuator_parity(t, frames, fine)?;
    let mut row = score(t, frames, fine)?;
    let mut window = Vec::new();
    let mut iterations = 0;
    let mut steps = 0;
    let mut first_impulse_difference = None;
    let mut first_com_difference = None;
    let mut max_window_distance = 0.0_f64;
    for (f, g) in frames.iter().zip(fine) {
        let impulse_delta = f.normal_impulse_ns - g.normal_impulse_ns;
        let com_delta = (com(f) - com(g)) * 1000.;
        if impulse_delta.abs() > 1e-12 {
            first_impulse_difference.get_or_insert(f.tick);
        }
        if com_delta.length() > 1e-9 {
            first_com_difference.get_or_insert(f.tick);
        }
        if !(t.onset..t.onset + WINDOW).contains(&f.tick) {
            continue;
        }
        max_window_distance = max_window_distance.max(com_delta.length());
        iterations += f.native_steps as usize * f.solver_iterations;
        steps += f.native_steps;
        window.push(json!({"tick":f.tick,"normal_ns":f.normal_impulse_ns,"normal_delta_ns":impulse_delta,
            "corner_ns":f.corner_ns,"corner_delta_ns":std::array::from_fn::<_,4,_>(|i|f.corner_ns[i]-g.corner_ns[i]),
            "com_delta_mm":com_delta.to_array(),"selected":f.selected,"secondary_start":f.secondary_start,
            "native_steps":f.native_steps,"iterations_per_step":f.solver_iterations}));
    }
    let number = |k: &str| row[k].as_f64().expect("numeric continuation measure").abs();
    let metrics = Metrics {
        first: number("common_first_delta_ns"),
        window: number("window_delta_ns"),
        com: number("window_end_com_delta_mm"),
        iterations,
        native_steps: steps,
    };
    row["policy"] = label.into();
    row["window"] = json!(window);
    row["second_loaded_tick"] = json!(loaded_ticks(frames, t.onset).get(1));
    row["first_impulse_difference_tick"] = json!(first_impulse_difference);
    row["first_com_difference_tick"] = json!(first_com_difference);
    row["max_window_com_delta_mm"] = max_window_distance.into();
    row["max_pad_position_difference_m"] = position_error.into();
    row["max_pad_command_error_m_s"] = speed_error.into();
    row["window_iterations"] = iterations.into();
    row["window_native_steps"] = steps.into();
    Ok(Measured { row, metrics })
}

fn grid(second_offset: Option<u64>) -> Vec<Budget> {
    let mut out = Vec::new();
    let mut lengths = vec![1, 2, 3];
    if let Some(n @ 4..=9) = second_offset {
        lengths.push(n as u8);
    }
    for onset in [Effort::Iterations(16), Effort::Split(4)] {
        out.push(Budget {
            onset,
            follow: Effort::Standard,
            follow_ticks: 0,
        });
        for follow in [
            Effort::Standard,
            Effort::Iterations(8),
            Effort::Iterations(16),
            Effort::Iterations(32),
            Effort::Split(2),
            Effort::Split(4),
        ] {
            for &follow_ticks in &lengths {
                out.push(Budget {
                    onset,
                    follow,
                    follow_ticks,
                });
            }
        }
    }
    out
}
struct Survey {
    t: Target,
    baseline: Vec<ReboundFrame>,
    fine: Vec<ReboundFrame>,
    base: Measured,
    trials: Vec<(Budget, Measured)>,
}
impl Survey {
    fn new(t: Target) -> Result<Self> {
        let baseline = continuation(t, Mode::Candidate)?;
        let fine = continuation(t, Mode::Fixed { onset: t.onset })?;
        let base = measure(t, &baseline, &fine, "baseline")?;
        let offset = loaded_ticks(&baseline, t.onset)
            .get(1)
            .map(|&tick| tick - t.onset);
        let mut trials = Vec::new();
        for b in grid(offset) {
            let f = continuation(t, Mode::Budget(b))?;
            trials.push((b, measure(t, &f, &fine, &budget_label(b))?));
        }
        Ok(Self {
            t,
            baseline,
            fine,
            base,
            trials,
        })
    }

    fn choose(&self) -> Result<Budget> {
        let base = self.base.metrics;
        if base.com <= 0. || base.window <= 0. {
            return Err("calibration requires nonzero baseline errors".into());
        }
        let winner = self
            .trials
            .iter()
            .min_by(|(_, a), (_, b)| {
                let (a, b) = (a.metrics, b.metrics);
                match (a.qualifies(base), b.qualifies(base)) {
                    (true, false) => std::cmp::Ordering::Less,
                    (false, true) => std::cmp::Ordering::Greater,
                    (true, true) => a
                        .iterations
                        .cmp(&b.iterations)
                        .then(a.native_steps.cmp(&b.native_steps))
                        .then(a.com.total_cmp(&b.com)),
                    (false, false) => a
                        .goal_ratio(base)
                        .max(a.first / base.first)
                        .total_cmp(&b.goal_ratio(base).max(b.first / base.first))
                        .then(a.iterations.cmp(&b.iterations)),
                }
            })
            .ok_or("empty effort grid")?;
        Ok(winner.0)
    }

    fn record(self, selected: Budget, out: &Path, training: bool) -> Result<Value> {
        let t = self.t;
        let candidate = continuation(t, Mode::Budget(selected))?;
        let chosen = measure(t, &candidate, &self.fine, &budget_label(selected))?;
        let goal = chosen.metrics.qualifies(self.base.metrics);
        let mut second_ticks = [&self.baseline, &self.fine]
            .into_iter()
            .filter_map(|f| loaded_ticks(f, t.onset).get(1).copied())
            .collect::<Vec<_>>();
        second_ticks.sort_unstable();
        second_ticks.dedup();
        let mut ablation = Vec::new();
        for tick in second_ticks {
            for effort in [
                Effort::Standard,
                Effort::Iterations(8),
                Effort::Iterations(16),
                Effort::Iterations(32),
                Effort::Split(2),
                Effort::Split(4),
            ] {
                let f = continuation(t, Mode::OverrideTick { tick, effort })?;
                let mut m = measure(t, &f, &self.fine, &effort_label(effort))?.row;
                m["override_tick"] = tick.into();
                ablation.push(m);
            }
        }
        let fixed = measure(t, &self.fine, &self.fine, "fine")?;
        if training {
            fs::create_dir_all(out.join("traces"))?;
            for (label, frames) in [
                ("baseline", &self.baseline),
                ("candidate", &candidate),
                ("fine", &self.fine),
            ] {
                let mut trace = vec![t.cut.with_effort()];
                trace.extend(frames.iter().map(|f| f.with_effort()));
                write_json(
                    &out.join("traces").join(format!("{label}.json")),
                    &json!({
                    "geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},
                    "policy":label,"onset_tick":t.onset,"frames":trace}),
                )?;
            }
        }
        let trials = self
            .trials
            .into_iter()
            .map(|(_, m)| m.row)
            .collect::<Vec<_>>();
        Ok(
            json!({"deck":t.deck.label(),"history":history(t.switched),"stroke":t.stroke+1,"training":training,
            "cut_tick":t.cut.tick,"onset_tick":t.onset,"cut_foot":t.cut.foot,"cut_upper":t.cut.upper,"cut_pad":t.cut.pad,
            "baseline":self.base.row,"fine":fixed.row,"selected":chosen.row,
            "meets_relative_goal":goal,"sweep":trials,"single_tick_ablation":ablation}),
        )
    }
}

fn heldout_targets() -> Result<[Target; 3]> {
    let required = |deck, switched, stroke| -> Result<Target> {
        target(deck, switched, stroke)?.ok_or_else(|| "matched secondary contact is absent".into())
    };
    Ok([
        required(Deck::Flat, true, 2)?,
        required(Deck::Edge, false, 2)?,
        required(Deck::Edge, true, 1)?,
    ])
}

pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out)?;
    let calibration =
        target(Deck::Flat, false, 2)?.ok_or("missing flat dynamic training rebound")?;
    let train = Survey::new(calibration)?;
    let selected = train.choose()?;
    let mut records = vec![train.record(selected, out, true)?];
    let heldouts = heldout_targets()?;
    for heldout in heldouts {
        records.push(Survey::new(heldout)?.record(selected, out, false)?);
    }
    write_json(
        &out.join("study.json"),
        &json!({"outer_us":STEP_US,"selected":budget_label(selected),
        "qualification_error_fraction":RELATIVE_LIMIT,"cases":records}),
    )?;
    super::grid_validation::write(out, selected)?;
    super::super::budget_continuous::write(out, selected)?;
    let mut bench = Vec::new();
    for t in std::iter::once(calibration).chain(heldouts) {
        bench.push(benchmark_settings(
            t,
            [
                ("baseline", Mode::Candidate),
                ("candidate", Mode::Budget(selected)),
                ("fine", Mode::Fixed { onset: t.onset }),
            ],
        )?);
    }
    write_json(
        &out.join("bench.json"),
        &json!({"outer_us":STEP_US,"blocks":BLOCKS,
        "repetitions_per_block":REPETITIONS,"selected":budget_label(selected),"cases":bench}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn calibrate_on_flat_dynamic_rebound_before_any_heldout_runs() -> Result<()> {
        let train =
            Survey::new(target(Deck::Flat, false, 2)?.ok_or("missing calibration impact")?)?;
        let b = train.choose()?;
        let label = budget_label(b);
        for (trial, m) in &train.trials {
            println!(
                "{} first={:.8} window={:.8} com={:.8} budget={} goal={}",
                budget_label(*trial),
                m.metrics.first,
                m.metrics.window,
                m.metrics.com,
                m.metrics.iterations,
                m.metrics.qualifies(train.base.metrics)
            );
        }
        println!(
            "selected {label}. Baseline and fine loaded ticks: {:?} / {:?}",
            &loaded_ticks(&train.baseline, train.t.onset)[..3],
            &loaded_ticks(&train.fine, train.t.onset)[..3]
        );
        assert_eq!(
            loaded_ticks(&train.baseline, train.t.onset)[1],
            train.t.onset + 1
        );
        let winner = train
            .trials
            .iter()
            .find(|(budget, _)| budget_label(*budget) == label)
            .unwrap();
        assert!(winner.1.metrics.qualifies(train.base.metrics));
        assert_eq!(winner.1.metrics.iterations, train.base.metrics.iterations);
        assert_eq!(label, "split4_iter16_1");
        let baseline = &train.base.row;
        let equivalent = train
            .trials
            .iter()
            .find(|(b, _)| budget_label(*b) == "iter16_iter16_1")
            .unwrap();
        assert_eq!(baseline["window"], equivalent.1.row["window"]);
        assert_eq!(
            baseline["stroke_end_com_delta_mm"],
            equivalent.1.row["stroke_end_com_delta_mm"]
        );
        Ok(())
    }
}
