// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Primary and secondary reactions as state changes propagate through all three lift strokes.
use super::*;

const WINDOW: usize = 10;

#[derive(Clone, Copy)]
struct Landing {
    tick: usize,
    cycle: usize,
    primary: bool,
    peak_gap_m: f64,
}

struct History {
    frames: Vec<ReboundFrame>,
    landings: Vec<Landing>,
    prescribed: Vec<bool>,
}

fn cycle(tick: u64) -> usize {
    sweep::command(tick.saturating_sub(1) * STEP_US, 0).0
}

fn stroke_end(stroke: usize) -> usize {
    (sweep::DEPART_US[stroke] / STEP_US + STROKES[stroke].start_park()) as usize
}

fn parked(tick: u64) -> bool {
    tick <= sweep::DEPART_US[0] / STEP_US || tick as usize > stroke_end(cycle(tick))
}

impl History {
    fn run(deck: Deck, switched: bool, mode: Mode) -> Result<Self> {
        let mut rig = Rig::new(deck, switched, mode)?;
        let count = (sweep::END_US / STEP_US) as usize;
        let mut frames = Vec::with_capacity(count + 1);
        let mut prescribed = Vec::with_capacity(count + 1);
        frames.push(rig.frame()?);
        prescribed.push(rig.case.lane.tier()? == Prescribed);
        let mut landings = Vec::new();
        let mut seen = [false; 3];
        let mut peak_gap = 0.0_f64;
        for tick in 1..=count {
            rig.advance()?;
            let frame = rig.frame()?;
            if frame.normal_impulse_ns == 0. {
                peak_gap = peak_gap.max(frame.gap_m);
            } else {
                if peak_gap > 10e-6 {
                    let c = cycle(frame.tick);
                    landings.push(Landing {
                        tick,
                        cycle: c,
                        primary: !seen[c],
                        peak_gap_m: peak_gap,
                    });
                    seen[c] = true;
                }
                peak_gap = 0.;
            }
            prescribed.push(rig.case.lane.tier()? == Prescribed);
            frames.push(frame);
        }
        Ok(Self {
            frames,
            landings,
            prescribed,
        })
    }

    fn window(&self, tick: usize) -> f64 {
        self.frames[tick..(tick + WINDOW).min(self.frames.len())]
            .iter()
            .map(|f| f.normal_impulse_ns)
            .sum()
    }

    fn stroke_ns(&self, stroke: usize) -> f64 {
        let first = (sweep::DEPART_US[stroke] / STEP_US + 1) as usize;
        self.frames[first..=stroke_end(stroke)]
            .iter()
            .map(|f| f.normal_impulse_ns)
            .sum()
    }

    fn report(&self) -> Value {
        let steps = &self.frames[1..];
        let native: usize = steps.iter().map(|f| f.native_steps as usize).sum();
        let iterations: usize = steps
            .iter()
            .map(|f| f.native_steps as usize * f.solver_iterations)
            .sum();
        let selected = steps.iter().filter(|f| f.selected).count();
        let effort_ticks=steps.iter().filter(|f| f.selected || f.secondary_start).map(|f| json!({
            "tick":f.tick,"secondary_start":f.secondary_start,"selected":f.selected,
            "native_steps":f.native_steps,"iterations_per_solve":f.solver_iterations,"normal_ns":f.normal_impulse_ns
        })).collect::<Vec<_>>();
        let landings = self.landings.iter().map(|event|{
            let f=self.frames[event.tick];
            json!({"cycle":event.cycle+1,"tick":f.tick,"time_us":f.time_us,"primary":event.primary,
            "peak_gap_m":event.peak_gap_m,"first_outer_ns":f.normal_impulse_ns,"corners_ns":f.corner_ns,"window_ns":self.window(event.tick),
            "selected":f.selected,"secondary_start":f.secondary_start,"native_steps":f.native_steps,"iterations_per_solve":f.solver_iterations})
        }).collect::<Vec<_>>();
        let false_starts = steps
            .iter()
            .filter(|f| {
                f.secondary_start
                    && !self
                        .landings
                        .iter()
                        .any(|e| !e.primary && e.tick == f.tick as usize)
            })
            .map(|f| f.tick)
            .collect::<Vec<_>>();
        json!({"outer_steps":steps.len(),"duration_us":steps.len() as u64*STEP_US,
          "native_solves":native,"iteration_slots":iterations,"ordinary_iteration_slots":4*steps.len(),
          "extra_native_solves":native-steps.len(),"selected_ticks":selected,"selected_us":selected as u64*STEP_US,
          "prescribed_boundaries":self.prescribed[1..].iter().filter(|&&p|p).count(),
          "parked_ticks":steps.iter().filter(|f|parked(f.tick)).count(),
          "parked_selected_ticks":steps.iter().filter(|f|parked(f.tick) && f.selected).count(),
          "starts_without_secondary_landing":false_starts,"effort_ticks":effort_ticks,"landings":landings})
    }
}

fn same_state(a: &ReboundFrame, b: &ReboundFrame) -> bool {
    a.foot == b.foot
        && a.upper == b.upper
        && a.pad == b.pad
        && a.corner_ns == b.corner_ns
        && a.normal_impulse_ns == b.normal_impulse_ns
        && a.gap_m == b.gap_m
}

fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    DVec3::from(a).distance(DVec3::from(b))
}

fn comparison(base: &History, candidate: &History, budget: Budget) -> Result<Value> {
    let paired = || base.frames.iter().zip(&candidate.frames);
    let first_change = paired().position(|(a, b)| {
        a.native_steps != b.native_steps || a.solver_iterations != b.solver_iterations
    });
    let first_divergence = paired().position(|(a, b)| !same_state(a, b));
    let prefix = first_change.unwrap_or(base.frames.len());
    if !paired().take(prefix).all(|(a, b)| {
        same_state(a, b) && a.secondary_start == b.secondary_start && a.selected == b.selected
    }) || base.prescribed[..prefix] != candidate.prescribed[..prefix]
    {
        return Err("continuous controls diverged before changing the secondary effort".into());
    }
    if let Some(tick) = first_change {
        let start = base.frames[..=tick].iter().rposition(|f| f.secondary_start);
        if base.frames[tick].secondary_start != candidate.frames[tick].secondary_start
            || !start.is_some_and(|start| tick - start <= usize::from(budget.follow_ticks.max(1)))
        {
            return Err("first changed effort was outside its causal secondary budget".into());
        }
    }
    let mut strokes = Vec::new();
    for i in 0..3 {
        let first_primary = |history: &History| {
            history
                .landings
                .iter()
                .find(|e| e.primary && e.cycle == i)
                .map(|e| e.tick)
                .ok_or("missing primary in continuous return")
        };
        let (a, b) = (first_primary(base)?, first_primary(candidate)?);
        let primary_effort = [(base, a), (candidate, b)].map(|(h, onset)| {
            h.frames[onset..onset + 2]
                .iter()
                .all(|f| f.selected && f.native_steps == 1 && f.solver_iterations == 16)
        });
        if a + 1 < prefix && (a != b || primary_effort != [true, true]) {
            return Err("primary effort changed before the first changed secondary solve".into());
        }
        let first = (sweep::DEPART_US[i] / STEP_US + 1) as usize;
        let stop = stroke_end(i);
        let primary_tick_shift = b as i64 - a as i64;
        let first_delta = candidate.frames[b].normal_impulse_ns - base.frames[a].normal_impulse_ns;
        let total = [base.stroke_ns(i), candidate.stroke_ns(i)];
        let max_foot = (first..=stop)
            .map(|t| {
                distance(
                    base.frames[t].foot.position,
                    candidate.frames[t].foot.position,
                )
            })
            .fold(0., f64::max);
        strokes.push(json!({"cycle":i+1,"moving_ticks":[first,stop],"primary_ticks":[a,b],"primary_both_ticks_16_iterations":primary_effort,"primary_tick_shift":primary_tick_shift,
            "primary_first_delta_ns":first_delta,"primary_event_relative_5ms_delta_ns":candidate.window(b)-base.window(a),
            "moving_stroke_ns":total,"moving_stroke_delta_ns":total[1]-total[0],"max_foot_position_delta_m":max_foot,
            "end_foot_position_delta_m":distance(base.frames[stop].foot.position,candidate.frames[stop].foot.position)}));
    }
    let max_pad = paired()
        .map(|(a, b)| distance(a.pad.position, b.pad.position))
        .fold(0., f64::max);
    let max_command_error = paired()
        .skip(1)
        .map(|(a, b)| {
            let expected = [0., 0., sweep::command((a.tick - 1) * STEP_US, 0).1];
            distance(a.pad.velocity, expected).max(distance(b.pad.velocity, expected))
        })
        .fold(0., f64::max);
    if max_pad > 1e-12
        || max_command_error > 1e-11
        || !paired().all(|(a, b)| {
            a.pad.orientation == b.pad.orientation
                && a.time_us == b.time_us
                && a.time_us == a.tick * STEP_US
        })
    {
        return Err("drive changed between continuous budget controls".into());
    }
    Ok(
        json!({"identical_incoming_states_before_tick":first_change,"identical_prefix_outer_ticks":prefix-1,
        "first_different_effort_tick":first_change,"first_different_state_or_reaction_tick":first_divergence,"max_pad_position_delta_m":max_pad,"max_pad_speed_command_error_m_s":max_command_error,"strokes":strokes}),
    )
}

/// Score the same physical clock interval in all three independently evolved
/// worlds. An ordinal rebound can disappear or be replaced after an earlier
/// intervention, so secondary events are never paired by ordinal index.
fn reference_scores(prior: &History, budget: &History, reference: &History) -> Value {
    let mut events = Vec::new();
    for event in &reference.landings {
        let tick = event.tick;
        let first = reference.frames[tick].normal_impulse_ns;
        let window = reference.window(tick);
        let delta = |h: &History| {
            json!({
                "first_delta_ns":h.frames[tick].normal_impulse_ns-first,
                "window_delta_ns":h.window(tick)-window,
                "event_observed_on_same_tick":h.landings.iter().any(|e|e.tick==tick),
            })
        };
        events.push(
            json!({"tick":tick,"cycle":event.cycle+1,"primary":event.primary,
            "reference_first_ns":first,"reference_window_ns":window,
            "prior":delta(prior),"budget":delta(budget)}),
        );
    }
    let strokes:Vec<_>=(0..3).map(|i| {
       let total=reference.stroke_ns(i);
       let a=prior.stroke_ns(i)-total;
       let b=budget.stroke_ns(i)-total;
       let end=stroke_end(i);
       let com=|h:&History| (DVec3::from(h.frames[end].foot.position)+DVec3::from(h.frames[end].upper.position))*0.5;
       json!({"cycle":i+1,"reference_moving_stroke_ns":total,"prior_delta_ns":a,"budget_delta_ns":b,
          "change_in_abs_stroke_error_ns":b.abs()-a.abs(),"prior_end_com_error_mm":1000.*com(prior).distance(com(reference)),
          "budget_end_com_error_mm":1000.*com(budget).distance(com(reference))})
    }).collect();
    json!({"events_on_reference_clock":events,"strokes":strokes})
}

pub(super) fn write(out: &Path, budget: Budget) -> Result<()> {
    let mut runs = Vec::new();
    let reference_budget = Budget {
        onset: Effort::Split(4),
        follow: Effort::Split(4),
        follow_ticks: 9,
    };
    for deck in [Deck::Flat, Deck::Edge] {
        for switched in [false, true] {
            let base = History::run(deck, switched, Mode::Candidate)?;
            let candidate = History::run(deck, switched, Mode::Budget(budget))?;
            let reference = History::run(deck, switched, Mode::Budget(reference_budget))?;
            runs.push(json!({"deck":deck.label(),"history":if switched{"switched"}else{"always_dynamic"},
                "prior_two_ticks":base.report(),"budget":candidate.report(),"prospective_reference":reference.report(),
                 "comparison":comparison(&base,&candidate,budget)?,"relative_to_prospective_reference":reference_scores(&base,&candidate,&reference)}));
        }
    }
    write_json(
        &out.join("continuous.json"),
        &json!({"outer_us":STEP_US,"event_window_us":WINDOW as u64*STEP_US,
        "release_witness_m":10e-6,"selected_budget":budget,"reference_budget":reference_budget,"runs":runs}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn equal_secondary_effort_reproduces_both_full_continuous_histories() -> Result<()> {
        let equal = Budget {
            onset: Effort::Iterations(16),
            follow: Effort::Iterations(16),
            follow_ticks: 1,
        };
        for (deck, switched) in [(Deck::Flat, true), (Deck::Edge, false)] {
            let a = History::run(deck, switched, Mode::Candidate)?;
            let b = History::run(deck, switched, Mode::Budget(equal))?;
            let audit = comparison(&a, &b, equal)?;
            assert_eq!(audit["first_different_effort_tick"], Value::Null);
            assert_eq!(audit["first_different_state_or_reaction_tick"], Value::Null);
            assert_eq!(audit["identical_prefix_outer_ticks"], 2171);
            assert_eq!(a.report(), b.report());
        }
        Ok(())
    }
    #[test]
    fn changed_secondary_budget_preserves_every_tick_before_its_first_intervention() -> Result<()> {
        let budget = Budget {
            onset: Effort::Split(4),
            follow: Effort::Standard,
            follow_ticks: 0,
        };
        let a = History::run(Deck::Flat, true, Mode::Candidate)?;
        let b = History::run(Deck::Flat, true, Mode::Budget(budget))?;
        let audit = comparison(&a, &b, budget)?;
        assert_eq!(audit["first_different_effort_tick"], 1454);
        assert_eq!(audit["identical_prefix_outer_ticks"], 1453);
        assert_eq!(b.report()["parked_selected_ticks"], 0);
        Ok(())
    }
}
