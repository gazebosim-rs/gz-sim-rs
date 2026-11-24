// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Wall time per advancing 500 µs tick, including all of that tick's native solves.
use super::*;
use bevy_ecs::prelude::{Res, ResMut};
use gz_sim_core::{Phase, StepFault, UpdateInfo};
use std::time::Instant;

const BLOCKS: usize = 6;
const REPEATS: usize = 8;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Scope {
    Bevy,
    Native,
}

#[derive(Clone, Copy)]
struct Condition {
    scope: Scope,
    receiver: usize,
    motion: usize,
    factor: u32,
}

type States = [BodyState; 3];

fn expected_states(cut: &Cut, case: Condition, steps: &[Outer]) -> Result<Vec<States>> {
    let donor = (case.motion != case.receiver).then_some(case.motion);
    let mut fork = cut.fork(case.receiver, donor)?;
    let mut states = vec![fork.states()];
    for step in steps {
        fork.step_outer(step.substeps(case.factor), step.command, |_| {})?;
        states.push(fork.states());
    }
    Ok(states)
}

// Only this known three-body SDF is used here. Keep its input application, ECS commits,
// tier policies and clock intact. Replace its one physics step system in the refined lanes.
fn install_window(lane: &mut Lane, factor: u32) -> Result<()> {
    let first_iteration = lane.sim.world().resource::<UpdateInfo>().iteration + 1;
    if lane
        .sim
        .remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)?
        != 1
    {
        return Err("expected one physics step system in the landing fixture".into());
    }
    lane.sim.add_systems(
        Phase::PhysicsStep,
        move |t: Res<UpdateInfo>, fault: Res<StepFault>, mut physics: ResMut<PhysicsBackend>| {
            if t.dt.is_zero() {
                return;
            }
            let n = if (first_iteration..first_iteration + WINDOW_STEPS as u64)
                .contains(&t.iteration)
            {
                factor
            } else {
                1
            };
            for _ in 0..n {
                if let Err(err) = physics.step(t.dt / n) {
                    fault.report(err);
                    break;
                }
            }
        },
    );
    lane.sim.initialize_schedules()?;
    Ok(())
}

fn live_states(lane: &Lane) -> Result<States> {
    let backend = lane.sim.world().resource::<PhysicsBackend>();
    let state = |id| backend.state(id).ok_or("live body missing");
    Ok([state(lane.foot)?, state(lane.upper)?, state(lane.pad)?])
}

// Compare every state on the untimed warm-up. On timed runs check the beginning
// and end only, so that diagnostic reads do not warm the contact loop between ticks.
fn run(
    cut: &Cut,
    case: Condition,
    steps: &[Outer],
    expected: &[States],
    check_all: bool,
) -> Result<Vec<u64>> {
    let mut samples = Vec::with_capacity(steps.len());
    match case.scope {
        Scope::Bevy => {
            let mut lane = Cut::live_history(case.receiver == 1)?;
            if live_states(&lane)? != expected[0] {
                return Err("full scene did not recreate the measured incoming state".into());
            }
            if case.factor > 1 {
                install_window(&mut lane, case.factor)?;
            }
            for (i, step) in steps.iter().enumerate() {
                if let Some(speed) = step.command {
                    lane.drive(speed)?;
                }
                let start = Instant::now();
                lane.sim.step_exact(1)?;
                samples.push(start.elapsed().as_nanos() as u64);
                if (check_all || i + 1 == steps.len()) && live_states(&lane)? != expected[i + 1] {
                    return Err(format!(
                        "live scene diverged from native fork at tick {}, receiver {}, factor {}",
                        step.tick, case.receiver, case.factor
                    )
                    .into());
                }
            }
        }
        Scope::Native => {
            let donor = (case.receiver != case.motion).then_some(case.motion);
            let mut fork = cut.fork(case.receiver, donor)?;
            for (i, step) in steps.iter().enumerate() {
                let n = step.substeps(case.factor);
                let start = Instant::now();
                fork.step_outer(n, step.command, |_| {})?;
                samples.push(start.elapsed().as_nanos() as u64);
                if (check_all || i + 1 == steps.len()) && fork.states() != expected[i + 1] {
                    return Err("timed native fork diverged from recorded control".into());
                }
            }
        }
    }
    Ok(samples)
}

fn quantiles(mut samples: Vec<u64>) -> Value {
    samples.sort_unstable();
    let count = samples.len();
    let rank = |p: usize| samples[(count * p).div_ceil(100) - 1];
    json!({"n":count,"p50_ns":rank(50),"p95_ns":rank(95)})
}

fn statistics(samples: &[Vec<u64>]) -> Value {
    let window = samples
        .iter()
        .flat_map(|r| r[..WINDOW_STEPS].iter().copied())
        .collect();
    let remainder = samples
        .iter()
        .flat_map(|r| r[WINDOW_STEPS..].iter().copied())
        .collect();
    json!({"window":quantiles(window),"remainder":quantiles(remainder),
        "continuation":quantiles(samples.iter().flatten().copied().collect())})
}

pub(super) fn write(cut: &Cut, output: &Path) -> Result<()> {
    let steps = schedule();
    let mut cases = Vec::new();
    for factor in [1, 2, 4] {
        for receiver in 0..2 {
            cases.push(Condition {
                scope: Scope::Bevy,
                receiver,
                motion: receiver,
                factor,
            });
            for motion in 0..2 {
                cases.push(Condition {
                    scope: Scope::Native,
                    receiver,
                    motion,
                    factor,
                });
            }
        }
    }
    let mut expected = Vec::new();
    for &case in &cases {
        let states = expected_states(cut, case, &steps)?;
        run(cut, case, &steps, &states, true)?;
        expected.push(states);
    }
    let mut samples = vec![Vec::new(); cases.len()];
    for round in 0..BLOCKS * REPEATS {
        for offset in 0..cases.len() {
            let i = if round % 2 == 0 {
                (round + offset) % cases.len()
            } else {
                (round + cases.len() - offset) % cases.len()
            };
            samples[i].push(run(cut, cases[i], &steps, &expected[i], false)?);
        }
    }
    let mut rows = Vec::new();
    let mut raw = Vec::new();
    for (case, records) in cases.into_iter().zip(samples) {
        let mut row = statistics(&records);
        row["blocks"] = records
            .chunks_exact(REPEATS)
            .map(statistics)
            .collect::<Vec<_>>()
            .into();
        let scope = if case.scope == Scope::Bevy {
            "bevy"
        } else {
            "native"
        };
        row["scope"] = scope.into();
        row["receiver"] = LABELS[case.receiver].into();
        row["motion"] = LABELS[case.motion].into();
        row["factor"] = case.factor.into();
        rows.push(row);
        raw.push(json!({"scope":scope,"receiver":LABELS[case.receiver],"motion":LABELS[case.motion],"factor":case.factor,"elapsed_ns":records}));
    }
    fs::write(
        output.join("timing-samples.json"),
        serde_json::to_vec(&raw)?,
    )?;
    fs::write(
        output.join("timings.json"),
        serde_json::to_vec_pretty(&json!({
            "blocks":BLOCKS,"repetitions_per_block":REPEATS,"first_tick":steps[0].tick,
            "last_tick":steps.last().unwrap().tick,"window_ticks":WINDOW_STEPS, "outer_us":STEP_US,
            "percentile":"nearest rank over individual fixed-duration outer steps", "rows":rows
        }))?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn refinement_cost_is_qualified_on_the_full_bevy_clock() -> Result<()> {
        let cut = Cut::new()?;
        let steps = schedule();
        for receiver in 0..2 {
            for factor in [1, 2, 4] {
                let case = Condition {
                    scope: Scope::Bevy,
                    receiver,
                    motion: receiver,
                    factor,
                };
                let states = expected_states(&cut, case, &steps)?;
                let times = run(&cut, case, &steps, &states, true)?;
                assert_eq!(times.len(), steps.len());
            }
        }
        Ok(())
    }
    #[test]
    fn timing_windows_count_original_outer_ticks() {
        let row = (1..=20).collect::<Vec<u64>>();
        let stats = statistics(&[row.clone(), row]);
        assert_eq!(stats["window"]["p95_ns"], 10);
        assert_eq!(stats["remainder"]["p50_ns"], 15);
        assert_eq!(stats["continuation"]["p95_ns"], 19);
        assert_eq!(stats["continuation"]["n"], 40);
    }
}
