// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Full-scene continuous triggers and fixed-clock latency, including contact reporting.
use super::*;
use bevy_ecs::prelude::{Res, ResMut, Resource};
use gz_sim_core::{Phase, StepFault, UpdateInfo};
use std::time::Instant;

#[derive(Resource, Default)]
struct StepReaction {
    normal: f64,
    selected: bool,
    native_steps: u32,
}
#[derive(Clone, Copy)]
struct Setting {
    policy: Policy,
    stock: bool,
}
const STOCK: Setting = Setting {
    policy: COARSE,
    stock: true,
};

fn install(lane: &mut Lane, policy: Policy, first: u64) -> Result<()> {
    let foot = lane.foot;
    let pad = lane.pad;
    let mut gate = Gate::default();
    let mut epoch = lane.sim.epoch();
    if lane
        .sim
        .remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)?
        != 1
    {
        return Err("expected one native stepping system on this fixture".into());
    }
    lane.sim.world_mut().init_resource::<StepReaction>();
    lane.sim.add_systems(
        Phase::PhysicsStep,
        move |time: Res<UpdateInfo>,
              fault: Res<StepFault>,
              mut backend: ResMut<PhysicsBackend>,
              mut r: ResMut<StepReaction>| {
            if time.dt.is_zero() {
                return;
            }
            if epoch != time.epoch {
                gate = Gate::default();
                epoch = time.epoch;
            }
            let tick = time.iteration.saturating_sub(SETTLE_STEPS);
            let active = if policy.online {
                let (Some(b), Some(p)) = (backend.state(foot), backend.state(pad)) else {
                    fault.report(PhysicsError::MissingBody);
                    return;
                };
                gate.advance(b, p, policy.ticks, time.dt.as_secs_f64())
            } else {
                policy.active(tick, first)
            };
            r.selected = active;
            r.normal = 0.;
            r.native_steps = if active { policy.substeps } else { 1 };
            let solver = backend
                .downcast_mut::<MultiFidelityBackend>()
                .expect("fixture native owner");
            let dt = time.dt / r.native_steps;
            for _ in 0..r.native_steps {
                let result = match (active, policy.iterations) {
                    (true, Some(n)) => solver.step_landing_with_iterations(dt, n),
                    _ => gz_physics::DynamicBackend::step(solver, dt),
                };
                if let Err(e) = result {
                    fault.report(e);
                    return;
                }
                // Complete outer-step receipt. A sensor sampling only the final
                // substep can miss the landing, including most of the next tick's load.
                r.normal += solver
                    .rigid_contact_observations()
                    .iter()
                    .filter(|p| (p.a, p.b) == (foot, pad) || (p.a, p.b) == (pad, foot))
                    .map(|p| p.normal_impulse)
                    .sum::<f64>();
            }
        },
    );
    lane.sim.initialize_schedules()?;
    Ok(())
}

fn label(s: Setting) -> &'static str {
    if s.stock { "stock" } else { s.policy.label }
}
struct Live {
    wall: Vec<u64>,
    selected: Vec<u64>,
    window: f64,
    post: f64,
}
fn live(f: Flight, setting: Setting, verify: bool) -> Result<Live> {
    let mut lane = f.case.lane;
    if !setting.stock {
        install(&mut lane, setting.policy, f.baseline_onset)?;
    }
    let mut last = sweep::command((f.cut - 1) * STEP_US, 0).1;
    let mut out = Live {
        wall: Vec::with_capacity((f.stop - f.cut) as usize),
        selected: Vec::new(),
        window: 0.,
        post: 0.,
    };
    for tick in f.cut + 1..=f.stop {
        let speed = sweep::command((tick - 1) * STEP_US, 0).1;
        if speed != last {
            lane.drive(speed)?;
            last = speed;
        }
        let begin = Instant::now();
        lane.sim.step_exact(1)?;
        out.wall.push(begin.elapsed().as_nanos() as u64);
        if verify {
            let j = if setting.stock {
                lane.sample()?.normal_impulse_ns
            } else {
                let r = lane.sim.world().resource::<StepReaction>();
                if r.selected {
                    out.selected.push(tick);
                }
                r.normal
            };
            out.post += j;
            if (f.baseline_onset..f.baseline_onset + 10).contains(&tick) {
                out.window += j;
            }
        }
    }
    Ok(out)
}

// Exercise the rising, braking, parked and departing lift in one live scene.
// Each subsequent touchdown depends on the simulated previous one.
fn continuous(history: usize, policy: Policy) -> Result<Value> {
    let mut c = matched::Case::new(if history == 0 { 0 } else { 7 }, 1800)?;
    install(&mut c.lane, policy, 0)?;
    c.advance(0, None)?;
    let until = sweep::END_US / STEP_US;
    let mut last = 0.;
    let mut fired = false;
    let mut events = Vec::new();
    let mut landings = Vec::new();
    let mut free = false;
    let mut selected = 0;
    let mut substeps = 0;
    let mut prescribed = 0;
    let mut park_selected = 0;
    for tick in 1..=until {
        let (i, speed) = sweep::command((tick - 1) * STEP_US, 0);
        c.advance(tick as usize, (speed != last).then_some(speed))?;
        last = speed;
        let r = c.lane.sim.world().resource::<StepReaction>();
        if r.selected && !fired {
            events.push(json!({"cycle":i+1,"tick":tick}));
        }
        if r.selected {
            selected += 1;
            let park = sweep::DEPART_US[i] / STEP_US + STROKES[i].start_park();
            if tick <= sweep::DEPART_US[0] / STEP_US || tick > park {
                park_selected += 1;
            }
        }
        substeps += r.native_steps;
        fired = r.selected;
        prescribed += usize::from(c.lane.tier()? == Prescribed);
        let current = c.samples[tick as usize];
        if r.normal == 0. && current.vertical_gap_m > 1e-5 {
            free = true;
        }
        if free && r.normal > 0. {
            landings
                .push(json!({"cycle":i+1,"tick":tick,"outer_ns":r.normal,"selected":r.selected}));
            free = false;
        }
    }
    Ok(
        json!({"history":LABELS[history],"policy":policy.label,"trigger_events":events,"landings":landings,
        "selected_ticks":selected,"native_steps":substeps,"outer_steps":until,"prescribed_boundaries":prescribed,"selected_parked_ticks":park_selected}),
    )
}

const BLOCKS: usize = 6;
const PER_BLOCK: usize = 5;
fn p95_ns(v: &[u64]) -> u64 {
    let mut v = v.to_vec();
    v.sort_unstable();
    v[(v.len() * 95).div_ceil(100) - 1]
}

pub(super) fn write(out: &Path) -> Result<Value> {
    let policies = [
        STOCK,
        Setting {
            policy: COARSE,
            stock: false,
        },
        Setting {
            policy: TEN,
            stock: false,
        },
        Setting {
            policy: GATE,
            stock: false,
        },
        Setting {
            policy: POLICIES
                .iter()
                .copied()
                .find(|p| p.label == "gate_2")
                .unwrap(),
            stock: false,
        },
        Setting {
            policy: GATE_ITER,
            stock: false,
        },
        Setting {
            policy: POLICIES
                .iter()
                .copied()
                .find(|p| p.label == "coarse_16iter")
                .unwrap(),
            stock: false,
        },
    ];
    let mut cases = Vec::new();
    let mut raw = Vec::new();
    for history in 0..2 {
        for setting in policies {
            let f = flight(2, history, 0)?;
            let window_index = (f.baseline_onset - f.cut - 1) as usize;
            let check = run(&f, setting.policy, 0, None)?;
            let parity = live(f, setting, true)?;
            if (parity.window - check["window_ns"].as_f64().unwrap()).abs() > 1e-11
                || (parity.post - check["stroke_ns"].as_f64().unwrap()).abs() > 1e-11
                || json!(parity.selected) != check["selected_ticks"]
            {
                return Err(format!(
                    "live schedule differed from detached control on {}",
                    label(setting)
                )
                .into());
            }
            cases.push((history, setting, window_index));
            raw.push(Vec::new());
        }
    }
    for rep in 0..BLOCKS * PER_BLOCK {
        for pos in 0..cases.len() {
            let i = if rep % 2 == 0 {
                (rep + pos) % cases.len()
            } else {
                (rep + cases.len() - pos) % cases.len()
            };
            let (h, p, _) = cases[i];
            raw[i].push(live(flight(2, h, 0)?, p, false)?.wall);
        }
    }
    let mut rows = Vec::new();
    let mut records = Vec::new();
    for ((h, p, i), v) in cases.into_iter().zip(raw) {
        let windows = |samples: &[Vec<u64>]| {
            samples
                .iter()
                .flat_map(|x| x[i..i + 10].iter().copied())
                .collect::<Vec<_>>()
        };
        let window = windows(&v);
        let continuation = v.iter().flatten().copied().collect::<Vec<_>>();
        let us = |v: &[u64]| p95_ns(v) as f64 * 0.001;
        let first2 = v
            .iter()
            .flat_map(|x| x[i..i + 2].iter().copied())
            .collect::<Vec<_>>();
        let blocks = v
            .chunks_exact(PER_BLOCK)
            .map(|b| us(&windows(b)))
            .collect::<Vec<_>>();
        let window_cost = v
            .iter()
            .map(|x| x[i..i + 10].iter().sum::<u64>())
            .sum::<u64>() as f64
            / (v.len() as f64 * 1000.);
        rows.push(json!({"history":LABELS[h],"policy":label(p),"window_samples":window.len(),"start_index":i,
            "p95_window_us":us(&window),"p95_first_two_us":us(&first2),"p95_continuation_us":us(&continuation),
            "mean_window_total_us":window_cost,"block_window_p95_us":blocks}));
        records.push(json!({"history":LABELS[h],"policy":label(p),"outer_elapsed_ns":v}));
    }
    let mut continuous_runs = Vec::new();
    for h in 0..2 {
        for policy in [COARSE, GATE, GATE_ITER] {
            continuous_runs.push(continuous(h, policy)?);
        }
    }
    fs::write(
        out.join("continuous.json"),
        serde_json::to_vec_pretty(&continuous_runs)?,
    )?;
    fs::write(
        out.join("timing-samples.json"),
        serde_json::to_vec(&records)?,
    )?;
    let report = json!({"blocks":BLOCKS,"repeats_per_block":PER_BLOCK,"rows":rows});
    fs::write(
        out.join("timings.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn one_live_gate_rearms_for_all_three_strokes_without_triggering_while_parked() -> Result<()> {
        for h in 0..2 {
            let r = continuous(h, GATE_ITER)?;
            assert_eq!(r["selected_parked_ticks"], 0);
            let events = r["trigger_events"].as_array().unwrap();
            for n in 1..=3 {
                assert!(events.iter().any(|e| e["cycle"] == n));
            }
            assert_eq!(r["native_steps"], r["outer_steps"]);
        }
        Ok(())
    }
}
