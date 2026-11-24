// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Closed-loop primary-onset refinement across new fixture placements and drive speeds.
use super::*;
#[path = "primary_timing.rs"]
mod benchmark;
pub(crate) use benchmark::timing;

const HELD_OUT: [Fixture; 8] = [
    Fixture::new("mid-slow", 0.200, 0.92, false),
    Fixture::new("mid-fast", 0.200, 1.08, false),
    Fixture::new("edge-slow", 0.250, 0.92, false),
    Fixture::new("edge-fast", 0.250, 1.08, false),
    Fixture::new("mid-slow-holds", 0.200, 0.92, true),
    Fixture::new("mid-fast-holds", 0.200, 1.08, true),
    Fixture::new("edge-slow-holds", 0.250, 0.92, true),
    Fixture::new("edge-fast-holds", 0.250, 1.08, true),
];
const fn primary_grid(calls: u32, iterations: usize) -> Mode {
    Mode::PrimaryOnset {
        secondary: BUDGET,
        primary: Effort::Grid {
            calls,
            iterations,
            phase: GridPhase::Aligned,
        },
    }
}
const LIVE: [(&str, Mode); 5] = [
    ("current", ADAPTIVE),
    ("4x16", primary_grid(4, 16)),
    ("8x4", primary_grid(8, 4)),
    ("8x8", primary_grid(8, 8)),
    ("16x4", primary_grid(16, 4)),
];
const REFERENCES: [(&str, Mode); 3] = [
    ("uniform16", Mode::Uniform(Effort::Split(16))),
    ("uniform32", Mode::Uniform(Effort::Split(32))),
    ("uniform64", Mode::Uniform(Effort::Split(64))),
];

fn fixtures() -> impl Iterator<Item = (Fixture, bool)> {
    CASES
        .into_iter()
        .filter(|f| f.name.starts_with("outer-"))
        .map(|f| (f, false))
        .chain(HELD_OUT.into_iter().map(|f| (f, true)))
}
fn raw(frames: &[ReboundFrame]) -> Vec<Value> {
    frames.iter().map(|r|json!({"tick":r.tick,"normal_ns":r.normal_impulse_ns,"corners_ns":r.corner_ns,"tangent_on_foot_ns":r.tangent_on_foot_ns,
        "com_m":com(r).to_array(),"velocity_m_s":velocity(r).to_array(),"spin_rad_s":r.foot.angular_velocity,
        "primary_start":r.primary_start,"native_calls":r.native_steps,"solver_iterations":r.solver_iterations,"first_loaded_substep":r.first_loaded_substep})).collect()
}
fn matched(f: Fixture, current: &[ReboundFrame], out: &Path) -> Result<Vec<Value>> {
    let mut result = vec![];
    for cycle in 0..3 {
        let load = primary(current, cycle)?;
        // Re-enter before either the candidate prediction or the first positive native load.
        let depart = (sweep::DEPART_US[cycle] / STEP_US) as usize;
        let trig = current[depart + 1..]
            .iter()
            .take_while(|r| cycle == 2 || r.tick <= sweep::DEPART_US[cycle + 1] / STEP_US)
            .find(|r| r.primary_start)
            .map(|r| r.tick as usize);
        let cut_tick = trig.map_or(load, |i| load.min(i)) - 1;
        let cut = &current[cut_tick];
        let end = load + 25;
        let mut entries = vec![];
        for (name, mode) in LIVE.into_iter().chain(REFERENCES) {
            let frames = branch(f, ADAPTIVE, cut, mode, end)?;
            if name == "current" {
                native_prefix_check(&frames, &current[cut_tick..=end])?;
            }
            if f.name == HELD_OUT[0].name
                && cycle == 0
                && let Some(file) = match name {
                    "current" => Some("current"),
                    "8x8" => Some("live_8x8"),
                    "uniform32" => Some("uniform32"),
                    _ => None,
                }
            {
                trace(out, file, &frames)?;
            }
            entries.push(json!({"mode":name,"samples":raw(&frames)}));
        }
        result.push(json!({"stroke":cycle+1,"cut_tick":cut_tick,"baseline_first_load":load,"baseline_live_trigger":trig,"entries":entries}));
    }
    Ok(result)
}
fn continuous(frames: &[ReboundFrame], holds: usize) -> Result<Value> {
    let events = (0..3)
        .map(|i| primary(frames, i))
        .collect::<Result<Vec<_>>>()?;
    let compact = frames
        .iter()
        .skip(1)
        .map(|r| (com(r).to_array(), r.normal_impulse_ns))
        .collect::<Vec<_>>();
    let triggers=frames.iter().filter(|r|r.primary_start).map(|r|json!({"tick":r.tick,"prior_ns":frames[r.tick as usize-1].normal_impulse_ns,
        "normal_ns":r.normal_impulse_ns,"parked":parked(r.tick),"calls":r.native_steps,"iterations":r.solver_iterations})).collect::<Vec<_>>();
    Ok(
        json!({"observed_primary_ticks":events,"triggers":triggers,"prescribed_boundaries":holds,
        "native_calls":frames[1..].iter().map(|r|r.native_steps as usize).sum::<usize>(),"motion_impulse":compact}),
    )
}

pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut matched_runs = vec![];
    let mut continuous_runs = vec![];
    for (f, held) in fixtures() {
        let (current, holds) = run(f, ADAPTIVE)?;
        matched_runs
            .push(json!({"fixture":f,"held_out":held,"primaries":matched(f,&current,out)?}));
        let mut histories = vec![json!({"mode":"current","run":continuous(&current,holds)?})];
        for (name, mode) in LIVE[1..].iter().chain(&REFERENCES[..2]) {
            let (frames, holds) = run(f, *mode)?;
            histories.push(json!({"mode":name,"run":continuous(&frames,holds)?}));
        }
        continuous_runs.push(json!({"fixture":f,"held_out":held,"histories":histories}));
    }
    write_json(
        &out.join("matched.json"),
        &json!({"outer_us":STEP_US,"fixtures":matched_runs}),
    )?;
    write_json(
        &out.join("continuous.json"),
        &json!({"outer_us":STEP_US,"ticks":DURATION,"fixtures":continuous_runs}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn an_armed_primary_changes_only_the_first_tick_and_recovers_from_a_live_prefix() -> Result<()>
    {
        let f = HELD_OUT[0];
        let (prior, _) = run(f, ADAPTIVE)?;
        let first = prior
            .iter()
            .position(|r| r.primary_start)
            .ok_or("no primary")?;
        let configured = primary_grid(8, 8);
        let live = run(f, configured)?.0;
        let restarted = branch(f, ADAPTIVE, &prior[first - 1], configured, first + 4)?;
        native_prefix_check(&restarted, &live[first - 1..=first + 4])?;
        assert!(
            live[..first]
                .iter()
                .zip(&prior)
                .all(|(a, b)| same_state(a, b))
        );
        let trigger = &live[first];
        assert!(trigger.primary_start && trigger.normal_impulse_ns > 0.);
        assert_eq!((trigger.native_steps, trigger.solver_iterations), (8, 8));
        let after = &live[first + 1];
        assert!(!after.primary_start);
        assert_eq!((after.native_steps, after.solver_iterations), (1, 16));
        assert_eq!(
            (
                live[first + 2].native_steps,
                live[first + 2].solver_iterations
            ),
            (1, 4)
        );
        let primary_ticks = (0..3)
            .map(|stroke| primary(&live, stroke))
            .collect::<Result<Vec<_>>>()?;
        let selected = live
            .iter()
            .filter(|r| r.primary_start)
            .map(|r| r.tick as usize)
            .collect::<Vec<_>>();
        assert_eq!(selected, primary_ticks);
        Ok(())
    }
}
