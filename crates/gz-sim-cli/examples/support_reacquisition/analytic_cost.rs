// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Primary-tick wall time on the analytic fixture with diagnostic observations outside the timer.
use super::*;
use std::time::Instant;

const PASSES: [usize; 6] = [1, 2, 4, 8, 16, 32];
// All six cyclic orderings and their reversals, repeated four times.
const REPEATS: usize = 48;

#[derive(Resource)]
struct Grid(NativeStepGrid, Option<usize>);
fn advance(
    t: Res<UpdateInfo>,
    fault: Res<StepFault>,
    mut backend: ResMut<PhysicsBackend>,
    grid: Res<Grid>,
) {
    if t.dt.is_zero() {
        return;
    }
    for dt in grid.0.intervals() {
        let result = match grid.1 {
            Some(sweeps) => backend
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .step_landing_with_relaxation(dt, sweeps),
            None => backend.step(dt),
        };
        if let Err(e) = result {
            fault.report(e);
            return;
        }
    }
}
pub(super) fn measure(
    ideal: Ideal,
    tree: bool,
    passes: usize,
    endpoint: State,
    relaxation: Option<usize>,
) -> Result<Vec<u64>> {
    let (native, ids) = fresh(ideal, tree, passes)?;
    let mut sim = Simulation::new(OUTER)?;
    gz_physics::install_with_step(&mut sim, native, advance)?;
    sim.world_mut()
        .insert_resource(Grid(NativeStepGrid::new(OUTER, 8)?, relaxation));
    let mut ns = Vec::with_capacity(END_TICKS as usize);
    for _ in 0..END_TICKS {
        let clock = Instant::now();
        sim.step_exact(1)?;
        ns.push(clock.elapsed().as_nanos() as u64);
    }
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    if state(native, ids)? != endpoint {
        return Err("timed endpoint differs from fully observed Bevy run".into());
    }
    Ok(ns)
}
pub(super) fn rank(values: &mut [u64], q: usize) -> u64 {
    values.sort_unstable();
    values[(q * values.len()).div_ceil(100) - 1]
}
pub(crate) fn write(out: &Path) -> Result<()> {
    fs::create_dir_all(out)?;
    let hit = (ARRIVAL_NS.div_ceil(OUTER.as_nanos() as u64) - 1) as usize;
    let mut results = vec![];
    for fixture in FIXTURES
        .into_iter()
        .filter(|f| matches!(f.label, "level-fast" | "corner-a-fast"))
    {
        let ideal = Ideal::new(fixture);
        for tree in [true, false] {
            let mut expected = vec![];
            for passes in PASSES {
                let observed = run(ideal, tree, 8, 4, passes)?;
                let endpoint = observed.frames.last().unwrap().state;
                measure(ideal, tree, passes, endpoint, None)?;
                expected.push(endpoint);
            }
            let mut records: [Vec<Vec<u64>>; 6] = Default::default();
            for rep in 0..REPEATS {
                for index in 0..PASSES.len() {
                    let j = (rep + index) % PASSES.len();
                    let i = if (rep / PASSES.len()).is_multiple_of(2) {
                        j
                    } else {
                        PASSES.len() - 1 - j
                    };
                    records[i].push(measure(ideal, tree, PASSES[i], expected[i], None)?);
                }
            }
            for (passes, rows) in PASSES.into_iter().zip(records) {
                let mut primary: Vec<_> = rows.iter().map(|r| r[hit]).collect();
                let mut all: Vec<_> = rows.iter().flatten().copied().collect();
                results.push(json!({"fixture":fixture.label,"representation":if tree{"locked_pair"}else{"single_rigid"},
                    "fixed_time_pgs_passes":passes,"primary_n":primary.len(),"primary_p50_ns":rank(&mut primary,50),
                    "primary_p95_ns":rank(&mut primary,95),"all_n":all.len(),"all_p95_ns":rank(&mut all,95),"step_ns":rows}));
            }
        }
    }
    write_json(
        &out.join("cost.json"),
        &json!({"native_calls":8,"temporal_iterations_per_call":4,"outer_ns":OUTER.as_nanos() as u64,
        "first_tick":1,"analytic_impact_tick":hit+1,"repetitions":REPEATS,"timed_scope":"complete Bevy step_exact(1), native advances and prescribed-motion update; no point diagnostics", "results":results}),
    )
}
