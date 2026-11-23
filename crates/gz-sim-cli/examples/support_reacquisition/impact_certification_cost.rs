// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time matched certified fixtures through the normal Bevy actuator entry point.
use super::*;
use std::time::Instant;
const POLICIES: [Policy; 4] = [
    Policy::Default,
    Policy::Refresh,
    Policy::Combined,
    Policy::Scoped,
];
#[derive(Resource)]
struct Setup {
    ids: Bodies,
    policy: Policy,
}
fn step(
    t: Res<UpdateInfo>,
    fault: Res<StepFault>,
    setup: Res<Setup>,
    mut backend: ResMut<PhysicsBackend>,
) {
    if t.dt.is_zero() {
        return;
    }
    let engine = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
    for _ in 0..8 {
        let (refresh, relax) = match setup.policy {
            Policy::Default => (false, 1),
            Policy::Refresh => (true, 1),
            Policy::Combined => (true, 16),
            Policy::Relaxation => (false, 16),
            Policy::Scoped => match state(engine, setup.ids) {
                Ok(s) => schedule(s, setup.policy),
                Err(e) => {
                    fault.report(e);
                    return;
                }
            },
        };
        if let Err(e) = advance(engine, setup.ids, refresh, relax) {
            fault.report(e);
            return;
        }
    }
}
fn measure(r: Reference, pad_first: bool, policy: Policy, endpoint: State) -> Result<[u64; 8]> {
    let (native, ids) = fresh_in_order(r.state(0), true, 1, pad_first)?;
    let mut sim = Simulation::new(OUTER)?;
    gz_physics::install_with_step(&mut sim, native, step)?;
    sim.world_mut().insert_resource(Setup { ids, policy });
    let mut times = [0; 8];
    for t in &mut times {
        let start = Instant::now();
        sim.step_exact(1)?;
        *t = start.elapsed().as_nanos() as u64;
    }
    let native = sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    if state(native, ids)? != endpoint {
        return Err("cost trial differs from fully observed physical run".into());
    }
    Ok(times)
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut raw = vec![];
    let mut pool: [Vec<u64>; 4] = Default::default();
    for id in 0..N_CASES {
        let r = Reference::solve(build(id))?;
        if !r.certification().0 {
            continue;
        }
        let hit = r.case.arrival_ns.div_ceil(500_000) as usize - 1;
        for pad_first in [false, true] {
            let mut expected = vec![];
            for p in POLICIES {
                let frames = evaluate(r, true, pad_first, p, true)?.1;
                let end = frames.last().unwrap().state;
                measure(r, pad_first, p, end)?;
                expected.push(end);
            }
            let mut samples: [Vec<[u64; 8]>; 4] = Default::default();
            for rep in 0..4 {
                for pos in 0..4 {
                    let i = (rep + pos) % 4;
                    let i = if (id + usize::from(pad_first)).is_multiple_of(2) {
                        i
                    } else {
                        3 - i
                    };
                    let t = measure(r, pad_first, POLICIES[i], expected[i])?;
                    pool[i].push(t[hit]);
                    samples[i].push(t);
                }
            }
            for (p, t) in POLICIES.into_iter().zip(samples) {
                raw.push(json!({"id":id,"pad_first":pad_first,"policy":p,"primary_tick":hit+1,"tick_ns":t}));
            }
        }
    }
    let summary=POLICIES.into_iter().zip(pool).map(|(p,mut values)|json!({"policy":p,"primary_count":values.len(),"primary_p50_ns":super::super::cost::rank(&mut values,50),"primary_p95_ns":super::super::cost::rank(&mut values,95)})).collect::<Vec<_>>();
    write_json(
        &path.join("cost.json"),
        &json!({"measured":"Bevy step_exact(1), native solves, scoped pose/twist reads and selection, without diagnostic reads","repetitions":4,"summaries":summary,"samples":raw}),
    )
}
