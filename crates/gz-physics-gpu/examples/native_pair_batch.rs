// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! 2,048 resident environments, with up to 96 persistent Rapier contact solvers.
#[path = "support/native_timing.rs"]
mod native_timing;
#[allow(dead_code)]
#[path = "support/sphere_pair/reference.rs"]
mod reference;
use gz_physics_gpu::{
    BodySlot, CentralImpulse, Environment, EnvironmentState, GpuError, MixedBatch,
    NativeHandoffRefusal,
    PairSolver::{self, NativeRapier, SpecializedGpu},
    STEP, SpherePairLane as Pair,
    simulation::install_mixed,
};
use gz_sim_core::Simulation;
use reference::V;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn values(p: Pair) -> EnvironmentState {
    EnvironmentState::SpherePair(p)
}
fn lane(s: EnvironmentState) -> Pair {
    let EnvironmentState::SpherePair(p) = s else {
        unreachable!()
    };
    p
}
fn population() -> (Vec<Environment>, Vec<u64>) {
    let initial: Vec<_> = reference::fixture(2048)
        .into_iter()
        .enumerate()
        .map(|(i, p)| Environment {
            id: i as u64,
            state: values(p),
        })
        .collect();
    // Expose all first-at-rest, touching or near-impact GPU seeds in the device
    // population; select the first 96 *initially separated* transferable pairs
    // in a coprime permutation so the eligible set spans the authored parameter range.
    let selected = (0..2048)
        .map(|i| (i * 787 + 19) % 2048)
        .filter(|&id| {
            let p = lane(initial[id].state);
            let dist = V::from(p.a.position).distance(V::from(p.b.position));
            dist - p.a.radius - p.b.radius > 0.004
                && dist - p.a.radius - p.b.radius
                    > STEP.as_secs_f64() * V::from(p.a.velocity).distance(V::from(p.b.velocity))
                        + 1e-6
        })
        .take(96)
        .map(|id| id as u64)
        .collect::<Vec<_>>();
    assert_eq!(selected.len(), 96);
    (initial, selected)
}
fn simulation(adapter: &str, initial: Vec<Environment>, ids: &[u64]) -> Result<Simulation> {
    let batch = MixedBatch::from_fixed_scene(adapter, initial)?.with_guarded_native_pairs(
        &ids.iter()
            .map(|&id| (id, SpecializedGpu))
            .collect::<Vec<_>>(),
    )?;
    let mut sim = Simulation::new(STEP)?;
    install_mixed(&mut sim, batch)?;
    Ok(sim)
}
struct Oracle {
    p: Pair,
    n: Option<reference::Native>,
    pending: Option<[f64; 3]>,
}
impl Oracle {
    fn owner(&mut self, t: PairSolver) {
        self.n = (t == NativeRapier).then(|| reference::Native::new(self.p));
    }
    fn advance(&mut self) -> V {
        if let Some(j) = self.pending.take() {
            self.p.a.velocity =
                (V::from(self.p.a.velocity) + V::from(j) * self.p.a.inverse_mass).to_array();
            if let Some(n) = &mut self.n {
                n.impulse_a(j);
            }
        }
        if let Some(n) = &mut self.n {
            n.step();
            self.p = n.state;
            n.impulse_b
        } else {
            reference::analytical(&mut self.p);
            V::from(self.p.contact_normal) * self.p.normal_impulse
                + V::from(self.p.tangential_impulse)
        }
    }
}
#[derive(Debug, Default)]
struct Delta {
    position_m: f64,
    velocity_m_s: f64,
    spin_rad_s: f64,
    quat_chord: f64,
    impulse_n_s: f64,
}
fn qualify(sim: &mut Simulation, selected: &[u64], out: &mut BufWriter<File>) -> Result<()> {
    let mut oracles = selected
        .iter()
        .map(|&id| Oracle {
            p: lane(sim.world().resource::<MixedBatch>().environments()[id as usize].state),
            n: None,
            pending: None,
        })
        .collect::<Vec<_>>();
    let mut delta = Delta::default();
    let mut moved = 0;
    let mut refused = 0;
    let mut native_impacts = 0;
    let mut gpu_impacts = 0;
    for tick in 0..=600 {
        if tick > 0 {
            sim.step_exact(1)?;
            let batch = sim.world().resource::<MixedBatch>();
            for (i, &id) in selected.iter().enumerate() {
                let j = oracles[i].advance();
                let p = lane(batch.environments()[id as usize].state);
                let q = oracles[i].p;
                for (a, b) in [(p.a, q.a), (p.b, q.b)] {
                    delta.position_m = delta
                        .position_m
                        .max(V::from(a.position).distance(b.position.into()));
                    delta.velocity_m_s = delta
                        .velocity_m_s
                        .max(V::from(a.velocity).distance(b.velocity.into()));
                    delta.spin_rad_s = delta
                        .spin_rad_s
                        .max(V::from(a.angular_velocity).distance(b.angular_velocity.into()));
                    delta.quat_chord = delta.quat_chord.max(
                        a.orientation
                            .iter()
                            .zip(b.orientation)
                            .map(|(a, b)| (a - b).powi(2))
                            .sum::<f64>()
                            .sqrt(),
                    );
                }
                let actual =
                    V::from(p.contact_normal) * p.normal_impulse + V::from(p.tangential_impulse);
                delta.impulse_n_s = delta.impulse_n_s.max(actual.distance(j));
                if actual.length() > 1e-7 {
                    if oracles[i].n.is_some() {
                        native_impacts += 1;
                    } else {
                        gpu_impacts += 1;
                    }
                }
            }
        }
        if [0, 60, 170, 310, 455, 595].contains(&tick) {
            let mut batch = sim.world_mut().resource_mut::<MixedBatch>();
            let actions: Vec<_> = selected
                .iter()
                .enumerate()
                .filter(|(i, _)| i % 11 == tick as usize % 11)
                .map(|(i, &id)| {
                    let j = [0.004, -0.002, 0.001];
                    oracles[i].pending = Some(j);
                    CentralImpulse {
                        id,
                        body: BodySlot::A,
                        impulse: j,
                    }
                })
                .collect();
            batch.queue_impulses(tick, &actions)?;
            let before = batch.environments().to_vec();
            let pending = batch.pending_impulses().copied().collect::<Vec<_>>();
            for target in [NativeRapier, SpecializedGpu] {
                let mut goal = selected
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| {
                        (i + tick as usize / 30).is_multiple_of(3) == (target == SpecializedGpu)
                    })
                    .map(|(_, &id)| id)
                    .filter(|&id| batch.pair_solver(id).unwrap() != target)
                    .collect::<Vec<_>>();
                while !goal.is_empty() {
                    match batch.set_pair_solver(tick, &goal, target) {
                        Ok(report) => {
                            moved += report.moved;
                            for id in goal.drain(..) {
                                oracles[selected.iter().position(|&v| v == id).unwrap()]
                                    .owner(target);
                            }
                        }
                        Err(GpuError::NativeHandoff {
                            id,
                            reason:
                                NativeHandoffRefusal::ContactHistory
                                | NativeHandoffRefusal::SweptClearance,
                        }) => {
                            assert!(goal.contains(&id));
                            assert!(
                                goal.iter()
                                    .all(|&id| batch.pair_solver(id).unwrap() != target)
                            );
                            goal.retain(|&v| v != id);
                            refused += 1;
                        }
                        Err(e) => return Err(e.into()),
                    }
                    assert_eq!(batch.environments(), before);
                    assert_eq!(
                        batch.pending_impulses().copied().collect::<Vec<_>>(),
                        pending
                    );
                }
            }
            writeln!(
                out,
                "{tick},{moved},{refused},{native_impacts},{gpu_impacts},{},{},{},{},{}",
                delta.position_m,
                delta.velocity_m_s,
                delta.spin_rad_s,
                delta.quat_chord,
                delta.impulse_n_s
            )?;
        }
    }
    assert!(native_impacts > 0 && gpu_impacts > 0 && moved > 96);
    assert!(
        delta.position_m < 1e-8
            && delta.velocity_m_s < 1e-8
            && delta.spin_rad_s < 1e-7
            && delta.quat_chord < 1e-8
            && delta.impulse_n_s < 1e-8,
        "independent 96 native/event controls {delta:?}"
    );
    println!(
        "qualified 2048 environments, 96 tracked native/event references, 600 steps, {moved} unchanged-frame moves, {refused} physical refusals, {native_impacts} native impulse ticks, {gpu_impacts} GPU impulse ticks, peak {delta:?}"
    );
    Ok(())
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("provide ADAPTER OUTPUT_DIRECTORY")?;
    let directory = args.next().ok_or("provide a fresh output directory")?;
    if args.next().is_some() {
        return Err("unexpected argument".into());
    }
    std::fs::create_dir(&directory)?;
    let dir = Path::new(&directory);
    let (initial, ids) = population();
    let mut sim = simulation(&adapter, initial, &ids)?;
    let mut trajectory = BufWriter::new(File::create_new(dir.join("accuracy.csv"))?);
    writeln!(
        trajectory,
        "tick,moves,refusals,native_impulse_ticks,gpu_impulse_ticks,position_m,velocity_m_s,spin_rad_s,quaternion_chord,impulse_Ns"
    )?;
    qualify(&mut sim, &ids, &mut trajectory)?;
    let mut out = BufWriter::new(File::create_new(dir.join("timing.csv"))?);
    writeln!(
        out,
        "repeat,native_count,tick,switch_ns,whole_ns,native_solve_ns,gpu_dispatch_ns,upload_B,read_B"
    )?;
    native_timing::timing(&mut sim, &ids, &mut out)
}
