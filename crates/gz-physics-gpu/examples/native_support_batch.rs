// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Dense 2,048-lane inclined support qualification with two persistent contact laws.
#[path = "support/native_timing.rs"]
mod native_timing;
#[allow(dead_code)]
#[path = "support/sphere_plane/reference.rs"]
mod reference;
use gz_physics_gpu::{
    BodySlot, CentralImpulse, Environment, EnvironmentState, GpuError, MixedBatch,
    NativeHandoffRefusal,
    PairSolver::{self, NativeRapier, SpecializedGpu},
    STEP, SpherePlaneLane,
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
fn sphere(e: &Environment) -> SpherePlaneLane {
    let EnvironmentState::SpherePlane(s) = e.state else {
        unreachable!()
    };
    s
}
struct Oracle {
    s: SpherePlaneLane,
    n: Option<reference::Native>,
    kick: Option<[f64; 3]>,
}
impl Oracle {
    fn owner(&self) -> PairSolver {
        if self.n.is_some() {
            NativeRapier
        } else {
            SpecializedGpu
        }
    }
    fn assign(&mut self, owner: PairSolver) {
        self.n = (owner == NativeRapier).then(|| reference::Native::with_substeps(self.s, 16));
    }
    fn step(&mut self) {
        if let Some(j) = self.kick.take() {
            self.s.velocity =
                (V::from(self.s.velocity) + V::from(j) * self.s.inverse_mass).to_array();
            if let Some(n) = &mut self.n {
                n.impulse_a(j);
            }
        }
        if let Some(n) = &mut self.n {
            n.step();
            self.s = n.state;
        } else {
            reference::analytical(&mut self.s);
        }
    }
}
fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let adapter = args.next().ok_or("provide ADAPTER NEW_DIRECTORY")?;
    let directory = args.next().ok_or("provide new measurement directory")?;
    if args.next().is_some() {
        return Err("unexpected argument".into());
    }
    std::fs::create_dir(&directory)?;
    let dir = Path::new(&directory);
    let initial = reference::fixture(2048)
        .into_iter()
        .enumerate()
        .map(|(i, s)| Environment {
            id: i as u64,
            state: EnvironmentState::SpherePlane(s),
        })
        .collect::<Vec<_>>();
    let ids = (0..96)
        .map(|i| ((i * 787 + 19) % 2048) as u64)
        .collect::<Vec<_>>();
    let marks = ids
        .iter()
        .map(|&id| (id, SpecializedGpu))
        .collect::<Vec<_>>();
    let mut controls = ids
        .iter()
        .map(|id| Oracle {
            s: sphere(&initial[*id as usize]),
            n: None,
            kick: None,
        })
        .collect::<Vec<_>>();
    let batch =
        MixedBatch::from_fixed_scene(&adapter, initial)?.with_guarded_native_pairs(&marks)?;
    let mut sim = Simulation::new(STEP)?;
    install_mixed(&mut sim, batch)?;
    let mut data = BufWriter::new(File::create_new(dir.join("accuracy.csv"))?);
    writeln!(
        data,
        "tick,moves,loaded_refusals,max_position_m,max_velocity_m_s,max_spin_rad_s,max_quat_chord,max_impulse_Ns,max_weight_residual_Ns,min_measured_loaded_warm_Ns,last_max_abs_gap_m"
    )?;
    let mut peak = [0f64; 5];
    let mut moves = 0;
    let mut refused = 0;
    let mut worst_weight = 0f64;
    let mut min_warm = f64::INFINITY;
    let mut loaded_native = 0;
    let mut loaded_gpu = 0;
    let mut warm_tangents = 0;
    let mut frictionless_native = 0;
    let mut frictionless_gpu = 0;
    for tick in 0..=800u64 {
        let mut largest_gap = 0f64;
        if tick > 0 {
            sim.step_exact(1)?;
            let batch = sim.world().resource::<MixedBatch>();
            for (i, &id) in ids.iter().enumerate() {
                controls[i].step();
                let p = sphere(&batch.environments()[id as usize]);
                let q = controls[i].s;
                let n = V::from(p.plane_normal);
                let j = n * p.normal_impulse + V::from(p.tangential_impulse);
                let expected = n * q.normal_impulse + V::from(q.tangential_impulse);
                let d = [
                    V::from(p.position).distance(q.position.into()),
                    V::from(p.velocity).distance(q.velocity.into()),
                    V::from(p.angular_velocity).distance(q.angular_velocity.into()),
                    p.orientation
                        .iter()
                        .zip(q.orientation)
                        .map(|(a, b)| (a - b).powi(2))
                        .sum::<f64>()
                        .sqrt(),
                    j.distance(expected),
                ];
                for (m, x) in peak.iter_mut().zip(d) {
                    *m = m.max(x);
                }
                assert!(
                    d.iter()
                        .zip([1e-8, 1e-8, 1e-7, 1e-8, 1e-8])
                        .all(|(v, lim)| *v < lim),
                    "tick={tick} id={id} owner={:?} independent errors {d:?} p={p:?} ref={q:?}",
                    controls[i].owner()
                );
                let gap = n.dot(V::from(p.position)) - p.plane_offset - p.radius;
                largest_gap = largest_gap.max(gap.abs());
                if (245..=285).contains(&tick) || (750..=800).contains(&tick) {
                    let w = -0.001 * n.dot(V::from(p.gravity)) / p.inverse_mass;
                    let e = (w - p.normal_impulse).abs();
                    worst_weight = worst_weight.max(e);
                    assert!(
                        e < 3e-6,
                        "id={id} t={tick} weight={w} impulse={}",
                        p.normal_impulse
                    );
                    if let Some(obs) = batch.native_contact(id)? {
                        loaded_native += 1;
                        assert!(
                            obs.active_solver_contact
                                && obs.retained_points > 0
                                && obs.warmstart_normal_impulse > 1e-7
                        );
                        min_warm = min_warm.min(obs.warmstart_normal_impulse);
                        if p.friction == 0. {
                            frictionless_native += 1;
                            assert_eq!(obs.warmstart_tangent_impulse, 0.);
                        }
                        if obs.warmstart_tangent_impulse > 1e-7 {
                            warm_tangents += 1;
                        }
                    } else {
                        loaded_gpu += 1;
                        if p.friction == 0. {
                            frictionless_gpu += 1;
                            assert!(V::from(p.tangential_impulse).length() < 1e-12);
                        }
                    }
                }
            }
        }
        let mut batch = sim.world_mut().resource_mut::<MixedBatch>();
        if tick == 300 {
            let mut actions = vec![];
            for (i, &id) in ids.iter().enumerate() {
                let s = sphere(&batch.environments()[id as usize]);
                let n = V::from(s.plane_normal);
                let j = ((n * 1.15 + V::new(0.09, -0.045, 0.)) / s.inverse_mass).to_array();
                controls[i].kick = Some(j);
                actions.push(CentralImpulse {
                    id,
                    body: BodySlot::A,
                    impulse: j,
                });
            }
            batch.queue_impulses(tick, &actions)?;
        }
        if matches!(tick, 0 | 255 | 300 | 302 | 375) {
            let mut request = [vec![], vec![]];
            for (i, &id) in ids.iter().enumerate() {
                let target = if tick == 0 {
                    if (i / 3).is_multiple_of(2) {
                        NativeRapier
                    } else {
                        continue;
                    }
                } else if controls[i].owner() == NativeRapier {
                    SpecializedGpu
                } else {
                    NativeRapier
                };
                request[usize::from(target == NativeRapier)].push((i, id));
            }
            for (list, t) in request.into_iter().zip([SpecializedGpu, NativeRapier]) {
                if list.is_empty() {
                    continue;
                }
                let before = batch.environments().to_vec();
                let pending = batch.pending_impulses().copied().collect::<Vec<_>>();
                if tick == 255 || tick == 300 {
                    for &(_, id) in &list {
                        assert!(matches!(
                            batch.set_pair_solver(tick, &[id], t),
                            Err(GpuError::NativeHandoff {
                                reason: NativeHandoffRefusal::ContactHistory,
                                ..
                            })
                        ));
                        refused += 1;
                    }
                } else {
                    let selection = list.iter().map(|v| v.1).collect::<Vec<_>>();
                    let count = batch.set_pair_solver(tick, &selection, t)?.moved;
                    assert_eq!(count, selection.len());
                    moves += count;
                    for (i, _) in list {
                        controls[i].assign(t);
                    }
                }
                for (a, b) in batch.environments().iter().zip(&before) {
                    assert!(
                        a.state.values().iter().map(|v| v.to_bits()).eq(b
                            .state
                            .values()
                            .iter()
                            .map(|v| v.to_bits()))
                    );
                }
                assert_eq!(
                    batch.pending_impulses().copied().collect::<Vec<_>>(),
                    pending
                );
            }
        }
        if tick.is_multiple_of(25) || matches!(tick, 255 | 302) {
            writeln!(
                data,
                "{tick},{moves},{refused},{},{},{},{},{},{worst_weight},{},{largest_gap}",
                peak[0],
                peak[1],
                peak[2],
                peak[3],
                peak[4],
                if min_warm.is_finite() { min_warm } else { 0. }
            )?;
        }
    }
    assert_eq!(moves, 240);
    assert_eq!(refused, 192);
    assert!(warm_tangents > 100 && frictionless_native > 100 && frictionless_gpu > 100);
    println!(
        "2048 lanes, 96 independent controls, 800 committed ticks, {moves} transfers, {refused} loaded refusals, {loaded_native} native and {loaded_gpu} specialized supported samples. Zero-friction native/GPU samples={frictionless_native}/{frictionless_gpu}; native nonzero warm tangent samples={warm_tangents} min measured warm normal={min_warm} Ns; independent maxima [position,velocity,spin,orientation,impulse]={peak:?}; max supported weight residual={worst_weight} Ns"
    );
    let mut file = BufWriter::new(File::create_new(dir.join("timing.csv"))?);
    writeln!(
        file,
        "repeat,native_count,tick,switch_ns,whole_ns,native_solve_ns,gpu_dispatch_ns,upload_B,read_B"
    )?;
    native_timing::timing(&mut sim, &ids, &mut file)
}
