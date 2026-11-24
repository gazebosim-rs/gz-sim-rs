// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Advance one continuous pool through repeated loaded transfers, faults and checkpoint restores.
#[path = "support/endurance_diagnostics.rs"]
mod diagnostics;
#[path = "support/endurance_encounters.rs"]
mod encounters;
#[path = "support/native_contact_control.rs"]
mod native;
#[path = "support/endurance_order.rs"]
mod order;
#[path = "support/changing_worlds.rs"]
mod scene;
use clap::Parser;
use gz_physics_distributed::{
    coordinator::Error,
    physics::{Body, BodyWrench, GroundPlane, SimulationStep},
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{
    error::Error as StdError,
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
    thread,
    time::{Duration, Instant},
};
type Result<T> = std::result::Result<T, Box<dyn StdError>>;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, value_enum, default_value = "separated")]
    layout: scene::Layout,
    /// Complete this many 350-tick cycles when no wall-time duration is requested.
    #[arg(long, default_value_t = 1, value_parser=clap::value_parser!(u32).range(1..))]
    cycles: u32,
    /// Run until the first complete cycle at or beyond this elapsed time.
    #[arg(long, conflicts_with="cycles", value_parser=clap::value_parser!(u32).range(1..))]
    seconds: Option<u32>,
    /// Minimum wall time from the start of one step to the start of the next, including the oracle.
    #[arg(long, default_value_t = 0)]
    pace_ms: u16,
    /// Accept changes of physical contact topology only after exact native verification.
    #[arg(long)]
    contact_transitions: bool,
    /// Refine the entire distributed scene when any contact needs a fine solve.
    #[arg(long, requires = "contact_transitions")]
    scene_wide: bool,
    /// Use the same scene-wide rate and split native worlds once their contacts separate.
    #[arg(long, requires = "scene_wide")]
    split_separated: bool,
    /// Continue past small scene-wide floating-point deviations using fixed physical bounds; retain the first difference.
    #[arg(long, requires = "scene_wide")]
    diagnose_roundoff: bool,
    /// Change spacing between assembly centers in a transition-aware endurance fixture (metres).
    #[arg(long, requires = "contact_transitions")]
    site_spacing: Option<f64>,
    /// Save both immediately preceding physical states and inputs when a comparison differs.
    #[arg(long, requires = "contact_transitions")]
    failure_context: bool,
    /// Save one frame per five ticks near the first spilled cargo and later cross-assembly encounter.
    #[arg(long, requires = "contact_transitions")]
    encounter_clip: bool,
    /// Capture every native barrier around the standard profile's first refused 33-body union.
    #[arg(long, requires = "scene_wide")]
    capacity_clip: bool,
    /// Capture owner movement around the first live transfer and native ordering near 15,291.
    #[arg(long, requires = "scene_wide")]
    reference_clip: bool,
    /// Capture both complete warm native worlds and replay this single interval without workers.
    #[arg(long, requires = "native_64", value_parser = clap::value_parser!(u64).range(2..))]
    solver_order_at: Option<u64>,
    /// Require every worker and recovery image to support up to 64 bodies in one native solver.
    #[arg(long)]
    native_64: bool,
    /// Start one serial native world in canonical ID order and migrate it as a complete unit.
    #[arg(long, requires_all = ["native_64","scene_wide"], conflicts_with="split_separated")]
    canonical_reference: bool,
    /// Include ordinary barrier samples, spawn cost and native ownership in the per-cycle log.
    #[arg(long, requires = "scene_wide")]
    placement_timings: bool,
    /// Fresh output directory. Progress is flushed after each cycle.
    out: PathBuf,
}

#[derive(Default, Serialize)]
struct Counts {
    ticks: u64,
    body_states: u64,
    pair_receipts: u64,
    ground_receipts: u64,
    migrations: u64,
    lost_acks: u64,
    replacements: u64,
    restores: u64,
    touchdowns: u64,
}
struct Witness {
    order: Option<order::Probe>,
    controls: Vec<native::Control>,
    groups: Vec<Vec<Body>>,
    pads: [u64; 2],
    owners: Vec<usize>,
    loaded: [usize; 2],
    digest: Sha256,
    counts: Counts,
    topology: Option<Topology>,
}
struct Topology {
    out: PathBuf,
    baseline_supports: Vec<u64>,
    first_change: Option<u64>,
    encounters: encounters::Encounters,
    scene_wide: bool,
    site_spacing: f64,
    split_separated: bool,
    roundoff: Option<diagnostics::Roundoff>,
    clip: Option<BufWriter<File>>,
    capacity_clip: Option<BufWriter<File>>,
    reference_clip: Option<BufWriter<File>>,
    previous: Option<diagnostics::Previous>,
}
impl Witness {
    fn step(
        &mut self,
        frame: &PoolFrame,
        loads: &[BodyWrench],
        refined: Option<&[u64]>,
        rate: u8,
    ) -> Result<()> {
        let tick = self.counts.ticks + 1;
        let coarse_before: Vec<_> = if self.topology.is_some() {
            self.controls
                .iter()
                .flat_map(|c| &c.bodies)
                .filter(|b| refined.is_some_and(|ids| ids.binary_search(&b.id).is_err()))
                .cloned()
                .collect()
        } else {
            Vec::new()
        };
        let mut expected_pairs = vec![];
        let mut expected_floors = vec![];
        for control in &mut self.controls {
            let field: Vec<_> = loads
                .iter()
                .filter(|w| control.bodies.binary_search_by_key(&w.id, |b| b.id).is_ok())
                .cloned()
                .collect();
            let (pairs, floors) = control.advance(10, &field, &[])?;
            expected_pairs.extend(pairs);
            expected_floors.extend(floors);
        }
        let mut bodies = vec![];
        let mut pairs = vec![];
        let mut floors = vec![];
        for w in &frame.workers {
            bodies.extend(&w.snapshot.bodies);
            pairs.extend(&w.snapshot.impulses);
            floors.extend(&w.snapshot.supports);
        }
        bodies.sort_by_key(|b: &&Body| b.id);
        pairs.sort_by_key(|p: &&gz_physics_distributed::physics::ContactImpulse| (p.a, p.b));
        floors.sort_by_key(|p: &&gz_physics_distributed::physics::GroundSupport| p.body);
        let expected: Vec<_> = self.controls.iter().flat_map(|c| &c.bodies).collect();
        let state = bincode::serialize(&(tick, &bodies, &pairs, &floors))?;
        if frame.tick != tick {
            return Err(format!("incorrect barrier clock at tick {tick}").into());
        }
        let expected_state =
            bincode::serialize(&(tick, &expected, &expected_pairs, &expected_floors))?;
        if self.order.as_ref().is_some_and(|p| p.tick() == tick) {
            self.order
                .take()
                .unwrap()
                .compare(loads, rate, &state, &expected_state)?;
        }
        if state != expected_state {
            let mut accepted = false;
            if let Some(topology) = &mut self.topology {
                let first = topology
                    .roundoff
                    .as_ref()
                    .is_none_or(|r| r.first_tick.is_none());
                if let Some(r) = &mut topology.roundoff {
                    accepted = r.observe(
                        tick,
                        &bodies,
                        &expected,
                        &pairs,
                        &expected_pairs,
                        &floors,
                        &expected_floors,
                    );
                    if first || !accepted || tick.is_multiple_of(scene::END) {
                        fs::write(
                            topology.out.join("roundoff.json"),
                            serde_json::to_vec_pretty(r)?,
                        )?;
                    }
                }
                if first || !accepted {
                    let name = if first {
                        "native-mismatch.json"
                    } else {
                        "threshold-mismatch.json"
                    };
                    if let Some(previous) = &topology.previous {
                        fs::write(
                            topology.out.join(format!("previous-{name}")),
                            serde_json::to_vec_pretty(previous)?,
                        )?;
                        fs::write(
                            topology.out.join(format!("current-{name}")),
                            serde_json::to_vec_pretty(&serde_json::json!({
                                "tick":tick, "substeps":rate, "held_wrenches":loads,
                                "actual_bodies":bodies, "reference_bodies":expected,
                                "actual_pairs":pairs, "reference_pairs":expected_pairs,
                                "actual_floors":floors, "reference_floors":expected_floors,
                            }))?,
                        )?;
                    }
                    fs::write(
                        topology.out.join(name),
                        serde_json::to_vec_pretty(
                            &serde_json::json!({"tick": tick, "mismatched_bodies": bodies.iter().zip(&expected)
                    .filter(|(a,b)| a != b).map(|(a,b)| serde_json::json!({"distributed": a, "native": b})).collect::<Vec<_>>(),
                    "metrics": diagnostics::comparison(&bodies, &expected),
                    "isolated_freeflight": diagnostics::isolated_solve_check(&coarse_before,frame,&expected,loads)?,
                    "exact_through_tick": topology.roundoff.as_ref().and_then(|r|r.first_tick).unwrap_or(tick)-1,
                    "fine_ids": refined, "applied_wrenches": loads,
                    "distributed_pairs": pairs, "control_pairs": expected_pairs, "distributed_floor": floors,
                    "control_floor": expected_floors, "components": frame.workers.iter().map(|w| &w.components).collect::<Vec<_>>()}),
                        )?,
                    )?;
                }
            }
            if !accepted {
                return Err(format!("native state or contact mismatch at tick {tick}").into());
            }
        }
        if self.topology.is_none() {
            for (group, &owner) in self.groups.iter().zip(&self.owners) {
                for b in group {
                    if frame.body(b.id).map(|x| usize::from(x.0)) != Some(owner) {
                        return Err(format!("wrong owner for {} at tick {tick}", b.id).into());
                    }
                }
            }
        }
        if let Some(clip) = self.topology.as_mut().and_then(|t| t.clip.as_mut())
            && encounters::clip_tick(tick)
        {
            let owners: Vec<_> = bodies.iter().map(|b| frame.body(b.id).unwrap().0).collect();
            serde_json::to_writer(
                &mut *clip,
                &serde_json::json!({"tick":tick,"bodies":bodies,
                "owners":owners,"pairs":pairs,"floors":floors}),
            )?;
            writeln!(clip)?;
            clip.flush()?;
        }
        let physical_clip = self.topology.as_mut().and_then(|topology| {
            if (11_035..=11_115).contains(&tick) {
                topology.capacity_clip.as_mut()
            } else if (125..=170).contains(&tick) || (15_265..=15_315).contains(&tick) {
                topology.reference_clip.as_mut()
            } else {
                None
            }
        });
        if let Some(clip) = physical_clip {
            let owners: Vec<_> = bodies.iter().map(|b| frame.body(b.id).unwrap().0).collect();
            let components: Vec<_> = frame.workers.iter().map(|w| &w.components).collect();
            serde_json::to_writer(
                &mut *clip,
                &serde_json::json!({"tick":tick,"bodies":bodies,
                "owners":owners,"pairs":pairs,"floors":floors,"components":components,
                "trajectory_sha256_sample":format!("{:x}",Sha256::digest(&state)),
                "control_sha256_sample":format!("{:x}",Sha256::digest(&expected_state)),
                "generations":bodies.iter().zip(&owners).map(|(b,&o)|frame.workers[o as usize].generations[&b.id]).collect::<Vec<_>>()}),
            )?;
            writeln!(clip)?;
            clip.flush()?;
        }
        let loaded = self.pads.map(|id| {
            pairs
                .iter()
                .filter(|p| p.a == id && p.normal_impulse > 0.)
                .count()
        });
        if tick > scene::SETTLED {
            self.counts.touchdowns += self
                .loaded
                .iter()
                .zip(loaded)
                .filter(|(old, n)| **old == 0 && *n == 24)
                .count() as u64;
            if let Some(topology) = &mut self.topology {
                let ids: Vec<_> = floors
                    .iter()
                    .filter(|p| p.normal_impulse > 0.)
                    .map(|f| f.body)
                    .collect();
                if topology.baseline_supports.is_empty() {
                    topology.baseline_supports.clone_from(&ids);
                }
                topology
                    .encounters
                    .observe(tick, &topology.baseline_supports, &pairs, &floors);
                if ids != topology.baseline_supports && topology.first_change.is_none() {
                    let changed: Vec<_> = self
                        .groups
                        .iter()
                        .filter(|g| {
                            g.iter().any(|b| {
                                ids.contains(&b.id) != topology.baseline_supports.contains(&b.id)
                            })
                        })
                        .flat_map(|group| group.iter().map(|b| b.id))
                        .collect();
                    fs::write(
                        topology.out.join("first-support-change.json"),
                        serde_json::to_vec_pretty(&serde_json::json!({
                            "tick":tick,"dt_s":0.01,"baseline_positive_support_ids":topology.baseline_supports,
                            "positive_support_ids":ids,"wrenches":loads,"affected_original_assembly_ids":changed,
                            "distributed_bodies":bodies.iter().filter(|b| changed.contains(&b.id)).collect::<Vec<_>>(),
                            "control_bodies":expected.iter().filter(|b| changed.contains(&b.id)).collect::<Vec<_>>(),
                            "distributed_supports":floors,"control_supports":expected_floors,
                            "distributed_pair_impulses":pairs.iter().filter(|p| changed.contains(&p.a)||changed.contains(&p.b)).collect::<Vec<_>>(),
                            "control_pair_impulses":expected_pairs.iter().filter(|p| changed.contains(&p.a)||changed.contains(&p.b)).collect::<Vec<_>>(),
                        }))?,
                    )?;
                    topology.first_change = Some(tick);
                }
            } else if floors.iter().filter(|p| p.normal_impulse > 0.).count() != self.groups.len() {
                return Err(format!("lost support at tick {tick}").into());
            }
        }
        self.loaded = loaded;
        if let Some(previous) = self.topology.as_mut().and_then(|t| t.previous.as_mut()) {
            previous.remember(
                tick,
                frame,
                loads,
                &expected,
                &expected_pairs,
                &expected_floors,
            );
        }
        self.digest.update(state);
        self.counts.ticks = tick;
        self.counts.body_states += bodies.len() as u64;
        self.counts.pair_receipts += pairs.len() as u64;
        self.counts.ground_receipts += floors.len() as u64;
        Ok(())
    }
}

fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.out)?;
    let ground = Some(GroundPlane::new(0.)?);
    let mut groups = scene::seeds(args.layout);
    let site_spacing = args.site_spacing.unwrap_or(13.);
    if !site_spacing.is_finite() || site_spacing < 13. {
        return Err("site spacing must be finite and at least the original 13 metres".into());
    }
    if site_spacing != 13. {
        for group in &mut groups {
            let center = group[0].position;
            for b in group {
                for (axis, old) in center.into_iter().enumerate().take(2) {
                    b.position[axis] += old * (site_spacing / 13. - 1.);
                }
            }
        }
    }
    let placement = scene::Placement::Split;
    let loading = Instant::now();
    let mut pool = if args.canonical_reference {
        WorkerPool::spawn_canonical_scene(
            &args.worker,
            0x656e6475,
            groups.concat(),
            4,
            SimulationStep::Ms10,
            ground,
            256,
        )?
    } else {
        WorkerPool::spawn_with_profile(
            &args.worker,
            0x656e6475,
            placement.worker_seeds(&groups),
            SimulationStep::Ms10,
            ground,
            256,
            if args.native_64 {
                gz_physics_distributed::physics::NativeSolverProfile::Extended64
            } else {
                Default::default()
            },
        )?
    };
    let startup_us = loading.elapsed().as_secs_f64() * 1e6;
    let pads: Vec<_> = groups
        .iter()
        .enumerate()
        .filter(|(_, g)| g.len() == 25)
        .map(|(i, g)| (i, g[0].id))
        .collect();
    let [(migrant, pad), (_, other)] = pads[..] else {
        return Err("expected two cargo pads".into());
    };
    let encounters = encounters::Encounters::new(&groups);
    if args.solver_order_at.is_some() && !args.scene_wide {
        return Err("native order replay requires scene-wide refinement".into());
    }
    let mut oracle = Witness {
        order: None,
        controls: if args.contact_transitions {
            // One independently constructed solver admits later contact between any
            // two original assemblies, including cargo that crosses to another pad.
            vec![native::Control::new(
                &groups.concat(),
                ground,
                scene::GRAVITY,
            )?]
        } else {
            groups
                .iter()
                .map(|g| native::Control::new(g, ground, scene::GRAVITY))
                .collect::<Result<_>>()?
        },
        owners: placement.owners(&groups),
        groups,
        pads: [pad, other],
        loaded: [0; 2],
        digest: Sha256::new(),
        counts: Counts::default(),
        topology: if args.contact_transitions {
            Some(Topology {
                out: args.out.clone(),
                baseline_supports: vec![],
                first_change: None,
                encounters,
                scene_wide: args.scene_wide,
                split_separated: args.split_separated,
                roundoff: args.diagnose_roundoff.then(diagnostics::Roundoff::default),
                site_spacing,
                previous: args.failure_context.then(diagnostics::Previous::default),
                reference_clip: if args.reference_clip {
                    Some(BufWriter::new(File::create_new(
                        args.out.join("reference.jsonl"),
                    )?))
                } else {
                    None
                },
                capacity_clip: if args.capacity_clip {
                    Some(BufWriter::new(File::create_new(
                        args.out.join("capacity.jsonl"),
                    )?))
                } else {
                    None
                },
                clip: if args.encounter_clip {
                    Some(BufWriter::new(File::create_new(
                        args.out.join("encounter.jsonl"),
                    )?))
                } else {
                    None
                },
            })
        } else {
            None
        },
    };
    let mut progress = BufWriter::new(File::create_new(args.out.join("cycles.jsonl"))?);
    let scratch = tempfile::tempdir_in(&args.out)?;
    let begin = Instant::now();
    let pace = Duration::from_millis(u64::from(args.pace_ms));
    for cycle in 1..=u32::MAX {
        let before_landings = oracle.counts.touchdowns;
        let mut elapsed = Vec::with_capacity(scene::END as usize);
        let mut retry_us = 0.;
        let mut transfer_us = 0.;
        let mut transfer_wire_bytes = 0;
        let mut transfer_members = 0;
        let mut retry_restarted_workers = 0;
        let mut regular = Vec::with_capacity(scene::END as usize - 1);
        for step in 1..=scene::END {
            let started = Instant::now();
            let field = scene::wrenches(step, &oracle.groups);
            let owner = pool.frame().body(pad).ok_or("missing migrant pad")?.0;
            let advance_field = |pool: &mut WorkerPool| {
                if args.split_separated {
                    pool.advance_adaptive_with_separation(scene::GRAVITY, &field)?;
                } else if args.scene_wide {
                    pool.advance_adaptive_in_field(scene::GRAVITY, &field)?;
                } else {
                    pool.advance_islands_in_field(scene::GRAVITY, &field)?;
                }
                Ok::<(), Error>(())
            };
            let pids = |pool: &WorkerPool| (0..4).map(|w| pool.worker_pid(w)).collect::<Vec<_>>();
            let before_pids = (step == 160).then(|| pids(&pool));
            if step == 160 {
                let committed = pool.frame().clone();
                pool.drop_next_acknowledgement(owner)?;
                if !matches!(advance_field(&mut pool), Err(Error::Transport(_)))
                    || pool.frame() != &committed
                {
                    return Err("lost acknowledgement changed the published barrier".into());
                }
                oracle.counts.lost_acks += 1;
            }
            let advance = Instant::now();
            if let Err(error) = advance_field(&mut pool) {
                if args.contact_transitions {
                    diagnostics::admission(
                        &args.out,
                        pool.frame(),
                        &field,
                        "advance",
                        pool.native_solver_profile(),
                        &error,
                    )?;
                }
                return Err(error.into());
            }
            let us = advance.elapsed().as_secs_f64() * 1e6;
            elapsed.push(us);
            if let Some(before) = before_pids {
                retry_us = us;
                retry_restarted_workers = before
                    .into_iter()
                    .zip(pids(&pool))
                    .filter(|(a, b)| a != b)
                    .count();
            } else {
                regular.push(us);
            }
            if step == 145 {
                let wire = |pool: &WorkerPool| {
                    pool.traffic()
                        .iter()
                        .map(|t| t.request_bytes + t.reply_bytes)
                        .sum::<u64>()
                };
                let before = args.placement_timings.then(|| wire(&pool));
                let target = 1 - owner;
                if args.contact_transitions {
                    let committed = pool.frame().clone();
                    let component = committed.workers[owner as usize]
                        .components
                        .iter()
                        .find(|g| g.ids.binary_search(&pad).is_ok())
                        .ok_or("missing migrating component")?;
                    transfer_members = component.ids.len();
                    let transfer_start = Instant::now();
                    let result = pool.transfer_component(pad, target);
                    transfer_us = transfer_start.elapsed().as_secs_f64() * 1e6;
                    if let Err(error) = result {
                        diagnostics::admission(
                            &args.out,
                            pool.frame(),
                            &[],
                            "explicit handoff",
                            pool.native_solver_profile(),
                            &error,
                        )?;
                        return Err(error.into());
                    }
                    let moved = pool.frame();
                    for (source, observation) in committed.workers.iter().enumerate() {
                        for (id, old_generation) in &observation.generations {
                            let (worker, body) = moved.body(*id).ok_or("transfer lost body")?;
                            let member = component.ids.binary_search(id).is_ok();
                            if worker != (if member { target } else { source as u8 })
                                || moved.workers[worker as usize].generations[id]
                                    != old_generation + u64::from(member)
                                || Some(body) != committed.body(*id).map(|(_, b)| b)
                            {
                                return Err(format!("incorrect live handoff of {id}").into());
                            }
                        }
                    }
                } else {
                    let transfer_start = Instant::now();
                    pool.transfer_component(pad, target)?;
                    transfer_us = transfer_start.elapsed().as_secs_f64() * 1e6;
                }
                if let Some(before) = before {
                    transfer_wire_bytes = wire(&pool) - before;
                }
                oracle.owners[migrant] = usize::from(target);
                oracle.counts.migrations += 1;
            }
            if step == 219 && cycle % 8 == 1 {
                let committed = pool.frame().clone();
                let prior = pool.worker_pid(owner);
                pool.terminate_worker(owner)?;
                pool.recover(owner)?;
                if pool.frame() != &committed || pool.worker_pid(owner) == prior {
                    return Err("replacement changed the barrier or reused the old process".into());
                }
                oracle.counts.replacements += 1;
            }
            if step == 225 && cycle % 32 == 1 {
                let committed = pool.frame().clone();
                let file = scratch.path().join("pool.gzpool");
                pool.save_native_recovery(&file)?;
                drop(pool);
                pool = PoolNativeRecovery::open(&file)?.resume(&args.worker)?;
                if pool.frame() != &committed {
                    return Err("restore changed committed pool frame".into());
                }
                fs::rename(file, args.out.join("latest.gzpool"))?;
                oracle.counts.restores += 1;
            }
            oracle.step(
                pool.frame(),
                &field,
                pool.last_island_refined_bodies(),
                pool.last_substeps().unwrap(),
            )?;
            if args.solver_order_at == Some(pool.frame().tick + 1) {
                oracle.order = Some(order::Probe::capture(
                    &args.out,
                    &mut pool,
                    &mut oracle.controls[0],
                )?);
            }
            if !args.contact_transitions && step == 145 && oracle.loaded[0] != 24 {
                return Err(format!("migrant pad lost load on cycle {cycle}").into());
            }
            if let Some(remaining) = pace.checked_sub(started.elapsed()) {
                thread::sleep(remaining);
            }
        }
        if !args.contact_transitions && oracle.counts.touchdowns - before_landings != 6 {
            return Err(format!(
                "cycle {cycle} had incorrect landing count: {:?}",
                oracle.loaded
            )
            .into());
        }
        regular.sort_by(f64::total_cmp);
        elapsed.sort_by(f64::total_cmp);
        let mut record = serde_json::json!({"cycle":cycle,"elapsed_s":begin.elapsed().as_secs_f64(),"counts":oracle.counts,
            "retry_us":retry_us, "retry_restarted_workers":retry_restarted_workers,
            "regular_p95_us":regular[(regular.len()*95).div_ceil(100)-1],
            "regular_total_us":regular.iter().sum::<f64>(), "tick_p95_us":elapsed[(elapsed.len()*95).div_ceil(100)-1], "tick_max_us":elapsed.last(),
            "trajectory_sha256":format!("{:x}",oracle.digest.clone().finalize())});
        if args.canonical_reference {
            record["canonical_reference"] = true.into();
        }
        if args.placement_timings {
            record["spawn_us"] = startup_us.into();
            record["transfer_us"] = transfer_us.into();
            record["transfer_wire_bytes"] = transfer_wire_bytes.into();
            record["transfer_members"] = transfer_members.into();
            record["regular_samples_us"] = serde_json::to_value(&regular)?;
            record["resident_component_sizes"] = serde_json::to_value(
                pool.frame()
                    .workers
                    .iter()
                    .map(|o| o.components.iter().map(|c| c.ids.len()).collect::<Vec<_>>())
                    .collect::<Vec<_>>(),
            )?;
        }
        if args.native_64 {
            record["native_solver_capacity"] = pool.native_solver_profile().max_bodies().into();
        }
        if let Some(t) = &oracle.topology {
            record["independent_native_worlds"] = 1.into();
            record["first_support_change"] = t.first_change.into();
            record["scene_wide_refinement"] = t.scene_wide.into();
            record["split_separated"] = t.split_separated.into();
            if let Some(r) = &t.roundoff {
                record["roundoff"] = serde_json::to_value(r)?;
            }
            record["site_spacing_m"] = t.site_spacing.into();
            record["encounters"] = serde_json::to_value(&t.encounters)?;
        }
        serde_json::to_writer(&mut progress, &record)?;
        writeln!(progress)?;
        progress.flush()?;
        if args.seconds.map_or(cycle >= args.cycles, |s| {
            begin.elapsed() >= Duration::from_secs(u64::from(s))
        }) {
            fs::write(
                args.out.join("result.json"),
                serde_json::to_vec_pretty(&record)?,
            )?;
            return Ok(());
        }
    }
    Err("exhausted cycle counter".into())
}
