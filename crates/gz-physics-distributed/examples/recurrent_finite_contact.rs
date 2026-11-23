// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Long-lived positive finite support and cyclic lift-off/landing under prescribed forces.
#[path = "support/recurrent_activity.rs"]
mod activity;
#[path = "support/native_contact_control.rs"]
mod native;
use activity::{Cycle, DT, GRAVITY, LIFT, PERIOD, WARMUP, Windows};
use clap::Parser;
use gz_physics_distributed::{
    coordinator::Error as PoolError,
    physics::{Body, BodyWrench, ContactImpulse, GroundSupport, Primitive, SimulationStep, Tier},
    pool::{PoolFrame, PoolNativeRecovery, WorkerPool},
};
use serde::Serialize;
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// Number of one-second driven cycles following one second of settling.
    #[arg(long,default_value_t=10_000,value_parser=clap::value_parser!(u64).range(1..=100_000))]
    cycles: u64,
    /// Keep both pairs in a single sorted native reference solver on one owner.
    #[arg(long)]
    canonical_reference: bool,
    /// Seed both stations on one worker, then enable measured whole-island placement.
    #[arg(long, conflicts_with = "canonical_reference")]
    exercise_placement: bool,
    /// Fresh evidence directory. Every 100 driven cycles flush a compact progress report.
    out: PathBuf,
}
fn station(base: u64, x: f64) -> [Body; 2] {
    let table = Body {
        id: base,
        primitive: Primitive::Box,
        tier: Tier::Fixed,
        mass: 0.,
        friction: 0.,
        size: [2.6, 2.6, 0.4],
        position: [x, 0., 0.7],
        orientation: [0., 0., 0., 1.],
        linear: [0.; 3],
        angular: [0.; 3],
    };
    let ball = Body {
        id: base + 1,
        primitive: Primitive::Sphere,
        tier: Tier::Dynamic,
        mass: 1.,
        size: [0.4; 3],
        position: [x, 0., 1.15],
        ..table.clone()
    };
    [table, ball]
}
#[derive(Default, Serialize)]
struct Counts {
    ticks: u64,
    body_states: u64,
    finite_receipts: u64,
    positive_finite_receipts: u64,
    landings: u64,
    migrations: u64,
    lost_acks: u64,
    replacements: u64,
    restores: u64,
}
#[derive(Default, Serialize, PartialEq)]
struct Physical {
    bodies: Vec<Body>,
    pairs: Vec<ContactImpulse>,
    floors: Vec<GroundSupport>,
}
impl Physical {
    fn read(frame: &PoolFrame) -> Self {
        let mut obs = Self::default();
        for worker in &frame.workers {
            obs.bodies.extend_from_slice(&worker.snapshot.bodies);
            obs.pairs.extend_from_slice(&worker.snapshot.impulses);
            obs.floors.extend_from_slice(&worker.snapshot.supports);
        }
        obs.bodies.sort_by_key(|b| b.id);
        obs.pairs.sort_by_key(|p| (p.a, p.b));
        obs.floors.sort_by_key(|f| f.body);
        obs
    }
    fn load(&self, table: u64) -> Option<f64> {
        self.pairs
            .iter()
            .find(|p| (p.a, p.b) == (table, table + 1))
            .map(|p| p.normal_impulse)
    }
    fn vz(&self) -> [f64; 2] {
        [self.bodies[1].linear[2], self.bodies[3].linear[2]]
    }
}
fn archive(path: impl AsRef<Path>, data: &impl Serialize) -> Result<()> {
    fs::write(path, serde_json::to_vec_pretty(data)?)?;
    Ok(())
}
fn events_at(tick: u64, kind: &str, extra: Value) -> Value {
    json!({"tick":tick,"operation":kind,"details":extra})
}
fn late_frame(pool: &WorkerPool, p: &Physical, forces: &[BodyWrench], state_hash: &[u8]) -> Value {
    let frame = pool.frame();
    let ownership: Vec<_> = p
        .bodies
        .iter()
        .map(|b| frame.body(b.id).unwrap().0)
        .collect();
    let generations: Vec<_> = ownership
        .iter()
        .zip(&p.bodies)
        .map(|(&owner, b)| frame.workers[owner as usize].generations[&b.id])
        .collect();
    json!({"tick":frame.tick,"bodies":p.bodies,"pairs":p.pairs,"floors":p.floors,
        "owners":ownership,"generations":generations,"components":frame.workers.iter().map(|w|&w.components).collect::<Vec<_>>(),
        "substeps":pool.last_substeps(),"wrenches":forces,"trajectory_sha256_sample":format!("{:x}",Sha256::digest(state_hash))})
}
#[cfg_attr(test, allow(dead_code))]
fn main() -> Result<()> {
    let args = Args::parse();
    qualify(
        &args.worker,
        args.cycles,
        args.canonical_reference,
        args.exercise_placement,
        &args.out,
    )
}

pub(crate) fn qualify(
    worker: &Path,
    cycles: u64,
    canonical_reference: bool,
    exercise_placement: bool,
    out: &Path,
) -> Result<()> {
    fs::create_dir(out)?;
    let scratch = tempfile::tempdir_in(out)?;
    let a = station(1, -15.);
    let b = station(3, 15.);
    let all = [a.as_slice(), b.as_slice()].concat();
    let mut control = native::Control::new(&all, None, [0., 0., GRAVITY])?;
    let mut pool = if canonical_reference {
        WorkerPool::spawn_canonical_scene(
            worker,
            0x5246,
            all.clone(),
            4,
            SimulationStep::Ms10,
            None,
            1024,
        )?
    } else {
        WorkerPool::spawn(
            worker,
            0x5246,
            if exercise_placement {
                vec![all.clone(), vec![], vec![], vec![]]
            } else {
                vec![a.to_vec(), b.to_vec(), vec![], vec![]]
            },
            SimulationStep::Ms10,
            None,
            1024,
        )?
    };
    let configure = |pool: &mut WorkerPool| -> Result<()> {
        pool.set_packed_handoffs(true);
        if exercise_placement {
            pool.set_local_helper_limit(Some(0));
            pool.set_periodic_placement(Some("3,10,24,0,4".parse()?))?;
        }
        Ok(())
    };
    configure(&mut pool)?;
    let mut placement_log = exercise_placement
        .then(|| File::create_new(out.join("placement.jsonl")))
        .transpose()?
        .map(BufWriter::new);
    let mut total_moves = 0u64;
    let end_tick = WARMUP + PERIOD * cycles;
    let clip_begin = end_tick.saturating_sub(2 * PERIOD);
    let mut progress = BufWriter::new(File::create_new(out.join("windows.jsonl"))?);
    let mut cycles_file = BufWriter::new(File::create_new(out.join("cycles.csv"))?);
    writeln!(
        cycles_file,
        "cycle,end_tick,stable_positive,driven_positive,driven_absent,release_phase,landing_phase,landing_ns,max_clearance_m,stable_contact_ns,driven_contact_ns,drive_ns,stable_momentum_error_ns,driven_momentum_error_ns"
    )?;
    let mut clip = BufWriter::new(File::create_new(out.join("frames.jsonl"))?);
    let mut counts = Counts::default();
    let mut windows = Windows::default();
    let mut cycle = Cycle::default();
    let mut before = Physical::read(pool.frame());
    if before.bodies != control.bodies || !before.pairs.is_empty() || !before.floors.is_empty() {
        return Err("initial distributed state differs from the direct native seed".into());
    }
    let mut before_velocity = before.vz();
    let mut digest = Sha256::new();
    let mut input_digest = Sha256::new();
    let mut late_events = vec![];
    let start = Instant::now();
    for tick in 1..=end_tick {
        let driven_tick = tick.saturating_sub(WARMUP);
        let epoch = driven_tick.div_ceil(PERIOD);
        let phase = if driven_tick == 0 {
            0
        } else {
            (driven_tick - 1) % PERIOD + 1
        };
        let lift = if (1..=10).contains(&phase) { LIFT } else { 0. };
        let input = (lift != 0.).then_some(BodyWrench {
            id: 4,
            force: [0., 0., lift],
            torque: [0.; 3],
        });
        let wrenches = input.as_slice();
        let owner = pool.frame().body(4).ok_or("missing driven sphere")?.0;
        let ownership = |pool: &WorkerPool| {
            [1, 2, 3, 4].map(|id| {
                let frame = pool.frame();
                let w = frame.body(id).unwrap().0;
                (w, frame.workers[usize::from(w)].generations[&id])
            })
        };
        let expected = exercise_placement.then(|| ownership(&pool));
        let mut events = vec![];
        if phase == 1 {
            cycle = Cycle {
                cycle: epoch,
                ..Default::default()
            };
            before_velocity = before.vz();
        }
        if phase == 45 && epoch.is_multiple_of(4) {
            let old = pool.frame().clone();
            let pid = pool.worker_pid(owner);
            pool.drop_next_acknowledgement(owner)?;
            if !matches!(
                pool.advance_adaptive_in_field([0., 0., GRAVITY], wrenches),
                Err(PoolError::Transport(_))
            ) || pool.frame() != &old
            {
                return Err("ambiguous step changed the published barrier".into());
            }
            pool.advance_adaptive_in_field([0., 0., GRAVITY], wrenches)?;
            if pool.worker_pid(owner) != pid {
                return Err("a healthy worker restarted after a lost reply".into());
            }
            counts.lost_acks += 1;
            events.push(events_at(
                tick,
                "lost acknowledgment and cached retry",
                json!({"owner":owner}),
            ));
        } else {
            pool.advance_adaptive_in_field([0., 0., GRAVITY], wrenches)?;
        }
        if let (Some(log), Some(prior), Some((commit, decision))) = (
            &mut placement_log,
            expected,
            pool.placement_status().and_then(|s| s.last_move.as_ref()),
        ) && *commit == tick
        {
            for (i, ((before, generation), (now, next))) in
                prior.into_iter().zip(ownership(&pool)).enumerate()
            {
                if decision.members.contains(&(i as u64 + 1)) {
                    if before != decision.source
                        || now != decision.destination
                        || next != generation + 1
                    {
                        return Err(format!("invalid automatic transfer at {tick}").into());
                    }
                } else if (now, next) != (before, generation) {
                    return Err(format!("automatic move changed a nonmember at {tick}").into());
                }
            }
            let event = events_at(
                tick,
                "automatic whole-component placement",
                json!({
                    "members":decision.members,"source":decision.source,"destination":decision.destination,
                    "commit_owners_and_generations":ownership(&pool)
                }),
            );
            serde_json::to_writer(&mut *log, &event)?;
            writeln!(log)?;
            events.push(event);
            total_moves += 1;
        }
        if phase == 70 && epoch.is_multiple_of(4) {
            let owner = pool
                .frame()
                .body(4)
                .ok_or("missing driven sphere after advance")?
                .0;
            let old = pool.frame().clone();
            let moved = old.workers[owner as usize]
                .components
                .iter()
                .find(|c| c.ids.contains(&4))
                .ok_or("missing native component")?;
            let destination = (owner + 1) % 4;
            let expected_size = if canonical_reference { 4 } else { 2 };
            if moved.ids.len() != expected_size
                || !old.workers[owner as usize]
                    .snapshot
                    .impulses
                    .iter()
                    .any(|p| (p.a, p.b) == (3, 4) && p.normal_impulse > 0.)
            {
                archive(
                    out.join("migration-precondition.json"),
                    &json!({
                        "tick":tick, "owner":owner, "expected_members":expected_size, "actual_members":moved.ids,
                        "workers":old.workers.iter().map(|w|json!({"components":w.components,"pairs":w.snapshot.impulses})).collect::<Vec<_>>()
                    }),
                )?;
                return Err(format!(
                    "attempted migration without the expected loaded finite pair at {tick}"
                )
                .into());
            }
            pool.transfer_component(4, destination)?;
            for worker in &old.workers {
                for b in &worker.snapshot.bodies {
                    let (prior, old_body) = old.body(b.id).unwrap();
                    let (now, new_body) = pool.frame().body(b.id).ok_or("missing migrated body")?;
                    let member = moved.ids.contains(&b.id);
                    if (if member { destination } else { prior }) != now
                        || old_body != new_body
                        || pool.frame().workers[now as usize].generations[&b.id]
                            != worker.generations[&b.id] + u64::from(member)
                    {
                        return Err(
                            "warm move changed a state or the wrong ownership generation".into(),
                        );
                    }
                }
            }
            if Physical::read(&old) != Physical::read(pool.frame()) {
                return Err("warm move changed a physical receipt".into());
            }
            counts.migrations += 1;
            events.push(events_at(
                tick,
                "loaded whole-component migration",
                json!({"source":owner,"destination":destination,"ids":moved.ids}),
            ));
        }
        if phase == 75 && epoch.is_multiple_of(20) {
            let owner = pool.frame().body(4).unwrap().0;
            let old = pool.frame().clone();
            let old_pid = pool.worker_pid(owner);
            pool.terminate_worker(owner)?;
            pool.recover(owner)?;
            if pool.frame() != &old || pool.worker_pid(owner) == old_pid {
                return Err("worker replacement changed the barrier or reused the process".into());
            }
            counts.replacements += 1;
            events.push(events_at(
                tick,
                "terminated and recovered loaded owner",
                json!({"owner":owner,"before_pid":old_pid,"after_pid":pool.worker_pid(owner)}),
            ));
        }
        if phase == 80 && epoch.is_multiple_of(100) {
            let old = pool.frame().clone();
            let path = scratch.path().join("checkpoint.gzpool");
            pool.save_native_recovery(&path)?;
            if let (Some(log), Some(status)) = (&mut placement_log, pool.placement_status()) {
                write_sampler_epoch(log, tick, status)?;
            }
            drop(pool);
            pool = PoolNativeRecovery::open(&path)?.resume(worker)?;
            configure(&mut pool)?;
            if pool.frame() != &old {
                return Err("disk reload changed the public warm barrier".into());
            }
            counts.restores += 1;
            // Native saves create a fresh path. Publish the verified archive
            // and leave the scratch name free for the next durable recovery.
            fs::rename(path, out.join("last-checkpoint.gzpool"))?;
            events.push(events_at(
                tick,
                "full native pool reloaded",
                json!({"workers":4}),
            ));
        }
        let rate = pool.last_substeps().ok_or("missing native solve rate")?;
        if phase != 0 && rate != 10 {
            return Err(format!("lost fine contact solve at {tick}").into());
        }
        let (pairs, floors) = control.advance(rate, wrenches, &[])?;
        let actual = Physical::read(pool.frame());
        let state = bincode::serialize(&(tick, &actual.bodies, &actual.pairs, &actual.floors))?;
        if tick != pool.frame().tick
            || actual.bodies != control.bodies
            || actual.pairs != pairs
            || actual.floors != floors
        {
            archive(
                out.join("native-mismatch.json"),
                &json!({"tick":tick,"prior":before,"distributed":actual,"native_bodies":control.bodies,"native_pairs":pairs,"native_floors":floors,"wrenches":wrenches,"events":events}),
            )?;
            return Err(format!("native trajectory differs at {tick}").into());
        }
        if !actual.floors.is_empty()
            || actual
                .pairs
                .iter()
                .any(|p| !matches!((p.a, p.b), (1, 2) | (3, 4)))
        {
            return Err(format!("unexpected physical contact at tick {tick}").into());
        }
        if phase != 0 {
            cycle.observe(
                phase,
                actual.load(1),
                actual.load(3),
                lift,
                actual.bodies[3].position[2] - 1.1,
            );
        }
        counts.ticks = tick;
        counts.body_states += actual.bodies.len() as u64;
        counts.finite_receipts += actual.pairs.len() as u64;
        counts.positive_finite_receipts += actual
            .pairs
            .iter()
            .filter(|p| p.normal_impulse > 0.)
            .count() as u64;
        digest.update(&state);
        input_digest.update(bincode::serialize(&(tick, rate, wrenches))?);
        if tick >= clip_begin {
            serde_json::to_writer(&mut clip, &late_frame(&pool, &actual, wrenches, &state))?;
            writeln!(clip)?;
            late_events.extend(events);
        }
        if phase == PERIOD {
            if let Err(error) = cycle
                .complete(before_velocity, actual.vz())
                .and_then(|_| windows.observe(&cycle))
            {
                archive(
                    out.join("activity-failure.json"),
                    &json!({"tick":tick,"cycle":cycle,"window":windows,"body_states":actual,"reason":error.to_string()}),
                )?;
                return Err(error);
            }
            counts.landings += 1;
            writeln!(
                cycles_file,
                "{epoch},{tick},{},{},{},{},{},{},{},{},{},{},{},{}",
                cycle.stable_positive,
                cycle.driven_positive,
                cycle.driven_absent,
                cycle.first_absent_phase.unwrap(),
                cycle.landing_phase.unwrap(),
                cycle.landing_impulse_n_s,
                cycle.max_clearance_m,
                cycle.stable_contact_n_s,
                cycle.driven_contact_n_s,
                cycle.external_drive_n_s,
                cycle.momentum_residual_n_s[0],
                cycle.momentum_residual_n_s[1]
            )?;
            if epoch == 1 || epoch.is_multiple_of(100) || epoch == cycles {
                serde_json::to_writer(
                    &mut progress,
                    &json!({"tick":tick,"cycle":cycle,"rolling_qualification":windows,"counts":counts,"trajectory_sha256":format!("{:x}",digest.clone().finalize())}),
                )?;
                writeln!(progress)?;
                progress.flush()?;
                cycles_file.flush()?;
            }
        }
        before = actual;
    }
    cycles_file.flush()?;
    clip.flush()?;
    if let (Some(mut log), Some(status)) = (placement_log, pool.placement_status()) {
        write_sampler_epoch(&mut log, end_tick, status)?;
        log.flush()?;
        archive(
            out.join("placement-summary.json"),
            &json!({"policy":"3,10,24,0,4","measured_transfers":total_moves,"ticks":end_tick}),
        )?;
    }
    archive(
        out.join("summary.json"),
        &json!({
            "native_step_seconds":DT,"first_tick":clip_begin.max(1),"last_tick":end_tick,
            "canonical_reference":canonical_reference,"native_solver_capacity":pool.native_solver_profile().max_bodies(),
            "warmup_ticks":WARMUP,"period_ticks":PERIOD,"cycles":cycles,
            "loaded_pair":[1,2],"driven_pair":[3,4],"gravity_m_s2":GRAVITY,"drive_n":LIFT,"pulse_ticks":10,
            "rolling_window_ticks":PERIOD*activity::WINDOW as u64,"window_stride_ticks":PERIOD,
            "control":"one uninterrupted four-body native world","counts":counts,"last_cycle":cycle,"rolling_qualification":windows,
            "trajectory_sha256":format!("{:x}",digest.finalize()),"input_sha256":format!("{:x}",input_digest.finalize()),
            "events":late_events,"elapsed_s":start.elapsed().as_secs_f64()
        }),
    )?;
    Ok(())
}

fn write_sampler_epoch(
    log: &mut impl Write,
    tick: u64,
    status: &gz_physics_distributed::pool::PlacementStatus,
) -> Result<()> {
    serde_json::to_writer(
        &mut *log,
        &json!({"incarnation_end":tick,"started":status.windows_started,
        "completed":status.windows_completed,"interrupted":status.windows_interrupted,
        "ineligible":status.windows_ineligible,"deferred":status.windows_deferred,"woken":status.windows_woken,
        "unavailable":status.unavailable_probes,"committed":status.committed_moves}),
    )?;
    writeln!(log)?;
    Ok(())
}
