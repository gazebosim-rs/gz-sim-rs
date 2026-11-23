// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Counterbalanced periodic load measurement and automatic placement on changing contact islands.
#[path = "support/changing_oracle.rs"]
mod control;
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod fixture;
use clap::Parser;
use control::Oracle;
use gz_physics_distributed::{
    physics::{GroundPlane, SimulationStep},
    pool::{PeriodicPlacement, PoolBalanceDecision, PoolNativeRecovery, WorkerPool},
};
use serde::Serialize;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::{self, File},
    io::{BufWriter, Write},
    path::PathBuf,
    time::Instant,
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, value_enum)]
    layout: fixture::Layout,
    #[arg(long, value_enum, default_value = "single")]
    placement: fixture::Placement,
    #[arg(long)]
    helpers: Option<usize>,
    #[arg(long, default_value = "8,40,24,5000")]
    policy: PeriodicPlacement,
    #[arg(long,default_value_t=24,value_parser=clap::value_parser!(u16).range(1..))]
    rounds: u16,
    /// Stagger the six launch offsets by (phase + 3*i) modulo ten public ticks.
    #[arg(long,value_parser=clap::value_parser!(u8).range(0..=9))]
    launch_phase: Option<u8>,
    /// One recorded automatic run with a lost ACK, worker death and durable restore.
    #[arg(long)]
    encounter: bool,
    out: PathBuf,
}
#[derive(Clone, Copy, Serialize)]
#[serde(rename_all = "snake_case")]
enum Mode {
    Fixed,
    Observe,
    Automatic,
}
#[derive(Serialize)]
struct Timing {
    mode: Mode,
    full_us: f64,
    steady_p95_us: f64,
    landing_us: Vec<f64>,
    moves: Vec<Value>,
    windows_started: u64,
    windows_ineligible: u64,
    windows_deferred: u64,
    windows_woken: u64,
    sample_starts: Vec<u64>,
    contact_transitions: Vec<u64>,
    closing_without_move_us: Vec<f64>,
    opening_only_us: Vec<f64>,
    windows_completed: u64,
    windows_interrupted: u64,
    unavailable_probes: u64,
}
fn observe(config: PeriodicPlacement, mode: Mode) -> Option<PeriodicPlacement> {
    match mode {
        Mode::Fixed => None,
        Mode::Observe => Some(PeriodicPlacement {
            handoff_us: u32::MAX,
            ..config
        }),
        Mode::Automatic => Some(config),
    }
}
fn configure(pool: &mut WorkerPool, args: &Args, mode: Mode) -> Result<()> {
    pool.set_handoff_confirmation(true);
    pool.set_local_helper_limit(args.helpers);
    pool.set_packed_handoffs(true);
    pool.set_periodic_placement(observe(args.policy, mode))?;
    Ok(())
}
type Owners = BTreeMap<u64, (u8, u64)>;
fn apply_move(tick: u64, decision: &PoolBalanceDecision, owners: &mut Owners) -> Result<Value> {
    for id in &decision.members {
        let (owner, generation) = owners.get_mut(id).ok_or("unknown scheduled body")?;
        if *owner != decision.source {
            return Err("wrong pre-handoff owner".into());
        }
        *owner = decision.destination;
        *generation += 1;
    }
    Ok(
        json!({"tick":tick,"source":decision.source,"destination":decision.destination,"members":decision.members,
  "before_lower_ns":decision.before_lower_ns,"after_serial_peak_ns":decision.after_serial_peak_ns,
  "estimated_payback_ticks":decision.estimated_payback_ticks}),
    )
}
fn counters(pool: &WorkerPool) -> [u64; 7] {
    pool.placement_status().map_or([0; 7], |s| {
        [
            s.windows_started,
            s.windows_completed,
            s.windows_interrupted,
            s.unavailable_probes,
            s.windows_ineligible,
            s.windows_deferred,
            s.windows_woken,
        ]
    })
}
fn sample(
    pool: &WorkerPool,
    owners: &Owners,
    pads: [u64; 2],
    cost: f64,
    events: &[&str],
    decision: Option<&Value>,
) -> Value {
    let f = pool.frame();
    let bodies: Vec<_> = owners.keys().map(|&id| f.body(id).unwrap().1).collect();
    let mut pairs: Vec<_> = f
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.impulses)
        .collect();
    pairs.sort_by_key(|p| (p.a, p.b));
    let mut floors: Vec<_> = f
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.supports)
        .collect();
    floors.sort_by_key(|p| p.body);
    let counts = pads.map(|id| {
        pairs
            .iter()
            .filter(|p| p.a == id && p.normal_impulse > 0.)
            .count()
    });
    json!({"tick":f.tick,"bodies":bodies,"owners":owners.values().map(|v|v.0).collect::<Vec<_>>(),"pairs":pairs,"floors":floors,
  "loaded_pairs":counts,"elapsed_us":cost,"events":events,"decision":decision,"window_counters":&counters(pool)[..4],
  "ineligible_windows":counters(pool)[4],"deferred_windows":counters(pool)[5],
  "pids":(0..4).map(|w|pool.worker_pid(w).unwrap()).collect::<Vec<_>>()})
}
fn trial(args: &Args, oracle: &Oracle, launch_ticks: &[u64; 6], mode: Mode) -> Result<Timing> {
    let mut pool = WorkerPool::spawn(
        &args.worker,
        29018,
        args.placement.worker_seeds(&oracle.groups),
        SimulationStep::Ms10,
        Some(GroundPlane::new(0.)?),
        96,
    )?;
    configure(&mut pool, args, mode)?;
    let mut owners: Owners = oracle
        .groups
        .iter()
        .zip(args.placement.owners(&oracle.groups))
        .flat_map(|(g, owner)| g.iter().map(move |b| (b.id, (owner as u8, 0))))
        .collect();
    let pads: Vec<_> = oracle
        .groups
        .iter()
        .filter(|g| g.len() == 25)
        .map(|g| g[0].id)
        .collect();
    let pads: [u64; 2] = pads.try_into().map_err(|_| "expected two loaded pads")?;
    let mut trace = if args.encounter {
        Some(BufWriter::new(File::create_new(
            args.out.join("frames.jsonl"),
        )?))
    } else {
        None
    };
    if let Some(f) = &mut trace {
        writeln!(f, "{}", sample(&pool, &owners, pads, 0., &[], None))?;
    }
    let mut moves = vec![];
    let mut times = vec![];
    let mut landing_us = vec![];
    let mut previous = [24; 2];
    let mut retired_counters = [0; 7];
    let mut sample_starts = Vec::new();
    let mut contact_transitions = Vec::new();
    let mut closing_without_move_us = Vec::new();
    let mut opening_only_us = Vec::new();
    for tick in 1..=fixture::END {
        let field = fixture::wrenches_at(tick, &oracle.groups, launch_ticks);
        let mut events = vec![];
        let affected = if args.encounter && tick == 80 {
            pool.drop_next_acknowledgement(3)?;
            events.push("missing_ack");
            Some((3, pool.worker_pid(3)))
        } else if args.encounter && tick == 140 {
            let owner = owners[&pads[0]].0;
            let pid = pool.worker_pid(owner);
            pool.terminate_worker(owner)?;
            events.push("worker_kill");
            Some((owner, pid))
        } else {
            None
        };
        let before_counts = counters(&pool);
        let start = Instant::now();
        let result = pool
            .advance_islands_in_field(fixture::GRAVITY, &field)
            .map(|_| ());
        if let Some((w, pid)) = affected {
            if result.is_ok() || pool.frame().tick != tick - 1 {
                return Err(format!("fault escaped its barrier at {tick}").into());
            }
            pool.advance_islands_in_field(fixture::GRAVITY, &field)?;
            if (pool.worker_pid(w) == pid) != (tick == 80) {
                return Err("incorrect fault recovery PID".into());
            }
        } else {
            result?;
        }
        let us = start.elapsed().as_secs_f64() * 1e6;
        times.push(us);
        let after_counts = counters(&pool);
        if after_counts[0] > before_counts[0] {
            sample_starts.push(tick - 1);
        }
        let decision = pool
            .placement_status()
            .and_then(|s| s.last_move.as_ref())
            .filter(|(t, _)| *t == tick)
            .map(|(_, d)| apply_move(tick, d, &mut owners))
            .transpose()?;
        if after_counts[1] > before_counts[1] && decision.is_none() {
            closing_without_move_us.push(us);
        } else if after_counts[0] > before_counts[0] && after_counts[1] == before_counts[1] {
            opening_only_us.push(us);
        }
        if matches!(mode, Mode::Observe) && decision.is_some() {
            return Err("monitor-only control moved an island".into());
        }
        oracle.verify(pool.frame(), &owners)?;
        let counts = pads.map(|id| {
            pool.frame()
                .workers
                .iter()
                .flat_map(|w| &w.snapshot.impulses)
                .filter(|p| p.a == id && p.normal_impulse > 0.)
                .count()
        });
        if tick > fixture::SETTLED
            && counts
                .iter()
                .zip(previous)
                .any(|(&n, last)| last == 0 && n == 24)
        {
            landing_us.push(us);
        }
        if tick > fixture::SETTLED
            && counts
                .iter()
                .zip(previous)
                .any(|(&n, p)| (n == 0) != (p == 0))
        {
            contact_transitions.push(tick);
        }
        previous = counts;
        if args.encounter && tick == 151 {
            let path = args.out.join("committed.gzpool");
            pool.save_native_recovery(&path)?;
            let saved = pool.frame().clone();
            let old = (0..4)
                .map(|w| pool.worker_pid(w).unwrap())
                .collect::<Vec<_>>();
            retired_counters = counters(&pool);
            drop(pool);
            pool = PoolNativeRecovery::open(&path)?.resume(&args.worker)?;
            if pool.frame() != &saved || (0..4).any(|w| old.contains(&pool.worker_pid(w).unwrap()))
            {
                return Err("fresh pool changed its committed cut".into());
            }
            configure(&mut pool, args, mode)?;
            events.push("fresh_pool");
        }
        if let Some(f) = &mut trace {
            writeln!(
                f,
                "{}",
                sample(&pool, &owners, pads, us, &events, decision.as_ref())
            )?;
        }
        if let Some(d) = decision {
            moves.push(d);
        }
    }
    if let Some(f) = &mut trace {
        f.flush()?;
    }
    if landing_us.len() != 6 {
        return Err("missing observed touchdown".into());
    }
    let full_us = times.iter().sum();
    let steady = &mut times[fixture::SETTLED as usize..];
    steady.sort_by(f64::total_cmp);
    let [
        windows_started,
        windows_completed,
        windows_interrupted,
        unavailable_probes,
        windows_ineligible,
        windows_deferred,
        windows_woken,
    ] = std::array::from_fn(|i| counters(&pool)[i] + retired_counters[i]);
    Ok(Timing {
        mode,
        full_us,
        steady_p95_us: steady[(95 * steady.len()).div_ceil(100) - 1],
        landing_us,
        moves,
        windows_started,
        windows_completed,
        windows_interrupted,
        unavailable_probes,
        windows_ineligible,
        windows_deferred,
        windows_woken,
        sample_starts,
        contact_transitions,
        closing_without_move_us,
        opening_only_us,
    })
}
fn main() -> Result<()> {
    let args = Args::parse();
    fs::create_dir(&args.out)?;
    let launch_ticks = std::array::from_fn(|i| {
        fixture::LAUNCHES[i]
            + args
                .launch_phase
                .map_or(0, |p| (u64::from(p) + 3 * i as u64) % 10)
    });
    let oracle = match args.launch_phase {
        Some(_) => Oracle::with_schedule(args.layout, &launch_ticks)?,
        None => Oracle::new(args.layout)?,
    };
    let mut log = BufWriter::new(File::create_new(args.out.join("trials.jsonl"))?);
    let modes = [Mode::Fixed, Mode::Observe, Mode::Automatic];
    let orders = [
        [0, 1, 2],
        [1, 2, 0],
        [2, 0, 1],
        [2, 1, 0],
        [0, 2, 1],
        [1, 0, 2],
    ];
    let rounds = if args.encounter { 1 } else { args.rounds };
    for round in 0..rounds {
        let order: &[usize] = if args.encounter {
            &[2]
        } else {
            &orders[usize::from(round) % 6]
        };
        for &i in order {
            let data = trial(&args, &oracle, &launch_ticks, modes[i])?;
            writeln!(log, "{}", json!({"round":round,"data":data}))?;
            log.flush()?;
        }
    }
    fs::write(
        args.out.join("proof.json"),
        serde_json::to_vec_pretty(
            &json!({"rounds":rounds,"layout":args.layout,"initial":args.placement,"helpers":args.helpers,
    "policy":args.policy,"launch_ticks":launch_ticks,"native_sha256":oracle.digest,"native_steps_per_trial":fixture::END,"native_pairs_per_trial":oracle.pairs,"native_floors_per_trial":oracle.floors}),
        )?,
    )?;
    Ok(())
}
