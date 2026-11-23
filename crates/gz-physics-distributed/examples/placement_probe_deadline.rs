// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Stop a real sampling participant at a loaded barrier, then verify every successor.
#[path = "support/changing_oracle.rs"]
mod control;
#[allow(dead_code)]
#[path = "support/changing_worlds.rs"]
mod fixture;
#[cfg(target_os = "linux")]
mod linux {
    use super::{control::Oracle, fixture};
    use clap::{Parser, ValueEnum};
    use gz_physics_distributed::{
        physics::{GroundPlane, SimulationStep},
        pool::WorkerPool,
    };
    use rustix::process::{Pid, PidfdFlags, Signal, pidfd_open, pidfd_send_signal};
    use serde_json::{Value, json};
    use std::{
        collections::BTreeMap,
        error::Error,
        fs::{self, File},
        io::{BufWriter, Write},
        path::PathBuf,
        thread,
        time::{Duration, Instant},
    };
    type Result<T> = std::result::Result<T, Box<dyn Error>>;
    #[derive(Clone, Copy, ValueEnum)]
    enum Boundary {
        Start,
        Read,
    }
    #[derive(Clone, Copy, ValueEnum)]
    enum Mode {
        Fixed,
        Automatic,
        Manual,
    }
    #[derive(Parser)]
    pub(super) struct Args {
        #[arg(long)]
        worker: PathBuf,
        #[arg(long, value_enum)]
        layout: fixture::Layout,
        #[arg(long, value_enum)]
        boundary: Boundary,
        #[arg(long, value_enum, default_value = "automatic")]
        mode: Mode,
        /// 0 is healthy. A positive duration thaws the worker. Omit to suspend until replacement.
        #[arg(long,value_parser=clap::value_parser!(u64).range(0..=1500))]
        thaw_ms: Option<u64>,
        /// Delay SIGCONT from the observed first failed physical dispatch, rather than from SIGSTOP.
        #[arg(long, conflicts_with="thaw_ms", value_parser=clap::value_parser!(u64).range(0..=1000))]
        thaw_after_first_timeout_ms: Option<u64>,
        /// Applied only to cached final-step responses. First and reconstructed solves keep one second.
        #[arg(long, value_parser=clap::value_parser!(u64).range(1..=1000))]
        step_confirm_ms: Option<u64>,
        /// The requested delayed thaw arrives after the selected grace. Verify replacement of its PID.
        #[arg(long, requires = "thaw_after_first_timeout_ms")]
        expect_late_replacement: bool,
        /// The owner of the first loaded slab is 0, and 2 is initially idle.
        #[arg(long,value_parser=clap::value_parser!(u8).range(0..=3),default_value_t=0)]
        peer: u8,
        #[arg(long,value_parser=clap::value_parser!(u16).range(1..),default_value_t=6)]
        rounds: u16,
        out: PathBuf,
    }
    fn run(args: &Args, oracle: &Oracle, number: u16) -> Result<Value> {
        let seeds = fixture::Placement::Split.worker_seeds(&oracle.groups);
        let mut pool = WorkerPool::spawn(
            &args.worker,
            49603,
            seeds,
            SimulationStep::Ms10,
            Some(GroundPlane::new(0.)?),
            96,
        )?;
        pool.set_local_helper_limit(Some(0));
        if let Some(ms) = args.step_confirm_ms {
            pool.set_step_confirmation_window(Duration::from_millis(ms))?;
        }
        let owners: BTreeMap<_, _> = oracle
            .groups
            .iter()
            .zip(fixture::Placement::Split.owners(&oracle.groups))
            .flat_map(|(g, w)| g.iter().map(move |b| (b.id, (w as u8, 0))))
            .collect();
        let at = match args.boundary {
            Boundary::Start => 72,
            Boundary::Read => 74,
        };
        let mut window = None;
        let mut evidence = None;
        for tick in 1..=fixture::END {
            if tick == 73 {
                if matches!(args.mode, Mode::Automatic) {
                    pool.set_periodic_placement(Some("2,8,1,4294967295".parse()?))?;
                } else if matches!(args.mode, Mode::Manual)
                    && matches!(args.boundary, Boundary::Read)
                {
                    window = Some(pool.start_load_sample(2.try_into()?)?);
                }
            }
            let field = fixture::wrenches(tick, &oracle.groups);
            if tick == at + 1 {
                let old = pool.frame().clone();
                let positive = old
                    .workers
                    .iter()
                    .flat_map(|w| &w.snapshot.impulses)
                    .filter(|p| p.normal_impulse > 0.)
                    .count();
                if positive == 0 {
                    return Err("expected positive finite contact at fault".into());
                }
                let pids: Vec<_> = (0..4).map(|i| pool.worker_pid(i).unwrap()).collect();
                let pid = pids[usize::from(args.peer)];
                let fd = pidfd_open(
                    Pid::from_raw(pid.try_into()?).ok_or("invalid PID")?,
                    PidfdFlags::empty(),
                )?;
                if args.thaw_ms != Some(0) {
                    pidfd_send_signal(&fd, Signal::STOP)?;
                    let until = Instant::now() + Duration::from_secs(2);
                    while !fs::read_to_string(format!("/proc/{pid}/status"))?
                        .lines()
                        .any(|l| l.starts_with("State:") && l.contains("T (stopped)"))
                    {
                        if Instant::now() >= until {
                            return Err("worker did not stop".into());
                        }
                        thread::yield_now();
                    }
                }
                let start = Instant::now();
                let mut manual_ok = None;
                let mut retry = false;
                let mut publish_us = None;
                let mut first_failure_us = None;
                let expect_restart = (args.thaw_ms.is_none()
                    && args.thaw_after_first_timeout_ms.is_none())
                    || args.expect_late_replacement;
                thread::scope(|scope| -> Result<()> {
                    let (first_failed, wait_for_first) = std::sync::mpsc::sync_channel(1);
                    let late = args.thaw_after_first_timeout_ms;
                    let thaw = late.or(args.thaw_ms.filter(|&n| n != 0)).map(|n| {
                        scope.spawn(move || {
                            if late.is_some() {
                                wait_for_first
                                    .recv_timeout(Duration::from_secs(4))
                                    .map_err(|_| "first physical timeout was missed")?;
                            }
                            thread::sleep(Duration::from_millis(n));
                            match pidfd_send_signal(&fd, Signal::CONT) {
                                Ok(()) => Ok::<_, String>(()),
                                Err(rustix::io::Errno::SRCH) if expect_restart => Ok(()),
                                Err(e) => Err(e.to_string()),
                            }
                        })
                    });
                    if matches!(args.mode, Mode::Manual) {
                        manual_ok = Some(match window.take() {
                            Some(w) => pool.finish_load_sample(w).is_ok(),
                            None => pool.start_load_sample(2.try_into()?).is_ok(),
                        });
                    }
                    if pool
                        .advance_islands_in_field(fixture::GRAVITY, &field)
                        .is_err()
                    {
                        retry = true;
                        first_failure_us = Some(start.elapsed().as_secs_f64() * 1e6);
                        if late.is_some() {
                            first_failed.send(())?;
                        }
                        if pool.frame() != &old {
                            return Err("unconfirmed solve changed published state".into());
                        }
                        pool.advance_islands_in_field(fixture::GRAVITY, &field)?;
                    }
                    publish_us = Some(start.elapsed().as_secs_f64() * 1e6);
                    drop(first_failed);
                    if let Some(job) = thaw {
                        job.join().map_err(|_| "thaw task panicked")??;
                    }
                    Ok(())
                })?;
                let elapsed_us = publish_us.ok_or("missing publication time")?;
                let after: Vec<_> = (0..4).map(|i| pool.worker_pid(i).unwrap()).collect();
                for (i, (a, b)) in pids.iter().zip(&after).enumerate() {
                    if (a != b) != (expect_restart && i == usize::from(args.peer)) {
                        return Err("unexpected replacement set".into());
                    }
                }
                let status = pool.placement_status();
                evidence = Some(
                    json!({"round":number,"tick":tick,"positive_pairs_before":positive,"before_pids":pids,"after_pids":after,"elapsed_us":elapsed_us,"first_step_retry":retry,"first_failure_us":first_failure_us,
                    "manual_probe_accepted":manual_ok,"windows_started":status.map_or(0,|s|s.windows_started),"windows_completed":status.map_or(0,|s|s.windows_completed),"unavailable_probes":status.map_or(0,|s|s.unavailable_probes)}),
                );
            } else {
                pool.advance_islands_in_field(fixture::GRAVITY, &field)?;
            }
            oracle.verify(pool.frame(), &owners)?;
        }
        evidence.ok_or_else(|| "suspension was missed".into())
    }
    pub(super) fn main() -> Result<()> {
        let args = Args::parse();
        fs::create_dir(&args.out)?;
        let oracle = Oracle::new(args.layout)?;
        let mut log = BufWriter::new(File::create_new(args.out.join("episodes.jsonl"))?);
        for round in 0..args.rounds {
            writeln!(log, "{}", run(&args, &oracle, round)?)?;
            log.flush()?;
        }
        fs::write(
            args.out.join("proof.json"),
            serde_json::to_vec_pretty(
                &json!({"layout":args.layout,"native_sha256":oracle.digest,"pairs_per_run":oracle.pairs,"floors_per_run":oracle.floors,"ticks_per_run":fixture::END,"rounds":args.rounds,"peer":args.peer,"thaw_ms":args.thaw_ms,"thaw_after_first_timeout_ms":args.thaw_after_first_timeout_ms,"step_confirm_ms":args.step_confirm_ms,"expect_late_replacement":args.expect_late_replacement}),
            )?,
        )?;
        Ok(())
    }
}
#[cfg(target_os = "linux")]
fn main() -> Result<(), Box<dyn std::error::Error>> {
    linux::main()
}
#[cfg(not(target_os = "linux"))]
fn main() {
    eprintln!("This process suspension qualification requires Linux pidfds.");
    std::process::exit(2);
}
