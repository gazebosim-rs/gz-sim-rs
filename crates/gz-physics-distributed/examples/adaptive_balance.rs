// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Balance 14 spatially separated contact pairs using native component solve timings.
#[path = "support/replay.rs"]
mod replay;
use clap::Parser;
use gz_physics_distributed::{
    coordinator::{BalancePolicy, Coordinator, Frame},
    physics::{Body, Island, Snapshot, fixture},
    transport::WorkerProcess,
};
use std::{
    error::Error,
    io,
    path::{Path, PathBuf},
    time::Instant,
};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: Option<PathBuf>,
    #[arg(long)]
    capture: Option<PathBuf>,
    #[arg(long, default_value_t = 300)]
    measured_ticks: u16,
    #[arg(long, default_value_t = 40)]
    window: u16,
}

fn seeds() -> Vec<Body> {
    let mut bodies = Vec::new();
    for lane in 0..14 {
        for side in 0..2 {
            let sign = if side == 0 { -1. } else { 1. };
            bodies.push(Body {
                id: lane * 2 + side + 1,
                position: [sign * 0.36, lane as f64 * 12., 0.],
                linear: [-sign * 0.8, 0., 0.],
                ..fixture()[0].promoted()
            });
        }
    }
    bodies
}
fn create(
    worker: &Path,
    run: u64,
    bodies: &[Body],
    limit: usize,
) -> Result<Coordinator<WorkerProcess>, Box<dyn Error>> {
    Ok(Coordinator::new_recorded(
        run,
        [
            WorkerProcess::spawn(worker, run, 0, &bodies[..24])?,
            WorkerProcess::spawn(worker, run, 1, &bodies[24..])?,
        ],
        limit,
    )?)
}
fn check(frame: &Frame, reference: &Snapshot) {
    assert_eq!(frame.tick, reference.tick);
    for b in &reference.bodies {
        assert_eq!(
            frame.body(b.id).unwrap().1,
            b,
            "body {} at {}",
            b.id,
            frame.tick
        );
    }
    let mut contacts: Vec<_> = frame
        .workers
        .iter()
        .flat_map(|w| &w.snapshot.impulses)
        .collect();
    contacts.sort_by_key(|p| (p.a, p.b));
    assert!(
        contacts.into_iter().eq(&reference.impulses),
        "native contacts at {}",
        frame.tick
    );
}
// Each physical lane is translated as a whole into a 4-column contact inset.
// The two body transforms relative to each other are unmodified.
fn display(frame: &Frame) -> Frame {
    let mut f = frame.clone();
    for obs in &mut f.workers {
        for b in &mut obs.snapshot.bodies {
            let lane = (b.id - 1) / 2;
            b.position[0] += (lane % 4) as f64 * 2. - 3.;
            b.position[1] -= lane as f64 * 12.;
            b.position[2] += (lane / 4) as f64 * 1.15;
        }
    }
    f
}

struct Trial {
    adaptive: Coordinator<WorkerProcess>,
    fixed: Coordinator<WorkerProcess>,
    reference: Island,
    measured: [Vec<f64>; 2],
}
impl Trial {
    fn step(&mut self, measure: bool) -> Result<(), Box<dyn Error>> {
        let order = if self.adaptive.frame().tick.is_multiple_of(2) {
            [0, 1]
        } else {
            [1, 0]
        };
        for i in order {
            let c = if i == 0 {
                &mut self.adaptive
            } else {
                &mut self.fixed
            };
            let before = Instant::now();
            c.advance_overlapped()?;
            if measure {
                self.measured[i].push(before.elapsed().as_secs_f64() * 1e6);
            }
        }
        self.reference.step()?;
        let snapshot = self.reference.snapshot();
        check(self.adaptive.frame(), &snapshot);
        check(self.fixed.frame(), &snapshot);
        Ok(())
    }
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    if args.window == 0 || args.measured_ticks == 0 {
        return Err(io::Error::other("use positive sampling and benchmark windows").into());
    }
    let path = match args.worker {
        Some(p) => p,
        None => std::env::current_exe()?
            .ancestors()
            .nth(2)
            .ok_or_else(|| io::Error::other("pass --worker"))?
            .join(format!("physics_worker{}", std::env::consts::EXE_SUFFIX)),
    };
    let b = seeds();
    let limit = 128 + usize::from(args.measured_ticks) + usize::from(args.window) * 10;
    let mut trial = Trial {
        adaptive: create(&path, 1341, &b, limit)?,
        fixed: create(&path, 1342, &b, limit)?,
        reference: Island::restore(&Snapshot {
            bodies: b,
            ..Default::default()
        })?,
        measured: Default::default(),
    };
    let mut frames = vec![display(trial.adaptive.frame())];
    for _ in 0..20 {
        trial.step(false)?;
        frames.push(display(trial.adaptive.frame()));
    }
    let mut migrations = 0;
    let mut movement_us = 0.;
    for _ in 0..10 {
        let w = trial.adaptive.start_load_sample(args.window)?;
        for _ in 0..args.window {
            trial.step(false)?;
            if trial.adaptive.frame().tick.is_multiple_of(5) {
                frames.push(display(trial.adaptive.frame()));
            }
        }
        let profile = trial.adaptive.finish_load_sample(w)?;
        let before = Instant::now();
        let moved = trial.adaptive.rebalance_one(
            &profile,
            BalancePolicy {
                minimum_savings_ns: 1000,
                minimum_savings_per_mille: 30,
            },
        )?;
        let us = before.elapsed().as_secs_f64() * 1e6;
        if let Some(m) = moved {
            migrations += 1;
            movement_us += us;
            assert!(m.warm);
            check(trial.adaptive.frame(), &trial.reference.snapshot());
            println!(
                "tick {} moved {:?} from {} to {}, predicted solve ns {:?} -> {:?}, handoff {:.2} us",
                trial.adaptive.frame().tick,
                m.members,
                m.source,
                m.destination,
                m.before_ns,
                m.estimated_after_ns,
                us
            );
            frames.push(display(trial.adaptive.frame()));
        } else {
            break;
        }
    }
    assert!(migrations > 0, "no beneficial warm handoff measured");
    for _ in 0..args.measured_ticks {
        trial.step(true)?;
    }
    frames.push(display(trial.adaptive.frame()));
    let mut medians = [0.; 2];
    for (i, times) in trial.measured.iter_mut().enumerate() {
        times.sort_by(f64::total_cmp);
        let p50 = times[(times.len() - 1) / 2];
        let p99 = times[(times.len() * 99).div_ceil(100) - 1];
        medians[i] = p50;
        println!(
            "{}: {} measured overlapped ticks, p50_us={p50:.2}, p99_us={p99:.2}",
            ["adaptive", "fixed"][i],
            times.len()
        );
    }
    println!(
        "Final native pairs per worker {:?}, warm migrations {migrations}, total handoff {movement_us:.2} us, median barrier saving {:.2} us. All {} ticks matched both references.",
        trial
            .adaptive
            .frame()
            .workers
            .each_ref()
            .map(|w| w.snapshot.contacts),
        medians[1] - medians[0],
        trial.adaptive.frame().tick
    );
    if let Some(path) = args.capture {
        replay::write(&path, &frames)?;
    }
    Ok(())
}
