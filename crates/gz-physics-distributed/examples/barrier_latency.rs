// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare sequential and overlapped two-process barriers with identical trajectories.
use clap::Parser;
use gz_physics_distributed::{
    coordinator::Coordinator,
    physics::{Body, BodyWrench, fixture},
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
    /// Number of measured ticks per fresh world.
    #[arg(long, default_value_t = 300)]
    ticks: usize,
    /// Use new worker processes for each repetition; startup is outside the timer.
    #[arg(long, default_value_t = 5)]
    repetitions: usize,
    /// Also apply uniform gravity and an equal held force to two separated bodies.
    #[arg(long)]
    field: bool,
    /// Number of initially approaching pairs on each worker.
    #[arg(long, value_delimiter = ',', default_value = "1,4,16")]
    pairs: Vec<usize>,
}

fn spawn(
    worker: &Path,
    run: u64,
    pairs: usize,
    capacity: usize,
) -> Result<Coordinator<WorkerProcess>, Box<dyn Error>> {
    let mut groups = [Vec::new(), Vec::new()];
    for (w, group) in groups.iter_mut().enumerate() {
        for lane in 0..pairs {
            for side in 0..2 {
                let sign = 2. * side as f64 - 1.;
                group.push(Body {
                    id: (w * 32 + lane * 2 + side + 1) as u64,
                    position: [sign * 0.36, lane as f64 * 12., w as f64 * 30.],
                    linear: [-sign * 0.8, 0., 0.],
                    ..fixture()[0].promoted()
                });
            }
        }
    }
    let a = WorkerProcess::spawn(worker, run, 0, &groups[0])?;
    let b = WorkerProcess::spawn(worker, run, 1, &groups[1])?;
    Ok(Coordinator::new(run, [a, b], capacity)?)
}

#[derive(Default)]
struct Metrics {
    us: Vec<f64>,
    contacts: usize,
}
impl Metrics {
    fn record(
        &mut self,
        c: &mut Coordinator<WorkerProcess>,
        overlap: bool,
        field: bool,
    ) -> Result<(), Box<dyn Error>> {
        let now = Instant::now();
        let inputs = [1, 33].map(|id| BodyWrench {
            id,
            force: [0., 0.2, 0.],
            torque: [0.; 3],
        });
        let frame = if field && overlap {
            c.advance_in_field_overlapped([0., 0., -9.81], &inputs)?
        } else if field {
            c.advance_in_field([0., 0., -9.81], &inputs)?
        } else if overlap {
            c.advance_overlapped()?
        } else {
            c.advance()?
        };
        self.us.push(now.elapsed().as_secs_f64() * 1e6);
        self.contacts += frame
            .workers
            .iter()
            .map(|w| w.snapshot.contacts)
            .sum::<usize>();
        Ok(())
    }
    fn print(&mut self, pairs: usize, mode: &str) {
        self.us.sort_unstable_by(f64::total_cmp);
        let n = self.us.len();
        let quantile = |pct: usize| self.us[(n * pct).div_ceil(100) - 1];
        println!(
            "{pairs},{mode},{n},{:.3},{:.3},{:.3},{:.3},{}",
            self.us.iter().sum::<f64>() / n as f64,
            quantile(50),
            quantile(95),
            quantile(99),
            self.contacts
        );
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    if args.ticks == 0
        || args.ticks > 10_000
        || args.repetitions == 0
        || args.repetitions > 100
        || args.pairs.iter().any(|p| !(1..=16).contains(p))
    {
        return Err("expected 1–10000 ticks, 1–100 repetitions and 1–16 pairs per worker".into());
    }
    let worker = match args.worker {
        Some(p) => p,
        None => std::env::current_exe()?
            .ancestors()
            .nth(2)
            .ok_or_else(|| io::Error::other("missing worker profile directory"))?
            .join(format!("physics_worker{}", std::env::consts::EXE_SUFFIX)),
    };
    println!("pairs_per_worker,mode,samples,mean_us,median_us,p95_us,p99_us,active_pair_samples");
    for pairs in args.pairs {
        let (mut serial, mut overlap) = (Metrics::default(), Metrics::default());
        for repetition in 0..args.repetitions {
            let run = u64::from(std::process::id()) * 2;
            let mut a = spawn(&worker, run, pairs, args.ticks + 32)?;
            let mut b = spawn(&worker, run + 1, pairs, args.ticks + 32)?;
            for _ in 0..16 {
                assert_eq!(a.advance()?, b.advance_overlapped()?);
            }
            for t in 0..args.ticks {
                if (t + repetition) % 2 == 0 {
                    serial.record(&mut a, false, args.field)?;
                    overlap.record(&mut b, true, args.field)?;
                } else {
                    overlap.record(&mut b, true, args.field)?;
                    serial.record(&mut a, false, args.field)?;
                }
                assert_eq!(
                    a.frame(),
                    b.frame(),
                    "divergence at tick {}",
                    a.frame().tick
                );
            }
            for mut p in a.into_peers().into_iter().chain(b.into_peers()) {
                p.shutdown()?;
            }
        }
        assert_eq!(serial.contacts, overlap.contacts);
        serial.print(pairs, "sequential");
        overlap.print(pairs, "overlapped");
    }
    Ok(())
}
