// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure the spatial cut on an actual stable population of supported native solvers.
use clap::Parser;
use gz_physics_distributed::{
    coordinator::{BalancePolicy, Coordinator},
    physics::{Body, GroundPlane, MAX_PARTITION_BODIES, SimulationStep, fixture},
    transport::WorkerProcess,
};
use std::{error::Error, hint::black_box, path::PathBuf, time::Instant};
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long, value_delimiter = ',', default_value = "32,128,256,384")]
    sizes: Vec<usize>,
    #[arg(long, default_value_t = 31)]
    trials: usize,
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    println!("bodies,left,right,p50_us,p95_us,peak_ns,predicted_after_ns,moved_warm,axis,member");
    for n in args.sizes {
        assert!((4..2 * MAX_PARTITION_BODIES).contains(&n));
        assert!(args.trials >= 5);
        let first = (n * 3 / 4).min(MAX_PARTITION_BODIES);
        let base = fixture()[0].promoted();
        let boxes = (0..n)
            .map(|i| Body {
                id: i as u64,
                position: [10. * (i / 16) as f64, 10. * (i % 16) as f64, 0.35],
                linear: [0.1, 0., 0.],
                size: [0.4; 3],
                friction: 0.4,
                ..base.clone()
            })
            .collect::<Vec<_>>();
        let run = 0x4b412 + n as u64;
        let step = SimulationStep::Ms5;
        let ground = Some(GroundPlane::with_friction(0., 0.6)?);
        let mut c = Coordinator::new(
            run,
            [
                WorkerProcess::spawn_with_ground(
                    &args.worker,
                    run,
                    0,
                    &boxes[..first],
                    step,
                    ground,
                )?,
                WorkerProcess::spawn_with_ground(
                    &args.worker,
                    run,
                    1,
                    &boxes[first..],
                    step,
                    ground,
                )?,
            ],
            4096,
        )?;
        let g = [0., 0., -9.81];
        for _ in 0..70 {
            c.advance_in_field_overlapped(g, &[])?;
        }
        assert_eq!(
            c.frame()
                .workers
                .iter()
                .map(|w| w.snapshot.supports.len())
                .sum::<usize>(),
            n
        );
        let window = c.start_load_sample(12)?;
        for _ in 0..12 {
            c.advance_in_field_overlapped(g, &[])?;
        }
        let sample = c.finish_load_sample(window)?;
        let p = BalancePolicy {
            minimum_savings_ns: 0,
            minimum_savings_per_mille: 0,
        };
        let expected = c
            .plan_rebalance(&sample, p)?
            .ok_or("no feasible improving cut")?;
        let mut times = Vec::new();
        for _ in 0..args.trials {
            let timer = Instant::now();
            let result = black_box(c.plan_rebalance(black_box(&sample), p)?);
            times.push(timer.elapsed().as_secs_f64() * 1e6);
            assert_eq!(result.as_ref(), Some(&expected));
        }
        times.sort_by(f64::total_cmp);
        let before = c.frame().clone();
        let decision = c.rebalance_one(&sample, p)?.unwrap();
        assert!(decision.warm);
        assert_eq!(expected, decision);
        for w in &before.workers {
            for b in &w.snapshot.bodies {
                assert_eq!(c.frame().body(b.id).unwrap().1, b);
            }
        }
        assert_eq!(
            c.frame()
                .workers
                .iter()
                .map(|w| w.snapshot.supports.len())
                .sum::<usize>(),
            n
        );
        println!(
            "{n},{first},{},{:.3},{:.3},{},{},{},{},{}",
            n - first,
            times[times.len() / 2],
            times[(times.len() * 95 / 100).min(times.len() - 1)],
            decision.before_ns.into_iter().max().unwrap(),
            decision.estimated_after_ns.into_iter().max().unwrap(),
            decision.warm,
            decision.axis,
            decision.members[0]
        );
        for mut p in c.into_peers() {
            p.shutdown()?;
        }
    }
    Ok(())
}
