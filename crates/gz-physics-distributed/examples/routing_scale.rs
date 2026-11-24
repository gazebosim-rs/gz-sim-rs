// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time deterministic swept grouping on separated lanes, packed pairs and a dense cluster.
use clap::Parser;
use gz_physics_distributed::physics::{
    Body, ComponentInfo, Snapshot, fixture, potential_contact_groups,
};
use std::{hint::black_box, time::Instant};
#[derive(Parser)]
struct Args {
    #[arg(long, default_value_t = 19)]
    trials: usize,
    #[arg(long, value_delimiter = ',', default_value = "32,128,512,1024")]
    sizes: Vec<usize>,
}
fn main() {
    let args = Args::parse();
    assert!(args.trials >= 5);
    println!("bodies,layout,components,p50_us,p95_us,max_us");
    for n in args.sizes {
        assert!((2..=8192).contains(&n));
        for layout in ["separated", "pairs", "dense"] {
            let make = |i: usize| {
                let x = (i / 2) % 16;
                let y = i / 32;
                let pos = match layout {
                    "separated" => [(i % 32) as f64 * 16., (i / 32) as f64 * 16., 0.],
                    "pairs" => [x as f64 * 16. + (i % 2) as f64 * 0.65, y as f64 * 16., 0.],
                    _ => [i as f64 * 0.0001, 0., 0.],
                };
                Body {
                    id: i as u64,
                    position: pos,
                    linear: [0.25, 0., 0.],
                    ..fixture()[0].promoted()
                }
            };
            let seed = Snapshot {
                bodies: (0..n).map(make).collect(),
                ..Default::default()
            };
            let components = (0..n)
                .map(|i| ComponentInfo {
                    ids: vec![i as u64],
                    contact_history: i % 3 == 0,
                })
                .collect::<Vec<_>>();
            let expected = potential_contact_groups(&seed, &components, &[]).unwrap();
            let mut timings = Vec::new();
            for _ in 0..args.trials {
                let t = Instant::now();
                let result = black_box(potential_contact_groups(
                    black_box(&seed),
                    black_box(&components),
                    &[],
                ))
                .unwrap();
                timings.push(t.elapsed().as_nanos() as f64 / 1000.);
                assert_eq!(result, expected);
            }
            timings.sort_by(f64::total_cmp);
            println!(
                "{n},{layout},{},{:.3},{:.3},{:.3}",
                expected.len(),
                timings[args.trials / 2],
                timings[(args.trials * 95 / 100).min(args.trials - 1)],
                timings[args.trials - 1]
            );
        }
    }
}
