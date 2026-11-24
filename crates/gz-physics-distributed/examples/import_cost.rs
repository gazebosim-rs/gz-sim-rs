// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure admission of a cold box into populated worker partitions.
use gz_physics_distributed::physics::{Body, Partition, Snapshot, fixture};
use std::{error::Error, hint::black_box, time::Instant};

fn main() -> Result<(), Box<dyn Error>> {
    println!("resident_bodies,scenario,iterations,p50_us,p95_us");
    for count in [1, 8, 16, 31] {
        let mut bodies = Vec::new();
        for i in 0..count {
            bodies.push(Body {
                id: i + 1,
                position: [i as f64 * 12., 0., 0.],
                orientation: [0., 0., (0.03 * i as f64).sin(), (0.03 * i as f64).cos()],
                ..fixture()[1].clone()
            });
        }
        let partition = Partition::restore(&Snapshot {
            bodies: bodies.clone(),
            ..Default::default()
        })?;
        for (name, x, should_pass) in [
            ("clear", 12. * count as f64, true),
            ("intersects_last", bodies.last().unwrap().position[0], false),
        ] {
            let incoming = Body {
                id: 64,
                position: [x, 0., 0.],
                ..bodies[0].clone()
            };
            let mut samples = Vec::new();
            for iteration in 0..1200 {
                let now = Instant::now();
                let result = black_box(&partition).validate_import(black_box(&incoming));
                let latency = now.elapsed().as_secs_f64() * 1e6;
                assert_eq!(result.is_ok(), should_pass);
                if iteration >= 200 {
                    samples.push(latency);
                }
            }
            samples.sort_by(f64::total_cmp);
            println!(
                "{count},{name},{},{:.3},{:.3}",
                samples.len(),
                samples[499],
                samples[949]
            );
        }
    }
    Ok(())
}
