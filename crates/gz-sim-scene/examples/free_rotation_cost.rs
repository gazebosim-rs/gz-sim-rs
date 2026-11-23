// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure an isolated native rotor through the ordinary Bevy update loop.
#[path = "../../../examples/support/free_top.rs"]
#[allow(dead_code)]
mod free_top;
use free_top::{Run, torque};
use std::{hint::black_box, time::Instant};
fn main() {
    println!("mode,driven,step_ms,repeats,ticks_per_repeat,median_us,p95_us");
    for ms in [1, 2, 5, 10] {
        for driven in [false, true] {
            for enabled in [false, true] {
                let mut costs = vec![];
                for i in 0..13 {
                    let mut run = Run::new(enabled, ms).unwrap();
                    let ticks = 6000 / ms;
                    let start = Instant::now();
                    for tick in 0..ticks {
                        run.advance(torque(tick * ms, driven)).unwrap();
                    }
                    let elapsed = start.elapsed().as_secs_f64() * 1e6 / f64::from(ticks);
                    black_box(run.state());
                    if i > 1 {
                        costs.push(elapsed);
                    }
                }
                costs.sort_by(f64::total_cmp);
                println!(
                    "{},{driven},{ms},{},{},{:.6},{:.6}",
                    if enabled { "isolated" } else { "native" },
                    costs.len(),
                    6000 / ms,
                    costs[5],
                    costs[10]
                );
            }
        }
    }
}
