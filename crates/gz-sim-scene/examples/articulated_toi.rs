// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Report true internal event time and measure a complete public native/Bevy tick.
#[allow(dead_code)]
#[path = "support/articulated_mobility.rs"]
mod fixture;
use fixture::*;
use gz_sim_scene::{deformable, reset_loaded_world};
use std::{error::Error, time::Instant};
const CLOTH: &str = include_str!("../../../examples/articulated_first_impact.sdf");
const TRANSIENT: &str = include_str!("../../../examples/articulated_transient_cable.sdf");
fn main() -> Result<(), Box<dyn Error>> {
    let bench = std::env::args().any(|s| s == "--bench");
    let cases = if std::env::args().any(|s| s == "--ballistic") {
        [
            (
                include_str!("../../../examples/articulated_ballistic_cloth.sdf"),
                "gravity-driven cloth",
                214,
            ),
            (
                include_str!("../../../examples/articulated_ballistic_apex.sdf"),
                "thrown cable",
                1,
            ),
        ]
    } else {
        [(CLOTH, "cloth", 101), (TRANSIENT, "transient cable", 1)]
    };
    for (xml, name, count) in cases {
        let mut g = Gripper::new(xml)?;
        g.sim.step_exact(count)?;
        let hit = deformable::bodies(&g.sim)
            .next()
            .unwrap()
            .1
            .latest_articulated_impact()
            .ok_or("missing swept material impact")?;
        let time = hit.detection.ok_or("missing continuous arrival time")?;
        println!(
            "{name} public cut {:.9} s, first continuous impact {:.12} s, native/material clock {} ns, spatial quantization bound {:e} m, {} simultaneous witnesses, energy loss {:.9} J",
            g.sim.elapsed().as_secs_f64(),
            time.public_start.as_secs_f64() + time.seconds_after_start,
            hit.time.as_nanos(),
            time.rounding_distance,
            hit.material.loads.len(),
            hit.material.dissipated_energy
        );
        if bench {
            let mut event = vec![];
            let mut clear = vec![];
            for i in 0..240 {
                reset_loaded_world(&mut g.sim, |_| ())?;
                if count > 1 {
                    g.sim.step_exact(count - 2)?;
                    let now = Instant::now();
                    g.sim.step_exact(1)?;
                    let ns = now.elapsed().as_nanos();
                    if i >= 40 {
                        clear.push(ns);
                    }
                }
                let now = Instant::now();
                g.sim.step_exact(1)?;
                let ns = now.elapsed().as_nanos();
                if i >= 40 {
                    event.push(ns);
                }
            }
            for (label, mut ns) in [("clear swept tick", clear), ("split impact tick", event)] {
                if !ns.is_empty() {
                    ns.sort_unstable();
                    println!(
                        "{name} {label}, 200 post-warmup public Bevy steps, p50/p95/p99 {} / {} / {} ns",
                        ns[99], ns[189], ns[197]
                    );
                }
            }
        }
    }
    Ok(())
}
