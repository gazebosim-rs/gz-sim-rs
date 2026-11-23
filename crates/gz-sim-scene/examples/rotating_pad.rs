// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "support/rotating_pad.rs"]
mod source;
use gz_sim_scene::reset_loaded_world;
use source::*;
use std::{error::Error, time::Instant};
fn main() -> Result<(), Box<dyn Error>> {
    let bench = std::env::args().any(|s| s == "--bench");
    for (name, source, ticks) in [
        ("rotating pad and cable", CABLE, 1),
        ("rotating pad and falling sheet", CLOTH, 81),
    ] {
        let mut rotor = Rotor::new(source)?;
        rotor.advance(ticks)?;
        let hit = rotor
            .material()
            .latest_articulated_impact()
            .ok_or("missing impact")?;
        let d = hit.detection.ok_or("missing off-grid time")?;
        println!(
            "{name}: continuous time {:.12} s, clock {} ns, angular speed after {:.10} rad/s, energy loss {:.9} J",
            d.public_start.as_secs_f64() + d.seconds_after_start,
            hit.time.as_nanos(),
            rotor.state().angular_velocity[2],
            hit.material.dissipated_energy
        );
        if bench {
            let mut clear = vec![];
            let mut impact = vec![];
            for i in 0..240 {
                reset_loaded_world(&mut rotor.sim, |_| ())?;
                if ticks > 1 {
                    rotor.advance(ticks - 2)?;
                    let clock = Instant::now();
                    rotor.advance(1)?;
                    if i >= 40 {
                        clear.push(clock.elapsed().as_nanos());
                    }
                }
                let clock = Instant::now();
                rotor.advance(1)?;
                if i >= 40 {
                    impact.push(clock.elapsed().as_nanos());
                }
            }
            for (label, mut ns) in [("clear", clear), ("impact", impact)] {
                if !ns.is_empty() {
                    ns.sort_unstable();
                    println!(
                        "{name}, {label}, complete public Bevy step, 200 post-warmup samples, p50/p95/p99 {} / {} / {} ns",
                        ns[99], ns[189], ns[197]
                    );
                }
            }
        }
    }
    Ok(())
}
