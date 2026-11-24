// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "support/inclined_sheet.rs"]
mod sheet;
use glam::DVec3;
use sheet::*;
use std::error::Error;
fn benchmark() -> Result<(), Box<dyn Error>> {
    use std::time::Instant;
    for (name, mu, v0) in [
        ("static", 0.45, 0.),
        ("kinetic", 0.2, 0.),
        ("arrest", 0.45, 0.32),
    ] {
        let mut g = Incline::new(&sdf(mu, v0, 250000))?;
        let mut times = Vec::new();
        let mut trials = 0u64;
        let mut intervals = 0u64;
        for epoch in 0..10 {
            if epoch > 0 {
                gz_sim_scene::reset_loaded_world(&mut g.sim, |_| ())?;
            }
            for _ in 0..330 {
                let t = Instant::now();
                g.step()?;
                let ns = t.elapsed().as_nanos();
                if epoch >= 2 {
                    times.push(ns);
                    for r in g.material().support_this_step() {
                        trials += u64::from(r.trajectory_trials);
                        intervals += 1;
                    }
                }
            }
        }
        times.sort_unstable();
        let q = |p: usize| times[(p * times.len()).div_ceil(100) - 1];
        println!(
            "{name} four-pad ordinary SDF, {} measured 1 ms ticks, p50/p95/p99 {} / {} / {} ns, {intervals} accepted contact intervals, {trials} complete nonlinear trajectory trials",
            times.len(),
            q(50),
            q(95),
            q(99)
        );
    }
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    if args.first().is_some_and(|s| s == "--bench") {
        return benchmark();
    }
    let mu = args.first().map_or(Ok(0.45), |v| v.parse::<f64>())?;
    let steps = args.get(1).map_or(Ok(400), |v| v.parse::<u64>())?;
    let width = args.get(2).map_or(Ok(250000), |v| v.parse::<u64>())?;
    let speed = args.get(3).map_or(Ok(0.), |v| v.parse::<f64>())?;
    let mut g = Incline::new(&sdf(mu, speed, width))?;
    let start = g.centroid().0;
    let along = downhill();
    let normal = normal();
    let mut nonzero = 0;
    for step in 1..=steps {
        g.step()?;
        let (x, v, _) = g.centroid();
        let s = g.material();
        let mut load = DVec3::ZERO;
        let mut ratios = [0.; 4];
        let mut jn = [0.; 4];
        let mut center_max = 0f64;
        let mut slip = 0f64;
        for a in s
            .support_this_step()
            .iter()
            .flat_map(|r| r.beginning.iter().chain(&r.end))
        {
            for l in &a.material {
                let j = -DVec3::from(l.on_rigid);
                load += j;
                let n = DVec3::from(l.site.normal);
                jn[l.site.sphere] += n.dot(j);
                if let Some(f) = &l.friction {
                    ratios[l.site.sphere] += DVec3::from(f.on_material).length();
                    slip = slip.max(DVec3::from(f.final_slip).length());
                }
            }
        }
        nonzero += s.support_this_step().len();
        for r in g.states() {
            center_max = center_max.max(DVec3::from(r.angular_velocity).length());
        }
        if step < 4 || step % 25 == 0 || step == steps {
            println!(
                "t={:.3} dx={:.9e} cross={:.3e} penetration_dir={:.3e} v={:.9e} Fn={:.7} Ft={:.7} ratios={:?} slip={:.6e} native_w={:.3e} intervals={nonzero}",
                g.sim.elapsed().as_secs_f64(),
                (x - start).dot(along),
                (x - start).y,
                (x - start).dot(normal),
                v.dot(along),
                load.dot(normal) * 1000.,
                load.dot(along) * 1000.,
                std::array::from_fn::<_, 4, _>(|i| ratios[i] / jn[i]),
                slip,
                center_max
            );
        }
    }
    Ok(())
}
