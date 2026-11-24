// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "support/articulated_mobility.rs"]
mod fixture;
use fixture::*;
use glam::DVec3;
use gz_sim_scene::deformable;
use parry3d_f64::{
    query::PointQuery,
    shape::{Segment, Triangle},
};
use std::{
    error::Error,
    time::{Duration, Instant},
};
const REST: &str = include_str!("../../../examples/articulated_supported_cable.sdf");
const CLOTH: &str = include_str!("../../../examples/articulated_supported_cloth.sdf");
const TILTED: &str = include_str!("../../../examples/articulated_ballistic_cloth.sdf");
const RELEASE: &str = include_str!("../../../examples/articulated_releasing_cable.sdf");
fn benchmark() -> Result<(), Box<dyn Error>> {
    for (name, source) in [
        ("unequal supported cable", REST),
        ("two sheet interiors", CLOTH),
        ("rolling release", RELEASE),
    ] {
        let mut g = Gripper::new(source)?;
        let mut groups: [Vec<u64>; 3] = Default::default();
        for pass in 0..18 {
            if pass > 0 {
                gz_sim_scene::reset_loaded_world(&mut g.sim, |_| ())?;
            }
            for _ in 0..220 {
                let start = Instant::now();
                g.sim.step_exact(1)?;
                let elapsed = start.elapsed().as_nanos() as u64;
                let soft = deformable::bodies(&g.sim).next().unwrap().1;
                let group = if !soft.impacts_this_step().is_empty() {
                    2
                } else if !soft.normal_support_this_step().is_empty() {
                    1
                } else {
                    0
                };
                if pass >= 2 {
                    groups[group].push(elapsed);
                }
            }
        }
        for (kind, mut times) in ["clear", "loaded", "onset"].into_iter().zip(groups) {
            times.sort_unstable();
            let q = |p: usize| times[(p * times.len()).div_ceil(100) - 1];
            println!(
                "{name}, {kind}, {} complete public 1 ms ticks, p50/p95/p99 {} / {} / {} ns",
                times.len(),
                q(50),
                q(95),
                q(99)
            );
        }
    }
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let a: Vec<_> = std::env::args().skip(1).collect();
    if a.iter().any(|s| s == "--bench") {
        return benchmark();
    }
    let mut src = if a.iter().any(|s| s == "release") {
        RELEASE
    } else {
        REST
    }
    .to_string();
    if a.iter().any(|s| s == "asymmetric") {
        src = RELEASE
            .replacen("mass=\"0.215\"", "mass=\"0.17\"", 1)
            .replace("mass=\"0.215\"", "mass=\"0.26\"");
    }
    if a.iter().any(|s| s == "cloth" || s == "sag") {
        src = CLOTH.to_string();
    }
    if a.iter().any(|s| s == "tilted" || s == "tilted-friction") {
        src = TILTED.replace(
            "detection=\"first_impact\" restitution=\"0.55\"",
            "detection=\"persistent_contact\" restitution=\"0\" max_contact_step_ns=\"250000\"",
        );
    }
    if a.iter().any(|s| s == "tilted") {
        src = src.replace("friction=\"0.9\"", "friction=\"0\"");
    }
    if a.iter().any(|s| s == "sag") {
        for line in CLOTH.lines().filter(|l| l.contains("<particle position=")) {
            let i = line.find(" mass=").unwrap() + 7;
            let j = i + line[i..].find('"').unwrap();
            src = src.replace(&line[i..j], "0.1075");
        }
    }
    if let Some(dt) = a.iter().find_map(|s| s.strip_prefix("width=")) {
        src = src.replace(
            "max_contact_step_ns=\"250000\"",
            &format!("max_contact_step_ns=\"{dt}\""),
        );
    }
    if a.iter().any(|s| s == "initial") {
        src = src.replace("0.712", "0.692");
    }
    let mut g = Gripper::new(&src)?;
    let mut sum = [DVec3::ZERO; 2];
    let mut end = Duration::ZERO;
    let mut count = 0;
    let mut min = f64::INFINITY;
    let mut max_v = 0.0f64;
    let steps = a
        .iter()
        .find_map(|s| s.strip_prefix("millis="))
        .map_or(Ok(600), str::parse::<u64>)?;
    while g.sim.elapsed() < Duration::from_millis(steps) {
        if let Err(e) = g.sim.step_exact(1) {
            return Err(format!(
                "{e} after {:?}, fault {:?}, active Through {end:?}, total impulse {sum:?}",
                g.sim.elapsed(),
                g.sim.fault()
            )
            .into());
        }
        let soft = deformable::bodies(&g.sim).next().unwrap().1;
        let nodes = soft.particles();
        let seg = Segment::new(nodes[0].position.into(), nodes[1].position.into());
        for id in [1, 2] {
            let c = DVec3::from(g.states()[id].position);
            let d = if nodes.len() == 2 {
                c.distance(seg.project_local_point(c, false).point)
            } else {
                [[0, 1, 3], [0, 3, 2]]
                    .map(|i| {
                        let [a, b, c0] = i.map(|i| DVec3::from(nodes[i].position));
                        c.distance(Triangle::new(a, b, c0).project_local_point(c, false).point)
                    })
                    .into_iter()
                    .fold(f64::INFINITY, f64::min)
            };
            min = min.min(d - 0.112);
        }
        if g.sim.elapsed() > Duration::from_millis(75) {
            max_v = max_v.max(
                nodes
                    .iter()
                    .map(|p| DVec3::from(p.velocity).length())
                    .fold(0., f64::max),
            );
        }
        let mut step = [DVec3::ZERO; 2];
        for s in soft.normal_support_this_step() {
            end = s.start + s.duration;
            count += 1;
            for a in s.beginning.iter().chain(&s.end) {
                for l in &a.material {
                    let j = DVec3::from(l.on_rigid);
                    step[l.site.sphere] += j;
                    sum[l.site.sphere] += j;
                }
            }
        }
        if !soft.impacts_this_step().is_empty() || g.sim.elapsed().as_millis() % 50 == 0 {
            println!(
                "{:?}: {} discrete impacts, {} support intervals this step, loads {:?} N, nodes z/vz {:?}",
                g.sim.elapsed(),
                soft.articulated_impact_count(),
                soft.normal_support_this_step().len(),
                step.map(|v| v / 0.001),
                nodes
                    .iter()
                    .map(|p| (p.position[2], p.velocity[2]))
                    .collect::<Vec<_>>()
            );
        }
    }
    println!(
        "through {:?}, {} loaded intervals, last support {:?}, accumulated two native-link impulses {:?} N s, min independent gap {min:e} m, largest nodal speed after 75ms {max_v:e} m/s",
        g.sim.elapsed(),
        count,
        end,
        sum
    );
    Ok(())
}
