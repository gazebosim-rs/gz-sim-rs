// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure complete Bevy public ticks on matched authored two-pad scenes.
use gz_sim_scene::{
    PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions, load_world_with_runtime,
    primary_contact::last_refinement,
};
use serde_json::json;
use std::{error::Error, fs, path::PathBuf, time::Instant};
struct Case {
    name: &'static str,
    source: &'static str,
    ticks: u64,
}
const CASES: [Case; 3] = [
    Case {
        name: "yaw",
        source: include_str!("../../../examples/multifidelity_spatial_shared_supports.sdf"),
        ticks: 120,
    },
    Case {
        name: "different_belts",
        source: include_str!("../../../examples/multifidelity_uniform_shared_slide.sdf"),
        ticks: 182,
    },
    Case {
        name: "common_belts",
        source: include_str!("../../../examples/multifidelity_comoving_support_arrest.sdf"),
        ticks: 180,
    },
];
fn cpu() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        let t = rustix::time::clock_gettime(rustix::time::ClockId::ThreadCPUTime);
        Some(t.tv_sec as u64 * 1_000_000_000 + t.tv_nsec as u64)
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}
fn ordinary(mut s: String) -> String {
    let p = s
        .find("<plugin name=\"gz::sim::systems::PrimaryContactRefinement\"")
        .expect("registered example");
    let n = s[p..].find("</plugin>").unwrap() + 9;
    s.replace_range(p..p + n, "");
    s
}
fn main() -> Result<(), Box<dyn Error>> {
    let file = PathBuf::from(
        std::env::args_os()
            .nth(1)
            .ok_or("supply output JSON filename")?,
    );
    let mut samples = vec![];
    for rep in 0..32 {
        for j in 0..3 {
            let case = &CASES[(j + rep / 2) % 3];
            for registered in [rep % 2 == 0, rep % 2 != 0] {
                let s = if registered {
                    case.source.to_owned()
                } else {
                    ordinary(case.source.to_owned())
                };
                let mut sim = load_world_with_runtime(
                    &s,
                    "cost.sdf",
                    "default",
                    &sdformat::NoResources,
                    WorldLoadOptions {
                        engine: PhysicsEngine::MultiFidelity,
                        step: None,
                        noise_seed: [0; 32],
                        distributed_worker: None,
                    },
                    WorldRuntimeOptions {
                        rapier_per_point_friction: true,
                        ..Default::default()
                    },
                )?;
                let mut wall = vec![];
                let mut thread = vec![];
                let mut mode = vec![];
                let mut states = vec![];
                for tick in 1..=case.ticks {
                    let before = cpu();
                    let now = Instant::now();
                    sim.step_exact(1)?;
                    wall.push(now.elapsed().as_nanos() as u64);
                    thread.push(cpu().zip(before).map(|(a, b)| a - b));
                    let s = last_refinement(&sim)
                        .filter(|r| r.iteration == tick)
                        .and_then(|r| r.interval.shared_faces.last());
                    mode.push(match s {
                        None => 0,
                        Some(p) if p.comoving_rest => 3,
                        Some(p) if p.slip_start_end.is_some() => 2,
                        Some(_) => 1,
                    });
                    states.push(s.map(|p| {
                        [
                            p.selected_fraction_b,
                            p.total_normal_impulse,
                            p.dissipated_energy,
                            p.support_tangent_work,
                        ]
                    }));
                }
                samples.push(json!({"case":case.name,"registered":registered,"rep":rep,"wall_ns":wall,"thread_cpu_ns":thread,"modes":mode,"selected_fraction_normal_heat_beltwork":states}));
            }
        }
    }
    fs::write(file, serde_json::to_vec(&samples)?)?;
    Ok(())
}
