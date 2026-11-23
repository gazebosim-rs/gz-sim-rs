// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time actual changing native states through the Bevy entry point, including static clutter.
use super::*;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, primary_contact};
use std::time::Instant;
const SDF: &str = include_str!("../../../../examples/multifidelity_loaded_deck_edge.sdf");
fn xml(obstacles: usize, mode: &str, sign: f64, mu: f64) -> String {
    use std::fmt::Write;
    let mut clutter = String::new();
    for i in 0..obstacles {
        let x = 3. + 0.8 * (i % 32) as f64;
        let y = 0.5 + 0.8 * (i / 32) as f64;
        write!(&mut clutter,r#"<model name="obstacle_{i}"><static>true</static><pose>{x} {y} 0.2 0 0 0</pose><link name="b"><collision name="c"><geometry><box><size>0.2 0.2 0.4</size></box></geometry></collision></link></model>"#).unwrap();
    }
    let mut source = SDF
        .replace("3.22 0 -1.1", &format!("{} 0 -1.1", sign * 3.22))
        .replace("0.38 0 -0.06", &format!("{} 0 -0.06", sign * 0.38))
        .replace(
            "<mu>0</mu><mu2>0</mu2>",
            &format!("<mu>{mu}</mu><mu2>{mu}</mu2>"),
        )
        .replace("</world>", &format!("{clutter}</world>"));
    if mode == "patch_only" {
        source = source.replace("<follow_planar_edge>true", "<follow_planar_edge>false");
    }
    source
}
pub(super) fn run(out: &Path) -> Result<()> {
    let modes = ["ordinary", "patch_only", "continuous"];
    #[derive(Default)]
    struct Timing {
        ticks: Vec<u64>,
        active: Vec<u64>,
        inactive: Vec<u64>,
        impact: Vec<u64>,
        admitted: u64,
        observed: u64,
        end_digest: f64,
    }
    let mut rows = vec![];
    for obstacles in [0, 16, 128, 512] {
        let mut measures: [Timing; 3] = Default::default();
        for repeat in 0..15 {
            let sign = if repeat % 2 == 0 { 1. } else { -1. };
            let mu = [0., 0.2, 0.6][repeat % 3];
            for offset in 0..3 {
                let i = (offset + repeat) % 3;
                let mode = modes[i];
                let mut s = load_world_with_sdf_step(
                    &xml(obstacles, mode, sign, mu),
                    "budget.sdf",
                    "default",
                    &sdformat::NoResources,
                    PhysicsEngine::MultiFidelity,
                )?;
                if i == 0 {
                    primary_contact::set_enabled(&mut s, false)?;
                }
                let data = &mut measures[i];
                for tick in 1..=33 {
                    let now = Instant::now();
                    s.step_exact(1)?;
                    let elapsed = now.elapsed().as_nanos() as u64;
                    if repeat < 3 {
                        continue;
                    }
                    if tick == 1 {
                        data.impact.push(elapsed);
                    } else {
                        data.ticks.push(elapsed);
                    }
                    let mut admitted = false;
                    if let Some(r) =
                        primary_contact::last_refinement(&s).filter(|r| r.iteration == tick)
                    {
                        admitted = !r.interval.sustained_edges.is_empty();
                        data.admitted += u64::from(admitted);
                        data.end_digest += r
                            .interval
                            .sustained_edges
                            .iter()
                            .map(|q| q.compressive_impulse)
                            .sum::<f64>();
                    }
                    if tick > 1 {
                        if admitted {
                            data.active.push(elapsed);
                        } else {
                            data.inactive.push(elapsed);
                        }
                    }
                    data.observed += 1;
                }
            }
        }
        fn stats(v: &mut [u64]) -> Value {
            if v.is_empty() {
                return Value::Null;
            }
            v.sort_unstable();
            let q = |p: usize| v[(p * v.len()).div_ceil(100) - 1];
            json!({"n":v.len(),"median_ns":q(50),"p95_ns":q(95),"p99_ns":q(99),"max_ns":v.last().unwrap()})
        }
        for (mut t, mode) in measures.into_iter().zip(modes) {
            rows.push(json!({"static_boxes":obstacles,"mode":mode,"impact":stats(&mut t.impact),"continued_ticks":stats(&mut t.ticks),"admitted_ticks":stats(&mut t.active),"other_ticks":stats(&mut t.inactive),"admitted":t.admitted,"observed":t.observed,"sum_sustained_ns":t.end_digest}));
        }
    }
    fs::create_dir_all(out)?;
    write_json(
        &out.join("cost.json"),
        &json!({"discarded_warmup_repetitions":3,"rows":rows}),
    )
}
