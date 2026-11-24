// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_scene::deformable::{bodies, material_energy};

const WORLD: &str = include_str!("../../../../examples/worlds/local_damped_cloth.sdf");
const NAMES: [&str; 3] = ["elastic", "light", "heavy"];
const TICKS: usize = 900;

fn positions(sim: &Simulation) -> Result<Vec<[f64; 3]>> {
    let material = bodies(sim).collect::<std::collections::BTreeMap<_, _>>();
    let mut points = Vec::with_capacity(12);
    for name in NAMES {
        let particles = material
            .get(name)
            .ok_or("missing measured cloth")?
            .particles();
        if particles.len() != 4 {
            return Err("expected one free corner on each cloth".into());
        }
        points.extend(particles.iter().map(|p| p.position));
    }
    Ok(points)
}

pub(super) fn run(out: &Path) -> Result<()> {
    let mut r = Recording::new(WORLD, "local_damped_cloth.sdf", "default")?;
    let mut initial = [0.; 3];
    let mut final_energy = [0.; 3];
    let mut early = [0_f64; 3];
    let mut late = [0_f64; 3];
    let mut frames = Vec::new();
    let mut first = Vec::with_capacity(TICKS + 1);
    for pass in 0..2 {
        if pass == 1 {
            r.sim.reset()?;
        }
        for tick in 0..=TICKS {
            if tick > 0 {
                if pass == 0 {
                    r.advance()?
                } else {
                    r.sim.step_exact(1)?;
                }
            }
            let points = positions(&r.sim)?;
            if pass == 1 {
                if points != first[tick] {
                    return Err("cloth reset changed a measured particle".into());
                }
                continue;
            }
            for (i, name) in NAMES.into_iter().enumerate() {
                let p = points[4 * i + 1];
                let displacement = (p[2] - 0.23).abs();
                if tick < 200 {
                    early[i] = early[i].max(displacement);
                }
                if tick > 700 {
                    late[i] = late[i].max(displacement);
                }
                if tick == 0 || tick == TICKS {
                    let energy = material_energy(&r.sim, name)
                        .ok_or("missing cloth energy")??
                        .total();
                    if tick == 0 {
                        initial[i] = energy;
                    } else {
                        final_energy[i] = energy;
                    }
                }
            }
            if tick % 5 == 0 {
                r.capture()?;
                frames.push(json!({"time_ns":tick*1_000_000,"positions":points}));
            }
            first.push(points);
        }
    }
    if !(late[0] > late[1]
        && late[1] > late[2]
        && final_energy[0] > final_energy[1]
        && final_energy[1] > final_energy[2]
        && early.iter().all(|&v| v > 0.005))
    {
        return Err("the three measured decay envelopes did not separate".into());
    }
    r.save(out)?;
    write_json(&out.join("particle-frames.json"), &json!(frames))?;
    let labels = NAMES
        .iter()
        .flat_map(|n| (0..4).map(move |i| format!("default::{n}::markers::node_{i}")))
        .collect::<Vec<_>>();
    write_json(
        &out.join("frames.json"),
        &json!({"kind":"authored-archive-v1","camera":"right","dt_ns":1,"particle_visuals":labels,"captures":{"initial":0,"first_peak":10,"second_cycle":50,"decay":140,"end":180}}),
    )?;
    write_json(
        &out.join("damping.json"),
        &json!({"materials":NAMES,"coefficient_nms_per_rad":[0.,0.08,0.30],"public_step_ns":1_000_000,"material_substeps":4,"reset_states":first.len(),"initial_energy_j":initial,"final_energy_j":final_energy,"peak_first_200ms_m":early,"peak_final_200ms_m":late}),
    )?;
    fs::write(out.join("world.sdf"), WORLD)?;
    Ok(())
}

#[cfg(test)]
#[path = "hinge_damping_tests.rs"]
mod tests;
