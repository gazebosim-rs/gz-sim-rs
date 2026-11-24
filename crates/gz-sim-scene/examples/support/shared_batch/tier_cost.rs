// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Public-tick cost and measured support on a synchronized park / resume workload.
use super::*;
use gz_sim_scene::{
    fidelity_zone::{self, FidelityZone},
    model_fidelity, queue_link_world_velocity, support_loss_promotion,
};
use std::time::Duration;
pub(super) fn run(out: &Path) -> Result<()> {
    let mut rows = vec![];
    for rep in 0..5 {
        for step in 0..4 {
            let count = [1, 4, 16, 48][(step + rep) % 4];
            let stations = (0..count).collect::<Vec<_>>();
            for switched in [rep % 2 == 0, rep % 2 == 1] {
                let mut rig = Rig::new(&stations, true, 0.72, true)?;
                let mut pads = vec![];
                for (i, station) in rig.ids.iter().enumerate() {
                    if switched {
                        fidelity_zone::configure_with_eligibility_dwell(
                            &mut rig.sim,
                            station.0,
                            vec![FidelityZone {
                                center: [99., 0., 0.],
                                promote_radius: 0.2,
                                demote_radius: 1.,
                            }],
                            Duration::ZERO,
                            Duration::from_millis(2),
                        )?;
                    }
                    pads.push(
                        ["lift", "toe"]
                            .map(|n| bind(&rig.sim, &format!("default::{n}_{i}::pad")).0),
                    );
                }
                let mut samples = vec![];
                let mut wakes = 0;
                let mut previous = 0;
                for t in 1..=65 {
                    if t == 18 || t == 32 {
                        let v = if t == 18 {
                            velocity(1).normalize() * (0.31 * 9.81 * 0.00321381976)
                        } else {
                            DVec3::ZERO
                        };
                        for pair in &pads {
                            for id in pair {
                                queue_link_world_velocity(
                                    &mut rig.sim,
                                    *id,
                                    v.to_array(),
                                    [0.; 3],
                                )?;
                            }
                        }
                    }
                    let before = thread_clock_ns();
                    let now = Instant::now();
                    rig.sim.step_exact(1)?;
                    let wall = now.elapsed().as_nanos() as u64;
                    let cpu = thread_clock_ns().zip(before).map(|(a, b)| a - b);
                    let held = rig
                        .ids
                        .iter()
                        .map(|s| model_fidelity(&rig.sim, s.0))
                        .collect::<std::result::Result<Vec<_>, _>>()?
                        .iter()
                        .filter(|tier| **tier == Prescribed)
                        .count();
                    let wake = rig
                        .ids
                        .iter()
                        .filter(|s| {
                            support_loss_promotion::last_promotion(&rig.sim, s.0)
                                .is_some_and(|v| v.iteration == t)
                        })
                        .count();
                    let loads = rig.loads(t);
                    let weight = loads.iter().flatten().map(|p| p[0] + p[1]).sum::<f64>();
                    wakes += wake;
                    if switched && (t == 18 && wake != count || t == 65 && held != count) {
                        return Err(format!("unexpected synchronized command: tick {t} held {held} wakes {wake} for {count}").into());
                    }
                    let kind = if wake > 0 {
                        "wake"
                    } else if held > previous {
                        "demote"
                    } else if held == count {
                        "parked"
                    } else if t < 7 || (39..=43).contains(&t) {
                        "dwell"
                    } else {
                        "dynamic"
                    };
                    previous = held;
                    samples.push(json!({"tick":t,"phase":kind,"held":held,"wake":wake,"total_j_ns":weight,"registered":loads.iter().flatten().count(),"wall_ns":wall,"thread_cpu_ns":cpu}));
                }
                rows.push(json!({"rep":rep,"stations":count,"switched":switched,"wakes":wakes,"samples":samples}));
            }
        }
    }
    fs::write(out.join("tiers.json"), serde_json::to_vec(&rows)?)?;
    Ok(())
}
