// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_multifidelity::{ArticulationTier, BallJointState};
use gz_sim_core::Simulation;
use gz_sim_scene::{WorldPose, model_fidelity, set_model_fidelity};

use std::time::Instant;

type Sample = ([f64; 3], BallJointState);

pub(super) const HANGING: &str =
    include_str!("../../../../examples/worlds/local_hanging_ball_wrist.sdf");
const RESUME: usize = 351;
const END: usize = 1500;

fn load(xml: &str) -> Result<Simulation, Box<dyn Error>> {
    Ok(load_world_with_sdf_step(
        xml,
        "hanging-wrist.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}

struct Wrist {
    model: u64,
    bearing: u64,
    tool: bevy_ecs::entity::Entity,
    joint: bevy_ecs::entity::Entity,
}
impl Wrist {
    fn bind(sim: &Simulation) -> Result<Self, Box<dyn Error>> {
        let part = |name: &str| {
            sim.world()
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
                .ok_or("missing wrist part")
        };
        let model = part("default::wrist")?
            .get::<EntityId>()
            .ok_or("missing model ID")?
            .get();
        let joint = part("default::wrist::bearing")?;
        Ok(Self {
            model,
            bearing: joint.get::<EntityId>().ok_or("missing bearing ID")?.get(),
            joint: joint.id(),
            tool: part("default::wrist::tool")?.id(),
        })
    }
    fn state(&self, sim: &Simulation) -> Result<Sample, Box<dyn Error>> {
        let w = sim.world();
        Ok((
            w.get::<WorldPose>(self.tool)
                .ok_or("missing tool pose")?
                .translation,
            w.get::<BallJointSample>(self.joint)
                .ok_or("missing bearing sample")?
                .state,
        ))
    }
}

struct Trial {
    states: Vec<Sample>,
    parked_phase_p95_ns: u128,
}

fn episode(
    sim: &mut Simulation,
    park: bool,
    mut capture: impl FnMut(&Simulation) -> Result<(), Box<dyn Error>>,
) -> Result<Trial, Box<dyn Error>> {
    let wrist = Wrist::bind(sim)?;
    let mut states = Vec::with_capacity(END);
    let mut costs = Vec::with_capacity(300);
    capture(sim)?;
    for tick in 1..=END {
        if park && tick == 51 {
            set_model_fidelity(sim, wrist.model, ArticulationTier::Prescribed)?;
        }
        if (RESUME..RESUME + 200).contains(&tick) {
            queue_ball_joint_effort(sim, wrist.bearing, [0.55, 1.2, 0.07])?;
            if tick == RESUME {
                let before = wrist.state(sim)?;
                sim.set_paused(true);
                sim.tick()?;
                sim.set_paused(false);
                if wrist.state(sim)? != before {
                    return Err("paused torque advanced the wrist".into());
                }
            }
        }
        let start = Instant::now();
        sim.step_exact(1)?;
        let cost = start.elapsed().as_nanos();
        let hold = (51..RESUME).contains(&tick);
        if hold {
            costs.push(cost);
        }
        if (model_fidelity(sim, wrist.model)? == ArticulationTier::Prescribed) != (park && hold) {
            return Err(format!("unexpected wrist tier at tick {tick}").into());
        }
        states.push(wrist.state(sim)?);
        if tick % 10 == 0 {
            capture(sim)?;
        }
    }
    costs.sort_unstable();
    Ok(Trial {
        states,
        parked_phase_p95_ns: costs[(costs.len() * 95).div_ceil(100) - 1],
    })
}

// A rotation and its negated quaternion represent the same physical orientation.
fn separation(q: glam::DQuat) -> f64 {
    2.0 * q.xyz().length().atan2(q.w.abs())
}

pub(super) fn record(path: &Path, xml: &str) -> Result<(), Box<dyn Error>> {
    let sdf = sdformat::load_str(xml, "hanging-wrist.sdf", &sdformat::NoResources)?;
    let mut switched = load(xml)?;
    let mut frames = Vec::new();
    let parked = episode(&mut switched, true, |sim| {
        frames.push(gz_sim_cli::primitive_frame(
            sim,
            &sdf,
            "default",
            frames.len() as u64 + 1,
        )?);
        Ok(())
    })?;
    let control = episode(&mut load(xml)?, false, |_| Ok(()))?;
    gz_sim_scene::reset_loaded_world(&mut switched, |_| ())?;
    let reset = episode(&mut switched, true, |_| Ok(()))?;
    if reset.states != parked.states {
        return Err("loaded reset changed the wrist trajectory".into());
    }
    let (mut position_error, mut angle_error, mut spin_error, mut swing) =
        (0_f64, 0_f64, 0_f64, 0_f64);
    for ((p, j), (q, k)) in parked.states.iter().zip(&control.states) {
        let qa = glam::DQuat::from_array(j.orientation);
        let qb = glam::DQuat::from_array(k.orientation);
        position_error = position_error.max(DVec3::from_array(*p).distance((*q).into()));
        angle_error = angle_error.max(separation(qa.conjugate() * qb));
        spin_error =
            spin_error.max(DVec3::from(j.angular_velocity).distance(k.angular_velocity.into()));
        swing = swing.max(separation(qa));
    }
    let first_spin = parked.states[RESUME - 1].1.angular_velocity;
    if position_error > 1e-5
        || angle_error > 1e-4
        || spin_error > 1e-3
        || DVec3::from(first_spin).length() < 1e-4
        || swing < 0.05
    {
        return Err(format!("wrist comparison missed its limits: {position_error} m, {angle_error} rad, {spin_error} rad/s, swing {swing}").into());
    }
    println!(
        "{}",
        serde_json::json!({"ticks":END,"parked_ticks":RESUME-51,"reset_states":reset.states.len(),"first_resumed_tick":RESUME,"first_resumed_angular_velocity_rad_s":first_spin,"max_tool_position_error_m":position_error,"max_bearing_orientation_error_rad":angle_error,"max_bearing_velocity_error_rad_s":spin_error,"max_rotation_rad":swing,"rest_phase_p95_ns":{"parked":parked.parked_phase_p95_ns,"dynamic":control.parked_phase_p95_ns,"reset_parked":reset.parked_phase_p95_ns},"visual_frames":frames.len()})
    );
    gz_sim_cli::export_archive(path, Default::default(), frames)?;
    Ok(())
}

#[cfg(test)]
#[path = "mounted_tests.rs"]
mod tests;
