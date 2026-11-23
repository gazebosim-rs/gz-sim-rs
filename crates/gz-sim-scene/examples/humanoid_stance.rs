// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Floating 30-joint bilateral reach under gravity. Usage: humanoid_stance
//! [reduced|impulse|passive] [kg_per_hand] [forward_impulse_Ns] [step_us]
//! [output_dir]. Optional output contains the actual SDF, summary, 20 ms
//! trajectory and visual samples. All physics/control observations run every tick.
#[path = "support/humanoid.rs"]
pub mod humanoid;
#[path = "support/pose_capture.rs"]
mod pose_capture;
#[path = "support/stance.rs"]
mod stance;

use bevy_ecs::entity::Entity;
use glam::{DQuat, DVec3};
use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_physics_rapier::RapierBackend;
use gz_sim_core::Simulation;
use gz_sim_scene::{BodyHandle, Name, PhysicsEngine, WorldPose, load_world_with_sdf_step};
use humanoid::Humanoid;
use pose_capture::Capture;
use serde_json::json;
use stance::{FLOOR, support};
use std::{error::Error, fs, path::PathBuf, time::Instant};

const SHOVE_US: u64 = 1_800_000;
const END_US: u64 = 3_000_000;
fn find(sim: &Simulation, key: &str) -> Entity {
    let name = format!("default::{key}");
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
fn position(sim: &Simulation, entity: Entity) -> DVec3 {
    DVec3::from_array(sim.world().get::<WorldPose>(entity).unwrap().translation)
}

fn main() -> Result<(), Box<dyn Error>> {
    let result = run(std::env::args().skip(1))?;
    println!("{}", serde_json::to_string(&result)?);
    Ok(())
}

fn run(mut args: impl Iterator<Item = String>) -> Result<serde_json::Value, Box<dyn Error>> {
    let mode = args.next().unwrap_or_else(|| "reduced".into());
    let engine = match mode.as_str() {
        "reduced" | "passive" => PhysicsEngine::MultiFidelity,
        "impulse" => PhysicsEngine::Rapier,
        _ => return Err("select reduced, impulse, or passive".into()),
    };
    let load: f64 = args.next().unwrap_or_else(|| "2".into()).parse()?;
    let shove: f64 = args.next().unwrap_or_else(|| "15".into()).parse()?;
    let step_us: u64 = args.next().unwrap_or_else(|| "1000".into()).parse()?;
    let output = args.next().map(PathBuf::from);
    if args.next().is_some()
        || !load.is_finite()
        || !(0.0..=10.).contains(&load)
        || !shove.is_finite()
        || shove.abs() > 100.0
        || !(50..=2000).contains(&step_us)
        || !20_000u64.is_multiple_of(step_us)
    {
        return Err("expected 0–10 kg per hand, |impulse| ≤ 100 N s and a 50–2000 µs timestep dividing 20 ms".into());
    }
    let dt = step_us as f64 * 1e-6;
    let reduced = engine == PhysicsEngine::MultiFidelity;
    let model = Humanoid::new().with_hand_load(load);
    let src = model.world([0., 0., -9.81], dt, reduced, FLOOR);
    let mut sim = load_world_with_sdf_step(
        &src,
        "stance.sdf",
        "default",
        &sdformat::NoResources,
        engine,
    )?;
    let control = model.bind(&sim);
    let initial_hands = control.hands_in_pelvis(&sim);
    let feet = [
        find(&sim, "humanoid::left_foot"),
        find(&sim, "humanoid::right_foot"),
    ];
    let footids = feet.map(|e| sim.world().get::<BodyHandle>(e).unwrap().id());
    let initial_feet = feet.map(|e| position(&sim, e));
    let floor = sim
        .world()
        .get::<BodyHandle>(find(&sim, "floor::body"))
        .unwrap()
        .id();
    let pelvis = find(&sim, "humanoid::pelvis");
    let hip = sim.world().get::<BodyHandle>(pelvis).unwrap().id();
    let mass = control.mass();
    let mut capture = output.as_ref().map(|_| {
        Capture::new(
            &sim,
            &["default::floor", "default::humanoid"],
            stance::paint,
        )
    });
    if let Some(c) = &mut capture {
        c.sample(&sim);
    }
    let mut rows = Vec::new();
    let mut both_ticks = 0;
    let mut other_ticks = 0;
    let mut peak_error = 0_f64;
    let mut peak_slip = 0_f64;
    let mut peak_tilt = 0_f64;
    let mut min_margin = f64::INFINITY;
    let mut peak_depth = 0_f64;
    let mut total_reaction = 0.;
    let mut tail_reaction = 0.;
    let mut tail_momentum_z = 0.;
    let mut peak_shove_speed = 0_f64;
    let mut tail_speed = 0_f64;
    let mut stable_from = None;
    let mut first_body_contact = None;
    let mut step_cost = Vec::with_capacity((END_US / step_us) as usize);
    let start = Instant::now();
    for tick in 1..=END_US / step_us {
        let before = (tick - 1) * step_us;
        let time = tick * step_us;
        let step_start = Instant::now();
        if before == SHOVE_US && shove != 0. {
            sim.world_mut()
                .resource_mut::<PhysicsBackend>()
                .add_world_wrench(hip, [shove / dt, 0., 0.], [0.; 3])?;
        }
        let error = if mode == "passive" {
            0.
        } else {
            control.drive(&mut sim)?
        };
        peak_error = peak_error.max(error);
        sim.step_exact(1)?;
        let backend = sim.world().resource::<PhysicsBackend>();
        let pairs = if reduced {
            backend
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap()
                .rigid_contact_observations()
        } else {
            backend
                .downcast_ref::<RapierBackend>()
                .unwrap()
                .rigid_contact_observations()
        };
        let mut reactions = [0.; 2];
        let mut other = false;
        for p in pairs.iter().filter(|p| p.active) {
            let other_id = if p.a == floor {
                p.b
            } else if p.b == floor {
                p.a
            } else {
                continue;
            };
            if let Some(index) = footids.iter().position(|&id| id == other_id) {
                reactions[index] += p.normal_impulse;
            } else {
                other = true;
            }
        }
        if other {
            other_ticks += 1;
            first_body_contact.get_or_insert(time);
        }
        let both = reactions.iter().all(|&v| v > 0.);
        both_ticks += u64::from(both);
        let center = control.center_of_mass(&sim);
        let momentum = control.momentum(&sim);
        let speed = (momentum / mass).truncate().length();
        let pose = sim.world().get::<WorldPose>(pelvis).unwrap();
        let tilt = (DQuat::from_array(pose.rotation) * DVec3::Z)
            .z
            .clamp(-1., 1.)
            .acos();
        let (margin, depth) = support(&sim, feet, center);
        if both {
            min_margin = min_margin.min(margin);
        }
        for (e, initial) in feet.into_iter().zip(initial_feet) {
            peak_slip = peak_slip.max((position(&sim, e) - initial).truncate().length());
        }
        peak_depth = peak_depth.max(depth);
        peak_tilt = peak_tilt.max(tilt);
        let reaction = reactions.iter().sum::<f64>();
        total_reaction += reaction;
        if time == 2_500_000 {
            tail_momentum_z = momentum.z;
        }
        if time > 2_500_000 {
            tail_reaction += reaction;
            tail_speed = tail_speed.max(speed);
        }
        if time > SHOVE_US {
            if time <= SHOVE_US + 100_000 {
                peak_shove_speed = peak_shove_speed.max(speed);
            }
            if speed < 0.03 && tilt < 0.035 && both && !other && margin > 0. {
                stable_from.get_or_insert(time);
            } else {
                stable_from = None;
            }
        }
        step_cost.push(step_start.elapsed().as_nanos() as u64);
        if time.is_multiple_of(20_000) && output.is_some() {
            let hands = control.hands_in_pelvis(&sim);
            rows.push(json!({"time_us":time,"com":center.to_array(),"com_v":(momentum/mass).to_array(),"tilt_rad":tilt,"foot_normal_impulse_ns":reactions,"support_margin_m":margin,"pelvis_z_m":pose.translation[2],"tracking_error_rad":error,"hand_forward_m":hands[0].x-initial_hands[0].x,"other_body_ground_contact":other}));
            capture.as_mut().unwrap().sample(&sim);
        }
    }
    let wall = start.elapsed().as_secs_f64();
    step_cost.sort_unstable();
    let steps = END_US / step_us;
    let (hold_error, hold_velocity) = control.hold_residual(&sim)?;
    let hands = control.hands_in_pelvis(&sim);
    let desired_hands = model.desired_hands(sim.elapsed().as_secs_f64());
    let goal_error = std::array::from_fn::<_, 2, _>(|i| (hands[i] - desired_hands[i]).length());
    let d_pz = control.momentum(&sim).z - tail_momentum_z;
    let weight = mass * 9.81;
    let recovery = stable_from
        .filter(|&t| END_US - t >= 100_000)
        .map(|t| (t - SHOVE_US) as f64 * 1e-6);
    let result = json!({
        "engine": mode,
        "step_us": step_us,
        "kg_per_hand": load,
        "shove_ns": shove,
        "mass_kg": mass,
        "steps": steps,
        "duration_s": 3.,
        "wall_s": wall,
        "observed_realtime_factor": 3. / wall,
        "ticks_p50_us": step_cost[step_cost.len()/2] as f64 / 1000.,
        "ticks_p99_us": step_cost[step_cost.len()*99/100] as f64 / 1000.,
        "reach_m": [hands[0].x-initial_hands[0].x, hands[1].x-initial_hands[1].x],
        "commanded_reach_m": desired_hands[0].x - initial_hands[0].x,
        "final_hand_target_error_m": goal_error,
        "peak_tracking_rad": (mode!="passive").then_some(peak_error),
        "final_tracking_rad": hold_error,
        "final_joint_speed_rad_s": hold_velocity,
        "both_feet_fraction": both_ticks as f64/steps as f64,
        "other_ground_ticks": other_ticks,
        "first_other_ground_us": first_body_contact,
        "max_pelvis_tilt_rad": peak_tilt,
        "max_foot_displacement_m": peak_slip,
        "min_double_support_margin_m": min_margin,
        "max_sole_penetration_m": peak_depth,
        "peak_horizontal_speed_after_shove_m_s": peak_shove_speed,
        "last_half_second_max_horizontal_speed_m_s": tail_speed,
        "stable_after_shove_s": recovery,
        "final_com_m": control.center_of_mass(&sim).to_array(),
        "total_sole_impulse_ns": total_reaction,
        "weight_n": weight,
        "final_half_second_normal_n": tail_reaction/0.5,
        "final_half_second_vertical_balance_residual_ns": tail_reaction-weight*0.5-d_pz,
    });
    if let Some(out) = output {
        fs::create_dir_all(&out)?;
        fs::write(out.join("stance.sdf"), src)?;
        fs::write(
            out.join("summary.json"),
            serde_json::to_vec_pretty(&result)?,
        )?;
        fs::write(out.join("trajectory.json"), serde_json::to_vec(&rows)?)?;
        fs::write(
            out.join("visuals.json"),
            serde_json::to_vec(&capture.unwrap().finish())?,
        )?;
    }
    Ok(result)
}

#[cfg(test)]
#[path = "support/stance_tests.rs"]
mod tests;
