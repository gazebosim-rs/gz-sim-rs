// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Observe a free-spinning five-hinge solar array or translating 30-joint
//! humanoid across whole-assembly fidelity boundaries. Usage:
//! articulation_coast [satellite|humanoid] [coast|native] [output_dir].
#[path = "support/coasting.rs"]
mod coasting;
#[path = "support/humanoid.rs"]
pub mod humanoid;
#[path = "support/pose_capture.rs"]
mod pose_capture;
use coasting::{ARRAY, Mechanism};
use glam::{DQuat, DVec3};
use gz_sim_core::Simulation;
use gz_sim_scene::{
    ArticulationTier, PhysicsEngine, WorldPose, begin_model_free_coast, load_world_with_sdf_step,
    model_fidelity, queue_joint_effort, queue_link_world_wrench,
};
use pose_capture::Capture;
use serde_json::{Value, json};
use std::{error::Error, fs, path::PathBuf, time::Instant};

fn load(xml: &str) -> Result<Simulation, Box<dyn Error>> {
    Ok(load_world_with_sdf_step(
        xml,
        "coast.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args().skip(1);
    let model = args.next().unwrap_or_else(|| "satellite".into());
    let mode = args.next().unwrap_or_else(|| "coast".into());
    let out = args.next().map(PathBuf::from);
    if args.next().is_some() || !matches!(mode.as_str(), "coast" | "native") {
        return Err(
            "usage: articulation_coast [satellite|humanoid] [coast|native] [output_dir]".into(),
        );
    }
    println!("{}", run(&model, &mode, out)?);
    Ok(())
}
fn paint(name: &str) -> [f64; 3] {
    if name.contains("tier_indicator") {
        [0.06, 0.94, 0.56]
    } else if name.contains("panel") {
        [0.1, 0.46, 0.75]
    } else if name.contains("boom") {
        [0.87, 0.61, 0.20]
    } else if name.contains("turntable") {
        [0.03, 0.69, 0.53]
    } else {
        [0.64, 0.73, 0.79]
    }
}
fn record(c: &mut Capture, sim: &Simulation, name: &str, model: u64) {
    if name == "satellite" && model_fidelity(sim, model).unwrap() == ArticulationTier::Prescribed {
        c.sample_with_offset(
            sim,
            Some((
                "default::satellite::turntable::tier_indicator",
                [0., 0., 0.085],
            )),
        );
    } else {
        c.sample(sim);
    }
}

fn run(name: &str, mode: &str, out: Option<PathBuf>) -> Result<Value, Box<dyn Error>> {
    let (xml, omega, braking) = match name {
        "satellite" => (ARRAY.to_owned(), 1., 1000),
        "humanoid" => (humanoid::Humanoid::new().world_sdf(""), 0., 0),
        _ => return Err("choose satellite or humanoid".into()),
    };
    let mut sim = load(&xml)?;
    let rig = Mechanism::bind(&sim, &xml, name);
    let mut capture = out
        .as_ref()
        .map(|_| Capture::new(&sim, &[&format!("default::{name}")], paint));
    if let Some(c) = &mut capture {
        record(c, &sim, name, rig.model);
    }
    rig.prepare(&mut sim, omega, braking, |s, tick| {
        if tick.is_multiple_of(20)
            && tick < braking
            && let Some(c) = &mut capture
        {
            record(c, s, name, rig.model);
        }
    })?;
    let initial = rig.sample(&sim);
    let transit = Instant::now();
    if mode == "coast" {
        begin_model_free_coast(&mut sim, rig.model)?;
        assert_eq!(rig.sample(&sim), initial);
    }
    if let Some(c) = &mut capture {
        record(c, &sim, name, rig.model);
    }
    let demote_us = transit.elapsed().as_secs_f64() * 1e6;
    let mut max_center = 0_f64;
    let mut max_p = 0_f64;
    let mut max_l = 0_f64;
    let mut max_energy = 0_f64;
    let mut max_q = 0_f64;
    let mut max_qdot = 0_f64;
    let mut trace = Vec::new();
    let transit_start = sim.elapsed();
    let wall = Instant::now();
    for tick in 1..=2000 {
        sim.step_exact(1)?;
        let s = rig.sample(&sim);
        let elapsed = (sim.elapsed() - transit_start).as_secs_f64();
        let center = initial.center + elapsed * initial.momentum / rig.mass();
        max_center = max_center.max(s.center.distance(center));
        max_p = max_p.max(s.momentum.distance(initial.momentum));
        max_l = max_l.max(s.angular.distance(initial.angular));
        max_energy = max_energy.max((s.energy - initial.energy).abs());
        for (a, b) in s.joints.iter().zip(&initial.joints) {
            max_q = max_q.max((a.position - b.position).abs());
            max_qdot = max_qdot.max(a.velocity.abs());
        }
        if tick % 20 == 0 && out.is_some() {
            record(capture.as_mut().unwrap(), &sim, name, rig.model);
            trace.push(json!({"t":sim.elapsed().as_secs_f64(),"time_since_coast":elapsed,"center":s.center.to_array(),"root_q":s.links[0].orientation,"momentum":s.momentum.to_array(),"angular_momentum":s.angular.to_array(),"energy":s.energy,"joints":s.joints.iter().map(|j|[j.position,j.velocity]).collect::<Vec<_>>(),"tier":format!("{:?}",model_fidelity(&sim,rig.model)?)}));
        }
    }
    let coast_s = wall.elapsed().as_secs_f64();
    let pre_wake = rig.sample(&sim);
    let pose = sim
        .world()
        .get::<WorldPose>(coasting::root_entity(&sim, &rig))
        .unwrap();
    let turned = DQuat::from_array(initial.links[0].orientation)
        .angle_between(DQuat::from_array(pose.rotation));
    queue_link_world_wrench(&mut sim, rig.links[0].0, [0., 80., 0.], [0.; 3])?;
    sim.tick()?;
    assert_eq!(rig.sample(&sim), pre_wake);
    let wake = Instant::now();
    sim.step_exact(1)?;
    let wake_us = wake.elapsed().as_secs_f64() * 1e6;
    assert_eq!(model_fidelity(&sim, rig.model)?, ArticulationTier::Dynamic);
    if let Some(c) = &mut capture {
        record(c, &sim, name, rig.model);
    }
    let post_wake = rig.sample(&sim);
    let wake_error = (post_wake.momentum - pre_wake.momentum - DVec3::new(0., 0.08, 0.)).length();
    // Actuate the distal joint after promotion to expose stale dormant axes.
    queue_joint_effort(&mut sim, rig.joints.last().unwrap().0, 8.)?;
    for tick in 1..=300 {
        sim.step_exact(1)?;
        if tick % 20 == 0
            && let Some(c) = &mut capture
        {
            record(c, &sim, name, rig.model);
        }
    }
    let final_sample = rig.sample(&sim);
    assert_eq!(Mechanism::bind(&sim, &xml, name), rig);
    let result = json!({
        "model":name,"mode":mode,"links":rig.links.len(),"joints":rig.joints.len(),
        "step_ms":1,"coast_ticks":2000,"coast_wall_s":coast_s,"observed_realtime_factor":2./coast_s,
        "switch_and_check_us":demote_us,"first_disturbed_tick_us":wake_us,
        "peak_com_error_m":max_center,"peak_momentum_error_kg_m_s":max_p,
        "peak_angular_momentum_error_kg_m2_s":max_l,"peak_energy_error_j":max_energy,
        "peak_joint_drift_rad":max_q,"peak_joint_rate_rad_s":max_qdot,
        "root_turn_rad":turned,"travel_m":pre_wake.center.distance(initial.center),
        "first_disturbed_tick_momentum_error":wake_error,
        "last_joint_velocity_after_actuation":final_sample.joints.last().unwrap().velocity,
        "final_root_position":final_sample.links[0].position,
    });
    if let Some(dir) = out {
        fs::create_dir_all(&dir)?;
        fs::write(
            dir.join("visuals.json"),
            serde_json::to_vec(&capture.unwrap().finish())?,
        )?;
        fs::write(dir.join("trajectory.json"), serde_json::to_vec(&trace)?)?;
        fs::write(
            dir.join("summary.json"),
            serde_json::to_vec_pretty(&result)?,
        )?;
    }
    Ok(result)
}

#[cfg(test)]
#[path = "support/coast_tests.rs"]
mod tests;

#[cfg(test)]
#[path = "support/coast_reference_tests.rs"]
mod reference_tests;

#[cfg(test)]
#[path = "support/coast_lifecycle_tests.rs"]
mod lifecycle_tests;
