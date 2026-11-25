// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Check a single gravity-loaded chain containing all six joint types at every committed step.
use glam::{DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    JointSample, Name, PhysicsEngine, UniversalJointSample, WorldPose, load_world_with_sdf_step,
    queue_universal_joint_effort, set_joint_force,
};
use serde_json::json;
use std::error::Error;

const LINKS: [&str; 6] = ["root", "bracket", "carriage", "swivel", "yoke", "tool"];
const JOINTS: [&str; 6] = ["shoulder", "weld", "slide", "socket", "wrist", "spindle"];
const SHOULDER: usize = 0;
const SLIDE: usize = 2;
const SOCKET: usize = 3;
const WRIST: usize = 4;
const SPINDLE: usize = 5;
const SDF: &str = include_str!("../../../examples/worlds/loaded_joint_chain.sdf");
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[allow(deprecated)]
fn entity(sim: &Simulation, short: &str) -> bevy_ecs::entity::Entity {
    let name = format!("default::chain::{short}");
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .expect("authored chain entity")
        .id()
}

fn measure(sdf: &str) -> Result<(serde_json::Value, bool)> {
    let mut sim = load_world_with_sdf_step(
        sdf,
        "loaded_joint_chain.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let links = LINKS.map(|n| entity(&sim, n));
    let joints = JOINTS.map(|n| entity(&sim, n));
    let ids = joints.map(|j| sim.world().get::<EntityId>(j).unwrap().get());
    let mut max_gap = [0_f64; 6];
    let mut max_rotation = [0_f64; 6];
    let mut peak_socket_swing = 0_f64;
    let mut peaks = [0_f64; 5];
    let mut peak_rates = [0_f64; 5];
    let mut endpoints = Vec::new();
    for sign in [1., -1.] {
        for (index, effort) in [(SHOULDER, 12.), (SLIDE, 9.), (SPINDLE, 0.14)] {
            set_joint_force(&mut sim, ids[index], sign * effort)?;
        }
        for _ in 0..4000 {
            queue_universal_joint_effort(&mut sim, ids[WRIST], [1.4 * sign, -1.1 * sign])?;
            sim.step_exact(1)?;
            let poses = links.map(|e| {
                let p = sim.world().get::<WorldPose>(e).unwrap();
                (DVec3::from(p.translation), DQuat::from_array(p.rotation))
            });
            let [shoulder, slide, spindle] = [SHOULDER, SLIDE, SPINDLE]
                .map(|i| sim.world().get::<JointSample>(joints[i]).unwrap().state);
            let wrist = sim
                .world()
                .get::<UniversalJointSample>(joints[WRIST])
                .unwrap()
                .state;
            let [a, b] = wrist.position;
            let positions = [shoulder.position, slide.position, a, b, spindle.position];
            let rates = [
                shoulder.velocity,
                slide.velocity,
                wrist.velocity[0],
                wrist.velocity[1],
                spindle.velocity,
            ];
            for (peak, value) in peaks.iter_mut().zip(positions) {
                *peak = peak.max(value.abs());
            }
            for (peak, rate) in peak_rates.iter_mut().zip(rates) {
                if !rate.is_finite() {
                    return Err("nonfinite joint rate".into());
                }
                *peak = peak.max(rate.abs());
            }
            // The joint frame starts at each child origin. Reconstruct allowed motion in the
            // current parent frame, which moves as the upstream shoulder, slide and socket turn.
            for i in 0..6 {
                let (parent, rotation) = if i == 0 {
                    (DVec3::new(0., 0., 1.4), DQuat::IDENTITY)
                } else {
                    poses[i - 1]
                };
                let (child, attitude) = poses[i];
                let rest = if i == 0 {
                    DVec3::ZERO
                } else {
                    -0.14 * DVec3::Z
                };
                let relative = rotation.inverse() * (child - parent) - rest;
                let travel = match i {
                    SLIDE => slide.position * DVec3::X,
                    SPINDLE => spindle.position * 0.04 / std::f64::consts::TAU * DVec3::Z,
                    _ => DVec3::ZERO,
                };
                let gap = relative.distance(travel);
                let measured = rotation.inverse() * attitude;
                let expected = match i {
                    SHOULDER => DQuat::from_rotation_y(shoulder.position),
                    SOCKET => {
                        peak_socket_swing =
                            peak_socket_swing.max(measured.angle_between(DQuat::IDENTITY));
                        measured
                    }
                    WRIST => DQuat::from_rotation_x(a) * DQuat::from_rotation_y(b),
                    SPINDLE => DQuat::from_rotation_z(spindle.position),
                    _ => DQuat::IDENTITY,
                };
                let error = expected.angle_between(measured);
                if !gap.is_finite() || !error.is_finite() {
                    return Err("nonfinite joint error".into());
                }
                max_gap[i] = max_gap[i].max(gap);
                max_rotation[i] = max_rotation[i].max(error);
            }
        }
        endpoints.push([SHOULDER, SLIDE, SPINDLE].map(|i| {
            sim.world()
                .get::<JointSample>(joints[i])
                .unwrap()
                .state
                .position
        }));
    }
    let orientation_errors: Vec<_> = max_rotation
        .iter()
        .enumerate()
        .map(|(i, error)| (i != SOCKET).then_some(error))
        .collect();
    let passed = max_gap.iter().all(|v| *v <= 0.001)
        && max_rotation.iter().all(|v| *v <= 0.01)
        && peak_socket_swing >= 0.1
        && peaks[0] >= 0.3
        && peaks[1] >= 0.04
        && peaks[2] >= 0.2
        && peaks[3] >= 0.2
        && endpoints[0][2] >= std::f64::consts::TAU
        && endpoints[1][2] <= -std::f64::consts::TAU;
    let report = json!({
        "passed":passed, "step_s": 0.00025, "steps": 8000, "total_moving_mass_kg": 1.5,
        "joints": JOINTS, "peak_anchor_or_travel_error_m": max_gap,
        "peak_constrained_rotation_error_rad": orientation_errors,
        "peak_ball_swing_rad": peak_socket_swing,
        "driven_coordinates": ["shoulder_rad", "slide_m", "wrist_x_rad", "wrist_y_rad", "spindle_rad"],
        "peak_abs_coordinate": peaks, "peak_abs_rate_per_s": peak_rates,
        "shoulder_slide_spindle_after_each_leg": endpoints,
    });
    Ok((report, passed))
}

fn main() -> Result<()> {
    let default_solver = SDF.replace(
        "<additional_pgs_iterations>4</additional_pgs_iterations>",
        "",
    );
    let (baseline, _) = measure(&default_solver)?;
    let (qualified, passed) = measure(SDF)?;
    let results = json!({"default_iterations": baseline, "four_extra_iterations": qualified});
    println!("{}", serde_json::to_string_pretty(&results)?);
    if !passed {
        return Err(
            "loaded chain exceeded its geometry tolerance or missed the commanded travel".into(),
        );
    }
    Ok(())
}
