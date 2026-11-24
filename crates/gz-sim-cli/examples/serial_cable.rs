// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Carry a free XPBD cable across a finite barrier with a screw-driven gripper.
#![allow(deprecated)]
#[path = "serial_cable/geometry.rs"]
mod geometry;
#[path = "serial_cable/measure.rs"]
mod measure;
#[path = "serial_cable/snag.rs"]
mod snag;
use bevy_ecs::entity::Entity;
use geometry::BoxShape;
use glam::{DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    JointForceCommand, JointSample, Name, PhysicsEngine, WorldPose, deformable,
    load_world_with_sdf_step, set_joint_controller_command,
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::{error::Error, path::Path};

const SDF: &str = include_str!("../../../examples/worlds/local_serial_cable.sdf");
const DT: f64 = 0.001;
const WEIGHT: f64 = 11. * 0.002 * 9.81;
const RADIUS: f64 = 0.025;
const SPACING: f64 = 0.022;
const END: usize = 14000;
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Clone, Copy)]
struct Joint {
    entity: Entity,
    id: u64,
}
fn joint(sim: &Simulation, name: &str) -> Result<Joint> {
    let full = format!("default::gripper::{name}");
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == full))
        .ok_or(full)?;
    Ok(Joint {
        entity: e.id(),
        id: e.get::<EntityId>().ok_or("missing joint ID")?.get(),
    })
}
struct Robot {
    rail: Joint,
    spindle: Joint,
    wrist: Joint,
    jaws: [Joint; 2],
    pads: [Joint; 2],
    bar: BoxShape,
    shelf: BoxShape,
    other: Vec<BoxShape>,
    pad_boxes: [BoxShape; 2],
}
impl Robot {
    fn load(sim: &Simulation, sdf: &sdformat::Sdf) -> Result<Self> {
        let other = [
            "gripper::palm",
            "gripper::nut",
            "gripper::carriage",
            "overhead::rail",
        ]
        .map(|n| BoxShape::find(sim, sdf, n))
        .into_iter()
        .collect::<Result<_>>()?;
        Ok(Self {
            pad_boxes: [
                BoxShape::find(sim, sdf, "gripper::left")?,
                BoxShape::find(sim, sdf, "gripper::right")?,
            ],
            other,
            bar: BoxShape::find(sim, sdf, "obstacle::bar")?,
            shelf: BoxShape::find(sim, sdf, "receiver::shelf")?,
            rail: joint(sim, "travel")?,
            spindle: joint(sim, "spindle")?,
            wrist: joint(sim, "wrist")?,
            jaws: [joint(sim, "close_left")?, joint(sim, "close_right")?],
            pads: [joint(sim, "left")?, joint(sim, "right")?],
        })
    }
    fn drive(
        &self,
        sim: &mut Simulation,
        tick: usize,
        open: bool,
        release: Option<usize>,
        lift: f64,
    ) -> Result<()> {
        if tick == 900 {
            set_joint_controller_command(sim, self.spindle.id, lift)?;
        }
        if tick == 2400 {
            set_joint_controller_command(sim, self.wrist.id, 0.045)?;
        }
        if tick == 5600 {
            set_joint_controller_command(sim, self.wrist.id, 0.)?;
        }
        if release.is_some_and(|when| tick == when + 375) {
            set_joint_controller_command(sim, self.spindle.id, lift)?;
        }
        if tick == 9600 {
            set_joint_controller_command(sim, self.spindle.id, 0.70)?;
        }
        let target = if open || release.is_some() {
            0.
        } else {
            0.034 * (tick as f64 / 400.).min(1.)
        };
        let jaw_effort = 300. * (target - 0.048);
        let mut drive = 0.;
        if tick > 1 {
            let t = (tick.saturating_sub(5400) as f64 / 4000.).min(1.);
            let goal = 0.46 * t.powi(3) * (10. + t * (-15. + 6. * t));
            let speed = 0.46 * 30. * t.powi(2) * (1. - t).powi(2) / 4.;
            let rail = sim
                .world()
                .get::<JointSample>(self.rail.entity)
                .ok_or("missing rail state")?
                .state;
            drive = 100. * (goal - rail.position) + 28. * (speed - rail.velocity);
        }
        JointForceCommand::set_batch(
            sim,
            [
                (self.jaws[0].id, jaw_effort),
                (self.jaws[1].id, jaw_effort),
                (self.rail.id, drive.clamp(-80., 80.)),
            ],
        )?;
        Ok(())
    }
}
#[derive(Default, Debug, Clone, PartialEq, Serialize)]
struct Sample {
    particle_state_sha256: String,
    center: [f64; 3],
    source: f64,
    shelf: f64,
    bar: f64,
    jaw_up: [f64; 2],
    squeeze: [f64; 2],
    clearance: f64,
    near_bar: bool,
    other_gap: f64,
    robot_bar_gap: f64,
    jaw_friction_excess_n: f64,
    shelf_margin: f64,
    tip_sag: f64,
    wrist: [f64; 2],
    spindle: f64,
    release_commanded: bool,
    min_z: f64,
    peak_speed: f64,
    strain: f64,
    momentum: [f64; 3],
    residual: f64,
}
fn sample(sim: &Simulation, robot: &Robot, prev: DVec3) -> Result<Sample> {
    sample_payload(sim, robot, prev, [0.; 3], 0.7)
}
fn sample_payload(
    sim: &Simulation,
    robot: &Robot,
    prev: DVec3,
    external_n: [f64; 3],
    friction: f64,
) -> Result<Sample> {
    let (_, cable) = deformable::bodies(sim)
        .find(|(n, _)| *n == "cable")
        .ok_or("no cable")?;
    let particles = cable.particles();
    let [left, right, source, shelf, bar] = cable.last_boundary_impulses() else {
        return Err("expected two jaws and three finite boundaries".into());
    };
    let mut s = Sample {
        source: -source.linear[2] / DT,
        shelf: -shelf.linear[2] / DT,
        bar: DVec3::from(bar.linear).length() / DT,
        min_z: f64::INFINITY,
        clearance: f64::INFINITY,
        ..Default::default()
    };
    let bar_shape = robot.bar.at(sim)?;
    let shelf_shape = robot.shelf.at(sim)?;
    let other = robot
        .other
        .iter()
        .map(|g| g.at(sim))
        .collect::<Result<Vec<_>>>()?;
    // The first three are moving robot links; the overhead rail is static.
    s.robot_bar_gap = f64::INFINITY;
    for g in &other[..3] {
        s.robot_bar_gap = s.robot_bar_gap.min(g.box_gap(&bar_shape)?);
    }
    for pad in &robot.pad_boxes {
        s.robot_bar_gap = s.robot_bar_gap.min(pad.at(sim)?.box_gap(&bar_shape)?);
    }
    s.other_gap = f64::INFINITY;
    s.shelf_margin = f64::INFINITY;
    s.wrist = sim
        .world()
        .get::<gz_sim_scene::UniversalJointSample>(robot.wrist.entity)
        .ok_or("missing wrist coordinates")?
        .state
        .position;
    s.spindle = sim
        .world()
        .get::<JointSample>(robot.spindle.entity)
        .ok_or("missing spindle coordinate")?
        .state
        .position;
    s.tip_sag = particles[5].position[2] - particles[0].position[2].min(particles[10].position[2]);
    let reference_mass = particles[0].inverse_mass.recip();
    let mut mass_kg = reference_mass * particles.len() as f64;
    let mut cm = DVec3::ZERO;
    let mut momentum = DVec3::ZERO;
    let mut state_hash = Sha256::new();
    for p in particles {
        for component in p.position.into_iter().chain(p.velocity) {
            state_hash.update(component.to_le_bytes());
        }
        let pos = DVec3::from(p.position);
        let v = DVec3::from(p.velocity);
        cm += pos;
        let m = p.inverse_mass.recip();
        mass_kg += m - reference_mass;
        momentum += v * m;
        s.min_z = s.min_z.min(pos.z - RADIUS);
        s.peak_speed = s.peak_speed.max(v.length());
        let q = bar_shape.distances(pos);
        s.clearance = s.clearance.min(bar_shape.clearance(pos));
        for collider in &other {
            s.other_gap = s.other_gap.min(collider.clearance(pos));
        }
        let shelf_delta = shelf_shape.distances(pos);
        s.shelf_margin = s
            .shelf_margin
            .min(-shelf_delta.x.max(shelf_delta.y) - RADIUS);
        s.near_bar |= q.x < RADIUS && q.y < RADIUS;
    }
    s.particle_state_sha256 = format!("{:x}", state_hash.finalize());
    s.center = (cm / particles.len() as f64).into();
    s.momentum = momentum.into();
    let sum = cable
        .last_boundary_impulses()
        .iter()
        .map(|j| DVec3::from(j.linear))
        .sum::<DVec3>();
    let weight = mass_kg * 9.81;
    s.residual =
        (momentum - prev + DT * weight * DVec3::Z + sum - DT * DVec3::from(external_n)).length();
    for pair in particles.windows(2) {
        let length = DVec3::from(pair[0].position).distance(pair[1].position.into());
        s.strain = s.strain.max((length / SPACING - 1.).abs());
    }
    for (i, j) in [left, right].into_iter().enumerate() {
        let pose = sim
            .world()
            .get::<WorldPose>(robot.pads[i].entity)
            .ok_or("missing pad pose")?;
        let outward = DQuat::from_array(pose.rotation) * DVec3::X * (if i == 0 { -1. } else { 1. });
        s.squeeze[i] = DVec3::from(j.linear).dot(outward) / DT;
        let tangential = (DVec3::from(j.linear) / DT - outward * s.squeeze[i]).length();
        s.jaw_friction_excess_n = s
            .jaw_friction_excess_n
            .max(tangential - friction * s.squeeze[i]);
        s.jaw_up[i] = -j.linear[2] / DT;
    }
    Ok(s)
}
fn run(
    sim: &mut Simulation,
    sdf: &sdformat::Sdf,
    open: bool,
    lift: f64,
    dir: Option<&Path>,
) -> Result<Vec<Sample>> {
    let control = Robot::load(sim, sdf)?;
    let mut rows = Vec::with_capacity(END);
    let mut momentum = DVec3::ZERO;
    let mut shelf_ticks = 0;
    let mut released = None;
    let mut frames = vec![];
    let mut particle_frames = vec![];
    if dir.is_some() {
        capture(sim, sdf, &mut frames, &mut particle_frames)?;
    }
    for tick in 1..=END {
        control.drive(sim, tick, open, released, lift)?;
        sim.step_exact(1)
            .map_err(|e| format!("tick {tick}: {e}, {:?}", sim.fault()))?;
        let mut s = sample(sim, &control, momentum)?;
        s.release_commanded = released.is_some();
        momentum = s.momentum.into();
        if tick > 9600 && s.shelf > 0.90 * WEIGHT {
            shelf_ticks += 1;
        } else {
            shelf_ticks = 0;
        }
        if shelf_ticks >= 30 && released.is_none() {
            released = Some(tick);
        }
        rows.push(s);
        if dir.is_some() && tick % 70 == 0 {
            capture(sim, sdf, &mut frames, &mut particle_frames)?;
        }
    }
    if let Some(d) = dir {
        std::fs::create_dir(d)?;
        gz_sim_cli::export_archive(&d.join("replay"), Default::default(), frames)?;
        std::fs::write(d.join("trace.json"), serde_json::to_vec(&rows)?)?;
        std::fs::write(
            d.join("particle-frames.json"),
            serde_json::to_vec(&particle_frames)?,
        )?;
        let captures = serde_json::json!({"kind":"authored-archive-v1","dt_ns":1,"camera":"front","particle_visuals":(0..11).map(|i|format!("default::stand::post::cable_{i}")).collect::<Vec<_>>(),"captures":{"pickup":10,"lift":75,"bar":105,"place":158,"rest":200}});
        std::fs::write(d.join("frames.json"), serde_json::to_vec(&captures)?)?;
    }
    Ok(rows)
}
fn capture(
    sim: &Simulation,
    sdf: &sdformat::Sdf,
    frames: &mut Vec<(String, String)>,
    coords: &mut Vec<serde_json::Value>,
) -> Result<()> {
    let (_, body) = deformable::bodies(sim)
        .find(|(n, _)| *n == "cable")
        .ok_or("no published cable")?;
    let positions = body
        .particles()
        .iter()
        .map(|p| p.position)
        .collect::<Vec<_>>();
    coords.push(serde_json::json!({"time_ns":sim.elapsed().as_nanos(),"positions":positions}));
    frames.push(gz_sim_cli::primitive_frame(
        sim,
        sdf,
        "default",
        frames.len() as u64 + 1,
    )?);
    Ok(())
}
fn load() -> Result<Simulation> {
    Ok(load_world_with_sdf_step(
        SDF,
        "serial-cable.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?)
}
fn main() -> Result<()> {
    let out = std::env::args_os()
        .nth(1)
        .ok_or("supply fresh output directory")?;
    if out == "--weigh" {
        let d = std::env::args_os()
            .nth(2)
            .ok_or("supply fresh weighing directory")?;
        return snag::tactile::weighing::record(Path::new(&d));
    }
    if out == "--braking" {
        let d = std::env::args_os()
            .nth(2)
            .ok_or("supply fresh braking output directory")?;
        return snag::tactile::braking::record(Path::new(&d));
    }
    if out == "--tactile" {
        let output = std::env::args_os()
            .nth(2)
            .ok_or("supply fresh tactile output directory")?;
        return snag::tactile::record(Path::new(&output));
    }
    if out == "--snag" {
        let output = std::env::args_os()
            .nth(2)
            .ok_or("supply fresh snag output directory")?;
        return snag::record(Path::new(&output));
    }
    let d = Path::new(&out);
    std::fs::create_dir(d)?;
    let sdf = sdformat::load_str(SDF, "serial-cable.sdf", &sdformat::NoResources)?;
    let mut reports = vec![];
    for (name, open, lift) in [
        ("clear", false, 2.70),
        ("open", true, 2.70),
        ("low", false, 1.90),
    ] {
        let mut sim = load()?;
        let first = run(&mut sim, &sdf, open, lift, Some(&d.join(name)))?;
        gz_sim_scene::reset_loaded_world(&mut sim, |_| ())?;
        let repeat = run(&mut sim, &sdf, open, lift, None)?;
        if first != repeat {
            return Err(format!("reset diverged for {name}").into());
        }
        let report = measure::Report::from_run(&first);
        std::fs::write(
            d.join(format!("{name}.json")),
            serde_json::to_vec_pretty(&report)?,
        )?;
        report.qualify(name)?;
        reports.push(serde_json::json!({"case":name,"reset_identical_ticks":first.len(),"measurements":report}));
    }
    std::fs::write(
        d.join("comparison.json"),
        serde_json::to_vec_pretty(&reports)?,
    )?;
    Ok(())
}

#[cfg(test)]
#[path = "serial_cable/tests.rs"]
mod tests;
