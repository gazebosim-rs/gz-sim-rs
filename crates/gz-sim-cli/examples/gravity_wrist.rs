// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare gravity feedforward to a freely swinging spherical wrist with a welded payload.
#![allow(deprecated)]
use bevy_ecs::entity::Entity;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine, WorldPose, joint_gravity_load, load_world_with_sdf_step,
    queue_ball_joint_effort, reset_loaded_world,
};
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    path::{Path, PathBuf},
};

#[path = "gravity_wrist/inertia.rs"]
mod inertia;

const XML: &str = include_str!("../../../examples/worlds/gravity_wrist.sdf");
const TICKS: usize = 1200;
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[derive(Parser)]
struct Options {
    /// Directory for the native replay, measured trajectory and joint gravity comparison.
    output: PathBuf,
    /// Accelerate and brake the loaded wrist using its measured composite inertia.
    #[arg(long)]
    accelerate: bool,
    /// Recover from a measured disturbance using a torque-limited spherical drive.
    #[arg(long, conflicts_with = "accelerate")]
    stabilize: bool,
    /// Native steps per 1 ms actuator tick in the two moving-wrist experiments.
    #[arg(long,default_value_t=1,value_parser=clap::value_parser!(u8).range(1..=4))]
    physics_substeps: u8,
}

fn main() -> Result<()> {
    let options = Options::parse();
    if options.accelerate || options.stabilize {
        inertia::record(&options.output, options.stabilize, options.physics_substeps)
    } else if options.physics_substeps != 1 {
        Err("select --accelerate or --stabilize to vary the native step".into())
    } else {
        record(&options.output)
    }
}

fn named(sim: &Simulation, name: &str) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .expect("authored fixture entity")
        .id()
}

fn pose(sim: &Simulation, entity: Entity) -> (DVec3, DQuat) {
    let pose = sim
        .world()
        .get::<WorldPose>(entity)
        .expect("authored link pose");
    (pose.translation.into(), DQuat::from_array(pose.rotation))
}

fn rotation([roll, pitch, yaw]: [f64; 3]) -> DQuat {
    DQuat::from_rotation_z(yaw) * DQuat::from_rotation_y(pitch) * DQuat::from_rotation_x(roll)
}

struct Wrist {
    bearing: u64,
    clamp: u64,
    carrier: Entity,
    tool: Entity,
    payload: Entity,
}
impl Wrist {
    fn new(sim: &Simulation, model: &str) -> Self {
        let link = |part: &str| named(sim, &format!("default::{model}::{part}"));
        let joint = |part: &str| sim.world().get::<EntityId>(link(part)).unwrap().get();
        Self {
            bearing: joint("bearing"),
            clamp: joint("clamp"),
            carrier: link("carrier"),
            tool: link("tool"),
            payload: link("payload"),
        }
    }
    // Independent load calculation from public SDF link poses and authored masses.
    fn observe(&self, sim: &Simulation) -> Result<Observation> {
        let (origin, carrier) = pose(sim, self.carrier);
        let (tool, tool_orientation) = pose(sim, self.tool);
        let (payload, payload_orientation) = pose(sim, self.payload);
        let weight = DVec3::new(0., 0., -9.81);
        let axes = carrier * rotation([0.31, -0.42, 0.63]);
        let payload_com = payload + payload_orientation * DVec3::new(0.016, -0.012, 0.005);
        let moment = (tool - origin).cross(2. * weight) + (payload_com - origin).cross(weight);
        let load = joint_gravity_load(sim, self.bearing)?;
        let clamp = joint_gravity_load(sim, self.clamp)?;
        // The fixed joint's SDF axes differ from its solver constraint axes.
        let clamp_axes = tool_orientation * rotation([-0.2, 0.4, 0.55]);
        // The parent owns the joint origin, even during finite solver deflection.
        let clamp_arm = payload_com - (tool + tool_orientation * DVec3::new(0.245, 0.053, 0.008));
        let mut force_error = (axes * DVec3::from(load.force_n)).distance(3. * weight);
        let mut torque_error = (axes * DVec3::from(load.torque_nm)).distance(moment);
        force_error = force_error.max((clamp_axes * DVec3::from(clamp.force_n)).distance(weight));
        torque_error = torque_error
            .max((clamp_axes * DVec3::from(clamp.torque_nm)).distance(clamp_arm.cross(weight)));
        Ok(Observation {
            payload: payload.to_array(),
            orientation: payload_orientation.to_array(),
            force: load.force_n,
            torque: load.torque_nm,
            force_error,
            torque_error,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
struct Observation {
    payload: [f64; 3],
    orientation: [f64; 4],
    force: [f64; 3],
    torque: [f64; 3],
    force_error: f64,
    torque_error: f64,
}

fn episode(
    sim: &mut Simulation,
    mut capture: impl FnMut(usize, &Simulation) -> Result<()>,
) -> Result<Vec<[Observation; 2]>> {
    let wrists = ["held", "free"].map(|n| Wrist::new(sim, n));
    let mut samples = Vec::with_capacity(TICKS + 1);
    for tick in 0..=TICKS {
        let held = wrists[0].observe(sim)?;
        samples.push([held, wrists[1].observe(sim)?]);
        if tick % 10 == 0 {
            capture(tick, sim)?;
        }
        if tick < TICKS {
            queue_ball_joint_effort(sim, wrists[0].bearing, held.torque.map(|v| -v))?;
            if tick == 0 {
                sim.set_paused(true);
                sim.tick()?;
                assert_eq!(
                    joint_gravity_load(sim, wrists[0].bearing)?.torque_nm,
                    held.torque
                );
                sim.set_paused(false);
            }
            sim.step_exact(1)?;
        }
    }
    Ok(samples)
}

fn summary(samples: &[[Observation; 2]]) -> Value {
    let mut displacement = [0_f64; 2];
    let mut angle = [0_f64; 2];
    let (mut ferr, mut terr) = (0_f64, 0_f64);
    for pair in samples {
        for i in 0..2 {
            let cur = pair[i];
            displacement[i] = displacement[i]
                .max(DVec3::from(cur.payload).distance(samples[0][i].payload.into()));
            let relative = DQuat::from_array(cur.orientation)
                * DQuat::from_array(samples[0][i].orientation).inverse();
            angle[i] = angle[i].max(2. * relative.xyz().length().atan2(relative.w.abs()));
            ferr = ferr.max(cur.force_error);
            terr = terr.max(cur.torque_error);
        }
    }
    assert!(
        displacement[0] < 0.001 && displacement[1] > 0.08,
        "payload displacements {displacement:?}"
    );
    assert!(
        angle[0] < 0.003 && angle[1] > 0.25,
        "payload rotations {angle:?}"
    );
    assert!(
        ferr < 1e-10 && terr < 1e-10,
        "force error {ferr}, moment error {terr}"
    );
    json!({"ticks":TICKS,"dt_s":0.001,"compared_models":["held","free"],"max_payload_displacement_m":displacement,"max_payload_angle_rad":angle,"max_analytical_force_error_n":ferr,"max_analytical_torque_error_nm":terr,"initial_joint_force_n":samples[0][0].force,"initial_joint_torque_nm":samples[0][0].torque,"reset_sample_pairs":samples.len()})
}

fn record(out: &Path) -> Result<()> {
    let sdf = sdformat::load_str(XML, "gravity_wrist.sdf", &sdformat::NoResources)?;
    let mut sim = load_world_with_sdf_step(
        XML,
        "gravity_wrist.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut references = Vec::new();
    for model in &sdf.worlds[0].models {
        for link in &model.links {
            let entity = named(&sim, &link.scoped_name);
            for visual in &link.visuals {
                let transform = sdf.resolve_pose(
                    &visual.pose,
                    &model.scoped_name,
                    &link.scoped_name,
                    &link.scoped_name,
                )?;
                references.push((
                    format!("{}::{}", link.scoped_name, visual.name),
                    entity,
                    transform,
                ));
            }
        }
    }
    let (mut frames, mut display) = (Vec::new(), Vec::new());
    let samples = episode(&mut sim, |tick, s| {
        let mut poses = BTreeMap::new();
        for (name, entity, local) in &references {
            let (p, q) = pose(s, *entity);
            poses.insert(name,json!({"translation":(p+q*DVec3::from(local.translation)).to_array(),"rotation":(q*DQuat::from_array(local.rotation)).to_array()}));
        }
        display.push(json!({"time_ns":tick*1_000_000,"poses":poses}));
        frames.push(gz_sim_cli::primitive_frame(
            s,
            &sdf,
            "default",
            frames.len() as u64 + 1,
        )?);
        Ok(())
    })?;
    reset_loaded_world(&mut sim, |_| ())?;
    let reset = episode(&mut sim, |_, _| Ok(()))?;
    let mut max_reset_position = 0_f64;
    let mut max_reset_angle = 0_f64;
    let mut max_reset_torque = 0_f64;
    let mut max_reset_force = 0_f64;
    for (a, b) in samples.iter().flatten().zip(reset.iter().flatten()) {
        max_reset_position =
            max_reset_position.max(DVec3::from(a.payload).distance(b.payload.into()));
        let dq = DQuat::from_array(a.orientation).inverse() * DQuat::from_array(b.orientation);
        max_reset_angle = max_reset_angle.max(2. * dq.xyz().length().atan2(dq.w.abs()));
        max_reset_torque = max_reset_torque.max(DVec3::from(a.torque).distance(b.torque.into()));
        max_reset_force = max_reset_force.max(DVec3::from(a.force).distance(b.force.into()));
    }
    let mut result = summary(&samples);
    result["reset"] = json!({"max_position_m":max_reset_position,"max_angle_rad":max_reset_angle,"max_torque_nm":max_reset_torque,"max_force_n":max_reset_force});
    assert!(
        max_reset_position < 1e-9
            && max_reset_angle < 1e-8
            && max_reset_torque < 1e-7
            && max_reset_force < 1e-7,
        "{result}"
    );
    fs::create_dir_all(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
    for (name, value) in [
        ("comparison.json", result.clone()),
        ("trajectory.json", serde_json::to_value(samples)?),
        ("display.json", json!(display)),
    ] {
        fs::write(out.join(name), serde_json::to_vec(&value)?)?;
    }
    println!("{}", serde_json::to_string_pretty(&result)?);
    Ok(())
}

#[cfg(test)]
#[path = "gravity_wrist/tests.rs"]
mod tests;
