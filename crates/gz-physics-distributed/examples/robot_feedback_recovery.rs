// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record independent 1 ms and worker-local 16 ms commits plus a passive human-sized comparison.
#[allow(dead_code)]
#[path = "../../gz-sim-cli/examples/soft_replay/mod.rs"]
mod soft_replay;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_physics::{PhysicsBackend, posture::PostureMode};
use gz_physics_distributed::grounded_robot::{
    GroundedRobotPool, RobotPostureInterval, RobotPostureProgram,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, entity_by_id};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, distributed_hinge::distributed_grounded_robot_spec,
    joint_posture::JointPostures, load_world_with_sdf_step,
};
use serde_json::json;
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    error::Error,
    fs,
    io::{BufWriter, Write},
    path::PathBuf,
    process::Command,
    time::Duration,
};
#[derive(Parser)]
struct Args {
    output: PathBuf,
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    sessions: PathBuf,
}
const SDF: &str = include_str!("../../../examples/humanoid_stand.sdf");
fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    fs::create_dir(&a.sessions)?;
    let spec = distributed_grounded_robot_spec(SDF, "humanoid.sdf", "default")?;
    let mut sim = load_world_with_sdf_step(
        SDF,
        "humanoid.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let lookup = |name: &str| {
        let e = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap();
        (
            e.get::<EntityId>().unwrap().get(),
            e.get::<BodyHandle>().unwrap().id(),
        )
    };
    let (chest_public, chest) = lookup("default::humanoid::chest");
    let root = lookup("default::humanoid::pelvis").1;
    let floor = lookup("default::floor::ground").1;
    let names = spec
        .links
        .iter()
        .map(|l| {
            (
                l.id,
                sim.world()
                    .get::<Name>(entity_by_id(sim.world(), l.id).unwrap())
                    .unwrap()
                    .0
                    .split("::")
                    .last()
                    .unwrap()
                    .to_owned(),
            )
        })
        .collect::<BTreeMap<_, _>>();
    let ctl = sim
        .world()
        .resource::<JointPostures>()
        .get(spec.model_id)
        .unwrap();
    let law = ctl.law();
    let active = ctl.targets();
    let mut passive = active.clone();
    passive.mode = PostureMode::Passive;
    let labels = ["single", "batch16", "passive"];
    let colors = [
        [0.20, 0.65, 0.91, 1.],
        [0.15, 0.86, 0.60, 1.],
        [0.91, 0.58, 0.17, 1.],
    ];
    let dirs = labels.map(|name| a.sessions.join(name));
    let mut pools = dirs
        .iter()
        .map(|d| GroundedRobotPool::create(&a.worker, d, 3, spec.clone()).map(Some))
        .collect::<Result<Vec<_>, _>>()?;
    let mut frames = Vec::new();
    let mut rows = BufWriter::new(fs::File::create_new(a.sessions.join("states.jsonl"))?);
    let mut deaths = 0;
    let mut transfers = 0;
    let mut reopens = 0;
    for end in (0..=1920u64).step_by(16) {
        if end != 0 {
            let start = end - 16;
            if [640, 1040, 1552].contains(&start) {
                for (i, slot) in pools.iter_mut().enumerate() {
                    let p = slot.as_mut().unwrap();
                    p.transfer((p.owner() + 1) % p.width())?;
                    transfers += 1;
                    let pid = p.worker_pid(p.owner()).unwrap();
                    assert!(
                        Command::new("kill")
                            .args(["-KILL", &pid.to_string()])
                            .status()?
                            .success()
                    );
                    deaths += 1;
                    if start != 1040 {
                        slot.take();
                        *slot = Some(GroundedRobotPool::resume(&a.worker, &dirs[i])?);
                        reopens += 1;
                    }
                }
            }
            let mut on = Vec::new();
            let mut off = Vec::new();
            for t in start..end {
                let forces = if (1000..1080).contains(&t) {
                    let f = [140., 0., 0.];
                    sim.world_mut()
                        .resource_mut::<PhysicsBackend>()
                        .add_world_wrench(chest, f, [0.; 3])?;
                    vec![(chest_public, f)]
                } else {
                    vec![]
                };
                sim.step_exact(1)?;
                let ctl = sim
                    .world()
                    .resource::<JointPostures>()
                    .get(spec.model_id)
                    .unwrap();
                let efforts = ctl
                    .axes()
                    .iter()
                    .zip(ctl.last_commands())
                    .map(|(j, c)| (j.entity_id, c.effort_nm))
                    .collect::<Vec<_>>();
                pools[0]
                    .as_mut()
                    .unwrap()
                    .advance_with_inputs_at(0, t, &efforts, &forces)?;
                on.push(RobotPostureInterval {
                    targets: active.clone(),
                    forces: forces.clone(),
                });
                off.push(RobotPostureInterval {
                    targets: passive.clone(),
                    forces,
                });
            }
            for (i, intervals) in [(1, on), (2, off)] {
                pools[i].as_mut().unwrap().advance_posture_batch_at(
                    0,
                    start,
                    &RobotPostureProgram {
                        law: law.clone(),
                        intervals,
                    },
                )?;
            }
            let image = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap()
                .checkpoint_grounded_robot_tree(root, floor)?;
            assert!(pools[0].as_ref().unwrap().native_image() == Some(&image));
            assert!(pools[1].as_ref().unwrap().native_image() == Some(&image));
        }
        let mut frame = Frame::default();
        let mut row = json!({"tick":end,"push_N":if (1000..1080).contains(&end){140.}else{0.}});
        let rot = DQuat::from_rotation_z(-0.8);
        for (i, s) in pools.iter().enumerate() {
            let p = s.as_ref().unwrap();
            let shift = DVec3::Y * (1. - i as f64) * 1.30;
            frame.add(
                &format!("{}_floor", labels[i]),
                shift + DVec3::new(0., 0., -0.026),
                DQuat::IDENTITY,
                json!({"kind":"box","size":[1.3,1.21,0.05]}),
                [0.25, 0.27, 0.31, 1.],
            );
            for (b, state) in spec.links.iter().zip(&p.observation().links) {
                frame.add(
                    &format!("{}_{}", labels[i], names[&b.id]),
                    rot * DVec3::from(state.position) + shift,
                    rot * DQuat::from_array(state.orientation),
                    json!({"kind":"box","size":b.size}),
                    colors[i],
                );
            }
            row[labels[i]] = json!({"owner":p.owner(),"generation":p.generation(),"links":p.observation().links,"patches":p.ground_patches(),"feedback_intervals":p.last_posture_batch().map_or(0,|r|r.receipt.intervals.len())});
        }
        serde_json::to_writer(&mut rows, &row)?;
        writeln!(&mut rows)?;
        frames.push(frame.finish(Duration::from_millis(end), frames.len() + 1));
    }
    rows.flush()?;
    gz_sim_cli::export_archive(&a.output, BTreeMap::new(), frames)?;
    println!(
        "121 frames of 96 actual robot/floor visual poses; 1920 ordinary and native single intervals, 120 exact 16-interval managed endpoint cuts; real worker deaths={deaths} loaded transfers={transfers} reopens={reopens}."
    );
    Ok(())
}
