// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run a normal SDF through worker deaths, a queued impulse and native torque.
#[path = "support/replay.rs"]
mod replay;
use clap::Parser;
use gz_physics::{DynamicBackend, PhysicsBackend};
use gz_physics_distributed::{SceneBackend, coordinator::Frame};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, WorldLoadOptions, load_world_configured,
    queue_link_world_wrench,
};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{
    error::Error,
    path::{Path, PathBuf},
    process::Command,
    time::Instant,
};

#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: Option<PathBuf>,
    #[arg(long)]
    capture: Option<PathBuf>,
}
fn load(worker: &Path) -> Result<Simulation, Box<dyn Error>> {
    Ok(load_world_configured(
        include_str!("../../../examples/distributed_boxes.sdf"),
        "distributed_boxes.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
    )?)
}
fn backend(sim: &Simulation) -> &SceneBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref()
        .unwrap()
}
struct Sample {
    frame: Frame,
    replacements: [u8; 2],
}
impl replay::Sample for Sample {
    fn committed(&self) -> &Frame {
        &self.frame
    }
    fn guides(&self, visuals: &mut Vec<Value>, models: &mut Vec<Value>) {
        for (worker, count) in self.replacements.into_iter().enumerate() {
            let model = 1000 + 5 * worker;
            let name = format!("default::worker_{worker}_replacements");
            let z = -0.9 - 0.4 * worker as f64;
            let color = if worker == 0 {
                [0.04, 0.80, 0.91, 1.]
            } else {
                [1., 0.34, 0.04, 1.]
            };
            for (offset, part, y, size, rgb) in [
                (
                    1,
                    "zero_to_two",
                    0.,
                    [0.04, 2., 0.05],
                    [0.40, 0.43, 0.47, 1.],
                ),
                (
                    2,
                    "completed",
                    1. - f64::from(count),
                    [0.16, 0.14, 0.18],
                    color,
                ),
            ] {
                visuals.push(json!({"id":model+offset,"name":format!("{name}::{part}"),"translation":[0.,y,z],"rotation":[0.,0.,0.,1.],"shape":{"kind":"box","size":size},"color":rgb}));
            }
            models.push(json!({"model_id":model,"name":name,"visual_ids":[model+1,model+2],"can_remove":false,"can_set_state":false}));
        }
    }
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let worker = match args.worker {
        Some(p) => p,
        None => std::env::current_exe()?
            .ancestors()
            .nth(2)
            .ok_or("pass --worker")?
            .join(format!("physics_worker{}", std::env::consts::EXE_SUFFIX)),
    };
    let mut scene = load(&worker)?;
    let mut uninterrupted = load(&worker)?;
    let cargo = scene
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::cargo::body")
        })
        .ok_or("missing cargo link")?;
    let entity = cargo.get::<EntityId>().unwrap().get();
    let body = cargo.get::<BodyHandle>().unwrap().id();
    let mut counts = [0u8; 2];
    let mut samples = vec![Sample {
        frame: backend(&scene).frame().unwrap().clone(),
        replacements: counts,
    }];
    for tick in 1..=195 {
        let frame = backend(&scene).frame().unwrap();
        let owner = usize::from(frame.body(body.raw()).unwrap().0);
        let replace = match tick {
            55 => Some(1 - owner),
            107 | 150 => Some(owner),
            _ => None,
        };
        if tick == 107 {
            assert!(frame.workers[owner].snapshot.contacts > 0);
            for sim in [&mut scene, &mut uninterrupted] {
                let mut physics = sim.world_mut().resource_mut::<PhysicsBackend>();
                let backend = physics.downcast_mut::<SceneBackend>().unwrap();
                backend.queue_linear_impulse(body, [0.1, 0., 0.])?;
                backend.apply_impulse(body, [0., 0., 0.03], [0.04, 0.02, 0.0])?;
                queue_link_world_wrench(sim, entity, [0., 2., 0.], [0., 0., 0.09])?;
            }
        }
        let pids = backend(&scene).worker_pids().unwrap();
        if let Some(i) = replace
            && !Command::new("kill")
                .args(["-KILL", &pids[i].to_string()])
                .status()?
                .success()
        {
            return Err("worker kill failed".into());
        }
        let start = Instant::now();
        scene.step_exact(1)?;
        let latency = start.elapsed();
        uninterrupted.step_exact(1)?;
        let actual = backend(&scene);
        let frame = actual.frame().unwrap();
        assert_eq!(Some(frame), backend(&uninterrupted).frame());
        if let Some(i) = replace {
            let next = actual.worker_pids().unwrap();
            assert_ne!(pids[i], next[i]);
            assert_eq!(pids[1 - i], next[1 - i]);
            counts[i] += 1;
            println!(
                "tick {tick}: worker {i} PID {} -> {}, recovered step {:.3} ms",
                pids[i],
                next[i],
                latency.as_secs_f64() * 1e3
            );
        }
        if tick % 3 == 0 || replace.is_some() {
            samples.push(Sample {
                frame: frame.clone(),
                replacements: counts,
            });
        }
    }
    if let Some(path) = args.capture {
        replay::write(&path, &samples)?;
    }
    println!(
        "195 exact SDF barriers with three supervised worker replacements. Cargo spin {:?} rad/s.",
        backend(&scene).state(body).unwrap().angular_velocity
    );
    Ok(())
}
