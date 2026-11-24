// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Record a finite rotating bar grazing both particles of an initially unloaded cable.
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    Name, PhysicsEngine, WorldPose, deformable::bodies, load_world_with_sdf_step,
    queue_link_world_velocity,
};
use serde_json::{Value, json};
use std::{collections::BTreeMap, error::Error, path::PathBuf};

#[derive(Parser)]
struct Args {
    /// New replay archive directory.
    output: PathBuf,
}
const XML: &str = include_str!("../../../examples/rotating_cuboid_cable.sdf");
fn bar(sim: &Simulation) -> bevy_ecs::world::EntityRef<'_> {
    sim.world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|name| name.0 == "default::bar::body")
        })
        .unwrap()
}
fn visual(id: u64, name: &str, p: DVec3, q: DQuat, shape: Value, color: [f64; 4]) -> Value {
    json!({"id":id,"name":name,"translation":p.to_array(),"rotation":q.to_array(),"shape":shape,"color":color})
}
fn sample(sim: &Simulation, sequence: usize) -> (String, String) {
    let pose = bar(sim).get::<WorldPose>().copied().unwrap();
    let (_, soft) = bodies(sim).next().unwrap();
    let particles = soft.particles();
    let (a, b) = (
        DVec3::from(particles[0].position),
        DVec3::from(particles[1].position),
    );
    let delta = b - a;
    let visuals = [
        visual(
            3,
            "default::bar::body::blue",
            pose.translation.into(),
            DQuat::from_array(pose.rotation),
            json!({"kind":"box","size":[2.,0.16,0.28]}),
            [0.06, 0.38, 0.58, 1.],
        ),
        visual(
            5,
            "default::cable::lower",
            a,
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.04}),
            [1., 0.28, 0.06, 1.],
        ),
        visual(
            6,
            "default::cable::upper",
            b,
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.04}),
            [1., 0.7, 0.12, 1.],
        ),
        visual(
            7,
            "default::cable::strand",
            (a + b) * 0.5,
            DQuat::from_rotation_arc(DVec3::Z, delta.normalize()),
            json!({"kind":"box","size":[0.012,0.012,delta.length()]}),
            [0.96, 0.58, 0.14, 1.],
        ),
    ];
    let t = sim.elapsed();
    let scene = json!({"version":1,"kind":"scene","world":"default","world_id":1,"epoch":0,"sequence":sequence,
        "seconds":t.as_secs(),"nanoseconds":t.subsec_nanos(),"paused":false,"visuals":visuals});
    let model = |id: u64, name: &str, vs: &[u64]| json!({"model_id":id,"name":name,"visual_ids":vs,"can_remove":false,"can_set_state":false});
    let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,
        "models":[model(2,"bar",&[3]),model(4,"cable",&[5,6,7])],"templates":[]});
    (scene.to_string(), catalog.to_string())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let mut sim = load_world_with_sdf_step(
        XML,
        "rotating_cuboid_cable.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let id = bar(&sim).get::<EntityId>().unwrap().get();
    queue_link_world_velocity(&mut sim, id, [0.; 3], [0., 0., 8.])?;
    let mut frames = vec![sample(&sim, 1)];
    let mut impacts = Vec::new();
    for tick in 1..=120 {
        sim.step_exact(1)
            .map_err(|e| format!("tick {tick}: {e}; {:?}", sim.fault()))?;
        let (_, soft) = bodies(&sim).next().unwrap();
        if soft.last_boundary_impulses()[0].linear != [0.; 3] {
            impacts.push(tick);
            println!(
                "tick {tick} receiver impulse {:?}, particle velocities {:?}",
                soft.last_boundary_impulses()[0],
                soft.particles()
                    .iter()
                    .map(|p| p.velocity)
                    .collect::<Vec<_>>()
            );
        }
        frames.push(sample(&sim, tick + 1));
    }
    if impacts.is_empty() {
        return Err("missing measured contact".into());
    }
    gz_sim_cli::export_archive(&args.output, BTreeMap::new(), frames)?;
    println!("121 measured frames over 1.20 seconds, impact ticks {impacts:?}");
    Ok(())
}
