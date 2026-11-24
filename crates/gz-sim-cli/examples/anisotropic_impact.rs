// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run the SDF cable/rigid-sphere glancing impact and export its measured gzweb poses.
//! cargo run --release -p gz-sim-cli --example anisotropic_impact -- OUTPUT_DIRECTORY
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_sim_core::Simulation;
use gz_sim_scene::{Name, PhysicsEngine, WorldPose, deformable::bodies, load_world_with_sdf_step};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{error::Error, path::PathBuf};

#[path = "support/garment_geometry.rs"]
mod garment_geometry;
use bytes::Bytes;
use std::collections::BTreeMap;

const PAINT: [[f64; 3]; 3] = [[0.94, 0.24, 0.09], [0.96, 0.72, 0.12], [0.22, 0.83, 0.84]];
const WORLD: &str = include_str!("../../../examples/anisotropic_sphere_impact.sdf");

#[derive(Parser)]
struct Args {
    /// Use the oriented cuboid fixture and its measured box pose.
    #[arg(long)]
    cuboid: bool,
    /// New destination directory for the gzweb replay archive.
    output: PathBuf,
}

fn visual(
    id: u64,
    name: &str,
    position: DVec3,
    rotation: DQuat,
    geometry: Value,
    color: [f64; 3],
) -> Value {
    let translation = position + DVec3::Z * 0.62;
    json!({"id":id,"name":format!("default::{name}"),"translation":translation.to_array(),"rotation":rotation.to_array(),
        "shape":geometry,"color":[color[0],color[1],color[2],1.]})
}
fn model(id: u64, name: &str, visuals: &[u64]) -> Value {
    json!({"model_id":id,"name":name,"visual_ids":visuals,"can_remove":false,"can_set_state":false})
}

fn frame(sim: &Simulation, sequence: u64, cuboid: bool) -> (String, String) {
    let pose = sim
        .world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<Name>()?.0 == "default::receiver::shell")
                .then(|| *e.get::<WorldPose>().expect("loaded rigid link"))
        })
        .expect("loaded receiver");
    let center = DVec3::from(pose.translation);
    let rotation = DQuat::from_array(pose.rotation);
    let principal = rotation
        * DQuat::from_rotation_z(0.5)
        * DQuat::from_rotation_y(0.15)
        * DQuat::from_rotation_x(-0.3);
    let mut visuals = vec![visual(
        3,
        "receiver::shell",
        center,
        rotation,
        if cuboid {
            json!({"kind":"box","size":[0.85,0.54,0.4]})
        } else {
            json!({"kind":"sphere","radius":0.5})
        },
        [0.08, 0.29, 0.41],
    )];
    let mut shell_ids = vec![3];
    // Color bands follow the actual three principal planes, making tumble
    // visible on spherical geometry. These are paint marks, not colliders.
    for (i, axis) in [DVec3::X, DVec3::Y, DVec3::Z].into_iter().enumerate() {
        let id = 4 + i as u64;
        if cuboid {
            let (offset, size) = [
                ([0., 0.13, 0.2], [0.85, 0.038, 0.006]),
                ([0.28, 0., 0.2], [0.038, 0.54, 0.009]),
                ([0.25, -0.27, 0.], [0.038, 0.006, 0.4]),
            ][i];
            visuals.push(visual(
                id,
                &format!("receiver::orientation_mark_{i}"),
                center + rotation * DVec3::from(offset),
                rotation,
                json!({"kind":"box","size":size}),
                PAINT[i],
            ));
        } else {
            let q = principal * DQuat::from_rotation_arc(DVec3::Z, axis);
            let uri = format!("{}/assets/{i}/band.glb", gz_sim_cli::EXPORT_ASSET_PREFIX);
            visuals.push(visual(
                id,
                &format!("receiver::principal_band_{i}"),
                center,
                q,
                json!({"kind":"mesh","uri":uri,"scale":[1,1,1]}),
                PAINT[i],
            ));
        }
        shell_ids.push(id);
    }
    let (_, body) = bodies(sim).next().expect("measured soft state");
    let a = DVec3::from(body.particles()[0].position);
    let b = DVec3::from(body.particles()[1].position);
    for (id, p) in [(21, a), (22, b)] {
        visuals.push(visual(
            id,
            &format!("impactor::particle_{id}"),
            p,
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.04}),
            [1., 0.48, 0.09],
        ));
    }
    // Interpolate the observed strand endpoints for the connector. Each bead
    // has an immutable visual definition throughout the gzweb archive.
    for i in 1..=5 {
        visuals.push(visual(
            22 + i,
            &format!("impactor::strand_{i}"),
            a.lerp(b, i as f64 / 6.),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.016}),
            [0.91, 0.72, 0.35],
        ));
    }
    let ns = sim.elapsed().as_nanos() as u64;
    let scene = json!({"version":if cuboid{1}else{2},"kind":"scene","world":"default","world_id":1,"epoch":0,
        "sequence":sequence,"seconds":ns/1_000_000_000,"nanoseconds":ns%1_000_000_000,
        "paused":false,"visuals":visuals});
    let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,
        "scene_sequence":sequence,"models":[model(2,"receiver",&shell_ids),model(20,"impactor",&[21,22,23,24,25,26,27])],"templates":[]});
    (scene.to_string(), catalog.to_string())
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let mut sim = load_world_with_sdf_step(
        if args.cuboid {
            include_str!("../../../examples/cuboid_soft_impact.sdf")
        } else {
            WORLD
        },
        "soft_impact.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut frames = vec![frame(&sim, 1, args.cuboid)];
    let mut impacts = Vec::new();
    for tick in 1..=1200 {
        sim.step_exact(1)?;
        let reaction = bodies(&sim)
            .next()
            .expect("loaded impactor")
            .1
            .last_boundary_impulses()[0];
        if reaction.linear != [0.; 3] {
            impacts.push(tick);
        }
        if tick % 10 == 0 {
            frames.push(frame(&sim, frames.len() as u64 + 1, args.cuboid));
        }
    }
    if impacts.is_empty() {
        return Err("fixture produced no contact".into());
    }
    let mut assets = BTreeMap::new();
    for (i, color) in PAINT.into_iter().enumerate().filter(|_| !args.cuboid) {
        let (mut vertices, mut faces) = (Vec::new(), Vec::new());
        for j in 0..96 {
            let a = std::f64::consts::TAU * j as f64 / 96.;
            for z in [-0.017, 0.017] {
                vertices.push([0.501 * a.cos(), 0.501 * a.sin(), z]);
            }
            let (start, next) = (2 * j, 2 * ((j + 1) % 96));
            faces.extend([[start, next, next + 1], [start, next + 1, start + 1]]);
        }
        let bytes = garment_geometry::surface_glb(
            "gz-sim-rs",
            "Principal inertia plane",
            &vertices,
            &faces,
            color.map(|v| v as f32),
        )?;
        assets.insert(format!("/assets/{i}/band.glb"), Bytes::from(bytes));
    }
    gz_sim_cli::export_archive(&args.output, assets, frames)?;
    println!(
        "Wrote 121 measured frames over 1.2 s to {}. Contact ticks (1 ms): {impacts:?}",
        args.output.display()
    );
    Ok(())
}
