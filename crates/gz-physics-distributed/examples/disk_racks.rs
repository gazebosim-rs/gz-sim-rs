// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two tilted stacks of densely spaced finite disks. No physical constraints connect them.
use clap::Parser;
use gz_physics::{BodyId, BodySpec, DynamicBackend, PhysicsBackend, RotationSpec};
use gz_physics_distributed::{SceneBackend, physics::Tier};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured, reset_loaded_world};
use sdformat::NoResources;
use std::{
    error::Error,
    fmt::Write as _,
    fs::File,
    io::Write as _,
    path::{Path, PathBuf},
    time::Duration,
};
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    #[arg(long)]
    write_scene: Option<PathBuf>,
}
const TILT: f64 = 0.56;
fn world() -> String {
    let mut s = String::from(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -0.35</gravity><physics name="racks" type="ignored"><max_step_size>0.005</max_step_size><real_time_factor>0.35</real_time_factor></physics>"#,
    );
    for lane in 0..2 {
        for i in 0..40 {
            let (sn, cs) = TILT.sin_cos();
            let along = (i as f64 - 19.5) * 0.15;
            let x = along * sn;
            let z = 2.7 + along * cs;
            let y = if lane == 0 { -1.15 } else { 1.15 };
            let color = if lane == 0 {
                "0.93 0.38 0.10"
            } else {
                "0.13 0.55 0.89"
            };
            let plugin = if lane == 0 {
                r#"<plugin filename="gz-sim-prescribed-motion-system" name="gz::sim::systems::PrescribedMotion"><linear_velocity>0 0 0.38</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>"#
            } else {
                ""
            };
            let geom = "<geometry><cylinder><radius>0.52</radius><length>0.055</length></cylinder></geometry>";
            write!(s,r#"<model name="rack_{lane}_disk_{i:02}"><pose>{x} {y} {z} 0 {TILT} 0</pose><link name="wafer"><inertial><mass>1</mass><inertia><ixx>{it}</ixx><iyy>{it}</iyy><izz>{iz}</izz></inertia></inertial><collision name="disk">{geom}</collision><visual name="paint">{geom}<material><diffuse>{color} 1</diffuse></material></visual></link>{plugin}</model>"#,it=(3.*0.52_f64.powi(2)+0.055_f64.powi(2))/12.,iz=0.52_f64.powi(2)/2.).unwrap();
        }
    }
    s.push_str("</world></sdf>");
    s
}
fn run(path: &Path, sdf: &str) -> Result<(), Box<dyn Error>> {
    let dt = Duration::from_millis(5);
    let gravity = [0., 0., -0.35];
    let mut sim = load_world_configured(
        sdf,
        "disks.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::DistributedPool(4),
            distributed_worker: Some(path),
            noise_seed: [0; 32],
            step: None,
        },
    )?;
    let current = |s: &gz_sim_core::Simulation| {
        s.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .unwrap()
            .pool_frame()
            .unwrap()
            .clone()
    };
    let seed = current(&sim);
    let mut native = MultiFidelityBackend::new(gravity)?;
    native.set_partition_step(dt)?;
    for id in 0..80 {
        let b = seed.body(id).unwrap().1;
        assert_eq!(
            native
                .spawn_partition_primitive(
                    BodySpec {
                        shape: b.shape()?,
                        mass: Some(b.mass),
                        position: b.position,
                        velocity: b.linear
                    },
                    RotationSpec {
                        orientation: b.orientation,
                        angular_velocity: b.angular
                    },
                    b.tier == Tier::Prescribed,
                    b.friction
                )?
                .raw(),
            id
        );
    }
    for tick in 1..=400 {
        sim.step_exact(1)?;
        native.step(dt)?;
        let f = current(&sim);
        for w in &f.workers {
            assert_eq!(w.components.len(), 20);
            assert!(w.snapshot.impulses.is_empty());
        }
        for id in 0..80 {
            let (owner, b) = f.body(id).unwrap();
            let control = native.state(BodyId::from_raw(id)).unwrap();
            assert_eq!(owner, seed.body(id).unwrap().0);
            assert_eq!(
                (b.position, b.orientation, b.linear, b.angular),
                (
                    control.position,
                    control.orientation,
                    control.velocity,
                    control.angular_velocity
                ),
                "{tick}: {id}"
            );
        }
    }
    let last = current(&sim);
    let displacement =
        |id| last.body(id).unwrap().1.position[2] - seed.body(id).unwrap().1.position[2];
    assert!((displacement(0) - 0.76).abs() < 1e-12 && (displacement(40) + 0.70).abs() < 0.001);
    println!(
        "80 finite disks in two tilted racks, four workers, 32000 exact native body-frames at 5 ms, all 80 owners retained; vertical motion +{:.9} / {:.9} m vs analytical +0.76 / -0.70 m",
        displacement(0),
        displacement(40)
    );
    reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(current(&sim), seed);
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let sdf = world();
    if let Some(path) = args.write_scene {
        File::create_new(path)?.write_all(sdf.as_bytes())?;
    }
    run(&args.worker, &sdf)
}
