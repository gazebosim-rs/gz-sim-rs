// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Sixty-four independent vertical lanes with alternating prescribed and free capsules.
use clap::Parser;
use gz_physics::{BodyId, BodySpec, DynamicBackend, PhysicsBackend, RotationSpec};
use gz_physics_distributed::{
    SceneBackend,
    physics::{Primitive, Tier},
};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::{PhysicsEngine, WorldLoadOptions, load_world_configured, reset_loaded_world};
use sdformat::NoResources;
use std::{
    error::Error,
    fmt::Write,
    fs,
    path::{Path, PathBuf},
    time::Duration,
};
#[derive(Parser)]
struct Args {
    #[arg(long)]
    worker: PathBuf,
    /// Write the ordinary SDF and its local display mesh into an empty or new directory.
    #[arg(long)]
    write_scene: Option<PathBuf>,
    #[arg(long, default_value_t = 500)]
    ticks: usize,
}
fn sdf() -> String {
    let mut s = String::from(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -0.35</gravity><physics name="lanes" type="ignored"><max_step_size>0.005</max_step_size><real_time_factor>0.5</real_time_factor></physics>"#,
    );
    for y in 0..8 {
        for x in 0..8 {
            let plugin = if (x + y) % 2 == 0 {
                r#"<plugin filename="gz-sim-prescribed-motion-system" name="gz::sim::systems::PrescribedMotion"><linear_velocity>0 0 0.45</linear_velocity><angular_velocity>0 0 0.4</angular_velocity></plugin>"#
            } else {
                ""
            };
            let label = y * 8 + x;
            let (x, y) = ((x as f64 - 3.5) * 0.82, (y as f64 - 3.5) * 0.82);
            write!(s,r#"<model name="lane_{label:02}"><pose>{x} {y} 1.8 0 0 0</pose><link name="roller"><inertial><mass>1</mass><inertia><ixx>0.16684615384615384</ixx><iyy>0.16684615384615384</iyy><izz>0.042230769230769225</izz></inertia></inertial><collision name="shell"><geometry><capsule><radius>0.3</radius><length>0.9</length></capsule></geometry></collision><visual name="paint"><geometry><mesh><uri>roller-capsule.glb</uri></mesh></geometry></visual></link>{plugin}</model>"#).unwrap();
        }
    }
    s.push_str("</world></sdf>");
    s
}
fn qualify(xml: &str, worker: &Path, ticks: usize) -> Result<(), Box<dyn Error>> {
    let dt = Duration::from_millis(5);
    let mut sim = load_world_configured(
        xml,
        "capsule-lanes.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::Distributed,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(worker),
        },
    )?;
    let current = |s: &gz_sim_core::Simulation| {
        s.world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .unwrap()
            .frame()
            .unwrap()
            .clone()
    };
    let initial = current(&sim);
    let mut control = MultiFidelityBackend::new([0., 0., -0.35])?;
    control.set_partition_step(dt)?;
    for raw in 0..64 {
        let b = initial.body(raw).unwrap().1;
        assert_eq!(b.primitive, Primitive::Capsule);
        assert_eq!(
            control
                .spawn_partition_primitive(
                    BodySpec {
                        shape: b.shape()?,
                        position: b.position,
                        velocity: b.linear,
                        mass: Some(b.mass)
                    },
                    RotationSpec {
                        orientation: b.orientation,
                        angular_velocity: b.angular
                    },
                    b.tier == Tier::Prescribed,
                    b.friction
                )?
                .raw(),
            raw
        );
    }
    let mut displacement = [0f64; 2];
    for tick in 1..=ticks {
        sim.step_exact(1)?;
        control.step(dt)?;
        let f = current(&sim);
        assert_eq!(
            f.workers.iter().map(|w| w.components.len()).sum::<usize>(),
            64
        );
        assert!(
            f.workers
                .iter()
                .all(|w| w.snapshot.impulses.is_empty() && w.snapshot.supports.is_empty())
        );
        for raw in 0..64 {
            let (owner, b) = f.body(raw).unwrap();
            assert_eq!(owner, initial.body(raw).unwrap().0);
            let c = control.state(BodyId::from_raw(raw)).unwrap();
            assert_eq!(
                (b.position, b.linear, b.orientation, b.angular),
                (c.position, c.velocity, c.orientation, c.angular_velocity),
                "tick {tick}, body {raw}"
            );
            displacement[usize::from(b.tier == Tier::Prescribed)] =
                b.position[2] - initial.body(raw).unwrap().1.position[2];
        }
    }
    let elapsed = ticks as f64 * dt.as_secs_f64();
    let physical = [-0.175 * elapsed * elapsed, 0.45 * elapsed];
    println!(
        "64 independent capsule lanes, 32 per worker, {} exact native body-frames over {elapsed:.3} s, no predicted joins or contacts; free/prescribed vertical travel {:.9}/{:.9} m, analytical {:.9}/{:.9} m",
        ticks * 64,
        displacement[0],
        displacement[1],
        physical[0],
        physical[1]
    );
    assert!((displacement[0] - physical[0]).abs() < 0.35 * dt.as_secs_f64() * elapsed + 1e-12);
    assert!((displacement[1] - physical[1]).abs() < 1e-10);
    reset_loaded_world(&mut sim, |_| ())?;
    assert_eq!(initial, current(&sim));
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    let xml = sdf();
    if let Some(dir) = args.write_scene {
        fs::create_dir_all(&dir)?;
        use std::io::Write;
        fs::File::create_new(dir.join("roller-capsule.glb"))?.write_all(include_bytes!(
            "../../../examples/assets/multifidelity/roller-capsule.glb"
        ))?;
        fs::File::create_new(dir.join("capsule-lanes.sdf"))?.write_all(xml.as_bytes())?;
    }
    qualify(&xml, &args.worker, args.ticks)
}
