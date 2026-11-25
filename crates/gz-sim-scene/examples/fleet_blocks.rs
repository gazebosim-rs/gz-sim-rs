// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure public commit and optional readers separately in the same moving SDF world.
#[allow(dead_code)]
#[path = "support/fleet_observation.rs"]
mod observation;

use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, UpdateInfo};
use gz_sim_scene::{ImuSample, LocalPose, Name, Velocity, WorldPose};
use std::{error::Error, hint::black_box, time::Instant};

fn main() -> Result<(), Box<dyn Error>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    let [count, layout, mode, ticks] = args.as_slice() else {
        return Err(
            "usage: fleet_consumers COUNT prescribed|loaded bare|none|sparse|full TICKS".into(),
        );
    };
    let count: usize = count.parse()?;
    let ticks: usize = ticks.parse()?;
    let mixed = match layout.as_str() {
        "prescribed" => false,
        "loaded" => true,
        _ => return Err("unknown physics layout".into()),
    };
    if !matches!(mode.as_str(), "bare" | "none" | "sparse" | "full") || count == 0 || ticks == 0 {
        return Err("expected a supported consumer and positive counts".into());
    }
    let columns = count.isqrt() + usize::from(count.isqrt().pow(2) != count);
    let mut sim = observation::scene_with_sensor(count, mixed, columns, mode != "bare")?;

    sim.step_exact(20)?;
    let world = sim.world_mut();
    let mut poses = world.query::<(&EntityId, &WorldPose, &LocalPose)>();
    let mut twists = world.query::<&Velocity>();
    let witnesses: Vec<_> = [0, count / 2, count - 1]
        .map(|i| format!("default::deck_{i}::body"))
        .iter()
        .map(|name| {
            world
                .iter_entities()
                .find(|e| e.get::<Name>().is_some_and(|n| &n.0 == name))
                .expect("authored deck link")
                .id()
        })
        .collect();
    let imu = world
        .iter_entities()
        .find(|e| e.contains::<ImuSample>())
        .map(|e| e.id());
    use std::io::{BufRead,Write};
    println!("ready"); std::io::stdout().flush()?;
    for command in std::io::stdin().lock().lines() {
    let command=command?;
    let (mode, ticks)=command.split_once(" ").unwrap();let ticks=ticks.parse::<usize>()?;
    let observed=mode=="sparse" || mode=="full";
    let mut samples = Vec::new();
    for _ in 0..ticks {
        let start = Instant::now();
        sim.step_exact(1)?;
        let commit = start.elapsed();
        let world = sim.world();
        let consumer_start = Instant::now();
        if observed {
            for entity in &witnesses {
                let e = world.entity(*entity);
                black_box((
                    e.get::<EntityId>().unwrap().get(),
                    *e.get::<WorldPose>().unwrap(),
                    *e.get::<LocalPose>().unwrap(),
                    *e.get::<Velocity>().unwrap(),
                ));
            }
            let sample = world.get::<ImuSample>(imu.unwrap()).unwrap();
            assert_eq!(sample.time, sim.elapsed());
            assert_eq!(sample.epoch, world.resource::<UpdateInfo>().epoch);
            assert_eq!(
                sample.reading.as_ref().unwrap().specific_force,
                [0., 0., 9.81]
            );
        }
        if mode == "full" {
            for (id, world, local) in poses.iter(world) {
                black_box((id.get(), *world, *local));
            }
            for v in twists.iter(world) {
                black_box(*v);
            }
        }
        let readers = consumer_start.elapsed();
        let ray_start = Instant::now();
        if observed {
            let deck = world.get::<WorldPose>(witnesses[0]).unwrap().translation;
            // Near the edge of the deck, clear of its payload even in the loaded run.
            let hit = world
                .resource::<PhysicsBackend>()
                .cast_ray([deck[0] + 0.35, deck[1], 1.], [0., 0., -1.], 2., None)?
                .unwrap();
            assert!((hit - 0.9).abs() < 1e-12);
        }
        samples.push([
            commit.as_nanos(),
            readers.as_nanos(),
            ray_start.elapsed().as_nanos(),
            start.elapsed().as_nanos(),
        ]);
    }
    let world = sim.world();
    let backend = world
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    let patches = backend.rigid_contact_patches();
    assert_eq!(patches.len(), if mixed { count.div_ceil(100) } else { 0 });
    let public_poses = poses.iter(world).count();
    let public_twists = twists.iter(world).count();
    let info = world.resource::<UpdateInfo>();
    println!(
        "{}",
        serde_json::json!({"decks":count, "layout":layout, "consumer":mode,
        "public_poses":public_poses, "public_twists":public_twists, "epoch":info.epoch, "active_imus":usize::from(imu.is_some()),
        "time_ns":info.sim_time.as_nanos(), "contacts":patches.len(),
        "native_points":patches.iter().map(|p|p.points.len()).sum::<usize>(),
        "columns_ns":["commit","readers","ray","total"], "samples":samples})
    );
    std::io::stdout().flush()?;
    }
    Ok(())
}
