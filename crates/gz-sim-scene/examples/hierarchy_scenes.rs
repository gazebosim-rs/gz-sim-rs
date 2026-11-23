// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Time normal repository fixtures and save their public state for exact comparison.
use gz_sim_core::EntityId;
use gz_sim_scene::{
    ImuSample, LocalPose, PhysicsEngine, Velocity, WorldPose, load_world_with_sdf_step,
};
use std::{collections::BTreeMap, error::Error, fs, path::PathBuf, time::Instant};
fn main() -> Result<(), Box<dyn Error>> {
    let mut args = std::env::args_os().skip(1);
    let path = PathBuf::from(
        args.next()
            .ok_or("usage: hierarchy_scenes WORLD_SDF FINAL_STATE")?,
    );
    let output = PathBuf::from(args.next().ok_or("missing final state file")?);
    if args.next().is_some() {
        return Err("unexpected arguments".into());
    }
    let xml = fs::read_to_string(&path)?;
    let parsed = sdformat::load_str(
        &xml,
        path.to_str().ok_or("invalid path")?,
        &sdformat::NoResources,
    )?;
    let scene = &parsed.worlds.first().ok_or("missing world")?.name;
    let mut sim = load_world_with_sdf_step(
        &xml,
        path.to_str().unwrap(),
        scene,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    sim.step_exact(20)?;
    let mut samples = Vec::with_capacity(1000);
    let begin = Instant::now();
    for _ in 0..1000 {
        let t = Instant::now();
        sim.step_exact(1)?;
        samples.push(t.elapsed());
    }
    let wall = begin.elapsed().as_secs_f64();
    samples.sort_unstable();
    let mut states = BTreeMap::new();
    let mut sensor_count = 0;
    for e in sim.world().iter_entities() {
        let (Some(id), Some(world), Some(local)) = (
            e.get::<EntityId>(),
            e.get::<WorldPose>(),
            e.get::<LocalPose>(),
        ) else {
            continue;
        };
        let mut bits = Vec::new();
        for p in [world, &local.0] {
            bits.extend(p.translation.map(f64::to_bits));
            bits.extend(p.rotation.map(f64::to_bits));
        }
        if let Some(v) = e.get::<Velocity>() {
            bits.extend(v.linear.map(f64::to_bits));
            bits.extend(v.angular.map(f64::to_bits));
        }
        if let Some(s) = e.get::<ImuSample>() {
            let r = s.reading.as_ref().map_err(|_| "invalid IMU sample")?;
            bits.extend([s.epoch, s.time.as_nanos() as u64]);
            bits.extend(r.orientation.map(f64::to_bits));
            bits.extend(r.specific_force.map(f64::to_bits));
            bits.extend(r.angular_velocity.map(f64::to_bits));
            sensor_count += 1;
        }
        assert!(states.insert(id.get(), bits).is_none());
    }
    fs::write(output, format!("{states:?}\n"))?;
    println!(
        "fixture={} observed={} imus={sensor_count} warmup=20 measured=1000 wall_s={wall:.8} p50_us={:.4} p99_us={:.4}",
        path.display(),
        states.len(),
        samples[499].as_secs_f64() * 1e6,
        samples[989].as_secs_f64() * 1e6
    );
    Ok(())
}
