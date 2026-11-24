// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Opt-in full-fleet qualification through the normal world loader and reset schedule.
#[allow(dead_code)]
#[path = "../examples/support/fleet_observation.rs"]
mod observation;

use gz_physics::PhysicsBackend;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{LocalPose, Velocity, WorldPose, reset_loaded_world};
use sha2::{Digest, Sha256};
use std::{error::Error, time::Instant};

fn digest(sim: &Simulation, epoch: u64) -> Result<String, Box<dyn Error>> {
    let world = sim.world();
    let mut rows: Vec<_> = world
        .iter_entities()
        .filter_map(|e| {
            Some((
                e.get::<EntityId>()?.get(),
                e.get::<WorldPose>()?,
                e.get::<LocalPose>()?,
                e.get::<Velocity>(),
            ))
        })
        .collect();
    rows.sort_unstable_by_key(|r| r.0);
    let mut hash = Sha256::new();
    let mut put = |v: f64| hash.update(v.to_le_bytes());
    let sample = observation::sample(sim);
    assert_eq!(sample.epoch, epoch);
    assert_eq!(sample.time, sim.elapsed());
    let reading = sample.reading.as_ref().unwrap();
    for v in reading
        .orientation
        .into_iter()
        .chain(reading.angular_velocity)
        .chain(reading.specific_force)
    {
        put(v);
    }
    let backend = world.resource::<PhysicsBackend>();
    for origin in [[0., 0., 1.], [100., 0., 1.], [0., 4., 1.]] {
        put(backend
            .cast_ray(origin, [0., 0., -1.], 2., None)?
            .unwrap_or(f64::INFINITY));
    }
    for (id, world, local, velocity) in rows {
        hash.update(id.to_le_bytes());
        for v in world
            .translation
            .into_iter()
            .chain(world.rotation)
            .chain(local.0.translation)
            .chain(local.0.rotation)
        {
            hash.update(v.to_le_bytes());
        }
        if let Some(v) = velocity {
            for x in v.linear.into_iter().chain(v.angular) {
                hash.update(x.to_le_bytes());
            }
        }
    }
    let mut patches = backend
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap()
        .rigid_contact_patches();
    patches.sort_by_key(|p| (p.a, p.b));
    for p in patches {
        hash.update(p.a.raw().to_le_bytes());
        hash.update(p.b.raw().to_le_bytes());
        hash.update((p.points.len() as u64).to_le_bytes());
        for x in p.normal_on_a {
            hash.update(x.to_le_bytes());
        }
        for point in p.points {
            for x in point
                .lever_a
                .into_iter()
                .chain(point.lever_b)
                .chain([point.normal_impulse])
                .chain(point.tangent_impulse_on_a.into_iter().flatten())
            {
                hash.update(x.to_le_bytes());
            }
        }
    }
    Ok(format!("{:x}", hash.finalize()))
}

#[test]
#[ignore = "full 100,000-deck memory and cold/reset timing campaign"]
fn cold_and_reset_publications_match_at_the_first_two_solves() -> Result<(), Box<dyn Error>> {
    let mixed = std::env::var("GZ_FLEET_RESET_LAYOUT").as_deref() != Ok("prescribed");
    let start = Instant::now();
    let mut sim = observation::scene(100_000, mixed)?;
    let load_s = start.elapsed().as_secs_f64();
    let inspect = |sim: &mut Simulation, epoch| -> Result<_, Box<dyn Error>> {
        let mut times = Vec::new();
        let mut hashes = Vec::new();
        for _ in 0..2 {
            let t = Instant::now();
            sim.step_exact(1)?;
            times.push(t.elapsed().as_secs_f64() * 1000.);
            hashes.push(digest(sim, epoch)?);
        }
        Ok((times, hashes))
    };
    let (first, reference) = inspect(&mut sim, 0)?;
    sim.step_exact(18)?;
    let t = Instant::now();
    reset_loaded_world(&mut sim, |_| ())?;
    let reset_ms = t.elapsed().as_secs_f64() * 1000.;
    let (restored, repeated) = inspect(&mut sim, 1)?;
    assert_eq!(repeated, reference);
    println!(
        "{}",
        serde_json::json!({"loaded":mixed,"load_s":load_s,"first_ms":first,
        "reset_ms":reset_ms,"restored_ms":restored,"tick_sha256":reference})
    );
    Ok(())
}
