#[path = "support/fleet_observation.rs"]
#[allow(dead_code)]
mod observation;
use gz_sim_core::EntityId;
use gz_sim_scene::{LocalPose, Velocity, WorldPose};
use sha2::{Digest, Sha256};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let layout = std::env::args().nth(1).unwrap();
    let mixed = layout != "sparse";
    let pulse = layout == "pulse";
    let mut sim = observation::scene(100_000, mixed)?;
    let payload = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<gz_sim_scene::Name>()
                .is_some_and(|n| n.0 == "default::payload_0::body")
        })
        .and_then(|e| e.get::<EntityId>().map(|id| id.get()));
    for tick in 1..=if pulse { 220 } else { 120 } {
        if pulse && tick == 121 {
            gz_sim_scene::queue_link_world_wrench(
                &mut sim,
                payload.unwrap(),
                [510., 0., 185.],
                [0.; 3],
            )?;
        }
        sim.step_exact(1)?;
        let mut digest = Sha256::new();
        let (mut frames, mut motions) = (0, 0);
        for entity in sim.world().iter_entities() {
            let Some(world) = entity.get::<WorldPose>() else {
                continue;
            };
            let Some(id) = entity.get::<EntityId>() else {
                continue;
            };
            let local = entity
                .get::<LocalPose>()
                .expect("all authored nodes own local pose")
                .0;
            digest.update(id.get().to_le_bytes());
            let mut fields = [0u8; 112];
            for (slot, v) in fields.chunks_mut(8).zip(
                world
                    .translation
                    .into_iter()
                    .chain(world.rotation)
                    .chain(local.translation)
                    .chain(local.rotation),
            ) {
                slot.copy_from_slice(&v.to_le_bytes());
            }
            digest.update(fields);
            frames += 1;
            if let Some(v) = entity.get::<Velocity>() {
                for x in v.linear.into_iter().chain(v.angular) {
                    digest.update(x.to_le_bytes());
                }
                motions += 1;
            }
        }
        let sample = observation::sample(&sim);
        assert_eq!(sample.time, sim.elapsed());
        let reading = sample.reading.as_ref().unwrap();
        for x in reading
            .orientation
            .into_iter()
            .chain(reading.angular_velocity)
            .chain(reading.specific_force)
        {
            digest.update(x.to_le_bytes());
        }
        digest.update(sample.epoch.to_le_bytes());
        digest.update(sample.time.as_nanos().to_le_bytes());
        let backend = sim
            .world()
            .resource::<gz_physics::PhysicsBackend>()
            .downcast_ref::<gz_physics_multifidelity::MultiFidelityBackend>()
            .unwrap();
        for origin in [
            [0.25 * tick as f64 * 0.01, 0., 1.],
            [12.1, 0., 1.],
            [0., 109., 2.],
        ] {
            let hit = sim
                .world()
                .resource::<gz_physics::PhysicsBackend>()
                .cast_ray(origin, [0., 0., -1.], 3., None)?;
            digest.update([u8::from(hit.is_some())]);
            if let Some(distance) = hit {
                digest.update(distance.to_le_bytes());
            }
        }
        let mut patches = backend.rigid_contact_patches();
        patches.sort_by_key(|p| (p.a, p.b));
        let mut npoints = 0usize;
        let mut load = 0.;
        for patch in &patches {
            digest.update(patch.a.raw().to_le_bytes());
            digest.update(patch.b.raw().to_le_bytes());
            for v in patch.normal_on_a {
                digest.update(v.to_le_bytes());
            }
            digest.update((patch.points.len() as u64).to_le_bytes());
            for point in &patch.points {
                npoints += 1;
                load += point.normal_impulse;
                for v in point
                    .lever_a
                    .into_iter()
                    .chain(point.lever_b)
                    .chain([point.normal_impulse])
                {
                    digest.update(v.to_le_bytes());
                }
                digest.update([u8::from(point.tangent_impulse_on_a.is_some())]);
                for v in point.tangent_impulse_on_a.iter().flatten() {
                    digest.update(v.to_le_bytes());
                }
            }
        }
        println!(
            "{tick} {frames} {motions} {} {npoints} {load:.17e} {:x}",
            patches.len(),
            digest.finalize()
        );
    }
    Ok(())
}
