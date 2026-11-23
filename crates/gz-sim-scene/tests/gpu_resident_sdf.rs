// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "gpu-batch")]
use gz_physics_gpu::{
    BodySlot, GpuError, SpherePairBatch, SpherePairBlockBatch, SpherePairBlockCheckpoint,
    SphereRotation,
};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    ImuSample, Name, PhysicsEngine, Velocity, WorldLoadOptions, WorldPose,
    gpu_scene::{
        GpuSceneBindings, GpuSceneObservations, batch_boundary, execution_counts,
        queue_link_impulse, resident_pair_observation,
    },
    load_world_on_adapter, reset_loaded_world,
};
use sdformat::NoResources;
use std::time::Duration;
const SDF: &str = include_str!("../../../examples/gpu_resident_contacts.sdf");
fn load(xml: &str, adapter: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_on_adapter(
        xml,
        "resident.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(adapter),
    )
}
#[test]
fn resident_options_and_clock_are_checked_before_opening_a_device() {
    for (before, after, needle) in [
        (
            "ticks=\"8\"",
            "ticks=\"8\" workers=\"0\"",
            "workers must be",
        ),
        (
            "ticks=\"8\"",
            "ticks=\"8\" workers=\"17\"",
            "workers must be",
        ),
        (
            "ticks=\"8\"",
            "ticks=\"8\" workers=\"3\"",
            "nonempty environment",
        ),
        ("ticks=\"8\"", "ticks=\"0\"", "1 through 4096"),
        ("ticks=\"8\"", "ticks=\"4097\"", "1 through 4096"),
        (
            "rotation=\"fourth-order\"",
            "rotation=\"exact\"",
            "rotation must",
        ),
        (
            "<max_step_size>0.008",
            "<max_step_size>0.001",
            "public max_step_size",
        ),
        (
            "<observe every=\"2\"/>",
            "<resident_block ticks=\"8\"/>",
            "one resident_block",
        ),
        ("id=\"7\"", "execution=\"cpu\" id=\"7\"", "independent GPU"),
        (
            "id=\"41\"",
            "native_handoff=\"true\" id=\"41\"",
            "independent GPU",
        ),
        (
            "<observe every=\"2\"/>",
            "<automatic_placement/>",
            "independent GPU",
        ),
    ] {
        let e = load(&SDF.replace(before, after), "absent").err().unwrap();
        assert!(e.to_string().contains(needle), "{e}, expected {needle}");
    }
}
fn entity(sim: &Simulation, name: &str) -> bevy_ecs::entity::Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
        .id()
}
#[test]
#[ignore = "requires named f64 Vulkan hardware"]
fn substep_impacts_publish_at_the_sdf_clock_and_survive_reset() {
    let adapter = std::env::var("GZ_GPU_ADAPTER").unwrap();
    let mut s = load(SDF, &adapter).unwrap();
    assert_eq!(execution_counts(&s), Some((0, 2)));
    let initial = s
        .world()
        .resource::<SpherePairBlockBatch>()
        .lanes()
        .iter()
        .map(|r| r.state)
        .collect();
    let mut dense =
        SpherePairBatch::with_rotation(&adapter, initial, SphereRotation::FourthOrder).unwrap();
    let mut history = Vec::new();
    for epoch in 0..=1 {
        if epoch != 0 {
            reset_loaded_world(&mut s, |_| ()).unwrap();
            dense.reset();
        }
        let mut impulse_blocks = 0;
        let mut previous = resident_pair_observation(&s, 7).unwrap().state.a.velocity;
        for barrier in 1..=64u64 {
            let mut jn = [0.; 2];
            let mut count = [0u32; 2];
            for _ in 0..8 {
                dense.step().unwrap();
                for (i, pair) in dense.lanes().iter().enumerate() {
                    jn[i] += pair.normal_impulse;
                    count[i] += u32::from(pair.normal_impulse > 0.);
                }
            }
            s.step_exact(1).unwrap();
            for (i, (id, name)) in [(7, "impact"), (41, "clear")].into_iter().enumerate() {
                let obs = resident_pair_observation(&s, id).unwrap();
                assert_eq!(obs.state, dense.lanes()[i]);
                assert_eq!(obs.normal_impulse_sum, jn[i]);
                assert_eq!(obs.contact_ticks(), count[i]);
                if id == 41 {
                    assert_eq!(obs.contact_times(), None);
                }
                if jn[i] > 0. {
                    impulse_blocks += 1;
                    let (t, last) = obs.contact_times().unwrap();
                    assert!(t > 0. && last < 0.008);
                }
                for (side, slot, b) in [
                    ("a", BodySlot::A, obs.state.a),
                    ("b", BodySlot::B, obs.state.b),
                ] {
                    let model = entity(&s, &format!("default::{name}_{side}"));
                    let link = entity(&s, &format!("default::{name}_{side}::body"));
                    let w = s.world();
                    for e in [model, link] {
                        let p = w.get::<WorldPose>(e).unwrap();
                        assert_eq!(p.translation, b.position);
                        assert_eq!(p.rotation, b.orientation);
                    }
                    let v = w.get::<Velocity>(link).unwrap();
                    assert_eq!(v.linear, b.velocity);
                    assert_eq!(v.angular, b.angular_velocity);
                    let public = w.get::<EntityId>(link).unwrap().get();
                    assert_eq!(
                        w.resource::<GpuSceneBindings>().resolve(public),
                        Some((id, slot))
                    );
                }
            }
            let obs = *resident_pair_observation(&s, 7).unwrap();
            let imu = s
                .world()
                .get::<ImuSample>(entity(&s, "default::impact_a::body::imu"))
                .unwrap();
            assert_eq!(imu.epoch, epoch);
            assert_eq!(imu.time, Duration::from_millis(barrier * 8));
            let v = glam::DVec3::from_array(obs.state.a.velocity);
            let force =
                (v - glam::DVec3::from_array(previous)) / 0.008 - glam::DVec3::new(0., 0., -0.16);
            let body = glam::DQuat::from_array(obs.state.a.orientation).inverse() * force;
            assert!(
                body.distance(glam::DVec3::from_array(
                    imu.reading.as_ref().unwrap().specific_force
                )) < 1e-10
            );
            previous = obs.state.a.velocity;
            let sampled = s.world().resource::<GpuSceneObservations>();
            assert_eq!(sampled.tick(), barrier / 2 * 2);
            if epoch == 0 {
                history.push(obs);
            } else {
                assert_eq!(obs, history[barrier as usize - 1]);
            }
        }
        assert_eq!(impulse_blocks, 1);
        let cut = SpherePairBlockCheckpoint::capture_simulation(&s).unwrap();
        assert_eq!(cut.tick(), 512);
        assert_eq!(cut.last_block_ticks(), 8);
        assert_eq!(
            cut.observations(),
            s.world().resource::<SpherePairBlockBatch>().lanes()
        );
        let at = batch_boundary(&s);
        let e = entity(&s, "default::impact_a::body");
        let link_id = s.world().get::<EntityId>(e).unwrap().get();
        assert!(matches!(
            queue_link_impulse(&mut s, at, link_id, [0.; 3]),
            Err(GpuError::StaleBoundary)
        ));
        assert!(matches!(
            resident_pair_observation(&s, 404),
            Err(GpuError::UnknownEnvironment(404))
        ));
    }
}
