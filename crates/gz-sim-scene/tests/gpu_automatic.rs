// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "gpu-batch")]
use bevy_ecs::prelude::*;
use gz_physics_gpu::{
    BodySlot, CentralImpulse, Environment,
    ExecutionPlacement::{self, Cpu, Gpu},
    MixedBatch,
};
use gz_sim_core::{EntityId, Phase, Simulation, UpdateInfo};
use gz_sim_scene::{
    AltimeterSample, ImuSample, Name, PhysicsEngine, WorldLoadOptions,
    gpu_scene::{
        automatic::{
            AutoPlacementConfig, AutoPlacementError, AutomaticPlacement, PlacementReason,
            SamplingPhase, install_auto_placement, set_placement_workload,
        },
        batch_boundary, environment_placement, queue_link_impulse, set_environment_placement,
    },
    load_world_on_adapter, reset_loaded_world,
};
use sdformat::NoResources;
const WORLD: &str = include_str!("../../../examples/gpu_sensor_world.sdf");
const TAG: &str = "<automatic_placement window_ticks='8' min_residency_ticks='32' reprobe_ticks='96' hysteresis_percent='5'/>";
fn load(xml: &str, device: &str) -> Result<Simulation, gz_sim_scene::LoadError> {
    load_world_on_adapter(
        xml,
        "automatic.sdf",
        "default",
        &NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(device),
    )
}
#[test]
fn admission_rejects_invalid_windows_ambiguous_starting_owner_and_duplicate_policy() {
    for (tag, expected) in [
        (
            "<automatic_placement window_ticks='0'/>",
            "invalid automatic placement",
        ),
        (
            "<automatic_placement min_residency_ticks='3'/>",
            "invalid automatic placement",
        ),
        (
            "<automatic_placement reprobe_ticks='4'/>",
            "invalid automatic placement",
        ),
        (
            "<automatic_placement hysteresis_percent='60'/>",
            "invalid automatic placement",
        ),
        (
            "<automatic_placement window_ticks='x'/>",
            "invalid automatic placement",
        ),
        (
            "<automatic_placement foo='1'/>",
            "expected one automatic_placement",
        ),
        (
            "<automatic_placement/><automatic_placement/>",
            "expected one automatic_placement",
        ),
    ] {
        let text = WORLD.replace("<observe ", &format!("{tag}<observe "));
        let err = load(&text, "unused").err().unwrap();
        assert!(err.to_string().contains(expected), "{err:?}");
    }
    let text = WORLD
        .replace("<observe ", &format!("{TAG}<observe "))
        .replace("id=\"9\"", "id=\"9\" execution='cpu'");
    assert!(
        load(&text, "unused")
            .err()
            .unwrap()
            .to_string()
            .contains("uniform initial")
    );
    let mut plain = Simulation::new(std::time::Duration::from_millis(1)).unwrap();
    assert!(matches!(
        install_auto_placement(&mut plain, AutoPlacementConfig::default()),
        Err(AutoPlacementError::Population)
    ));
    let bad = AutoPlacementConfig {
        reprobe_ticks: 1,
        ..Default::default()
    };
    assert!(matches!(
        install_auto_placement(&mut plain, bad),
        Err(AutoPlacementError::Configuration)
    ));
    let at = batch_boundary(&plain);
    assert!(matches!(
        set_placement_workload(&mut plain, at, 1),
        Err(AutoPlacementError::Boundary)
    ));
}
/// Queue at Publish, after the current physics advance and before a possible
/// automatic decision at Finalize. The next advancing tick consumes exactly once.
fn queued(info: Res<UpdateInfo>, mut b: ResMut<MixedBatch>) {
    if !info.dt.is_zero() {
        b.queue_impulses(
            info.iteration,
            &[CentralImpulse {
                id: 31,
                body: BodySlot::A,
                impulse: [0., 0., 0.001],
            }],
        )
        .unwrap();
    }
}
#[derive(Debug, PartialEq)]
struct Sensors {
    time: u64,
    epoch: u64,
    values: Vec<u64>,
}
fn observe(s: &Simulation) -> Vec<Sensors> {
    let mut seen = vec![];
    for e in s.world().iter_entities() {
        let stamp = |t: std::time::Duration, epoch, values: Vec<f64>| Sensors {
            time: t.as_nanos() as u64,
            epoch,
            values: values.into_iter().map(f64::to_bits).collect(),
        };
        if let Some(a) = e.get::<AltimeterSample>() {
            let a0 = a.reading.as_ref().unwrap();
            seen.push((
                e.get::<EntityId>().unwrap().get(),
                stamp(
                    a.time,
                    a.epoch,
                    vec![
                        a0.vertical_position,
                        a0.vertical_velocity,
                        a0.vertical_reference,
                    ],
                ),
            ));
        }
        if let Some(a) = e.get::<ImuSample>() {
            let v = a.reading.as_ref().unwrap();
            seen.push((
                e.get::<EntityId>().unwrap().get(),
                stamp(
                    a.time,
                    a.epoch,
                    v.orientation
                        .into_iter()
                        .chain(v.specific_force)
                        .chain(v.angular_velocity)
                        .collect(),
                ),
            ));
        }
    }
    seen.sort_by_key(|(id, _)| *id);
    seen.into_iter().map(|(_, s)| s).collect()
}
fn device() -> String {
    std::env::var("GZ_GPU_ADAPTER").unwrap()
}
#[test]
#[ignore = "requires named Vulkan f64 device"]
fn auto_timing_can_be_replayed_exactly_with_sensor_stamps_and_pending_impulses() {
    let device = device();
    let text = WORLD.replace("<observe ", &format!("{TAG}<observe "));
    let mut sim = load(&text, &device).unwrap();
    sim.add_systems(Phase::Publish, queued);
    assert!(matches!(
        install_auto_placement(&mut sim, AutoPlacementConfig::default()),
        Err(AutoPlacementError::AlreadyInstalled)
    ));
    let mut replay = load(WORLD, &device).unwrap();
    replay.add_systems(Phase::Publish, queued);
    let ids = [9, 31, 80];
    let mut previous = Gpu;
    let mut witnessed = false;
    for t in 1..=190 {
        let old = batch_boundary(&sim);
        sim.tick().unwrap();
        assert_eq!(batch_boundary(&sim), old);
        sim.step_exact(1).unwrap();
        replay.step_exact(1).unwrap();
        let actual = sim.world().resource::<MixedBatch>();
        let reference = replay.world().resource::<MixedBatch>();
        let av: &[Environment] = actual.environments();
        assert_eq!(av, reference.environments());
        assert_eq!(
            actual.pending_impulses().collect::<Vec<_>>(),
            reference.pending_impulses().collect::<Vec<_>>()
        );
        assert_eq!(actual.pending_impulses().len(), 1);
        let physical = observe(&sim);
        assert_eq!(physical.len(), 4);
        assert_eq!(physical, observe(&replay));
        let owner = environment_placement(&sim, 9).unwrap();
        if owner != previous {
            assert_eq!(batch_boundary(&sim).tick, t);
            let at = batch_boundary(&replay);
            set_environment_placement(&mut replay, at, &ids, owner).unwrap();
            witnessed = true;
            previous = owner;
        }
    }
    assert!(
        witnessed,
        "the real device must receive and return the alternative probe"
    );
    let stale = batch_boundary(&sim);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert!(matches!(
        set_placement_workload(&mut sim, stale, 1),
        Err(AutoPlacementError::Boundary)
    ));
    assert!(
        sim.world()
            .resource::<AutomaticPlacement>()
            .events()
            .is_empty()
    );
    assert!(
        sim.world()
            .resource::<MixedBatch>()
            .pending_impulses()
            .next()
            .is_none()
    );
    assert!(observe(&sim).is_empty());
    assert_eq!(environment_placement(&sim, 9).unwrap(), Gpu);
    // Real public link IDs and the original sensor clocks survive a full reset.
    let link = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::free_a::action_free")
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    let at = batch_boundary(&sim);
    queue_link_impulse(&mut sim, at, link, [0., 0., 0.1]).unwrap();
    sim.step_exact(1).unwrap();
    assert!(
        observe(&sim)
            .iter()
            .all(|s| s.epoch == 1 && s.time == 1_000_000)
    );
}
#[test]
#[ignore = "requires named Vulkan f64 device"]
fn changing_workload_cancels_a_probe_and_manual_partial_ownership_suspends_decisions() {
    let mut sim = load(
        &WORLD.replace("<observe ", &format!("{TAG}<observe ")),
        &device(),
    )
    .unwrap();
    for _ in 0..2000 {
        sim.step_exact(1).unwrap();
        if sim.world().resource::<AutomaticPlacement>().phase() == SamplingPhase::Alternative {
            break;
        }
    }
    assert_eq!(environment_placement(&sim, 9).unwrap(), Cpu);
    let at = batch_boundary(&sim);
    set_placement_workload(&mut sim, at, 12).unwrap();
    assert_eq!(environment_placement(&sim, 9).unwrap(), Gpu);
    assert_eq!(sim.world().resource::<AutomaticPlacement>().workload(), 12);
    let count = sim.world().resource::<AutomaticPlacement>().events().len();
    set_placement_workload(&mut sim, at, 12).unwrap();
    assert_eq!(
        sim.world().resource::<AutomaticPlacement>().events().len(),
        count
    );
    let at = batch_boundary(&sim);
    set_environment_placement(&mut sim, at, &[9], Cpu).unwrap();
    let p = sim.world().resource::<AutomaticPlacement>();
    assert_eq!(p.phase(), SamplingPhase::Manual);
    assert_eq!(p.events().back().unwrap().owner, None);
    assert_eq!(
        p.events().back().unwrap().reason,
        PlacementReason::ManualOverride
    );
    assert!(matches!(
        set_placement_workload(&mut sim, at, 99),
        Err(AutoPlacementError::Manual)
    ));
    sim.step_exact(80).unwrap();
    assert_eq!(
        environment_placement(&sim, 31).unwrap(),
        ExecutionPlacement::Gpu
    );
    assert_eq!(
        environment_placement(&sim, 9).unwrap(),
        ExecutionPlacement::Cpu
    );
    let at = batch_boundary(&sim);
    assert!(matches!(
        gz_sim_scene::gpu_scene::automatic::resume_auto_placement(&mut sim, at),
        Err(AutoPlacementError::Population)
    ));
    set_environment_placement(&mut sim, at, &[9], Gpu).unwrap();
    gz_sim_scene::gpu_scene::automatic::resume_auto_placement(&mut sim, at).unwrap();
    assert_eq!(
        sim.world().resource::<AutomaticPlacement>().phase(),
        SamplingPhase::Waiting
    );
    assert_eq!(
        sim.world()
            .resource::<AutomaticPlacement>()
            .events()
            .back()
            .unwrap()
            .reason,
        PlacementReason::Resumed
    );
}

#[test]
#[ignore = "requires named Vulkan f64 device"]
fn profiling_validates_world_ownership_initial_placement_and_bounded_history() {
    let device = device();
    let mut plain = Simulation::new(std::time::Duration::from_millis(1)).unwrap();
    let at = batch_boundary(&plain);
    assert!(matches!(
        gz_sim_scene::gpu_scene::automatic::resume_auto_placement(&mut plain, at),
        Err(AutoPlacementError::Boundary)
    ));
    let mut sim = load(
        &WORLD.replace(" profile=", " execution='cpu' profile="),
        &device,
    )
    .unwrap();
    install_auto_placement(&mut sim, AutoPlacementConfig::default()).unwrap();
    assert_eq!(sim.world().resource::<AutomaticPlacement>().probes(), 0);
    let at = batch_boundary(&sim);
    for key in 1..=84 {
        set_placement_workload(&mut sim, at, key).unwrap();
    }
    let p = sim.world().resource::<AutomaticPlacement>();
    assert_eq!(p.events().len(), 64);
    assert_eq!(p.events().front().unwrap().workload, 21);
    set_environment_placement(&mut sim, at, &[9], Gpu).unwrap();
    let history = sim.world().resource::<AutomaticPlacement>().events().len();
    set_environment_placement(&mut sim, at, &[9], Gpu).unwrap();
    assert_eq!(
        sim.world().resource::<AutomaticPlacement>().events().len(),
        history
    );
    // Bypassing the loaded-scene control remains observable; a partial physical
    // cohort must never be claimed as CPU or GPU by an active automatic policy.
    set_environment_placement(&mut sim, at, &[9], Cpu).unwrap();
    gz_sim_scene::gpu_scene::automatic::resume_auto_placement(&mut sim, at).unwrap();
    sim.world_mut()
        .resource_mut::<MixedBatch>()
        .set_placement(at.tick, &[9], Gpu)
        .unwrap();
    assert!(matches!(
        set_placement_workload(&mut sim, at, 85),
        Err(AutoPlacementError::Boundary)
    ));
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().resource::<AutomaticPlacement>().phase(),
        SamplingPhase::Manual
    );
    let event = sim
        .world()
        .resource::<AutomaticPlacement>()
        .events()
        .back()
        .unwrap();
    assert_eq!(
        (event.reason, event.owner),
        (PlacementReason::OwnershipChanged, None)
    );
    assert!(matches!(
        gz_sim_scene::gpu_scene::automatic::resume_auto_placement(&mut sim, at),
        Err(AutoPlacementError::Boundary)
    ));
    let now = batch_boundary(&sim);
    let bindings = sim
        .world_mut()
        .remove_resource::<gz_sim_scene::gpu_scene::GpuSceneBindings>()
        .unwrap();
    assert!(matches!(
        set_placement_workload(&mut sim, now, 1),
        Err(AutoPlacementError::Boundary)
    ));
    assert!(matches!(
        gz_sim_scene::gpu_scene::automatic::resume_auto_placement(&mut sim, now),
        Err(AutoPlacementError::Boundary)
    ));
    sim.world_mut().insert_resource(bindings);
    let state = sim
        .world_mut()
        .remove_resource::<AutomaticPlacement>()
        .unwrap();
    assert!(matches!(
        install_auto_placement(&mut sim, AutoPlacementConfig::default()),
        Err(AutoPlacementError::Population)
    ));
    sim.world_mut()
        .resource_mut::<MixedBatch>()
        .set_placement(now.tick, &[9], Cpu)
        .unwrap();
    assert!(matches!(
        install_auto_placement(&mut sim, AutoPlacementConfig::default()),
        Err(AutoPlacementError::AlreadyInstalled)
    ));
    assert!(matches!(
        gz_sim_scene::gpu_scene::automatic::resume_auto_placement(&mut sim, now),
        Err(AutoPlacementError::Boundary)
    ));
    sim.world_mut().insert_resource(state);
    sim.world_mut().resource_mut::<MixedBatch>().step().unwrap();
    assert!(matches!(
        set_placement_workload(&mut sim, now, 99),
        Err(AutoPlacementError::Boundary)
    ));
    assert!(matches!(
        gz_sim_scene::gpu_scene::automatic::resume_auto_placement(&mut sim, now),
        Err(AutoPlacementError::Boundary)
    ));
    let mut other = load(
        &WORLD.replace("id=\"9\"", "id=\"9\" execution='cpu'"),
        &device,
    )
    .unwrap();
    let t = batch_boundary(&other);
    set_environment_placement(&mut other, t, &[9], Gpu).unwrap();
    assert!(matches!(
        install_auto_placement(&mut other, AutoPlacementConfig::default()),
        Err(AutoPlacementError::Population)
    ));
    let foreign = other.world_mut().remove_resource::<MixedBatch>().unwrap();
    sim.world_mut().insert_resource(foreign);
    sim.world_mut().remove_resource::<AutomaticPlacement>();
    assert!(matches!(
        install_auto_placement(&mut sim, AutoPlacementConfig::default()),
        Err(AutoPlacementError::Boundary)
    ));
    sim.world_mut()
        .remove_resource::<gz_sim_scene::gpu_scene::GpuSceneBindings>();
    assert!(matches!(
        install_auto_placement(&mut sim, AutoPlacementConfig::default()),
        Err(AutoPlacementError::Population)
    ));
}
