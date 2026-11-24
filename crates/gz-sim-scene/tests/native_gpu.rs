// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "gpu-batch")]
#[path = "../examples/support/native_pair_drive.rs"]
mod runner;
use gz_physics_gpu::{BodySlot, CentralImpulse, MixedBatch};
use gz_sim_core::EntityId;
use gz_sim_scene::{
    Name, PhysicsEngine, WorldLoadOptions,
    gpu_scene::{
        self, GpuError, GpuSceneObservations, NativeHandoffRefusal as Refusal,
        PairSolver::{NativeRapier, SpecializedGpu},
        batch_boundary,
    },
    load_world_on_adapter, reset_loaded_world,
};
use runner::{Drive, Event, IDS, SDF};
fn device() -> String {
    std::env::var("GZ_GPU_ADAPTER").unwrap()
}
#[test]
fn loaded_scope_rejects_guards_on_nonpairs_and_conflicts_before_opening_a_device() {
    for (xml, text) in [
        (
            SDF.replace(
                "<observe ",
                "<native_staging workers='2' overlap_gpu='sometimes'/><observe ",
            ),
            "overlap_gpu must be true or false",
        ),
        (
            SDF.replace("<observe ", "<native_staging workers='0'/><observe "),
            "between 1 and 4",
        ),
        (
            SDF.replace("<observe ", "<native_staging workers='5'/><observe "),
            "between 1 and 4",
        ),
        (
            SDF.replace(
                "<observe ",
                "<native_staging workers='2'/><native_staging workers='3'/><observe ",
            ),
            "one native_staging",
        ),
        (
            SDF.replace("native_handoff=\"true\"", "native_handoff=\"false\"")
                .replace("execution=\"native\"", "execution=\"gpu\"")
                .replace("<observe ", "<native_staging workers='2'/><observe "),
            "requires marked native contacts",
        ),
        (
            SDF.replacen("angular=\"0.4 1.8 1\"", "angular=\"0 9000 0\"", 1),
            "substep envelope",
        ),
        (
            SDF.replacen("0.035 2.17 0.82", "0.035 1.77 0.82", 1),
            "swept surface clearance",
        ),
        (
            SDF.replacen("native_handoff=\"true\"", "native_handoff=\"sometimes\"", 1),
            "true or false",
        ),
        (
            SDF.replacen("execution=\"native\"", "execution=\"cpu\"", 1),
            "execution must be native or gpu",
        ),
        (
            SDF.replacen("native_handoff=\"true\"", "native_handoff=\"false\"", 1),
            "unmarked",
        ),
        (
            SDF.replacen("profile=\"sphere-pair\"", "profile=\"supported-pair\"", 1),
            "requires a sphere-pair or sphere-plane",
        ),
        (
            SDF.replace("<observe ", "<automatic_placement/><observe "),
            "cannot own guarded native pairs",
        ),
    ] {
        let err = load_world_on_adapter(
            &xml,
            "test.sdf",
            "default",
            &sdformat::NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::GpuBatch,
                step: None,
                noise_seed: [0; 32],
                distributed_worker: None,
            },
            Some("unused"),
        )
        .err()
        .unwrap();
        assert!(err.to_string().contains(text), "{err}");
    }
}
#[test]
#[ignore = "requires named Vulkan f64 device"]
fn normal_sdf_roundtrip_enforces_both_solver_histories_and_reproduces_through_reset() {
    let mut sim = runner::load(&device()).unwrap();
    let mut history = vec![];
    for epoch in 0..2 {
        let mut drive = Drive::new(&sim);
        let mut impact = [false; 3];
        let mut accepted = 0;
        for t in 0..=700usize {
            if t > 0 {
                sim.step_exact(1).unwrap();
            }
            let states = sim
                .world()
                .resource::<GpuSceneObservations>()
                .environments()
                .to_vec();
            if epoch == 0 {
                history.push(states);
            } else {
                runner::same_frame_bits(&history[t], &states);
            }
            for event in drive.boundary(&mut sim).unwrap() {
                match event {
                    Event::Moved(_, _) => accepted += 1,
                    Event::ContactRefusal(id) => {
                        impact[IDS.iter().position(|i| *i == id).unwrap()] = true
                    }
                    _ => {}
                }
            }
        }
        assert_eq!(accepted, 8);
        assert_eq!(impact, [true; 3]);
        reset_loaded_world(&mut sim, |_| ()).unwrap();
        assert_eq!(gpu_scene::execution_counts(&sim), Some((2, 1)));
    }
}
#[test]
#[ignore = "requires named Vulkan f64 device"]
fn queued_input_and_mixed_id_failures_are_transactional_at_the_scene_boundary() {
    let mut sim = runner::load(&device()).unwrap();
    let mut control = runner::load(&device()).unwrap();
    let start = batch_boundary(&sim);
    for (ids, expected) in [
        (&[31, 31][..], GpuError::DuplicateEnvironment(31)),
        (&[31, u64::MAX][..], GpuError::UnknownEnvironment(u64::MAX)),
    ] {
        let error = gpu_scene::set_pair_solver(&mut sim, start, ids, NativeRapier).unwrap_err();
        assert_eq!(error.to_string(), expected.to_string());
        assert_eq!(gpu_scene::pair_solver(&sim, 31).unwrap(), SpecializedGpu);
        assert_eq!(
            sim.world().resource::<MixedBatch>().environments(),
            control.world().resource::<MixedBatch>().environments()
        );
    }
    // Approach to within ~8 mm. The admissible pending action crosses that gap
    // before the next 1 ms step, so it stays with the original GPU contact law.
    sim.step_exact(310).unwrap();
    control.step_exact(310).unwrap();
    let at = batch_boundary(&sim);
    let ids = [9, 31, 80];
    let snapshots = sim.world().resource::<MixedBatch>().environments().to_vec();
    for s in [&mut sim, &mut control] {
        let id = s
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::pair_31_a::body")
            })
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        gpu_scene::queue_link_impulse(s, at, id, [0., -25., 0.]).unwrap();
    }
    let previous = sim
        .world()
        .resource::<MixedBatch>()
        .pending_impulses()
        .copied()
        .collect::<Vec<CentralImpulse>>();
    assert_eq!(previous[0].body, BodySlot::A);
    assert!(matches!(
        gpu_scene::set_pair_solver(&mut sim, at, &[31], NativeRapier),
        Err(GpuError::NativeHandoff {
            reason: Refusal::SweptClearance,
            ..
        })
    ));
    assert_eq!(
        sim.world().resource::<MixedBatch>().environments(),
        snapshots
    );
    assert_eq!(
        sim.world()
            .resource::<MixedBatch>()
            .pending_impulses()
            .copied()
            .collect::<Vec<_>>(),
        previous
    );
    assert_eq!(gpu_scene::pair_solver(&sim, 31).unwrap(), SpecializedGpu);
    assert!(matches!(
        gpu_scene::set_environment_placement(
            &mut sim,
            at,
            &ids,
            gpu_scene::ExecutionPlacement::Cpu
        ),
        Err(GpuError::NativeHandoff {
            reason: Refusal::GuardRequired,
            ..
        })
    ));
    sim.step_exact(1).unwrap();
    control.step_exact(1).unwrap();
    assert_eq!(
        sim.world().resource::<MixedBatch>().environments(),
        control.world().resource::<MixedBatch>().environments()
    );
    assert!(matches!(
        gpu_scene::set_pair_solver(&mut sim, at, &ids, SpecializedGpu),
        Err(GpuError::StaleBoundary)
    ));
}

#[test]
#[ignore = "requires named Vulkan f64 device"]
fn one_loaded_plane_refuses_atomic_move_with_separate_uncoupled_flying_pair() {
    const MIXED: &str = include_str!("../../../examples/gpu_mixed_world.sdf");
    let sdf = MIXED
        .replace(
            "profile=\"sphere-plane\"",
            "profile=\"sphere-plane\" native_handoff=\"true\" execution=\"native\"",
        )
        .replace(
            "profile=\"sphere-pair\"",
            "profile=\"sphere-pair\" native_handoff=\"true\" execution=\"native\"",
        );
    let mut sim = load_world_on_adapter(
        &sdf,
        "mixed.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::GpuBatch,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        Some(&device()),
    )
    .unwrap();
    sim.step_exact(600).unwrap();
    let at = batch_boundary(&sim);
    let old = sim.world().resource::<MixedBatch>().environments().to_vec();
    let witness = gpu_scene::native_contact(&sim, 9).unwrap().unwrap();
    assert!(witness.active_solver_contact && witness.warmstart_normal_impulse > 1e-8);
    assert!(matches!(
        gpu_scene::set_pair_solver(&mut sim, at, &[31, 9], SpecializedGpu),
        Err(GpuError::NativeHandoff {
            id: 9,
            reason: Refusal::ContactHistory
        })
    ));
    assert_eq!(gpu_scene::pair_solver(&sim, 31).unwrap(), NativeRapier);
    assert_eq!(gpu_scene::native_contact(&sim, 9).unwrap(), Some(witness));
    runner::same_frame_bits(&old, sim.world().resource::<MixedBatch>().environments());
    assert_eq!(
        gpu_scene::set_pair_solver(&mut sim, at, &[31], SpecializedGpu)
            .unwrap()
            .moved,
        1
    );
    assert_eq!(gpu_scene::execution_counts(&sim), Some((1, 2)));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(gpu_scene::execution_counts(&sim), Some((2, 1)));
    assert_eq!(
        gpu_scene::native_contact(&sim, 9).unwrap(),
        Some(Default::default())
    );
}

#[test]
#[ignore = "requires named Vulkan f64 device"]
fn ordinary_sdf_worker_budget_reproduces_serial_support_clocks_and_reset() {
    let xml = include_str!("../../../examples/native_gpu_support.sdf");
    let adapter = device();
    let load = |s: &str| {
        load_world_on_adapter(
            s,
            "support.sdf",
            "default",
            &sdformat::NoResources,
            WorldLoadOptions {
                engine: PhysicsEngine::GpuBatch,
                step: None,
                noise_seed: [0; 32],
                distributed_worker: None,
            },
            Some(&adapter),
        )
    };
    let mut serial = load(xml).unwrap();
    let mut staged =
        load(&xml.replace("<observe ", "<native_staging workers='4'/><observe ")).unwrap();
    let mut selected = load(&xml.replace(
        "<observe ",
        "<native_staging workers='4' overlap_gpu='true'/><observe ",
    ))
    .unwrap();
    for epoch in 0..2 {
        for t in 0..=265 {
            if t > 0 {
                serial.step_exact(1).unwrap();
                staged.step_exact(1).unwrap();
                selected.step_exact(1).unwrap();
            }
            let s = serial.world().resource::<MixedBatch>();
            let p = selected.world().resource::<MixedBatch>();
            assert_eq!(
                (s.native_staging_workers(), p.native_staging_workers()),
                (1, 4)
            );
            assert!(!s.overlaps_native_gpu() && p.overlaps_native_gpu());
            let host = staged.world().resource::<MixedBatch>();
            assert_eq!(
                (host.native_staging_workers(), host.overlaps_native_gpu()),
                (4, false)
            );
            runner::same_frame_bits(host.environments(), p.environments());
            runner::same_frame_bits(s.environments(), p.environments());
            assert_eq!(
                gpu_scene::batch_boundary(&serial),
                gpu_scene::batch_boundary(&selected)
            );
            assert_eq!(gpu_scene::batch_boundary(&selected).epoch, epoch);
            for id in [9, 31, 80] {
                assert_eq!(s.native_contact(id).unwrap(), p.native_contact(id).unwrap());
            }
        }
        assert!(
            selected
                .world()
                .resource::<MixedBatch>()
                .native_contact(9)
                .unwrap()
                .unwrap()
                .warmstart_normal_impulse
                > 0.
        );
        for s in [&mut serial, &mut selected, &mut staged] {
            reset_loaded_world(s, |_| ()).unwrap();
        }
    }
}

#[test]
fn native_queries_and_link_commands_require_a_bound_loaded_world() {
    let mut sim = gz_sim_core::Simulation::new(gz_physics_gpu::STEP).unwrap();
    assert_eq!(gpu_scene::execution_counts(&sim), None);
    assert!(matches!(
        gpu_scene::native_contact(&sim, 9),
        Err(gpu_scene::GpuError::StaleBoundary)
    ));
    assert!(matches!(
        gpu_scene::pair_solver(&sim, 9),
        Err(gpu_scene::GpuError::StaleBoundary)
    ));
    let at = batch_boundary(&sim);
    assert!(matches!(
        gpu_scene::queue_link_impulse(&mut sim, at, 9, [0.; 3]),
        Err(gpu_scene::GpuError::UnknownSceneLink(9))
    ));
}
