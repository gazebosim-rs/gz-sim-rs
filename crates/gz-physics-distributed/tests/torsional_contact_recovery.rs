// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use clap::Parser;
use gz_physics_distributed::pool::NativeContactSnapshot;
use serde_json::Value;
use std::{error::Error, fs};

#[path = "../examples/torsional_contact.rs"]
mod pool;
#[path = "../examples/torsional_contact_sdf.rs"]
mod scene;

#[test]
fn moving_rectangle_preserves_four_positive_native_points_and_physical_braking_through_faults()
-> Result<(), Box<dyn Error>> {
    let dir = tempfile::tempdir()?;
    let file = dir.path().join("table");
    let args = pool::Args::try_parse_from([
        "torsional_contact",
        "--worker",
        env!("CARGO_BIN_EXE_physics_worker"),
        "--rectangular",
        "--moving",
        "--packed",
        file.to_str().unwrap(),
    ])?;
    pool::run(args)?;
    let row: Value = serde_json::from_slice(&fs::read(file.join("summary.json"))?)?;
    assert_eq!(row["saved_native_patch_comparisons"], 12);
    assert!((row["total_reaction_nms"].as_f64().unwrap() + 0.96).abs() < 1e-6);
    assert!(
        (row["measured_stopping_s"].as_f64().unwrap() - row["ideal_stopping_s"].as_f64().unwrap())
            .abs()
            < 0.02
    );
    Ok(())
}

#[test]
fn public_reversed_spin_archive_has_four_loaded_points_on_the_new_owner_and_no_ground()
-> Result<(), Box<dyn Error>> {
    let dir = tempfile::tempdir()?;
    let file = dir.path().join("scene");
    let args = scene::Args::try_parse_from([
        "torsional_contact_sdf",
        "--worker",
        env!("CARGO_BIN_EXE_physics_worker"),
        "--reverse",
        file.to_str().unwrap(),
    ])?;
    scene::run(args)?;
    let cut = NativeContactSnapshot::open(file.join("public-38.gzscene"))?;
    assert_eq!(cut.frame.tick, 38);
    assert!(cut.patches.iter().all(|w| w.ground.is_empty()));
    let loaded: Vec<_> = cut
        .patches
        .iter()
        .enumerate()
        .flat_map(|(owner, w)| w.finite.iter().map(move |p| (owner, p)))
        .collect();
    assert_eq!(loaded.len(), 1);
    let (owner, patch) = loaded[0];
    assert_eq!(owner, 1);
    assert_eq!((patch.a, patch.b), (0, 1));
    assert_eq!(patch.points.len(), 4);
    assert!(patch.points.iter().all(|p| p.normal_impulse > 0.));
    Ok(())
}

#[test]
fn authored_slide_and_yaw_recover_the_selected_law_and_reject_cross_model_resume()
-> Result<(), Box<dyn Error>> {
    use gz_sim_scene::{
        PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions, passive_checkpoint::PassivePoolScene,
    };
    use std::path::Path;

    let dir = tempfile::tempdir()?;
    let mut endpoints = Vec::new();
    for point in [false, true] {
        let path = dir.path().join(if point { "point" } else { "grouped" });
        let mut flags = vec![
            "torsional_contact_sdf",
            "--worker",
            env!("CARGO_BIN_EXE_physics_worker"),
            "--slide",
        ];
        if point {
            flags.push("--per-point");
        }
        flags.push(path.to_str().unwrap());
        scene::run(scene::Args::try_parse_from(flags)?)?;
        let report: Value = serde_json::from_slice(&fs::read(path.join("summary.json"))?)?;
        endpoints.push(report["final_position_m"][0].as_f64().unwrap());
        let cut = path.join("public-38.gzscene");
        let err = PassivePoolScene::resume(
            include_str!("../../../examples/distributed_torsional_contact.sdf"),
            "yaw.sdf",
            "default",
            WorldLoadOptions {
                engine: PhysicsEngine::DistributedIslands(4),
                step: None,
                noise_seed: [0; 32],
                distributed_worker: Some(Path::new("/missing/check-profile-before-worker-spawn")),
            },
            WorldRuntimeOptions {
                rapier_per_point_friction: !point,
                distributed_contact_placement: true,
                distributed_packed_handoffs: true,
                ..Default::default()
            },
            &cut,
        )
        .err()
        .expect("cross-model resume must fail before launching a worker");
        assert!(
            matches!(err, gz_sim_scene::passive_checkpoint::Error::Domain(_)),
            "{err}"
        );
    }
    assert!((endpoints[0] - endpoints[1]).abs() > 0.1, "{endpoints:?}");
    Ok(())
}
