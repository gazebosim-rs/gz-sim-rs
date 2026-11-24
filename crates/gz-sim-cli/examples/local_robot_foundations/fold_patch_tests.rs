// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn quadrature_preserves_patch_area_centroid_and_material_locations_after_remeshing() {
    for columns in [3, 5, 7] {
        let mut weight = [0.; 2];
        let mut moment = [DVec3::ZERO; 2];
        for site in sites(0.0425) {
            let s = site.stencil(columns);
            let point = s
                .particles
                .into_iter()
                .zip(s.weights)
                .map(|(i, w)| {
                    let u = (i % columns) as f64 / (columns - 1) as f64;
                    let row = i / columns;
                    arm_sheet::seed(3 * row, 0.).lerp(arm_sheet::seed(3 * row + 2, 0.), u) * w
                })
                .sum::<DVec3>();
            let base = arm_sheet::seed(site.panel * 9, 0.);
            assert!(
                (point - DVec3::new(base.x, site.width_m, base.z + site.height_m)).length() < 1e-15
            );
            assert!(s.weights.iter().all(|&w| w >= 0.));
            weight[site.panel] += s.stiffness_weight;
            moment[site.panel] += point * s.stiffness_weight;
        }
        assert_eq!(weight, [2.; 2]);
        assert!(moment.iter().all(|m| m.y.abs() < 1e-15
            && (m.z / 2. - (arm_sheet::BENCH + RADIUS + 0.043 / 2.)).abs() < 1e-15));
    }
}

#[test]
fn short_hold_uses_the_same_jaws_and_archives_the_actual_mesh_particles() -> Result<()> {
    let out = tempfile::tempdir()?;
    run_hold(out.path(), 2)?;
    let read = |p: PathBuf| -> Result<Value> { Ok(serde_json::from_slice(&fs::read(p)?)?) };
    let report = read(out.path().join("comparison.json"))?;
    assert_eq!(report["ticks"], 2);
    for t in report["trials"].as_array().unwrap() {
        let columns = t["columns"].as_u64().unwrap() as usize;
        assert_eq!(t["status"], "completed");
        assert_eq!(t["completed_ticks"], 2);
        assert!(t["min_mid_surface_jaw_core_clearance_m"].as_f64().unwrap() > 0.01);
        assert!(t["max_pad_attribution_residual_ns"].as_f64().unwrap() < 1e-10);
        let normal = t["mean_pad_normal_by_jaw_panel_n"].as_array().unwrap();
        assert!(normal[0][0].as_f64().unwrap() > 0.5 && normal[1][1].as_f64().unwrap() > 0.5);
        assert_eq!(
            (normal[0][1].as_f64(), normal[1][0].as_f64()),
            (Some(0.), Some(0.))
        );
        let sub = out
            .path()
            .join(format!("{}-{columns}", t["model"].as_str().unwrap()));
        let trace = read(sub.join("cross-panel-force.json"))?;
        let forces = trace["cross_panel_n"].as_array().unwrap();
        assert_eq!(forces.len(), 2);
        assert_eq!(t["first_cross_panel_n"], forces[0]);
        assert_eq!(t["trailing_window_ticks"], 2);
        let mean = forces.iter().map(|f| f.as_f64().unwrap()).sum::<f64>() / 2.;
        assert!((t["trailing_mean_cross_panel_n"].as_f64().unwrap() - mean).abs() < 1e-15);
        if columns == 3 {
            assert!(t["ticks_with_cross_panel_impulse"].as_u64().unwrap() > 0);
        } else {
            assert_eq!(t["ticks_with_cross_panel_impulse"], 0);
            assert!(t["last_cross_panel_tick"].is_null());
        }
        let samples = read(sub.join("particle-frames.json"))?;
        let archive = fs::read_to_string(sub.join("replay/frames.jsonl"))?;
        let mut seen = 0;
        for line in archive.lines() {
            let e: Value = serde_json::from_str(line)?;
            if e["kind"] != "frame" {
                continue;
            }
            let scene: Value = serde_json::from_str(e["scene"].as_str().unwrap())?;
            let positions = samples[seen]["positions"].as_array().unwrap();
            assert_eq!(positions.len(), 4 * columns);
            assert_eq!(
                samples[seen]["time_ns"],
                1_650_000_000_u64 + seen as u64 * 2_000_000
            );
            for (i, p) in positions.iter().enumerate() {
                let marker = scene["visuals"]
                    .as_array()
                    .unwrap()
                    .iter()
                    .find(|v| v["name"] == arm_sheet::name(i))
                    .unwrap();
                assert_eq!(marker["translation"], *p);
            }
            seen += 1;
        }
        assert_eq!(seen, 2);
    }
    Ok(())
}

#[test]
fn absent_support_or_a_core_crossing_cannot_create_a_successful_patch_tick() -> Result<()> {
    let anchor = Anchor {
        loaded: (0..12)
            .map(|i| Particle {
                position: arm_sheet::seed(i, 0.).to_array(),
                inverse_mass: 1. / 0.0007,
                velocity: [0.; 3],
            })
            .collect(),
        half: [DVec3::new(0.012, 0.0425, 0.07); 2],
    };
    let jaw = |side: f64| BodyState {
        position: [
            0.32 + side * (0.012 + 2. * RADIUS),
            0.,
            arm_sheet::BENCH + 0.04,
        ],
        orientation: DQuat::IDENTITY.to_array(),
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    };
    let jaws = [jaw(-1.), jaw(1.)];
    let mut away = jaws;
    away[1].position[1] = 1.;
    assert!(Branch::new(&anchor, 3, away, true).is_err());
    let mut branch = Branch::new(&anchor, 3, jaws, true)?;
    let prior = branch.body.particles().to_vec();
    let mut buried = jaws;
    buried[0].position[0] += 0.023;
    branch.advance(anchor.half, &[jaws, buried])?;
    assert!(branch.refusal.is_some());
    branch.advance(anchor.half, &[jaws, jaws])?;
    assert!(branch.times.is_empty() && branch.cross_forces.is_empty());
    assert_eq!(prior, branch.body.particles());
    Ok(())
}

fn loaded_fixture() -> Result<(Recording, Anchor, [BodyState; 2], Vec<u64>)> {
    let xml = arm_sheet::scene_with_loads(
        arm::SCENE,
        arm_sheet::Case {
            substeps: 8,
            ..Default::default()
        },
        0.,
        true,
    )?;
    let mut r = Recording::new(&xml, "fold_patch_test.sdf", "default")?;
    let controller = arm::Controller::new(&r.sim)?;
    for tick in 1..=MATCH {
        fold_hold::drive(&mut r, &controller, tick)?;
    }
    let anchor = Anchor::capture(&r.sim)?;
    let jaw = |suffix: &str| -> Result<BodyState> {
        state(
            &r.sim,
            entity_id(&r.sim, &format!("default::arm::{suffix}"))?,
        )
    };
    let jaws = [jaw("left")?, jaw("right")?];
    let ids = (0..12)
        .map(|i| entity_id(&r.sim, &arm_sheet::name(i)))
        .collect::<Result<_>>()?;
    Ok((r, anchor, jaws, ids))
}

#[test]
fn refused_archive_keeps_the_last_captured_state_and_reports_partial_elapsed_time() -> Result<()> {
    let (mut r, anchor, jaws, ids) = loaded_fixture()?;
    let mut buried = jaws;
    buried[0].position[0] += 0.023;
    for completed in [0, 1] {
        let mut branch = Branch::new(&anchor, 3, jaws, true)?;
        branch.capture(&mut r, &ids)?;
        for _ in 0..completed {
            branch.advance(anchor.half, &[jaws, jaws])?;
        }
        branch.advance(anchor.half, &[jaws, buried])?;
        let dest = tempfile::tempdir()?;
        let report = branch.save(dest.path())?;
        assert_eq!(report["status"], "refused");
        assert!(report["refusal"].is_string());
        assert_eq!(report["completed_ticks"], completed);
        let directory = dest.path().join("patch-3");
        let read = |file: &str| -> Result<Value> {
            Ok(serde_json::from_slice(&fs::read(directory.join(file))?)?)
        };
        let metadata = read("frames.json")?;
        assert_eq!(
            metadata["last_committed_ns"],
            (MATCH + completed) * 1_000_000
        );
        assert_eq!(metadata["last_captured_ns"], MATCH * 1_000_000);
        assert_eq!(read("replay/manifest.json")?["frames"], 1);
        assert_eq!(
            read("cross-panel-force.json")?["cross_panel_n"]
                .as_array()
                .unwrap()
                .len(),
            completed
        );
        if completed == 0 {
            assert!(
                report["first_cross_panel_n"].is_null()
                    && report["min_held_panel_skin_gap_m"].is_null()
            );
            assert!(
                report.get("step_p95_ns").is_none()
                    && report.get("trailing_mean_cross_panel_n").is_none()
            );
        }
    }
    Ok(())
}

#[test]
fn archive_and_pose_errors_cannot_publish_a_partial_comparison() -> Result<()> {
    let (mut r, anchor, jaws, ids) = loaded_fixture()?;
    let dir = tempfile::tempdir()?;
    let dest = dir.path().join("complete");
    for filename in [
        "",
        "replay",
        "particle-frames.json",
        "frames.json",
        "cross-panel-force.json",
    ] {
        let mut branch = Branch::new(&anchor, 3, jaws, true)?;
        branch.capture(&mut r, &ids)?;
        let err = write_new(&dest, |stage| {
            let lane = stage.join("patch-3");
            if filename.is_empty() {
                fs::write(lane, "this path is a file")?;
            } else {
                fs::create_dir(&lane)?;
                fs::create_dir(lane.join(filename))?;
            }
            branch.save(stage)?;
            Ok(())
        });
        assert!(
            err.is_err() && !dest.exists(),
            "failed to refuse occupied {filename}"
        );
    }
    let mut branch = Branch::new(&anchor, 3, jaws, true)?;
    assert!(branch.capture(&mut r, &[u64::MAX]).is_err());
    let first = entity_by_id(r.sim.world(), ids[0]).unwrap();
    r.sim.world_mut().entity_mut(first).remove::<WorldPose>();
    assert!(branch.capture(&mut r, &ids).is_err());
    assert!(branch.frames.is_empty() && branch.observations.is_empty());
    Ok(())
}

#[test]
fn a_later_replay_failure_rolls_back_the_already_saved_mesh_pair() -> Result<()> {
    let out = tempfile::tempdir()?;
    let destination = out.path().join("comparison");
    let error = write_new(&destination, |stage| {
        fs::write(stage.join("patch-5"), "occupied")?;
        let result = run_hold(stage, 1);
        assert!(stage.join("patch-3/replay/manifest.json").is_file());
        assert!(stage.join("particle-5/replay/manifest.json").is_file());
        result
    })
    .unwrap_err();
    assert_eq!(
        error.downcast_ref::<std::io::Error>().unwrap().kind(),
        std::io::ErrorKind::AlreadyExists
    );
    assert!(!destination.exists());
    assert_eq!(fs::read_dir(out.path())?.count(), 0);
    Ok(())
}
