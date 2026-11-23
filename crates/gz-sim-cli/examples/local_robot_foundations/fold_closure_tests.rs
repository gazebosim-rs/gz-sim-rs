// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn samples(stage: usize, force: f64) -> Vec<Sample> {
    (0..PLATEAU)
        .map(|tick| Sample {
            time_ns: ((stage * PLATEAU + tick + 1) * 1_000_000) as u128,
            stage,
            command_stroke_m: START + INCREMENT * stage as f64,
            aperture_m: 0.050 - stage as f64 * 1e-5,
            cross_panel_n: force,
            panel_gap_m: 1e-6,
            jaw_up_n: [0.04, 0.04],
            jaw_squeeze_n: [0.6, 0.6],
            bench_up_n: 0.,
            min_bench_clearance_m: 0.15,
            step_ns: tick as u64 + 1,
        })
        .collect()
}

#[test]
fn onset_uses_a_full_terminal_window_and_confirmation_at_the_next_level() {
    let mut trace = samples(2, FORCE);
    for t in &mut trace[..PLATEAU - WINDOW] {
        t.cross_panel_n = 0.;
        t.aperture_m = 0.1;
    }
    let last = plateau(&trace);
    assert!(last.sustained);
    assert_eq!(last.step_p95_ns, 95);
    assert!((last.mean_aperture_m - (0.050 - 2e-5)).abs() < 1e-15);
    trace[PLATEAU - WINDOW].cross_panel_n = 0.;
    assert!(!plateau(&trace).sustained);

    let mut stages = vec![
        plateau(&samples(0, 0.)),
        plateau(&samples(1, FORCE)),
        plateau(&samples(2, 0.)),
    ];
    assert!(bracket(&stages).is_none());
    stages.push(plateau(&samples(3, FORCE)));
    stages.push(plateau(&samples(4, FORCE * 2.)));
    let measured = bracket(&stages).unwrap();
    assert_eq!((measured.open_stage, measured.loaded_stage), (2, 3));
    // A threshold crossed during an unchanged-target dwell has no command bracket.
    stages[3].command_stroke_m = stages[2].command_stroke_m;
    assert!(bracket(&stages).is_none());
    assert!(bracket(&[plateau(&samples(0, FORCE)), plateau(&samples(1, FORCE))]).is_none());
}

#[test]
fn bilateral_weight_support_requires_clearance_load_on_each_jaw_and_no_bench_reaction() {
    let mut t = samples(0, 0.).pop().unwrap();
    assert!(supported(&t, 0.08));
    t.bench_up_n = 0.001;
    assert!(!supported(&t, 0.08));
    t.bench_up_n = 0.;
    t.min_bench_clearance_m = 0.01;
    assert!(!supported(&t, 0.08));
    t.min_bench_clearance_m = 0.15;
    t.jaw_up_n = [0.08, 0.];
    assert!(!supported(&t, 0.08));
    t.jaw_up_n = [0.03, 0.04];
    assert!(!supported(&t, 0.08));
}

#[test]
fn an_authored_trial_exports_the_fixed_target_world_trace_and_native_cloth_poses() -> Result<()> {
    let dir = tempfile::tempdir()?;
    assert!(trial(4, dir.path(), CLOCK, None).is_err());
    let out = trial(3, dir.path(), (8, 24), Some(0))?;
    assert!(out.preceding.is_some() && out.anchor["particles"].as_array().unwrap().len() == 12);
    let dst = dir.path().join("mesh-3-fixed");
    let sdf = fs::read_to_string(dst.join("world.sdf"))?;
    assert_eq!(sdf, closing_sheet_scene::scene(3)?);
    let r = Recording::new(&sdf, "rest.sdf", "default")?;
    assert!((Jaws::new(&r)?.aperture(&r.sim)? - 0.196).abs() < 1e-12);
    let read =
        |name: &str| -> Result<Value> { Ok(serde_json::from_slice(&fs::read(dst.join(name))?)?) };
    assert_eq!(out.report["status"], "fixed_target");
    let trace = read("trace.json")?;
    let t = trace.as_array().unwrap();
    assert_eq!(t.len(), 3 * PLATEAU);
    let k = (APPROACH + 1) * 1_000_000;
    assert_eq!(t[0]["time_ns"], k);
    assert!(t.iter().all(|v| v["command_stroke_m"] == START));
    let poses = read("particle-frames.json")?;
    let pts = poses.as_array().unwrap();
    assert_eq!(read("replay/manifest.json")?["frames"], pts.len());
    assert_eq!(pts.last().unwrap()["time_ns"], t.last().unwrap()["time_ns"]);
    assert_eq!(
        pts.last().unwrap()["positions"].as_array().unwrap().len(),
        12
    );
    Ok(())
}

#[test]
fn observation_requires_live_jaws_and_a_completed_material_reaction() -> Result<()> {
    let xml = closing_sheet_scene::scene(3)?;
    let mut r = Recording::new(&xml, "sensor.sdf", "default")?;
    let jaws = Jaws::new(&r)?;
    assert!(
        sample(&r, &jaws, 3, 0, START)
            .err()
            .unwrap()
            .to_string()
            .contains("two jaws and the bench")
    );
    r.advance()?;
    for side in 0..2 {
        let mut stale = Jaws {
            ids: jaws.ids,
            half: jaws.half,
        };
        stale.ids[side] = u64::MAX;
        assert!(stale.aperture(&r.sim).is_err());
        assert!(sample(&r, &stale, 3, 0, START).is_err());
    }
    let bare = Recording::new(arm::SCENE, "empty_gripper.sdf", "default")?;
    assert!(
        sample(&bare, &jaws, 3, 0, START)
            .err()
            .unwrap()
            .to_string()
            .contains("missing sheet")
    );
    // Match each user's authored collision by name. Missing geometry or handles
    // cannot be replaced with a guessed finger width or opening.
    for side in ["left", "right"] {
        let mut geom = Recording::new(&xml, "geometry.sdf", "default")?;
        let sdf = xml
            .replace(
                &format!("<link name=\"{side}\">"),
                &format!("<link name=\"unmeasured_{side}\">"),
            )
            .replace(
                &format!("<child>{side}</child>"),
                &format!("<child>unmeasured_{side}</child>"),
            );
        geom.sdf = sdformat::load_str(&sdf, "different.sdf", &sdformat::NoResources)?;
        assert!(Jaws::new(&geom).is_err());
        let id = entity_id(&geom.sim, &format!("default::arm::{side}"))?;
        let entity = entity_by_id(geom.sim.world(), id).unwrap();
        geom.sim.world_mut().entity_mut(entity).remove::<Name>();
        assert!(Jaws::new(&geom).is_err());
    }
    Ok(())
}

#[test]
fn partial_world_recording_and_invalid_clocks_do_not_publish_a_study() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let dest = dir.path().join("out");
    assert!(trial(3, &dest, (0, 24), Some(0)).is_err());
    assert!(!dest.exists());
    assert!(primary_and_control(4, &dest, trial).is_err());
    for obstacle in ["replay", "world.sdf", "particle-frames.json", "trace.json"] {
        let err = write_new(&dest, |stage| {
            let case = stage.join("mesh-3-fixed");
            fs::create_dir(&case)?;
            if obstacle == "replay" {
                fs::write(case.join(obstacle), "occupied")?;
            } else {
                fs::create_dir(case.join(obstacle))?;
            }
            trial(3, stage, (8, 24), Some(0)).map(|_| ())
        });
        assert!(err.is_err(), "{obstacle}");
        assert!(!dest.exists());
        assert_eq!(fs::read_dir(dir.path())?.count(), 0);
    }
    Ok(())
}

#[test]
fn onset_does_not_credit_a_loaded_start_or_a_contact_that_never_persists() {
    let loaded = || plateau(&samples(0, FORCE));
    let free = || plateau(&samples(0, 0.));
    assert_eq!(
        onset_status(&[loaded(), loaded()], false, false),
        "already_loaded"
    );
    assert_eq!(
        onset_status(&[loaded(), free()], false, false),
        "unbracketed"
    );
    assert_eq!(onset_status(&[free(), free()], false, false), "unbracketed");
    let mut window = samples(7, 0.);
    window[10].bench_up_n = 0.08;
    let e = check_support(&window, 0.08, 5).unwrap_err().to_string();
    assert!(e.contains("plateau 7") && e.contains(&window[10].time_ns.to_string()));
}

fn fixture(columns: usize, clock: (u32, u32), hold: Option<usize>) -> Outcome {
    Outcome {
        anchor: json!({"positions":[0.,0.1],"columns":columns}),
        preceding: Some(json!({"aperture_m":0.043,"pending_jaw_ns":[0.00004,-0.00004]})),
        lower: Some(4),
        loaded_at_fixed_target: false,
        report: json!({"columns":columns,"substeps":clock.0,"hold_after":hold,"status":if hold.is_some() {"fixed_target"}else{"bracketed"}}),
    }
}

#[test]
fn matched_comparison_reports_delayed_loading_and_skips_controls_without_a_bracket() -> Result<()> {
    let dir = tempfile::tempdir()?;
    run_comparison(dir.path(), |c, _, clock, hold| {
        let mut out = fixture(c, clock, hold);
        if c == 5 && hold.is_some() {
            out.loaded_at_fixed_target = true;
        }
        if c == 7 {
            out.lower = None;
            out.preceding = None;
            out.report["status"] = "already_loaded".into();
        }
        Ok(out)
    })?;
    let v: Value = serde_json::from_slice(&fs::read(dir.path().join("comparison.json"))?)?;
    assert_eq!(v["trials"][0]["status"], "also_loaded_at_fixed_target");
    assert_eq!(v["trials"][1]["status"], "already_loaded");
    assert_eq!(v["reference_trials"][0]["substeps"], REFERENCE.0);
    assert_eq!(v["fixed_target_controls"].as_array().unwrap().len(), 1);
    assert_eq!(v["fixed_target_controls"][0]["hold_after"], 4);
    Ok(())
}

#[test]
fn incomplete_or_unmatched_workers_cannot_publish_closure_results() -> Result<()> {
    let cases: [(&str, Simulate); 6] = [
        ("fixed-target control", |c, _, clock, hold| {
            let mut v = fixture(c, clock, hold);
            if hold.is_some() {
                v.preceding.as_mut().unwrap()["pending_jaw_ns"][0] = 0.0.into();
            }
            Ok(v)
        }),
        ("fixed-target control", |c, _, clock, hold| {
            let mut v = fixture(c, clock, hold);
            v.preceding = None;
            Ok(v)
        }),
        ("clock comparison", |c, _, clock, hold| {
            let mut v = fixture(c, clock, hold);
            if clock == REFERENCE {
                v.anchor["positions"][0] = 0.001.into();
            }
            Ok(v)
        }),
        ("fixed run refused", |c, _, clock, hold| {
            if hold.is_some() {
                Err("fixed run refused".into())
            } else {
                Ok(fixture(c, clock, hold))
            }
        }),
        ("reference sweep refused", |c, _, clock, hold| {
            if clock == REFERENCE {
                Err("reference sweep refused".into())
            } else {
                Ok(fixture(c, clock, hold))
            }
        }),
        ("closure trial panicked", |_, _, _, _| {
            panic!("simulated worker failure")
        }),
    ];
    let dir = tempfile::tempdir()?;
    let out = dir.path().join("result");
    for (reason, simulate) in cases {
        let e = write_new(&out, |stage| run_comparison(stage, simulate))
            .unwrap_err()
            .to_string();
        assert!(e.contains(reason), "{e}");
        assert!(!out.exists() && fs::read_dir(dir.path())?.count() == 0);
    }
    Ok(())
}
