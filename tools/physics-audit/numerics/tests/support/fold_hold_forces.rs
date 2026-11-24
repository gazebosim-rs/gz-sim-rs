// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::json;

#[test]
fn compares_each_panels_reaction_with_the_time_integral() {
    let mut trial = json!({
        "samples":[
            {"time_ns":101, "self_contact_n":0.5, "cross_panel_n":0.2},
            {"time_ns":102, "self_contact_n":0.7, "cross_panel_n":0.4}
        ],
        "mean_self_contact_n":0.6, "mean_cross_panel_n":0.3,
        "mean_cross_panel_node_normal_n":[0.1,0.0,0.0,0.0,0.2,0.0,0.0,0.3,0.0,0.0,0.0,0.0]
    });
    let load = read_loads(&trial, 100, 1, 2).unwrap();
    for (actual, expected) in load.width.into_iter().zip([0.05, 0.25, 0.0]) {
        assert!((actual - expected).abs() < 1e-14);
    }
    assert!((load.history[0].1 - 300.).abs() < 1e-12);
    trial["mean_cross_panel_node_normal_n"][0] = json!(0.05);
    assert!(read_loads(&trial, 100, 1, 2).is_err());
    trial["samples"][0]["cross_panel_n"] = json!(0.6);
    assert!(read_loads(&trial, 100, 1, 2).is_err());
    trial["samples"][0]["time_ns"] = json!(102);
    assert!(read_loads(&trial, 100, 1, 2).is_err());
    assert!(read_loads(&trial, 100, 1, 0).is_err());
}

#[test]
fn reader_rejects_partial_archives_and_off_clock_or_inconsistent_measurements() -> Result<()> {
    struct Scratch(std::path::PathBuf);
    impl Drop for Scratch {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }
    let scratch =
        Scratch(std::env::temp_dir().join(format!("gz-fold-reader-{}", std::process::id())));
    let root = &scratch.0;
    fs::create_dir(root)?;
    let write = |relative: &str, bytes: &[u8]| fs::write(root.join(relative), bytes);
    let checked = |message: &str| {
        let error = verify_prefix(root, 1).unwrap_err().to_string();
        assert!(error.contains(message), "{error} vs {message}");
    };
    let observation = json!([
        {"time_ns":0,"positions":[[0.,0.,0.]]},
        {"time_ns":1,"positions":[[0.,0.,1.]]}
    ]);
    let frame = |ns| json!({"scene":json!({"seconds":0,"nanoseconds":ns}).to_string()}).to_string();
    let header = "{\"kind\":\"header\"}";
    let stream = format!("{header}\n{}\n{}\n", frame(0), frame(1));
    for branch in ["coarse", "fine"] {
        fs::create_dir_all(root.join(branch).join("replay"))?;
        write(
            &format!("{branch}/particle-frames.json"),
            observation.to_string().as_bytes(),
        )?;
        write(&format!("{branch}/replay/frames.jsonl"), stream.as_bytes())?;
    }
    assert_eq!(verify_prefix(root, 1)?, 2);
    // Both independent observation files and both wire streams are required.
    for branch in ["coarse", "fine"] {
        let obs = format!("{branch}/particle-frames.json");
        let frames = format!("{branch}/replay/frames.jsonl");
        fs::remove_file(root.join(&obs))?;
        assert!(verify_prefix(root, 1).is_err());
        write(&obs, b"{")?;
        assert!(verify_prefix(root, 1).is_err());
        write(&obs, observation.to_string().as_bytes())?;
        fs::remove_file(root.join(&frames))?;
        assert!(verify_prefix(root, 1).is_err());
        write(&frames, b"not JSON\n")?;
        assert!(verify_prefix(root, 1).is_err());
        write(&frames, &[255, b'\n'])?;
        assert!(verify_prefix(root, 1).is_err());
        let mut invalid = format!("{header}\n").into_bytes();
        invalid.extend_from_slice(&[255, b'\n']);
        write(&frames, &invalid)?;
        assert!(verify_prefix(root, 1).is_err());
        write(&frames, stream.as_bytes())?;
    }
    for packet in [
        String::from("{"),
        json!({}).to_string(),
        json!({"scene":"{"}).to_string(),
        json!({"scene":json!({"nanoseconds":0}).to_string()}).to_string(),
        json!({"scene":json!({"seconds":0}).to_string()}).to_string(),
    ] {
        let bad = format!("{header}\n{packet}\n");
        for branch in ["coarse", "fine"] {
            write(&format!("{branch}/replay/frames.jsonl"), bad.as_bytes())?;
        }
        assert!(verify_prefix(root, 1).is_err());
    }
    for branch in ["coarse", "fine"] {
        write(&format!("{branch}/replay/frames.jsonl"), stream.as_bytes())?;
    }

    for (bytes, message) in [
        (String::new(), "empty replay"),
        ("{}".to_owned(), "missing replay header"),
        (header.to_owned(), "incomplete fine"),
        (format!("{header}\n{}\n", frame(2)), "publications differ"),
    ] {
        write("fine/replay/frames.jsonl", bytes.as_bytes())?;
        checked(message);
    }
    write("fine/replay/frames.jsonl", stream.as_bytes())?;
    write("coarse/replay/frames.jsonl", header.as_bytes())?;
    checked("incomplete coarse");
    let late = format!("{header}\n{}\n{}\n", frame(0), frame(2));
    for branch in ["coarse", "fine"] {
        write(&format!("{branch}/replay/frames.jsonl"), late.as_bytes())?;
    }
    checked("clocks differ");
    write("fine/particle-frames.json", b"[{\"time_ns\":2}]")?;
    checked("material prefix differs");
    write("coarse/particle-frames.json", b"[{\"time_ns\":2}]")?;
    checked("no matched replay frame");
    for branch in ["coarse", "fine"] {
        write(
            &format!("{branch}/particle-frames.json"),
            observation.to_string().as_bytes(),
        )?;
        write(&format!("{branch}/replay/frames.jsonl"), stream.as_bytes())?;
    }
    let full_trial = json!({"substeps":8,"iterations":24,"samples":[{"time_ns":2,"cross_panel_n":0.,"self_contact_n":0.}],
        "mean_cross_panel_n":0.,"mean_self_contact_n":0.,"mean_cross_panel_node_normal_n":[]});
    let mut trial = full_trial;
    trial["mean_cross_panel_node_normal_n"] =
        json!([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.]);
    let mut report = json!({"matched_time_ns":1,"dt_ns":1,"hold_ticks":1,"reference":{"substeps":64,"iterations":48},"trials":[trial]});
    let output = root.join("figure.svg");
    let error = |report: &Value| {
        write("comparison.json", report.to_string().as_bytes()).unwrap();
        run(root, &output).unwrap_err().to_string()
    };
    assert!(run(root, &output).is_err());
    write("comparison.json", b"{")?;
    assert!(run(root, &output).is_err());
    assert!(error(&json!({})).contains("integer"));
    assert!(error(&json!({"matched_time_ns":1})).contains("integer"));
    assert!(error(&json!({"matched_time_ns":1,"dt_ns":1})).contains("integer"));
    assert!(
        error(&json!({"matched_time_ns":1,"dt_ns":1,"hold_ticks":1})).contains("measured holds")
    );
    for (field, bad) in [
        ("substeps", Value::Null),
        ("iterations", Value::Null),
        ("samples", Value::Null),
        ("mean_self_contact_n", json!(0.01)),
        ("mean_cross_panel_n", json!(0.01)),
        ("mean_cross_panel_node_normal_n", json!([])),
    ] {
        let mut invalid = report.clone();
        invalid["trials"][0][field] = bad;
        assert!(!error(&invalid).is_empty());
    }
    for field in ["cross_panel_n", "self_contact_n", "time_ns"] {
        let mut invalid = report.clone();
        invalid["trials"][0]["samples"][0][field] = json!(-1);
        assert!(!error(&invalid).is_empty());
    }
    report["trials"].as_array_mut().unwrap().push(trial.clone());
    assert!(error(&report).contains("duplicate solver setting"));
    report["trials"][1]["substeps"] = json!(64);
    report["trials"][1]["iterations"] = json!(48);
    write("comparison.json", report.to_string().as_bytes())?;
    assert_eq!(
        run(root, &output)
            .unwrap_err()
            .downcast_ref::<std::io::Error>()
            .unwrap()
            .kind(),
        std::io::ErrorKind::NotFound
    );
    fs::create_dir(root.join("mesh"))?;
    write("mesh/comparison.json", b"{}")?;
    assert!(error(&report).contains("integer"));
    write("mesh/comparison.json", br#"{"ticks":1,"trials":[]}"#)?;
    assert!(error(&report).contains("mesh study has no trials"));
    let mut no_coarse = report.clone();
    no_coarse["trials"].as_array_mut().unwrap().remove(0);
    assert!(error(&no_coarse).contains("missing coarse hold"));
    let mut no_ref = report.clone();
    no_ref["trials"].as_array_mut().unwrap().pop();
    assert!(error(&no_ref).contains("missing reference hold"));
    let mut invalid = report.clone();
    invalid["reference"]["iterations"] = Value::Null;
    assert!(error(&invalid).contains("integer"));
    write("mesh/comparison.json", b"{")?;
    assert!(!error(&report).is_empty());
    write(
        "mesh/comparison.json",
        br#"{"ticks":1,"trials":[{"spacing_m":0.01,"status":"refused","completed_ticks":0}]}"#,
    )?;
    write("comparison.json", report.to_string().as_bytes())?;
    fs::write(&output, b"keep")?;
    assert_eq!(
        run(root, &output)
            .unwrap_err()
            .downcast_ref::<std::io::Error>()
            .unwrap()
            .kind(),
        std::io::ErrorKind::AlreadyExists
    );
    assert_eq!(fs::read(&output)?, b"keep");
    Ok(())
}
