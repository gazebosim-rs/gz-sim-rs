// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::json;

fn fixture() -> Value {
    let mut trials = vec![];
    for columns in COLUMNS.into_iter().rev() {
        for model in MODELS.into_iter().rev() {
            trials.push(
                json!({"columns":columns,"model":model, "status":"completed","completed_ticks":300,
                "ticks_with_cross_panel_impulse":230,"mean_cross_panel_n":0.018,"mean_closest_panel_skin_gap_m":-3e-9,
                "mean_jaw_up_n":[0.020,0.062],"step_p95_ns":22_000_000}),
            );
        }
    }
    json!({"trials":trials,"patch_width_m":0.08,"patch_height_m":0.04,"sites":[{"panel":0,"width_m":-0.02,"height_m":0.01},{"panel":1,"width_m":0.03,"height_m":0.03}]})
}
#[test]
fn units_signs_and_partial_runs_are_visible() -> Result<()> {
    let mut data = fixture();
    data["trials"][0]["status"] = json!("refused");
    data["trials"][0]["completed_ticks"] = json!(10);
    let p = pairs(&data)?;
    assert!(matches!(p[2][1], Outcome::Refused));
    let Outcome::Held {
        values: v,
        loaded: 230,
    } = p[0][0]
    else {
        panic!()
    };
    assert_eq!(v, [18., -0.003, 82., 22.]);
    let svg = draw(&data)?;
    for s in [
        "Left cloth panel",
        "Right cloth panel",
        "80 × 40 mm",
        "3 points",
        "7 points",
        "stopped",
        "18.00",
        "22.00",
        "82.00",
        "-3e-3",
    ] {
        assert!(svg.contains(s), "{s}");
    }
    Ok(())
}
#[test]
fn rejects_incomplete_duplicate_and_inconsistent_measurements() {
    let mut d = fixture();
    d["trials"].as_array_mut().unwrap().pop();
    assert!(pairs(&d).is_err());
    let duplicate = d["trials"][0].clone();
    d["trials"].as_array_mut().unwrap().push(duplicate);
    assert!(pairs(&d).is_err());
    for (field, value) in [
        ("columns", json!(8)),
        ("model", json!("other")),
        ("completed_ticks", Value::Null),
        ("completed_ticks", json!(12)),
        ("mean_cross_panel_n", Value::Null),
        ("mean_closest_panel_skin_gap_m", Value::Null),
        ("mean_jaw_up_n", json!([0.01])),
        ("mean_jaw_up_n", Value::Null),
        ("step_p95_ns", Value::Null),
        ("mean_cross_panel_n", json!(-1.)),
        ("mean_cross_panel_n", json!(0.)),
        ("ticks_with_cross_panel_impulse", json!(400)),
        ("ticks_with_cross_panel_impulse", Value::Null),
        ("step_p95_ns", json!(0)),
    ] {
        let mut d = fixture();
        d["trials"][0][field] = value;
        assert!(pairs(&d).is_err(), "{field}");
    }
    assert!(pairs(&Value::Null).is_err());
}

#[test]
fn validates_contact_footprints_before_plotting() {
    for (path, value) in [
        ("/patch_width_m", Value::Null),
        ("/patch_height_m", Value::Null),
        ("/patch_width_m", json!(0)),
        ("/sites", Value::Null),
        ("/sites/0/panel", Value::Null),
        ("/sites/0/width_m", Value::Null),
        ("/sites/0/height_m", Value::Null),
        ("/sites/0/panel", json!(2)),
        ("/sites/1/panel", json!(0)),
        ("/sites/0/width_m", json!(-0.041)),
        ("/sites/1/height_m", json!(0.041)),
    ] {
        let mut d = fixture();
        *d.pointer_mut(path).unwrap() = value;
        assert!(draw(&d).is_err(), "{path}");
    }
}

#[test]
fn only_complete_input_can_create_an_output_and_an_existing_figure_is_preserved() -> Result<()> {
    let dir = std::env::temp_dir().join(format!("fold-patch-chart-{}", std::process::id()));
    fs::create_dir(&dir)?;
    let source = dir.join("study.json");
    let figure = dir.join("forces.svg");
    assert!(render(&source, &figure).is_err());
    fs::write(&source, b"[")?;
    assert!(render(&source, &figure).is_err());
    fs::write(&source, b"{}")?;
    assert!(render(&source, &figure).is_err());
    assert!(!figure.exists());
    fs::write(&source, serde_json::to_vec(&fixture())?)?;
    render(&source, &figure)?;
    let previous = fs::read(&figure)?;
    assert!(render(&source, &figure).is_err());
    assert!(previous.starts_with(b"<svg") && fs::read(&figure)? == previous);
    fs::remove_dir_all(&dir)?;
    Ok(())
}
