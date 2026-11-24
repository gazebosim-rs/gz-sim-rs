// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::json;

fn fixture() -> Value {
    let rows = json!([{"command_stroke_m":0.07,"mean_aperture_m":0.04301,"mean_cross_panel_n":0.0002,"min_cross_panel_n":0.,"sustained":false},
        {"command_stroke_m":0.0701,"mean_aperture_m":0.043,"mean_cross_panel_n":0.002,"min_cross_panel_n":0.0001,"sustained":true},
        {"command_stroke_m":0.0702,"mean_aperture_m":0.04299,"mean_cross_panel_n":0.003,"min_cross_panel_n":0.0002,"sustained":true}]);
    let trials = |substeps, iterations| json!([{"columns":7,"material_substeps":substeps,"solver_iterations":iterations,"stages":rows},{"columns":5,"material_substeps":substeps,"solver_iterations":iterations,"stages":rows}]);
    let mut controls = trials(32, 24);
    for c in controls.as_array_mut().unwrap() {
        for s in c["stages"].as_array_mut().unwrap() {
            s["command_stroke_m"] = json!(0.07);
        }
    }
    json!({"dt_ns":1_000_000,"window_ticks":20,"plateau_ticks":100,"force_floor_n":0.0001,
      "trials":trials(32,24),"reference_trials":trials(64,48),"fixed_target_controls":controls})
}

#[test]
fn transient_force_does_not_qualify_a_plateau_and_bounds_use_actual_aperture() -> Result<()> {
    let mut d = fixture();
    let mesh = stages(&d["trials"][0], 0.0001)?;
    assert_eq!(mesh[0].force_mn, 0.2);
    assert_eq!(onset(&mesh), "43.010000 to 43.000000 mm");
    d["trials"][0]["stages"].as_array_mut().unwrap().pop();
    d["reference_trials"][1]["stages"][0]["command_stroke_m"] = json!(0.0701);
    d["fixed_target_controls"].as_array_mut().unwrap().remove(0);
    let mut svg = draw(&d)?;
    assert!(svg.contains("unchanged setting"));
    for s in [
        "No confirmed load",
        "0.100 mN",
        "20 ms",
        "43.010000 to 43.000000",
        "Mean force between layers",
    ] {
        assert!(svg.contains(s), "{s}");
    }
    d["trials"][1]["stages"].as_array_mut().unwrap().remove(0);
    d["fixed_target_controls"] = json!([]);
    svg = draw(&d)?;
    assert!(svg.contains("no bracket (5 points)"));
    assert!(svg.contains("Loaded at first opening (43.000000 mm)"));
    Ok(())
}

#[test]
fn refuses_incomplete_or_inconsistent_study_records() {
    for (path, value) in [
        ("/force_floor_n", Value::Null),
        ("/dt_ns", Value::Null),
        ("/window_ticks", Value::Null),
        ("/force_floor_n", json!(0)),
        ("/dt_ns", json!(0)),
        ("/window_ticks", json!(0)),
        ("/trials", Value::Null),
        ("/trials", json!([])),
        ("/reference_trials", Value::Null),
        ("/fixed_target_controls", Value::Null),
        ("/fixed_target_controls", json!([])),
        ("/fixed_target_controls/0/columns", json!(4)),
        (
            "/fixed_target_controls/0/stages/0/mean_aperture_m",
            Value::Null,
        ),
        ("/fixed_target_controls/0/columns", json!(5)),
        (
            "/fixed_target_controls/0/stages",
            json!([fixture()["trials"][0]["stages"][0]]),
        ),
        (
            "/fixed_target_controls/0/stages/1/command_stroke_m",
            json!(0.71),
        ),
        (
            "/fixed_target_controls/0/stages/2/command_stroke_m",
            json!(0.71),
        ),
        ("/trials/0/stages/0/command_stroke_m", json!(0.62)),
        ("/plateau_ticks", json!(15)),
        ("/trials/0/material_substeps", Value::Null),
        ("/trials/1/solver_iterations", Value::Null),
        ("/trials/0/solver_iterations", json!(0)),
        ("/trials/0/material_substeps", json!(0)),
        ("/trials/1/material_substeps", json!(18)),
        ("/trials/0/columns", json!(5)),
        ("/trials/0/columns", json!(4)),
        ("/trials/0/stages", json!([])),
        ("/trials/0/stages/0/command_stroke_m", Value::Null),
        ("/trials/0/stages/0/mean_aperture_m", Value::Null),
        ("/trials/0/stages/0/mean_cross_panel_n", Value::Null),
        ("/trials/0/stages/0/min_cross_panel_n", Value::Null),
        ("/trials/0/stages/0/sustained", Value::Null),
        ("/trials/0/stages/0/mean_aperture_m", json!(0)),
        ("/trials/0/stages/0/min_cross_panel_n", json!(-1.)),
        ("/trials/0/stages/0/min_cross_panel_n", json!(0.1)),
        ("/trials/0/stages/0/sustained", json!(true)),
    ] {
        let mut d = fixture();
        *d.pointer_mut(path).unwrap() = value;
        assert!(draw(&d).is_err(), "{path}");
    }
}

#[test]
fn render_preserves_an_existing_result_and_rejects_invalid_input() -> Result<()> {
    let dir = std::env::temp_dir().join(format!("gz-fold-closure-chart-{}", std::process::id()));
    fs::create_dir(&dir)?;
    let input = dir.join("report.json");
    let output = dir.join("chart.svg");
    assert!(render(&input, &output).is_err());
    fs::write(&input, b"[")?;
    assert!(render(&input, &output).is_err());
    fs::write(&input, b"{}")?;
    assert!(render(&input, &output).is_err() && !output.exists());
    fs::write(&input, serde_json::to_vec(&fixture())?)?;
    render(&input, &output)?;
    let prev = fs::read(&output)?;
    assert!(render(&input, &output).is_err() && prev == fs::read(&output)?);
    fs::remove_dir_all(dir)?;
    Ok(())
}
