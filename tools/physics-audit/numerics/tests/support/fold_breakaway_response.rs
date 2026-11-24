// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::json;

fn row(load: u64, left: f64, right: f64, n: u64) -> Value {
    json!({"load_mn":load,"final_min_pad_m":[left,right],"lasting_slip":left.max(right)>0.00025,"recovered_support_ticks":n})
}
fn trial() -> Value {
    json!({"cases":[row(800,0.00040,0.00002,12),row(0,0.,0.,20),row(700,0.00025,-0.00002,20)],"bracket_mn":[700,800]})
}
fn report() -> Value {
    let a = trial();
    let mut fine = a.clone();
    fine["bracket_mn"] = Value::Null;
    fine["cases"][0] = row(800, 0.00023, -0.0001, 20);
    json!({"slip_m":0.00025,"dt_ns":1000000,"window_ticks":20,"trials":[{"columns":7,"primary":a,"fine":fine},{"columns":5,"primary":a,"fine":a}]})
}
#[test]
fn verifies_both_fingers_and_strict_threshold_even_if_support_is_lost() -> Result<()> {
    let result = sweep(&trial(), 0.00025, 20)?;
    assert_eq!(result.bracket, Some((0.7, 0.8)));
    assert!(!result.points[2].supported);
    assert_eq!(result.points[1].travel_mm, 0.25);
    let mut two = trial();
    two["cases"][0] = row(800, -0.01, 0.00031, 20);
    assert_eq!(sweep(&two, 0.00025, 20)?.points[2].travel_mm, 0.31);
    let mut alternate = two.clone();
    alternate["bracket_mn"] = Value::Null;
    assert!(sweep(&alternate, 0.00025, 20).is_err());
    alternate["cases"][0] = row(800, -0.01, 0.0001, 20);
    assert!(sweep(&alternate, 0.00025, 20)?.bracket.is_none());
    Ok(())
}
#[test]
fn inconsistent_report_cannot_bracket_a_slip() {
    for (key, val) in [
        ("load_mn", json!(-1)),
        ("final_min_pad_m", json!([0.001])),
        ("final_min_pad_m", json!([null, 0.])),
        ("final_min_pad_m", json!([0., null])),
        ("lasting_slip", Value::Null),
        ("lasting_slip", json!(false)),
        ("recovered_support_ticks", json!(21)),
    ] {
        let mut v = trial();
        v["cases"][0][key] = val;
        assert!(sweep(&v, 0.00025, 20).is_err());
    }
    for (key, val) in [
        ("cases", Value::Null),
        ("cases", json!([])),
        ("bracket_mn", json!(800)),
        ("bracket_mn", json!([0, 800])),
        ("bracket_mn", json!([700, null])),
        ("bracket_mn", json!([null, 800])),
    ] {
        let mut v = trial();
        v[key] = val;
        assert!(sweep(&v, 0.00025, 20).is_err());
    }
    for (index, bad) in [
        (0, row(700, 0.0004, 0.001, 20)),
        (1, row(0, 0.0001, 0., 20)),
        (1, row(0, -0.0001, 0., 20)),
        (1, row(100, 0., 0., 20)),
    ] {
        let mut v = trial();
        v["cases"][index] = bad;
        assert!(sweep(&v, 0.00025, 20).is_err());
    }
}
#[test]
fn draws_measured_intervals_and_preserves_output_on_bad_inputs() -> Result<()> {
    let dir = std::env::temp_dir().join(format!("gz-breakaway-chart-{}", std::process::id()));
    fs::create_dir(&dir)?;
    let input = dir.join("study.json");
    let out = dir.join("chart.svg");
    let result = (|| -> Result<()> {
        assert!(render(&input, &out).is_err());
        fs::write(&input, "{")?;
        assert!(render(&input, &out).is_err());
        let v = report();
        for (key, val) in [
            ("slip_m", Value::Null),
            ("slip_m", json!(-0.00025)),
            ("dt_ns", json!(0)),
            ("window_ticks", json!(0)),
            ("trials", json!([])),
        ] {
            let mut bad = v.clone();
            bad[key] = val;
            assert!(draw(&bad).is_err());
        }
        for key in ["columns", "primary", "fine"] {
            let mut bad = v.clone();
            bad["trials"][0][key] = Value::Null;
            fs::write(&input, bad.to_string())?;
            assert!(render(&input, &out).is_err());
        }
        assert!(!out.exists());
        fs::write(&input, v.to_string())?;
        render(&input, &out)?;
        let svg = fs::read_to_string(&out)?;
        for s in [
            "5 points across",
            "7 points across",
            "0.700 to 0.800 N",
            "onset unbracketed",
            "0.25 mm",
        ] {
            assert!(svg.contains(s));
        }
        assert!(render(&input, &out).is_err());
        assert_eq!(fs::read_to_string(&out)?, svg);
        Ok(())
    })();
    fs::remove_dir_all(dir)?;
    result
}
