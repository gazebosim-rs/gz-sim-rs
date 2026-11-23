// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::json;

fn row(i: usize, force: f64) -> Value {
    json!({"time_ns":1_000_000*i,"force_y_n":force,"jaw_frame_y_m":0.1+0.00025*i as f64,"cross_panel_n":0.002})
}

#[test]
fn displacement_is_measured_from_the_last_unloaded_sample() -> Result<()> {
    let trace = json!([row(0, 0.), row(1, 0.), row(2, 1.), row(3, 0.)]);
    let p = samples(&trace, 1_000_000)?;
    assert_eq!(p.len(), 3);
    assert_eq!(p[0].slip_mm, 0.);
    assert!((p[1].slip_mm - 0.25).abs() < 1e-12);
    assert_eq!(p[2].layer_mn, 2.);
    let mut control = p.clone();
    control[1].sideways = 0.;
    let pair = Pair::new(5, p.clone(), control.clone(), 1_000_000)?;
    assert_eq!(pair.last_loaded_ms, 1.);
    assert!(Pair::new(5, p.clone(), p.clone(), 1_000_000).is_err());
    assert!(Pair::new(5, p.clone(), control[..2].to_vec(), 1_000_000).is_err());
    assert!(Pair::new(5, control.clone(), control.clone(), 1_000_000).is_err());
    assert!(Pair::new(5, p[..2].to_vec(), control[..2].to_vec(), 1_000_000).is_err());
    assert!(Pair::new(5, p.clone(), control.clone(), 2_000_000).is_err());
    control[2].ns += 1;
    assert!(Pair::new(5, p, control, 1_000_000).is_err());
    Ok(())
}

#[test]
fn charts_require_a_trace_at_the_true_grip_baseline() {
    let good = json!([row(1, 0.), row(2, 1.), row(3, 0.)]);
    for (trace, t) in [
        (Value::Null, 0),
        (good.clone(), 0),
        (good.clone(), 2_000_000),
        (good.clone(), 4_000_000),
    ] {
        assert!(samples(&trace, t).is_err());
    }
    for k in ["time_ns", "force_y_n", "jaw_frame_y_m", "cross_panel_n"] {
        let mut d = good.clone();
        d[0][k] = Value::Null;
        assert!(samples(&d, 1_000_000).is_err());
    }
}

#[test]
fn original_force_and_both_controls_are_required_before_a_chart_is_written() -> Result<()> {
    let root = std::env::temp_dir().join(format!("fold-pull-figure-{}", std::process::id()));
    fs::create_dir(&root)?;
    let output = root.join("result.svg");
    let report = root.join("comparison.json");
    let run = || render(&root, &output);
    assert!(run().is_err());
    fs::write(&report, "{")?;
    assert!(run().is_err());
    let good = json!({"target_n":0.002,"dt_ns":1_000_000,"trials":[{"baseline_time_ns":1_000_000,"columns":7},{"baseline_time_ns":1_000_000,"columns":5}]});
    for (key, value) in [
        ("dt_ns", json!(0)),
        ("target_n", Value::Null),
        ("trials", Value::Null),
        ("trials", json!([])),
        ("trials", json!([{"columns":5}])),
    ] {
        let mut broken = good.clone();
        broken[key] = value;
        fs::write(&report, broken.to_string())?;
        assert!(run().is_err());
    }
    fs::write(&report, good.to_string())?;
    assert!(run().is_err());
    for c in [5, 7] {
        for (role, load) in [("pull", 0.5), ("sham", 0.)] {
            let d = root.join(format!("mesh-{c}-{role}"));
            fs::create_dir(&d)?;
            let t: Vec<_> = (0..23)
                .map(|i| row(i, if (2..=12).contains(&i) { load } else { 0. }))
                .collect();
            fs::write(d.join("trace.json"), json!(t).to_string())?;
        }
    }
    run()?;
    let svg = fs::read_to_string(&output)?;
    for text in [
        "5 points across",
        "7 points across",
        "2.00 mN",
        "No-pull control",
        "Mean layer force",
    ] {
        assert!(svg.contains(text), "{text}");
    }
    assert!(run().is_err());
    assert_eq!(svg, fs::read_to_string(&output)?);
    fs::remove_dir_all(&root)?;
    Ok(())
}

#[test]
fn plots_select_units_that_resolve_the_observed_motion() -> Result<()> {
    for (slip_mm, label) in [(0.00004, "(nm)"), (0.04, "(µm)"), (4., "(mm)")] {
        let mut samples = vec![
            Sample {
                ns: 0,
                sideways: 0.,
                slip_mm: 0.,
                layer_mn: 2.
            };
            3
        ];
        samples[1].ns = 1_000_000;
        samples[2].ns = 2_000_000;
        let sham = samples.clone();
        samples[1].sideways = 0.3;
        samples[1].slip_mm = slip_mm;
        let pair = Pair::new(5, samples, sham, 1_000_000)?;
        assert!(draw(&[pair], 2.)?.contains(&format!("Sheet displacement {label}")));
    }
    Ok(())
}
