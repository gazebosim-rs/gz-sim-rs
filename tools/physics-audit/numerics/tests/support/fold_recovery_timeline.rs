// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use serde_json::json;
fn trace() -> Value {
    json!((0..=30).map(|i|{
        let (a,b,bench)=match i {0..=10=>(4.9,4.91,0.),11..=20=>(0.,1.,3.),_=>(0.1,0.,8.)};
        json!({"time_ns":i*20_000_000,"mass_kg":1.,"jaw_up_n":[a,b],"bench_up_n":bench,"min_bench_clearance_m":if i>20{0.}else{0.05}})
    }).collect::<Vec<_>>())
}
fn case() -> Value {
    let control = json!({"observed_postgrip_ticks":30,"escaped_after_tick":null,"tighten_after_tick":null,"lower_after_tick":null,"release_after_tick":null,"patch_motion_at_400ms_m":[0.0001,0.002],"final_bilateral_ticks":0,"final_bench_supported_ticks":10});
    let mut guard = control.clone();
    guard["tighten_after_tick"] = json!(4);
    guard["lower_after_tick"] = json!(8);
    guard["release_after_tick"] = json!(28);
    json!({"baseline_ns":0,"columns":7,"peak_mn":1250,"guard":guard,"control":control})
}
#[test]
fn follows_recorded_support_and_labels_tactile_actions() -> Result<()> {
    let t = trace();
    let c = case();
    let g = make_row(&t, &c, "guard", 20_000_000, 30)?;
    assert!(
        g.states[9] == Support::Fingers
            && g.states[19] == Support::Other
            && g.states[29] == Support::Bench
    );
    assert_eq!(g.events, vec![(4, "T"), (8, "L"), (28, "R")]);
    assert_eq!(motion(0.000036), "36.0 nm");
    assert_eq!(motion(0.3), "300.0 µm");
    assert_eq!(motion(34.), "34.00 mm");
    let figure = draw(
        &[g, make_row(&t, &c, "control", 20_000_000, 30)?],
        20_000_000,
        30,
    )?;
    for word in [
        "7 points",
        "1.25 N",
        "Guard",
        "Control",
        "2.00 mm",
        "Bench carries",
    ] {
        assert!(figure.contains(word));
    }
    Ok(())
}
#[test]
fn rejects_missing_physical_samples_or_actions_not_supported_by_the_run() {
    let good = trace();
    let c = case();
    for key in [
        "time_ns",
        "bench_up_n",
        "mass_kg",
        "min_bench_clearance_m",
        "jaw_up_n",
    ] {
        let mut t = good.clone();
        t[12][key] = Value::Null;
        assert!(make_row(&t, &c, "guard", 20_000_000, 30).is_err());
    }
    let mut t = good.clone();
    t[1]["mass_kg"] = json!(0.);
    assert!(make_row(&t, &c, "guard", 20_000_000, 30).is_err());
    t = good.clone();
    t[15]["time_ns"] = json!(16_000);
    assert!(make_row(&t, &c, "guard", 20_000_000, 30).is_err());
    assert!(make_row(&json!(null), &c, "guard", 20_000_000, 30).is_err());
    assert!(make_row(&good, &c, "guard", 20_000_000, 31).is_err());
    for (k, v) in [
        ("final_bilateral_ticks", json!(9)),
        ("final_bench_supported_ticks", Value::Null),
        ("tighten_after_tick", json!(0)),
        ("lower_after_tick", json!(3)),
        ("release_after_tick", json!(31)),
        ("patch_motion_at_400ms_m", json!([null, 0.])),
        ("patch_motion_at_400ms_m", json!([0., null])),
    ] {
        let mut c = case();
        c["guard"][k] = v;
        assert!(make_row(&good, &c, "guard", 20_000_000, 30).is_err());
    }
    let mut c = case();
    c["control"]["tighten_after_tick"] = json!(3);
    assert!(make_row(&good, &c, "control", 20_000_000, 30).is_err());
    c["guard"]
        .as_object_mut()
        .unwrap()
        .remove("lower_after_tick");
    assert!(make_row(&good, &c, "guard", 20_000_000, 30).is_err());
}
#[test]
fn both_branch_traces_are_required_and_existing_figures_are_preserved() -> Result<()> {
    let root = std::env::temp_dir().join(format!("gz-fold-retention-{}", std::process::id()));
    fs::create_dir(&root)?;
    let r = (|| -> Result<()> {
        let file = root.join("comparison.json");
        let out = root.join("figure.svg");
        let run = || render(&root, &out);
        assert!(run().is_err());
        fs::write(&file, "[")?;
        assert!(run().is_err());
        let study = json!({"dt_ns":20_000_000,"horizon_ticks":30,"cases":[case()]});
        for (k, v) in [
            ("dt_ns", json!(0)),
            ("horizon_ticks", Value::Null),
            ("cases", json!([])),
        ] {
            let mut bad = study.clone();
            bad[k] = v;
            fs::write(&file, bad.to_string())?;
            assert!(run().is_err());
        }
        fs::write(&file, study.to_string())?;
        for role in ["guard", "control"] {
            assert!(run().is_err());
            let p = root.join(format!("mesh-7-1250-{role}"));
            fs::create_dir(&p)?;
            fs::write(p.join("trace.json"), trace().to_string())?;
        }
        let mut duplicate = study.clone();
        duplicate["cases"] = json!([case(), case()]);
        fs::write(&file, duplicate.to_string())?;
        assert!(run().is_err());
        assert!(!out.exists());
        fs::write(&file, study.to_string())?;
        run()?;
        let svg = fs::read_to_string(&out)?;
        assert!(run().is_err());
        assert_eq!(fs::read_to_string(out)?, svg);
        Ok(())
    })();
    fs::remove_dir_all(root)?;
    r
}

#[test]
fn escape_marks_the_last_tick_and_leaves_the_rest_of_the_band_empty() -> Result<()> {
    let mut c = case();
    let mut t = trace();
    for r in t.as_array_mut().unwrap() {
        r["time_ns"] = json!(u(&r["time_ns"])? / 2);
    }
    c["guard"]["escaped_after_tick"] = json!(30);
    c["guard"]["patch_motion_at_400ms_m"] = Value::Null;
    let row = make_row(&t, &c, "guard", 10_000_000, 120)?;
    assert_eq!(row.events.last(), Some(&(30, "E")));
    assert!(row.motion_mm.is_none());
    assert!(draw(&[row], 10_000_000, 120)?.contains("Ended earlier"));
    c["guard"]["escaped_after_tick"] = json!(29);
    assert!(make_row(&t, &c, "guard", 10_000_000, 120).is_err());
    c["guard"]["escaped_after_tick"] = json!(30);
    c["guard"]["patch_motion_at_400ms_m"] = json!([0., 0.]);
    assert!(make_row(&t, &c, "guard", 10_000_000, 120).is_err());
    Ok(())
}
