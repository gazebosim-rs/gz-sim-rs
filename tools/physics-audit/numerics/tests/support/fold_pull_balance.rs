// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn row(k: usize, force: f64, jaws: [f64; 2], bench: f64, momentum: f64) -> Sample {
    Sample {
        time_ns: 100 + k as u64 * 1_000_000,
        force,
        jaws_on_sheet: jaws,
        bench_on_sheet: bench,
        momentum,
    }
}

fn matched() -> (Vec<Sample>, Vec<Sample>) {
    // The unloaded reference is already drifting and its jaws have unequal loads.
    let sham: Vec<_> = (0..4).map(|i| row(i, 0., [0.1, -0.1], 0., 0.3)).collect();
    let pull = vec![
        row(0, 0., [0.1, -0.1], 0., 0.3),
        row(1, 2., [-0.9, -0.6], 0., 0.3005),
        row(2, 2., [-0.4, -0.6], -0.5, 0.301),
        row(3, 0., [-0.4, -0.6], 0., 0.3),
    ];
    (pull, sham)
}

#[test]
fn recovers_impulse_on_unequal_jaws_and_bench_alongside_a_moving_sheet() {
    let (p, s) = matched();
    let r = budget(&p, &s, 100, 1_000_000).unwrap();
    assert!(
        (number(&r["last_loaded_tick"]["sheet_momentum_change_ns"]).unwrap() - 0.001).abs() < 1e-14
    );
    assert_eq!(
        r["last_loaded_tick"]["jaws_on_sheet_ns"],
        json!([-0.0015, -0.001])
    );
    assert_eq!(r["after_recovery"]["bench_on_sheet_ns"], -0.0005);
    assert_eq!(r["after_recovery"]["sheet_momentum_change_ns"], 0.);
    assert!(number(&r["max_cumulative_residual_ns"]).unwrap() < 1e-15);
    let mut wrong = p;
    wrong[1].jaws_on_sheet[0] = 1.1;
    let error = budget(&wrong, &s, 100, 1_000_000).unwrap();
    assert!(
        (number(&error["max_residual_fraction_of_applied_impulse"]).unwrap() - 0.5).abs() < 1e-12
    );
}

#[test]
fn rejects_mismatched_clocks_reference_loads_and_incomplete_pulses() {
    let (mut p, mut s) = matched();
    assert!(budget(&p, &s[..2], 100, 1_000_000).is_err());
    assert!(budget(&p[..1], &s[..1], 100, 1_000_000).is_err());
    assert!(budget(&p, &s, 100, 0).is_err());
    assert!(budget(&p, &s, 99, 1_000_000).is_err());
    s[2].time_ns += 1;
    assert!(budget(&p, &s, 100, 1_000_000).is_err());
    s[2].time_ns -= 1;
    s[2].force = 2.;
    assert!(budget(&p, &s, 100, 1_000_000).is_err());
    s[2].force = 0.;
    p[2].force = -2.;
    assert!(budget(&p, &s, 100, 1_000_000).is_err());
    p[2].force = 2.;
    p[0].momentum += 0.1;
    assert!(budget(&p, &s, 100, 1_000_000).is_err());
    p[0].momentum = s[0].momentum;
    p[0].jaws_on_sheet[1] += 0.1;
    assert!(budget(&p, &s, 100, 1_000_000).is_err());
    p[0].jaws_on_sheet = s[0].jaws_on_sheet;
    p[0].bench_on_sheet = 0.1;
    assert!(budget(&p, &s, 100, 1_000_000).is_err());
    p[0].bench_on_sheet = 0.;
    p[0].force = 1.;
    assert!(budget(&p, &s, 100, 1_000_000).is_err());
    p[0].force = 0.;
    assert!(budget(&s, &s, 100, 1_000_000).is_err());
    assert!(budget(&p[..3], &s[..3], 100, 1_000_000).is_err());
}

#[test]
fn reads_world_reaction_signs_and_refuses_missing_or_malformed_evidence() -> Result<()> {
    let root = std::env::temp_dir().join(format!("gz-pull-balance-{}", std::process::id()));
    fs::create_dir(&root)?;
    let result = (|| -> Result<()> {
        let path = root.join("comparison.json");
        assert!(audit(&root).is_err());
        fs::write(&path, "[")?;
        assert!(audit(&root).is_err());
        let mut study = json!({});
        for field in ["dt_ns", "trials"] {
            assert!(audit(&root).is_err());
            study[field] = if field == "dt_ns" {
                json!(1_000_000)
            } else {
                json!([])
            };
            fs::write(&path, study.to_string())?;
        }
        let (p, s) = matched();
        let mut cases = vec![];
        for columns in [5, 7] {
            assert!(audit(&root).is_err());
            cases.push(json!({"columns":columns}));
            study["trials"] = json!(cases);
            fs::write(&path, study.to_string())?;
            assert!(audit(&root).is_err());
            cases.last_mut().unwrap()["baseline_time_ns"] = json!(100);
            study["trials"] = json!(cases);
            fs::write(&path, study.to_string())?;
            for (name, rows) in [("pull", &p), ("sham", &s)] {
                assert!(audit(&root).is_err());
                let dir = root.join(format!("mesh-{columns}-{name}"));
                fs::create_dir(&dir)?;
                let trace = dir.join("trace.json");
                fs::write(&trace, "[")?;
                assert!(audit(&root).is_err());
                fs::write(&trace, "[{}]")?;
                assert!(audit(&root).is_err());
                let mut data = vec![json!({"time_ns":0})];
                for r in rows {
                    data.push(json!({"time_ns":r.time_ns,"force_y_n":r.force,
                        "jaw_side_reaction_n":r.jaws_on_sheet.map(|v|-v),"bench_side_reaction_n":-r.bench_on_sheet,
                        "momentum_y_kg_m_s":r.momentum}));
                }
                for key in [
                    "force_y_n",
                    "jaw_side_reaction_n",
                    "bench_side_reaction_n",
                    "momentum_y_kg_m_s",
                ] {
                    let mut invalid = data.clone();
                    invalid[1][key] = Value::Null;
                    fs::write(&trace, json!(invalid).to_string())?;
                    assert!(audit(&root).is_err());
                }
                fs::write(&trace, json!(data).to_string())?;
            }
        }
        let report = audit(&root)?;
        assert_eq!(
            report["meshes"][1]["balance"]["after_recovery"]["applied_ns"],
            0.004
        );
        Ok(())
    })();
    fs::remove_dir_all(root)?;
    result
}
