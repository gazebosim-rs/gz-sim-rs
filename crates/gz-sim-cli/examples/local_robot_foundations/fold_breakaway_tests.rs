// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::fold_pull::tests::series;

#[test]
fn adaptive_bracket_repeats_limits_and_can_move_or_remain_open() -> Result<()> {
    for (prior, expected) in [
        (None, Some([1100, 1125])),
        (Some([1100, 1125]), Some([1100, 1125])),
        (Some([1200, 1225]), Some([1106, 1125])),
        (Some([0, 25]), Some([1100, 1125])),
    ] {
        let mut tested = Vec::new();
        let b = search(prior, |f| {
            assert!(!tested.contains(&f));
            tested.push(f);
            Ok(f >= 1120)
        })?;
        assert_eq!(b, expected);
        if let Some([lo, hi]) = prior {
            assert!(lo == 0 || tested.contains(&lo));
            assert!(tested.contains(&hi));
        }
    }
    assert_eq!(search(None, |_| Ok(false))?, None);
    assert_eq!(search(None, |f| Ok(f > 0))?, Some([0, 18]));
    for (prev, fail) in [(None, 300), (Some([1100, 1125]), 1125), (None, 975)] {
        assert!(
            search(prev, |f| if f == fail {
                Err("missing load observation".into())
            } else {
                Ok(f >= 1050)
            })
            .is_err()
        );
    }
    Ok(())
}

#[test]
fn only_persistent_directional_motion_counts_and_support_is_independent() -> Result<()> {
    let sham = series();
    let mut o = series();
    let start = o.trace.len() - WINDOW;
    for t in &mut o.trace[start..] {
        t.pad_frame_y_m = [SLIP_M, -2. * SLIP_M];
    }
    assert!(!compare("test".into(), 1, &o, &sham)?.lasting_slip);
    for t in &mut o.trace[start + 1..] {
        t.pad_frame_y_m[0] = 2. * SLIP_M;
    }
    assert!(!compare("test".into(), 1, &o, &sham)?.lasting_slip);
    o.trace[start].pad_frame_y_m[0] = 2. * SLIP_M;
    o.trace[start].bench_up_n = 0.08;
    let m = compare("test".into(), 1, &o, &sham)?;
    assert!(m.lasting_slip);
    assert_eq!(m.recovered_support_ticks, WINDOW - 1);
    assert_eq!(m.recovered_bench_ticks, 1);
    o.anchor = Value::Null;
    assert!(compare("bad".into(), 0, &o, &sham).is_err());
    o.anchor = sham.anchor.clone();
    o.trace[0].force_y_n += 1.;
    assert!(compare("bad".into(), 0, &o, &sham).is_err());
    o.baseline += 1;
    assert!(compare("bad".into(), 0, &o, &sham).is_err());
    Ok(())
}

#[test]
fn failed_branches_cannot_publish_a_partial_or_unmatched_result() -> Result<()> {
    let tmp = tempfile::tempdir()?;
    let d = tmp.path().join("study");
    for trial in [
        (|c, _, _, _, _: &Path| -> Result<Outcome> {
            if c == 5 {
                panic!("test worker stopped")
            };
            Ok(series())
        }) as Trial,
        |c, _, _, _, _| {
            if c == 7 {
                Err("failed sham".into())
            } else {
                Ok(series())
            }
        },
        |_, f, _, _, _| {
            if f > 0. {
                Err("failed pull".into())
            } else {
                Ok(series())
            }
        },
        |_, f, _, _, _| {
            let mut o = series();
            o.anchor = (f == 0.).into();
            Ok(o)
        },
        |_, _, clock, _, _| {
            let mut o = series();
            o.anchor = clock.into();
            Ok(o)
        },
        |_, _, clock, _, _| {
            let mut o = series();
            o.trace[0].command_stroke_m = clock as f64;
            Ok(o)
        },
    ] {
        assert!(write_new(&d, |p| run_with(p, trial)).is_err());
        assert!(!d.exists());
    }
    write_new(&d, |p| {
        run_with(p, |_, f, _, _, _| {
            let mut o = series();
            for t in &mut o.trace[WINDOW..] {
                t.pad_frame_y_m = [f * SLIP_M; 2]
            }
            Ok(o)
        })
    })?;
    let j: Value = serde_json::from_slice(&fs::read(d.join("comparison.json"))?)?;
    assert_eq!(j["trials"][1]["fine"]["bracket_mn"], json!([993, 1012]));
    Ok(())
}
