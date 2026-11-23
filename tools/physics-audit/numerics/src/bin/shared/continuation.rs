// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Raw shared-receipt equality, independently fitted postcontact decisions and score readback.
use super::*;
#[derive(Clone, Copy)]
enum Common {
    Lost,
    Empty,
    Loaded([f64; 3]),
}
pub(super) struct Row {
    feature: Common,
    pub(super) errors: [[std::result::Result<[f64; 2], V>; 2]; 2],
}

pub(super) fn receipt(trial: &Value, tick: u64) -> Result<Row> {
    let observed = |expected: Policy| -> Result<(&Value, Option<&Value>, bool)> {
        let p = arr(&trial["paths"])?
            .iter()
            .find(|p| pol(&p["profile"]).is_ok_and(|r| r == expected))
            .ok_or("missing continuation treatment")?;
        let first_sample = arr(&p["samples"])?
            .iter()
            .find(|s| s["i"] == 1)
            .ok_or("missing first public state")?;
        let first = p["first"]["end_ns"]
            .as_u64()
            .filter(|t| *t <= 500_000)
            .map(|_| &p["first"]);
        Ok((
            first_sample,
            first,
            p["first_disjoint_tick"]
                .as_u64()
                .is_some_and(|t| t <= tick + 1),
        ))
    };
    let (s, first, lost) = observed((8, 8, true))?;
    assert_eq!((s, first, lost), observed(R8)?);
    let j = d(&s["cumulative_normal_ns"])?;
    assert_eq!(first.is_some(), j > 0.);
    let feature = if lost {
        Common::Lost
    } else if j == 0. {
        Common::Empty
    } else {
        let sum_squares = arr(&s["cumulative_corners_ns"])?
            .iter()
            .map(|p| d(p).map(|v| v * v))
            .collect::<Result<Vec<_>>>()?
            .into_iter()
            .sum::<f64>();
        let tangent = vec3(&s["cumulative_tangent_ns"])?;
        Common::Loaded([
            j * j / sum_squares,
            d(&s["impulse_weighted_abs_post_normal_m_s"])?,
            dist(tangent, [0.; 3]) / j,
        ])
    };
    Ok(Row {
        feature,
        errors: [[Err(V::Pending); 2]; 2],
    })
}

type Tagged<'a> = (&'a Key, &'a Row);

fn check_scores(
    section: &Value,
    rows: &[Tagged],
    selected: impl Fn(&Key) -> bool,
) -> Result<usize> {
    let mut series = 0;
    for (w, when) in ["at_5ms", "at_13ms"].into_iter().enumerate() {
        for (r, profile) in [FINE, ALT].into_iter().enumerate() {
            let s = &section[name(profile)][when];
            let mut values: [Vec<f64>; 6] = Default::default();
            let (mut unavailable, mut lost, mut continued) = (0, 0, 0);
            for &(k, row) in rows {
                match row.errors[w][r] {
                    Ok([stop, keep]) => {
                        let pick = if selected(k) {
                            continued += 1;
                            keep
                        } else {
                            stop
                        };
                        for (v, n) in values.iter_mut().zip([
                            stop,
                            keep,
                            pick,
                            keep - stop,
                            pick - stop,
                            pick - keep,
                        ]) {
                            v.push(n);
                        }
                    }
                    Err(V::Disjoint) => lost += 1,
                    Err(V::Pending) => unavailable += 1,
                    _ => unreachable!(),
                }
            }
            assert_eq!(u(&s["default_pending"])?, unavailable);
            assert_eq!(u(&s["disjoint"])?, lost);
            assert_eq!(u(&s["selected_eight"])?, continued);
            assert_eq!(u(&s["matched"])? as usize, values[0].len());
            for (i, stat) in [
                &s["absolute_com_mm"]["one"],
                &s["absolute_com_mm"]["eight"],
                &s["absolute_com_mm"]["selected"],
                &s["eight_minus_one_mm"],
                &s["selected_minus_one_mm"],
                &s["selected_minus_eight_mm"],
            ]
            .into_iter()
            .enumerate()
            {
                signatures::check_distr(stat, &values[i])?;
                series += 1;
            }
        }
    }
    Ok(series)
}

pub(super) fn verify(
    out: &Path,
    rows: &BTreeMap<Key, Row>,
    ids: &BTreeMap<Key, (u64, u64)>,
) -> Result<Value> {
    let report = val(&out.join("relaxation-continuation.json"))?;
    let receipts = fs::read_to_string(out.join("relaxation-contact-receipts.jsonl"))?;
    let mut saved = BTreeMap::new();
    for line in receipts.lines() {
        let v: Value = serde_json::from_str(line)?;
        let k = &v["site"];
        let key = (
            u(&k["offset_mm"])? as usize,
            u(&k["drive_percent"])? as usize,
            k["holds"].as_bool().ok_or("history")?,
            u(&k["stroke"])? as usize,
        );
        assert_eq!(ids[&key], (u(&k["index"])?, u(&k["cut_tick"])?));
        assert!(saved.insert(key, v).is_none());
    }
    assert_eq!(saved.len(), rows.len());
    assert_eq!(
        u(&report["each_cut_first_receipt_exactly_equal"])? as usize,
        rows.len()
    );
    let mut features = BTreeMap::new();
    for (&k, r) in rows {
        let m = &saved[&k]["common_first_receipt"];
        match r.feature {
            Common::Lost => assert_eq!(*m, "Disjoint"),
            Common::Empty => assert_eq!(*m, "Unloaded"),
            Common::Loaded(a) => {
                let meas = &m["Loaded"];
                let v = [
                    d(&meas["effective_quadrants"])?,
                    d(&meas["residual_m_s"])?,
                    d(&meas["traction_ratio"])?,
                ];
                for (old, new) in a.into_iter().zip(v) {
                    eq(old, new)?;
                }
                features.insert(k, v);
            }
        }
    }
    let is_held = |k: &Key| k.0.is_multiple_of(25);
    let mut limits = [None; 2];
    for (i, field) in [
        "training_median_residual_m_s",
        "training_median_traction_ratio",
    ]
    .iter()
    .enumerate()
    {
        let mut samples = features
            .iter()
            .filter(|(k, _)| !is_held(k))
            .map(|(_, f)| f[i + 1])
            .collect::<Vec<_>>();
        samples.sort_unstable_by(f64::total_cmp);
        limits[i] = (!samples.is_empty()).then(|| samples[samples.len().div_ceil(2) - 1]);
        if let Some(t) = limits[i] {
            eq(t, d(&report[field])?)?;
        } else {
            assert!(report[field].is_null());
        }
    }
    let bucket = |k: &Key| -> Option<usize> {
        let f = features.get(k)?;
        let count = if f[0] <= 1.5 {
            0
        } else if f[0] <= 2.5 {
            1
        } else if f[0] <= 3.5 {
            2
        } else {
            3
        };
        Some(count + usize::from(f[1] > limits[0]?) * 4 + usize::from(f[2] > limits[1]?) * 8)
    };
    let mut picks = [false; 16];
    let mut series = 0;
    for (i, pick) in picks.iter_mut().enumerate() {
        let (test, train): (Vec<_>, Vec<_>) = rows
            .iter()
            .filter(|(k, _)| bucket(k) == Some(i))
            .partition(|(k, _)| is_held(k));
        let paired: Vec<_> = train
            .iter()
            .filter_map(|(_, r)| Some([r.errors[1][0].ok()?, r.errors[1][1].ok()?]))
            .collect();
        let b = &report["bins"][i];
        assert_eq!(u(&b["comparable_training_cuts"])? as usize, paired.len());
        assert_eq!(u(&b["training_cuts"])? as usize, train.len());
        assert_eq!(u(&b["held_out_cuts"])? as usize, test.len());
        *pick = !paired.is_empty();
        for r in 0..2 {
            let sum: f64 = paired.iter().map(|p| p[r][1] - p[r][0]).sum();
            if paired.is_empty() {
                assert!(b["training_mean_continued_minus_stopped_mm"][r].is_null());
            } else {
                eq(
                    d(&b["training_mean_continued_minus_stopped_mm"][r])?,
                    sum / paired.len() as f64,
                )?;
            }
            *pick &= sum < 0.;
        }
        assert_eq!(b["continue_eight"], *pick);
        series += check_scores(&b["training"], &train, |_| *pick)?;
        series += check_scores(&b["held_out"], &test, |_| *pick)?;
    }
    let chosen = |k: &Key| bucket(k).is_some_and(|i| picks[i]);
    for k in rows.keys() {
        assert_eq!(saved[k]["bucket"], json!(bucket(k)));
        assert_eq!(saved[k]["continue_eight"], chosen(k));
    }
    let (held, train): (Vec<_>, Vec<_>) = rows.iter().partition(|(k, _)| is_held(k));
    let check_split = |summary: &Value, set: &[Tagged]| -> Result<usize> {
        let loads = set
            .iter()
            .filter(|(_, r)| matches!(r.feature, Common::Loaded(_)))
            .count();
        let zero = set
            .iter()
            .filter(|(_, r)| matches!(r.feature, Common::Empty))
            .count();
        assert_eq!(summary["loaded"], loads);
        assert_eq!(summary["unloaded"], zero);
        assert_eq!(summary["first_interval_disjoint"], set.len() - loads - zero);
        check_scores(&summary["scores"], set, chosen)
    };
    series += check_split(&report["training"], &train)? + check_split(&report["held_out"], &held)?;
    for (i, o) in [175, 200, 225, 250, 275].into_iter().enumerate() {
        let subset: Vec<_> = held.iter().copied().filter(|(k, _)| k.0 == o).collect();
        series += check_split(&report["held_by_offset"][i]["evaluation"], &subset)?;
    }
    let mut candidates = held
        .iter()
        .filter_map(|(k, r)| {
            if !matches!(r.feature, Common::Loaded(_)) {
                return None;
            }
            let [a, b] = r.errors[1][0].ok()?;
            Some((b - a, ids[k].0, k.3))
        })
        .collect::<Vec<_>>();
    candidates.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
    if let Some((value, idx, stroke)) = candidates.first() {
        eq(*value, d(&report["outlier"]["continued_minus_stopped_mm"])?)?;
        assert_eq!(report["outlier"]["site"]["index"], *idx);
        assert_eq!(report["outlier"]["site"]["stroke"], *stroke);
    } else {
        assert!(report["outlier"].is_null());
    }
    let marks = chart(out, &report)?;
    Ok(
        json!({"raw_matching_prefixes_checked":rows.len(),"positive_contact_features_checked":features.len()*3,"scores_checked":series,"training_decisions_checked":16,"held_out_svg_cells_checked":marks,"held_out_outlier_checked":!candidates.is_empty()}),
    )
}
fn chart(out: &Path, data: &Value) -> Result<usize> {
    let svg = fs::read_to_string(out.join("relaxation-continuation.svg"))?;
    let mut cells = BTreeMap::new();
    let mut labels = BTreeMap::new();
    let mut lines = svg.lines();
    while let Some(tag) = lines.next() {
        if tag.starts_with("<rect ")
            && svg_cells::attr(tag, "width")? == "98"
            && svg_cells::attr(tag, "height")? == "85"
        {
            cells.insert(
                (
                    svg_cells::attr(tag, "x")?.parse::<usize>()?,
                    svg_cells::attr(tag, "y")?.parse::<usize>()?,
                ),
                svg_cells::attr(tag, "fill")?,
            );
        } else if tag.starts_with("<text ") {
            labels.insert(
                (
                    svg_cells::attr(tag, "x")?.parse::<usize>()?,
                    svg_cells::attr(tag, "y")?.parse::<usize>()?,
                ),
                lines.next().ok_or("missing label")?,
            );
        }
    }
    let xs: Vec<_> = cells
        .keys()
        .map(|p| p.0)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    let ys: Vec<_> = cells
        .keys()
        .map(|p| p.1)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    assert_eq!((xs.len(), ys.len(), cells.len()), (8, 4, 32));
    for ((x, y), shade) in cells {
        let col = xs.binary_search(&x).unwrap();
        let row = ys.binary_search(&y).unwrap();
        let r = [FINE, ALT][col / 4];
        let s =
            &data["bins"][4 * row + col % 4]["held_out"][name(r)]["at_13ms"]["eight_minus_one_mm"];
        let mu = s["mean"].as_f64();
        let n = u(&s["n"])?;
        assert_eq!(
            labels[&(x + 49, y + 29)],
            mu.map_or_else(|| "No data".into(), |v| format!("{v:+.3} mm"))
        );
        assert_eq!(labels[&(x + 49, y + 56)], format!("n = {n}"));
        assert_eq!(
            shade,
            if mu.is_some_and(|v| v < 0.) {
                "#B4DDD8"
            } else if mu.is_some_and(|v| v > 0.) {
                "#F7D1B9"
            } else {
                "#DCE0E0"
            }
        );
    }
    Ok(32)
}
