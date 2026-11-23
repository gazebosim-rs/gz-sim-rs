// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent direct intersection of two rectangles, plus held-out decision readback.
use super::*;

const METRICS: [&str; 5] = [
    "support_height_span_mm",
    "prospective_tangent_m_s",
    "supported_projected_area_fraction",
    "spin_rad_s",
    "recorded_gap_mm",
];
const SPLITS: [&str; 2] = ["training", "held_out"];
pub(super) struct Record {
    features: Option<[f64; 5]>,
    pub errors: [[std::result::Result<[f64; 2], V>; 2]; 2], // window, reference, candidate
}
pub(super) fn raw(cut: &Value, pad_x: f64) -> Result<Record> {
    let q = arr(&cut["foot_pose_xyzw"])?;
    let (x, y, z, w) = (d(&q[0])?, d(&q[1])?, d(&q[2])?, d(&q[3])?);
    let a = [
        1. - 2. * (y * y + z * z),
        2. * (x * y + z * w),
        2. * (x * z - y * w),
    ];
    let b = [
        2. * (x * y - z * w),
        1. - 2. * (x * x + z * z),
        2. * (y * z + x * w),
    ];
    let c = [
        2. * (x * z + y * w),
        2. * (y * z - x * w),
        1. - 2. * (x * x + y * y),
    ];
    let com = vec3(&cut["com_m"])?;
    let u = vec3(&cut["com_velocity_m_s"])?;
    let omega = vec3(&cut["foot_angular_velocity_rad_s"])?;
    let center: [f64; 3] = std::array::from_fn(|i| com[i] - 0.13 * c[i]);
    let sole = [(-0.18, -0.1), (0.18, -0.1), (0.18, 0.1), (-0.18, 0.1)].map(|(i, j)| {
        [
            center[0] + a[0] * i + b[0] * j,
            center[1] + a[1] * i + b[1] * j,
        ]
    });
    let pad = [
        (pad_x - 0.33, -0.18),
        (pad_x + 0.33, -0.18),
        (pad_x + 0.33, 0.18),
        (pad_x - 0.33, 0.18),
    ]
    .map(|(i, j)| [i, j]);
    let cross = |p: [f64; 2], q: [f64; 2]| p[0] * q[1] - p[1] * q[0];
    let sub = |p: [f64; 2], q: [f64; 2]| [p[0] - q[0], p[1] - q[1]];
    let det = cross([a[0], a[1]], [b[0], b[1]]);
    let local = |p: [f64; 2]| {
        let d = sub(p, [center[0], center[1]]);
        [cross(d, [b[0], b[1]]) / det, cross([a[0], a[1]], d) / det]
    };
    let mut corners = Vec::<[f64; 2]>::new();
    for &p in &sole {
        if (p[0] - pad_x).abs() <= 0.33 + 1e-14 && p[1].abs() <= 0.18 + 1e-14 {
            corners.push(p);
        }
    }
    for &p in &pad {
        let [u, v] = local(p);
        if u.abs() <= 0.18 + 1e-14 && v.abs() <= 0.1 + 1e-14 {
            corners.push(p);
        }
    }
    // Find every edge intersection directly, independently of the clipping implementation.
    for i in 0..4 {
        for j in 0..4 {
            let p = sole[i];
            let r = sub(sole[(i + 1) % 4], p);
            let q = pad[j];
            let s = sub(pad[(j + 1) % 4], q);
            let div = cross(r, s);
            if div.abs() < 1e-15 {
                continue;
            }
            let t = cross(sub(q, p), s) / div;
            let v = cross(sub(q, p), r) / div;
            if (-1e-13..=1. + 1e-13).contains(&t) && (-1e-13..=1. + 1e-13).contains(&v) {
                corners.push([p[0] + t * r[0], p[1] + t * r[1]]);
            }
        }
    }
    let mut poly = Vec::new();
    for p in corners {
        if !poly
            .iter()
            .any(|q: &[f64; 2]| (p[0] - q[0]).hypot(p[1] - q[1]) < 1e-12)
        {
            poly.push(p);
        }
    }
    let features = if poly.len() < 3 {
        None
    } else {
        let center_xy = [0, 1].map(|i| poly.iter().map(|v| v[i]).sum::<f64>() / poly.len() as f64);
        let angle = |p: &[f64; 2]| (p[1] - center_xy[1]).atan2(p[0] - center_xy[0]);
        poly.sort_unstable_by(|a, b| angle(a).total_cmp(&angle(b)));
        let area2 = poly
            .iter()
            .enumerate()
            .map(|(i, a)| cross(*a, poly[(i + 1) % poly.len()]))
            .sum::<f64>()
            .abs();
        let xyz = poly
            .iter()
            .map(|&p| {
                let [u, v] = local(p);
                [p[0], p[1], center[2] + u * a[2] + v * b[2]]
            })
            .collect::<Vec<_>>();
        let lo = xyz.iter().map(|v| v[2]).min_by(f64::total_cmp).unwrap();
        let hi = xyz.iter().map(|v| v[2]).max_by(f64::total_cmp).unwrap();
        let lowest = xyz.iter().filter(|p| p[2] - lo <= 1e-9).collect::<Vec<_>>();
        let r = [0, 1, 2]
            .map(|i| lowest.iter().map(|p| p[i]).sum::<f64>() / lowest.len() as f64 - com[i]);
        let vx = u[0] + omega[1] * r[2] - omega[2] * r[1];
        let vy = u[1] + omega[2] * r[0] - omega[0] * r[2];
        (area2 >= 2e-14).then_some([
            (hi - lo) * 1e3,
            vx.hypot(vy),
            area2 / (2. * 0.36 * 0.2 * det.abs()),
            dist(omega, [0.; 3]),
            d(&cut["gap_m"])? * 1000.,
        ])
    };
    Ok(Record {
        features,
        errors: [[Err(V::Pending); 2]; 2],
    })
}
// Compare the geometry oracle before using the stored scalar bit patterns for
// bins: an anchor equal to a nearest-rank training quantile belongs to the lower bin.
pub(super) fn check_features(rec: &mut Record, measured: &Value) -> Result<()> {
    if let Some(v) = rec.features {
        let mut stored = [0.; 5];
        for (i, key) in METRICS.iter().enumerate() {
            stored[i] = d(&measured[key])?;
            eq(v[i], stored[i])?;
        }
        rec.features = Some(stored);
    } else {
        assert!(measured.is_null());
    }
    Ok(())
}
pub(super) fn check_distr(reported: &Value, vals: &[f64]) -> Result<()> {
    assert_eq!(u(&reported["n"])? as usize, vals.len());
    assert_eq!(
        u(&reported["positive"])? as usize,
        vals.iter().filter(|&&x| x > 0.).count()
    );
    assert_eq!(
        u(&reported["negative"])? as usize,
        vals.iter().filter(|&&x| x < 0.).count()
    );
    if vals.is_empty() {
        assert!(reported["mean"].is_null());
        return Ok(());
    }
    eq(
        d(&reported["mean"])?,
        vals.iter().sum::<f64>() / vals.len() as f64,
    )?;
    for (val, k) in [("p50", 50), ("p95", 95)] {
        eq(d(&reported[val])?, q(vals, k))?;
    }
    Ok(())
}
pub(super) fn verify(out: &Path, obs: &BTreeMap<Key, Record>) -> Result<Value> {
    let report = val(&out.join("relaxation-signatures.json"))?;
    let test = |k: Key| k.0.is_multiple_of(25);
    let eligible = obs
        .iter()
        .filter(|(_, v)| v.features.is_some())
        .collect::<Vec<_>>();
    let train = eligible
        .iter()
        .copied()
        .filter(|&(&k, _)| !test(k))
        .collect::<Vec<_>>();
    let anchor = |i: usize| {
        let mut x = train
            .iter()
            .map(|(_, v)| v.features.unwrap()[i])
            .collect::<Vec<_>>();
        x.sort_unstable_by(f64::total_cmp);
        [1usize, 2].map(|n| x[(n * x.len()).div_ceil(3) - 1])
    };
    let trained = |rows: &[(&Key, &Record)]| {
        let scores = rows
            .iter()
            .filter_map(|(_, r)| r.errors[1][0].ok())
            .collect::<Vec<_>>();
        (!scores.is_empty()).then(|| scores.iter().map(|v| v[1] - v[0]).sum::<f64>() < 0.)
    };
    assert_eq!(u(&report["native_cuts"])? as usize, obs.len());
    assert_eq!(
        u(&report["without_supported_sole_polygon"])? as usize,
        obs.len() - eligible.len()
    );
    assert_eq!(u(&report["training_classified"])? as usize, train.len());
    assert_eq!(
        u(&report["held_out_classified"])? as usize,
        eligible.len() - train.len()
    );
    let Some(global) = trained(&train) else {
        assert_eq!(report["status"], "insufficient training cuts");
        return Ok(json!({"fitted":false}));
    };
    let cut = [anchor(0), anchor(1)];
    for (i, metric) in ["span_mm_training_tertiles", "tangent_m_s_training_tertiles"]
        .iter()
        .enumerate()
    {
        for k in [0, 1] {
            eq(cut[i][k], d(&report[metric][k])?)?;
        }
    }
    let bucket = |r: &Record| {
        let f = r.features.unwrap();
        let category = |i: usize| {
            if f[i] <= cut[i][0] {
                0
            } else if f[i] <= cut[i][1] {
                1
            } else {
                2
            }
        };
        category(0) + 3 * category(1)
    };
    let choose: [bool; 9] = std::array::from_fn(|bin| {
        trained(
            &train
                .iter()
                .copied()
                .filter(|(_, v)| bucket(v) == bin)
                .collect::<Vec<_>>(),
        )
        .unwrap_or(global)
    });
    let span_choices: [bool; 3] = std::array::from_fn(|bin| {
        trained(
            &train
                .iter()
                .copied()
                .filter(|(_, r)| bucket(r) % 3 == bin)
                .collect::<Vec<_>>(),
        )
        .unwrap_or(global)
    });
    assert_eq!(
        report["span_only_control"]["selected_eight_by_span"],
        json!(span_choices)
    );
    let agreement = &report["reference_agreement"];
    let mut joint_choices = [false; 9];
    if !agreement.is_null() {
        for (i, take_eight) in joint_choices.iter_mut().enumerate() {
            let paired: Vec<_> = train
                .iter()
                .filter(|(_, r)| bucket(r) == i)
                .filter_map(|(_, r)| Some([r.errors[1][0].ok()?, r.errors[1][1].ok()?]))
                .collect();
            let saved = &agreement["bins"][i];
            assert_eq!(u(&saved["matched"])? as usize, paired.len());
            let mut strictly_better = [false; 2];
            for r in 0..2 {
                let mean = &saved["mean_eight_minus_one_mm"][r];
                if paired.is_empty() {
                    assert!(mean.is_null());
                } else {
                    let sum = paired
                        .iter()
                        .map(|pair| pair[r][1] - pair[r][0])
                        .sum::<f64>();
                    eq(d(mean)?, sum / paired.len() as f64)?;
                    strictly_better[r] = sum < 0.;
                }
            }
            *take_eight = strictly_better == [true, true];
            assert_eq!(saved["take_eight"], *take_eight);
        }
    }
    let mut series = 0;
    for (split, name) in SPLITS.iter().enumerate() {
        let rows = eligible
            .iter()
            .copied()
            .filter(|(k, _)| test(**k) == (split == 1))
            .map(|(_, r)| r.features.unwrap())
            .collect::<Vec<_>>();
        let cov = &report["covariates"][name];
        assert_eq!(
            u(&cov["below_0_1_mm_s"])? as usize,
            rows.iter().filter(|v| v[1] < 1e-4).count()
        );
        for (i, label) in [
            "height_span_mm",
            "tangent_m_s",
            "supported_area_fraction",
            "spin_rad_s",
            "gap_mm",
        ]
        .iter()
        .enumerate()
        {
            check_distr(&cov[label], &rows.iter().map(|v| v[i]).collect::<Vec<_>>())?;
            series += 1;
        }
    }
    let last_group: usize = if agreement.is_null() { 10 } else { 16 };
    for group in 0..=last_group {
        let per_offset = group.checked_sub(12_usize);
        let block = if let Some(offset_idx) = per_offset {
            let row = &agreement["held_out_by_offset"][offset_idx];
            assert_eq!(u(&row["offset_mm"])?, 175 + 25 * offset_idx as u64);
            &row["scores"]
        } else if group == 11 {
            agreement
        } else if group == 10 {
            &report["span_only_control"]
        } else if group == 9 {
            &report
        } else {
            assert_eq!(
                u(&report["strata"][group]["selected_relaxation"])?,
                if choose[group] { 8 } else { 1 }
            );
            &report["strata"][group]
        };
        for (split, name_split) in SPLITS.iter().enumerate() {
            if per_offset.is_some() && split == 0 {
                continue;
            }
            for (refidx, policy) in [FINE, ALT].into_iter().enumerate() {
                for w in 0..2 {
                    let split_scores = if per_offset.is_some() {
                        block
                    } else {
                        &block[name_split]
                    };
                    let s = &split_scores[name(policy)][NAMES[w + 1]];
                    let mut scores: [Vec<f64>; 6] = Default::default();
                    let mut disjoint = 0;
                    let mut pending = 0;
                    let mut count8 = 0;
                    for &(&key, r) in &eligible {
                        let g = bucket(r);
                        if (group < 9 && g != group)
                            || test(key) != (split == 1)
                            || per_offset.is_some_and(|i| key.0 != 175 + 25 * i)
                        {
                            continue;
                        }
                        match r.errors[w][refidx] {
                            Ok([one, eight]) => {
                                let use_r8 = if group >= 11 {
                                    joint_choices[g]
                                } else if group == 10 {
                                    span_choices[g % 3]
                                } else {
                                    choose[g]
                                };
                                let picked = if use_r8 {
                                    count8 += 1;
                                    eight
                                } else {
                                    one
                                };
                                for (v, e) in scores.iter_mut().zip([
                                    one,
                                    eight,
                                    picked,
                                    eight - one,
                                    picked - one,
                                    picked - eight,
                                ]) {
                                    v.push(e);
                                }
                            }
                            Err(V::Disjoint) => disjoint += 1,
                            Err(V::Pending) => pending += 1,
                            _ => unreachable!(),
                        }
                    }
                    assert_eq!(u(&s["selected_eight"])? as usize, count8);
                    assert_eq!(u(&s["disjoint"])? as usize, disjoint);
                    assert_eq!(u(&s["default_pending"])? as usize, pending);
                    for (vals, reported) in scores.iter().zip([
                        &s["absolute_com_mm"]["one"],
                        &s["absolute_com_mm"]["eight"],
                        &s["absolute_com_mm"]["selected"],
                        &s["eight_minus_one_mm"],
                        &s["selected_minus_one_mm"],
                        &s["selected_minus_eight_mm"],
                    ]) {
                        check_distr(reported, vals)?;
                        series += 1;
                    }
                }
            }
        }
    }
    check_figure(out, &report)?;
    let agreement_points = if agreement.is_null() {
        0
    } else {
        check_agreement_figure(out, agreement)?
    };
    Ok(
        json!({"fitted":true,"svg_cells_checked":18,"agreement_plot_data_or_missing_labels_checked":agreement_points,"signature_scalars_checked":eligible.len()*5,"statistics_vectors_checked":series,"training_choices_checked":if agreement.is_null(){12}else{21},"training_and_held_out_cuts_checked":eligible.len()}),
    )
}

fn check_figure(out: &Path, report: &Value) -> Result<()> {
    let xml = fs::read_to_string(out.join("relaxation-signatures.svg"))?;
    let mut rects = BTreeMap::new();
    let mut texts = BTreeMap::new();
    let mut lines = xml.lines();
    while let Some(tag) = lines.next() {
        if tag.starts_with("<rect ") || tag.starts_with("<text ") {
            let x = svg_cells::attr(tag, "x")?.parse::<usize>()?;
            let y = svg_cells::attr(tag, "y")?.parse::<usize>()?;
            if tag.starts_with("<text ") {
                texts.insert((x, y), lines.next().ok_or("missing plot label")?);
            } else if svg_cells::attr(tag, "width")? == "140"
                && svg_cells::attr(tag, "height")? == "86"
            {
                rects.insert((x, y), svg_cells::attr(tag, "fill")?);
            }
        }
    }
    let xs = rects
        .keys()
        .map(|p| p.0)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();
    let ys = rects
        .keys()
        .map(|p| p.1)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();
    assert_eq!((xs.len(), ys.len(), rects.len()), (6, 3, 18));
    for ((x, y), color) in rects {
        let col = xs.binary_search(&x).unwrap();
        let row = ys.binary_search(&y).unwrap();
        let s = &report["strata"][3 * row + col % 3][SPLITS[col / 3]]["4096x4_r8"]["at_13ms"]["eight_minus_one_mm"];
        let n = u(&s["n"])?;
        let number = if n > 0 {
            format!("{:+.3} mm", d(&s["mean"])?)
        } else {
            "No data".into()
        };
        let shade = if n == 0 {
            "#DCDEDE"
        } else if d(&s["mean"])? < 0. {
            "#B7DCD6"
        } else {
            "#F6CDB5"
        };
        assert_eq!(color, shade);
        assert_eq!(texts[&(x + 70, y + 31)], number);
        assert_eq!(
            texts[&(x + 70, y + 59)],
            format!("{n} landing{}", if n == 1 { "" } else { "s" })
        );
    }
    Ok(())
}

fn check_agreement_figure(out: &Path, agreement: &Value) -> Result<usize> {
    let xml = fs::read_to_string(out.join("relaxation-reference-agreement.svg"))?;
    let mut marks = BTreeMap::new();
    let mut labels = BTreeMap::new();
    let mut lines = xml.lines();
    while let Some(tag) = lines.next() {
        if tag.starts_with("<circle ") && svg_cells::attr(tag, "r")? == "4" {
            let x = svg_cells::attr(tag, "cx")?.parse::<i32>()?;
            let y = svg_cells::attr(tag, "cy")?.parse::<i32>()?;
            assert!(
                marks
                    .insert((x / 480, y), (x, svg_cells::attr(tag, "fill")?))
                    .is_none()
            );
        }
        if tag.starts_with("<text ") {
            let x = svg_cells::attr(tag, "x")?.parse::<i32>()?;
            let y = svg_cells::attr(tag, "y")?.parse::<i32>()?;
            labels.insert((x, y), lines.next().ok_or("missing agreement plot text")?);
        }
    }
    let rows = arr(&agreement["held_out_by_offset"])?;
    assert_eq!(rows.len(), 5);
    let mut checked = 0;
    for (panel, stat) in ["mean", "p95"].iter().enumerate() {
        let mut values = Vec::new();
        for (i, row) in rows.iter().enumerate() {
            let mut counts = [0; 2];
            for (j, policy) in [FINE, ALT].into_iter().enumerate() {
                let r = &row["scores"][name(policy)]["at_13ms"];
                counts[j] = u(&r["matched"])?;
                let s = &r["absolute_com_mm"];
                let val = match (s["selected"][stat].as_f64(), s["one"][stat].as_f64()) {
                    (Some(a), Some(b)) => Some(a - b),
                    (None, None) => None,
                    _ => return Err("asymmetrical agreement cohort".into()),
                };
                values.push((i, j, val));
            }
            assert_eq!(
                labels[&(480 * panel as i32 + 6, 172 + 46 * i as i32)],
                format!("n = {}/{}", counts[0], counts[1])
            );
        }
        let limit = 1.08
            * values
                .iter()
                .filter_map(|(_, _, v)| *v)
                .map(f64::abs)
                .fold(0.01, f64::max);
        let zero = 266 + 480 * panel as i32;
        for (i, j, val) in values {
            let y = 154 + 46 * i as i32 + 12 * j as i32;
            if let Some(v) = val {
                let (x, color) = marks
                    .remove(&(panel as i32, y))
                    .ok_or("missing scored agreement marker")?;
                assert_eq!(x, zero + (v * 160. / limit).round() as i32);
                assert_eq!(color, ["#0D7188", "#A74E28"][j]);
            } else {
                assert_eq!(labels[&(zero + 5, y)], "No data");
            }
            checked += 1;
        }
    }
    assert!(marks.is_empty());
    Ok(checked)
}
