// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Choose the subsequent relaxation budget using only a verified common contact interval.
use super::*;
use std::cmp::Ordering;

const STOP: Profile = Profile::onset(8);
const SITES: [f64; 3] = [1.5, 2.5, 3.5];
const BINS: usize = 16;

#[derive(Clone, Copy, Serialize)]
pub(super) enum Receipt {
    Disjoint,
    Unloaded,
    Loaded {
        effective_quadrants: f64,
        residual_m_s: f64,
        traction_ratio: f64,
    },
}

pub(super) fn shared_receipt(paths: &[RawPath], cut: u64) -> Result<Receipt> {
    let mut pair = Vec::with_capacity(2);
    for profile in [STOP, CAND] {
        let path = paths
            .iter()
            .find(|p| p.profile == profile)
            .ok_or("missing shared-prefix treatment")?;
        let first = path
            .samples
            .iter()
            .find(|s| s.i == 1)
            .ok_or("no first-interval measurement")?;
        let onset = path.first.filter(|r| r.end_ns <= 500_000);
        let disjoint = path.first_disjoint_tick.is_some_and(|t| t <= cut + 1);
        pair.push((first, onset, disjoint));
    }
    if pair[0] != pair[1] {
        return Err("treatments disagree before the continuation decision".into());
    }
    let (s, onset, disjoint) = pair[0];
    let normal = s.normal_ns;
    let points = s
        .positive_point_count
        .ok_or("missing native point census")?;
    if onset.is_some() != (normal > 0.)
        || onset.is_some() != (points > 0)
        || onset.is_some() != s.impulse_weighted_abs_post_normal_m_s.is_some()
        || normal != s.cumulative_normal_ns
        || normal < 0.
        || !normal.is_finite()
        || s.cumulative_corners_ns
            .iter()
            .any(|p| !p.is_finite() || *p < 0.)
        || (s.cumulative_corners_ns.iter().sum::<f64>() - normal).abs() > 1e-10 * (1. + normal)
        || !DVec3::from(s.cumulative_tangent_ns).is_finite()
        || s.max_post_closing_m_s
            .is_none_or(|v| !v.is_finite() || v < 0.)
        || s.impulse_weighted_abs_post_normal_m_s
            .is_some_and(|v| !v.is_finite() || v < 0.)
    {
        return Err("invalid common native contact receipt".into());
    }
    if disjoint {
        return Ok(Receipt::Disjoint);
    }
    if s.footprint_gap_m.is_none_or(|g| !g.is_finite()) {
        return Err("missing first-interval finite footprint".into());
    }
    if normal == 0. {
        return Ok(Receipt::Unloaded);
    }
    let fractions = s.cumulative_corners_ns.map(|v| v / normal);
    let effective_quadrants = 1. / fractions.iter().map(|f| f * f).sum::<f64>();
    if !(1. - 1e-9..=4. + 1e-9).contains(&effective_quadrants) {
        return Err("invalid effective quadrant count".into());
    }
    Ok(Receipt::Loaded {
        effective_quadrants,
        residual_m_s: s.impulse_weighted_abs_post_normal_m_s.unwrap(),
        traction_ratio: DVec3::from(s.cumulative_tangent_ns).length() / normal,
    })
}

fn median(values: impl Iterator<Item = f64>) -> Option<f64> {
    let mut v: Vec<_> = values.collect();
    v.sort_unstable_by(f64::total_cmp);
    (!v.is_empty()).then(|| v[(v.len() - 1) / 2])
}

fn bucket(receipt: Receipt, limits: [Option<f64>; 2]) -> Option<usize> {
    let Receipt::Loaded {
        effective_quadrants,
        residual_m_s,
        traction_ratio,
    } = receipt
    else {
        return None;
    };
    let sites = SITES.partition_point(|k| effective_quadrants > *k);
    Some(
        sites
            + 4 * usize::from(residual_m_s > limits[0]?)
            + 8 * usize::from(traction_ratio > limits[1]?),
    )
}

fn effects(c: &Cell, reference: Profile, w: usize) -> Option<f64> {
    Some(c.pair(CAND, reference, w).ok()?.com_mm - c.pair(STOP, reference, w).ok()?.com_mm)
}
fn scores(cells: &[&Cell], choose: impl Fn(&Cell) -> bool + Copy) -> Value {
    [FINE, ALT]
        .into_iter()
        .map(|r| {
            let s = [1, 2].map(|w| (LABELS[w], strata::evaluate(cells, choose, r, w, STOP)));
            (r.label(), json!(BTreeMap::from(s)))
        })
        .collect::<serde_json::Map<_, _>>()
        .into()
}

pub(super) fn export(c: &Census, out: &Path) -> Result<()> {
    let (held, train): (Vec<_>, Vec<_>) = c
        .cells
        .iter()
        .partition(|p| strata::HELD.contains(&p.site.offset_mm));
    let limits = [false, true].map(|traction| {
        median(train.iter().filter_map(|c| match c.first_receipt {
            Receipt::Loaded {
                residual_m_s,
                traction_ratio,
                ..
            } => Some(if traction {
                traction_ratio
            } else {
                residual_m_s
            }),
            _ => None,
        }))
    });
    let mut decisions = [false; BINS];
    let mut bins = vec![];
    for (k, decision) in decisions.iter_mut().enumerate() {
        let training: Vec<_> = train
            .iter()
            .copied()
            .filter(|c| bucket(c.first_receipt, limits) == Some(k))
            .collect();
        let held_bin: Vec<_> = held
            .iter()
            .copied()
            .filter(|c| bucket(c.first_receipt, limits) == Some(k))
            .collect();
        let mut totals = [0.; 2];
        let mut joined = 0;
        for c in &training {
            if let [Some(a), Some(b)] = [FINE, ALT].map(|r| effects(c, r, 2)) {
                joined += 1;
                totals[0] += a;
                totals[1] += b;
            }
        }
        *decision = joined > 0 && totals.iter().all(|v| *v < 0.);
        bins.push(json!({"quadrant_bin":k%4,"high_residual":(k/4)%2!=0,"high_traction":k>=8,
            "comparable_training_cuts":joined,"training_cuts":training.len(),"held_out_cuts":held_bin.len(),
            "training_mean_continued_minus_stopped_mm":totals.map(|s|(joined>0).then(||s/joined as f64)),
            "continue_eight":decision,"training":scores(&training,|_|*decision),"held_out":scores(&held_bin,|_|*decision)}));
    }
    let choose = |c: &Cell| bucket(c.first_receipt, limits).is_some_and(|k| decisions[k]);
    let mut rows = BufWriter::new(fs::File::create(
        out.join("relaxation-contact-receipts.jsonl"),
    )?);
    for c in &c.cells {
        serde_json::to_writer(
            &mut rows,
            &json!({"site":c.site,"common_first_receipt":c.first_receipt,"bucket":bucket(c.first_receipt,limits),"continue_eight":choose(c)}),
        )?;
        rows.write_all(b"\n")?;
    }
    rows.flush()?;
    let split = |rows: &[&Cell]| {
        let loaded = rows
            .iter()
            .filter(|c| matches!(c.first_receipt, Receipt::Loaded { .. }))
            .count();
        let zero = rows
            .iter()
            .filter(|c| matches!(c.first_receipt, Receipt::Unloaded))
            .count();
        json!({"verified":rows.len(),"loaded":loaded,"unloaded":zero,"first_interval_disjoint":rows.len()-zero-loaded,"scores":scores(rows,choose)})
    };
    let winner = held
        .iter()
        .copied()
        .filter_map(|c| match c.first_receipt {
            Receipt::Loaded { .. } => Some((c, effects(c, FINE, 2)?)),
            _ => None,
        })
        .max_by(|(a, ax), (b, bx)| {
            ax.total_cmp(bx)
                .then_with(|| (b.site.index, b.site.stroke).cmp(&(a.site.index, a.site.stroke)))
        });
    let report = json!({"format":"first-contact-continuation-v1","scheduled_fixtures":c.scheduled,"completed_fixtures":c.completed.len(),
        "pending_fixtures":c.scheduled-c.completed.len(),"joined_default_fixtures":c.matched_default.len(),
        "missing_donor_strokes":3*c.completed.len()-c.cells.len(),"each_cut_first_receipt_exactly_equal":c.cells.len(),
        "held_offsets_mm":strata::HELD,"effective_quadrants_edges":SITES,"training_median_residual_m_s":limits[0],"training_median_traction_ratio":limits[1],
        "reference_order":[FINE.label(),ALT.label()],"bins":bins,"training":split(&train),"held_out":split(&held),
        "held_by_offset":strata::HELD.map(|offset|{let rows:Vec<_>=held.iter().copied().filter(|c|c.site.offset_mm==offset).collect();json!({"offset_mm":offset,"evaluation":split(&rows)})}),
        "outlier":winner.map(|(c,delta)|json!({"site":c.site,"continued_minus_stopped_mm":delta,"receipt":c.first_receipt,"selected_eight":choose(c)})),
        "selection_final":c.completed.len()==c.scheduled,
        "score_labels":{"one":"one relaxation pass after the shared first eight-pass interval","eight":"continue eight after the shared interval","selected":"training-bin decision after common receipt"}});
    fs::write(
        out.join("relaxation-continuation.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    plot(&report, &out.join("relaxation-continuation.svg"))
}

fn plot(data: &Value, path: &Path) -> Result<()> {
    use plotters::prelude::{Color, IntoDrawingArea, RGBColor, Rectangle, SVGBackend, Text, WHITE};
    use plotters::style::{
        IntoTextStyle,
        text_anchor::{HPos, Pos, VPos},
    };
    let root = SVGBackend::new(path, (1100, 628)).into_drawing_area();
    root.fill(&WHITE)?;
    let body = root.titled(
        "Should relaxation continue after the measured touchdown?",
        ("sans-serif", 24),
    )?;
    body.draw(&Text::new("Held-out mean change in 13 ms COM error when continuing eight passes. Negative favors continuing.",(25,20),("sans-serif",17)))?;
    let fmt = |k: &str| {
        data[k]
            .as_f64()
            .map_or_else(|| "no data".into(), |x| format!("{x:.5}"))
    };
    body.draw(&Text::new(
        format!(
            "Training medians: residual {} m/s, net tangential / normal impulse {}.",
            fmt("training_median_residual_m_s"),
            fmt("training_median_traction_ratio")
        ),
        (25, 50),
        ("sans-serif", 15),
    ))?;
    let (_, panels) = body.split_vertically(76);
    for (panel, r) in panels.split_evenly((1, 2)).iter().zip([FINE, ALT]) {
        let center = |n| {
            ("sans-serif", n)
                .into_text_style(panel)
                .pos(Pos::new(HPos::Center, VPos::Center))
        };
        let label = if r == FINE {
            "eight passes"
        } else {
            "one pass"
        };
        panel.draw(&Text::new(
            format!("4,096 native calls · {label}"),
            (275, 18),
            center(19),
        ))?;
        panel.draw(&Text::new(
            "Effective loaded quadrants",
            (330, 54),
            center(15),
        ))?;
        for (k, row) in data["bins"].as_array().unwrap().iter().enumerate() {
            let col = k % 4;
            let band = k / 4;
            let x = 111 + 104 * col as i32;
            let y = 100 + 101 * band as i32;
            if band == 0 {
                panel.draw(&Text::new(
                    ["≤ 1.5", "1.5–2.5", "2.5–3.5", "> 3.5"][col],
                    (x + 49, y - 15),
                    center(14),
                ))?;
            }
            if col == 0 {
                let t = if band >= 2 {
                    "High traction"
                } else {
                    "Low traction"
                };
                let n = if band % 2 == 1 {
                    "High residual"
                } else {
                    "Low residual"
                };
                panel.draw(&Text::new(t, (54, y + 31), center(13)))?;
                panel.draw(&Text::new(n, (54, y + 51), center(13)))?;
            }
            let stat = &row["held_out"][r.label()]["at_13ms"]["eight_minus_one_mm"];
            let mean = stat["mean"].as_f64();
            let n = stat["n"].as_u64().unwrap();
            let color = match mean.map(|v| v.total_cmp(&0.)) {
                Some(Ordering::Less) => RGBColor(180, 221, 216),
                Some(Ordering::Greater) => RGBColor(247, 209, 185),
                _ => RGBColor(220, 224, 224),
            };
            panel.draw(&Rectangle::new([(x, y), (x + 98, y + 85)], color.filled()))?;
            panel.draw(&Text::new(
                mean.map_or_else(|| "No data".into(), |v| format!("{v:+.3} mm")),
                (x + 49, y + 29),
                center(15),
            ))?;
            panel.draw(&Text::new(format!("n = {n}"), (x + 49, y + 56), center(13)))?;
        }
    }
    root.present()?;
    Ok(())
}
