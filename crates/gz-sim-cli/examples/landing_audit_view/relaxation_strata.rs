// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fit ordinal kinematic bins on complete training offsets, evaluate on untouched offsets.
use super::*;
use plotters::prelude::{Color, IntoDrawingArea, RGBColor, Rectangle, SVGBackend, Text, WHITE};
use plotters::style::{
    IntoTextStyle,
    text_anchor::{HPos, Pos, VPos},
};
use signature::Signature;

#[path = "relaxation_agreement.rs"]
mod agreement;

pub(super) const HELD: [u16; 5] = [175, 200, 225, 250, 275];

fn groups(s: Signature, limits: [[f64; 2]; 2]) -> usize {
    let bin = |x, bounds: [f64; 2]| bounds.partition_point(|b| x > *b);
    3 * bin(s.prospective_tangent_m_s, limits[1]) + bin(s.support_height_span_mm, limits[0])
}

fn tertiles(values: impl Iterator<Item = f64>) -> Option<[f64; 2]> {
    let mut v = values.collect::<Vec<_>>();
    v.sort_unstable_by(f64::total_cmp);
    if v.is_empty() {
        return None;
    }
    Some([1, 2].map(|k| v[(v.len() * k).div_ceil(3) - 1]))
}

fn train(cells: &[&Cell]) -> Option<bool> {
    let mut count = 0;
    let mut sum_mm = 0.;
    for d in cells.iter().filter_map(|c| c.change(CAND, FINE, 2).ok()) {
        count += 1;
        sum_mm += d;
    }
    (count != 0).then_some(sum_mm < 0.)
}

pub(super) fn evaluate(
    cells: &[&Cell],
    take_eight: impl Fn(&Cell) -> bool,
    reference: Profile,
    w: usize,
    baseline: Profile,
) -> Value {
    let mut base = Vec::new();
    let mut candidate = Vec::new();
    let mut picked = Vec::new();
    let mut n_eight = 0;
    let mut disjoint = 0;
    let mut pending = 0;
    for &c in cells {
        match (c.pair(baseline, reference, w), c.pair(CAND, reference, w)) {
            (Ok(r1), Ok(r8)) => {
                let eight = take_eight(c);
                base.push(r1.com_mm);
                candidate.push(r8.com_mm);
                picked.push(if eight { r8.com_mm } else { r1.com_mm });
                n_eight += usize::from(eight);
            }
            (Err(Missing::Disjoint), _) | (_, Err(Missing::Disjoint)) => disjoint += 1,
            _ => pending += 1,
        }
    }
    let differences = |a: &[f64], b: &[f64]| stats(a.iter().zip(b).map(|(x, y)| x - y).collect());
    json!({"matched":base.len(),"selected_eight":n_eight,"disjoint":disjoint,"default_pending":pending,
        "eight_minus_one_mm":differences(&candidate,&base),"selected_minus_one_mm":differences(&picked,&base),
        "selected_minus_eight_mm":differences(&picked,&candidate),"absolute_com_mm":{"one":stats(base),"eight":stats(candidate),"selected":stats(picked)}})
}

fn evaluations(cells: &[&Cell], choose: impl Fn(&Cell) -> bool + Copy) -> Value {
    [FINE, ALT]
        .into_iter()
        .map(|reference| {
            let intervals =
                [1, 2].map(|w| (LABELS[w], evaluate(cells, choose, reference, w, BASE)));
            (reference.label(), json!(BTreeMap::from(intervals)))
        })
        .collect::<serde_json::Map<_, _>>()
        .into()
}

pub(super) fn export(c: &Census, out: &Path) -> Result<()> {
    let classified: Vec<_> = c.cells.iter().filter(|x| x.incoming.is_some()).collect();
    let (held, training): (Vec<_>, Vec<_>) = classified
        .iter()
        .copied()
        .partition(|x| HELD.contains(&x.site.offset_mm));
    let heights = tertiles(
        training
            .iter()
            .map(|x| x.incoming.unwrap().support_height_span_mm),
    );
    let speeds = tertiles(
        training
            .iter()
            .map(|x| x.incoming.unwrap().prospective_tangent_m_s),
    );
    let baseline = train(&training);
    let covariates = |cuts: &[&Cell]| {
        let x: Vec<_> = cuts.iter().filter_map(|c| c.incoming).collect();
        json!({"height_span_mm":stats(x.iter().map(|s|s.support_height_span_mm).collect()),
            "tangent_m_s":stats(x.iter().map(|s|s.prospective_tangent_m_s).collect()),
            "supported_area_fraction":stats(x.iter().map(|s|s.supported_projected_area_fraction).collect()),
            "spin_rad_s":stats(x.iter().map(|s|s.spin_rad_s).collect()),"gap_mm":stats(x.iter().map(|s|s.recorded_gap_mm).collect()),
            "below_0_1_mm_s":x.iter().filter(|s|s.prospective_tangent_m_s<0.0001).count()})
    };
    let mut result = json!({"format":"locked-sole-kinematics-v1","held_out_offsets_mm":HELD,
        "training_fixtures":c.completed.iter().filter(|i|!HELD.contains(&(165+5*((**i*613%1250)/50) as u16))).count(),
        "held_out_fixtures":c.completed.iter().filter(|i|HELD.contains(&(165+5*((**i*613%1250)/50) as u16))).count(),
        "native_cuts":c.cells.len(),"without_supported_sole_polygon":c.cells.len()-classified.len(),"training_classified":training.len(),
        "held_out_classified":held.len(),"span_mm_training_tertiles":heights,"tangent_m_s_training_tertiles":speeds,
        "default_to_eight_in_empty_bin":baseline,"covariates":{"training":covariates(&training),"held_out":covariates(&held)},
        "input":"native unloaded donor COM, velocity, foot quaternion, angular velocity, locked sole geometry, finite pad XY; measured historical positive contact velocities are unavailable",
        "training":"nearest-rank training tertiles, edge belongs to lower bin; choose r8 when its mean 13 ms COM error against 4096/r8 is smaller in that bin; use global training sign for empty bins",
        "endpoint":"post-cut 5 or 13 ms since matched airborne donor; compare only windows present and overlapped in both candidates and reference",
        "status":"insufficient training cuts"});
    if let (Some(h), Some(v), Some(fallback)) = (heights, speeds, baseline) {
        let edges = [h, v];
        let partition = |x: &&Cell| groups(x.incoming.unwrap(), edges);
        let train_bins: [Vec<_>; 9] = std::array::from_fn(|k| {
            training
                .iter()
                .copied()
                .filter(|x| partition(x) == k)
                .collect()
        });
        let test_bins: [Vec<_>; 9] =
            std::array::from_fn(|k| held.iter().copied().filter(|x| partition(x) == k).collect());
        let choices = train_bins
            .each_ref()
            .map(|span| train(span).unwrap_or(fallback));
        let choose = |x: &Cell| choices[groups(x.incoming.unwrap(), edges)];
        result["strata"]=json!((0..9).map(|k|{
            let tr=&train_bins[k]; let test=&test_bins[k];
            json!({"slip_bin":k/3,"span_bin":k%3,"selected_relaxation":if choices[k]{8}else{1},
                "uses_fallback":train(tr).is_none(),"training_cuts":tr.len(),"held_out_cuts":test.len(),
                "training":evaluations(tr,choose),"held_out":evaluations(test,choose)})
        }).collect::<Vec<_>>());
        // A reduced-feature control tests whether ranking near-zero slip contributes.
        let by_span: [bool; 3] = std::array::from_fn(|bin| {
            let rows = training
                .iter()
                .copied()
                .filter(|c| partition(c) % 3 == bin)
                .collect::<Vec<_>>();
            train(&rows).unwrap_or(fallback)
        });
        let choose_span = |c: &Cell| by_span[groups(c.incoming.unwrap(), edges) % 3];
        result["span_only_control"] = json!({"selected_eight_by_span":by_span,"training":evaluations(&training,choose_span),"held_out":evaluations(&held,choose_span)});
        result["reference_agreement"] =
            agreement::export(&train_bins, &training, &held, edges, out)?;
        result["training"] = evaluations(&training, choose);
        result["held_out"] = evaluations(&held, choose);
        result["status"] = json!("fitted");
        plot(&out.join("relaxation-signatures.svg"), &result)?;
    }
    fs::write(
        out.join("relaxation-signatures.json"),
        serde_json::to_vec_pretty(&result)?,
    )?;
    Ok(())
}

fn plot(path: &Path, data: &Value) -> Result<()> {
    let root = SVGBackend::new(path, (1080, 562)).into_drawing_area();
    root.fill(&WHITE)?;
    let body = root.titled(
        "When does eight-pass relaxation improve the 13 ms trajectory?",
        ("sans-serif", 24),
    )?;
    let (head, panels) = body.split_vertically(92);
    let th = |key: &str, i: usize| data[key][i].as_f64().unwrap();
    head.draw(&Text::new("Each cell shows the mean change in COM distance from the 4,096-call, eight-pass reference.",(24,16),("sans-serif",16)))?;
    head.draw(&Text::new(
        format!(
            "Columns group supported sole height span at {:.4} and {:.4} mm",
            th("span_mm_training_tertiles", 0),
            th("span_mm_training_tertiles", 1)
        ),
        (24, 41),
        ("sans-serif", 15),
    ))?;
    head.draw(&Text::new(
        format!(
            "Rows group prospective tangent speed at {:.3e} and {:.3e} m/s",
            th("tangent_m_s_training_tertiles", 0),
            th("tangent_m_s_training_tertiles", 1)
        ),
        (24, 65),
        ("sans-serif", 15),
    ))?;
    for (panel, (split, label)) in panels.split_evenly((1, 2)).iter().zip([
        ("training", "Training offsets"),
        ("held_out", "Held-out offsets"),
    ]) {
        let center = |font| {
            ("sans-serif", font)
                .into_text_style(panel)
                .pos(Pos::new(HPos::Center, VPos::Center))
        };
        let n = data[split]["4096x4_r8"]["at_13ms"]["matched"]
            .as_u64()
            .unwrap();
        panel.draw(&Text::new(
            format!("{label} · {n} matched"),
            (270, 23),
            center(19),
        ))?;
        for (r, row_name) in ["Low", "Middle", "High"].iter().enumerate() {
            let y = 76 + r as i32 * 98;
            panel.draw(&Text::new(*row_name, (34, y + 34), center(14)))?;
            for (col, col_name) in ["Low", "Middle", "High"].iter().enumerate() {
                let x = 69 + col as i32 * 150;
                if r == 0 {
                    panel.draw(&Text::new(*col_name, (x + 70, y - 18), center(14)))?;
                }
                let stats = &data["strata"][r * 3 + col][split]["4096x4_r8"]["at_13ms"]["eight_minus_one_mm"];
                let mean = stats["mean"].as_f64();
                let count = stats["n"].as_u64().unwrap();
                let color = match mean {
                    Some(m) if m < 0. => RGBColor(183, 220, 214),
                    Some(_) => RGBColor(246, 205, 181),
                    None => RGBColor(220, 222, 222),
                };
                panel.draw(&Rectangle::new([(x, y), (x + 140, y + 86)], color.filled()))?;
                panel.draw(&Text::new(
                    mean.map_or_else(|| "No data".to_string(), |m| format!("{m:+.3} mm")),
                    (x + 70, y + 31),
                    center(18),
                ))?;
                panel.draw(&Text::new(
                    format!("{count} landing{}", if count == 1 { "" } else { "s" }),
                    (x + 70, y + 59),
                    center(14),
                ))?;
            }
        }
    }
    root.present()?;
    Ok(())
}
