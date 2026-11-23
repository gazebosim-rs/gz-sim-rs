// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A conservative offline control fitted to paired training results from both empirical grids.
use super::*;
use plotters::prelude::{Circle, PathElement};

#[derive(Serialize)]
struct Decision {
    matched: usize,
    mean_eight_minus_one_mm: [Option<f64>; 2],
    take_eight: bool,
}

fn fit(bin: &[&Cell]) -> Decision {
    let mut matched = 0;
    let mut sums = [0.; 2];
    for cell in bin {
        if let [Ok(current), Ok(original)] = [FINE, ALT].map(|r| cell.change(CAND, r, 2)) {
            matched += 1;
            sums[0] += current;
            sums[1] += original;
        }
    }
    Decision {
        matched,
        mean_eight_minus_one_mm: sums.map(|sum| (matched > 0).then(|| sum / matched as f64)),
        take_eight: matched > 0 && sums.iter().all(|s| *s < 0.),
    }
}

pub(super) fn export(
    bins: &[Vec<&Cell>; 9],
    training: &[&Cell],
    held: &[&Cell],
    edges: [[f64; 2]; 2],
    out: &Path,
) -> Result<Value> {
    let decisions = bins.each_ref().map(|b| fit(b));
    let choose = |c: &Cell| decisions[groups(c.incoming.unwrap(), edges)].take_eight;
    let offsets = HELD.map(|offset| {
        let cells: Vec<_> = held
            .iter()
            .copied()
            .filter(|c| c.site.offset_mm == offset)
            .collect();
        json!({"offset_mm":offset,"classified":cells.len(),"scores":evaluations(&cells,choose)})
    });
    let data = json!({
        "method":"Use eight only when it reduces mean 13 ms COM error against each empirical reference on exactly the same training cuts in the kinematic bin. Unobserved, tied or disagreeing bins use one.",
        "training_references":[FINE.label(), ALT.label()],"bins":decisions,
        "training":evaluations(training,choose),"held_out":evaluations(held,choose),
        "held_out_by_offset":offsets,
    });
    plot(&out.join("relaxation-reference-agreement.svg"), &offsets)?;
    Ok(data)
}

fn plot(path: &Path, rows: &[Value; 5]) -> Result<()> {
    let root = SVGBackend::new(path, (960, 430)).into_drawing_area();
    root.fill(&WHITE)?;
    let body = root.titled(
        "Requiring both empirical references to favor extra relaxation",
        ("sans-serif", 23),
    )?;
    body.draw(&Text::new("Change in held-out 13 ms COM error relative to always using one pass (mm). Negative values show improvement.", (26,17), ("sans-serif", 16)))?;
    let colors = [RGBColor(13, 113, 136), RGBColor(167, 78, 40)];
    for (i, (profile, x)) in [(FINE, 28), (ALT, 316)].into_iter().enumerate() {
        body.draw(&Circle::new((x + 5, 42), 5, colors[i].filled()))?;
        body.draw(&Text::new(
            format!(
                "Reference · 4,096 calls, {} {}",
                profile.relaxation,
                if profile.relaxation == 1 {
                    "pass"
                } else {
                    "passes"
                }
            ),
            (x + 18, 49),
            ("sans-serif", 15),
        ))?;
    }
    let (_, panels) = body.split_vertically(66);
    for (panel, (stat, label)) in panels.split_evenly((1, 2)).iter().zip([
        ("mean", "Mean difference"),
        ("p95", "Difference between p95 values"),
    ]) {
        let values = rows.each_ref().map(|row| {
            [FINE, ALT].map(|r| {
                let scores = &row["scores"][r.label()]["at_13ms"]["absolute_com_mm"];
                Some(scores["selected"][stat].as_f64()? - scores["one"][stat].as_f64()?)
            })
        });
        let max = values
            .iter()
            .flatten()
            .flatten()
            .map(|v| v.abs())
            .fold(0.01, f64::max)
            * 1.08;
        let x_value = |v: f64| 266 + (v / max * 160.).round() as i32;
        let centre = |font| {
            ("sans-serif", font)
                .into_text_style(panel)
                .pos(Pos::new(HPos::Center, VPos::Center))
        };
        panel.draw(&Text::new(label, (246, 16), centre(18)))?;
        for v in [-max, 0., max] {
            let x = x_value(v);
            panel.draw(&PathElement::new(
                [(x, 42), (x, 265)],
                RGBColor(209, 215, 217),
            ))?;
            panel.draw(&Text::new(format!("{v:+.3}"), (x, 289), centre(14)))?;
        }
        for (i, (row, pair)) in rows.iter().zip(values).enumerate() {
            let y = 59 + i as i32 * 46;
            panel.draw(&Text::new(
                format!("{} mm", row["offset_mm"]),
                (6, y + 3),
                ("sans-serif", 15),
            ))?;
            let counts = [FINE, ALT].map(|r| {
                row["scores"][r.label()]["at_13ms"]["matched"]
                    .as_u64()
                    .unwrap()
            });
            panel.draw(&Text::new(
                format!("n = {}/{}", counts[0], counts[1]),
                (6, y + 18),
                ("sans-serif", 12),
            ))?;
            for (j, value) in pair.into_iter().enumerate() {
                let y = y + 12 * j as i32;
                if let Some(v) = value {
                    panel.draw(&PathElement::new(
                        [(x_value(0.), y), (x_value(v), y)],
                        colors[j].stroke_width(3),
                    ))?;
                    panel.draw(&Circle::new((x_value(v), y), 4, colors[j].filled()))?;
                } else {
                    panel.draw(&Text::new(
                        "No data",
                        (x_value(0.) + 5, y),
                        ("sans-serif", 11),
                    ))?;
                }
            }
        }
    }
    root.present()?;
    Ok(())
}
