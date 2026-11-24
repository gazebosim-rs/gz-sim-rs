// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Render paired finite-box return measurements with their actual denominators.
use super::*;
const INK: RGBColor = RGBColor(33, 49, 61);
const CAL: RGBColor = RGBColor(174, 108, 46);
const HOLD: RGBColor = RGBColor(27, 119, 123);
fn label(id: &str) -> &str {
    match id {
        "unmodified" => "Native",
        "every" => "Every query",
        "first_unloaded" => "First unloaded",
        "first_stale" => "Fixed threshold",
        "while_stale" => "Repeat fixed",
        "first_travel" => "Surface travel",
        _ => id,
    }
}
pub(super) fn field(out: &Path, v: &Value) -> Result<()> {
    let path = out.join("geometry-reaction.svg");
    let root = SVGBackend::new(&path, (1290, 830)).into_drawing_area();
    root.fill(&WHITE)?;
    root.draw(&Text::new(
        "Native geometry at the second support landing",
        (28, 41),
        ("sans-serif", 29).into_font().color(&INK),
    ))?;
    let cal = i(&v["calibration"]["measured"])?;
    let held = i(&v["holdout"]["measured"])?;
    root.draw(&Text::new(format!("8 native advances × 8 relaxation passes  ·  {cal} calibration and {held} held-out returns  ·  one 500 µs actuator interval"),(29,72),("sans-serif",17).into_font().color(&INK)))?;
    for (x, name, c) in [(32, "Calibration", CAL), (214, "Held out", HOLD)] {
        root.draw(&Rectangle::new([(x, 89), (x + 18, 102)], c.filled()))?;
        root.draw(&Text::new(
            name,
            (x + 26, 102),
            ("sans-serif", 15).into_font().color(&INK),
        ))?;
    }
    let (_, body) = root.split_vertically(123);
    let panels = [
        (
            "max_normal_angle_mrad",
            "First loaded normal · p95 absolute error",
            "mrad",
            true,
        ),
        (
            "history_spread_ns",
            "Retained versus rebuilt · p95 impulse difference",
            "N·s",
            true,
        ),
        (
            "max_to_all_eight_ns",
            "Largest distance to eight fine controls · p95",
            "N·s",
            true,
        ),
        (
            "mean_query_count",
            "Mean native manifold requeries per graph",
            "queries",
            false,
        ),
    ];
    for ((key, title, unit, is_rank), area) in panels.into_iter().zip(body.split_evenly((2, 2))) {
        area.draw(&Text::new(
            title,
            (22, 27),
            ("sans-serif", 19).into_font().color(&INK),
        ))?;
        let mut by_method = BTreeMap::new();
        for (cohort, col) in [("calibration", 0), ("holdout", 1)] {
            for r in cases(&v[cohort]["cells"])? {
                if r["calls"] != 8 || r["passes"] != 8 {
                    continue;
                }
                let name = r["method"].as_str().ok_or("missing method")?;
                let data = by_method.entry(name).or_insert([None; 2]);
                data[col] = if is_rank {
                    r[key]["p95"].as_f64()
                } else {
                    r[key].as_f64()
                };
            }
        }
        let peak = by_method
            .values()
            .flatten()
            .flatten()
            .copied()
            .fold(0f64, f64::max)
            .max(1e-12);
        let width = (area.dim_in_pixel().0 as i32) - 260;
        for (j, &m) in METHODS
            .iter()
            .filter(|&&m| by_method.contains_key(m))
            .enumerate()
        {
            let y = 47 + (j as i32) * 43;
            area.draw(&Text::new(
                label(m),
                (22, y + 14),
                ("sans-serif", 14).into_font().color(&INK),
            ))?;
            for (series, color) in [(0, CAL), (1, HOLD)] {
                let row = y + series as i32 * 12;
                let mut end = 155;
                let text = if let Some(value) = by_method[m][series] {
                    end += (width as f64 * value / peak).round() as i32;
                    area.draw(&Rectangle::new(
                        [(155, row), (end, row + 8)],
                        color.filled(),
                    ))?;
                    if value != 0. && value < 1e-4 {
                        format!("{value:.1e}")
                    } else if value < 1. {
                        format!("{value:.4}")
                    } else {
                        format!("{value:.3}")
                    }
                } else {
                    "pending".into()
                };
                area.draw(&Text::new(
                    text,
                    (end + 5, row + 8),
                    ("sans-serif", 11).into_font().color(&INK),
                ))?;
            }
        }
        let bottom = area.dim_in_pixel().1 as i32 - 22;
        area.draw(&Text::new(
            format!("Linear scale  0 – {peak:.4} {unit}"),
            (155, bottom),
            ("sans-serif", 13).into_font().color(&INK),
        ))?;
    }
    root.present()?;
    Ok(())
}
