// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Plot the measured discrepancy and separate inherited motion from post-cut integration.
use super::*;

pub(super) fn captures(rows: &[&[Sample]]) -> Result<serde_json::Value> {
    let first = &rows[0][0];
    let primary = rows[0]
        .iter()
        .skip(1)
        .find(|s| s.normal_impulse_ns > 0.)
        .ok_or("no adaptive landing in the selected interval")?;
    let mut marks = json!({"primary":primary.time_us-first.time_us});
    if let Some((s, _)) = rows[0]
        .iter()
        .zip(rows[2])
        .find(|(s, r)| com_error_um(s, r) >= 1000.)
    {
        marks["one_mm"] = json!(s.time_us - first.time_us);
    }
    Ok(marks)
}

pub(super) fn plot(path: &Path, rows: &[&[Sample]], peak: f64, error_um: f64) -> Result<()> {
    let root = SVGBackend::new(path, (1020, 910)).into_drawing_area();
    root.fill(&WHITE)?;
    let panels = root
        .titled("Outer support · second primary landing", ("sans-serif", 27))?
        .split_evenly((3, 1));
    let lo = rows[0][0].time_us as f64 * 0.001;
    let hi = rows[0].last().ok_or("missing final frame")?.time_us as f64 * 0.001;
    let post_cut_um = rows[0]
        .iter()
        .zip(rows[1])
        .map(|(a, b)| com_error_um(a, b))
        .fold(1e-6_f64, f64::max);
    for (panel, title, unit, limit) in [
        (
            0,
            "Normal reaction at the support",
            "Impulse per 500 µs tick (N·s)",
            peak,
        ),
        (
            1,
            "Separation from the continuous fine trajectory",
            "Distance (mm)",
            error_um * 0.001,
        ),
        (
            2,
            "Separation generated after the matched cut",
            "Distance (µm)",
            post_cut_um,
        ),
    ] {
        let mut chart = ChartBuilder::on(&panels[panel])
            .margin(12)
            .caption(title, ("sans-serif", 20))
            .x_label_area_size(42)
            .y_label_area_size(83)
            .build_cartesian_2d(lo..hi, 0.0..1.16 * limit)?;
        chart
            .configure_mesh()
            .x_desc("Simulation time after settling (ms)")
            .y_desc(unit)
            .max_light_lines(1)
            .draw()?;
        let names = if panel == 0 {
            ["Adaptive", "Fine from adaptive cut", "Continuous fine"]
        } else {
            [
                "Adaptive vs continuous fine",
                "Fine from cut vs continuous fine",
                "Adaptive vs fine from cut",
            ]
        };
        let curves: &[usize] = match panel {
            0 => &[2, 1, 0],
            1 => &[1, 0],
            _ => &[2],
        };
        for &k in curves {
            let pts = rows[k].iter().enumerate().map(|(i, s)| {
                let y = if panel == 0 {
                    s.normal_impulse_ns
                } else {
                    let (a, b) = [(0, 2), (1, 2), (0, 1)][k];
                    com_error_um(&rows[a][i], &rows[b][i]) * if panel == 1 { 0.001 } else { 1. }
                };
                (s.time_us as f64 * 0.001, y)
            });
            let color = INKS[k];
            chart
                .draw_series(LineSeries::new(pts, color.stroke_width(k as u32 + 1)))?
                .label(names[k])
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 21, y)], color.stroke_width(3))
                });
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperMiddle)
            .background_style(WHITE.mix(0.94))
            .draw()?;
    }
    root.present()?;
    Ok(())
}
