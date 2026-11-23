// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measured and closed-form translation and spin for the authored three-sphere slide.
use super::*;
use plotters::prelude::*;

#[derive(Deserialize)]
struct Study {
    traces: Vec<FrameState>,
}
#[derive(Deserialize)]
struct FrameState {
    time_ns: u64,
    states: [State; 3],
    analytic_x_v_w: [[f64; 3]; 3],
}
#[derive(Deserialize)]
struct State {
    p: [f64; 3],
    v: [f64; 3],
    w: [f64; 3],
}
const COLORS: [RGBColor; 3] = [
    RGBColor(34, 125, 180),
    RGBColor(193, 107, 24),
    RGBColor(7, 132, 78),
];
const NAMES: [&str; 3] = ["Frictionless", "μ = 0.2", "μ = 0.6"];
pub(super) fn export(dir: &Path, out: &Path) -> Result<()> {
    let s: Study = read(&dir.join("study.json"))?;
    let Some((first, rest)) = s.traces.split_first() else {
        return Err("empty material measurement".into());
    };
    if first.time_ns != 0
        || rest.is_empty()
        || s.traces.windows(2).any(|r| r[0].time_ns >= r[1].time_ns)
    {
        return Err("invalid material clock".into());
    }
    let ms = |r: &FrameState| r.time_ns as f64 / 1e6;
    let end = ms(s.traces.last().unwrap());
    fs::create_dir_all(out)?;
    let path = out.join("material.svg");
    let root = SVGBackend::new(&path, (1030, 958)).into_drawing_area();
    root.fill(&WHITE)?;
    let root = root.titled(
        "Sliding and rolling on an authored finite deck",
        ("sans-serif", 26),
    )?;
    root.draw(&Text::new(
        "Lines are native samples. Open circles mark the analytical trajectory every 50 ms.",
        (36, 16),
        ("sans-serif", 15),
    ))?;
    let areas = root.margin(27, 5, 0, 0).split_evenly((3, 1));
    for (i, label, unit) in [
        (0, "Horizontal center speed", "Speed (m/s)"),
        (1, "Transverse angular velocity", "Spin about y (rad/s)"),
        (
            2,
            "Horizontal position error",
            "Native minus analytical x (µm)",
        ),
    ] {
        let measured = |r: &FrameState, k: usize| match i {
            0 => r.states[k].v[0],
            1 => r.states[k].w[1],
            _ => (r.states[k].p[0] - r.analytic_x_v_w[k][0]) * 1e6,
        };
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for r in &s.traces {
            for k in 0..3 {
                let v = measured(r, k);
                if !v.is_finite() {
                    return Err("invalid sliding measurement".into());
                }
                lo = lo.min(v);
                hi = hi.max(v);
            }
        }
        let expand = (hi - lo).max(1e-8) * 0.12;
        let mut chart = ChartBuilder::on(&areas[i])
            .caption(label, ("sans-serif", 18))
            .margin(11)
            .margin_right(27)
            .x_label_area_size(37)
            .y_label_area_size(73)
            .build_cartesian_2d(0.0..end, lo - expand..hi + expand)?;
        chart
            .configure_mesh()
            .x_desc("Simulation time (ms)")
            .y_desc(unit)
            .max_light_lines(1)
            .draw()?;
        for (k, color) in COLORS.into_iter().enumerate() {
            chart
                .draw_series(LineSeries::new(
                    s.traces.iter().map(|r| (ms(r), measured(r, k))),
                    color.stroke_width(2),
                ))?
                .label(NAMES[k])
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 21, y)], color.stroke_width(3))
                });
            if i != 2 {
                chart.draw_series(
                    s.traces
                        .iter()
                        .filter(|r| r.time_ns.is_multiple_of(50_000_000))
                        .map(|r| {
                            Circle::new(
                                (ms(r), r.analytic_x_v_w[k][i + 1]),
                                4,
                                color.stroke_width(2),
                            )
                        }),
                )?;
            }
        }
        if i == 2 {
            chart
                .configure_series_labels()
                .position(SeriesLabelPosition::LowerLeft)
                .background_style(WHITE.mix(0.92))
                .draw()?;
        }
    }
    root.present()?;
    Ok(())
}
