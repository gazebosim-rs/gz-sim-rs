// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use plotters::prelude::*;

pub(super) struct Series {
    fixture: Fixture,
    // Seconds, filtered jaw drag in mN, obstacle reaction in mN.
    points: Vec<(f64, f64, f64)>,
}
impl Series {
    pub(super) fn from_run(fixture: Fixture, rows: &[Log]) -> Option<Self> {
        if !matches!(fixture, Fixture::Clear | Fixture::Snag | Fixture::Rub) {
            return None;
        }
        let points = rows
            .iter()
            .filter(|r| (6970..=7410).contains(&r.tick))
            .map(|r| {
                (
                    r.tick as f64 * DT,
                    r.signal.drag_n * 1000.,
                    r.truth.bar * 1000.,
                )
            })
            .collect();
        Some(Self { fixture, points })
    }
}

pub(super) fn write(path: &std::path::Path, series: &[Series]) -> Result<()> {
    let area = SVGBackend::new(path, (900, 530)).into_drawing_area();
    area.fill(&WHITE)?;
    let panels = area.split_evenly((2, 1));
    // The bottom panel uses an independent material/obstacle observation.
    for (i, panel) in panels.iter().enumerate() {
        let mut lower = 0.0_f64;
        let mut upper = 40.0_f64;
        for (_, jaw, obstacle) in series.iter().flat_map(|s| &s.points) {
            let value = if i == 0 { *jaw } else { *obstacle };
            lower = lower.min(value);
            upper = upper.max(value);
        }
        let force = (lower * 1.10)..(upper * 1.10);
        let mut chart = ChartBuilder::on(panel)
            .margin(12)
            .x_label_area_size(36)
            .y_label_area_size(62)
            .caption(
                if i == 0 {
                    "Drag sensed at the jaws"
                } else {
                    "Independent force on the obstacle"
                },
                ("sans-serif", 20),
            )
            .build_cartesian_2d(6.97..7.41, force)?;
        chart
            .configure_mesh()
            .max_light_lines(0)
            .x_desc("Simulation time (s)")
            .y_desc("Force (mN)")
            .draw()?;
        if i == 0 {
            chart
                .draw_series(LineSeries::new([(6.97, 40.), (7.41, 40.)], BLACK.mix(0.6)))?
                .label("Retreat threshold")
                .legend(|(x, y)| PathElement::new([(x, y), (x + 18, y)], BLACK.mix(0.6)));
        }
        for v in series {
            let (color, label) = match v.fixture {
                Fixture::Snag => (RGBColor(170, 58, 27), "Snag"),
                Fixture::Rub => (RGBColor(14, 102, 150), "Light rub"),
                _ => (RGBColor(92, 109, 97), "Clear"),
            };
            chart
                .draw_series(LineSeries::new(
                    v.points
                        .iter()
                        .map(|&(time, jaw, bar)| (time, if i == 0 { jaw } else { bar })),
                    color.stroke_width(2),
                ))?
                .label(label)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 18, y)], color.stroke_width(2))
                });
        }
        if i == 0 {
            chart
                .configure_series_labels()
                .position(SeriesLabelPosition::UpperLeft)
                .background_style(WHITE.mix(0.85))
                .draw()?;
        }
    }
    area.present()?;
    Ok(())
}
