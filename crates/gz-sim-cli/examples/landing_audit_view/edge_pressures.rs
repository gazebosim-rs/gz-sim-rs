// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Show how admissible responses load different places on the same finite edge.
use super::*;
use plotters::prelude::*;

pub(super) fn draw(out: &Path, control: &Value) -> Result<()> {
    let models = &control["conditional_pressure_models"];
    let alternatives = models["alternatives"]
        .as_array()
        .ok_or("missing pressure alternatives")?;
    let stopped = control["references"]
        .as_array()
        .and_then(|refs| refs.last())
        .ok_or("missing contact boundary")?;
    let points = &stopped["unresolved_line"]["line_points_m"];
    let start = vec3(&points[0])?;
    let span = vec3(&points[1])? - start;
    let length_mm = 1000. * span.length();
    let axis = span.try_normalize().ok_or("zero-length contact edge")?;
    let time_ms = 1000.
        * (number(models, "start_s")? - number(&control["references"][0]["fine"], "elapsed")?);
    if alternatives.len() < 2 || stopped["qualified"] != false {
        return Err("expected alternative responses at the unqualified contact boundary".into());
    }
    let root = SVGBackend::new(out, (1280, 455)).into_drawing_area();
    root.fill(&WHITE)?;
    let (head, body) = root.split_vertically(88);
    head.draw(&Text::new(
        format!("Possible loads on the same foot edge at {time_ms:.4} ms"),
        (32, 36),
        ("sans-serif", 26).into_font(),
    ))?;
    head.draw(&Text::new(
        "Each bar shows a possible upward impulse at one contact point.",
        (32, 67),
        ("sans-serif", 18).into_font().color(&RGBColor(70, 75, 78)),
    ))?;
    let panels = body.split_evenly((1, alternatives.len()));
    let mut series = Vec::new();
    for m in alternatives {
        let sites: [[f64; 3]; 2] = serde_json::from_value(m["sites_m"].clone())?;
        let impulses: [f64; 2] = serde_json::from_value(m["normal_ns"].clone())?;
        let mut samples = vec![];
        for (point, impulse) in sites.into_iter().zip(impulses) {
            let r = DVec3::from(point) - start;
            let x = r.dot(axis);
            if r.distance(x * axis) > 1e-11
                || x < -1e-12
                || x > span.length() + 1e-12
                || !impulse.is_finite()
                || impulse <= 0.
            {
                return Err("invalid pressure site on the finite edge".into());
            }
            samples.push((1000. * x, 1000. * impulse));
        }
        series.push(samples);
    }
    let max = series.iter().flatten().map(|p| p.1).fold(0., f64::max);
    for (panel, samples) in panels.iter().zip(series) {
        let pos = samples[1].0 / length_mm;
        let label = if (pos - 1.).abs() < 1e-9 {
            "Both ends of the edge".to_owned()
        } else if (pos - 0.75).abs() < 1e-9 {
            "One end and the three-quarter point".to_owned()
        } else if (pos - 0.5).abs() < 1e-9 {
            "One end and the midpoint".to_owned()
        } else {
            format!(
                "Load points at {:.0} and {:.0} mm",
                samples[0].0, samples[1].0
            )
        };
        let color = RGBColor(8, 126, 109);
        let mut chart = ChartBuilder::on(panel)
            .caption(label, ("sans-serif", 21))
            .margin(14)
            .x_label_area_size(54)
            .y_label_area_size(64)
            .build_cartesian_2d(-35.0..length_mm + 35., 0.0..max * 1.23)?;
        chart
            .configure_mesh()
            .disable_x_mesh()
            .max_light_lines(0)
            .x_labels(5)
            .y_labels(5)
            .x_desc("Position along the edge (mm)")
            .y_desc("Normal impulse (mN·s)")
            .light_line_style(RGBColor(225, 229, 231))
            .draw()?;
        let baseline = PathElement::new(
            vec![(0., 0.), (length_mm, 0.)],
            RGBColor(75, 81, 85).stroke_width(4),
        );
        chart.draw_series(std::iter::once(baseline))?;
        for (x, j) in samples {
            chart.draw_series(std::iter::once(Rectangle::new(
                [(x - 5., 0.), (x + 5., j)],
                color.filled(),
            )))?;
            chart.draw_series(std::iter::once(Text::new(
                format!("{j:.3}"),
                (x - 17., j + 0.05 * max),
                ("sans-serif", 15).into_font().color(&color),
            )))?;
        }
    }
    root.present()?;
    Ok(())
}
