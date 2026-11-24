// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Result, equal, integer, limit, number};
use plotters::{coord::Shift, prelude::*};
use serde_json::Value;

pub(super) fn draw(area: &DrawingArea<SVGBackend<'_>, Shift>, data: &Value) -> Result<()> {
    let ticks = integer(&data["ticks"])?;
    let mut rows = vec![];
    for trial in data["trials"].as_array().ok_or("missing refined meshes")? {
        let spacing = number(&trial["spacing_m"])? * 1000.;
        let done = integer(&trial["completed_ticks"])?;
        let measured = match trial["status"].as_str() {
            Some("completed") if done == ticks => {
                let cross = number(&trial["mean_cross_panel_n"])?;
                let width: [f64; 3] = serde_json::from_value(trial["cross_panel_width_n"].clone())?;
                equal(cross, width.into_iter().sum())?;
                let up: [f64; 2] = serde_json::from_value(trial["mean_jaw_up_n"].clone())?;
                Some([1000. * cross, 1000. * (up[0] + up[1])])
            }
            Some("refused") if done < ticks => None,
            _ => return Err("mesh completion disagrees with number of measured ticks".into()),
        };
        rows.push((spacing, done, measured));
    }
    if rows.is_empty() {
        return Err("mesh study has no trials".into());
    }
    let colors = [RGBColor(83, 76, 142), RGBColor(87, 112, 122)];
    let high = limit(rows.iter().filter_map(|(_, _, m)| *m).flatten());
    let mut chart = ChartBuilder::on(area)
        .margin(14)
        .caption("Refining the sheet at fixed jaw motion", ("sans-serif", 22))
        .x_label_area_size(62)
        .y_label_area_size(75)
        .build_cartesian_2d(
            (0..100 * rows.len()).with_key_points((0..rows.len()).map(|i| 100 * i + 50).collect()),
            0.0..1.28 * high,
        )?;
    chart
        .configure_mesh()
        .x_desc("Mesh spacing across the fold (mm)")
        .y_desc("Mean force (mN)")
        .x_label_formatter(&|v| format!("{:.2}", rows[*v / 100].0))
        .disable_x_mesh()
        .max_light_lines(1)
        .axis_desc_style(("sans-serif", 16))
        .label_style(("sans-serif", 14))
        .draw()?;
    for (i, (_, completed, measurement)) in rows.into_iter().enumerate() {
        match measurement {
            Some(means) => {
                for (j, mean) in means.into_iter().enumerate() {
                    let left = 100 * i + 16 + 36 * j;
                    let color = colors[j];
                    chart.draw_series(std::iter::once(Rectangle::new(
                        [(left, 0.), (left + 29, mean)],
                        color.filled(),
                    )))?;
                    chart.draw_series(std::iter::once(Text::new(
                        format!("{mean:.1}"),
                        (left + 1, mean + 0.045 * high),
                        ("sans-serif", 15).into_font(),
                    )))?;
                }
            }
            None => {
                chart.draw_series(std::iter::once(Text::new(
                    format!("Stopped at tick {}", completed + 1),
                    (100 * i + 2, 0.14 * high),
                    ("sans-serif", 15).into_font(),
                )))?;
            }
        }
    }
    for (color, name) in colors
        .into_iter()
        .zip(["Between panels", "Upward support from jaws"])
    {
        chart
            .draw_series(std::iter::empty::<Rectangle<(usize, f64)>>())?
            .label(name)
            .legend(move |(x, y)| Rectangle::new([(x, y - 4), (x + 18, y + 4)], color.filled()));
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.94))
        .label_font(("sans-serif", 15))
        .draw()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn renders_a_refusal_separately_from_a_completed_zero_load() {
        let mut svg = String::new();
        let root = SVGBackend::with_string(&mut svg, (740, 440)).into_drawing_area();
        let stopped = json!({"status":"refused","completed_ticks":0,"spacing_m":0.02});
        let zero = json!({"status":"completed","completed_ticks":3,"spacing_m":0.05,
            "mean_cross_panel_n":0.,"cross_panel_width_n":[0.,0.,0.],"mean_jaw_up_n":[0.04,0.04]});
        assert!(draw(&root, &json!({"ticks":3,"trials":[]})).is_err());
        assert!(draw(&root, &json!({"ticks":3})).is_err());
        for field in [
            "spacing_m",
            "completed_ticks",
            "mean_cross_panel_n",
            "cross_panel_width_n",
            "mean_jaw_up_n",
        ] {
            let mut bad = zero.clone();
            bad[field] = Value::Null;
            assert!(draw(&root, &json!({"ticks":3,"trials":[bad]})).is_err());
        }
        let mut wrong = zero.clone();
        wrong["cross_panel_width_n"][1] = json!(0.1);
        assert!(draw(&root, &json!({"ticks":3,"trials":[wrong]})).is_err());
        assert!(draw(&root, &json!({"ticks":4,"trials":[zero]})).is_err());
        draw(&root, &json!({"ticks":3,"trials":[stopped,zero]})).unwrap();
        drop(root);
        assert!(svg.contains("Stopped at tick 1"));
        assert!(svg.contains("80.0"));
    }
}
