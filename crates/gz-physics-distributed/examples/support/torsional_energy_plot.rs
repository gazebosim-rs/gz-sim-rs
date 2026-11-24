// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Result, Sample};
use plotters::prelude::*;
use std::{fs::File, path::Path};
fn samples(dir: &Path, name: &str) -> Result<Vec<Sample>> {
    Ok(serde_json::from_reader(File::open(
        dir.join("energy").join(format!("{name}.json")),
    )?)?)
}
pub fn draw(dir: &Path) -> Result<()> {
    let pos = samples(dir, "rectangle-positive-moving-rough-fine")?;
    let coarse = samples(dir, "rectangle-positive-moving-rough-coarse")?;
    let smooth = samples(dir, "rectangle-positive-moving-smooth-fine")?;
    let file = dir.join("spin-work.svg");
    let root = SVGBackend::new(&file, (1160, 540)).into_drawing_area();
    root.fill(&WHITE)?;
    let ink = RGBColor(41, 51, 60);
    let blue = RGBColor(41, 103, 151);
    let orange = RGBColor(161, 77, 39);
    let green = RGBColor(30, 124, 101);
    let gray = RGBColor(100, 110, 121);
    let chart = root.titled(
        "Rotational kinetic energy after a 0.96 N·m·s angular impulse",
        ("sans-serif", 27).into_font().color(&ink),
    )?;
    let area = chart.margin(10, 12, 10, 10).split_evenly((1, 2));
    let mut energy = ChartBuilder::on(&area[0])
        .caption("Moving 0.8 × 0.4 m block", ("sans-serif", 20).into_font())
        .margin(12)
        .x_label_area_size(51)
        .y_label_area_size(68)
        .build_cartesian_2d(0_f64..0.91, 0_f64..3.80)?;
    energy
        .configure_mesh()
        .x_labels(6)
        .y_labels(6)
        .max_light_lines(1)
        .bold_line_style(RGBColor(223, 228, 232))
        .label_style(("sans-serif", 14))
        .axis_desc_style(("sans-serif", 15))
        .x_desc("Seconds since the drive at 0.30 s")
        .y_desc("Spin kinetic energy (J)")
        .draw()?;
    let xy = |s: &Sample| (s.tick as f64 - 30.) * 0.01;
    energy
        .draw_series(LineSeries::new(
            smooth.iter().skip(30).map(|s| (xy(s), s.energy_j)),
            blue.stroke_width(3),
        ))?
        .label("Zero friction")
        .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 18, y)], blue.stroke_width(3)));
    energy
        .draw_series(LineSeries::new(
            pos.iter().skip(30).map(|s| (xy(s), s.ideal_j)),
            gray.stroke_width(2),
        ))?
        .label("Coulomb prediction")
        .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 18, y)], gray.stroke_width(2)));
    energy
        .draw_series(
            pos.iter()
                .skip(30)
                .step_by(3)
                .map(|s| Circle::new((xy(s), s.energy_j), 3, green.filled())),
        )?
        .label("Rough native samples")
        .legend(move |(x, y)| Circle::new((x + 9, y), 3, green.filled()));
    energy
        .configure_series_labels()
        .position(SeriesLabelPosition::MiddleRight)
        .background_style(WHITE.mix(0.94))
        .border_style(RGBColor(210, 216, 221))
        .draw()?;
    let mut err = ChartBuilder::on(&area[1])
        .caption(
            "Energy difference during braking",
            ("sans-serif", 20).into_font(),
        )
        .margin(12)
        .x_label_area_size(51)
        .y_label_area_size(80)
        .build_cartesian_2d(0_f64..0.35, -0.235_f64..0.025_f64)?;
    err.configure_mesh()
        .x_labels(7)
        .y_labels(7)
        .max_light_lines(1)
        .bold_line_style(RGBColor(223, 228, 232))
        .label_style(("sans-serif", 14))
        .axis_desc_style(("sans-serif", 15))
        .y_label_formatter(&|y| {
            if y.abs() < 1e-10 {
                "0".to_owned()
            } else {
                format!("{y:.2}")
            }
        })
        .x_desc("Seconds since drive")
        .y_desc("Native minus analytic (mJ)")
        .draw()?;
    for (s, color, name) in [
        (&coarse, orange, "10 ms native"),
        (&pos, green, "1 ms refined"),
    ] {
        err.draw_series(LineSeries::new(
            s.iter()
                .skip(30)
                .take(35)
                .map(|v| (xy(v), 1000. * (v.energy_j - v.ideal_j))),
            color.stroke_width(3),
        ))?
        .label(name)
        .legend(move |(x, y)| PathElement::new(vec![(x, y), (x + 18, y)], color.stroke_width(3)));
    }
    err.configure_series_labels()
        .position(SeriesLabelPosition::LowerRight)
        .background_style(WHITE.mix(0.92))
        .border_style(RGBColor(210, 216, 221))
        .draw()?;
    root.present()?;
    Ok(())
}
