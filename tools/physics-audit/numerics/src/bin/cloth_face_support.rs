// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Draw the measured contact footprint and the impulse history from the SDF study.
use plotters::prelude::*;
use serde_json::Value;
use std::{error::Error, fs, path::Path};
const TEAL: RGBColor = RGBColor(16, 113, 112);
const ORANGE: RGBColor = RGBColor(178, 72, 25);

fn draw(root: &Path, output: &Path) -> Result<(), Box<dyn Error>> {
    let frames: Vec<Value> = serde_json::from_slice(&fs::read(root.join("frames.json"))?)?;
    let report: Value = serde_json::from_slice(&fs::read(root.join("comparison.json"))?)?;
    let hit = report["first_tick"][0]
        .as_u64()
        .ok_or("missing narrow impact")? as usize;
    if frames.len() != 601 || hit >= frames.len() {
        return Err("expected a complete 600 ms world trace".into());
    }
    let mut loads = Vec::with_capacity(frames.len());
    let mut first = [None; 3];
    for (tick, row) in frames.iter().enumerate() {
        if row["time_ns"].as_u64() != Some(tick as u64 * 1_000_000) {
            return Err("invalid public clock".into());
        }
        let force = serde_json::from_value::<[f64; 3]>(row["support_n"].clone())?;
        for (lane, value) in force.iter().enumerate() {
            if *value > 1e-7 {
                first[lane].get_or_insert(tick);
            }
        }
        loads.push(force);
    }
    let declared = serde_json::from_value::<[Option<usize>; 3]>(report["first_tick"].clone())?;
    if first != declared {
        return Err("reported first contact disagrees with the measured impulses".into());
    }
    let first_j = serde_json::from_value::<[f64; 3]>(report["first_impulse_ns"].clone())?;
    for (lane, tick) in first.into_iter().enumerate() {
        let measured = tick.map_or(0., |tick| loads[tick][lane] * 0.001);
        if (first_j[lane] - measured).abs() > 1e-12 {
            return Err("reported impact impulse differs from the trace".into());
        }
    }
    let read = |name: &str| -> Result<[f64; 3], Box<dyn Error>> {
        Ok(serde_json::from_value(
            frames[hit]["poses"][format!("default::narrow::deck::{name}")]["translation"].clone(),
        )?)
    };
    let center = read("support")?;
    let nodes = (0..4)
        .map(|i| read(&format!("p{i}")))
        .collect::<Result<Vec<_>, _>>()?;
    let geom = |p: [f64; 3]| ((p[0] - center[0]) * 1000., (p[1] - center[1]) * 1000.);
    let page = SVGBackend::new(output, (1000, 510)).into_drawing_area();
    page.fill(&WHITE)?;
    let (header, body) = page.split_vertically(62);
    header.draw(&Text::new(
        "Cloth supported between its four material nodes",
        (24, 14),
        ("sans-serif", 26).into_font().style(FontStyle::Bold),
    ))?;
    header.draw(&Text::new(
        format!(
            "First upward impulse at {hit} ms — narrow {:.2} mN·s, wide {:.2} mN·s",
            first_j[0] * 1000.,
            first_j[1] * 1000.
        ),
        (25, 43),
        ("sans-serif", 15).into_font(),
    ))?;
    let panels = body.split_evenly((1, 2));
    let mut view = ChartBuilder::on(&panels[0])
        .margin(20)
        .caption("Top view at first contact", ("sans-serif", 20))
        .x_label_area_size(40)
        .y_label_area_size(54)
        .build_cartesian_2d(-320f64..160f64, -160f64..320f64)?;
    view.configure_mesh()
        .x_desc("x from support center (mm)")
        .y_desc("y from support center (mm)")
        .light_line_style(RGBColor(232, 236, 238))
        .draw()?;
    view.draw_series([Rectangle::new(
        [(-35., -35.), (35., 35.)],
        ORANGE.mix(0.75).filled(),
    )])?;
    for ids in [[0, 1, 3], [0, 3, 2]] {
        let line = ids
            .into_iter()
            .chain([ids[0]])
            .map(|i| geom(nodes[i]))
            .collect::<Vec<_>>();
        view.draw_series([PathElement::new(line, TEAL.stroke_width(2))])?;
    }
    view.draw_series(
        nodes
            .into_iter()
            .map(|p| Circle::new(geom(p), 4, TEAL.filled())),
    )?;
    view.draw_series([Text::new(
        "70 × 70 mm support",
        (-22., 57.),
        ("sans-serif", 14).into_font().color(&ORANGE),
    )])?;
    let mut force = ChartBuilder::on(&panels[1])
        .margin(20)
        .caption("Settling load (10 ms means)", ("sans-serif", 20))
        .x_label_area_size(40)
        .y_label_area_size(54)
        .build_cartesian_2d(70f64..600f64, 0f64..1.65f64)?;
    force
        .configure_mesh()
        .x_desc("simulation time (ms)")
        .y_desc("support force / sheet weight")
        .light_line_style(RGBColor(232, 236, 238))
        .draw()?;
    for (idx, label, color) in [
        (0, "Narrow support", ORANGE),
        (1, "Wide reference", TEAL),
        (2, "Off-edge control", RGBColor(113, 81, 153)),
    ] {
        let samples = loads[70..].chunks(10).enumerate().map(|(i, chunk)| {
            let ms = 70 + i * 10 + chunk.len() - 1;
            let f = chunk.iter().map(|load| load[idx]).sum::<f64>()
                / (chunk.len() as f64 * 0.14 * 9.81);
            (ms as f64, f)
        });
        force
            .draw_series(LineSeries::new(samples, color.stroke_width(2)))?
            .label(label)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], color.stroke_width(2)));
    }
    force
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.9))
        .border_style(RGBColor(218, 223, 226))
        .draw()?;
    page.present()?;
    Ok(())
}
fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();
    if let [input, output] = args.as_slice() {
        draw(Path::new(input), Path::new(output))
    } else {
        Err("expected measured directory and output SVG".into())
    }
}
