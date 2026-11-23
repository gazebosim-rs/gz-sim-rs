// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_proto::{
    msgs::{Altimeter, Imu},
    prost::Message,
};
use plotters::prelude::*;
use std::path::Path;
/// Physical samples measured by the independent Zenoh client, at their native rates.
pub(super) fn draw(directory: &Path, readings: &[Vec<(u64, Vec<u8>)>; 4]) -> super::Result<()> {
    let path = directory.join("sensors.svg");
    let canvas = SVGBackend::new(&path, (880, 500)).into_drawing_area();
    canvas.fill(&WHITE)?;
    let panels = canvas.split_evenly((2, 1));
    let mut top = ChartBuilder::on(&panels[0])
        .margin(12)
        .x_label_area_size(30)
        .y_label_area_size(68)
        .caption("Free sphere, offset altimeter at 50 Hz", ("sans-serif", 18))
        .build_cartesian_2d(0f64..42., -0.145f64..0.10)?;
    top.configure_mesh()
        .x_desc("Simulation time (ms)")
        .y_desc("Vertical velocity (m/s)")
        .draw()?;
    let free = readings[1]
        .iter()
        .map(|(tick, bytes)| {
            Ok((
                *tick as f64,
                Altimeter::decode(&bytes[..])?.vertical_velocity,
            ))
        })
        .collect::<super::Result<Vec<_>>>()?;
    top.draw_series(LineSeries::new(
        free.iter().copied(),
        &RGBColor(51, 117, 164),
    ))?;
    top.draw_series(
        free.iter()
            .map(|p| Circle::new(*p, 4, RGBColor(16, 104, 166).filled())),
    )?;
    let mut lower = ChartBuilder::on(&panels[1])
        .margin(12)
        .x_label_area_size(33)
        .y_label_area_size(68)
        .caption(
            "Supported sphere, rotated IMU at 200 Hz",
            ("sans-serif", 18),
        )
        .build_cartesian_2d(0f64..42., -4f64..110.)?;
    lower
        .configure_mesh()
        .x_desc("Simulation time (ms)")
        .y_desc("Tangential acceleration (m/s²)")
        .draw()?;
    for (tick, bytes) in &readings[2] {
        let a = Imu::decode(&bytes[..])?
            .linear_acceleration
            .ok_or("missing acceleration")?;
        let point = (*tick as f64, a.x.hypot(a.y));
        lower.draw_series([PathElement::new(
            vec![(point.0, 0.), point],
            RGBColor(173, 87, 45),
        )])?;
        lower.draw_series([Circle::new(point, 4, RGBColor(173, 87, 45).filled())])?;
    }
    canvas.present()?;
    Ok(())
}
