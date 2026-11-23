// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Result, Sample, chord, maximum_vector};
use plotters::prelude::*;
use std::{
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};

/// Plot the measured differences on their declared scales; a value of one reaches its gate.
pub(super) fn write(out: &Path, h: u64, reference: &[Sample], rotated: &[Sample]) -> Result<()> {
    let mut csv = BufWriter::new(File::create_new(out.join("symmetry.csv"))?);
    writeln!(
        csv,
        "time_s,sphere_position_difference_m,sphere_quaternion_chord,tangent_impulse_difference_n_s"
    )?;
    let mut curves: [Vec<(f64, f64)>; 3] = std::array::from_fn(|_| vec![]);
    let mut maximum = 1f64;
    for (i, (a, b)) in reference.iter().zip(rotated).enumerate() {
        let t = (i as u64 + 1) as f64 * h as f64 / 1000.;
        let values = [
            maximum_vector(a.positions[1], b.positions[1]),
            chord(a.rotations[1], b.rotations[1]),
            maximum_vector(a.tangent, b.tangent),
        ];
        writeln!(
            csv,
            "{t:.6},{:.16e},{:.16e},{:.16e}",
            values[0], values[1], values[2]
        )?;
        for ((curve, v), gate) in curves.iter_mut().zip(values).zip([1e-6, 1e-7, 1e-7]) {
            maximum = maximum.max(v / gate);
            curve.push((t, v / gate));
        }
    }
    csv.flush()?;
    let path = out.join("symmetry.svg");
    let root = SVGBackend::new(&path, (900, 450)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .caption(
            format!("Yaw-rotated finite contact at {h} ms"),
            ("sans-serif", 25),
        )
        .margin(16)
        .x_label_area_size(45)
        .y_label_area_size(57)
        .build_cartesian_2d(0f64..1.2, 0f64..maximum * 1.12)?;
    chart
        .configure_mesh()
        .x_desc("Simulation time (s)")
        .y_desc("Measured difference / declared limit")
        .light_line_style(RGBColor(228, 229, 233))
        .draw()?;
    chart
        .draw_series(LineSeries::new(
            [(0., 1.), (1.2, 1.)],
            BLACK.mix(0.6).stroke_width(1),
        ))?
        .label("Acceptance limit")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 18, y)], BLACK));
    for ((curve, label), color) in curves
        .into_iter()
        .zip([
            "Center displacement",
            "Quaternion chord",
            "Tangential impulse",
        ])
        .zip([
            RGBColor(22, 108, 133),
            RGBColor(109, 59, 152),
            RGBColor(193, 82, 32),
        ])
    {
        chart
            .draw_series(LineSeries::new(curve, color.stroke_width(2)))?
            .label(label)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 18, y)], color.stroke_width(2)));
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.9))
        .border_style(RGBColor(170, 174, 178))
        .draw()?;
    root.present()?;
    Ok(())
}
