// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use plotters::prelude::*;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::Path,
    time::Duration,
};
#[derive(Default)]
pub struct Report {
    rows: Vec<(usize, [Duration; 3])>,
}
impl Report {
    pub fn sample(&mut self, tick: usize, mixed: Duration, separate: Duration, cpu: Duration) {
        self.rows.push((tick, [mixed, separate, cpu]));
    }
    pub fn write(self, path: &Path) -> Result<(), Box<dyn Error>> {
        std::fs::create_dir(path)?;
        let mut csv = BufWriter::new(File::create_new(path.join("latency.csv"))?);
        writeln!(csv, "epoch,mixed_gpu_ns,separate_gpu_ns,scalar_cpu_ns")?;
        for (k, ds) in &self.rows {
            writeln!(
                csv,
                "{k},{},{},{}",
                ds[0].as_nanos(),
                ds[1].as_nanos(),
                ds[2].as_nanos()
            )?;
        }
        csv.flush()?;
        let chart_path = path.join("latency.svg");
        let root = SVGBackend::new(&chart_path, (900, 510)).into_drawing_area();
        root.fill(&WHITE)?;
        let mut sorted: [Vec<f64>; 3] = std::array::from_fn(|j| {
            self.rows
                .iter()
                .map(|(_, ds)| ds[j].as_secs_f64() * 1e6)
                .collect()
        });
        for ds in &mut sorted {
            ds.sort_by(f64::total_cmp);
        }
        let len = sorted[0].len();
        let upper = sorted.iter().map(|ds| ds[len - 1]).fold(1., f64::max);
        let floor = sorted.iter().map(|ds| ds[0]).fold(upper, f64::min).max(0.1);
        let mut c = ChartBuilder::on(&root)
            .caption(
                "Observed latency for 2,048 mixed environments",
                ("sans-serif", 26),
            )
            .margin(18)
            .x_label_area_size(43)
            .y_label_area_size(78)
            .build_cartesian_2d(0f64..100., (floor * 0.8..upper * 1.2).log_scale())?;
        c.configure_mesh()
            .x_desc("Percentile of 500 observed epochs")
            .y_desc("Wall time per 1 ms epoch (µs, log scale)")
            .draw()?;
        for (name, (ds, color)) in ["Shared GPU", "Three GPU submissions", "Serial scalar CPU"]
            .into_iter()
            .zip(sorted.into_iter().zip([
                RGBColor(0, 103, 140),
                RGBColor(185, 85, 29),
                RGBColor(34, 128, 81),
            ]))
        {
            c.draw_series(LineSeries::new(
                ds.into_iter()
                    .enumerate()
                    .map(|(i, d)| (100. * (i + 1) as f64 / len as f64, d)),
                color.stroke_width(3),
            ))?
            .label(name)
            .legend(move |(x, y)| {
                PathElement::new(vec![(x, y), (x + 22, y)], color.stroke_width(3))
            });
        }
        c.configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft)
            .background_style(WHITE.mix(0.88))
            .border_style(BLACK.mix(0.3))
            .draw()?;
        root.present()?;
        Ok(())
    }
}
