// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    runner::Runner,
    task::{EVALUATE, sphere},
};
use plotters::prelude::*;
use std::{
    error::Error,
    fs::File,
    io::{BufWriter, Write},
    path::Path,
};
#[derive(Default)]
pub struct Trajectory {
    rows: Vec<(u64, u64, f64, f64, f64)>,
}
impl Trajectory {
    pub fn add(&mut self, t: u64, a: &Runner, b: &Runner) {
        let i = 6;
        let e = &a.agent.episodes[i];
        self.rows.push((
            t,
            e.start,
            e.target,
            sphere(&a.states()[i].state).velocity[0],
            sphere(&b.states()[i].state).velocity[0],
        ));
    }
    pub fn write(self, out: &Path) -> Result<(), Box<dyn Error>> {
        let mut f = BufWriter::new(File::create_new(out.join("held-out-velocity.csv"))?);
        writeln!(f, "epoch,episode_start,target_m_s,greedy_m_s,coast_m_s")?;
        for (k, start, t, a, b) in &self.rows {
            writeln!(f, "{k},{start},{t},{a},{b}")?;
        }
        f.flush()?;
        let file = out.join("held-out-velocity.svg");
        let graph = SVGBackend::new(&file, (920, 490)).into_drawing_area();
        graph.fill(&WHITE)?;
        let max = self
            .rows
            .iter()
            .map(|x| x.2.abs().max(x.3.abs()).max(x.4.abs()))
            .fold(0., f64::max)
            * 1.12;
        let mut c = ChartBuilder::on(&graph)
            .caption("Tracking a held-out velocity command", ("sans-serif", 26))
            .margin(14)
            .x_label_area_size(44)
            .y_label_area_size(66)
            .build_cartesian_2d(0u64..EVALUATE, -max..max)?;
        c.configure_mesh()
            .x_desc("Simulation time (ms)")
            .y_desc("World X velocity (m/s)")
            .draw()?;
        for (i, name, color) in [
            (0, "Target", RGBColor(21, 119, 66)),
            (1, "Trained policy", RGBColor(12, 109, 170)),
            (2, "No impulse", RGBColor(180, 91, 18)),
        ] {
            c.draw_series(LineSeries::new(
                self.rows.iter().map(|x| (x.0, [x.2, x.3, x.4][i])),
                color.stroke_width(3),
            ))?
            .label(name)
            .legend(move |(x, y)| {
                PathElement::new(vec![(x, y), (x + 30, y)], color.stroke_width(3))
            });
        }
        c.configure_series_labels()
            .position(SeriesLabelPosition::LowerRight)
            .border_style(BLACK.mix(0.25))
            .background_style(WHITE.mix(0.92))
            .draw()?;
        graph.present()?;
        Ok(())
    }
}
