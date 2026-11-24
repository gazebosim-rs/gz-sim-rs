// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Chart the measured cut and stored application input from a selected recovery package.
use clap::Parser;
use gz_sim_cli::live::ConvoyCapsule;
use plotters::prelude::*;
use std::{error::Error, fs, io::Write, path::PathBuf};

#[derive(Parser)]
struct Args {
    /// Immutable supervisor package and its expected full-file digest.
    capsule: PathBuf,
    sha256: String,
    /// Fresh path for the SVG figure.
    output: PathBuf,
}

fn main() -> Result<(), Box<dyn Error>> {
    let a = Args::parse();
    let package = ConvoyCapsule::open(&a.capsule, &a.sha256)?;
    let cut = &package.cut().cut;
    let state = &cut.observation;
    let start = state.hinge.tick;
    let horizon = start
        .checked_add(20)
        .ok_or("insufficient remaining tick range")?;
    let input: Vec<_> = (start..horizon)
        .map(|t| package.drive().interval_effort(t))
        .collect();

    let bodies = [
        (
            "Base",
            state.hinge.links[0],
            cut.spec.hinge.links[0].radius,
            cut.owners[0],
        ),
        (
            "Tool",
            state.hinge.links[1],
            cut.spec.hinge.links[1].radius,
            cut.owners[0],
        ),
        (
            "Sphere 1",
            state.spheres[0],
            cut.spec.spheres[0].radius,
            cut.owners[1],
        ),
        (
            "Sphere 2",
            state.spheres[1],
            cut.spec.spheres[1].radius,
            cut.owners[2],
        ),
    ];
    let navy = RGBColor(24, 44, 60);
    let teal = RGBColor(2, 108, 122);
    let gray = RGBColor(88, 106, 117);
    let grid = RGBColor(229, 235, 236);
    let mut document = String::new();
    {
        let page = SVGBackend::with_string(&mut document, (1050, 550)).into_drawing_area();
        page.fill(&WHITE)?;
        page.draw(&Text::new(
            "Supervisor checkpoint",
            (28, 18),
            ("sans-serif", 35)
                .into_font()
                .style(FontStyle::Bold)
                .color(&navy),
        ))?;
        page.draw(&Text::new(format!("Epoch {}  ·  Simulated time {} ms  ·  {} worker slots  ·  Sphere-pair normal impulse {:.6} N·s",
            cut.epoch, start, cut.width, state.sphere_reaction.normal_impulse),
            (30, 71), ("sans-serif", 18).into_font().color(&navy)))?;
        let panels = page.margin(116, 108, 18, 22).split_evenly((1, 2));

        let lo = bodies
            .iter()
            .map(|(_, s, r, _)| s.position[0] - r)
            .fold(f64::INFINITY, f64::min);
        let hi = bodies
            .iter()
            .map(|(_, s, r, _)| s.position[0] + r)
            .fold(f64::NEG_INFINITY, f64::max);
        let margin = (hi - lo) * 0.08;
        let mut geometry = ChartBuilder::on(&panels[0])
            .margin(8)
            .caption("Body extents along world X", ("sans-serif", 22))
            .x_label_area_size(49)
            .y_label_area_size(122)
            .build_cartesian_2d(lo - margin..hi + margin, -0.7_f64..3.7)?;
        geometry
            .configure_mesh()
            .disable_y_mesh()
            .x_desc("World X (m)")
            .y_labels(4)
            .max_light_lines(0)
            .bold_line_style(grid)
            .label_style(("sans-serif", 13))
            .y_label_formatter(&|&y| {
                let i = y.round();
                if (0.0..=3.0).contains(&i) && (i - y).abs() < 0.01 {
                    let (name, _, _, slot) = bodies[3 - i as usize];
                    format!("{name} · slot {slot}")
                } else {
                    String::new()
                }
            })
            .draw()?;
        for (i, (_, s, r, _)) in bodies.iter().enumerate() {
            let y = (3 - i) as f64;
            geometry.draw_series(std::iter::once(PathElement::new(
                [(s.position[0] - r, y), (s.position[0] + r, y)],
                teal.mix(0.55).stroke_width(9),
            )))?;
            geometry.draw_series(std::iter::once(Circle::new(
                (s.position[0], y),
                5,
                navy.filled(),
            )))?;
        }

        let minimum = input.iter().copied().fold(0.0, f64::min);
        let maximum = input.iter().copied().fold(0.0, f64::max);
        let padding = if maximum == minimum {
            0.001
        } else {
            0.15 * (maximum - minimum)
        };
        let mut drive = ChartBuilder::on(&panels[1])
            .margin(8)
            .caption(
                "Stored drive over the next 20 intervals",
                ("sans-serif", 22),
            )
            .x_label_area_size(49)
            .y_label_area_size(65)
            .build_cartesian_2d(0_f64..20.0, minimum - padding..maximum + padding)?;
        drive
            .configure_mesh()
            .x_desc("Time after checkpoint (ms)")
            .y_desc("Torque (N·m)")
            .x_labels(6)
            .max_light_lines(0)
            .bold_line_style(grid)
            .label_style(("sans-serif", 13))
            .y_label_formatter(&|v| format!("{v:.3}"))
            .draw()?;
        let line = input
            .iter()
            .enumerate()
            .flat_map(|(i, &v)| [(i as f64, v), ((i + 1) as f64, v)]);
        drive.draw_series(LineSeries::new(line, teal.stroke_width(3)))?;
        if let Some(effort) = cut.pending_effort {
            drive.draw_series(std::iter::once(Circle::new((0., effort), 5, navy.filled())))?;
        }
        let pending = cut
            .pending_effort
            .map(|v| format!("{v:+.4} N·m"))
            .unwrap_or_else(|| "none".into());
        page.draw(&Text::new(format!("Pending effort at capture: {pending}. Each plateau is the mean torque applied over one simulated millisecond."),
            (30,461),("sans-serif",16).into_font().color(&navy)))?;
        page.draw(&Text::new(
            format!(
                "Package SHA-256 {}…   ·   Physics archive SHA-256 {}…",
                &package.sha256()[..20],
                &package.cut().sha256[..20]
            ),
            (30, 501),
            ("monospace", 15).into_font().color(&gray),
        ))?;
        page.present()?;
    }
    fs::File::create_new(a.output)?.write_all(document.as_bytes())?;
    Ok(())
}
