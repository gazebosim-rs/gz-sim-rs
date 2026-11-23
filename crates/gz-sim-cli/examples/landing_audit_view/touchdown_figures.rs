// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use plotters::prelude::*;

#[derive(Deserialize)]
struct Report {
    runs: Vec<Run>,
}
#[derive(Deserialize)]
struct Run {
    receiver: String,
    motion: String,
    factor: u64,
    first_native_ns: f64,
    first_outer_ns: f64,
    window_ns: f64,
    stroke_ns: f64,
}

fn plot_impulses(audit: &std::path::Path, out: &std::path::Path) -> Result<()> {
    let report: Report = read(&audit.join("summary.json"))?;
    let g = SVGBackend::new(out, (1090, 760)).into_drawing_area();
    g.fill(&WHITE)?;
    let areas = g
        .titled(
            "Third landing with refinement limited to 5 ms",
            ("sans-serif", 28),
        )?
        .split_evenly((2, 2));
    for (i, area) in areas.iter().enumerate() {
        let value = |r: &Run| {
            [
                r.first_native_ns,
                r.first_outer_ns,
                r.window_ns,
                r.stroke_ns,
            ][i]
        };
        let xmax = report
            .runs
            .iter()
            .filter(|r| r.motion == "native")
            .map(value)
            .fold(0f64, f64::max)
            * 1.15;
        let mut chart = ChartBuilder::on(area)
            .margin(12)
            .caption(
                [
                    "First positive native solve",
                    "First 500 µs outer step",
                    "Fixed 5 ms interval",
                    "Complete moving stroke",
                ][i],
                ("sans-serif", 19),
            )
            .x_label_area_size(45)
            .y_label_area_size(62)
            .build_cartesian_2d(0.5..3.5, 0.0..xmax)?;
        chart
            .configure_mesh()
            .max_light_lines(1)
            .light_line_style(RGBColor(232, 236, 239))
            .disable_x_mesh()
            .x_labels(3)
            .x_label_formatter(&|x: &f64| match *x {
                1. => "500".into(),
                2. => "250".into(),
                3. => "125".into(),
                _ => String::new(),
            })
            .x_desc("Native touchdown step (µs)")
            .y_desc("Normal impulse (N·s)")
            .draw()?;
        for (lane, name, color, offset) in [
            (
                "switched",
                "Switched history",
                RGBColor(13, 123, 114),
                -0.16,
            ),
            ("reference", "Always dynamic", RGBColor(46, 86, 165), 0.16),
        ] {
            let mut bars = Vec::new();
            for (k, n) in [1, 2, 4].into_iter().enumerate() {
                let r = report
                    .runs
                    .iter()
                    .find(|r| r.receiver == lane && r.motion == "native" && r.factor == n)
                    .ok_or("missing native history")?;
                let x = k as f64 + 1. + offset;
                bars.push(Rectangle::new(
                    [(x - 0.14, 0.), (x + 0.14, value(r))],
                    color.filled(),
                ));
            }
            chart.draw_series(bars)?.label(name).legend(move |(x, y)| {
                Rectangle::new([(x, y - 4), (x + 12, y + 4)], color.filled())
            });
        }
        if i == 0 {
            chart
                .configure_series_labels()
                .position(SeriesLabelPosition::UpperLeft)
                .background_style(WHITE.mix(0.9))
                .draw()?;
        }
    }
    g.present()?;
    Ok(())
}
#[derive(Deserialize)]
struct Benchmark {
    rows: Vec<Timing>,
}
#[derive(Deserialize)]
struct Timing {
    scope: String,
    receiver: String,
    motion: String,
    factor: u64,
    window: Quantiles,
}
#[derive(Deserialize)]
struct Quantiles {
    p95_ns: f64,
}

fn latency(audit: &std::path::Path, out: &std::path::Path) -> Result<()> {
    let times: Benchmark = read(&audit.join("timings.json"))?;
    let surface = SVGBackend::new(out, (1000, 435)).into_drawing_area();
    surface.fill(&WHITE)?;
    let panels = surface
        .titled("Touchdown cost per 500 µs outer step", ("sans-serif", 26))?
        .split_evenly((1, 2));
    let max = times
        .rows
        .iter()
        .map(|r| r.window.p95_ns / 1000.)
        .fold(0f64, f64::max)
        * 1.14;
    for (panel, (scope, title)) in panels
        .iter()
        .zip([("bevy", "Complete Bevy step"), ("native", "Native solver")])
    {
        let mut c = ChartBuilder::on(panel)
            .margin(12)
            .caption(title, ("sans-serif", 19))
            .x_label_area_size(48)
            .y_label_area_size(65)
            .build_cartesian_2d(0.5..3.5, 0.0..max)?;
        c.configure_mesh()
            .max_light_lines(1)
            .light_line_style(RGBColor(232, 236, 239))
            .disable_x_mesh()
            .x_labels(3)
            .x_label_formatter(&|x: &f64| match *x {
                1. => "500".into(),
                2. => "250".into(),
                3. => "125".into(),
                _ => String::new(),
            })
            .x_desc("Native touchdown step (µs)")
            .y_desc("p95 elapsed time (µs)")
            .draw()?;
        for (lane, name, col, offset) in [
            (
                "switched",
                "Switched history",
                RGBColor(13, 123, 114),
                -0.16,
            ),
            ("reference", "Always dynamic", RGBColor(46, 86, 165), 0.16),
        ] {
            let mut bars = Vec::new();
            for (index, factor) in [1, 2, 4].into_iter().enumerate() {
                let r = times
                    .rows
                    .iter()
                    .find(|r| {
                        r.scope == scope
                            && r.receiver == lane
                            && r.motion == lane
                            && r.factor == factor
                    })
                    .ok_or("missing same-history timing")?;
                let x = index as f64 + 1. + offset;
                bars.push(Rectangle::new(
                    [(x - 0.14, 0.), (x + 0.14, r.window.p95_ns / 1000.)],
                    col.filled(),
                ));
            }
            c.draw_series(bars)?
                .label(name)
                .legend(move |(x, y)| Rectangle::new([(x, y - 4), (x + 12, y + 4)], col.filled()));
        }
        if scope == "bevy" {
            c.configure_series_labels()
                .position(SeriesLabelPosition::UpperLeft)
                .background_style(WHITE.mix(0.9))
                .draw()?;
        }
    }
    surface.present()?;
    Ok(())
}
fn cumulative(trace: &WindowTrace) -> Result<Vec<(f64, f64)>> {
    let cut = trace.frames.first().ok_or("empty trace")?.time_us;
    let mut sum = 0.;
    let mut data = vec![(0., 0.)];
    for s in trace
        .frames
        .iter()
        .skip(1)
        .take_while(|r| r.time_us <= cut + 5000)
    {
        let t = (s.time_us - cut) as f64 / 1000.;
        data.push((t, sum));
        sum += s.normal_impulse_ns;
        data.push((t, sum));
    }
    Ok(data)
}
fn onset(audit: &std::path::Path, out: &std::path::Path) -> Result<()> {
    let root = SVGBackend::new(out, (1000, 495)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut traces = Vec::new();
    let dir = audit.join("traces");

    for r in ["reference", "switched"] {
        for n in [1, 2, 4] {
            let source: WindowTrace = read(&dir.join(format!("{r}-native-{n}x.json")))?;
            traces.push(cumulative(&source)?);
        }
    }
    let top = traces
        .iter()
        .flat_map(|t| t.iter().map(|p| p.1))
        .fold(0f64, f64::max)
        * 1.08;
    let panels = root
        .titled(
            "Accumulated normal impulse through touchdown",
            ("sans-serif", 26),
        )?
        .split_evenly((1, 2));
    for (k, a) in panels.iter().enumerate() {
        let mut c = ChartBuilder::on(a)
            .margin(12)
            .caption(
                ["Always-dynamic history", "Switched history"][k],
                ("sans-serif", 19),
            )
            .x_label_area_size(48)
            .y_label_area_size(64)
            .build_cartesian_2d(0.0..5.0, 0.0..top)?;
        c.configure_mesh()
            .max_light_lines(1)
            .light_line_style(RGBColor(232, 236, 239))
            .x_desc("Time after the shared cut (ms)")
            .y_desc("Impulse (N·s)")
            .draw()?;
        for (i, (dt, color)) in [
            (500, RGBColor(52, 92, 162)),
            (250, RGBColor(12, 132, 116)),
            (125, RGBColor(199, 104, 31)),
        ]
        .into_iter()
        .enumerate()
        {
            c.draw_series(LineSeries::new(
                traces[3 * k + i].iter().copied(),
                color.stroke_width(2),
            ))?
            .label(format!("{dt} µs"))
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 16, y)], color.stroke_width(2)));
        }
        c.configure_series_labels()
            .position(SeriesLabelPosition::LowerRight)
            .background_style(WHITE.mix(0.9))
            .draw()?;
    }
    root.present()?;
    Ok(())
}

fn matched(audit: &std::path::Path, out: &std::path::Path) -> Result<()> {
    let report: Report = read(&audit.join("summary.json"))?;
    let root = SVGBackend::new(out, (1000, 470)).into_drawing_area();
    root.fill(&WHITE)?;
    let panels = root
        .titled(
            "Effect of the receiving contact history at matched motion",
            ("sans-serif", 25),
        )?
        .split_evenly((1, 2));
    for (m, panel) in panels.iter().enumerate() {
        let value = |r: &Run| {
            if m == 0 {
                r.first_outer_ns
            } else {
                r.window_ns
            }
        };
        let mut values = Vec::new();
        for (k, motion) in ["switched", "reference"].iter().enumerate() {
            let mut row = Vec::new();
            for n in [1, 2, 4] {
                let get = |receiver: &str| {
                    report
                        .runs
                        .iter()
                        .find(|r| r.receiver == receiver && r.motion == *motion && r.factor == n)
                        .map(value)
                        .ok_or("matched donor missing")
                };
                row.push((get("switched")? - get("reference")?) * 1e6);
            }
            values.push((k, row));
        }
        let limit = values
            .iter()
            .flat_map(|(_, r)| r.iter())
            .fold(0_f64, |a, b| a.max(b.abs()))
            * 1.25;
        let mut c = ChartBuilder::on(panel)
            .margin(12)
            .caption(
                ["First 500 µs outer step", "Fixed 5 ms interval"][m],
                ("sans-serif", 19),
            )
            .x_label_area_size(48)
            .y_label_area_size(65)
            .build_cartesian_2d(0.5..3.5, -limit..limit)?;
        c.configure_mesh()
            .max_light_lines(1)
            .light_line_style(RGBColor(232, 236, 239))
            .disable_x_mesh()
            .x_labels(3)
            .x_label_formatter(&|x: &f64| match *x {
                1. => "500".into(),
                2. => "250".into(),
                3. => "125".into(),
                _ => String::new(),
            })
            .x_desc("Native touchdown step (µs)")
            .y_desc("Switched − dynamic receiver (µN·s)")
            .draw()?;
        c.draw_series(LineSeries::new(
            [(0.5, 0.), (3.5, 0.)],
            RGBColor(105, 111, 116),
        ))?;
        for (k, row) in values {
            let (label, col, offset) = [
                ("Switched donor motion", RGBColor(13, 123, 114), -0.16),
                ("Dynamic donor motion", RGBColor(46, 86, 165), 0.16),
            ][k];
            c.draw_series(row.into_iter().enumerate().map(|(i, v)| {
                let x = i as f64 + 1. + offset;
                Rectangle::new([(x - 0.14, 0.), (x + 0.14, v)], col.filled())
            }))?
            .label(label)
            .legend(move |(x, y)| Rectangle::new([(x, y - 4), (x + 12, y + 4)], col.filled()));
        }
        if m == 0 {
            c.configure_series_labels()
                .position(SeriesLabelPosition::UpperLeft)
                .background_style(WHITE.mix(0.90))
                .draw()?;
        }
    }
    root.present()?;
    Ok(())
}

pub(crate) fn draw(audit: &std::path::Path, out: &std::path::Path) -> Result<()> {
    fs::create_dir_all(out)?;
    plot_impulses(audit, &out.join("impulses.svg"))?;
    onset(audit, &out.join("onset.svg"))?;
    matched(audit, &out.join("matched.svg"))?;
    latency(audit, &out.join("p95.svg"))
}
