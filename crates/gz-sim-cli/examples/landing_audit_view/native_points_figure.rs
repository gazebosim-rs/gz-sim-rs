// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native point impulses and the evolving foot motion during one actuator interval.
use super::*;
use plotters::prelude::Rectangle;

#[derive(Deserialize)]
pub(in super::super) struct Source {
    file: String,
    fixture: String,
    substeps: Vec<u32>,
}
#[derive(Deserialize)]
struct Dataset {
    actuator_tick_ns: u64,
    cases: Vec<Case>,
}
#[derive(Deserialize)]
struct Case {
    fixture: Fixture,
    cut_tick: u64,
    grids: Vec<Grid>,
}
#[derive(Deserialize)]
struct Fixture {
    name: String,
}
#[derive(Deserialize)]
struct Grid {
    substeps: u32,
    samples: Vec<Native>,
}
#[derive(Deserialize)]
struct Native {
    tick: u64,
    time_ns: u64,
    dt_ns: u64,
    before: Pose,
    after: Pose,
    normal_ns: f64,
    tangent_impulse_ns: [f64; 3],
    points: Vec<Point>,
}
#[derive(Deserialize)]
struct Pose {
    foot: Body,
}
#[derive(Deserialize)]
struct Point {
    foot_local_m: [f64; 3],
    normal_ns: f64,
    tangent_impulse_ns: [f64; 3],
}
fn primary<'a>(grid: &'a Grid, cut_us: u64, onset: &Sample, clock: u64) -> Result<Vec<&'a Native>> {
    let window: Vec<_> = grid
        .samples
        .iter()
        .filter(|r| r.tick == onset.tick)
        .collect();
    let mut last = cut_us * 1000;
    for n in &window {
        if last.checked_add(n.dt_ns) != Some(n.time_ns) {
            return Err("nonuniform native time accounting at primary".into());
        }
        last = n.time_ns;
    }
    if window.len() != grid.substeps as usize
        || last != cut_us * 1000 + clock
        || last != onset.time_us * 1000
        || (window.iter().map(|r| r.normal_ns).sum::<f64>() - onset.normal_impulse_ns).abs() > 1e-10
        || window.last().is_none_or(|s| s.after.foot != onset.foot)
    {
        return Err("native points disagree with matched outer trace".into());
    }
    Ok(window)
}

#[derive(Clone, Copy)]
struct Curve {
    time_us: f64,
    normal: f64,
    tangent_y: f64,
    spin: f64,
}
fn curve(samples: &[&Native], start: u64) -> Result<Vec<Curve>> {
    let mut work = vec![Curve {
        time_us: 0.,
        normal: 0.,
        tangent_y: 0.,
        spin: 0.,
    }];
    let initial = DVec3::from(
        samples[0]
            .before
            .foot
            .angular_velocity
            .ok_or("missing preimpact spin")?,
    );
    for r in samples {
        let prior = *work.last().ok_or("empty point curve")?;
        let spin = DVec3::from(
            r.after
                .foot
                .angular_velocity
                .ok_or("missing postsolve spin")?,
        );
        work.push(Curve {
            time_us: (r.time_ns - start) as f64 * 0.001,
            normal: prior.normal + r.normal_ns,
            tangent_y: prior.tangent_y + r.tangent_impulse_ns[1],
            spin: spin.distance(initial),
        });
    }
    Ok(work)
}

pub(in super::super) fn plot(
    dir: &Path,
    out: &Path,
    source: &Source,
    layout: &Layout,
    rows: &[&[Sample]],
    g: &Geometry,
) -> Result<()> {
    if source.substeps.len() != rows.len() {
        return Err("native point subdivisions differ from replay lane count".into());
    }
    let report: Dataset = read(&dir.join(&source.file))?;
    let case = report
        .cases
        .iter()
        .find(|r| r.fixture.name == source.fixture)
        .ok_or("point figure case missing")?;
    let cut = &rows[0][0];
    if case.cut_tick != cut.tick {
        return Err("point figure cut differs from replay".into());
    }
    let grids: Vec<_> = source
        .substeps
        .iter()
        .map(|n| {
            case.grids
                .iter()
                .find(|r| r.substeps == *n)
                .ok_or("point grid missing")
        })
        .collect::<std::result::Result<_, _>>()?;
    let start = cut.time_us * 1000;
    let windows: Vec<_> = grids
        .iter()
        .enumerate()
        .map(|(k, g)| primary(g, cut.time_us, &rows[k][1], report.actuator_tick_ns))
        .collect::<Result<_>>()?;
    let first: Vec<_> = windows
        .iter()
        .map(|w| {
            w.iter()
                .copied()
                .find(|r| r.normal_ns > 0.)
                .ok_or("no first positive native load in primary tick")
        })
        .collect::<std::result::Result<_, _>>()?;
    let curves: Vec<_> = windows
        .iter()
        .map(|w| curve(w, start))
        .collect::<Result<_>>()?;
    let path = out.join("native-points.svg");
    let root = SVGBackend::new(&path, (1080, 1120)).into_drawing_area();
    root.fill(&WHITE)?;
    let (maps, graphs) = root
        .titled(
            "Outer-support impact at native solver resolution",
            ("sans-serif", 26),
        )?
        .split_vertically(390);
    let (maps, caption) = maps.split_vertically(351);
    caption.draw(&plotters::prelude::Text::new(
        "Circle area follows point normal impulse. Lines give tangential impulse in the incoming sole frame.",
        (54,23),("sans-serif",16)))?;
    let max_normal = first
        .iter()
        .flat_map(|s| &s.points)
        .map(|p| p.normal_ns)
        .fold(0_f64, f64::max)
        .max(1e-12);
    let max_tangent = first
        .iter()
        .flat_map(|s| &s.points)
        .map(|p| DVec3::from(p.tangent_impulse_ns).length())
        .fold(0_f64, f64::max)
        .max(1e-12);
    let hx = g.foot_half_extents[0] * 1000.;
    let hy = g.foot_half_extents[1] * 1000.;
    let scale = hy * 0.33 / max_tangent;
    for (k, panel) in maps.split_evenly((1, rows.len())).into_iter().enumerate() {
        let sample = first[k];
        let mut chart = ChartBuilder::on(&panel)
            .margin(10)
            .caption(
                format!(
                    "{} · {:.2} µs",
                    layout.lanes[k].name,
                    (sample.time_ns - start) as f64 * 0.001
                ),
                ("sans-serif", 18),
            )
            .x_label_area_size(43)
            .y_label_area_size(53)
            .build_cartesian_2d(-1.32 * hx..hx * 1.32, -hy * 1.58..hy * 1.58)?;
        chart
            .configure_mesh()
            .x_desc("Sole x (mm)")
            .y_desc("Sole y (mm)")
            .max_light_lines(1)
            .draw()?;
        chart.draw_series(std::iter::once(Rectangle::new(
            [(-hx, -hy), (hx, hy)],
            RGBColor(90, 99, 105).stroke_width(2),
        )))?;
        let q = DQuat::from_array(sample.before.foot.orientation).normalize();
        for p in &sample.points {
            let pos = DVec3::from(p.foot_local_m) * 1000.;
            let tangent = q.conjugate() * DVec3::from(p.tangent_impulse_ns) * scale;
            let end = pos + tangent;
            chart.draw_series(std::iter::once(Circle::new(
                (pos.x, pos.y),
                ((p.normal_ns / max_normal).sqrt() * 13.).round() as i32,
                INKS[k].mix(0.70).filled(),
            )))?;
            chart.draw_series(std::iter::once(PathElement::new(
                [(pos.x, pos.y), (end.x, end.y)],
                RGBColor(25, 35, 41).stroke_width(2),
            )))?;
            chart.draw_series(std::iter::once(Circle::new(
                (end.x, end.y),
                2,
                RGBColor(25, 35, 41).filled(),
            )))?;
        }
    }
    for (j, panel) in graphs.split_evenly((3, 1)).into_iter().enumerate() {
        let (title, unit) = match j {
            0 => (
                "Cumulative normal impulse in this actuator tick",
                "Impulse (N·s)",
            ),
            1 => (
                "Cumulative signed tangential impulse along world y",
                "Impulse (N·s)",
            ),
            _ => (
                "Change of foot angular velocity since the incoming cut",
                "Vector difference (rad/s)",
            ),
        };
        let val = |p: &Curve| match j {
            0 => p.normal,
            1 => p.tangent_y,
            _ => p.spin,
        };
        let (lo, hi) = curves
            .iter()
            .flatten()
            .map(val)
            .fold((0_f64, 0_f64), |(lo, hi), x| (lo.min(x), hi.max(x)));
        let margin = (hi - lo).max(1e-10) * 0.12;
        let mut chart = ChartBuilder::on(&panel)
            .margin(9)
            .x_label_area_size(34)
            .y_label_area_size(86)
            .caption(title, ("sans-serif", 18))
            .build_cartesian_2d(
                0.0..(report.actuator_tick_ns as f64 * 1e-3),
                lo - margin..hi + margin,
            )?;
        chart
            .configure_mesh()
            .x_desc("Native solve endpoint since cut (µs)")
            .y_desc(unit)
            .max_light_lines(1)
            .draw()?;
        for k in (0..rows.len()).rev() {
            let color = INKS[k];
            // Steps remain at their last committed value until a new native receipt arrives.
            let mut points = vec![(0., val(&curves[k][0]))];
            for pair in curves[k].windows(2) {
                points.push((pair[1].time_us, val(&pair[0])));
                points.push((pair[1].time_us, val(&pair[1])));
            }
            chart
                .draw_series(LineSeries::new(points, color.stroke_width(k as u32 + 1)))?
                .label(&layout.lanes[k].name)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 18, y)], color.stroke_width(3))
                });
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperLeft)
            .background_style(WHITE.mix(0.94))
            .draw()?;
    }
    root.present()?;
    Ok(())
}
