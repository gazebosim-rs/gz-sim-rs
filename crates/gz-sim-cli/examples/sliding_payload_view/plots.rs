#[path = "outcomes.rs"]
mod outcomes;
use super::{Body, Result, Sample, Trace, read};
use glam::{DQuat, DVec3};
use plotters::{coord::Shift, prelude::*};
use serde::Deserialize;
use std::{fs, path::Path};

const TEAL: RGBColor = RGBColor(15, 102, 107);
const AMBER: RGBColor = RGBColor(211, 123, 38);
const INK: RGBColor = RGBColor(47, 61, 78);
const GRID: RGBColor = RGBColor(223, 230, 232);

#[derive(Deserialize)]
struct Trial {
    dt_s: f64,
    offset_y_m: f64,
    first_promotion_tick: Option<u64>,
    first_contact_tick: Option<u64>,
    dynamic_ticks: u64,
    steps: u64,
}

fn series(
    area: &DrawingArea<SVGBackend, Shift>,
    trace: &Trace,
    title: &str,
    units: &str,
    values: impl Fn(&Sample) -> [f64; 3],
) -> Result<()> {
    let samples = &trace.samples;
    let time = |s: &Sample| s.tick as f64 * trace.dt_s * 1e3;
    let last = time(samples.last().ok_or("empty trace")?);
    let mut low = 0.0_f64;
    let mut high = 0.0_f64;
    for v in samples.iter().flat_map(&values) {
        if !v.is_finite() {
            return Err("nonfinite measured series".into());
        }
        low = low.min(v);
        high = high.max(v);
    }
    let pad = ((high - low) * 0.12).max(0.02);
    let range = (low - pad)..(high + pad);
    let mut chart = ChartBuilder::on(area)
        .margin(12)
        .caption(title, ("sans-serif", 21))
        .x_label_area_size(40)
        .y_label_area_size(67)
        .build_cartesian_2d(0.0..last, range.clone())?;
    chart
        .configure_mesh()
        .x_desc("Time since launch (ms)")
        .y_desc(units)
        .max_light_lines(1)
        .light_line_style(GRID)
        .draw()?;
    for (tick, color) in [
        (
            samples.iter().find(|s| s.tier == "dynamic"),
            RGBColor(125, 173, 139),
        ),
        (
            samples.iter().find(|s| s.automatic_impulse_ns > 0.),
            RGBColor(210, 168, 101),
        ),
    ] {
        if let Some(tick) = tick {
            chart.draw_series(LineSeries::new(
                [(time(tick), range.start), (time(tick), range.end)],
                color,
            ))?;
        }
    }
    for (lane, name, color) in [(2, "Near miss", AMBER), (0, "Automatic hit", TEAL)] {
        chart
            .draw_series(LineSeries::new(
                samples.iter().map(|s| (time(s), values(s)[lane])),
                color.stroke_width(3),
            ))?
            .label(name)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 19, y)], color.stroke_width(3)));
    }
    chart
        .draw_series(
            samples
                .iter()
                .step_by((samples.len() / 35).max(1))
                .map(|s| Circle::new((time(s), values(s)[1]), 3, INK.stroke_width(1))),
        )?
        .label("Always dynamic hit")
        .legend(|(x, y)| Circle::new((x + 9, y), 3, INK.stroke_width(1)));
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperRight)
        .background_style(WHITE.mix(0.90))
        .border_style(GRID)
        .draw()?;
    Ok(())
}

fn impulses(trace: &Trace, output: &Path) -> Result<()> {
    let mut totals = [0.0; 4];
    let mut data = Vec::new();
    for s in &trace.samples {
        // Sample zero still carries the last settling impulse.
        if s.tick > 0 {
            for (sum, j) in totals.iter_mut().zip([
                s.automatic_floor_impulse_ns,
                s.reference_floor_impulse_ns,
                s.automatic_impulse_ns,
                s.reference_impulse_ns,
            ]) {
                if !j.is_finite() || j < 0.0 {
                    return Err("invalid contact impulse".into());
                }
                *sum += j;
            }
        }
        data.push((s.tick as f64 * trace.dt_s * 1e3, totals));
    }
    let max = totals.into_iter().fold(0.0_f64, f64::max);
    let root = SVGBackend::new(output, (1050, 390)).into_drawing_area();
    root.fill(&WHITE)?;
    let mut chart = ChartBuilder::on(&root)
        .caption(
            "Cumulative normal impulse on the sliding payload",
            ("sans-serif", 24),
        )
        .margin(14)
        .x_label_area_size(40)
        .y_label_area_size(75)
        .build_cartesian_2d(0.0..data.last().ok_or("empty trace")?.0, 0.0..max * 1.12)?;
    chart
        .configure_mesh()
        .x_desc("Time since launch (ms)")
        .y_desc("Summed normal impulse (N·s)")
        .light_line_style(GRID)
        .max_light_lines(1)
        .draw()?;
    for (i, color, role) in [
        (0, TEAL, "Floor → payload"),
        (2, AMBER, "Arm tip → payload"),
    ] {
        chart
            .draw_series(LineSeries::new(
                data.iter().map(|(t, totals)| (*t, totals[i])),
                color.stroke_width(3),
            ))?
            .label(role)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 18, y)], color.stroke_width(3)));
        chart.draw_series(
            data.iter()
                .step_by(20)
                .map(|(t, totals)| Circle::new((*t, totals[i + 1]), 3, INK.stroke_width(1))),
        )?;
    }
    chart
        .draw_series(std::iter::empty::<Circle<(f64, f64), i32>>())?
        .label("Always dynamic sample")
        .legend(|(x, y)| Circle::new((x + 9, y), 3, INK.stroke_width(1)));
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.92))
        .border_style(GRID)
        .draw()?;
    root.present()?;
    Ok(())
}

/// Project the eight rotated box corners and keep the convex boundary in the floor plane.
fn footprint(body: &Body, size: [f64; 3]) -> Vec<(f64, f64)> {
    let half = DVec3::from(size) * 0.5;
    let rot = DQuat::from_array(body.orientation);
    let mut points = Vec::with_capacity(8);
    for x in [-half.x, half.x] {
        for y in [-half.y, half.y] {
            for z in [-half.z, half.z] {
                let p = DVec3::from(body.position) + rot * DVec3::new(x, y, z);
                points.push((p.x, p.y));
            }
        }
    }
    points.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)));
    let half_hull = |points: &[(f64, f64)]| {
        let mut hull = Vec::new();
        for &p in points {
            while hull.len() > 1 {
                let (a, b): ((f64, f64), (f64, f64)) = (hull[hull.len() - 2], hull[hull.len() - 1]);
                if (b.0 - a.0) * (p.1 - a.1) - (b.1 - a.1) * (p.0 - a.0) > 0. {
                    break;
                }
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
        hull
    };
    let mut polygon = half_hull(&points);
    points.reverse();
    polygon.extend(half_hull(&points));
    polygon.push(polygon[0]);
    polygon
}

fn paths(trace: &Trace, output: &Path) -> Result<()> {
    let first = trace.samples.first().ok_or("empty path")?;
    let last = trace.samples.last().ok_or("empty path")?;
    let impact = trace
        .samples
        .iter()
        .find(|s| s.automatic_impulse_ns > 0.)
        .ok_or("missing impact sample")?;
    let pass = trace
        .samples
        .iter()
        .min_by(|a, b| {
            let dx = |s: &Sample| (s.near_miss[2].position[0] - s.near_miss[1].position[0]).abs();
            dx(a).total_cmp(&dx(b))
        })
        .ok_or("missing pass")?;
    let root = SVGBackend::new(output, (1120, 474)).into_drawing_area();
    root.fill(&WHITE)?;
    let area = root.titled(
        "Footprints of the payload and arm on the floor",
        ("sans-serif", 25),
    )?;
    let (area, footer) = area.split_vertically(403);
    let panels = area.split_evenly((1, 2));
    type Lane = fn(&Sample) -> &[Body; 3];
    let lanes: [(&str, &Sample, Lane); 2] = [
        ("Strike", impact, |s| &s.automatic),
        ("Near miss", pass, |s| &s.near_miss),
    ];
    for ((label, event, lane), panel) in lanes.into_iter().zip(&panels) {
        let mut chart = ChartBuilder::on(panel)
            .margin(10)
            .caption(
                format!("{label} · highlighted at {} ms", event.tick),
                ("sans-serif", 20),
            )
            .x_label_area_size(40)
            .y_label_area_size(45)
            .build_cartesian_2d(-1.4..0.63, -0.35..1.51)?;
        chart
            .configure_mesh()
            .x_desc("World X (m)")
            .y_desc("World Y (m)")
            .max_light_lines(1)
            .light_line_style(GRID)
            .draw()?;
        for (i, size, color) in [
            (0, [0.28; 3], RGBColor(60, 112, 159)),
            (1, [0.28; 3], TEAL),
            (2, [0.32, 0.26, 0.32], AMBER),
        ] {
            chart.draw_series(LineSeries::new(
                trace.samples.iter().map(|s| {
                    let p = lane(s)[i].position;
                    (p[0], p[1])
                }),
                color.stroke_width(2),
            ))?;
            for edge in [first, last] {
                chart.draw_series(std::iter::once(PathElement::new(
                    footprint(&lane(edge)[i], size),
                    color.mix(0.48).stroke_width(1),
                )))?;
            }
            let polygon = footprint(&lane(event)[i], size);
            chart.draw_series(std::iter::once(Polygon::new(
                polygon.clone(),
                color.mix(0.35).filled(),
            )))?;
            chart.draw_series(std::iter::once(PathElement::new(
                polygon,
                color.stroke_width(2),
            )))?;
        }
    }
    footer.draw(&Text::new("The orange payload passes the teal tip and blue base. Lines trace centers. Fine outlines show launch and 500 ms.",(45,18),("sans-serif",16).into_font().color(&INK)))?;
    root.present()?;
    Ok(())
}

fn response(trace: &Trace, out: &Path) -> Result<()> {
    let root = SVGBackend::new(out, (1110, 625)).into_drawing_area();
    root.fill(&WHITE)?;
    let body = root.titled(
        "Sliding payload and supported two-link arm",
        ("sans-serif", 26),
    )?;
    let panels = body.split_evenly((2, 1));
    series(
        &panels[0],
        trace,
        "Payload velocity along the floor",
        "m/s",
        |s| {
            [
                s.automatic[2].velocity[0],
                s.reference[2].velocity[0],
                s.near_miss[2].velocity[0],
            ]
        },
    )?;
    series(&panels[1], trace, "Hinge angular velocity", "rad/s", |s| {
        [
            s.hinge_rate_rad_s,
            s.reference_hinge_rate_rad_s,
            s.near_miss_hinge_rate_rad_s,
        ]
    })?;
    root.present()?;
    Ok(())
}

fn offset_matrix(source: &Path, baseline: &Path, out: &Path) -> Result<()> {
    let current: Vec<Trial> = read(&source.join("matrix.json"))?;
    let prior: Vec<Trial> = read(&baseline.join("matrix.json"))?;
    let mut dts: Vec<_> = current.iter().map(|r| r.dt_s).collect();
    dts.sort_by(f64::total_cmp);
    dts.dedup();
    if current.is_empty() || current.len() != prior.len() {
        return Err("incomplete offset matrix".into());
    }
    let root = SVGBackend::new(out, (1120, 90 + 245 * dts.len() as u32)).into_drawing_area();
    root.fill(&WHITE)?;
    let header = root.titled(
        "Time spent in dynamic physics during the 500 ms pass",
        ("sans-serif", 26),
    )?;
    let (body, footer) = header.split_vertically(header.dim_in_pixel().1 - 37);
    footer.draw(&Text::new("Filled diamonds mark offsets with a measured payload–tip impulse. Open circles are passes without an impulse.",(55,18),("sans-serif",15).into_font().color(&INK)))?;
    let panels = body.split_evenly((dts.len(), 1));
    for (panel, dt) in panels.iter().zip(dts) {
        let mut rows: Vec<_> = current.iter().filter(|r| r.dt_s == dt).collect();
        rows.sort_by(|a, b| a.offset_y_m.total_cmp(&b.offset_y_m));
        let x0 = rows[0].offset_y_m - 0.04;
        let x1 = rows.last().unwrap().offset_y_m + 0.04;
        let max = rows.iter().map(|r| r.steps).max().unwrap() as f64 * dt * 1e3;
        let mut chart = ChartBuilder::on(panel)
            .margin(8)
            .caption(format!("{} ms physics step", dt * 1e3), ("sans-serif", 19))
            .x_label_area_size(40)
            .y_label_area_size(66)
            .build_cartesian_2d(x0..x1, -20.0..(max + 18.0))?;
        chart
            .configure_mesh()
            .x_desc("Payload launch Y (m)")
            .y_desc("Dynamic time (ms)")
            .max_light_lines(1)
            .light_line_style(GRID)
            .draw()?;
        for (set, color, name) in [
            (&prior, AMBER, "Previous predictor"),
            (&current, TEAL, "Supported-contact predictor"),
        ] {
            let mut points = Vec::new();
            for r in &rows {
                let old = set
                    .iter()
                    .find(|s| s.dt_s == dt && s.offset_y_m == r.offset_y_m)
                    .ok_or("baseline missing matching trial")?;
                if old.first_promotion_tick.is_none() && old.dynamic_ticks != 0 {
                    return Err("inconsistent tier measurement".into());
                }
                points.push((
                    r.offset_y_m,
                    old.dynamic_ticks as f64 * dt * 1e3,
                    old.first_contact_tick.is_some(),
                ));
            }
            chart
                .draw_series(LineSeries::new(
                    points.iter().map(|&(x, y, _)| (x, y)),
                    color.stroke_width(2),
                ))?
                .label(name)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 18, y)], color.stroke_width(2))
                });
            for (x, y, hit) in points {
                if hit {
                    chart.draw_series(std::iter::once(
                        EmptyElement::at((x, y))
                            + Polygon::new([(0, -5), (5, 0), (0, 5), (-5, 0)], color.filled()),
                    ))?;
                } else {
                    chart.draw_series(std::iter::once(Circle::new(
                        (x, y),
                        5,
                        color.stroke_width(2),
                    )))?;
                }
            }
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperRight)
            .background_style(WHITE.mix(0.93))
            .border_style(GRID)
            .draw()?;
    }
    root.present()?;
    Ok(())
}

#[derive(Deserialize)]
struct Costs {
    paired_trials: Option<usize>,
    rows: Vec<LayoutCost>,
}
#[derive(Deserialize)]
struct LayoutCost {
    layout: String,
    automatic: Timing,
    always_dynamic: Timing,
    distant_fixed_bodies: usize,
}
#[derive(Deserialize)]
struct Timing {
    p95_ns: f64,
    samples: u64,
}

fn cost(source: &Path, out: &Path) -> Result<()> {
    let before: Costs = read(&source.join("cost-before.json"))?;
    let after: Costs = read(&source.join("cost-after.json"))?;
    if before.rows.len() != after.rows.len() || after.rows.is_empty() {
        return Err("timing layouts disagree".into());
    }
    let max = before
        .rows
        .iter()
        .chain(&after.rows)
        .flat_map(|r| [r.automatic.p95_ns, r.always_dynamic.p95_ns])
        .fold(0.0_f64, f64::max)
        / 1000.;
    let root = SVGBackend::new(out, (1100, 65 + after.rows.len() as u32 * 205)).into_drawing_area();
    root.fill(&WHITE)?;
    if after.paired_trials != before.paired_trials {
        return Err("mismatched timing aggregations".into());
    }
    let title = after.paired_trials.map_or_else(
        || "Local CPU step latency at p95".to_string(),
        |n| format!("Local CPU step latency · median of {n} run p95s"),
    );
    let area = root.titled(&title, ("sans-serif", 26))?;
    for (r, panel) in after
        .rows
        .iter()
        .zip(area.split_evenly((after.rows.len(), 1)))
    {
        let old = before
            .rows
            .iter()
            .find(|old| {
                old.layout == r.layout && old.distant_fixed_bodies == r.distant_fixed_bodies
            })
            .ok_or("unmatched timing layout")?;
        let title = format!(
            "{} · {} extra fixed bodies · {} timed steps per series",
            r.layout.replace('_', " "),
            r.distant_fixed_bodies,
            r.automatic.samples
        );
        let mut chart = ChartBuilder::on(&panel)
            .caption(title, ("sans-serif", 19))
            .margin(6)
            .x_label_area_size(37)
            .y_label_area_size(155)
            .build_cartesian_2d(0.0..max * 1.21, -0.5..3.5)?;
        let labels = [
            "Reference / after",
            "Reference / before",
            "Automatic / after",
            "Automatic / before",
        ];
        chart
            .configure_mesh()
            .x_desc("p95 step time (µs)")
            .y_labels(4)
            .y_label_formatter(&|y: &f64| {
                let n = y.round();
                if (y - n).abs() < 0.01 && (0.0..=3.0).contains(&n) {
                    labels[n as usize].to_string()
                } else {
                    String::new()
                }
            })
            .disable_y_mesh()
            .light_line_style(GRID)
            .max_light_lines(1)
            .draw()?;
        for (i, (measurement, color)) in [
            (&r.always_dynamic, INK),
            (&old.always_dynamic, RGBColor(137, 147, 159)),
            (&r.automatic, TEAL),
            (&old.automatic, AMBER),
        ]
        .into_iter()
        .enumerate()
        {
            let y = i as f64;
            let value = measurement.p95_ns / 1000.;
            chart.draw_series(std::iter::once(Rectangle::new(
                [(0., y - 0.24), (value, y + 0.24)],
                color.filled(),
            )))?;
            chart.draw_series(std::iter::once(Text::new(
                format!("{value:.1}"),
                (value + max * 0.014, y + 0.09),
                ("sans-serif", 15),
            )))?;
        }
    }
    root.present()?;
    Ok(())
}

#[derive(Deserialize)]
struct BatchCost {
    targets: u32,
    batches: u32,
    median_batch_ns: f64,
    p95_batch_ns: f64,
}

fn query_cost(source: &Path, out: &Path) -> Result<()> {
    let old: BatchCost = read(&source.join("query-before.json"))?;
    let new: BatchCost = read(&source.join("query-after.json"))?;
    if old.targets != new.targets || old.batches != new.batches {
        return Err("query workloads disagree".into());
    }
    let max = old.p95_batch_ns.max(new.p95_batch_ns) / 1000.;
    let root = SVGBackend::new(out, (1010, 285)).into_drawing_area();
    root.fill(&WHITE)?;
    let area = root.titled(
        &format!(
            "Native sweep queries · {} targets per batch · {} batches",
            old.targets, old.batches
        ),
        ("sans-serif", 24),
    )?;
    let mut chart = ChartBuilder::on(&area)
        .margin(12)
        .x_label_area_size(42)
        .y_label_area_size(90)
        .build_cartesian_2d(0.0..max * 1.13, -0.3..1.48)?;
    chart
        .configure_mesh()
        .x_desc("Time per batch (µs)")
        .y_labels(4)
        .y_label_formatter(&|y: &f64| {
            if y.abs() < 0.01 {
                "After".to_string()
            } else if (y - 1.).abs() < 0.01 {
                "Before".to_string()
            } else {
                String::new()
            }
        })
        .disable_y_mesh()
        .light_line_style(GRID)
        .max_light_lines(1)
        .draw()?;
    for (sample, y, color) in [(old, 1., AMBER), (new, 0., TEAL)] {
        let (median, p95) = (sample.median_batch_ns / 1000., sample.p95_batch_ns / 1000.);
        chart.draw_series(LineSeries::new(
            [(median, y), (p95, y)],
            color.stroke_width(4),
        ))?;
        chart.draw_series(std::iter::once(Circle::new((median, y), 6, color.filled())))?;
        chart.draw_series(std::iter::once(
            EmptyElement::at((p95, y))
                + Polygon::new([(0, -7), (7, 0), (0, 7), (-7, 0)], color.filled()),
        ))?;
        chart.draw_series(std::iter::once(Text::new(
            format!("Median {median:.1} µs   ·   p95 {p95:.1} µs"),
            (max * 0.025, y + 0.30),
            ("sans-serif", 16).into_font().color(&INK),
        )))?;
    }
    root.present()?;
    Ok(())
}

pub(super) fn render(source: &Path, baseline: &Path, output: &Path) -> Result<()> {
    fs::create_dir_all(output)?;
    let summary = read(&source.join("summary.json"))?;
    outcomes::inclined(&summary, &output.join("inclined.svg"))?;
    outcomes::admission(&summary, &output.join("admission.svg"))?;
    let trace = read(&source.join("trajectory.json"))?;
    response(&trace, &output.join("response.svg"))?;
    paths(&trace, &output.join("paths.svg"))?;
    impulses(&trace, &output.join("impulses.svg"))?;
    offset_matrix(source, baseline, &output.join("offsets.svg"))?;
    cost(source, &output.join("latency.svg"))?;
    query_cost(source, &output.join("queries.svg"))?;
    Ok(())
}
