// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    Body, Result, read,
    replay::{self, VIEW},
    soft_replay::Frame,
};
use glam::DVec3;
use plotters::prelude::*;
use serde::Deserialize;
use serde_json::json;
use std::{fs, path::Path, time::Duration};

#[derive(Deserialize)]
struct Trace {
    dt_s: f64,
    settle_s: f64,
    samples: Vec<Step>,
}
#[derive(Deserialize)]
struct Step {
    tick: u64,
    tier: String,
    automatic: [Body; 3],
    reference: [Body; 3],
    automatic_impulse_ns: f64,
    automatic_arm_floor_cache: [Cache; 2],
    automatic_arm_floor_impulses_ns: [f64; 2],
    reference_arm_floor_impulses_ns: [f64; 2],
}

#[derive(Deserialize)]
struct Cache {
    normal_ns: f64,
    tangent_on_arm_ns: [f64; 3],
}
#[derive(Deserialize)]
struct Handoff {
    before_switch_cache: [Cache; 2],
    after_switch_before_solve_cache: [Cache; 2],
    last_prescribed_tick: u64,
    first_resumed_tick: u64,
}
#[derive(Deserialize)]
struct Summary {
    settled_arm_floor_cache: [Cache; 2],
    switch_probe: Handoff,
}

fn error_mm(step: &Step) -> f64 {
    step.automatic
        .iter()
        .zip(&step.reference)
        .map(|(a, b)| DVec3::from(a.position).distance(b.position.into()) * 1000.)
        .fold(0., f64::max)
}
fn peak(trace: &Trace) -> Result<(u64, f64)> {
    trace
        .samples
        .iter()
        .map(|s| (s.tick, error_mm(s)))
        .max_by(|a, b| a.1.total_cmp(&b.1))
        .ok_or_else(|| "empty grazing trace".into())
}
fn events(trace: &Trace) -> Result<(u64, u64)> {
    let promote = trace
        .samples
        .iter()
        .find(|s| s.tier == "dynamic")
        .ok_or("missing promotion")?
        .tick;
    let contact = trace
        .samples
        .iter()
        .find(|s| s.automatic_impulse_ns > 0.)
        .ok_or("missing payload impulse")?
        .tick;
    Ok((promote, contact))
}

pub(super) fn export(measurements: &Path, output: &Path) -> Result<()> {
    let old: Trace = read(&measurements.join("baseline/trajectory.json"))?;
    let new: Trace = read(&measurements.join("updated/trajectory.json"))?;
    let summary: Summary = read(&measurements.join("updated/summary.json"))?;
    if old.dt_s != 0.00025
        || new.dt_s != old.dt_s
        || new.settle_s != old.settle_s
        || new.samples.len() != old.samples.len()
        || new.samples.is_empty()
    {
        return Err("expected two aligned 250 µs grazing traces".into());
    }
    let mut min = DVec3::splat(f64::INFINITY);
    let mut max = DVec3::splat(f64::NEG_INFINITY);
    for (i, (a, b)) in old.samples.iter().zip(&new.samples).enumerate() {
        if a.tick != i as u64
            || b.tick != a.tick
            || a.reference.iter().zip(&b.reference).any(|(x, y)| {
                x.position != y.position
                    || x.orientation != y.orientation
                    || x.velocity != y.velocity
            })
            || a.reference_arm_floor_impulses_ns != b.reference_arm_floor_impulses_ns
        {
            return Err(format!("reference or clock mismatch at tick {i}").into());
        }
        for v in a.automatic.iter().chain(&b.automatic).chain(&b.reference) {
            let pos = DVec3::from(v.position);
            min = min.min(pos);
            max = max.max(pos);
        }
    }
    let (old_peak, old_mm) = peak(&old)?;
    let (new_peak, new_mm) = peak(&new)?;
    let (old_switch, old_hit) = events(&old)?;
    let (switch, hit) = events(&new)?;
    let width = max.x - min.x + 0.65;
    let spacing = width + 0.18;
    let height = max.y - min.y + 0.66;
    let floor = DVec3::new((min.x + max.x) * 0.5, (min.y + max.y) * 0.5, -0.1);
    let y = min.y - 0.61;
    let x = (min.x + max.x) * 0.5;
    let extent = width * 0.74;
    let scale = old_mm.max(new_mm).ceil().max(1.) as u64;
    let visuals = replay::visuals()?;
    let milestones = [old_switch, old_hit, switch, hit, old_peak, new_peak];
    let mut ticks = Vec::new();
    let mut frames = Vec::new();
    let mut running = [0_f64; 2];
    for (a, b) in old.samples.iter().zip(&new.samples) {
        for (v, s) in running.iter_mut().zip([a, b]) {
            *v = v.max(error_mm(s));
        }
        if b.tick % 48 != 0
            && b.tick + 1 != new.samples.len() as u64
            && !milestones.iter().any(|t| t.abs_diff(b.tick) <= 1)
        {
            continue;
        }
        let mut frame = Frame::default();
        for (lane, shift, bodies, err, color) in [
            (
                "previous",
                spacing,
                &a.automatic,
                running[0],
                [0.96, 0.45, 0.10, 1.],
            ),
            (
                "preserved",
                0.,
                &b.automatic,
                running[1],
                [0.17, 0.78, 0.66, 1.],
            ),
            (
                "reference",
                -spacing,
                &b.reference,
                0.,
                [0.65, 0.73, 0.81, 1.],
            ),
        ] {
            let point = |p: DVec3| VIEW * p + DVec3::Y * shift;
            frame.add(
                &format!("{lane}_floor"),
                point(floor),
                VIEW,
                json!({"kind":"box","size":[width,height,0.2]}),
                [0.30, 0.36, 0.41, 1.],
            );
            replay::add_bodies(&mut frame, lane, bodies, shift, &visuals)?;
            let mut indicator = |name: String, x: f64, z: f64, size: [f64; 3], color| {
                frame.add(
                    &format!("{lane}_{name}"),
                    point(DVec3::new(x, y, z)),
                    VIEW,
                    json!({"kind":"box","size":size}),
                    color,
                );
            };
            let graphite = [0.45, 0.53, 0.59, 1.];
            indicator(
                "error_scale".to_owned(),
                x,
                0.,
                [extent, 0.019, 0.025],
                graphite,
            );
            for mm in 0..=scale {
                indicator(
                    format!("{mm}_mm"),
                    x + extent * (mm as f64 / scale as f64 - 0.5),
                    0.012,
                    [0.015, 0.06, 0.02],
                    graphite,
                );
            }
            indicator(
                "max_center_error".to_owned(),
                x + extent * (err / scale as f64 - 0.5),
                0.06,
                [0.055, 0.085, 0.05],
                color,
            );
        }
        ticks.push(b.tick);
        frames.push(frame.finish(Duration::from_micros(b.tick * 250), frames.len() + 1));
    }
    fs::create_dir_all(output)?;
    gz_sim_cli::export_archive(&output.join("replay"), Default::default(), frames)?;
    fs::write(
        output.join("frames.json"),
        serde_json::to_vec_pretty(&json!({
            "ticks":ticks,"dt_us":250,"settle_s":new.settle_s,"promotion_tick":switch,"first_impulse_tick":hit,
            "old_peak_tick":old_peak,"new_peak_tick":new_peak,"old_peak_mm":old_mm,"new_peak_mm":new_mm,
            "spacing_m":spacing,"clipped_display_floor_m":[width,height,0.2],"ruler_x":x,"ruler_y":y,"ruler_width":extent,"ruler_max_mm":scale
        }))?,
    )?;
    plots(&old, &new, output, switch, hit, old_mm, new_mm)?;
    cache_plot(&old, &summary, output)
}

fn plots(
    old: &Trace,
    new: &Trace,
    output: &Path,
    promotion: u64,
    impact: u64,
    old_mm: f64,
    new_mm: f64,
) -> Result<()> {
    let path = output.join("grazing.svg");
    let root = SVGBackend::new(&path, (1050, 800)).into_drawing_area();
    root.fill(&WHITE)?;
    let title =
        format!("Grazing strike    |    peak body-center error {old_mm:.4} → {new_mm:.4} mm");
    let area = root.titled(&title, ("sans-serif", 24))?;
    let panels = area.split_evenly((3, 1));
    let time = |s: &Step| s.tick as f64 * new.dt_s * 1000.;
    let mut chart = ChartBuilder::on(&panels[0])
        .margin(10)
        .margin_right(30)
        .caption(
            "Largest current center-position error across the two links and payload",
            ("sans-serif", 18),
        )
        .x_label_area_size(35)
        .y_label_area_size(85)
        .build_cartesian_2d(
            0.0..time(new.samples.last().unwrap()),
            0.0..(old_mm.max(new_mm) * 1.13).max(0.01),
        )?;
    chart
        .configure_mesh()
        .max_light_lines(1)
        .light_line_style(RGBColor(232, 236, 238))
        .x_desc("Time since launch (ms)")
        .y_desc("Error (mm)")
        .draw()?;
    let colors = [
        RGBColor(199, 103, 35),
        RGBColor(19, 129, 117),
        RGBColor(64, 75, 96),
    ];
    for (i, trace) in [old, new].iter().enumerate() {
        let color = colors[i];
        chart
            .draw_series(LineSeries::new(
                trace.samples.iter().map(|s| (time(s), error_mm(s))),
                color.stroke_width(2),
            ))?
            .label(["Previous", "Preserved"][i])
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 18, y)], color.stroke_width(2)));
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.9))
        .draw()?;
    let start = promotion;
    let end = (promotion + 10).min(new.samples.last().unwrap().tick);
    for body in 0..2 {
        let delta = |s: &Step| {
            (s.automatic_arm_floor_impulses_ns[body] - s.reference_arm_floor_impulses_ns[body])
                * 1000.
        };
        let mut low = 0.0_f64;
        let mut high = 0.0_f64;
        for trace in [old, new] {
            for step in &trace.samples[start as usize..=end as usize] {
                let d = delta(step);
                low = low.min(d);
                high = high.max(d);
            }
        }
        let title = format!(
            "{} link   ·   floor impulse minus always-dynamic impulse after promotion",
            ["Base", "Tip"][body]
        );
        let pad = ((high - low) * 0.15).max(0.001);
        let mut c = ChartBuilder::on(&panels[body + 1])
            .margin(10)
            .margin_right(30)
            .caption(title, ("sans-serif", 17))
            .x_label_area_size(38)
            .y_label_area_size(85)
            .build_cartesian_2d(
                start as f64 * 0.25..end as f64 * 0.25,
                (low - pad)..(high + pad),
            )?;
        c.configure_mesh()
            .max_light_lines(1)
            .light_line_style(RGBColor(232, 236, 238))
            .x_desc(format!(
                "Time since launch (ms)   ·   promotion {:.2} ms, payload impact {:.2} ms",
                promotion as f64 * 0.25,
                impact as f64 * 0.25
            ))
            .y_desc("Impulse error (mN·s)")
            .x_label_formatter(&|v| format!("{v:.2}"))
            .draw()?;
        c.draw_series(LineSeries::new(
            [(start as f64 * 0.25, 0.), (end as f64 * 0.25, 0.)],
            RGBColor(137, 142, 147),
        ))?;
        for (j, trace) in [old, new].iter().enumerate() {
            let color = colors[j];
            let pts = trace.samples[start as usize..=end as usize]
                .iter()
                .map(|s| (time(s), delta(s)));
            c.draw_series(LineSeries::new(pts.clone(), color.stroke_width(2)))?
                .label(["Previous", "Preserved"][j])
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 14, y)], color.stroke_width(2))
                });
            c.draw_series(pts.map(|p| Circle::new(p, 3, color.filled())))?;
        }
        c.configure_series_labels()
            .position(SeriesLabelPosition::UpperRight)
            .background_style(WHITE.mix(0.90))
            .draw()?;
    }
    root.present()?;
    Ok(())
}

fn cache_plot(previous: &Trace, summary: &Summary, output: &Path) -> Result<()> {
    let probe = &summary.switch_probe;
    let prior = previous
        .samples
        .get(probe.last_prescribed_tick as usize)
        .ok_or("invalid handoff tick")?;
    let stages = [
        ("After settling", &summary.settled_arm_floor_cache),
        (
            "Previous policy, last prescribed step",
            &prior.automatic_arm_floor_cache,
        ),
        (
            "Updated policy, last prescribed step",
            &probe.before_switch_cache,
        ),
        (
            "Updated policy, after switch, before solve",
            &probe.after_switch_before_solve_cache,
        ),
    ];
    let path = output.join("ground-cache.svg");
    let root = SVGBackend::new(&path, (1090, 365)).into_drawing_area();
    root.fill(&WHITE)?;
    let font = |size| {
        ("sans-serif", size)
            .into_font()
            .color(&RGBColor(40, 52, 65))
    };
    root.draw(&Text::new(
        "Ground-contact warm starts on the arm",
        (24, 13),
        font(24),
    ))?;
    root.draw(&Text::new(format!("Native cache around promotion at {:.2} ms   ·   four contact points per link   ·   all values in mN·s",probe.first_resumed_tick as f64*0.25),(24,52),font(16)))?;
    for (j, label) in [
        "Base normal",
        "Tip normal",
        "Base |tangent|",
        "Tip |tangent|",
    ]
    .into_iter()
    .enumerate()
    {
        root.draw(&Text::new(label, (373 + 175 * j as i32, 93), font(16)))?;
    }
    for (i, (name, cache)) in stages.into_iter().enumerate() {
        let y = 135 + 50 * i as i32;
        root.draw(&Text::new(name, (24, y + 1), font(15)))?;
        for j in 0..4 {
            let value = if j < 2 {
                cache[j].normal_ns
            } else {
                DVec3::from(cache[j - 2].tangent_on_arm_ns).length()
            } * 1000.;
            let x = 373 + 175 * j as i32;
            let c = if j < 2 {
                RGBColor(24, 116, 112)
            } else {
                RGBColor(177, 108, 50)
            };
            root.draw(&Rectangle::new(
                [(x, y + 24), (x + 140, y + 29)],
                RGBColor(228, 232, 235).filled(),
            ))?;
            let w = (value * 180.).round() as i32;
            if w > 0 {
                root.draw(&Rectangle::new([(x, y + 24), (x + w, y + 29)], c.filled()))?;
            }
            root.draw(&Text::new(format!("{value:.5}"), (x, y), font(16)))?;
        }
    }
    root.draw(&Text::new("Tangential columns show the magnitude of the net world-frame tangent impulse cached for each link.",(24,338),font(15)))?;
    root.present()?;
    Ok(())
}
