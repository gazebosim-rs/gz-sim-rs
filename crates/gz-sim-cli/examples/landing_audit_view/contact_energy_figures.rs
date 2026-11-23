// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Render measured energy, momentum and rigid-body controls from the native records.
use super::*;
use plotters::prelude::{
    ChartBuilder, Circle, Color, IntoDrawingArea, IntoFont, LineSeries, PathElement, RGBColor,
    SVGBackend, Text, WHITE,
};

const C: [RGBColor; 5] = [
    RGBColor(42, 85, 153),
    RGBColor(11, 127, 111),
    RGBColor(190, 96, 31),
    RGBColor(128, 72, 146),
    RGBColor(92, 99, 107),
];
struct Curve {
    title: String,
    points: Vec<(f64, f64)>,
    color: RGBColor,
}
fn curve(title: impl Into<String>, color: usize, points: Vec<(f64, f64)>) -> Curve {
    Curve {
        title: title.into(),
        points,
        color: C[color],
    }
}
struct Panel {
    title: String,
    x: &'static str,
    y: &'static str,
    lines: Vec<Curve>,
    dots: bool,
}
fn lines(title: impl Into<String>, unit: &'static str, lines: Vec<Curve>) -> Panel {
    Panel {
        title: title.into(),
        x: "Time since the cut (ms)",
        y: unit,
        lines,
        dots: false,
    }
}
fn rates(title: impl Into<String>, unit: &'static str, lines: Vec<Curve>) -> Panel {
    Panel {
        title: title.into(),
        x: "Solves per 500 µs tick",
        y: unit,
        lines,
        dots: true,
    }
}
fn figure(
    file: &Path,
    title: &str,
    panels: Vec<Panel>,
    columns: usize,
    shared_y: bool,
) -> Result<()> {
    let nr = panels.len().div_ceil(columns);
    let root =
        SVGBackend::new(file, (540 * columns as u32, 52 + 395 * nr as u32)).into_drawing_area();
    root.fill(&WHITE)?;
    let slots = root
        .titled(title, ("sans-serif", 26))?
        .split_evenly((nr, columns));
    let yrange = |panels: &[Panel]| -> Result<(f64, f64)> {
        let mut lo = 0f64;
        let mut hi = 0f64;
        for s in panels.iter().flat_map(|p| &p.lines) {
            if s.points.is_empty() {
                return Err("empty plotted series".into());
            }
            for &(x, y) in &s.points {
                if !x.is_finite() || !y.is_finite() {
                    return Err("nonfinite plot data".into());
                }
                lo = lo.min(y);
                hi = hi.max(y);
            }
        }
        let span = hi - lo;
        let margin = if span == 0. { 1e-12 } else { span * 0.09 };
        Ok((if lo < 0. { lo - margin } else { lo }, hi + margin))
    };
    let shared = yrange(&panels)?;
    for (area, p) in slots.iter().zip(&panels) {
        let (lo, hi) = if shared_y {
            shared
        } else {
            yrange(std::slice::from_ref(p))?
        };
        let (mut xlo, mut xhi) = p
            .lines
            .iter()
            .flat_map(|s| &s.points)
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &(x, _)| {
                (lo.min(x), hi.max(x))
            });
        if p.dots {
            xlo -= 0.15;
            xhi += 0.15;
        }
        let height = area.dim_in_pixel().1 as i32;
        let (plot, legend) = area.split_vertically(height - 41);
        let mut chart = ChartBuilder::on(&plot)
            .margin(12)
            .caption(&p.title, ("sans-serif", 18))
            .x_label_area_size(43)
            .y_label_area_size(82)
            .build_cartesian_2d(xlo..xhi, lo..hi)?;
        chart
            .configure_mesh()
            .x_desc(p.x)
            .y_desc(p.y)
            .y_labels(5)
            .max_light_lines(1)
            .light_line_style(RGBColor(228, 233, 237))
            .x_labels(if p.dots { 4 } else { 6 })
            .x_label_formatter(&|x| {
                if p.dots && (![1., 2., 4.].contains(x)) {
                    String::new()
                } else {
                    format!("{x}")
                }
            })
            .y_label_formatter(&|y| {
                if hi - lo < 1e-3 {
                    format!("{y:.1e}")
                } else if hi - lo < 0.1 {
                    format!("{y:.3}")
                } else {
                    format!("{y:.2}")
                }
            })
            .draw()?;
        let columns = p.lines.len().clamp(1, 2);
        for (index, s) in p.lines.iter().enumerate() {
            chart.draw_series(LineSeries::new(
                s.points.iter().copied(),
                s.color.stroke_width(2),
            ))?;
            if p.dots {
                chart.draw_series(
                    s.points
                        .iter()
                        .map(|&pt| Circle::new(pt, 4, s.color.filled())),
                )?;
            }
            let x = 55 + (index % columns) as i32 * 233;
            let y = 9 + (index / columns) as i32 * 16;
            legend.draw(&PathElement::new(
                [(x, y), (x + 15, y)],
                s.color.stroke_width(2),
            ))?;
            legend.draw(&Text::new(
                s.title.as_str(),
                (x + 20, y - 7),
                ("sans-serif", 13).into_font(),
            ))?;
        }
    }
    root.present()?;
    Ok(())
}
fn num(v: &Value, key: &str) -> Result<f64> {
    v[key]
        .as_f64()
        .ok_or_else(|| format!("missing numeric field {key}").into())
}
fn frames(v: &Value) -> Result<&[Value]> {
    Ok(v["frames"].as_array().ok_or("missing native frames")?)
}
fn time(v: &Value) -> Result<u64> {
    v["time_us"]
        .as_u64()
        .ok_or_else(|| "missing native clock".into())
}
fn vectors(v: &Value, key: &str) -> Result<[f64; 3]> {
    Ok(serde_json::from_value(v[key].clone())?)
}
fn trace(audit: &Path, receiver: &str, motion: &str, n: u64) -> Result<Value> {
    read(&audit.join(format!("energy/traces/{receiver}-{motion}-{n}x.json")))
}
fn native(audit: &Path) -> Result<Vec<Value>> {
    [1, 2, 4]
        .into_iter()
        .map(|n| trace(audit, "reference", "native", n))
        .collect()
}
fn series(
    ledger: &Value,
    end: f64,
    mut get: impl FnMut(&Value) -> Result<f64>,
) -> Result<Vec<(f64, f64)>> {
    let fs = frames(ledger)?;
    let cut = time(fs.first().ok_or("missing initial sample")?)?;
    let mut result = vec![];
    for s in fs {
        let x = (time(s)? - cut) as f64 / 1000.;
        if x > end {
            break;
        }
        result.push((x, get(s)?));
    }
    Ok(result)
}
fn at(v: &Value, us: u64) -> Result<&Value> {
    let s = frames(v)?;
    let i = s
        .binary_search_by_key(&us, |v| v["time_us"].as_u64().unwrap_or(u64::MAX))
        .map_err(|_| format!("missing frame {us}"))?;
    Ok(&s[i])
}
fn articulated(audit: &Path, out: &Path) -> Result<()> {
    let ts = native(audit)?;
    let mut balance = vec![];
    for (k, t) in ts.iter().enumerate() {
        let fields = [
            ("Lift work", "lift_work_j"),
            ("Δ(K + U)", "mechanical_change_j"),
            ("Work − Δ(K + U)", "energy_loss_j"),
        ];
        let mut curves = vec![];
        for (i, (label, key)) in fields.iter().enumerate() {
            curves.push(curve(*label, i, series(t, 15., |v| num(v, key))?));
        }
        balance.push(lines(
            format!("{} µs at touchdown", 500 / [1, 2, 4][k]),
            "J",
            curves,
        ));
    }
    figure(
        &out.join("energy.svg"),
        "Work and mechanical energy during the always-dynamic landing",
        balance,
        3,
        true,
    )?;
    let mut components = vec![];
    for (name, key) in [
        ("COM translation", "kinetic_com_j"),
        ("Rotation", "kinetic_rotation_j"),
        ("Internal motion", "kinetic_internal_j"),
        ("Gravitational potential", "potential_j"),
    ] {
        let mut curves = vec![];
        for (i, t) in ts.iter().enumerate() {
            let fs = frames(t)?;
            let initial = num(&fs[0], key)?;
            curves.push(curve(
                format!("{} µs", 500 / [1, 2, 4][i]),
                i,
                series(t, 15., |v| Ok(num(v, key)? - initial))?,
            ));
        }
        components.push(lines(name, "Change from cut (J)", curves));
    }
    figure(
        &out.join("components.svg"),
        "Partition of the articulated foot’s mechanical energy",
        components,
        2,
        false,
    )?;
    let mut impulses = vec![];
    for (field, title) in [
        ("contact_sum_ns", "All contact"),
        ("tangent_sum_ns", "Tangential contact"),
    ] {
        for (axis, label) in ["X", "Y", "Z"].iter().enumerate() {
            let mut curves = vec![];
            for (i, t) in ts.iter().enumerate() {
                curves.push(curve(
                    format!("{} µs", 500 / [1, 2, 4][i]),
                    i,
                    series(t, 15., |v| Ok(vectors(v, field)?[axis]))?,
                ));
            }
            impulses.push(lines(
                format!("{title} · world {label}"),
                "Cumulative impulse (N·s)",
                curves,
            ));
        }
    }
    figure(
        &out.join("impulse.svg"),
        "Impulse components at the moving support",
        impulses,
        3,
        false,
    )?;
    let report: Value = read(&audit.join("energy/summary.json"))?;
    let runs = report["runs"]
        .as_array()
        .ok_or("missing articulated runs")?;
    let mut checkpoints = vec![];
    for (title, key, unit) in [
        ("Normal impulse", "normal_sum_ns", "N·s"),
        ("Lift work", "lift_work_j", "J"),
        ("Mechanical change", "mechanical_change_j", "J"),
    ] {
        let mut curves = vec![];
        for (i, interval) in ["first", "window", "stroke"].iter().enumerate() {
            let mut data = vec![];
            for n in [1, 2, 4] {
                let run = runs
                    .iter()
                    .find(|r| {
                        r["receiver"] == "reference" && r["motion"] == "native" && r["factor"] == n
                    })
                    .ok_or("missing reference checkpoints")?;
                data.push((n as f64, num(&run[interval], key)?));
            }
            curves.push(curve(
                ["First loaded step", "5 ms", "Full stroke"][i],
                i,
                data,
            ));
        }
        checkpoints.push(rates(title, unit, curves));
    }
    figure(
        &out.join("checkpoints.svg"),
        "Always-dynamic response across measurement intervals",
        checkpoints,
        3,
        false,
    )?;
    let mut balances = vec![];
    for (key, title, units) in [
        (
            "linear_residual_ns",
            "Linear momentum",
            "Residual norm (N·s)",
        ),
        (
            "angular_residual_nms",
            "Angular momentum",
            "Residual norm (N·m·s)",
        ),
        (
            "energy_closure_residual_j",
            "Contact-work reconstruction",
            "Residual (J)",
        ),
    ] {
        let mut curves = vec![];
        for (i, t) in ts.iter().enumerate() {
            curves.push(curve(
                format!("{} µs", 500 / [1, 2, 4][i]),
                i,
                series(t, 15., |v| {
                    if key == "energy_closure_residual_j" {
                        num(v, key)
                    } else {
                        Ok(DVec3::from_array(vectors(v, key)?).length())
                    }
                })?,
            ));
        }
        balances.push(lines(title, units, curves));
    }
    figure(
        &out.join("residuals.svg"),
        "Measured closure of impulse and work balances",
        balances,
        3,
        false,
    )?;
    let mut corners = vec![];
    for (i, t) in ts.iter().enumerate() {
        for axis in 0..2 {
            let mut curves = vec![];
            for (c, label) in ["−X −Y", "−X +Y", "+X −Y", "+X +Y"].iter().enumerate() {
                curves.push(curve(
                    *label,
                    c,
                    series(t, 5., |v| {
                        v["corner_angular_sum_nms"][c][axis]
                            .as_f64()
                            .ok_or_else(|| "missing accumulated corner moment".into())
                    })?,
                ));
            }
            corners.push(lines(
                format!(
                    "{} µs   ·   world {} moment",
                    500 / [1, 2, 4][i],
                    ["X", "Y"][axis]
                ),
                "Angular impulse (N·m·s)",
                curves,
            ));
        }
    }
    figure(
        &out.join("corner-moments.svg"),
        "Contact moment from each sole corner",
        corners,
        2,
        true,
    )?;
    let mut crossed = vec![];
    for key in ["lift_work_j", "mechanical_change_j", "energy_loss_j"] {
        let mut curves = vec![];
        for (i, m) in ["reference", "switched"].iter().enumerate() {
            let mut data = vec![];
            for n in [1, 2, 4] {
                let a = trace(audit, "switched", m, n)?;
                let b = trace(audit, "reference", m, n)?;
                data.push((
                    n as f64,
                    1e6 * (num(at(&a, 716500)?, key)? - num(at(&b, 716500)?, key)?),
                ));
            }
            curves.push(curve(format!("{m} motion"), i, data));
        }
        crossed.push(rates(
            match key {
                "lift_work_j" => "Lift work",
                "mechanical_change_j" => "Mechanical change",
                _ => "Work − mechanical change",
            },
            "Receiver difference (µJ)",
            curves,
        ));
    }
    figure(
        &out.join("receivers.svg"),
        "Switched minus reference contact history at identical incoming motion, 5 ms",
        crossed,
        3,
        true,
    )?;
    Ok(())
}

fn rigid(audit: &Path, out: &Path) -> Result<()> {
    let report: Value = read(&audit.join("rigid/summary.json"))?;
    let runs = report["runs"].as_array().ok_or("missing rigid runs")?;
    let mut variants = vec![];
    for (name, key, unit) in [
        ("First native reaction", "first_ns", "Impulse (N·s)"),
        ("First 5 ms", "window_ns", "Impulse (N·s)"),
        ("Work by lift, 5 ms", "window_work_j", "J"),
        ("Work − mechanical change, 5 ms", "window_loss_j", "J"),
    ] {
        let mut curves = vec![];
        for (i, variant) in ["matched", "level", "no_spin", "level_no_spin"]
            .iter()
            .enumerate()
        {
            let mut data = vec![];
            for n in [1, 2, 4] {
                let row = runs
                    .iter()
                    .find(|r| {
                        r["donor"] == "reference" && r["variant"] == *variant && r["factor"] == n
                    })
                    .ok_or_else(|| format!("missing rigid control {variant} {n}"))?;
                data.push((n as f64, num(row, key)?));
            }
            curves.push(curve(
                [
                    "Tilt and spin",
                    "Level",
                    "Spin removed",
                    "Level, spin removed",
                ][i],
                i,
                data,
            ));
        }
        variants.push(rates(name, unit, curves));
    }
    figure(
        &out.join("rigid-controls.svg"),
        "Equivalent free foot at the measured clearance",
        variants,
        2,
        false,
    )?;
    let mut checks = vec![];
    for (title, curves, unit) in [
        (
            "Level vertical landing at 5 ms",
            vec![
                ("Native box", "central", "window_ns", false),
                ("Plastic impact + weight", "central", "window_ns", true),
            ],
            "Impulse (N·s)",
        ),
        (
            "Energy lost at 5 ms",
            vec![
                ("Native box", "central", "window_loss_j", false),
                ("Plastic impact", "central", "plastic_impact_loss_j", true),
            ],
            "Work − Δ(K + U) (J)",
        ),
        (
            "Effect of reference frame",
            vec![
                ("World lift work", "matched", "window_work_j", false),
                ("Lift frame work", "relative_frame", "window_work_j", false),
                ("World loss", "matched", "window_loss_j", false),
                ("Lift frame loss", "relative_frame", "window_loss_j", false),
            ],
            "Energy (J)",
        ),
    ] {
        let mut lines = vec![];
        for (i, (label, variant, key, ideal)) in curves.into_iter().enumerate() {
            let mut data = vec![];
            for n in [1, 2, 4] {
                let run = runs
                    .iter()
                    .find(|r| {
                        r["donor"] == "reference" && r["variant"] == variant && r["factor"] == n
                    })
                    .ok_or("missing rigid control")?;
                data.push((n as f64, num(if ideal { &run["ideal"] } else { run }, key)?));
            }
            lines.push(curve(label, i, data));
        }
        checks.push(rates(title, unit, lines));
    }
    let mut rebuild = vec![];
    for (i, donor) in ["reference", "switched"].into_iter().enumerate() {
        let mut data = vec![];
        for n in [1, 2, 4] {
            let hot = trace(audit, donor, "native", n)?;
            let cold = runs
                .iter()
                .find(|r| r["donor"] == donor && r["factor"] == n && r["variant"] == "cold_tree")
                .ok_or("missing rebuilt articulation")?;
            let end = time(&frames(&hot)?[0])? + 5_000;
            data.push((
                n as f64,
                1e6 * (num(cold, "window_ns")? - num(at(&hot, end)?, "normal_sum_ns")?),
            ));
        }
        rebuild.push(curve(donor, i, data));
    }
    checks.push(rates(
        "Rebuilt minus original articulation, 5 ms",
        "Impulse difference (µN·s)",
        rebuild,
    ));
    figure(
        &out.join("independent-controls.svg"),
        "Independent rigid and articulated controls",
        checks,
        2,
        false,
    )?;
    let mut panels = vec![];
    for (title, energy, rigid, unit) in [
        ("Support work", "lift_work_j", "window_work_j", "J"),
        (
            "Mechanical change",
            "mechanical_change_j",
            "window_delta_energy_j",
            "J",
        ),
        (
            "Dissipated or unresolved energy",
            "energy_loss_j",
            "window_loss_j",
            "J",
        ),
    ] {
        let mut curves = vec![];
        for (i, donor) in ["reference", "switched"].iter().enumerate() {
            let mut single = vec![];
            let mut linked = vec![];
            for n in [1, 2, 4] {
                let t = trace(audit, donor, "native", n)?;
                linked.push((n as f64, num(at(&t, 716500)?, energy)?));
                let r = runs
                    .iter()
                    .find(|r| r["donor"] == *donor && r["variant"] == "matched" && r["factor"] == n)
                    .ok_or("missing matched box")?;
                single.push((n as f64, num(r, rigid)?));
            }
            curves.push(curve(format!("Jointed, {donor}"), 2 * i, linked));
            curves.push(curve(format!("Free box, {donor}"), 2 * i + 1, single));
        }
        panels.push(rates(title, unit, curves));
    }
    figure(
        &out.join("rigid-ledger.svg"),
        "Measured 5 ms energy balance after transfer to one rigid body",
        panels,
        3,
        true,
    )?;
    Ok(())
}

pub(crate) fn draw(dir: &Path, out: &Path) -> Result<()> {
    fs::create_dir_all(out)?;
    articulated(dir, out)?;
    rigid(dir, out)
}
