// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Native continuations from a common incoming state. The last lane is the reference.
use super::*;

#[path = "native_points_figure.rs"]
pub(super) mod points;

#[derive(Deserialize)]
pub(super) struct Layout {
    pub(super) title: String,
    #[serde(default = "standard_tick_ns")]
    pub(super) public_dt_ns: u64,
    #[serde(default)]
    pub(super) native_samples: bool,
    #[serde(default)]
    pub(super) interval_samples: bool,
    #[serde(default)]
    pub(super) clock_origin_ns: u64,
    pub(super) lanes: Vec<Lane>,
    pub(super) points: Option<points::Source>,
    #[serde(default)]
    pub(super) pair_totals_only: bool,
    #[serde(default)]
    pub(super) single_body: bool,
    pub(super) load_face: Option<String>,
    #[serde(default)]
    pub(super) independent_histories: bool,
    #[serde(default)]
    pub(super) sampled_ticks: bool,
    #[serde(default)]
    pub(super) capture_ticks: BTreeMap<String, u64>,
}
fn standard_tick_ns() -> u64 {
    500_000
}

#[derive(Deserialize)]
pub(super) struct Lane {
    pub(super) file: String,
    pub(super) label: String,
    pub(super) name: String,
    #[serde(default)]
    pub(super) analytical: bool,
    #[serde(default)]
    pub(super) analytical_ticks: Vec<u64>,
}

pub(super) fn load_layout(dir: &Path) -> Result<Layout> {
    let layout: Layout = read(&dir.join("comparison.json"))?;
    let mut labels = BTreeSet::new();
    if !(2..=3).contains(&layout.lanes.len())
        || layout.title.trim().is_empty()
        || layout
            .load_face
            .as_deref()
            .is_some_and(|k| !matches!(k, "top" | "bottom") || layout.pair_totals_only)
        || layout.public_dt_ns == 0
        || (layout.native_samples && (!layout.sampled_ticks || layout.interval_samples))
        || layout.lanes.iter().any(|lane| {
            lane.name.trim().is_empty()
                || lane.label.is_empty()
                || !lane
                    .label
                    .bytes()
                    .all(|c| c.is_ascii_alphanumeric() || c == b'_')
                || !labels.insert(&lane.label)
        })
    {
        return Err("comparison requires a title and two or three distinct named lanes".into());
    }
    Ok(layout)
}

// Equivalent analytical and rigid models can reconstruct the same physical
// pose with a quaternion of opposite sign or round-off in the final bit.
pub(super) fn equivalent_cut(a: &Sample, b: &Sample) -> bool {
    const TOL: f64 = 1e-12;
    let vector = |x: [f64; 3], y: [f64; 3]| DVec3::from(x).distance(DVec3::from(y)) <= TOL;
    let optional = |x: Option<[f64; 3]>, y: Option<[f64; 3]>| match (x, y) {
        (Some(v), Some(w)) => vector(v, w),
        (None, None) => true,
        _ => false,
    };
    let body = |a: &Body, b: &Body| {
        let qa = DQuat::from_array(a.orientation);
        let qb = DQuat::from_array(b.orientation);
        vector(a.position, b.position)
            && optional(a.velocity, b.velocity)
            && optional(a.angular_velocity, b.angular_velocity)
            && (qa - qb).length().min((qa + qb).length()) <= TOL
    };
    let optional_body = |x: &Option<Body>, y: &Option<Body>| match (x, y) {
        (Some(x), Some(y)) => body(x, y),
        (None, None) => true,
        _ => false,
    };
    body(&a.foot, &b.foot)
        && optional_body(&a.upper, &b.upper)
        && optional_body(&a.opponent, &b.opponent)
        && body(&a.pad, &b.pad)
        && (a.normal_impulse_ns - b.normal_impulse_ns).abs() <= TOL
        && match (a.corner_ns, b.corner_ns) {
            (Some(x), Some(y)) => x.into_iter().zip(y).all(|(i, j)| (i - j).abs() <= TOL),
            (None, None) => true,
            _ => false,
        }
}

pub(super) fn spin_distance(a: &Sample, b: &Sample) -> Result<f64> {
    let spin = |r: &Sample| {
        r.foot
            .angular_velocity
            .map(DVec3::from)
            .ok_or("missing native foot spin")
    };
    let distance = spin(a)?.distance(spin(b)?);
    if !distance.is_finite() {
        return Err("nonfinite native foot spin".into());
    }
    Ok(distance)
}

pub(super) fn captures(
    rows: &[&[Sample]],
    events: &BTreeMap<String, u64>,
    unit_ns: u64,
) -> Result<Value> {
    let start = rows[0][0].stamp_ns();
    let first = rows
        .iter()
        .flat_map(|r| {
            r.iter()
                .skip(1)
                .filter(|s| s.normal_impulse_ns > 0.)
                .take(1)
        })
        .map(Sample::stamp_ns)
        .min()
        .ok_or("no positive contact after cut")?;
    let mut peaks = (0., first);
    let reference = rows.last().ok_or("empty lane set")?;
    for (i, r) in reference.iter().enumerate() {
        for row in &rows[..rows.len() - 1] {
            let value = spin_distance(&row[i], r)?;
            if value > peaks.0 {
                peaks = (value, r.stamp_ns());
            }
        }
    }
    let mut marks = BTreeMap::from([
        ("primary".to_owned(), (first - start) / unit_ns),
        (
            "peak_spin_difference".to_owned(),
            (peaks.1 - start) / unit_ns,
        ),
    ]);
    for (name, tick) in events {
        if name.is_empty()
            || !name.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'_')
            || matches!(name.as_str(), "initial" | "final")
            || marks.contains_key(name)
        {
            return Err(format!("invalid or reserved capture name {name:?}").into());
        }
        let index = rows[0]
            .binary_search_by_key(tick, |s| s.tick)
            .map_err(|_| format!("capture {name} selects unrecorded native tick {tick}"))?;
        marks.insert(name.clone(), (rows[0][index].stamp_ns() - start) / unit_ns);
    }
    Ok(json!(marks))
}

pub(super) fn plot(
    path: &Path,
    layout: &Layout,
    rows: &[&[Sample]],
    limits: [f64; 3],
    single_com: Option<DVec3>,
) -> Result<()> {
    let root = SVGBackend::new(path, (1040, 960)).into_drawing_area();
    root.fill(&WHITE)?;
    let areas = root
        .titled(&layout.title, ("sans-serif", 27))?
        .split_evenly((3, 1));
    let ms = |s: &Sample| s.stamp_ns() as f64 * 1e-6;
    let lo = ms(&rows[0][0]);
    let hi = ms(rows[0].last().ok_or("empty native curve")?);
    let last = rows.len() - 1;
    let ref_name = &layout.lanes[last].name;
    for (panel, title, unit, scale) in [
        (
            0,
            if layout.native_samples {
                "Normal reaction per sampled native solve"
            } else if layout.interval_samples {
                "Normal reaction per sampled interval"
            } else if layout.sampled_ticks {
                "Normal reaction per sampled actuator tick"
            } else {
                "Normal reaction per actuator tick"
            }
            .to_owned(),
            "Impulse (N·s)",
            1.,
        ),
        (
            1,
            format!("Center-of-mass distance from {ref_name}"),
            "Distance (mm)",
            0.001,
        ),
        (
            2,
            format!("Foot angular-velocity difference from {ref_name}"),
            "Vector difference (rad/s)",
            1.,
        ),
    ] {
        let mut chart = ChartBuilder::on(&areas[panel])
            .margin(12)
            .caption(title, ("sans-serif", 19))
            .x_label_area_size(40)
            .y_label_area_size(78)
            .build_cartesian_2d(lo..hi, 0.0..limits[panel].max(1e-10) * scale * 1.23)?;
        chart
            .configure_mesh()
            .x_desc(if rows[0][0].stamp_ns() == 0 {
                "Time since initial state (ms)"
            } else if layout.independent_histories
                || rows[0][0].time_ns.is_some()
                || layout.lanes.iter().any(|lane| lane.analytical)
            {
                "Simulation time (ms)"
            } else {
                "Simulation time after settling (ms)"
            })
            .y_desc(unit)
            .max_light_lines(1)
            .draw()?;
        for k in (0..if panel == 0 { rows.len() } else { last }).rev() {
            let color = INKS[k];
            let points = rows[k]
                .iter()
                .zip(rows[last])
                .map(|(s, r)| {
                    let v = match panel {
                        0 => s.normal_impulse_ns,
                        1 => distance_um(s, r, single_com),
                        _ => spin_distance(s, r)?,
                    };
                    Ok((ms(s), v * scale))
                })
                .collect::<Result<Vec<_>>>()?;
            let style = color.stroke_width(k as u32 + 1);
            let series = if layout.sampled_ticks {
                chart.draw_series(
                    points
                        .windows(2)
                        .zip(rows[k].windows(2))
                        .filter(|(_, samples)| samples[1].tick == samples[0].tick + 1)
                        .map(|(p, _)| PathElement::new([p[0], p[1]], style)),
                )?
            } else {
                chart.draw_series(LineSeries::new(points.iter().copied(), style))?
            };
            series.label(&layout.lanes[k].name).legend(move |(x, y)| {
                PathElement::new([(x, y), (x + 25, y)], color.stroke_width(3))
            });
            chart.draw_series(
                points
                    .into_iter()
                    .map(|pt| Circle::new(pt, 2 + k as i32, color.stroke_width(1))),
            )?;
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperMiddle)
            .background_style(WHITE.mix(0.95))
            .draw()?;
    }
    root.present()?;
    Ok(())
}
