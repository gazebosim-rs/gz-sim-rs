// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Display synchronized landing histories with measured solver work and COM separation.
use super::*;
use plotters::prelude::{LineSeries, SeriesLabelPosition};

#[path = "native_comparison.rs"]
mod native;
#[path = "outer_divergence.rs"]
mod outer;

#[derive(Clone, Copy, PartialEq)]
pub(in super::super) enum Study {
    Weak,
    Outer,
    Native,
}
impl Study {
    fn layout(self, dir: &Path) -> Result<native::Layout> {
        let (labels, files) = match self {
            Self::Weak => (
                ["current_policy", "weak_recontact", "continuous_fine"],
                ["current", "weak", "fine"],
            ),
            Self::Outer => (
                ["adaptive", "matched_fine", "continuous_fine"],
                ["adaptive", "matched_fine", "continuous_fine"],
            ),
            Self::Native => return native::load_layout(dir),
        };
        Ok(native::Layout {
            title: String::new(),
            public_dt_ns: 500_000,
            native_samples: false,
            interval_samples: false,
            clock_origin_ns: 0,
            points: None,
            pair_totals_only: false,
            single_body: false,
            load_face: None,
            independent_histories: false,
            sampled_ticks: false,
            capture_ticks: BTreeMap::new(),
            lanes: (0..labels.len())
                .map(|k| native::Lane {
                    label: labels[k].into(),
                    name: labels[k].into(),
                    analytical: false,
                    analytical_ticks: vec![],
                    file: format!("traces/{}.json", files[k]),
                })
                .collect(),
        })
    }
}
const NAMES: [&str; 3] = ["Current policy", "Weak recontact", "Four substeps"];
const COLORS: [[f64; 4]; 3] = [[0.92, 0.48, 0.28, 1.], LANE[0], LANE[1]];
const INKS: [RGBColor; 3] = [
    RGBColor(182, 87, 40),
    RGBColor(8, 130, 111),
    RGBColor(44, 92, 157),
];

fn distance_um(sample: &Sample, reference: &Sample, single_com: Option<DVec3>) -> f64 {
    let center = |s: &Sample| {
        let foot = DVec3::from(s.foot.position);
        match single_com {
            Some(local) => foot + DQuat::from_array(s.foot.orientation) * local,
            None => {
                (foot
                    + DVec3::from(
                        s.upper
                            .as_ref()
                            .expect("validated two-link recording")
                            .position,
                    ))
                    * 0.5
            }
        }
    };
    center(sample).distance(center(reference)) * 1e6
}
fn com_error_um(sample: &Sample, reference: &Sample) -> f64 {
    distance_um(sample, reference, None)
}

fn plot(path: &Path, rows: &[&[Sample]], peak: f64, max_error: f64) -> Result<()> {
    let root = SVGBackend::new(path, (980, 700)).into_drawing_area();
    root.fill(&WHITE)?;
    let area = root.titled(
        "A weak landing from matched incoming states",
        ("sans-serif", 27),
    )?;
    let cut = rows[0][0].time_us;
    let ms = |s: &Sample| (s.time_us - cut) as f64 * 0.001;
    let end_ms = ms(rows[0].last().ok_or("missing final sample")?);
    let panels = area.split_evenly((2, 1));
    for (i, (title, axis, limit)) in [
        ("Ground reaction", "Impulse per 500 µs tick (N·s)", peak),
        (
            "Trajectory after recontact",
            "COM distance from four substeps (µm)",
            max_error,
        ),
    ]
    .into_iter()
    .enumerate()
    {
        let mut chart = ChartBuilder::on(&panels[i])
            .margin(10)
            .caption(title, ("sans-serif", 19))
            .x_label_area_size(42)
            .y_label_area_size(84)
            .build_cartesian_2d(0.0..end_ms, 0.0..(limit * 1.14))?;
        chart
            .configure_mesh()
            .max_light_lines(1)
            .x_desc("Time since matched state (ms)")
            .y_desc(axis)
            .draw()?;
        // Draw the wider fine trace first so identical measurements remain visible.
        for k in (0..3).rev() {
            if i == 1 && k == 2 {
                continue;
            }
            let pts: Vec<_> = rows[k]
                .iter()
                .zip(rows[2])
                .map(|(s, r)| {
                    (
                        ms(s),
                        if i == 0 {
                            s.normal_impulse_ns
                        } else {
                            com_error_um(s, r)
                        },
                    )
                })
                .collect();
            let color = INKS[k];
            chart
                .draw_series(LineSeries::new(
                    pts.iter().copied(),
                    color.stroke_width(1 + k as u32),
                ))?
                .label(NAMES[k])
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 22, y)], color.stroke_width(3))
                });
            chart.draw_series(
                pts.into_iter()
                    .map(|pt| Circle::new(pt, 3 + k as i32, color.stroke_width(2))),
            )?;
        }
        chart
            .configure_series_labels()
            .position(SeriesLabelPosition::UpperMiddle)
            .background_style(WHITE.mix(0.93))
            .draw()?;
    }
    root.present()?;
    Ok(())
}

pub(in super::super) fn export(dir: &Path, out: &Path, study: Study) -> Result<()> {
    let layout = study.layout(dir)?;
    let records = layout
        .lanes
        .iter()
        .map(|lane| read::<Recording>(&dir.join(&lane.file)))
        .collect::<Result<Vec<_>>>()?;
    let g = &records[0].geometry;
    let single_com = layout
        .single_body
        .then(|| DVec3::from(g.foot_com_local.unwrap_or([0.; 3])));
    if layout.single_body != g.upper_half_extents.is_none()
        || single_com.is_some_and(|x| !x.is_finite())
    {
        return Err("trace geometry disagrees with the number of native bodies".into());
    }
    let first = records[0].frames.first().ok_or("missing incoming state")?;
    let end = records[0].frames.last().ok_or("missing continuation")?.tick;
    if end <= first.tick {
        return Err("empty event window".into());
    }
    let analytical = layout.lanes.iter().any(|lane| lane.analytical);
    for (k, r) in records.iter().enumerate() {
        let cut = r.frames.first().ok_or("missing incoming state")?;
        if r.geometry != *g
            || r.frames.len() != records[0].frames.len()
            || cut.tick != first.tick
            || cut.stamp_ns() != first.stamp_ns()
            || !layout.independent_histories && analytical && !native::equivalent_cut(cut, first)
            || !layout.independent_histories
                && !analytical
                && (study != Study::Outer || k < 2)
                && (cut.foot != first.foot
                    || cut.upper != first.upper
                    || cut.pad != first.pad
                    || cut.opponent != first.opponent
                    || cut.normal_impulse_ns != first.normal_impulse_ns
                    || cut.corner_ns != first.corner_ns)
        {
            return Err(
                "incoming state, geometry or trace clocks differ across matched lanes".into(),
            );
        }
    }
    let rows = records
        .iter()
        .map(|r| {
            interval_with_receipts(
                r,
                first.tick,
                end,
                layout.pair_totals_only,
                layout.sampled_ticks,
                layout.single_body,
                (
                    layout.clock_origin_ns,
                    layout.public_dt_ns,
                    layout.native_samples,
                    layout.interval_samples,
                ),
            )
        })
        .collect::<Result<Vec<_>>>()?;
    let reference = *rows.last().ok_or("missing reference trajectory")?;
    if (layout.sampled_ticks || study == Study::Native)
        && rows[1..].iter().any(|row| {
            row.len() != rows[0].len()
                || row
                    .iter()
                    .zip(rows[0])
                    .any(|(a, b)| a.tick != b.tick || a.stamp_ns() != b.stamp_ns())
        })
    {
        return Err("sampled native histories have different physical clocks".into());
    }
    let mut peak = 0f64;
    let mut corner = 0f64;
    let mut error = 0f64;
    let mut low = f64::INFINITY;
    let mut matched_error = 0.0_f64;
    let mut max_spin = 0.0_f64;
    let mut max_steps = 1;
    for (k, row) in rows.iter().enumerate() {
        let listed = &layout.lanes[k].analytical_ticks;
        if (!listed.is_empty() && layout.lanes[k].analytical)
            || listed.windows(2).any(|w| w[0] >= w[1])
            || listed
                .iter()
                .any(|t| row.binary_search_by_key(t, |s| s.tick).is_err())
        {
            return Err("invalid per-tick analytical provenance".into());
        }
        for (i, s) in row.iter().enumerate() {
            if s.support_normal_ns.is_some() != first.support_normal_ns.is_some() {
                return Err("inconsistent measured support count".into());
            }
            let subs = s.native_steps.ok_or("missing native solve count")?;
            let mechanical = layout.lanes[k].analytical || listed.binary_search(&s.tick).is_ok();
            if mechanical && subs != 0 || !mechanical && subs == 0 && i != 0 {
                return Err("native solve count disagrees with lane provenance".into());
            }
            if study == Study::Native {
                max_spin = max_spin.max(native::spin_distance(s, &reference[i])?);
                max_steps = max_steps.max(subs);
            } else {
                let weak = s.weak_start.ok_or("missing live weak gate decision")?;
                if subs > 4
                    || weak && (k != usize::from(study == Study::Weak) || subs != 4)
                    || (k == 2 || study == Study::Outer && k == 1) && i > 0 && subs != 4
                {
                    return Err("inconsistent live weak recontact or reference effort".into());
                }
                max_steps = 4;
            }
            peak = peak.max(s.normal_impulse_ns);
            corner = s.corner_ns.into_iter().flatten().fold(corner, f64::max);
            error = error.max(distance_um(s, &reference[i], single_com));
            if study == Study::Outer {
                matched_error = matched_error.max(com_error_um(s, &rows[1][i]));
            }
            low = low.min(s.pad.position[2] - g.pad_half_extents[2]);
        }
    }
    if (!layout.pair_totals_only && corner <= 0.) || peak <= 0. {
        return Err("no measured contact in event window".into());
    }
    let max_error = error.max(1e-6);
    let unit_ns =
        if study == Study::Native && rows[0].iter().any(|s| !s.stamp_ns().is_multiple_of(1000)) {
            1
        } else {
            1000
        };
    let captures = match study {
        Study::Weak => {
            let onset = rows[1]
                .iter()
                .skip(1)
                .find(|s| s.weak_start == Some(true))
                .ok_or("this replay requires a live weak admission")?;
            json!({"weak_trigger":onset.time_us-first.time_us})
        }
        Study::Outer => outer::captures(&rows)?,
        Study::Native => native::captures(&rows, &layout.capture_ticks, unit_ns)?,
    };
    let lane_half_width = g
        .opponent_half_extents
        .map_or(g.pad_half_extents[0], |h| h[0].max(g.pad_half_extents[0]));
    let spacing = 2.6 * lane_half_width;
    let sole_z = low - 0.52 * g.pad_half_extents[0];
    let [hx, hy, _] = g.foot_half_extents;
    // These optional readings come from separately timed repetitions at this particular step.
    let matched: Vec<_> = rows
        .iter()
        .flat_map(|row| row.iter())
        .filter_map(|s| s.matched_step.as_ref())
        .collect();
    let max_wall_ns = matched.iter().map(|m| m.wall_p95_ns).max().unwrap_or(1);
    let max_j_error = matched
        .iter()
        .map(|m| m.impulse_error_ns)
        .fold(0., f64::max)
        .max(1e-15);
    let mut archive = Vec::new();
    let mut retired_markers = 0;
    for (i, r) in reference.iter().enumerate() {
        let mut frame = Frame::default();
        for (k, row) in rows.iter().enumerate() {
            let s = &row[i];
            let display = LaneDisplay {
                label: &layout.lanes[k].label,
                color: COLORS[k],
                offset: (k as f64 - (rows.len() - 1) as f64 * 0.5) * spacing,
                sole_z,
                full: corner,
            };
            display.add_bodies(
                &mut frame,
                g,
                &s.foot,
                s.upper.as_ref(),
                &s.pad,
                s.opponent.as_ref(),
            )?;
            if let Some(corners) = s.corner_ns {
                display.add_corners(&mut frame, g, corners)?;
                if let Some(face) = layout.load_face.as_deref() {
                    display.add_face_markers(&mut frame, g, &s.foot, corners, face == "top");
                }
            }
            let last_gauge = match study {
                Study::Native => (
                    "spin_difference",
                    native::spin_distance(s, r)? / max_spin.max(1e-12),
                    PAD,
                ),
                Study::Weak => (
                    "weak_trigger",
                    f64::from(s.weak_start.ok_or("missing weak decision")?),
                    PAD,
                ),
                Study::Outer => (
                    "distance_from_matched",
                    com_error_um(s, &rows[1][i]) / matched_error.max(1e-6),
                    PAD,
                ),
            };
            for (j, (name, val, color)) in [
                ("pair_impulse", s.normal_impulse_ns / peak, COLORS[k]),
                (
                    "com_error",
                    distance_um(s, r, single_com) / max_error,
                    COLORS[k],
                ),
                (
                    "native_steps",
                    f64::from(s.native_steps.ok_or("missing native steps")?) / f64::from(max_steps),
                    UPPER,
                ),
                last_gauge,
            ]
            .into_iter()
            .enumerate()
            {
                display.gauge(
                    &mut frame,
                    name,
                    hx,
                    sole_z
                        - (if layout.pair_totals_only { 0. } else { 1.62 } + 0.5 * j as f64) * hy,
                    val,
                    color,
                );
            }
            if let Some(loads) = s.support_normal_ns {
                for (j, (name, load)) in ["first_support_impulse", "second_support_impulse"]
                    .into_iter()
                    .zip(loads)
                    .enumerate()
                {
                    display.gauge(
                        &mut frame,
                        name,
                        hx,
                        sole_z - (2.0 + 0.5 * j as f64) * hy,
                        load / peak,
                        [PAD, [0.49, 0.45, 0.76, 1.]][j],
                    );
                }
            }
        }
        // Static scene identities persist. A new event always receives a fresh ID.
        let permanent = frame.visuals.len();
        for (k, row) in rows.iter().enumerate() {
            let shift = DVec3::Y * ((k as f64 - (rows.len() - 1) as f64 * 0.5) * spacing);
            let view = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
            if let Some(m) = row[i].matched_step.as_ref() {
                let display = LaneDisplay {
                    label: &layout.lanes[k].label,
                    color: COLORS[k],
                    offset: shift.y,
                    sole_z,
                    full: corner,
                };
                display.gauge(
                    &mut frame,
                    "mechanics_impulse_error",
                    hx,
                    sole_z - 2. * hy,
                    m.impulse_error_ns / max_j_error,
                    COLORS[k],
                );
                let label = format!("step_p95_{}ns", m.wall_p95_ns);
                display.gauge(
                    &mut frame,
                    &label,
                    hx,
                    sole_z - 2.5 * hy,
                    m.wall_p95_ns as f64 / max_wall_ns as f64,
                    UPPER,
                );
            }
            if let Some(last) = row[i].contact_intervals.last() {
                let scale = 2. * hx / last.end_ns as f64;
                for part in &row[i].contact_intervals {
                    let width = scale * part.duration_ns as f64;
                    let mid = shift.y - hx + scale * part.end_ns as f64 - 0.5 * width;
                    let (kind, color) = if part.material {
                        ("proved_contact", [0.05, 0.82, 0.68, 1.])
                    } else {
                        ("native_contact", [0.65, 0.69, 0.74, 1.])
                    };
                    let label = format!("{}_{kind}_to_{}ns", layout.lanes[k].label, part.end_ns);
                    // Leave a small visual gap between adjacent executed intervals.
                    frame.add(
                        &label,
                        DVec3::new(0.023, mid, sole_z - 2. * hy),
                        DQuat::IDENTITY,
                        json!({"kind":"box","size":[0.012,0.96*width,0.016]}),
                        color,
                    );
                }
            }
            for (h, hit) in row[i].interval_impacts.iter().enumerate() {
                let start = view * DVec3::from(hit.point_m) + shift;
                let delta = view * DVec3::from(hit.impulse_ns); // one metre per N·s
                let label = format!(
                    "{}_impact_{}_{}ns",
                    layout.lanes[k].label, h, hit.elapsed_ns
                );
                let color = [1., 0.31, 0.64, 1.];
                frame.add(
                    &label,
                    start,
                    DQuat::IDENTITY,
                    json!({"kind":"sphere","radius":0.012}),
                    color,
                );
                frame.edge(
                    &format!("{label}_impulse"),
                    start,
                    start + delta,
                    delta.length(),
                    0.008,
                    color,
                );
            }
        }
        for j in permanent..frame.visuals.len() {
            let id = 2 * (j + retired_markers) as u64 + 2;
            frame.models[j]["model_id"] = json!(id);
            frame.models[j]["visual_ids"] = json!([id + 1]);
            frame.visuals[j]["id"] = json!(id + 1);
        }
        retired_markers += frame.visuals.len() - permanent;
        archive.push(frame.finish(Duration::from_nanos(r.stamp_ns() - first.stamp_ns()), i + 1));
    }
    fs::create_dir_all(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), archive)?;
    match study {
        Study::Native => native::plot(
            &out.join("comparison.svg"),
            &layout,
            &rows,
            [peak, error, max_spin],
            single_com,
        )?,
        Study::Weak => plot(&out.join("weak-recontact.svg"), &rows, peak, max_error)?,
        Study::Outer => outer::plot(
            &out.join("outer-divergence.svg"),
            &rows,
            peak,
            max_error.max(matched_error),
        )?,
    }
    if let Some(source) = &layout.points {
        native::points::plot(dir, out, source, &layout, &rows, g)?;
    }
    let lanes = layout
        .lanes
        .iter()
        .map(|lane| {
            let mut source = json!({"file":lane.file,"label":lane.label});
            if !lane.analytical_ticks.is_empty() {
                source["analytical_ticks"] = json!(lane.analytical_ticks);
            }
            source
        })
        .collect::<Vec<_>>();
    let mut index = json!({"kind":match study {Study::Native=>"native-comparison-v1",Study::Weak=>"weak-recontact-v1",Study::Outer=>"outer-divergence-v1"},"origin_us":first.time_us,"dt_ns":unit_ns,
        "ticks":rows[0].iter().map(|s|(s.stamp_ns()-first.stamp_ns())/unit_ns).collect::<Vec<_>>(),
        "captures":captures,
        "sole_z":sole_z,"spacing":spacing,"full_corner_ns":corner,"peak_pair_ns":peak,
        "max_com_error_um":max_error,"lanes":lanes});
    if study == Study::Native {
        let partitions: usize = rows
            .iter()
            .flat_map(|r| r.iter())
            .map(|s| s.contact_intervals.len())
            .sum();
        if partitions > 0 {
            index["contact_interval_bars"] = json!(partitions);
        }
        let impacts: usize = rows
            .iter()
            .flat_map(|row| row.iter())
            .map(|s| s.interval_impacts.len())
            .sum();
        if !matched.is_empty() {
            index["matched_step_count"] = json!(matched.len());
            index["max_step_wall_p95_ns"] = json!(max_wall_ns);
            index["max_mechanics_impulse_error_ns"] = json!(max_j_error);
        }
        if impacts > 0 {
            index["interval_impact_count"] = json!(impacts);
            index["impulse_display_m_per_ns"] = json!(1.);
        }
        if unit_ns == 1 {
            index["origin_ns"] = json!(first.stamp_ns());
        }
        if first.support_normal_ns.is_some() {
            index["support_impulses"] = json!(true);
        }
        if g.opponent_half_extents.is_some() {
            index["has_opponent"] = json!(true);
        }
        if layout.single_body {
            index["single_body"] = json!(true);
        }
        if layout.sampled_ticks {
            index["sampled_ticks"] = json!(true);
        }
        if layout.independent_histories {
            index["independent_histories"] = json!(true);
            index["initial_com_distances_um"] = json!(
                rows.iter()
                    .map(|r| distance_um(&r[0], &reference[0], single_com))
                    .collect::<Vec<_>>()
            );
        }
        if layout.pair_totals_only {
            index["pair_totals_only"] = json!(true);
            index
                .as_object_mut()
                .expect("frame index")
                .remove("full_corner_ns");
        }
        index["title"] = json!(layout.title);
        index["source_public_dt_ns"] = json!(layout.public_dt_ns);
        if let Some(face) = &layout.load_face {
            index["load_face"] = json!(face);
        }
        if layout.native_samples {
            index["native_samples"] = json!(true);
        }
        if layout.interval_samples {
            index["interval_samples"] = json!(true);
        }
        index["source_clock_origin_ns"] = json!(layout.clock_origin_ns);
        index["max_native_steps"] = json!(max_steps);
        index["max_spin_difference_rad_s"] = json!(max_spin.max(1e-12));
    }
    if study == Study::Outer {
        index["max_matched_error_um"] = json!(matched_error.max(1e-6));
    }
    fs::write(out.join("frames.json"), serde_json::to_vec_pretty(&index)?)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn single_body_rotates_inertial_offset_from_the_native_link_frame() {
        let sample = |position: [f64; 3], rotation: DQuat| -> Sample {
            let body = json!({"position":position,"orientation":rotation.to_array()});
            serde_json::from_value(
                json!({"tick":0,"time_us":0,"foot":body,"pad":body,"normal_impulse_ns":0.}),
            )
            .unwrap()
        };
        let a = sample(
            [0., 0.07, 0.],
            DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
        );
        let b = sample([0., 0., -0.07], DQuat::IDENTITY);
        assert!(a.upper.is_none() && b.upper.is_none());
        assert!(distance_um(&a, &b, Some(DVec3::Z * 0.07)) < 1e-9);
        assert!(distance_um(&a, &b, Some(DVec3::ZERO)) > 90_000.);
    }
}
