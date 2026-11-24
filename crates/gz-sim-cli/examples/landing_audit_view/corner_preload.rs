// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Replay the uniform-material ridge at matching measured times in its two landing histories.
use super::*;

const PRIOR: &str = "Elastic before corner";
const EARLY: &str = "Elastic from corner";
const LATE: &str = "Elastic from line";
const CLOCK_TOLERANCE_S: f64 = 1e-12;

fn matched(samples: &[Sample], time: f64) -> Option<&Sample> {
    samples
        .iter()
        .find(|s| (s.original_time_s - time).abs() < CLOCK_TOLERANCE_S)
}

fn chart(path: &Path, early: &Study, late: &Study, prior: &Profile, origin: f64) -> Result<()> {
    let root = SVGBackend::new(path, (1220, 830)).into_drawing_area();
    root.fill(&WHITE)?;
    let panels = root.split_evenly((2, 2));
    let rgb =
        |[r, g, b, _]: [f64; 4]| RGBColor((255. * r) as u8, (255. * g) as u8, (255. * b) as u8);
    let ms = |s: &Sample| (s.original_time_s - origin) * 1000.;
    let first = ms(&prior.frames[0]);
    let last = ms(late.profiles[0]
        .frames
        .last()
        .ok_or("missing final sample")?);
    for (i, panel) in panels.iter().enumerate() {
        let path_panel = i == 3;
        let title = if path_panel {
            "Effect of the earlier start on motion".to_owned()
        } else {
            format!("Ground reaction • {}", early.profiles[i].name)
        };
        let mut series = Vec::new();
        if path_panel {
            for (k, (a, b)) in early.profiles.iter().zip(&late.profiles).enumerate() {
                let mut points = vec![];
                for s in &b.frames {
                    let f =
                        matched(&a.frames, s.original_time_s).ok_or("unmatched chart sample")?;
                    points.push((ms(s), 1e6 * f.com_m.distance(s.com_m)));
                }
                series.push((a.name.as_str(), rgb(COLORS[k]), points));
            }
        } else {
            for (j, (label, p)) in [(EARLY, &early.profiles[i]), (LATE, &late.profiles[i])]
                .into_iter()
                .enumerate()
            {
                series.push((
                    label,
                    rgb(COLORS[j]),
                    p.frames.iter().map(|s| (ms(s), s.normal_force_n)).collect(),
                ));
            }
        }
        if i == 0 {
            series.push((
                PRIOR,
                rgb(COLORS[2]),
                prior
                    .frames
                    .iter()
                    .map(|s| (ms(s), s.normal_force_n))
                    .collect(),
            ));
        }
        let hi = series
            .iter()
            .flat_map(|(_, _, v)| v.iter().map(|p| p.1))
            .fold(1e-9, f64::max);
        let mut axes = ChartBuilder::on(panel)
            .margin(17)
            .caption(title, ("sans-serif", 22))
            .x_label_area_size(48)
            .y_label_area_size(77)
            .build_cartesian_2d(first..last, 0.0..hi * 1.12)?;
        axes.configure_mesh()
            .max_light_lines(0)
            .x_desc("Time since restart (ms)")
            .y_desc(if path_panel {
                "Path separation (µm)"
            } else {
                "Normal load (N)"
            })
            .draw()?;
        for (name, color, points) in series {
            axes.draw_series(LineSeries::new(
                points.iter().copied(),
                color.stroke_width(2),
            ))?
            .label(name)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 17, y)], color.stroke_width(3)));
            axes.draw_series(
                points
                    .into_iter()
                    .map(|p| plotters::prelude::Circle::new(p, 3, color.filled())),
            )?;
        }
        axes.configure_series_labels()
            .background_style(WHITE.mix(0.88))
            .draw()?;
    }
    root.present()?;
    Ok(())
}

pub(in super::super) fn export(
    early_file: &Path,
    late_file: &Path,
    fixture: &Path,
    out: &Path,
) -> Result<()> {
    let early: Study = read(early_file)?;
    let late: Study = read(late_file)?;
    let rigid: Value = read(fixture)?;
    let origins = FixtureView::new(&rigid)?;
    if early.case != 80
        || early.stroke != 2
        || early.case != late.case
        || early.stroke != late.stroke
        || rigid["case"] != early.case
        || rigid["stroke"] != early.stroke
        || early.start_s >= late.start_s
        || early.profiles.len() != 3
        || late.profiles.len() != 3
        || early.profiles[0].name != "Uniform layers"
    {
        return Err("expected both start conditions for fixture 80, stroke 2".into());
    }
    for (a, b) in early.profiles.iter().zip(&late.profiles) {
        if a.name != b.name
            || a.frames.len() < 2
            || b.frames.len() < 2
            || (early.start_s - a.frames[0].original_time_s).abs() > CLOCK_TOLERANCE_S
            || (late.start_s - b.frames[0].original_time_s).abs() > CLOCK_TOLERANCE_S
            || (a.frames.last().unwrap().original_time_s - b.frames.last().unwrap().original_time_s)
                .abs()
                > CLOCK_TOLERANCE_S
        {
            return Err(
                "material histories do not bracket the two start times and a common horizon".into(),
            );
        }
        for p in [a, b] {
            if p.frames
                .windows(2)
                .any(|w| w[0].original_time_s >= w[1].original_time_s)
            {
                return Err("unordered material history".into());
            }
            for f in &p.frames {
                if !f.original_time_s.is_finite()
                    || !f.com_m.is_finite()
                    || !f.foot_m.is_finite()
                    || !f.orientation_xyzw.is_normalized()
                {
                    return Err("invalid material trajectory".into());
                }
                loads(f)?;
            }
        }
    }
    let a = &early.profiles[0].frames;
    let b = &late.profiles[0].frames;
    let control = early
        .earlier_start_control
        .as_ref()
        .ok_or("missing preceding-clock control")?;
    let preceding = &control.frames;
    if preceding.len() <= a.len()
        || preceding[0].original_time_s >= early.start_s
        || preceding
            .windows(2)
            .any(|p| p[0].original_time_s >= p[1].original_time_s)
        || a.iter()
            .any(|v| matched(preceding, v.original_time_s).is_none())
    {
        return Err("preceding-clock control does not cover the corner trajectory".into());
    }
    let mut peak_bin = 0f64;
    for f in a.iter().chain(b).chain(preceding) {
        if !f.original_time_s.is_finite()
            || !f.com_m.is_finite()
            || !f.foot_m.is_finite()
            || !f.orientation_xyzw.is_normalized()
        {
            return Err("invalid preload sample".into());
        }
        peak_bin = loads(f)?.into_iter().fold(peak_bin, f64::max);
    }
    let mut max_shift = 0f64;
    for s in b {
        let f = matched(a, s.original_time_s)
            .ok_or("missing corner-start sample at line-start clock")?;
        max_shift = max_shift.max(s.com_m.distance(f.com_m));
    }
    if peak_bin == 0. || max_shift == 0. {
        return Err("empty pressure or trajectory comparison".into());
    }
    let mut handoff = None;
    let mut archive = vec![];
    for (i, pre) in preceding.iter().enumerate() {
        let mut frame = Frame::default();
        origins.add_lane(
            &mut frame,
            pre,
            PRIOR,
            -0.77 * DVec3::Y,
            COLORS[2],
            peak_bin,
        )?;
        let corner = matched(a, pre.original_time_s);
        if let Some(f) = corner {
            origins.add_lane(&mut frame, f, EARLY, DVec3::ZERO, COLORS[0], peak_bin)?;
        }
        if let (Some(f), Some(s)) = (corner, matched(b, pre.original_time_s)) {
            origins.add_lane(&mut frame, s, LATE, 0.77 * DVec3::Y, COLORS[1], peak_bin)?;
            if (f.original_time_s - late.start_s).abs() < CLOCK_TOLERANCE_S {
                handoff = Some(i);
            }
            let position = f.com_m.distance(s.com_m) / max_shift;
            frame.add(
                &format!("Path separation, 0 to {:.2} um", 1e6 * max_shift),
                DVec3::new(0., 0., -0.55),
                DQuat::IDENTITY,
                cuboid([0.009, 0.42, 0.003]),
                IDLE,
            );
            frame.add(
                "Measured path separation",
                DVec3::new(0., 0.84 * (position - 0.5), -0.55),
                DQuat::IDENTITY,
                cuboid([0.011, 0.012, 0.012]),
                COLORS[2],
            );
        }
        archive.push(frame.finish(
            Duration::try_from_secs_f64(pre.original_time_s - origins.origin)?,
            i + 1,
        ));
    }
    if out.exists() {
        return Err(format!("output exists at {}", out.display()).into());
    }
    let parent = out.parent().unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = tempfile::Builder::new()
        .prefix(".corner-preload-view-")
        .tempdir_in(parent)?;
    gz_sim_cli::export_archive_at(
        &stage.path().join("replay"),
        Default::default(),
        0,
        Duration::try_from_secs_f64(preceding[0].original_time_s - origins.origin)?,
        archive,
    )?;
    let index = json!({"kind":"authored-archive-v1","corner_pressure_bins":BINS,"dt_ns":1,
        "captures":{"line_start":handoff.ok_or("missing former handoff clock")?,"last_measured":preceding.len()-1}});
    fs::write(
        stage.path().join("frames.json"),
        serde_json::to_vec_pretty(&index)?,
    )?;
    chart(
        &stage.path().join("corner-preload.svg"),
        &early,
        &late,
        control,
        origins.origin,
    )?;
    fs::rename(stage.path(), out)?;
    Ok(())
}
