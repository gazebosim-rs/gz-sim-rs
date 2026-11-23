// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Show measured contact refreshes and native solver iterations at the second loaded step.
use super::*;
use plotters::prelude::{LineSeries, SeriesLabelPosition};

fn com(s: &Sample) -> DVec3 {
    (DVec3::from(s.foot.position)
        + DVec3::from(
            s.upper
                .as_ref()
                .expect("validated two-link recording")
                .position,
        ))
        * 0.5
}
fn work(s: &Sample) -> Result<[u32; 2]> {
    let n = s
        .native_steps
        .ok_or("missing native contact-refresh count")?;
    let i = s
        .solver_iterations
        .ok_or("missing native iteration count")?;
    if n == 0 || i == 0 {
        return Err("a solved interval must report positive effort".into());
    }
    Ok([n, i])
}

fn com_plot(path: &Path, rows: [&[Sample]; 3], onset: u64, second: u64) -> Result<()> {
    let [candidate, baseline, fine] = rows;
    let error = |a: &Sample, b: &Sample| com(a).distance(com(b)) * 1e6;
    let max = [candidate, baseline]
        .iter()
        .flat_map(|s| s.iter().zip(fine))
        .map(|(a, b)| error(a, b))
        .fold(0., f64::max);
    let area = SVGBackend::new(path, (900, 530)).into_drawing_area();
    area.fill(&WHITE)?;
    let area = area.titled(
        "Center-of-mass error through the five-millisecond landing window",
        ("sans-serif", 25),
    )?;
    let mut chart = ChartBuilder::on(&area)
        .margin(26)
        .x_label_area_size(45)
        .y_label_area_size(72)
        .build_cartesian_2d(0.0_f64..5.0, 0.0_f64..(max * 1.15).max(1e-3))?;
    chart
        .configure_mesh()
        .max_light_lines(1)
        .x_desc("Time from start of first loaded outer step (ms)")
        .y_desc("Distance from the four-substep reference (µm)")
        .draw()?;
    let ms = |s: &Sample| (s.tick - (onset - 1)) as f64 * 0.5;
    let t = (second - (onset - 1)) as f64 * 0.5;
    chart
        .draw_series([PathElement::new(
            [(t, 0.), (t, max * 1.07)],
            RGBColor(173, 181, 189),
        )])?
        .label("Second loaded reference tick")
        .legend(|(x, y)| PathElement::new([(x, y), (x + 21, y)], RGBColor(173, 181, 189)));
    for (row, name, color, radius) in [
        (
            baseline,
            "Earlier two-tick policy",
            RGBColor(188, 100, 41),
            5,
        ),
        (
            candidate,
            "Selected prospective policy",
            RGBColor(10, 126, 116),
            3,
        ),
    ] {
        let pts = row
            .iter()
            .zip(fine)
            .map(|(a, b)| (ms(a), error(a, b)))
            .collect::<Vec<_>>();
        chart
            .draw_series(LineSeries::new(pts.iter().copied(), color.stroke_width(2)))?
            .label(name)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 21, y)], color.stroke_width(3)));
        chart.draw_series(
            pts.into_iter()
                .map(|pt| Circle::new(pt, radius, color.filled())),
        )?;
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.92))
        .border_style(RGBColor(206, 215, 216))
        .draw()?;
    area.present()?;
    Ok(())
}

pub(in super::super) fn export(dir: &Path, out: &Path, onset: u64) -> Result<()> {
    let files = [
        "traces/candidate.json",
        "traces/baseline.json",
        "traces/fine.json",
    ];
    let [c, b, f] = files.map(|path| read::<Recording>(&dir.join(path)));
    let records = [c?, b?, f?];
    let g = &records[0].geometry;
    if records.iter().any(|r| r.geometry != *g) {
        return Err("solver controls must use the same fixture".into());
    }
    let incoming = |r: &Recording| {
        r.frames.first().map(|s| {
            (
                s.tick,
                s.time_us,
                s.foot.position,
                s.upper.as_ref().map(|b| b.position),
                s.foot.orientation,
                s.upper.as_ref().map(|b| b.orientation),
                s.pad.position,
            )
        })
    };
    if incoming(&records[0]).is_none()
        || records[1..]
            .iter()
            .any(|r| incoming(r) != incoming(&records[0]))
    {
        return Err("missing common incoming pose".into());
    }
    let begin = onset.checked_sub(5).ok_or("need a pre-impact lead-in")?;
    let end = onset.checked_add(34).ok_or("end tick overflow")?;
    let [c, b, f] = records.each_ref().map(|r| interval(r, begin, end));
    let rows = [c?, b?, f?];
    let conf = [rows[0], rows[2]];
    let mut max_steps = 0u32;
    let mut max_iterations = 0u32;
    let mut max_pair = 0f64;
    let mut max_corner = 0f64;
    let mut low_pad = f64::INFINITY;
    for s in conf.iter().flat_map(|row| row.iter()) {
        let [n, it] = work(s)?;
        max_steps = max_steps.max(n);
        max_iterations = max_iterations.max(it);
        max_pair = max_pair.max(s.normal_impulse_ns);
        max_corner = s.corner_ns.into_iter().flatten().fold(max_corner, f64::max);
        low_pad = low_pad.min(s.pad.position[2] - g.pad_half_extents[2]);
    }
    if max_steps < 2 || max_iterations == 0 || max_corner <= 0. || max_pair <= 0. {
        return Err("missing budget or contact comparison".into());
    }
    // A scored interval contains ten solved ticks plus its initial boundary.
    let win = rows.map(|r| &r[4..15]);
    let mut second = [0; 3];
    for (k, r) in win.iter().enumerate() {
        let mut positive = r[1..].iter().filter(|s| s.normal_impulse_ns > 0.);
        let first = positive.next().ok_or("missing first loaded solve")?;
        second[k] = positive.next().ok_or("missing second loaded solve")?.tick;
        if first.tick != onset {
            return Err("requested onset differs from a measured first load".into());
        }
    }
    let labels = ["prospective_schedule", "fixed_four_substeps"];
    let origin = begin * 500;
    let spacing = 2.5 * g.pad_half_extents[0];
    let sole_z = low_pad - 0.52 * g.pad_half_extents[0];
    let [hx, hy, _] = g.foot_half_extents;
    let mut archive = Vec::with_capacity(conf[0].len());
    for i in 0..conf[0].len() {
        let mut frame = Frame::default();
        for (k, lane) in conf.iter().enumerate() {
            let s = &lane[i];
            let name = labels[k];
            let offset = (k as f64 - 0.5) * spacing;
            let display = LaneDisplay {
                label: name,
                color: LANE[k],
                offset,
                sole_z,
                full: max_corner,
            };
            display.add(
                &mut frame,
                g,
                [
                    &s.foot,
                    s.upper.as_ref().ok_or("missing articulated upper link")?,
                    &s.pad,
                ],
                s.corner_ns.ok_or("missing quadrant impulses")?,
            )?;
            let [sub, it] = work(s)?;
            for (j, (quantity, fraction, color)) in [
                ("pair_impulse", s.normal_impulse_ns / max_pair, LANE[k]),
                ("native_steps", sub as f64 / max_steps as f64, UPPER),
                ("solver_iterations", it as f64 / max_iterations as f64, PAD),
            ]
            .into_iter()
            .enumerate()
            {
                let z = sole_z - (1.62 + 0.5 * j as f64) * hy;
                display.gauge(&mut frame, quantity, hx, z, fraction, color);
            }
        }
        archive.push(frame.finish(Duration::from_micros(conf[0][i].time_us - origin), i + 1));
    }
    fs::create_dir_all(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), archive)?;
    com_plot(&out.join("com-error.svg"), win, onset, second[2])?;
    let captures = BTreeMap::from([
        ("first_loaded", (onset - begin) * 500),
        ("second_loaded_reference", (second[2] - begin) * 500),
        ("second_loaded_candidate", (second[0] - begin) * 500),
        ("five_ms", (onset + 9 - begin) * 500),
    ]);
    let com_curve=win[2].iter().enumerate().map(|(i,reference)|json!({
        "tick":reference.tick,"time_us":reference.time_us,"candidate_error_um":com(&win[0][i]).distance(com(reference))*1e6,
        "baseline_error_um":com(&win[1][i]).distance(com(reference))*1e6
    })).collect::<Vec<_>>();
    let index = json!({"kind":"rebound-budget-v1","origin_us":origin,"dt_ns":1000,"ticks":conf[0].iter().map(|s|s.time_us-origin).collect::<Vec<_>>(),
        "captures":captures,"sole_z":sole_z,"spacing":spacing,"full_corner_ns":max_corner,"peak_pair_ns":max_pair,"max_native_steps":max_steps,
        "max_solver_iterations":max_iterations,"analysis_onset_tick":onset,"com_curve":com_curve,
        "lanes":[{"file":files[0],"label":labels[0]},{"file":files[2],"label":labels[1]}]});
    fs::write(out.join("frames.json"), serde_json::to_vec_pretty(&index)?)?;
    Ok(())
}
