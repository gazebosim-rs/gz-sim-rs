// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::{collections::BTreeSet, time::Duration};

pub(super) fn write(out: &Path, old: &Trial, new: &Trial, single: &Trial) -> Result<()> {
    let mut lanes = [old.clone(), new.clone(), single.clone()];
    let names = ["Before change", "Updated weld", "Single body"];
    for (lane, name) in lanes.iter_mut().zip(names) {
        lane.name = name.to_owned();
    }
    let first = lanes[0].frames[0].tick;
    let take = (0..=END - first)
        .step_by(6)
        .chain(0..=6)
        .chain([END - first])
        .collect::<BTreeSet<_>>();
    let maximum = lanes
        .iter()
        .flat_map(|run| &run.frames)
        .map(|f| f.normal_impulse_ns)
        .fold(0., f64::max);
    let colors = [
        [0.98, 0.58, 0.20, 1.],
        [0.12, 0.80, 0.71, 1.],
        [0.44, 0.66, 0.98, 1.],
    ];
    let axes = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);
    let mut archive = Vec::new();
    let geometry = |size: [f64; 3]| json!({"kind":"box","size":size});
    for (seq, &i) in take.iter().enumerate() {
        let mut frame = soft_replay::Frame::default();
        for (lane, run) in lanes.iter().enumerate() {
            let name = &run.name;
            let s = &run.frames[i];
            let offset = 0.59 * (lane as f64 - 1.) * DVec3::Y;
            for (label, v) in [("foot", &s.foot), ("upper", &s.upper)] {
                frame.add(
                    &format!("{name} {label}"),
                    offset + axes * DVec3::from(v.p),
                    axes * DQuat::from_array(v.q),
                    geometry(v.size),
                    colors[lane],
                );
            }
            frame.add(
                &format!("{name} floor section"),
                offset - 0.01 * DVec3::Z,
                DQuat::IDENTITY,
                geometry([0.35, 0.48, 0.02]),
                [0.37, 0.43, 0.49, 1.],
            );
            let center = offset - 0.08 * DVec3::Z;
            let scale = [0.52, 0.57, 0.66, 1.];
            let marker = colors[lane];
            frame.edge(
                &format!("{name} impulse scale"),
                center - 0.17 * DVec3::Y,
                center + 0.17 * DVec3::Y,
                0.34,
                0.005,
                scale,
            );
            frame.add(
                &format!("{name} ground impulse"),
                center + 0.34 * (s.normal_impulse_ns / maximum - 0.5) * DVec3::Y,
                DQuat::IDENTITY,
                geometry([0.015, 0.012, 0.03]),
                marker,
            );
            let origin = offset - 0.29 * DVec3::Z;
            for (axis, label) in [(DVec3::Y, "X"), (DVec3::Z, "Y")] {
                frame.edge(
                    &format!("{name} lateral {label} scale"),
                    origin - 0.13 * axis,
                    origin + 0.13 * axis,
                    0.26,
                    0.0035,
                    scale,
                );
            }
            let shift = DVec3::from(s.com) - DVec3::from(run.frames[0].com);
            frame.add(
                &format!("{name} horizontal travel at 100x"),
                origin + DVec3::new(0.01, 100. * shift.x, 100. * shift.y),
                DQuat::IDENTITY,
                json!({"kind":"sphere","radius":0.012}),
                marker,
            );
        }
        archive.push(frame.finish(Duration::from_micros((first + i) as u64 * 500), seq + 1));
    }
    let dir = out.join("viewer");
    fs::create_dir(&dir)?;
    gz_sim_cli::export_archive_at(
        &dir.join("replay"),
        Default::default(),
        0,
        Duration::from_micros(first as u64 * 500),
        archive,
    )?;
    fs::write(
        dir.join("frames.json"),
        serde_json::to_vec_pretty(&json!({"kind":"authored-archive-v1","dt_ns":1,
        "welded_foot":true,"welded_impact_branch":true,"origin_ns":first*500000,"captures":{"first_impact":1,"after_impact":3,"resting":take.len()-1}}))?,
    )?;
    fs::write(out.join("replay_samples.json"), serde_json::to_vec(&lanes)?)?;
    plot_paths(out, &lanes)?;
    Ok(())
}

fn plot_paths(out: &Path, lanes: &[Trial; 3]) -> Result<()> {
    use plotters::prelude::*;
    let path = out.join("horizontal-motion.svg");
    let canvas = SVGBackend::new(&path, (1024, 440)).into_drawing_area();
    canvas.fill(&WHITE)?;
    let plots = canvas.split_evenly((1, 2));
    let colors = [
        RGBColor(186, 96, 12),
        RGBColor(0, 114, 103),
        RGBColor(31, 82, 163),
    ];
    for (axis, area) in plots.into_iter().enumerate() {
        let points = |run: &Trial| {
            run.frames
                .iter()
                .map(|f| {
                    (
                        f.tick as f64 * 0.5,
                        1000. * (f.com[axis] - run.frames[0].com[axis]),
                    )
                })
                .collect::<Vec<_>>()
        };
        let data = lanes.each_ref().map(points);
        let (mut lo, mut hi) = (0_f64, 0_f64);
        for (_, y) in data.iter().flatten() {
            lo = lo.min(*y);
            hi = hi.max(*y);
        }
        let pad = ((hi - lo) * 0.08).max(0.01);
        let mut chart = ChartBuilder::on(&area)
            .margin(16)
            .caption(
                ["Sideways motion along X", "Sideways motion along Y"][axis],
                ("sans-serif", 22),
            )
            .x_label_area_size(52)
            .y_label_area_size(67)
            .build_cartesian_2d(84f64..180f64, (lo - pad)..(hi + pad))?;
        chart
            .configure_mesh()
            .x_desc("Time from release (ms)")
            .y_desc("COM travel from 84 ms (mm)")
            .draw()?;
        for (i, trace) in data.iter().enumerate() {
            let color = colors[i];
            let style = color.stroke_width([3, 3, 1][i]);
            chart
                .draw_series(LineSeries::new(trace.iter().copied(), style))?
                .label(&lanes[i].name)
                .legend(move |(x, y)| PathElement::new([(x, y), (x + 20, y)], style));
        }
        chart
            .configure_series_labels()
            .background_style(WHITE.mix(0.9))
            .border_style(WHITE)
            .position(SeriesLabelPosition::LowerRight)
            .draw()?;
    }
    canvas.present()?;
    Ok(())
}
