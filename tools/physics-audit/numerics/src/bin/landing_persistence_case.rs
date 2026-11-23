// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Locate the first sustained ranking reversal on the complete public clock of a native three-stroke replay.
use glam::DVec3;
use plotters::prelude::*;
use serde_json::{Value, json};
use std::{env, fs, path::Path};
type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
fn read(p: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(p)?)?)
}
fn array(v: &Value) -> Result<&Vec<Value>> {
    v.as_array().ok_or_else(|| "array missing".into())
}
fn real(v: &Value) -> Result<f64> {
    v.as_f64()
        .filter(|v| v.is_finite())
        .ok_or_else(|| "nonfinite measurement".into())
}
fn tick(v: &Value) -> Result<u64> {
    v.as_u64().ok_or_else(|| "integer clock missing".into())
}
fn vec(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from(serde_json::from_value::<[f64; 3]>(v.clone())?))
}
fn close(x: f64, y: f64) -> Result<()> {
    if (x - y).abs() > 1e-11 * x.abs().max(y.abs()).max(1.) {
        return Err(format!("native readback {x} differs from {y}").into());
    }
    Ok(())
}
struct Frame {
    tick: u64,
    ms: f64,
    com: [DVec3; 3],
    spin: [DVec3; 3],
    normal: [f64; 3],
    tangent: [DVec3; 3],
    overlap: [bool; 3],
    errors: [f64; 2],
}
impl Frame {
    fn delta(&self) -> f64 {
        self.errors[1] - self.errors[0]
    }
}
fn first_rank_change(all: &[Frame], threshold: f64, count: usize) -> Option<usize> {
    all.get(26..)?
        .windows(count)
        .position(|w| w.iter().all(|r| r.delta() >= threshold))
        .map(|j| 26 + j)
}
fn milestone(all: &[Frame], k: usize) -> Value {
    let c = &all[k];
    let neighbors=[0,1,2].map(|j|{
        let last=all[..=k].iter().rfind(|s|s.normal[j]>0.);
        let next=all[k..].iter().find(|s|s.normal[j]>0.);
        let load=|r:&Frame|json!({"tick":r.tick,"ms_since_cut":r.ms,"normal_ns":r.normal[j]});
        json!({"at_tick_normal_ns":c.normal[j],"tangent_ns":c.tangent[j].to_array(),"footprint_overlap":c.overlap[j],
        "previous_including_now":last.map(load),"next_including_now":next.map(load)})
    });
    json!({"tick":c.tick,"ms_since_cut":c.ms,"ordinary_mm":c.errors[0],"finite_mm":c.errors[1],"finite_minus_ordinary_mm":c.delta(),"loads":neighbors})
}
fn figure(dir: &Path, all: &[Frame], cut: u64, captures: &[(String, usize)]) -> Result<()> {
    let p = dir.join("persistence-curve.svg");
    let root = SVGBackend::new(&p, (1190, 900)).into_drawing_area();
    root.fill(&WHITE)?;
    root.draw(&Text::new(
        "From the first landing to a later change in trajectory ranking",
        (22, 35),
        ("sans-serif", 24).into_font(),
    ))?;
    root.draw(&Text::new(format!("Every 0.5 ms public endpoint after cut tick {cut} • both first-impact treatments use the same 512 calls throughout the suffix"),(22,62),("sans-serif",15).into_font()))?;
    let charts = root.margin(82, 26, 14, 14).split_evenly((3, 1));
    let t = all.last().unwrap().ms;
    let a = RGBColor(187, 85, 34);
    let b = RGBColor(16, 128, 103);
    let fine = RGBColor(53, 99, 172);
    let max = all.iter().flat_map(|r| r.errors).fold(1f64, f64::max);
    let mut upper = ChartBuilder::on(&charts[0])
        .margin(5)
        .x_label_area_size(32)
        .y_label_area_size(74)
        .build_cartesian_2d(0.0..t, 0.0..max * 1.08)?;
    upper
        .configure_mesh()
        .y_desc("COM error (mm)")
        .x_desc("Time since common cut (ms)")
        .light_line_style(WHITE.mix(0.2))
        .draw()?;
    for (j, (name, color)) in [("Ordinary first impact", a), ("Finite first impact", b)]
        .into_iter()
        .enumerate()
    {
        upper
            .draw_series(LineSeries::new(
                all.iter().map(|r| (r.ms, r.errors[j])),
                color.stroke_width(2),
            ))?
            .label(name)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 18, y)], color.stroke_width(3)));
    }
    upper
        .configure_series_labels()
        .background_style(WHITE.mix(0.9))
        .border_style(BLACK.mix(0.25))
        .draw()?;
    let limit = all.iter().map(|r| r.delta().abs()).fold(1f64, f64::max) * 1.08;
    let mut signed = ChartBuilder::on(&charts[1])
        .margin(5)
        .x_label_area_size(32)
        .y_label_area_size(74)
        .build_cartesian_2d(0.0..t, -limit..limit)?;
    signed
        .configure_mesh()
        .y_desc("Finite minus ordinary (mm)")
        .x_desc("Time since common cut (ms)")
        .light_line_style(WHITE.mix(0.2))
        .draw()?;
    signed.draw_series(LineSeries::new([(0., 0.), (t, 0.)], BLACK.mix(0.4)))?;
    signed.draw_series(LineSeries::new(
        all.iter().map(|r| (r.ms, r.delta())),
        fine.stroke_width(2),
    ))?;
    for (_, k) in captures {
        let c = &all[*k];
        signed.draw_series(std::iter::once(Circle::new(
            (c.ms, c.delta()),
            4,
            BLACK.filled(),
        )))?;
    }
    let center = captures.last().map(|(_, k)| *k).unwrap_or(all.len() - 1);
    let lo = center.saturating_sub(20);
    let hi = (center + 20).min(all.len() - 1);
    let win = &all[lo..=hi];
    let jmax = win.iter().flat_map(|r| r.normal).fold(0.001f64, f64::max) * 1.1;
    let mut chart = ChartBuilder::on(&charts[2])
        .margin(5)
        .x_label_area_size(35)
        .y_label_area_size(74)
        .build_cartesian_2d(all[lo].ms - 0.25..all[hi].ms + 0.25, 0.0..jmax)?;
    chart
        .configure_mesh()
        .x_desc("Time since common cut around the sustained change (ms)")
        .y_desc("Normal impulse (N·s)")
        .light_line_style(WHITE.mix(0.2))
        .draw()?;
    for (j, (title, col)) in [("Ordinary", a), ("Finite", b), ("Continuous 4,096", fine)]
        .into_iter()
        .enumerate()
    {
        chart
            .draw_series(win.iter().map(|r| {
                let x = r.ms + (j as f64 - 1.) * 0.10;
                Rectangle::new([(x - 0.045, 0.), (x + 0.045, r.normal[j])], col.filled())
            }))?
            .label(title)
            .legend(move |(x, y)| Rectangle::new([(x, y - 2), (x + 16, y + 2)], col.filled()));
    }
    chart
        .configure_series_labels()
        .background_style(WHITE.mix(0.9))
        .border_style(BLACK.mix(0.25))
        .draw()?;
    root.present()?;
    Ok(())
}
fn main() -> Result<()> {
    let args = env::args_os().skip(1).collect::<Vec<_>>();
    if args.len() != 1 {
        return Err("landing_persistence_case CASE_DIR".into());
    }
    let dir = Path::new(&args[0]);
    let tracking = read(&dir.join("tracking.json"))?;
    let cut = tick(&tracking["cut_tick"])?;
    let mut frames = Vec::new();
    for (i, row) in array(&tracking["frames"])?.iter().enumerate() {
        let n = tick(&row["tick"])?;
        if n != cut + i as u64 + 1 {
            return Err("non-contiguous public trajectory".into());
        }
        let tr = |field: &str| -> Result<[DVec3; 3]> {
            if array(&row[field])?.len() != 3 {
                return Err("missing native lane".into());
            }
            Ok([
                vec(&row[field][0])?,
                vec(&row[field][1])?,
                vec(&row[field][2])?,
            ])
        };
        let com = tr("com_m")?;
        let errors = [0, 1].map(|j| 1000. * com[j].distance(com[2]));
        frames.push(Frame {
            tick: n,
            ms: (n - cut) as f64 * 0.5,
            com,
            spin: tr("spin_rad_s")?,
            normal: serde_json::from_value(row["normal_ns"].clone())?,
            tangent: tr("tangent_ns")?,
            overlap: [0, 1, 2].map(|j| !row["projected_gap_m"][j].is_null()),
            errors,
        });
    }
    if frames.last().is_none_or(|r| r.tick != 2171) {
        return Err("incomplete three-stroke run".into());
    }
    let mut checks = 0;
    for (j, label) in array(&tracking["label"])?.iter().enumerate() {
        let name = label.as_str().ok_or("missing lane name")?;
        let path = read(&dir.join(format!("traces/{name}.json")))?;
        for row in array(&path["frames"])?.iter().skip(1) {
            let n = tick(&row["tick"])?;
            let r = &frames[(n - cut - 1) as usize];
            let c = (vec(&row["foot"]["position"])? + vec(&row["upper"]["position"])?) * 0.5;
            close(c.distance(r.com[j]), 0.)?;
            close(
                vec(&row["foot"]["angular_velocity"])?.distance(r.spin[j]),
                0.,
            )?;
            close(real(&row["normal_impulse_ns"])?, r.normal[j])?;
            close(vec(&row["tangent_on_foot_ns"])?.distance(r.tangent[j]), 0.)?;
            checks += 1;
        }
    }
    let manifest = read(&dir.join("comparison.json"))?;
    let mut milestones = Vec::new();
    let mut captures = Vec::new();
    for (name, bound, count) in [
        ("first_three_ticks_over_ten_um", 0.01, 3),
        ("first_ten_ticks_over_one_mm", 1., 10),
    ] {
        let k = first_rank_change(&frames, bound, count);
        if manifest["capture_ticks"][name].as_u64() != k.map(|v| frames[v].tick) {
            return Err("browser capture tick differs from actual full-clock threshold".into());
        }
        if let Some(i) = k {
            captures.push((name.into(), i));
        }
        milestones.push(json!({"minimum_change_mm":bound,"consecutive_public_ticks":count,"first":k.map(|j|milestone(&frames,j))}));
    }
    let early = milestone(&frames, 25);
    let last = milestone(&frames, frames.len() - 1);
    let summary = json!({"case":tracking["case"],"cut_tick":cut,"public_steps":frames.len(),"visual_lane_endpoints_checked":checks,"fine_reference":"continuous 4096 r1","at_13ms":early,"first_sustained_changes":milestones,"last":last});
    fs::write(
        dir.join("persistence-curve.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    figure(dir, &frames, cut, &captures)
}
