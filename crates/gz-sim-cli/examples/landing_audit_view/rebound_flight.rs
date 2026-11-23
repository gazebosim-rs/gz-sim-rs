// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare equal-mass assembly COM flight with constant gravity at shared outer boundaries.
use super::*;
use plotters::prelude::{LineSeries, SeriesLabelPosition, TriangleMarker};

const GRAVITY: f64 = -9.81;
const LABELS: [&str; 2] = ["coarse_flight", "fine_flight"];
const COLORS: [RGBColor; 2] = [RGBColor(13, 112, 107), RGBColor(44, 89, 163)];

#[derive(Clone, Copy)]
struct Cut {
    position: DVec3,
    velocity: DVec3,
    us: u64,
}
impl Cut {
    fn ballistic(self, time_us: u64) -> DVec3 {
        let dt = (time_us - self.us) as f64 * 1e-6;
        self.position + self.velocity * dt + DVec3::Z * (0.5 * GRAVITY * dt * dt)
    }
}
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

#[derive(Clone, Copy)]
struct FlightPoint {
    tick: u64,
    time_us: u64,
    z_m: f64,
    ballistic_z_m: f64,
    dz_um: Option<f64>,
    gravity_bias_um: Option<f64>,
}

fn flight(cut: Cut, native: &[Sample], first_load: u64) -> Result<Vec<FlightPoint>> {
    let mut bias = 0.;
    let mut last = cut.us;
    let mut out = Vec::with_capacity(native.len());
    for s in native {
        let z = com(s).z;
        let analytical = cut.ballistic(s.time_us).z;
        if !z.is_finite() || !analytical.is_finite() {
            return Err("nonfinite flight state".into());
        }
        // Semi-implicit Euler point mass, exact constant gravity and equal internal steps.
        // The measured articulated COM can also accumulate kinematic discretization error.
        let free = s.tick < first_load;
        if free && s.time_us > last {
            let h = (s.time_us - last) as f64 * 1e-6;
            let splits = s
                .native_steps
                .ok_or("missing free-flight native step count")?;
            let iters = s
                .solver_iterations
                .ok_or("missing free-flight iteration count")?;
            if splits == 0 || iters == 0 {
                return Err("invalid free-flight solve effort".into());
            }
            bias += 0.5 * GRAVITY * h * h / (f64::from(splits) * f64::from(iters));
        }
        out.push(FlightPoint {
            tick: s.tick,
            time_us: s.time_us,
            z_m: z,
            ballistic_z_m: analytical,
            dz_um: free.then_some((z - analytical) * 1e6),
            gravity_bias_um: free.then_some(bias * 1e6),
        });
        last = s.time_us;
    }
    Ok(out)
}

fn graph(path: &Path, cut: Cut, curves: &[Vec<FlightPoint>; 2], impact: [u64; 2]) -> Result<()> {
    let area = SVGBackend::new(path, (950, 760)).into_drawing_area();
    area.fill(&WHITE)?;
    let area = area.titled(
        "Free flight from the same post-impact state",
        ("sans-serif", 28),
    )?;
    let panels = area.split_evenly((2, 1));
    let ms = |p: &FlightPoint| (p.time_us - cut.us) as f64 * 1e-3;
    let duration = ms(curves[0].last().ok_or("empty flight")?);
    let all = curves.iter().flatten();
    let (lo, hi) = all
        .clone()
        .flat_map(|p| [p.z_m, p.ballistic_z_m])
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), z| {
            (lo.min(z * 1000.), hi.max(z * 1000.))
        });
    let margin = ((hi - lo) * 0.12).max(0.001);
    let mut chart = ChartBuilder::on(&panels[0])
        .margin(15)
        .x_label_area_size(37)
        .y_label_area_size(84)
        .build_cartesian_2d(0.0..duration, lo - margin..hi + margin)?;
    chart
        .configure_mesh()
        .max_light_lines(1)
        .x_desc("Time since the matched cut (ms)")
        .y_desc("Assembly COM height (mm)")
        .draw()?;
    chart
        .draw_series(LineSeries::new(
            curves[0].iter().map(|p| (ms(p), p.ballistic_z_m * 1000.)),
            RGBColor(81, 87, 92).stroke_width(3),
        ))?
        .label("Continuous gravity")
        .legend(|(x, y)| {
            PathElement::new([(x, y), (x + 24, y)], RGBColor(81, 87, 92).stroke_width(3))
        });
    for (k, line) in curves.iter().enumerate() {
        let color = COLORS[k];
        let name = ["One native step", "Four native steps"][k];
        chart
            .draw_series(LineSeries::new(
                line.iter().map(|p| (ms(p), p.z_m * 1000.)),
                color.stroke_width(1),
            ))?
            .label(name)
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 24, y)], color.stroke_width(3)));
        let hit = line
            .iter()
            .find(|p| p.tick == impact[k])
            .ok_or("missing first contact")?;
        chart.draw_series([TriangleMarker::new(
            (ms(hit), hit.z_m * 1000.),
            6,
            color.filled(),
        )])?;
    }
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::UpperLeft)
        .background_style(WHITE.mix(0.92))
        .border_style(RGBColor(211, 219, 219))
        .draw()?;

    let values = all
        .clone()
        .flat_map(|p| [p.dz_um, p.gravity_bias_um].into_iter().flatten());
    let min = values.clone().fold(0., f64::min);
    let max = values.fold(0., f64::max);
    let margin = (max - min).max(0.1) * 0.12;
    let mut chart = ChartBuilder::on(&panels[1])
        .margin(15)
        .x_label_area_size(43)
        .y_label_area_size(84)
        .build_cartesian_2d(0.0..duration, min - margin..max + margin)?;
    chart
        .configure_mesh()
        .max_light_lines(1)
        .x_desc("Time since the matched cut (ms)")
        .y_desc("Native COM minus ballistic height (µm)")
        .draw()?;
    for (k, line) in curves.iter().enumerate() {
        let color = COLORS[k];
        let pts = line
            .iter()
            .filter_map(|p| p.dz_um.map(|d| (ms(p), d)))
            .collect::<Vec<_>>();
        chart
            .draw_series(LineSeries::new(pts.iter().copied(), color.stroke_width(2)))?
            .label(["Measured, one native step", "Measured, four native steps"][k])
            .legend(move |(x, y)| PathElement::new([(x, y), (x + 24, y)], color.stroke_width(2)));
        chart.draw_series(line.iter().filter_map(|p| {
            p.gravity_bias_um
                .map(|dz| Circle::new((ms(p), dz), 3, color.stroke_width(1)))
        }))?;
    }
    chart
        .draw_series(std::iter::empty::<Circle<(f64, f64), i32>>())?
        .label("Point-mass integration bias")
        .legend(|(x, y)| Circle::new((x + 12, y), 4, RGBColor(81, 87, 92).stroke_width(1)));
    chart
        .configure_series_labels()
        .position(SeriesLabelPosition::LowerLeft)
        .background_style(WHITE.mix(0.92))
        .border_style(RGBColor(211, 219, 219))
        .draw()?;
    area.present()?;
    Ok(())
}

pub(in super::super) fn export(dir: &Path, out: &Path) -> Result<()> {
    let files = ["traces/coarse.json", "traces/fine.json"];
    let [a, b] = files.map(|f| read::<Recording>(&dir.join(f)));
    let records = [a?, b?];
    let g = &records[0].geometry;
    if g != &records[1].geometry {
        return Err("different matched flight geometry".into());
    }
    let start = records[0].frames.first().ok_or("empty coarse trace")?;
    let peer = records[1].frames.first().ok_or("empty fine trace")?;
    if start.tick != peer.tick
        || start.time_us != peer.time_us
        || start.foot != peer.foot
        || start.upper != peer.upper
        || start.pad != peer.pad
        || [
            &start.foot,
            start.upper.as_ref().ok_or("missing incoming upper link")?,
            &start.pad,
        ]
        .iter()
        .any(|b| b.velocity.is_none() || b.angular_velocity.is_none())
    {
        return Err("free flight requires identical incoming body poses and twists".into());
    }
    let v0 = DVec3::from(start.foot.velocity.ok_or("missing cut sole velocity")?);
    let v1 = DVec3::from(
        start
            .upper
            .as_ref()
            .and_then(|b| b.velocity)
            .ok_or("missing cut upper velocity")?,
    );
    let cut = Cut {
        position: com(start),
        velocity: (v0 + v1) * 0.5,
        us: start.time_us,
    };
    let [a, b] = records.each_ref().map(|r| {
        r.frames
            .iter()
            .skip(1)
            .find(|s| s.normal_impulse_ns > 0.)
            .map(|s| s.tick)
            .ok_or("missing resumed contact")
    });
    let hits = [a?, b?];
    let end = hits[0].max(hits[1]);
    if hits[0].min(hits[1]) <= start.tick + 1 {
        return Err("no unloaded interval before first resumed contact".into());
    }
    let [a, b] = records.each_ref().map(|r| interval(r, start.tick, end));
    let rows = [a?, b?];
    let curves = [
        flight(cut, rows[0], hits[0])?,
        flight(cut, rows[1], hits[1])?,
    ];
    let full = rows
        .iter()
        .flat_map(|s| s.iter())
        .filter_map(|s| s.corner_ns)
        .flatten()
        .fold(0., f64::max);
    let peak = rows
        .iter()
        .flat_map(|s| s.iter())
        .map(|s| s.normal_impulse_ns)
        .fold(0., f64::max);
    let error_scale = curves
        .iter()
        .flatten()
        .filter_map(|p| p.dz_um)
        .map(f64::abs)
        .fold(0., f64::max);
    if !(full > 0. && peak > 0. && error_scale > 0.) {
        return Err("missing native load or free-flight deviation".into());
    }
    let spacing = 2.5 * g.pad_half_extents[0];
    let [hx, hy, _] = g.foot_half_extents;
    let sole_z = rows
        .iter()
        .flat_map(|s| s.iter())
        .map(|s| s.pad.position[2] - g.pad_half_extents[2])
        .fold(f64::INFINITY, f64::min)
        - 0.52 * g.pad_half_extents[0];
    let mut archive = Vec::with_capacity(rows[0].len());
    for i in 0..rows[0].len() {
        let mut f = Frame::default();
        for (k, lane) in rows.iter().enumerate() {
            let name = LABELS[k];
            let s = &lane[i];
            let q = curves[k][i];
            let offset = (k as f64 - 0.5) * spacing;
            let display = LaneDisplay {
                label: name,
                color: LANE[k],
                offset,
                sole_z,
                full,
            };
            display.add(
                &mut f,
                g,
                [
                    &s.foot,
                    s.upper.as_ref().ok_or("missing articulated upper link")?,
                    &s.pad,
                ],
                s.corner_ns.ok_or("missing quadrant impulses")?,
            )?;
            let z = sole_z - 1.62 * hy;
            display.gauge(
                &mut f,
                "signed_height_error",
                hx,
                z,
                q.dz_um.map_or(-0.12, |d| 0.5 + 0.5 * d / error_scale),
                LANE[k],
            );
            f.add(
                &format!("{name}_error_zero"),
                DVec3::new(0.01, offset, z),
                DQuat::IDENTITY,
                cuboid([0.003, 0.0025, 0.012]),
                UPPER,
            );
            display.gauge(
                &mut f,
                "normal_reaction",
                hx,
                z - 0.5 * hy,
                s.normal_impulse_ns / peak,
                UPPER,
            );
            display.gauge(
                &mut f,
                "contact_resumed",
                hx,
                z - hy,
                f64::from(s.tick >= hits[k]),
                PAD,
            );
        }
        archive.push(f.finish(Duration::from_micros(rows[0][i].time_us - cut.us), i + 1));
    }
    let relative = |tick: u64| (tick - start.tick) * 500;
    let captures = BTreeMap::from([
        ("last_common_free", relative(hits[0].min(hits[1]) - 1)),
        ("coarse_contact", relative(hits[0])),
        ("fine_contact", relative(hits[1])),
    ]);
    let metrics=curves.each_ref().map(|line|line.iter().map(|p|json!({"tick":p.tick,"time_us":p.time_us,"native_z_m":p.z_m,"ballistic_z_m":p.ballistic_z_m,"error_z_um":p.dz_um,"euler_bias_um":p.gravity_bias_um})).collect::<Vec<_>>());
    let index = json!({"kind":"rebound-flight-v1","origin_us":cut.us,"dt_ns":1000,"ticks":rows[0].iter().map(|s|s.time_us-cut.us).collect::<Vec<_>>(),"captures":captures,"full_corner_ns":full,"sole_z":sole_z,"spacing":spacing,"error_scale_um":error_scale,"peak_pair_ns":peak,"gravity_z_m_s2":GRAVITY,"first_resumed_ticks":hits,"flight_curves":metrics,"lanes":[{"file":files[0],"label":LABELS[0]},{"file":files[1],"label":LABELS[1]}]});
    fs::create_dir_all(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), archive)?;
    fs::write(out.join("frames.json"), serde_json::to_vec_pretty(&index)?)?;
    graph(&out.join("flight.svg"), cut, &curves, hits)?;
    Ok(())
}
