// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare solver selection, finite-support clearance and native impulses through a recontact.
use super::*;
#[path = "rebound_budget.rs"]
pub(super) mod budget;
#[path = "rebound_flight.rs"]
pub(super) mod flight;
#[path = "weak_recontact.rs"]
pub(super) mod weak;
use clap::ValueEnum;
use plotters::prelude::{
    ChartBuilder, Circle, Color, IntoDrawingArea, PathElement, RGBColor, SVGBackend, WHITE,
};

#[derive(Copy, Clone, ValueEnum)]
pub(super) enum Deck {
    Flat,
    Edge,
}
impl Deck {
    fn name(self) -> &'static str {
        match self {
            Self::Flat => "flat",
            Self::Edge => "edge",
        }
    }
}
/// Impulse applied during the interval ending at this sample. Both vectors
/// are in the laboratory frame, evaluated at impact rather than at frame end.
#[derive(Deserialize)]
struct IntervalImpact {
    elapsed_ns: u64,
    point_m: [f64; 3],
    impulse_ns: [f64; 3],
}
#[derive(Deserialize)]
struct ContactInterval {
    duration_ns: u64,
    end_ns: u64,
    material: bool,
}
#[derive(Deserialize)]
struct MatchedStep {
    impulse_error_ns: f64,
    wall_p95_ns: u64,
}
#[derive(Deserialize)]
struct Sample {
    matched_step: Option<MatchedStep>,
    #[serde(default)]
    contact_intervals: Vec<ContactInterval>,
    #[serde(default)]
    interval_impacts: Vec<IntervalImpact>,
    tick: u64,
    time_us: u64,
    time_ns: Option<u64>,
    foot: Body,
    upper: Option<Body>,
    pad: Body,
    opponent: Option<Body>,
    selected: Option<bool>,
    weak_start: Option<bool>,
    secondary_start: Option<bool>,
    normal_impulse_ns: f64,
    support_normal_ns: Option<[f64; 2]>,
    corner_ns: Option<[f64; 4]>,
    gap_m: Option<f64>,
    native_steps: Option<u32>,
    solver_iterations: Option<u32>,
}
impl Sample {
    fn stamp_ns(&self) -> u64 {
        self.time_ns
            .unwrap_or_else(|| self.time_us.saturating_mul(1000))
    }
}
#[derive(Deserialize)]
struct Recording {
    geometry: Geometry,
    frames: Vec<Sample>,
}

fn interval(trace: &Recording, start: u64, end: u64) -> Result<&[Sample]> {
    interval_with_receipts(
        trace,
        start,
        end,
        false,
        false,
        false,
        (0, 500_000, false, false),
    )
}
fn interval_with_receipts(
    trace: &Recording,
    start: u64,
    end: u64,
    pair_totals_only: bool,
    sampled_ticks: bool,
    single_body: bool,
    clock: (u64, u64, bool, bool),
) -> Result<&[Sample]> {
    let i = trace
        .frames
        .binary_search_by_key(&start, |r| r.tick)
        .map_err(|_| "missing first tick")?;
    let j = trace
        .frames
        .binary_search_by_key(&end, |r| r.tick)
        .map_err(|_| "missing last tick")?;
    let rows = trace
        .frames
        .get(i..=j)
        .ok_or("invalid measurement interval")?;
    let (origin_ns, dt_ns, native_samples, interval_samples) = clock;
    if dt_ns == 0 {
        return Err("zero public step duration".into());
    }
    for (k, r) in rows.iter().enumerate() {
        // A source can begin at a known intra-tick restart. All subsequent
        // observations must still land on an actual 500 µs public boundary.
        let ns = r.stamp_ns();
        let cut_inside_tick = k == 0
            && r.time_ns.is_some()
            && r.native_steps == Some(0)
            && ns.checked_sub(origin_ns).map(|t| t / dt_ns) == Some(r.tick);
        let measured_time =
            r.time_ns.is_some() && ns >= origin_ns && (k == 0 || ns > rows[k - 1].stamp_ns());
        let valid_clock = if interval_samples {
            measured_time
        } else if native_samples {
            r.time_ns.is_some()
                && ns >= origin_ns
                && (k == 0 || ns > rows[k - 1].stamp_ns())
                && (k == 0 && r.native_steps == Some(0) || k > 0 && r.native_steps == Some(1))
        } else {
            cut_inside_tick
                || r.tick
                    .checked_mul(dt_ns)
                    .and_then(|n| n.checked_add(origin_ns))
                    == Some(ns)
        };
        if (!sampled_ticks && r.tick != start + k as u64)
            || k > 0 && r.tick <= rows[k - 1].tick
            || ns / 1000 != r.time_us
            || !valid_clock
        {
            return Err("measurement disagrees with its declared public clock".into());
        }
        if !r.contact_intervals.is_empty() {
            let mut end = 0_u64;
            for part in &r.contact_intervals {
                if part.duration_ns == 0 || end.checked_add(part.duration_ns) != Some(part.end_ns) {
                    return Err("contact intervals have a gap or overlap".into());
                }
                end = part.end_ns;
            }
            if k == 0
                || end != ns - rows[k - 1].stamp_ns()
                || r.native_steps.map(|n| n as usize) != Some(r.contact_intervals.len())
            {
                return Err("contact interval budget and clock disagree".into());
            }
        }
        for hit in &r.interval_impacts {
            let j = DVec3::from(hit.impulse_ns);
            if k == 0
                || hit.elapsed_ns > ns - rows[k - 1].stamp_ns()
                || !DVec3::from(hit.point_m).is_finite()
                || !j.length().is_finite()
                || j.length() == 0.
            {
                return Err("invalid measured interval impact".into());
            }
        }
        if r.matched_step.as_ref().is_some_and(|m| {
            k == 0
                || !m.impulse_error_ns.is_finite()
                || m.impulse_error_ns < 0.
                || m.wall_p95_ns == 0
        }) {
            return Err("invalid matched step measurements".into());
        }
        if r.upper.is_none() != single_body
            || r.opponent.is_some() != trace.geometry.opponent_half_extents.is_some()
            || !r.normal_impulse_ns.is_finite()
            || r.normal_impulse_ns < 0.
            || r.support_normal_ns.is_some_and(|load| {
                r.opponent.is_none()
                    || !pair_totals_only
                    || load.iter().any(|x| !x.is_finite() || *x < 0.)
                    || (load[0] + load[1] - r.normal_impulse_ns).abs() > 1e-9
            })
            || r.corner_ns.is_none() != pair_totals_only
            || r.corner_ns.is_some_and(|values| {
                values.iter().any(|j| !j.is_finite() || *j < 0.)
                    || (values.iter().sum::<f64>() - r.normal_impulse_ns).abs() > 1e-9
            })
            || r.gap_m.is_some_and(|x| !x.is_finite())
        {
            return Err("invalid native contact sample".into());
        }
    }
    Ok(rows)
}

struct Scales {
    corner: f64,
    peak: f64,
    lo: f64,
    hi: f64,
}

fn plot(out: &Path, rows: &[&[Sample]; 2], scales: &Scales, event: u64, deck: Deck) -> Result<()> {
    let area = SVGBackend::new(out, (1060, 740)).into_drawing_area();
    area.fill(&WHITE)?;
    let grid = area
        .titled(
            &format!("Recontact on the {} deck", deck.name()),
            ("sans-serif", 28),
        )?
        .split_evenly((2, 2));
    let lo = rows[0][0].time_us as f64 * 1e-3;
    let hi = rows[0].last().unwrap().time_us as f64 * 1e-3;
    for (k, lane) in rows.iter().enumerate() {
        let color = if k == 0 {
            RGBColor(13, 119, 114)
        } else {
            RGBColor(51, 97, 162)
        };
        let name = if k == 0 {
            "Rebound policy"
        } else {
            "1 mm policy"
        };
        for (j, (axis, y)) in [
            (
                "Supported clearance (mm)",
                1000. * scales.lo..1000. * scales.hi,
            ),
            ("Normal impulse per tick (N·s)", 0.0..scales.peak * 1.08),
        ]
        .into_iter()
        .enumerate()
        {
            let mut chart = ChartBuilder::on(&grid[j * 2 + k])
                .margin(12)
                .caption(name, ("sans-serif", 20))
                .x_label_area_size(42)
                .y_label_area_size(73)
                .build_cartesian_2d(lo..hi, y.clone())?;
            chart
                .configure_mesh()
                .max_light_lines(1)
                .x_desc("Simulation time (ms)")
                .y_desc(axis)
                .draw()?;
            chart.draw_series([PathElement::new(
                vec![(event as f64 * 0.5, y.start), (event as f64 * 0.5, y.end)],
                RGBColor(182, 191, 192),
            )])?;
            let val = |r: &Sample| {
                if j == 0 {
                    r.gap_m.map(|v| v * 1000.)
                } else {
                    Some(r.normal_impulse_ns)
                }
            };
            for pair in lane.windows(2) {
                if let (Some(a), Some(b)) = (val(&pair[0]), val(&pair[1])) {
                    chart.draw_series([PathElement::new(
                        vec![
                            (pair[0].time_us as f64 * 1e-3, a),
                            (pair[1].time_us as f64 * 1e-3, b),
                        ],
                        color.stroke_width(2),
                    )])?;
                }
            }
            chart.draw_series(lane.iter().filter(|r| r.selected == Some(true)).filter_map(
                |r| {
                    val(r).map(|v| {
                        Circle::new(
                            (r.time_us as f64 * 1e-3, v),
                            5,
                            RGBColor(200, 112, 20).filled(),
                        )
                    })
                },
            ))?;
        }
    }
    area.present()?;
    Ok(())
}

pub(super) fn export(
    dir: &Path,
    deck: Deck,
    out: &Path,
    start: u64,
    end: u64,
    event: u64,
) -> Result<()> {
    if !(start < event && event < end) {
        return Err("recontact must be inside the displayed interval".into());
    }
    let files = ["candidate", "legacy"].map(|mode| format!("traces/{}-{mode}.json", deck.name()));
    let [a, b] = files.each_ref().map(|f| read::<Recording>(&dir.join(f)));
    let (a, b) = (a?, b?);
    let g = &a.geometry;
    if g.foot_half_extents != b.geometry.foot_half_extents
        || g.upper_half_extents != b.geometry.upper_half_extents
        || g.pad_half_extents != b.geometry.pad_half_extents
    {
        return Err("inconsistent support geometry".into());
    }
    let rows = [interval(&a, start, end)?, interval(&b, start, end)?];
    let all = rows.iter().flat_map(|t| t.iter());
    if all
        .clone()
        .any(|r| r.selected.is_none() || r.secondary_start.is_none())
    {
        return Err("missing recorded rebound admission".into());
    }
    let corner = all
        .clone()
        .filter_map(|r| r.corner_ns)
        .flatten()
        .fold(0., f64::max);
    let peak = all.clone().map(|r| r.normal_impulse_ns).fold(0., f64::max);
    let lo = all.clone().filter_map(|r| r.gap_m).fold(0., f64::min);
    let hi = all.clone().filter_map(|r| r.gap_m).fold(0., f64::max);
    if corner <= 0. || hi <= lo {
        return Err("measurement window has no reaction or clearance variation".into());
    }
    let scales = Scales {
        corner,
        peak,
        lo,
        hi,
    };
    let [hx, hy, _] = g.foot_half_extents;
    let spacing = 2.5 * g.pad_half_extents[0];
    let sole_z = all
        .map(|s| s.pad.position[2] - g.pad_half_extents[2])
        .fold(f64::INFINITY, f64::min)
        - 0.52 * g.pad_half_extents[0];
    let labels = ["rebound_policy", "one_mm_policy"];
    let origin = rows[0][0].time_us;
    let mut frames = Vec::with_capacity(rows[0].len());
    for i in 0..rows[0].len() {
        let mut f = Frame::default();
        for (k, lane) in rows.iter().enumerate() {
            let r = &lane[i];
            let offset = (k as f64 - 0.5) * spacing;
            let name = labels[k];
            let display = LaneDisplay {
                label: name,
                color: LANE[k],
                offset,
                sole_z,
                full: scales.corner,
            };
            display.add(
                &mut f,
                g,
                [
                    &r.foot,
                    r.upper.as_ref().ok_or("missing articulated upper link")?,
                    &r.pad,
                ],
                r.corner_ns.ok_or("missing quadrant impulses")?,
            )?;
            for (j, (quantity, fraction, color)) in [
                (
                    "supported_clearance",
                    r.gap_m.map_or(-0.12, |v| (v - lo) / (hi - lo)),
                    UPPER,
                ),
                ("pair_impulse", r.normal_impulse_ns / peak, LANE[k]),
                (
                    "solver_activity",
                    f64::from(r.selected.ok_or("missing recorded gate decision")?),
                    PAD,
                ),
            ]
            .into_iter()
            .enumerate()
            {
                let z = sole_z - (1.62 + 0.5 * j as f64) * hy;
                display.gauge(&mut f, quantity, hx, z, fraction, color);
                if j == 0 {
                    f.add(
                        &format!("{name}_clearance_zero"),
                        DVec3::new(0.012, offset + (-2. * lo / (hi - lo) - 1.) * hx, z),
                        DQuat::IDENTITY,
                        cuboid([0.0025, 0.0025, 0.012]),
                        IDLE,
                    );
                }
            }
        }
        frames.push(f.finish(Duration::from_micros(rows[0][i].time_us - origin), i + 1));
    }
    fs::create_dir_all(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
    plot(&out.join("rebound.svg"), &rows, &scales, event, deck)?;
    let mut marks = BTreeMap::from([
        ("recontact".to_owned(), (event - start) * 500),
        ("approach".into(), (event - start - 1) * 500),
    ]);
    for (n, r) in rows[0]
        .iter()
        .filter(|r| r.secondary_start == Some(true) && r.tick != event)
        .enumerate()
    {
        marks.insert(format!("secondary_{}", n + 1), r.time_us - origin);
    }
    let index = json!({"kind":"rebound-gate-v1","origin_us":origin,"dt_ns":1000,"ticks":rows[0].iter().map(|r|r.time_us-origin).collect::<Vec<_>>(),
        "sole_z":sole_z,"spacing":spacing,"full_corner_ns":scales.corner,"peak_pair_ns":peak,"gap_min_m":lo,"gap_max_m":hi,"captures":marks,
        "lanes":[{"file":files[0],"label":labels[0]},{"file":files[1],"label":labels[1]}]});
    fs::write(out.join("frames.json"), serde_json::to_vec_pretty(&index)?)?;
    Ok(())
}
