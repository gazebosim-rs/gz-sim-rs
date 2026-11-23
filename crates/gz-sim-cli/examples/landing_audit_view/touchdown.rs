// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Paired native touchdown observations at common physical boundaries.
use super::*;
use clap::ValueEnum;
#[path = "touchdown_figures.rs"]
mod plots;
pub(super) use plots::draw as figures;

#[derive(Clone, Copy, ValueEnum)]
pub(super) enum Comparison {
    /// 125 µs versus 500 µs at touchdown on the always-dynamic history.
    Resolution,
    /// Two receiving contact histories given the same always-dynamic motion.
    Matched,
    /// Both unmodified historical lanes with 125 µs touchdown steps.
    Histories,
}
impl Comparison {
    fn lanes(self) -> [(&'static str, &'static str); 2] {
        match self {
            Self::Resolution => [
                ("reference-native-4x", "dynamic_fine"),
                ("reference-native-1x", "dynamic_coarse"),
            ],
            Self::Matched => [
                ("switched-reference-4x", "switched_contacts"),
                ("reference-reference-4x", "dynamic_contacts"),
            ],
            Self::Histories => [
                ("switched-native-4x", "switched_history"),
                ("reference-native-4x", "dynamic_history"),
            ],
        }
    }
}
#[derive(Deserialize)]
struct WindowFrame {
    time_us: u64,
    dt_us: u64,
    foot: Body,
    upper: Body,
    pad: Body,
    normal_impulse_ns: f64,
    corner_ns: [f64; 4],
}
#[derive(Deserialize)]
struct WindowTrace {
    geometry: Geometry,
    frames: Vec<WindowFrame>,
}
struct PointAtBoundary<'a> {
    frame: &'a WindowFrame,
    corners: [f64; 4],
    window_ns: f64,
}
// Integrate all native impulses ending in (previous, current]. Each displayed
// body pose is sampled at the boundary itself. No missing pose is interpolated.
fn sample<'a>(
    trace: &'a WindowTrace,
    times: &[u64],
    origin: u64,
) -> Result<Vec<PointAtBoundary<'a>>> {
    let mut rows = Vec::with_capacity(times.len());
    let mut it = trace.frames.iter().peekable();
    let mut prev = None;
    let mut window_ns = 0.;
    for &at in times {
        let mut corners = [0.; 4];
        let mut row = None;
        while it.peek().is_some_and(|r| r.time_us <= at) {
            let r = it.next().unwrap();
            if let Some(t) = prev {
                if t >= r.time_us || r.time_us - t != r.dt_us {
                    return Err("discontinuous native timestep".into());
                }
                if r.corner_ns.iter().any(|j| !j.is_finite() || *j < 0.)
                    || !r.normal_impulse_ns.is_finite()
                    || (r.corner_ns.iter().sum::<f64>() - r.normal_impulse_ns).abs() > 1e-9
                {
                    return Err("corner impulses disagree with native pair".into());
                }
                for (j, v) in corners.iter_mut().zip(r.corner_ns) {
                    *j += v;
                }
                if r.time_us <= origin + 5000 {
                    window_ns += r.normal_impulse_ns;
                }
            } else if r.time_us != origin {
                return Err("lane begins at another source time".into());
            }
            prev = Some(r.time_us);
            row = Some(r);
        }
        let frame = row
            .filter(|r| r.time_us == at)
            .ok_or("unmeasured common boundary")?;
        rows.push(PointAtBoundary {
            frame,
            corners,
            window_ns,
        });
    }
    if it.next().is_some() {
        return Err("paired traces end on different boundaries".into());
    }
    Ok(rows)
}

pub(super) fn export(dir: &Path, comparison: Comparison, out: &Path) -> Result<()> {
    let lanes = comparison.lanes();
    let read_lane =
        |(file, _): (&str, &str)| read::<WindowTrace>(&dir.join(format!("{file}.json")));
    let [a, b] = lanes.map(read_lane);
    let (a, b) = (a?, b?);
    let first = a.frames.first().ok_or("empty native recording")?.time_us;
    let times: Vec<_> = b.frames.iter().map(|r| r.time_us).collect();
    if times.first() != Some(&first) || !times.contains(&(first + 5000)) {
        return Err("incomplete contact window".into());
    }
    let left = sample(&a, &times, first)?;
    let right = sample(&b, &times, first)?;
    let rows = [left, right];
    let g = &a.geometry;
    if g.foot_half_extents != b.geometry.foot_half_extents
        || g.pad_half_extents != b.geometry.pad_half_extents
        || g.upper_half_extents != b.geometry.upper_half_extents
    {
        return Err("different lane geometry".into());
    }
    let full = rows
        .iter()
        .flatten()
        .flat_map(|r| r.corners)
        .fold(0f64, f64::max);
    let window_full = rows
        .iter()
        .flatten()
        .map(|r| r.window_ns)
        .fold(0f64, f64::max);
    if full <= 0. || window_full <= 0. {
        return Err("missing touchdown".into());
    }
    let spacing = 2.5 * g.pad_half_extents[0];
    let min_z = rows
        .iter()
        .flatten()
        .map(|r| r.frame.pad.position[2] - g.pad_half_extents[2])
        .fold(f64::INFINITY, f64::min);
    let sole_z = min_z - 0.52 * g.pad_half_extents[0];
    let [hx, hy, _] = g.foot_half_extents;
    let summary_z = sole_z - 1.65 * hy;
    let mut archive = Vec::with_capacity(times.len());
    let mut marks = BTreeMap::new();
    let mut first_load = [false; 2];
    for (i, &time) in times.iter().enumerate() {
        let mut f = Frame::default();
        for (k, (_, label)) in lanes.iter().enumerate() {
            let r = &rows[k][i];
            let p = r.frame;
            let shift = if k == 0 {
                -0.5 * spacing
            } else {
                0.5 * spacing
            };
            LaneDisplay {
                label,
                color: LANE[k],
                offset: shift,
                sole_z,
                full,
            }
            .add(&mut f, g, [&p.foot, &p.upper, &p.pad], r.corners)?;
            let lo = shift - hx;
            let hi = shift + hx;
            f.edge(
                &format!("{label}_five_ms_scale"),
                DVec3::new(0., lo, summary_z),
                DVec3::new(0., hi, summary_z),
                hi - lo,
                0.005,
                IDLE,
            );
            f.add(
                &format!("{label}_five_ms_impulse"),
                DVec3::new(0.023, lo + (hi - lo) * r.window_ns / window_full, summary_z),
                DQuat::IDENTITY,
                cuboid([0.010; 3]),
                LANE[k],
            );
            if !first_load[k] && r.corners.iter().any(|j| *j > 0.) {
                first_load[k] = true;
                marks.insert(format!("{label}_touchdown"), time - first);
            }
        }
        archive.push(f.finish(Duration::from_micros(time - first), i + 1));
    }
    marks.insert("first_outer".into(), 500);
    marks.insert("five_ms".into(), 5000);
    fs::create_dir_all(out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), archive)?;
    fs::write(
        out.join("frames.json"),
        serde_json::to_vec_pretty(&json!({
            "kind":"touchdown-window-v1","dt_ns":1000,"ticks":times.iter().map(|t|t-first).collect::<Vec<_>>(),
            "origin_us":first,"capture_window_us":5000,"captures":marks,"spacing":spacing,"sole_z":sole_z,"full_corner_ns":full,"window_full_ns":window_full,
            "lanes":lanes.map(|(file,label)|json!({"file":format!("{file}.json"),"label":label}))
        }))?,
    )?;
    Ok(())
}
