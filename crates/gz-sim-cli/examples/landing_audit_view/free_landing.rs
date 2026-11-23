// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Render measured sole contact in a face-on map under each physical fixture.
use super::*;
const FREE_LABELS: [&str; 3] = ["Free approach", "Corner restart", "Line restart"];
const DAMPED_LABELS: [&str; 3] = ["Undamped", "Light damping", "Stronger damping"];

#[derive(Deserialize)]
struct Landing {
    case: u32,
    stroke: u32,
    horizon_s: f64,
    profiles: Vec<History>,
}
#[derive(Deserialize)]
struct History {
    name: String,
    damping_s_m: Option<f64>,
    start_s: f64,
    frames: Vec<Surface>,
}
#[derive(Deserialize)]
struct Surface {
    original_time_s: f64,
    #[serde(deserialize_with = "position")]
    foot_m: DVec3,
    #[serde(deserialize_with = "position")]
    com_m: DVec3,
    #[serde(deserialize_with = "orientation")]
    orientation_xyzw: DQuat,
    normal_force_n: f64,
    normal_dissipation_j: Option<f64>,
    loaded_area_m2: f64,
    center_of_pressure_m: Option<[f64; 3]>,
    loaded_polygon_m: Vec<[f64; 3]>,
}

fn stamp(f: &Surface) -> Result<Duration> {
    Ok(Duration::try_from_secs_f64(f.original_time_s)?)
}
fn at(h: &History, t: f64) -> Option<&Surface> {
    h.frames
        .iter()
        .find(|f| (f.original_time_s - t).abs() < 1e-12)
}
fn polygon(frame: &mut Frame, label: &str, pts: &[DVec3], width: f64, color: [f64; 4]) {
    for (i, (a, b)) in pts.iter().zip(pts.iter().cycle().skip(1)).enumerate() {
        let d = *b - *a;
        let length = d.length();
        if length > 1e-12 {
            frame.add(
                &format!("{label} {}", i + 1),
                (*a + *b) * 0.5,
                DQuat::from_rotation_arc(DVec3::Y, d / length),
                cuboid([width, 0.5 * length, width]),
                color,
            );
        }
    }
}
fn render(
    frame: &mut Frame,
    fixture: &FixtureView,
    s: &Surface,
    name: &str,
    lane: usize,
    peak: f64,
) -> Result<()> {
    let offset = 0.8 * (lane as f64 - 1.);
    let color = COLORS[lane];
    fixture.add_bodies(
        frame,
        name,
        offset * DVec3::Y,
        color,
        s.original_time_s,
        (s.foot_m, s.orientation_xyzw),
    )?;
    // The map shows world XY relative to this frame's foot origin. All three
    // lanes use the same 1.5:1 scale for both the sole outline and loaded patch.
    let map = |p: DVec3, depth| {
        DVec3::new(
            depth,
            offset + 1.5 * (p.x - s.foot_m.x),
            -0.34 + 1.5 * (p.y - s.foot_m.y),
        )
    };
    let [x, y, z] = fixture.geometry.foot_half_extents;
    let corners = [[-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z]]
        .map(|p| map(s.foot_m + s.orientation_xyzw * DVec3::from_array(p), 0.));
    polygon(
        frame,
        &format!("{name} sole outline"),
        &corners,
        0.002,
        IDLE,
    );
    let patch = s
        .loaded_polygon_m
        .iter()
        .map(|p| map(DVec3::from_array(*p), 0.006))
        .collect::<Vec<_>>();
    polygon(frame, &format!("{name} loaded patch"), &patch, 0.004, color);
    if let Some(cop) = s.center_of_pressure_m {
        frame.add(
            &format!("{name} pressure center"),
            map(DVec3::from_array(cop), 0.015),
            DQuat::IDENTITY,
            json!({"kind":"sphere","radius":0.014}),
            color,
        );
    }
    frame.add(
        &format!("{name} measured force"),
        DVec3::new(0., offset + 0.54 * (s.normal_force_n / peak - 0.5), -0.57),
        DQuat::IDENTITY,
        cuboid([0.008, 0.010, 0.012]),
        color,
    );
    Ok(())
}

fn plot(path: &Path, histories: &[History], start: f64, last: f64, damped: bool) -> Result<()> {
    let root = SVGBackend::new(path, (1190, 760)).into_drawing_area();
    root.fill(&WHITE)?;
    let panels = root.split_evenly((2, 2));
    for (k, panel) in panels.iter().enumerate() {
        let (title, y_axis) = if damped {
            match k {
                0 => ("Ground reaction during impact", "Normal force (N)"),
                1 => ("Energy absorbed by normal damping", "Dissipated energy (J)"),
                2 => (
                    "Portion of the sole carrying load",
                    "Loaded sole area (cm²)",
                ),
                _ => (
                    "Vertical movement relative to the moving deck",
                    "Change from starting height (mm)",
                ),
            }
        } else {
            match k {
                0 => ("Ground reaction over the full landing", "Normal force (N)"),
                1 => ("Ground reaction from the corner strike", "Normal force (N)"),
                2 => (
                    "Portion of the sole carrying load",
                    "Loaded sole area (cm²)",
                ),
                _ => (
                    "Motion after restarting the material model",
                    "Distance from free-approach path (mm)",
                ),
            }
        };
        let first = if !damped && (k == 1 || k == 3) {
            histories[1].start_s
        } else {
            start
        };
        let last = if damped && k < 3 {
            // Include one clock after the last nonzero force across all materials.
            // Using the same contact zoom in these panels makes their peaks legible.
            let through = histories
                .iter()
                .flat_map(|h| &h.frames)
                .filter(|f| f.normal_force_n > 0.)
                .map(|f| f.original_time_s)
                .fold(start, f64::max);
            histories[0]
                .frames
                .iter()
                .find(|f| f.original_time_s > through)
                .map_or(last, |f| f.original_time_s)
        } else {
            last
        };
        let mut series = vec![];
        for (i, h) in histories.iter().enumerate() {
            if !damped && k == 3 && i == 0 {
                continue;
            }
            let mut points = vec![];
            for f in h
                .frames
                .iter()
                .filter(|f| f.original_time_s >= first - 1e-12 && f.original_time_s <= last + 1e-12)
            {
                let y = match k {
                    1 if damped => f
                        .normal_dissipation_j
                        .ok_or("missing measured damping loss")?,
                    3 if damped => 1000. * (f.com_m.z - h.frames[0].com_m.z),
                    0 | 1 => f.normal_force_n,
                    2 => f.loaded_area_m2 * 1e4,
                    _ => {
                        1e3 * f.com_m.distance(
                            at(&histories[0], f.original_time_s)
                                .ok_or("unmatched plot time")?
                                .com_m,
                        )
                    }
                };
                points.push(((f.original_time_s - start) * 1000., y));
            }
            if points.is_empty() {
                return Err("empty chart series".into());
            }
            series.push((i, points));
        }
        let max = series
            .iter()
            .flat_map(|(_, v)| v)
            .map(|p| p.1)
            .fold(1e-8, f64::max);
        let min = series
            .iter()
            .flat_map(|(_, v)| v)
            .map(|p| p.1)
            .fold(0., f64::min);
        let mut ax = ChartBuilder::on(panel)
            .margin(16)
            .caption(title, ("sans-serif", 20))
            .x_label_area_size(47)
            .y_label_area_size(68)
            .build_cartesian_2d(
                (first - start) * 1000.0..(last - start) * 1000.,
                min * 1.1..max * 1.15,
            )?;
        ax.configure_mesh()
            .max_light_lines(0)
            .x_desc("Time since free approach (ms)")
            .y_desc(y_axis)
            .draw()?;
        for (i, v) in series {
            let [r, g, b, _] = COLORS[i];
            let color = RGBColor((255. * r) as u8, (255. * g) as u8, (255. * b) as u8);
            let label = if damped {
                format!(
                    "{} ({} s/m)",
                    histories[i].name,
                    histories[i]
                        .damping_s_m
                        .ok_or("missing normal damping coefficient")?
                )
            } else {
                histories[i].name.clone()
            };
            ax.draw_series(LineSeries::new(v.iter().copied(), color.stroke_width(2)))?
                .label(label)
                .legend(move |(x, y)| {
                    PathElement::new([(x, y), (x + 16, y)], color.stroke_width(2))
                });
            ax.draw_series(
                v.iter()
                    .map(|p| plotters::prelude::Circle::new(*p, 2, color.filled())),
            )?;
        }
        ax.configure_series_labels()
            .background_style(WHITE.mix(0.9))
            .draw()?;
    }
    root.present()?;
    Ok(())
}

pub(in super::super) fn export(reference: &Path, fixture_file: &Path, out: &Path) -> Result<()> {
    export_histories(reference, fixture_file, out, false)
}
pub(in super::super) fn export_damped(
    reference: &Path,
    fixture_file: &Path,
    out: &Path,
) -> Result<()> {
    export_histories(reference, fixture_file, out, true)
}
fn export_histories(reference: &Path, fixture_file: &Path, out: &Path, damped: bool) -> Result<()> {
    let study: Landing = read(reference)?;
    let labels = if damped { DAMPED_LABELS } else { FREE_LABELS };
    let fixture: Value = read(fixture_file)?;
    if (study.case, study.stroke) != (80, 2)
        || fixture["case"] != 80
        || fixture["stroke"] != 2
        || study.profiles.len() != labels.len()
    {
        return Err("expected three runs of fixture 80, stroke 2".into());
    }
    let fixture = FixtureView::new(&fixture)?;
    let base = &study.profiles[0];
    let mut peaks = [0.0_f64; 3];
    let mut energy_scale = 0_f64;
    if damped {
        let levels = study
            .profiles
            .iter()
            .map(|h| h.damping_s_m)
            .collect::<Option<Vec<_>>>()
            .ok_or("missing normal damping coefficient")?;
        if levels[0] != 0.
            || !levels.iter().all(|a| a.is_finite())
            || levels.windows(2).any(|p| p[0] >= p[1])
        {
            return Err("expected ascending normal damping coefficients".into());
        }
    }
    for ((h, name), peak) in study.profiles.iter().zip(labels).zip(&mut peaks) {
        if h.name != name
            || h.frames.len() < 2
            || (h.frames[0].original_time_s - h.start_s).abs() > 1e-12
            || (h.frames.last().unwrap().original_time_s - study.horizon_s).abs() > 1e-12
            || h.frames
                .windows(2)
                .any(|ab| ab[0].original_time_s >= ab[1].original_time_s)
        {
            return Err("mismatched material history".into());
        }
        if damped
            && (h.frames.len() != base.frames.len()
                || h.frames[0].foot_m != base.frames[0].foot_m
                || h.frames[0].orientation_xyzw != base.frames[0].orientation_xyzw
                || h.start_s != base.start_s)
        {
            return Err("damping histories require identical initial states and clocks".into());
        }
        let mut previous_energy = 0.;
        for f in &h.frames {
            if damped {
                let e = f.normal_dissipation_j.ok_or("missing dissipated energy")?;
                if !e.is_finite() || e < previous_energy || (name == DAMPED_LABELS[0] && e != 0.) {
                    return Err("invalid normal damping loss".into());
                }
                previous_energy = e;
                energy_scale = energy_scale.max(e);
            }
            if !f.com_m.is_finite()
                || !f.foot_m.is_finite()
                || !f.orientation_xyzw.is_normalized()
                || !f.original_time_s.is_finite()
                || !f.normal_force_n.is_finite()
                || f.normal_force_n < 0.
                || !f.loaded_area_m2.is_finite()
                || f.loaded_area_m2 < 0.
                || (f.normal_force_n > 0.) != f.center_of_pressure_m.is_some()
                || f.loaded_polygon_m
                    .iter()
                    .chain(f.center_of_pressure_m.iter())
                    .flatten()
                    .any(|x| !x.is_finite())
                || at(base, f.original_time_s).is_none()
            {
                return Err("invalid or unmatched area-contact sample".into());
            }
            *peak = peak.max(f.normal_force_n);
        }
    }
    if peaks.iter().any(|p| *p <= 0.) || base.frames[0].normal_force_n != 0. {
        return Err("expected an initially unloaded landing".into());
    }
    if damped {
        if energy_scale <= 0. {
            return Err("missing loss for damped sole".into());
        }
        let max_force = peaks.into_iter().fold(0., f64::max);
        peaks.fill(max_force);
    }
    let mut archive = vec![];
    let mut captures = BTreeMap::new();
    let mut previous_ids = BTreeMap::new();
    let mut next_id = 2_u64;
    for (i, s) in base.frames.iter().enumerate() {
        let mut frame = Frame::default();
        for (j, h) in study.profiles.iter().enumerate() {
            // Give all lanes permanent scale rails so frame-all includes both future restarts.
            frame.add(
                &format!("Force scale for {}, 0 to {:.1} N", h.name, peaks[j]),
                DVec3::new(0., 0.8 * (j as f64 - 1.), -0.57),
                DQuat::IDENTITY,
                cuboid([0.005, 0.27, 0.003]),
                IDLE,
            );
            if let Some(f) = at(h, s.original_time_s) {
                render(&mut frame, &fixture, f, &h.name, j, peaks[j])?;
                if damped {
                    let offset = 0.8 * (j as f64 - 1.);
                    let energy = f.normal_dissipation_j.ok_or("missing damping loss")?;
                    frame.add(
                        &format!(
                            "Normal damping loss scale for {}, 0 to {:.3} J",
                            h.name, energy_scale
                        ),
                        DVec3::new(0., offset, -0.64),
                        DQuat::IDENTITY,
                        cuboid([0.005, 0.27, 0.003]),
                        IDLE,
                    );
                    frame.add(
                        &format!("{} absorbed energy", h.name),
                        DVec3::new(0., offset + 0.54 * (energy / energy_scale - 0.5), -0.64),
                        DQuat::IDENTITY,
                        cuboid([0.008, 0.010, 0.012]),
                        COLORS[j],
                    );
                }
                if i > 0 && (f.original_time_s - h.start_s).abs() < 1e-12 {
                    captures.insert(format!("start_{}", j + 1), i);
                }
            }
        }
        // A changing polygon changes both membership and segment lengths. Recorder
        // identities retain their shape while live and cannot return after retirement.
        let mut active_ids = BTreeMap::new();
        for (visual, model) in frame.visuals.iter_mut().zip(&mut frame.models) {
            let key =
                serde_json::to_string(&(&visual["name"], &visual["shape"], &visual["color"]))?;
            let id = previous_ids.remove(&key).unwrap_or_else(|| {
                let id = next_id;
                next_id += 2;
                id
            });
            visual["id"] = json!(id + 1);
            model["model_id"] = json!(id);
            model["visual_ids"] = json!([id + 1]);
            active_ids.insert(key, id);
        }
        previous_ids = active_ids;
        archive.push(frame.finish(stamp(s)?, i + 1));
    }
    let k = base
        .frames
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.normal_force_n.total_cmp(&b.normal_force_n))
        .unwrap()
        .0;
    captures.insert(
        if damped {
            "peak_sampled_undamped_load"
        } else {
            "peak_sampled_free_load"
        }
        .into(),
        k,
    );
    captures.insert("last_measured".into(), archive.len() - 1);
    if out.exists() {
        return Err("replay output already exists".into());
    }
    let parent = out.parent().unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = tempfile::Builder::new()
        .prefix(".free-landing-")
        .tempdir_in(parent)?;
    gz_sim_cli::export_archive_at(
        &stage.path().join("replay"),
        Default::default(),
        0,
        stamp(&base.frames[0])?,
        archive,
    )?;
    let mut index =
        json!({"kind":"authored-archive-v1","free_landing":true,"dt_ns":1,"captures":captures});
    if damped {
        index["normal_damping"] = json!(true);
    }
    fs::write(
        stage.path().join("frames.json"),
        serde_json::to_vec_pretty(&index)?,
    )?;
    plot(
        &stage.path().join(if damped {
            "damped-landing.svg"
        } else {
            "free-landing.svg"
        }),
        &study.profiles,
        base.start_s,
        study.horizon_s,
        damped,
    )?;
    fs::rename(stage.path(), out)?;
    Ok(())
}
