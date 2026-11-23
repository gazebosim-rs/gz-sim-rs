// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Display conditional material trajectories and normal loads in the original fixture.
#[path = "corner_preload.rs"]
pub(super) mod corner;
#[path = "free_landing.rs"]
pub(super) mod free_landing;
use super::*;
use plotters::prelude::{
    ChartBuilder, Color, IntoDrawingArea, LineSeries, PathElement, RGBColor, SVGBackend, WHITE,
};

const BINS: usize = 16;
const COLORS: [[f64; 4]; 3] = [
    [0.04, 0.69, 0.62, 1.],
    [0.30, 0.52, 0.87, 1.],
    [0.88, 0.42, 0.21, 1.],
];

#[derive(Deserialize)]
struct Study {
    case: u32,
    stroke: u32,
    start_s: f64,
    profiles: Vec<Profile>,
    earlier_start_control: Option<Profile>,
}
#[derive(Deserialize)]
struct Profile {
    name: String,
    frames: Vec<Sample>,
}
#[derive(Deserialize)]
struct Sample {
    original_time_s: f64,
    #[serde(deserialize_with = "position")]
    foot_m: DVec3,
    #[serde(deserialize_with = "position")]
    com_m: DVec3,
    #[serde(deserialize_with = "orientation")]
    orientation_xyzw: DQuat,
    pressure: Vec<Cell>,
    normal_force_n: f64,
}
#[derive(Deserialize)]
struct Cell {
    fraction: f64,
    weight_m: f64,
    line_load_n_m: f64,
    foot_compression_m: f64,
    deck_compression_m: f64,
}
fn position<'de, D: serde::Deserializer<'de>>(d: D) -> std::result::Result<DVec3, D::Error> {
    <[f64; 3]>::deserialize(d).map(DVec3::from)
}
fn orientation<'de, D: serde::Deserializer<'de>>(d: D) -> std::result::Result<DQuat, D::Error> {
    <[f64; 4]>::deserialize(d).map(DQuat::from_array)
}
/// The horizontal bins show loads integrated over equal portions of the clipped edge.
fn loads(s: &Sample) -> Result<[f64; BINS]> {
    if !s.normal_force_n.is_finite() || s.normal_force_n < 0. {
        return Err("missing normal reaction".into());
    }
    let mut values = [0.; BINS];
    for cell in &s.pressure {
        if [cell.foot_compression_m, cell.deck_compression_m]
            .into_iter()
            .any(|v| !v.is_finite() || v < 0.)
            || !(0.0..1.).contains(&cell.fraction)
            || !cell.line_load_n_m.is_finite()
            || cell.line_load_n_m < 0.
            || !cell.weight_m.is_finite()
            || cell.weight_m <= 0.
        {
            return Err("invalid pressure cell".into());
        }
        values[(cell.fraction * BINS as f64) as usize] += cell.line_load_n_m * cell.weight_m;
    }
    if (values.iter().sum::<f64>() - s.normal_force_n).abs() > 1e-9 * s.normal_force_n.max(1.) {
        return Err("displayed pressure cells do not integrate to the measured reaction".into());
    }
    Ok(values)
}

fn plot(path: &Path, study: &Study, peak: usize, origin: f64) -> Result<()> {
    let root = SVGBackend::new(path, (1200, 810)).into_drawing_area();
    root.fill(&WHITE)?;
    let panels = root.split_evenly((2, 2));
    let uniform = &study.profiles[0].frames;
    let time = 1000. * (uniform[peak].original_time_s - origin);
    let captions = [
        (
            format!("Load along the edge at {time:.3} ms"),
            "Normal force per length (N/m)",
        ),
        (
            "Change in the foot assembly's path".into(),
            "Distance from uniform layers (µm)",
        ),
        (
            format!("Foot compression at {time:.3} ms"),
            "Compression (µm)",
        ),
        (
            format!("Deck compression at {time:.3} ms"),
            "Compression (µm)",
        ),
    ];
    for (i, (panel, (title, y_axis))) in panels.iter().zip(captions).enumerate() {
        let series: Vec<Vec<(f64, f64)>> = study
            .profiles
            .iter()
            .map(|p| {
                if i == 1 {
                    p.frames
                        .iter()
                        .zip(uniform)
                        .map(|(s, base)| {
                            (
                                1000. * (s.original_time_s - origin),
                                1e6 * s.com_m.distance(base.com_m),
                            )
                        })
                        .collect()
                } else {
                    p.frames[peak]
                        .pressure
                        .iter()
                        .map(|c| {
                            (
                                100. * c.fraction,
                                match i {
                                    0 => c.line_load_n_m,
                                    2 => 1e6 * c.foot_compression_m,
                                    _ => 1e6 * c.deck_compression_m,
                                },
                            )
                        })
                        .collect()
                }
            })
            .collect();
        let (xmin, xmax, x_axis) = if i == 1 {
            (
                series[0][0].0,
                series[0].last().ok_or("empty trajectory")?.0,
                "Time since restart (ms)",
            )
        } else {
            (0., 100., "Position along the edge (%)")
        };
        let ymax = series.iter().flatten().map(|p| p.1).fold(1e-9, f64::max);
        let mut chart = ChartBuilder::on(panel)
            .margin(20)
            .caption(title, ("sans-serif", 22))
            .x_label_area_size(51)
            .y_label_area_size(78)
            .build_cartesian_2d(xmin..xmax, 0.0..1.1 * ymax)?;
        chart
            .configure_mesh()
            .max_light_lines(0)
            .x_desc(x_axis)
            .y_desc(y_axis)
            .draw()?;
        for (k, p) in series.iter().enumerate() {
            let [r, g, b, _] = COLORS[k];
            let color = RGBColor((r * 255.) as u8, (g * 255.) as u8, (b * 255.) as u8);
            chart
                .draw_series(LineSeries::new(p.iter().copied(), color.stroke_width(3)))?
                .label(&study.profiles[k].name)
                .legend(move |(x, y)| {
                    PathElement::new(vec![(x, y), (x + 18, y)], color.stroke_width(3))
                });
        }
        chart
            .configure_series_labels()
            .background_style(WHITE.mix(0.85))
            .draw()?;
    }
    root.present()?;
    Ok(())
}

struct FixtureView {
    geometry: Geometry,
    origin: f64,
    drive: DVec3,
    deck: Body,
    deck_origin: DVec3,
    upper_local: DVec3,
    rotation: DQuat,
}
impl FixtureView {
    fn new(rigid: &Value) -> Result<Self> {
        let geometry: Geometry = serde_json::from_value(rigid["geometry"].clone())?;
        let origin = rigid["references"][0]["fine"]["elapsed"]
            .as_f64()
            .ok_or("missing restart clock")?;
        let drive = DVec3::from(serde_json::from_value::<[f64; 3]>(rigid["drive"].clone())?);
        let first = &rigid["frames"][0];
        let deck: Body = serde_json::from_value(first["pad"].clone())?;
        let foot: Body = serde_json::from_value(first["foot"].clone())?;
        let upper: Body = serde_json::from_value(first["upper"].clone())?;
        let upper_local = DQuat::from_array(foot.orientation).inverse()
            * (DVec3::from(upper.position) - DVec3::from(foot.position));
        let deck_origin = DVec3::from(deck.position) - drive * origin;
        let rotation = DQuat::from_rotation_z(std::f64::consts::FRAC_PI_2);

        Ok(Self {
            geometry,
            origin,
            drive,
            deck,
            deck_origin,
            upper_local,
            rotation,
        })
    }
    fn add_bodies(
        &self,
        frame: &mut Frame,
        label: &str,
        shift: DVec3,
        color: [f64; 4],
        time_s: f64,
        (position, orientation): (DVec3, DQuat),
    ) -> Result<()> {
        let pose = |v| self.rotation * (v + time_s * self.drive) + shift;
        let q = self.rotation * orientation;
        let upper_half = self
            .geometry
            .upper_half_extents
            .ok_or("missing upper link shape")?;
        for (link, pos, orient, half, col) in [
            ("foot", position, q, self.geometry.foot_half_extents, color),
            (
                "upper",
                position + orientation * self.upper_local,
                q,
                upper_half,
                UPPER,
            ),
            (
                "deck",
                self.deck_origin,
                self.rotation * DQuat::from_array(self.deck.orientation),
                self.geometry.pad_half_extents,
                SOLE,
            ),
        ] {
            frame.add(
                &format!("{} {link}", label),
                pose(pos),
                orient,
                cuboid(half),
                col,
            );
        }
        Ok(())
    }
    fn add_lane(
        &self,
        frame: &mut Frame,
        s: &Sample,
        label: &str,
        shift: DVec3,
        color: [f64; 4],
        peak_bin: f64,
    ) -> Result<()> {
        let display = |x, z| DVec3::new(0., x, z) + shift;
        self.add_bodies(
            frame,
            label,
            shift,
            color,
            s.original_time_s,
            (s.foot_m, s.orientation_xyzw),
        )?;
        const WIDTH: f64 = 0.57;
        const CHART_BOTTOM: f64 = -0.42;
        frame.add(
            &format!("{} edge load, 0 to {:.2} N per bin", label, peak_bin),
            display(0., CHART_BOTTOM),
            DQuat::IDENTITY,
            cuboid([0.011, WIDTH / 2., 0.002]),
            IDLE,
        );
        for (j, n) in loads(s)?.iter().enumerate() {
            let x = WIDTH * ((j as f64 + 0.5) / BINS as f64 - 0.5);
            let z = CHART_BOTTOM + 0.22 * n / peak_bin;
            frame.add(
                &format!("{} edge bin {}", label, j + 1),
                display(x, z),
                DQuat::IDENTITY,
                cuboid([0.009, 0.43 * WIDTH / BINS as f64, 0.006]),
                color,
            );
        }

        Ok(())
    }
}
pub(super) fn export(reference: &Path, fixture: &Path, out: &Path) -> Result<()> {
    let study: Study = read(reference)?;
    let rigid: Value = read(fixture)?;
    if study.case != 80
        || study.stroke != 2
        || rigid["case"] != study.case
        || rigid["stroke"] != study.stroke
        || study.profiles.len() != COLORS.len()
        || study.profiles[0].name != "Uniform layers"
    {
        return Err("expected the matched three-layer fixture 80 study".into());
    }
    let base = &study.profiles[0].frames;
    if base.len() < 2
        || (base[0].original_time_s - study.start_s).abs() > 1e-10
        || base
            .windows(2)
            .any(|p| p[0].original_time_s >= p[1].original_time_s)
    {
        return Err("unordered or incomplete material history".into());
    }
    let mut peak_bin = 0f64;
    let mut max_shift = 0f64;
    for p in &study.profiles {
        if p.frames.len() != base.len() || p.frames[0].com_m.distance(base[0].com_m) > 1e-13 {
            return Err("material profiles do not share an initial state and horizon".into());
        }
        for (s, b) in p.frames.iter().zip(base) {
            if !s.foot_m.is_finite()
                || !s.com_m.is_finite()
                || !s.orientation_xyzw.is_normalized()
                || (s.original_time_s - b.original_time_s).abs() > 1e-11
            {
                return Err("invalid or unaligned material state".into());
            }
            peak_bin = loads(s)?.into_iter().fold(peak_bin, f64::max);
            max_shift = max_shift.max(b.com_m.distance(s.com_m));
        }
    }
    if peak_bin == 0. || max_shift == 0. {
        return Err("empty pressure or motion comparison".into());
    }
    let scene = FixtureView::new(&rigid)?;
    let origin = scene.origin;
    let mut archive = Vec::new();
    let peak = base
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.normal_force_n.total_cmp(&b.normal_force_n))
        .ok_or("missing peak")?
        .0;
    for (i, b) in base.iter().enumerate() {
        let mut frame = Frame::default();
        for (k, p) in study.profiles.iter().enumerate() {
            let s = &p.frames[i];
            let shift = DVec3::Y * ((k as f64 - 1.) * 0.76);
            let display = |x, z| DVec3::new(0., x, z) + shift;
            const WIDTH: f64 = 0.57;
            const TRACK: f64 = -0.51;
            let color = COLORS[k];
            scene.add_lane(&mut frame, s, &p.name, shift, color, peak_bin)?;
            frame.add(
                &format!(
                    "{} distance from uniform, 0 to {:.2} um",
                    p.name,
                    1e6 * max_shift
                ),
                display(0., TRACK),
                DQuat::IDENTITY,
                cuboid([0.007, WIDTH / 2., 0.003]),
                IDLE,
            );
            frame.add(
                &format!("{} path difference", p.name),
                display(WIDTH * (s.com_m.distance(b.com_m) / max_shift - 0.5), TRACK),
                DQuat::IDENTITY,
                cuboid([0.010, 0.012, 0.012]),
                color,
            );
        }
        archive.push(frame.finish(
            Duration::try_from_secs_f64(b.original_time_s - origin)?,
            i + 1,
        ));
    }
    if out.try_exists()? {
        return Err(format!("output already exists at {}", out.display()).into());
    }
    let parent = out.parent().unwrap_or(Path::new("."));
    fs::create_dir_all(parent)?;
    let stage = tempfile::Builder::new()
        .prefix(".compliance-view-")
        .tempdir_in(parent)?;
    gz_sim_cli::export_archive_at(
        &stage.path().join("replay"),
        Default::default(),
        0,
        Duration::try_from_secs_f64(study.start_s - origin)?,
        archive,
    )?;
    let index = json!({"kind":"authored-archive-v1","material_pressure_bins":BINS,"dt_ns":1,"captures":{"peak_uniform_load":peak,"last_measured":base.len()-1}});
    fs::write(
        stage.path().join("frames.json"),
        serde_json::to_vec_pretty(&index)?,
    )?;
    plot(
        &stage.path().join("material-comparison.svg"),
        &study,
        peak,
        origin,
    )?;
    fs::rename(stage.path(), out)?;
    Ok(())
}
