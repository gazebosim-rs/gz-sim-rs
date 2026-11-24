// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Export the measured garment_grasp example to a gzweb read-only replay archive.
//! `cargo run --release --locked -p gz-sim-cli --example garment_replay -- capture.json outdir`
//! `cargo run --release --locked -p gz-sim-cli --bin gz-sim-rs -- outdir --server --replay --web-viewer 9012`
use bytes::Bytes;
use clap::Parser;
use glam::{DQuat, DVec3};
use gz_sim_cli::{EXPORT_ASSET_PREFIX, export_archive};
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::BTreeMap,
    fs::File,
    io::{self, Read},
    path::{Path, PathBuf},
};

#[path = "support/garment_geometry.rs"]
mod garment_geometry;

#[derive(Parser)]
struct Args {
    capture: PathBuf,
    output: PathBuf,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Tip {
    name: String,
    radius: f64,
    color: String,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Cloth {
    rows: usize,
    columns: usize,
    particle_radius: f64,
    color: String,
    triangles: Vec<[usize; 3]>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Frame {
    time_ns: u64,
    tips: [[f64; 3]; 2],
    particles: Vec<[f64; 3]>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Capture {
    version: u8,
    units: String,
    up: String,
    ground_z: f64,
    tips: [Tip; 2],
    cloth: Cloth,
    frames: Vec<Frame>,
}

fn invalid(message: &str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

fn read_capture(path: &Path) -> io::Result<Capture> {
    // The producer emits 1,401 frames of this 5 × 7 swatch. Keep malformed
    // or unrelated documents out of the offline exporter before allocation.
    const LIMIT: u64 = 16 * 1024 * 1024;
    let mut bytes = Vec::new();
    File::open(path)?.take(LIMIT + 1).read_to_end(&mut bytes)?;
    if bytes.len() as u64 > LIMIT {
        return Err(invalid("garment capture exceeds 16 MiB"));
    }
    let capture: Capture = serde_json::from_slice(&bytes).map_err(io::Error::other)?;
    let c = &capture.cloth;
    if capture.version != 1
        || capture.units != "m"
        || capture.up != "z"
        || !capture.ground_z.is_finite()
        || capture.frames.len() != 1401
        || c.rows != 5
        || c.columns != 7
        || !(c.particle_radius > 0. && c.particle_radius.is_finite())
        || c.triangles.len() != 2 * (c.rows - 1) * (c.columns - 1)
        || c.triangles
            .iter()
            .flatten()
            .any(|&i| i >= c.rows * c.columns)
    {
        return Err(invalid(
            "expected the measured Z-up, metre, 5 × 7 garment_grasp capture",
        ));
    }
    if capture.tips[0].name == capture.tips[1].name
        || capture.tips.iter().any(|tip| {
            tip.name.is_empty()
                || tip.name.len() > 220
                || !tip.radius.is_finite()
                || tip.radius <= 0.
        })
    {
        return Err(invalid("invalid garment pad definitions"));
    }
    for (i, f) in capture.frames.iter().enumerate() {
        if f.time_ns != i as u64 * 1_000_000
            || f.particles.len() != c.rows * c.columns
            || f.particles
                .iter()
                .chain(&f.tips)
                .flatten()
                .any(|v| !(*v as f32).is_finite())
        {
            return Err(invalid(
                "expected a complete measured 1 ms frame sequence starting at time zero",
            ));
        }
    }
    Ok(capture)
}

fn color(hex: &str) -> io::Result<[f32; 3]> {
    let digits = hex
        .strip_prefix('#')
        .filter(|s| s.len() == 6 && s.bytes().all(|b| b.is_ascii_hexdigit()))
        .ok_or_else(|| invalid("expected an sRGB color in #RRGGBB format"))?;
    Ok(std::array::from_fn(|i| {
        let s = f32::from(
            u8::from_str_radix(&digits[2 * i..2 * i + 2], 16).expect("validated hex byte"),
        ) / 255.;
        if s <= 0.04045 {
            s / 12.92
        } else {
            ((s + 0.055) / 1.055).powf(2.4)
        }
    }))
}

fn visual(id: u64, name: &str, translation: [f64; 3], shape: Value, rgb: [f32; 3]) -> Value {
    json!({"id":id,"name":name,"translation":translation,"rotation":[0,0,0,1],"shape":shape,"color":[rgb[0],rgb[1],rgb[2],1]})
}

fn model(id: u64, name: &str, visuals: &[u64]) -> Value {
    json!({"model_id":id,"name":name,"visual_ids":visuals,"can_remove":false,"can_set_state":false})
}

const BASE_WIDTH: f64 = 0.30;
const BASE_DEPTH: f64 = 0.26;
const MARGIN: f64 = 0.004;
const CLOTH_MODEL: u64 = 10;

struct Parking {
    rotation: DQuat,
    rotated_center: DVec3,
    size: DVec3,
}

fn render_vertex(particle: [f64; 3]) -> DVec3 {
    // glTF stores f32 coordinates. Fit the vertices the viewer actually receives.
    DVec3::from_array(particle.map(|v| f64::from(v as f32)))
}

fn bounds(vertices: impl Iterator<Item = DVec3>) -> (DVec3, DVec3) {
    vertices.fold(
        (DVec3::splat(f64::INFINITY), DVec3::splat(f64::NEG_INFINITY)),
        |(low, high), vertex| (low.min(vertex), high.max(vertex)),
    )
}

impl Parking {
    fn fit(particles: &[[f64; 3]]) -> io::Result<Self> {
        let vertices = particles.iter().copied().map(render_vertex);
        let (low, high) = bounds(vertices.clone());
        let size = high - low;
        let quarter_turn = std::f64::consts::FRAC_PI_2;
        let rotation = if size.x < size.y && size.x < size.z {
            DQuat::from_rotation_y(-quarter_turn)
        } else if size.y < size.z {
            DQuat::from_rotation_x(quarter_turn)
        } else {
            DQuat::IDENTITY
        };
        let (low, high) = bounds(vertices.map(|p| rotation * p));
        let size = high - low;
        if !size.is_finite()
            || size.x + 2. * MARGIN > BASE_WIDTH
            || size.y + 2. * MARGIN > BASE_DEPTH
        {
            return Err(invalid(
                "inactive swatch would protrude through the side of the display base",
            ));
        }
        Ok(Self {
            rotation,
            rotated_center: (low + high) * 0.5,
            size,
        })
    }

    fn translation(&self, base_center: DVec3) -> DVec3 {
        base_center - self.rotated_center
    }
}

fn export(args: Args) -> io::Result<()> {
    let capture = read_capture(&args.capture)?;
    let cloth_color = color(&capture.cloth.color)?;
    let tip_colors = [
        color(&capture.tips[0].color)?,
        color(&capture.tips[1].color)?,
    ];
    let samples: Vec<_> = capture.frames.iter().step_by(20).collect();
    let parking = samples
        .iter()
        .map(|sample| Parking::fit(&sample.particles))
        .collect::<io::Result<Vec<_>>>()?;
    let height = parking.iter().map(|p| p.size.z).fold(0.0, f64::max) + 2. * MARGIN;
    let base_center = DVec3::new(0., 0., capture.ground_z - height * 0.5);
    let platform = visual(
        3,
        "default::platform::surface",
        base_center.to_array(),
        json!({"kind":"box","size":[BASE_WIDTH,BASE_DEPTH,height]}),
        [0.38, 0.42, 0.46],
    );

    let mut assets = BTreeMap::new();
    let mut cloth_ids = Vec::new();
    let mut parked_visuals = Vec::new();
    for (i, (sample, parked)) in samples.iter().zip(parking).enumerate() {
        let translation = parked.translation(base_center);
        // A box is convex. Strict containment of all rendered vertices proves
        // every triangle of the parked mesh is behind the opaque box surface.
        let half = DVec3::new(BASE_WIDTH, BASE_DEPTH, height) * 0.5;
        for &particle in &sample.particles {
            let vertex = translation + parked.rotation * render_vertex(particle);
            if !(vertex - base_center).abs().cmplt(half).all() {
                return Err(invalid(
                    "inactive swatch is outside the opaque display base",
                ));
            }
        }
        let route = format!("/assets/{i}/swatch.glb");
        let mesh = garment_geometry::surface_glb(
            "gz-sim-rs garment replay",
            "XPBD swatch",
            &sample.particles,
            &capture.cloth.triangles,
            cloth_color,
        )?;
        let shape =
            json!({"kind":"mesh","uri":format!("{EXPORT_ASSET_PREFIX}{route}"),"scale":[1,1,1]});
        assets.insert(route, Bytes::from(mesh));
        let id = CLOTH_MODEL + 1 + i as u64;
        cloth_ids.push(id);
        let mut v = visual(
            id,
            &format!("default::swatch::sample_{i:02}"),
            translation.to_array(),
            shape,
            [1.; 3],
        );
        v["rotation"] = json!(parked.rotation.to_array());
        parked_visuals.push(v);
    }
    let mut models = vec![model(2, "default::platform", &[3])];
    for (j, tip) in capture.tips.iter().enumerate() {
        let id = 4 + 2 * j as u64;
        let name = tip
            .name
            .rsplit_once("::")
            .map_or(tip.name.as_str(), |(model, _)| model);
        models.push(model(id, name, &[id + 1]));
    }
    models.push(model(CLOTH_MODEL, "default::swatch", &cloth_ids));

    let mut frames = Vec::with_capacity(samples.len());
    for (i, frame) in samples.iter().enumerate() {
        let mut visuals = vec![platform.clone()];
        for (j, tip) in capture.tips.iter().enumerate() {
            visuals.push(visual(
                5 + 2 * j as u64,
                &tip.name,
                frame.tips[j],
                json!({"kind":"sphere","radius":tip.radius}),
                tip_colors[j],
            ));
        }
        // A stable roster lets gzweb resolve each immutable mesh once. Release
        // exactly one measured sample from the base without changing its vertices.
        let mut cloth = parked_visuals.clone();
        cloth[i]["translation"] = json!([0, 0, 0]);
        cloth[i]["rotation"] = json!([0, 0, 0, 1]);
        visuals.extend(cloth);
        let sequence = i as u64 + 1;
        let scene = json!({"version":2,"kind":"scene","world":"default","world_id":1,"epoch":0,"seconds":frame.time_ns/1_000_000_000,"nanoseconds":frame.time_ns%1_000_000_000,"sequence":sequence,"paused":false,"visuals":visuals});
        let catalog = json!({"version":1,"kind":"lifecycle_catalog","world_id":1,"epoch":0,"scene_sequence":sequence,"templates":[],"models":models});
        frames.push((scene.to_string(), catalog.to_string()));
    }
    export_archive(&args.output, assets, frames)?;
    println!(
        "Exported {} measured frames spanning 0–1.4 s to {} (base thickness {:.4} m)",
        samples.len(),
        args.output.display(),
        height
    );
    Ok(())
}

fn main() -> io::Result<()> {
    export(Args::parse())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parking_rotates_the_narrow_axis_inside_the_base() {
        for narrow_axis in 0..3 {
            let mut extent = [0.12, 0.14, 0.11];
            extent[narrow_axis] = 0.002;
            let mut corners = Vec::new();
            for x in [-0.5, 0.5] {
                for y in [-0.5, 0.5] {
                    for z in [-0.5, 0.5] {
                        corners.push([
                            0.03 + x * extent[0],
                            -0.07 + y * extent[1],
                            0.14 + z * extent[2],
                        ]);
                    }
                }
            }
            let pose = Parking::fit(&corners).unwrap();
            assert!((pose.size.z - extent[narrow_axis]).abs() < 1e-7);
            let base_center = DVec3::new(0., 0., -pose.size.z * 0.5 - MARGIN);
            let translation = pose.translation(base_center);
            for corner in corners {
                let p = pose.rotation * DVec3::from(corner) + translation;
                assert!(p.x.abs() < BASE_WIDTH * 0.5 && p.y.abs() < BASE_DEPTH * 0.5);
                assert!(p.z < 0. && p.z > 2. * base_center.z);
            }
        }
        assert!(Parking::fit(&[[0.; 3], [1., 1., 0.]]).is_err());
    }

    #[test]
    fn exported_frames_keep_one_catalog_and_exactly_one_unparked_mesh() {
        let temp = tempfile::tempdir().unwrap();
        let source = temp.path().join("measured.json");
        let output = temp.path().join("playback");
        let mut triangles = Vec::new();
        for r in 0..4 {
            for c in 0..6 {
                let a = r * 7 + c;
                triangles.extend([[a, a + 1, a + 8], [a, a + 8, a + 7]]);
            }
        }
        let frames: Vec<_>=(0..1401).map(|i| {
            let positions:Vec<_>=(0..35).map(|j|[(j%7) as f64*0.01,(j/7) as f64*0.01,0.02+i as f64*1e-5]).collect();
            json!({"time_ns":i*1_000_000,"particles":positions,"tips":[[0.0,0.03,0.1],[0.0,-0.03,0.1]]})
        }).collect();
        let input = json!({"version":1,"units":"m","up":"z","ground_z":0,
            "tips":[{"name":"left","radius":0.03,"color":"#112233"},{"name":"right","radius":0.03,"color":"#445566"}],
            "cloth":{"rows":5,"columns":7,"particle_radius":0.003,"color":"#123456","triangles":triangles},
            "frames":frames});
        serde_json::to_writer(File::create(&source).unwrap(), &input).unwrap();
        export(Args {
            capture: source,
            output: output.clone(),
        })
        .unwrap();
        let stream = std::fs::read_to_string(output.join("frames.jsonl")).unwrap();
        let mut count = 0;
        for line in stream.lines() {
            let record: Value = serde_json::from_str(line).unwrap();
            if record["kind"] != "frame" {
                continue;
            }
            let scene: Value = serde_json::from_str(record["scene"].as_str().unwrap()).unwrap();
            let catalog: Value = serde_json::from_str(record["catalog"].as_str().unwrap()).unwrap();
            let cloth = &scene["visuals"].as_array().unwrap()[3..];
            assert_eq!(cloth.len(), 71);
            assert_eq!(catalog["models"][3]["model_id"], CLOTH_MODEL);
            for (i, mesh) in cloth.iter().enumerate() {
                let id = CLOTH_MODEL + 1 + i as u64;
                assert_eq!(mesh["id"], id);
                assert_eq!(catalog["models"][3]["visual_ids"][i], id);
                assert_eq!(
                    mesh["shape"]["uri"],
                    format!("{EXPORT_ASSET_PREFIX}/assets/{i}/swatch.glb")
                );
                let t: [f64; 3] = serde_json::from_value(mesh["translation"].clone()).unwrap();
                let q: [f64; 4] = serde_json::from_value(mesh["rotation"].clone()).unwrap();
                assert_eq!(t == [0.; 3] && q == [0., 0., 0., 1.], i == count);
                if i != count {
                    let z = 0.02 + (i * 20) as f64 * 1e-5;
                    let center = DQuat::from_array(q) * DVec3::new(0.03, 0.02, z) + DVec3::from(t);
                    assert!(center.z < 0. && center.z > -2. * MARGIN);
                }
            }
            count += 1;
        }
        assert_eq!(count, 71);
    }
}
