// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent finite-triangle checks on the ordinary arm-sheet replay and material observations.
//! To audit a saved run set GZ_SHEET_REPLAY to its absolute directory (the parent of replay/)
//! and run this test target with --ignored --nocapture. GZ_SHEET_GEOMETRY_REPORT can name a JSON output.
use parry3d_f64::{
    math::{Pose, Rotation, Vector},
    query::{PointQuery, distance, intersection_test},
    shape::{Cuboid, Triangle},
};
use serde_json::{Value, json};
use std::{env, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

struct Face {
    nodes: [usize; 3],
    panel: Option<usize>,
}
struct Grid {
    rows: usize,
    columns: usize,
    radius: f64,
    last_left: usize,
    first_right: usize,
}

impl Grid {
    fn read(index: &Value) -> Result<Self> {
        let g = &index["cloth_grid"];
        let grid = Self {
            rows: serde_json::from_value(g["rows"].clone())?,
            columns: serde_json::from_value(g["columns"].clone())?,
            radius: g["skin_radius_m"]
                .as_f64()
                .ok_or("missing material radius")?,
            last_left: serde_json::from_value(g["last_row_left"].clone())?,
            first_right: serde_json::from_value(g["first_row_right"].clone())?,
        };
        let particle_count = index["particle_visuals"]
            .as_array()
            .ok_or("missing particle bindings")?
            .len();
        if grid.columns < 2
            || grid.last_left == 0
            || grid.first_right <= grid.last_left
            || grid.first_right >= grid.rows.saturating_sub(1)
            || grid.radius <= 0.
            || grid.rows.checked_mul(grid.columns) != Some(particle_count)
        {
            return Err("invalid cloth mesh or panel extents".into());
        }
        Ok(grid)
    }

    fn faces(&self) -> Vec<Face> {
        let mut faces = vec![];
        for row in 1..self.rows {
            for col in 1..self.columns {
                let br = row * self.columns + col;
                let tl = br - self.columns - 1;
                // The rows spanning the crease participate in all jaw queries.
                let panel = if row <= self.last_left {
                    Some(0)
                } else if row > self.first_right {
                    Some(1)
                } else {
                    None
                };
                for nodes in [[tl, br - self.columns, br], [tl, br, br - 1]] {
                    faces.push(Face { nodes, panel });
                }
            }
        }
        faces
    }
}

#[derive(Default)]
struct Closest {
    sample: Option<(f64, u64, [usize; 2])>,
}
impl Closest {
    fn observe(&mut self, d: f64, time: u64, pair: [usize; 2]) {
        if self.sample.is_none_or(|(best, _, _)| d < best) {
            self.sample = Some((d, time, pair));
        }
    }
    fn json(&self, skin: f64) -> Value {
        self.sample.map_or(Value::Null, |(distance, time, pair)| {
            json!({
                "core_distance_m": distance,
                "skin_clearance_m": distance - skin,
                "time_ns": time,
                "pair_indices": pair,
            })
        })
    }
}

fn jaw(scene: &Value, name: &str) -> Result<(Pose, Cuboid)> {
    let mut matches = scene["visuals"]
        .as_array()
        .ok_or("missing visuals")?
        .iter()
        .filter(|v| v["name"] == name);
    let v = matches.next().ok_or("missing jaw")?;
    if matches.next().is_some() || v["shape"]["kind"] != "box" {
        return Err("expected one centered jaw box".into());
    }
    let xyz: [f64; 3] = serde_json::from_value(v["translation"].clone())?;
    let q = Rotation::from_array(serde_json::from_value(v["rotation"].clone())?);
    let half = Vector::from_array(serde_json::from_value(v["shape"]["size"].clone())?) * 0.5;
    if (q.length_squared() - 1.).abs() > 1e-9 || half.min_element() <= 0. {
        return Err("invalid jaw pose or extents".into());
    }
    Ok((Pose::from_parts(xyz.into(), q), Cuboid::new(half)))
}

#[derive(Default)]
struct Geometry {
    frames: usize,
    pad_pairs: usize,
    panel_pairs: usize,
    pad_intersections: usize,
    panel_intersections: usize,
    pad_min: [Closest; 2],
    panel_min: Closest,
    extra_indentation_m: f64,
}
impl Geometry {
    fn observe(
        &mut self,
        faces: &[Face],
        positions: &[[f64; 3]],
        pads: &[(Pose, Cuboid); 2],
        t: u64,
        skin: f64,
    ) -> Result<()> {
        let tris: Vec<_> = faces
            .iter()
            .map(|f| {
                let [a, b, c] = f.nodes.map(|i| Vector::from_array(positions[i]));
                Triangle::new(a, b, c)
            })
            .collect();
        self.frames += 1;
        for (f, tri) in tris.iter().enumerate() {
            for (j, (pose, pad)) in pads.iter().enumerate() {
                let d = distance(pose, pad, &Pose::IDENTITY, tri)?.distance;
                self.pad_pairs += 1;
                self.pad_intersections +=
                    usize::from(intersection_test(pose, pad, &Pose::IDENTITY, tri)?.intersecting);
                self.pad_min[j].observe(d, t, [j, f]);
                let vertex_d = [tri.a, tri.b, tri.c]
                    .map(|p| pad.distance_to_point(pose, p, true))
                    .into_iter()
                    .fold(f64::INFINITY, f64::min);
                self.extra_indentation_m = self
                    .extra_indentation_m
                    .max((skin - d).max(0.) - (skin - vertex_d).max(0.));
            }
            if faces[f].panel != Some(0) {
                continue;
            }
            for (g, other) in tris
                .iter()
                .enumerate()
                .filter(|(g, _)| faces[*g].panel == Some(1))
            {
                let d = distance(&Pose::IDENTITY, tri, &Pose::IDENTITY, other)?.distance;
                self.panel_pairs += 1;
                self.panel_intersections += usize::from(
                    intersection_test(&Pose::IDENTITY, tri, &Pose::IDENTITY, other)?.intersecting,
                );
                self.panel_min.observe(d, t, [f, g]);
            }
        }
        Ok(())
    }

    fn json(&self, skin: f64) -> Value {
        json!({
            "frames": self.frames,
            "triangle_box_pairs": self.pad_pairs,
            "cross_panel_triangle_pairs": self.panel_pairs,
            "triangle_box_core_intersections": self.pad_intersections,
            "panel_midsurface_intersections": self.panel_intersections,
            "closest_jaws": self.pad_min.each_ref().map(|m| m.json(skin)),
            "closest_panels": self.panel_min.json(2. * skin),
            "max_triangle_indentation_beyond_nearest_vertex_m": self.extra_indentation_m,
        })
    }
}

fn audit(dir: &Path) -> Result<Value> {
    let index: Value = serde_json::from_slice(&fs::read(dir.join("frames.json"))?)?;
    let grid = Grid::read(&index)?;
    let faces = grid.faces();
    let obs: Vec<Value> = serde_json::from_slice(&fs::read(dir.join("particle-frames.json"))?)?;
    let stream = fs::read_to_string(dir.join("replay/frames.jsonl"))?;
    let mut all = Geometry::default();
    let mut hold = Geometry::default();
    for line in stream.lines() {
        let frame: Value = serde_json::from_str(line)?;
        if frame["kind"] != "frame" {
            continue;
        }
        let scene: Value = serde_json::from_str(frame["scene"].as_str().ok_or("missing scene")?)?;
        let sample = obs.get(all.frames).ok_or("missing material observation")?;
        let t = sample["time_ns"].as_u64().ok_or("missing material clock")?;
        if frame["ordinal"].as_u64() != Some(all.frames as u64 + 1)
            || scene["seconds"].as_u64() != Some(t / 1_000_000_000)
            || scene["nanoseconds"].as_u64() != Some(t % 1_000_000_000)
        {
            return Err("mismatched replay and material clocks or ordinals".into());
        }
        let nodes: Vec<[f64; 3]> = serde_json::from_value(sample["positions"].clone())?;
        if nodes.len() != grid.rows * grid.columns {
            return Err("incomplete material frame".into());
        }
        let pads = [
            jaw(&scene, "default::arm::left::left")?,
            jaw(&scene, "default::arm::right::right")?,
        ];
        all.observe(&faces, &nodes, &pads, t, grid.radius)?;
        if (1_651_000_000..=1_950_000_000).contains(&t) {
            hold.observe(&faces, &nodes, &pads, t, grid.radius)?;
        }
    }
    if all.frames != obs.len() || hold.frames == 0 {
        return Err("incomplete folded-sheet capture".into());
    }
    let panel_triangles = [0, 1].map(|i| faces.iter().filter(|f| f.panel == Some(i)).count());
    Ok(json!({
        "cloth_grid": index["cloth_grid"],
        "triangles": faces.len(),
        "panel_triangles": panel_triangles,
        "all": all.json(grid.radius),
        "hold": hold.json(grid.radius)
    }))
}

#[test]
fn triangle_interior_hits_jaw_with_all_nodes_clear_and_parallel_faces_keep_their_gap() -> Result<()>
{
    let grid = Grid::read(
        &json!({"cloth_grid":{"rows":4,"columns":2,"last_row_left":1,"first_row_right":2,"skin_radius_m":0.01},"particle_visuals":[0,1,2,3,4,5,6,7]}),
    )?;
    let points = [
        [-1., -1., 0.],
        [1., -1., 0.],
        [-1., 1., 0.],
        [1., 1., 0.],
        [-1., 1., 0.03],
        [1., 1., 0.03],
        [-1., -1., 0.03],
        [1., -1., 0.03],
    ];
    let pad = Cuboid::new(Vector::splat(0.005));
    let pose = Pose::from_parts(Vector::new(0.4, 0., 0.), Rotation::from_rotation_z(0.32));
    let other = Pose::from_parts(Vector::new(0., 0., 1.), Rotation::IDENTITY);
    let mut g = Geometry::default();
    g.observe(
        &grid.faces(),
        &points,
        &[(pose, pad), (other, pad)],
        0,
        grid.radius,
    )?;
    assert_eq!(g.pad_intersections, 1);
    assert_eq!(g.panel_intersections, 0);
    assert!((g.extra_indentation_m - 0.01).abs() < 1e-12);
    assert!((g.panel_min.json(0.02)["skin_clearance_m"].as_f64().unwrap() - 0.01).abs() < 1e-10);
    Ok(())
}

#[test]
#[ignore = "audits recorded arm-sheet particles and jaws; set GZ_SHEET_REPLAY to a saved trial"]
fn measured_sheet_faces_clear_the_jaw_cores_and_opposite_panel() -> Result<()> {
    let directory = env::var_os("GZ_SHEET_REPLAY").ok_or("GZ_SHEET_REPLAY is required")?;
    let report = audit(Path::new(&directory))?;
    let text = serde_json::to_string_pretty(&report)?;
    if let Some(path) = env::var_os("GZ_SHEET_GEOMETRY_REPORT") {
        fs::write(path, format!("{text}\n"))?;
    }
    println!("{text}");
    assert_eq!(report["all"]["triangle_box_core_intersections"], 0);
    assert_eq!(report["all"]["panel_midsurface_intersections"], 0);
    Ok(())
}
