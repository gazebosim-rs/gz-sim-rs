// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independently audit a saved fold-breakaway study. Set GZ_FOLD_BREAKAWAY to the study
//! directory and run this test with --ignored. GZ_FOLD_BREAKAWAY_AUDIT optionally saves JSON.
use glam::{DQuat, DVec3};
use serde_json::{Value, json};
use std::{collections::BTreeMap, env, error::Error, fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn Error>>;
fn read(path: &Path) -> Result<Value> {
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}
fn scalar(v: &Value) -> f64 {
    v.as_f64().expect("numeric observation")
}
fn uint(v: &Value) -> u64 {
    v.as_u64().expect("unsigned observation")
}
fn near(a: f64, b: f64) {
    assert!(
        (a - b).abs() <= 3e-12 * (1. + a.abs().max(b.abs())),
        "{a} vs {b}"
    );
}

struct Material {
    mass: Vec<f64>,
    sites: Vec<([usize; 3], [f64; 3])>,
}
impl Material {
    fn read(sdf: &str) -> Result<Self> {
        let doc = roxmltree::Document::parse(sdf)?;
        let sheet = doc
            .descendants()
            .find(|n| n.has_tag_name("cloth") && n.attribute("name") == Some("sheet"))
            .ok_or("missing sheet")?;
        let mut mass = Vec::new();
        let mut sites = Vec::new();
        for p in sheet.children() {
            if p.has_tag_name("particle") {
                mass.push(p.attribute("mass").ok_or("missing mass")?.parse()?);
            }
            if p.has_tag_name("pad_sample") {
                let ids: Vec<usize> = p
                    .attribute("particles")
                    .ok_or("missing stencil")?
                    .split_whitespace()
                    .map(str::parse)
                    .collect::<std::result::Result<_, _>>()?;
                let w: Vec<f64> = p
                    .attribute("weights")
                    .ok_or("missing weights")?
                    .split_whitespace()
                    .map(str::parse)
                    .collect::<std::result::Result<_, _>>()?;
                near(w.iter().sum(), 1.);
                sites.push((
                    ids.try_into().map_err(|_| "expected triangle")?,
                    w.try_into().map_err(|_| "expected three weights")?,
                ));
            }
        }
        assert_eq!(sites.len(), 16);
        assert!(mass.iter().all(|m| *m > 0.));
        Ok(Self { mass, sites })
    }

    fn coords(&self, nodes: &[DVec3], jaws: [(DVec3, DQuat); 2]) -> [f64; 3] {
        assert_eq!(self.mass.len(), nodes.len());
        let center = nodes
            .iter()
            .zip(&self.mass)
            .map(|(p, m)| p * m)
            .sum::<DVec3>()
            / self.mass.iter().sum::<f64>();
        let mut result = [0.; 3];
        result[0] = (center - (jaws[0].0 + jaws[1].0) * 0.5).dot(jaws[0].1 * DVec3::Y);
        for (j, patch) in self.sites.chunks_exact(8).enumerate() {
            for (ids, w) in patch {
                let site = (0..3).map(|i| nodes[ids[i]] * w[i]).sum::<DVec3>();
                result[j + 1] += (site - jaws[j].0).dot(jaws[j].1 * DVec3::Y) / 8.;
            }
        }
        result
    }
}
struct Archive {
    trace: Vec<Value>,
    prefix: Vec<Value>,
    baseline: usize,
    summary: Value,
}
fn inspect(root: &Path, name: &str, cols: u64, start: u64) -> Result<Archive> {
    let dir = root.join(name);
    let material = Material::read(&fs::read_to_string(dir.join("world.sdf"))?)?;
    let weight = material.mass.iter().sum::<f64>() * 9.81;
    assert_eq!(material.mass.len(), 4 * cols as usize);
    let trace: Vec<Value> = serde_json::from_slice(&fs::read(dir.join("trace.json"))?)?;
    let times: Vec<u64> = trace.iter().map(|t| uint(&t["time_ns"])).collect();
    assert!(times.windows(2).all(|p| p[0] < p[1]));
    let baseline = times.binary_search(&start).expect("unloaded grip");
    let observed: Vec<Value> =
        serde_json::from_slice(&fs::read(dir.join("particle-frames.json"))?)?;
    let source = fs::read_to_string(dir.join("replay/frames.jsonl"))?;
    let mut prefix = vec![];
    let mut frames = 0;
    let mut matched = 0;
    let mut max_error: f64 = 0.;
    for line in source.lines() {
        let frame: Value = serde_json::from_str(line)?;
        if frame["kind"] != "frame" {
            continue;
        }
        let scene: Value = serde_json::from_str(frame["scene"].as_str().ok_or("missing scene")?)?;
        let ns = uint(&scene["seconds"]) * 1_000_000_000 + uint(&scene["nanoseconds"]);
        if ns <= start {
            prefix.push(json!([frame["scene"], frame["catalog"]]));
        }
        let items: BTreeMap<_, _> = scene["visuals"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| (v["name"].as_str().unwrap(), v))
            .collect();
        let position = |v: &Value| -> Result<DVec3> {
            Ok(DVec3::from_array(serde_json::from_value(
                v["translation"].clone(),
            )?))
        };
        let nodes: Vec<_> = (0..4 * cols)
            .map(|i| position(items[format!("default::source::body::sheet_{i}").as_str()]))
            .collect::<Result<_>>()?;
        let f = &observed[frames];
        assert_eq!(f["time_ns"], ns);
        let xyz: Vec<[f64; 3]> = serde_json::from_value(f["positions"].clone())?;
        assert_eq!(xyz.len(), nodes.len());
        for (a, b) in nodes.iter().zip(xyz) {
            assert!((*a - DVec3::from_array(b)).length() < 3e-12);
        }
        let jaw = |side| -> Result<(DVec3, DQuat)> {
            let j = items[format!("default::arm::{side}::{side}").as_str()];
            Ok((
                position(j)?,
                DQuat::from_array(serde_json::from_value(j["rotation"].clone())?),
            ))
        };
        if let Ok(i) = times.binary_search(&ns) {
            let p = material.coords(&nodes, [jaw("left")?, jaw("right")?]);
            let t = &trace[i];
            near(weight, scalar(&t["mass_kg"]) * 9.81);
            for (a, b) in p.into_iter().zip([
                &t["jaw_frame_y_m"],
                &t["pad_frame_y_m"][0],
                &t["pad_frame_y_m"][1],
            ]) {
                near(a, scalar(b));
                max_error = max_error.max((a - scalar(b)).abs());
                matched += 1;
            }
        }
        frames += 1;
    }
    assert_eq!(frames, observed.len());
    assert!(matched >= 48);
    assert_eq!(
        uint(&observed.last().unwrap()["time_ns"]),
        *times.last().unwrap()
    );
    let summary = json!({
        "native_frames": frames,
        "native_positions": frames * material.mass.len(),
        "reconstructed_coordinates": matched,
        "max_coordinate_error_m": max_error,
        "trace_ticks": trace.len(),
        "matched_grip_frames": prefix.len(),
    });
    Ok(Archive {
        trace,
        prefix,
        baseline,
        summary,
    })
}

fn outcome(spec: &Value, case: &Value, data: &Archive, sham: &Archive) -> Value {
    let n = data.baseline;
    assert_eq!(n, sham.baseline);
    assert_eq!(data.trace[..=n], sham.trace[..=n]);
    assert_eq!(data.prefix, sham.prefix);
    assert_eq!(data.trace.len(), sham.trace.len());
    let ramp = uint(&spec["ramp_ticks"]);
    let hold = uint(&spec["hold_ticks"]);
    let after = uint(&spec["recovery_ticks"]);
    let window = uint(&spec["window_ticks"]) as usize;
    let count = data.trace.len();
    let load = scalar(&case["load_mn"]) / 1000.;
    let dt = uint(&spec["dt_ns"]);
    assert_eq!(count - n - 1, (2 * ramp + hold + after) as usize);
    let mut min = [f64::INFINITY; 2];
    let mut sum = [0.; 2];
    let mut support = 0;
    let mut bench = 0;
    for (index, (p, s)) in data.trace.iter().zip(&sham.trace).enumerate().skip(n + 1) {
        let tick = (index - n) as u64;
        let amplitude =
            tick.min(ramp).min((2 * ramp + hold).saturating_sub(tick)) as f64 / ramp as f64;
        near(scalar(&p["force_y_n"]), load * amplitude);
        near(scalar(&s["force_y_n"]), 0.);
        assert_eq!(p["time_ns"], s["time_ns"]);
        assert_eq!(
            uint(&p["time_ns"]),
            uint(&data.trace[n]["time_ns"]) + dt * tick
        );
        if index >= count - window {
            let jaw = [scalar(&p["jaw_up_n"][0]), scalar(&p["jaw_up_n"][1])];
            let bench_loaded = scalar(&p["bench_up_n"]).abs() >= 1e-6;
            bench += usize::from(bench_loaded);
            let w = scalar(&p["mass_kg"]) * 9.81;
            support += usize::from(
                !bench_loaded
                    && scalar(&p["min_bench_clearance_m"]) > 0.01
                    && jaw.iter().all(|f| *f > 0.)
                    && ((jaw[0] + jaw[1]) / w - 1.).abs() < 0.025,
            );
            for side in 0..2 {
                let diff = scalar(&p["pad_frame_y_m"][side]) - scalar(&s["pad_frame_y_m"][side]);
                sum[side] += diff;
                min[side] = min[side].min(diff);
            }
        }
    }
    for j in 0..2 {
        near(min[j], scalar(&case["final_min_pad_m"][j]));
        near(sum[j] / window as f64, scalar(&case["residual_pad_m"][j]));
    }
    let slip = min.iter().any(|v| *v > scalar(&spec["slip_m"]));
    assert_eq!(case["lasting_slip"], slip);
    assert_eq!(case["recovered_bench_ticks"], bench);
    assert_eq!(case["recovered_support_ticks"], support);
    json!({
        "load_mn": case["load_mn"],
        "archive": case["archive"],
        "final_min_pad_m": min,
        "qualified_slip": slip,
        "supported_recovery_ticks": support,
        "bench_recovery_ticks": bench,
        "observations": data.summary,
    })
}

#[test]
#[ignore = "requires saved fold-breakaway replay archives"]
fn native_geometry_and_signed_matched_recovery_agree_with_the_study() -> Result<()> {
    let input = env::var_os("GZ_FOLD_BREAKAWAY").ok_or("set GZ_FOLD_BREAKAWAY")?;
    let root = Path::new(&input);
    let spec = read(&root.join("comparison.json"))?;
    let mut evidence = vec![];
    for trial in spec["trials"].as_array().ok_or("missing trials")? {
        let cols = uint(&trial["columns"]);
        let start = uint(&trial["baseline_time_ns"]);
        let mut previous = None;
        for clock in ["primary", "fine"] {
            let cases = trial[clock]["cases"].as_array().ok_or("missing cases")?;
            let reference = cases
                .iter()
                .find(|c| c["load_mn"] == 0)
                .ok_or("missing matched unloaded case")?;
            let sham = inspect(root, reference["archive"].as_str().unwrap(), cols, start)?;
            if let Some((prefix, trace)) = previous.take() {
                assert_eq!(sham.prefix, prefix);
                assert_eq!(sham.trace[..=sham.baseline], trace);
            }
            for case in cases {
                let fresh;
                let loaded = if case["load_mn"] == 0 {
                    &sham
                } else {
                    fresh = inspect(root, case["archive"].as_str().unwrap(), cols, start)?;
                    &fresh
                };
                evidence.push(outcome(&spec, case, loaded, &sham));
            }
            previous = Some((sham.prefix, sham.trace[..=sham.baseline].to_vec()));
        }
    }
    if let Some(output) = env::var_os("GZ_FOLD_BREAKAWAY_AUDIT") {
        fs::write(output, serde_json::to_vec_pretty(&evidence)?)?;
    }
    Ok(())
}

#[test]
fn recovered_patch_travel_follows_rotated_jaws_and_authored_mass() {
    let sites = (0..16)
        .map(|i| {
            if i < 8 {
                ([0, 1, 2], [0.5, 0.5, 0.])
            } else {
                ([0, 1, 2], [0., 0., 1.])
            }
        })
        .collect();
    let material = Material {
        mass: vec![1., 1., 2.],
        sites,
    };
    let nodes = [DVec3::ZERO, DVec3::Y * 2., DVec3::Y * 5.];
    let turn = DQuat::from_rotation_z(0.7);
    let t = DVec3::new(9., 2., 1.);
    let moved = nodes.map(|p| turn * p + t);
    let jaws = [(t, turn), (turn * DVec3::Y + t, turn)];
    let a = material.coords(&moved, jaws);
    for (v, w) in a.into_iter().zip([2.5, 1., 4.]) {
        near(v, w);
    }
}

#[path = "private/fold_bench_replay.rs"]
mod bench;
#[path = "private/fold_recovery_replay.rs"]
mod recovery;
