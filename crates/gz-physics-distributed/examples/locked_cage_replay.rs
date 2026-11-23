// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Export two raw reduced-coordinate runs with a common physical and diagnostic scale.
#[allow(dead_code)]
#[path = "support/primitive_replay_view.rs"]
mod primitive_replay_view;
use primitive_replay_view::View;
use serde::Deserialize;
use serde_json::json;
use std::{
    error::Error,
    fs,
    io::{BufRead, BufReader},
    path::{Path, PathBuf},
};
type Result<T> = std::result::Result<T, Box<dyn Error>>;
#[derive(Deserialize)]
struct Ball {
    p: [f64; 3],
    q: [f64; 4],
}
#[derive(Deserialize)]
struct Row {
    tick: u64,
    balls: Vec<Ball>,
    barycenter: [f64; 3],
    expected_center: [f64; 3],
    momentum_residual: [f64; 3],
}
fn rows(p: &Path) -> Result<Vec<Row>> {
    BufReader::new(fs::File::open(p)?)
        .lines()
        .map(|l| Ok(serde_json::from_str(&l?)?))
        .collect()
}
fn main() -> Result<()> {
    let mut args = std::env::args_os().skip(1).map(PathBuf::from);
    let traces = args.next().ok_or("need raw trace directory")?;
    let out = args.next().ok_or("need new output directory")?;
    let arms = [
        rows(&traces.join("ReducedStopped.jsonl"))?,
        rows(&traces.join("ReducedFixed.jsonl"))?,
    ];
    if arms[0].is_empty() || arms[0].len() != arms[1].len() {
        return Err("unmatched raw controls".into());
    }
    let mut frames = Vec::new();
    let mut gauge = Vec::new();
    for k in 0..arms[0].len() {
        if k % 4 != 0 && k + 1 != arms[0].len() {
            continue;
        }
        let mut v = View::default();
        let mut observations = Vec::new();
        for (i, arm) in arms.iter().enumerate() {
            let r = &arm[k];
            if r.tick as usize != k || r.balls.len() != 9 {
                return Err("unexpected cage layout".into());
            }
            let offset = [-0.52, 0.52][i];
            let label = ["compliant_stop", "exact_fixed"][i];
            let base = 10 + i as u64 * 100;
            for (b, ball) in r.balls.iter().enumerate() {
                let [x, y, z] = ball.p;
                let payload = b + 1 == r.balls.len();
                let color = if payload {
                    [0.95, 0.69, 0.18, 1.]
                } else if i == 0 {
                    [0.92, 0.33, 0.15, 1.]
                } else {
                    [0.09, 0.65, 0.78, 1.]
                };
                v.add(
                    base + 2 * b as u64,
                    &format!(
                        "{label}_{}",
                        if payload {
                            "payload".to_string()
                        } else {
                            format!("link_{b}")
                        }
                    ),
                    [x + offset, y, z - 0.7],
                    ball.q,
                    json!({"kind":"sphere","radius":if payload{0.18}else{0.06}}),
                    color,
                );
            }
            let dp = r
                .momentum_residual
                .iter()
                .map(|x| x * x)
                .sum::<f64>()
                .sqrt();
            let center = (0..3)
                .map(|j| (r.barycenter[j] - r.expected_center[j]).powi(2))
                .sum::<f64>()
                .sqrt();
            for (j, (n, value, range, color)) in [
                ("momentum", dp, 0.08, [0.88, 0.36, 0.59, 1.]),
                ("centroid", center, 0.0015, [0.63, 0.73, 0.28, 1.]),
            ]
            .into_iter()
            .enumerate()
            {
                let y = -0.415 - j as f64 * 0.1;
                v.cuboid(
                    base + 30 + j as u64 * 4,
                    &format!("{label}_{n}_scale"),
                    [offset, y, -0.02],
                    [0.55, 0.012, 0.008],
                    [0.53, 0.57, 0.64, 1.],
                );
                v.add(
                    base + 32 + j as u64 * 4,
                    &format!("{label}_{n}_marker"),
                    [offset - 0.275 + 0.55 * (value / range).min(1.), y, 0.005],
                    [0., 0., 0., 1.],
                    json!({"kind":"sphere","radius":0.026}),
                    color,
                );
            }
            observations.push(json!({"name":label,"delta_p_ns":dp,"centroid_error_m":center}));
        }
        gauge.push(json!({"tick":k,"arms":observations}));
        frames.push(v.finish_at(frames.len() as u64 + 1, k as u64 * 1000000));
    }
    fs::create_dir(&out)?;
    gz_sim_cli::export_archive(&out.join("replay"), Default::default(), frames)?;
    fs::write(
        out.join("index.json"),
        serde_json::to_vec(
            &json!({"frames":gauge,"momentum_scale_ns":0.08,"centroid_scale_m":0.0015,"lane_offsets_m":[-0.52,0.52]}),
        )?,
    )?;
    Ok(())
}
