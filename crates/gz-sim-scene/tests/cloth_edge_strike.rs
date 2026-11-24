// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_sim_scene::{PhysicsEngine, deformable::bodies, load_world_with_sdf_step};
use sdformat::NoResources;
use serde_json::{Value, json};
use std::{collections::BTreeMap, env, error::Error, fs, path::Path};
const SDF: &str = include_str!("../../../examples/worlds/local_cloth_edge_strike.sdf");
type Result<T> = std::result::Result<T, Box<dyn Error>>;

#[test]
fn ordinary_box_edge_contacts_balance_and_replay_geometry() -> Result<()> {
    let root = env::var_os("GZ_CLOTH_EDGE_AUDIT").map(std::path::PathBuf::from);
    let records: Option<Vec<Value>> = root
        .as_ref()
        .map(|p| -> Result<_> {
            Ok(serde_json::from_slice(&fs::read(
                p.join("qualified/trace.json"),
            )?)?)
        })
        .transpose()?;
    let mut sim = load_world_with_sdf_step(
        SDF,
        "edge.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let mut previous = [[DVec3::ZERO; 2]; 2];
    let mut seed = [[DVec3::ZERO; 4]; 2];
    let mut first = None;
    let mut force_ticks = 0;
    let mut first_j = DVec3::ZERO;
    let mut particle_clearance = 0.;
    let mut corner_clearance = 0.;
    let mut residual = [0_f64; 2];
    let mut miss_gap = f64::INFINITY;
    let mut theory = [0.; 2];
    let mut positions = Vec::new();
    let pose = |xyz: DVec3| json!({"translation":xyz.to_array(),"rotation":[0.,0.,0.,1.]});
    for tick in 0..=90 {
        if tick > 0 {
            sim.step_exact(1)?;
        }
        let mut poses = BTreeMap::new();
        for (name, mat) in bodies(&sim) {
            let i = usize::from(name == "miss");
            let base = DVec3::X * if i == 0 { -0.24 } else { 0.24 };
            let p: &[_; 4] = mat.particles().try_into()?;
            let xyz = p.map(|p| DVec3::from(p.position));
            let centroid = (xyz[0] + xyz[1]) * 0.5;
            let ridge0 = base + DVec3::new(0.025, 0., 0.02);
            let impulse = mat.last_boundary_impulses();
            let totals = p.iter().fold([DVec3::ZERO; 2], |[v, l], p| {
                let mv = DVec3::from(p.velocity) / p.inverse_mass;
                [v + mv, l + (DVec3::from(p.position) - base).cross(mv)]
            });
            if let Some(trace) = &records {
                let record = &trace[tick]["cases"][name];
                assert_eq!(trace[tick]["time_ns"], tick * 1_000_000);
                let archived: Vec<[f64; 3]> =
                    serde_json::from_value(record["positions_m"].clone())?;
                assert!(
                    archived
                        .iter()
                        .zip(xyz)
                        .all(|(v, x)| DVec3::from(*v).distance(x) < 1e-12)
                );
            }
            if tick == 0 {
                seed[i] = xyz;
                if i == 1 {
                    // The leading segment stays perpendicular to the box ridge. Once its
                    // midpoint passes the end cap, minimize its true squared distance.
                    let n = DVec3::new(0.6, 0., 0.8);
                    let d = (centroid - ridge0).dot(n);
                    let vy = p[0].velocity[1];
                    let vn = -DVec3::from(p[0].velocity).dot(n);
                    let t = (vn * d + vy * (0.16 - centroid.y)) / (vy * vy + vn * vn);
                    assert!(centroid.y + vy * t > 0.16);
                    theory = [t, (d - vn * t).hypot(centroid.y + vy * t - 0.16) - 0.006];
                }
            } else {
                assert_eq!(impulse[1 - i].linear, [0.; 3]);
                let j = DVec3::from(impulse[i].linear);
                let l = DVec3::from(impulse[i].angular);
                for (k, rate) in [j, l].into_iter().enumerate() {
                    residual[k] = residual[k].max((totals[k] - previous[i][k] + rate).length());
                }
                if i == 1 {
                    assert_eq!(j, DVec3::ZERO);
                    for (node, initial) in xyz.iter().zip(seed[i]) {
                        assert!(
                            node.distance(
                                initial + tick as f64 * 0.001 * DVec3::new(-0.21, 2., -0.28)
                            ) < 1e-12
                        );
                    }
                } else if j.length() > 1e-10 {
                    if first.is_none() {
                        first = Some(tick);
                        first_j = j;
                        let half = DVec3::new(0.025, 0.16, 0.02);
                        particle_clearance = xyz
                            .iter()
                            .map(|&x| ((x - base).abs() - half).max(DVec3::ZERO).length() - 0.006)
                            .fold(f64::INFINITY, f64::min);
                        let max_y = xyz.iter().map(|x| x.y.abs()).fold(0_f64, f64::max);
                        corner_clearance = half.y - max_y - 0.006;
                        assert!(particle_clearance > 0.08 && corner_clearance > 0.07);
                    }
                    force_ticks += 1;
                }
            }
            previous[i] = totals;
            if i == 1 {
                let offset = centroid - ridge0;
                miss_gap = miss_gap.min(
                    offset
                        .dot(DVec3::new(0.6, 0., 0.8))
                        .hypot((offset.y.abs() - 0.16).max(0.))
                        - 0.006,
                );
            }
            let prefix = format!("default::{name}::ridge::");
            poses.insert(format!("{prefix}solid"), pose(base));
            for (id, p) in xyz.iter().enumerate() {
                poses.insert(format!("{prefix}p{id}"), pose(*p));
            }
            for (edge, (a, b)) in [(0, 1), (1, 3), (3, 2), (2, 0), (0, 3)]
                .into_iter()
                .enumerate()
            {
                for k in 1..8 {
                    poses.insert(
                        format!("{prefix}edge{edge}_{k}"),
                        pose((xyz[a] * f64::from(8 - k) + xyz[b] * f64::from(k)) / 8.),
                    );
                }
            }
        }
        positions.push(json!({"time_ns":tick*1_000_000,"poses":poses}));
    }
    assert_eq!(first, Some(40));
    assert!((theory[1] - 50e-6).abs() < 1e-12 && miss_gap >= theory[1] - 1e-12);
    let n = DVec3::new(0.6, 0., 0.8);
    let normal = -first_j.dot(n);
    let friction = (first_j + normal * n).length();
    assert!((normal - 0.0175).abs() < 1e-9 && (friction / normal - 0.25).abs() < 1e-8);
    assert!(residual.iter().all(|&e| e < 1e-10));
    let stats = json!({"first_hit_ms":first,"loaded_ticks":force_ticks,"minimum_particle_clearance_at_hit_m":particle_clearance,"lower_bound_corner_clearance_at_hit_m":corner_clearance,"impulse_on_box_ns":first_j.to_array(),"normal_ns":normal,"tangent_ns":friction,
        "analytic_minimum_time_ms":theory[0]*1000.,"analytic_clearance_m":theory[1],"minimum_sampled_clearance_m":miss_gap,
        "max_linear_balance_ns":residual[0],"max_angular_balance_nms":residual[1]});
    println!("{stats}");
    if let Some(root) = root {
        let view = Path::new(&root).join("viewer");
        fs::create_dir_all(&view)?;
        fs::write(
            view.join("comparison.json"),
            serde_json::to_vec_pretty(&stats)?,
        )?;
        fs::write(view.join("expected.json"), serde_json::to_vec(&positions)?)?;
        let names: BTreeMap<_, _> = positions[0]["poses"]
            .as_object()
            .unwrap()
            .keys()
            .map(|n| (n, n))
            .collect();
        let index = json!({"kind":"authored-archive-v1","camera":"perspective","dt_ns":1,"source_poses":"../../viewer/expected.json",
            "source_visual_poses":names,"captures":{"start":0,"approach":30,"glancing_hit":40,"past_tip":55,"end":90}});
        fs::write(view.join("index.json"), serde_json::to_vec_pretty(&index)?)?;
    }
    Ok(())
}
