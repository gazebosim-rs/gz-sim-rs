// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_sim_scene::{PhysicsEngine, deformable::bodies, load_world_with_sdf_step};
use sdformat::NoResources;
use serde_json::json;
use std::{env, fs, path::Path};
const SDF: &str = include_str!("../../../examples/worlds/local_cloth_face_support.sdf");
#[test]
fn local_sdf_finite_supported_face_agrees_with_wide_control_and_a_free_fall() {
    let mut sim = load_world_with_sdf_step(
        SDF,
        "cloth-face.sdf",
        "default",
        &NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let mut first = [None; 3];
    let mut impulse = [0.; 3];
    let mut weight = [0.; 3];
    let mut first_j = [0.; 3];
    let mut height = [0.; 3];
    let mut min_clearance = f64::INFINITY;
    let mut min_barycentric = f64::INFINITY;
    let mut pose_frames = vec![];
    for tick in 0..=600 {
        if tick > 0 {
            sim.step_exact(1).unwrap();
        }
        let mut poses = serde_json::Map::new();
        let mut forces = [0.; 3];
        let mut z = [0.; 3];
        for (name, body) in bodies(&sim) {
            let (idx, origin, offset, w) = match name {
                "narrow" => (0, -0.8, 0., 0.035),
                "wide" => (1, 0., 0., 0.30),
                "miss" => (2, 0.8, 0.42, 0.035),
                _ => unreachable!(),
            };
            let bench = DVec3::new(origin + 3. / 35. + offset, -3. / 35., -0.02);
            let visual = |xyz: [f64; 3]| json!({"translation":xyz,"rotation":[0.,0.,0.,1.]});
            poses.insert(
                format!("default::{name}::deck::support"),
                visual(bench.to_array()),
            );
            if tick > 0 {
                let rows = body.last_boundary_impulses();
                assert_eq!(rows.len(), 3);
                for (b, r) in rows.iter().enumerate() {
                    if b != idx {
                        assert_eq!(r.linear, [0.; 3]);
                    }
                }
                let normal = -rows[idx].linear[2];
                forces[idx] = normal / 0.001;
                impulse[idx] += normal;
                if normal > 1e-10 && first[idx].is_none() {
                    first[idx] = Some(tick);
                    first_j[idx] = normal;
                }
                if tick > 550 {
                    weight[idx] += normal / 0.05;
                }
            }
            for (i, p) in body.particles().iter().enumerate() {
                let delta = (DVec3::from(p.position) - bench).abs();
                let node_gap = (delta.x - w).max(0.).hypot((delta.y - w).max(0.)) - 0.006;
                if idx == 0 {
                    min_clearance = min_clearance.min(node_gap);
                }
                z[idx] += p.position[2] / p.inverse_mass / 0.14;
                poses.insert(format!("default::{name}::deck::p{i}"), visual(p.position));
            }
            if idx == 0 {
                // Verify all four corners of the box lid lie inside the same
                // projected material triangle, using its actual deformed nodes.
                let [a, b, c] =
                    [0, 1, 3].map(|i| DVec3::from(body.particles()[i].position).truncate());
                let det = (b - a).perp_dot(c - a);
                for dx in [-w, w] {
                    for dy in [-w, w] {
                        let corner = bench.truncate() + glam::DVec2::new(dx, dy);
                        let u = (corner - a).perp_dot(c - a) / det;
                        let v = (b - a).perp_dot(corner - a) / det;
                        min_barycentric = min_barycentric.min(u).min(v).min(1. - u - v);
                    }
                }
            }
            if tick == 600 {
                height[idx] = z[idx];
            }
        }
        pose_frames.push(
            json!({"time_ns":tick*1_000_000,"support_n":forces,"centroid_z_m":z,"poses":poses}),
        );
    }
    assert_eq!(first, [Some(64), Some(64), None]);
    assert!(min_clearance > 0.10 && min_barycentric > 0.1);
    assert!((weight[0] - 0.14 * 9.81).abs() < 0.02);
    assert_eq!(impulse[2], 0.);
    assert!((height[0] - height[1]).abs() < 0.0002 && height[2] < -1.7);
    let measurements = json!({"order":["narrow","wide","miss"],"first_tick":first,"first_impulse_ns":first_j,"total_impulse_ns":impulse,"last_50ms_mean_load_n":weight,"final_centroid_z_m":height,"minimum_narrow_node_skin_to_box_xy_m":min_clearance,"minimum_narrow_corner_barycentric_weight":min_barycentric});
    println!("{measurements}");
    if let Some(out) = env::var_os("GZ_CLOTH_FACE_OUTPUT") {
        let out = Path::new(&out);
        fs::create_dir_all(out).unwrap();
        fs::write(
            out.join("frames.json"),
            serde_json::to_vec(&pose_frames).unwrap(),
        )
        .unwrap();
        fs::write(
            out.join("comparison.json"),
            serde_json::to_vec_pretty(&measurements).unwrap(),
        )
        .unwrap();
    }
}
