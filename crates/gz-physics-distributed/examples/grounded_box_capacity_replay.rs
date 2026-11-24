// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Render the managed 32-link pressure array and its measured normal impulses.
#[path = "../../../examples/support/grounded_box_array.rs"]
mod grounded_box_array;
#[path = "../../gz-sim-cli/examples/soft_replay/mod.rs"]
mod soft_replay;
use glam::{DQuat, DVec3};
use gz_physics_distributed::{
    articulation::ArticulationLinkState, grounded_robot::RobotGroundPatch,
};
use serde::Deserialize;
use serde_json::json;
use soft_replay::Frame;
use std::{
    collections::BTreeMap,
    env,
    error::Error,
    fs::File,
    io::{BufRead, BufReader},
    path::Path,
    time::Duration,
};
#[derive(Deserialize)]
struct Sample {
    tick: u64,
    links: Vec<ArticulationLinkState>,
    patches: Vec<RobotGroundPatch>,
}
fn project(sample: &Sample) -> Frame {
    let mut frame = Frame::default();
    let rotation = DQuat::from_rotation_z(-0.62);
    let center = DVec3::new(0.56, 0.24, 0.);
    let world = |p: DVec3, side: f64| rotation * (p - center) + DVec3::Y * side;
    for (label, side) in [("simulated_floor", 0.79), ("normal_load_map", -0.79)] {
        frame.add(
            label,
            world(center - DVec3::Z * 0.02, side),
            rotation,
            json!({"kind":"box","size":[1.35,0.72,0.04]}),
            [0.17, 0.22, 0.26, 1.],
        );
    }
    let reference_start = world(DVec3::new(-0.06, -0.10, 0.1101), -0.79);
    let reference_end = world(DVec3::new(1.18, -0.10, 0.1101), -0.79);
    frame.edge(
        "one_kg_weight_9_81_N",
        reference_start,
        reference_end,
        1.24,
        0.009,
        [0.49, 0.66, 0.80, 1.],
    );
    for (i, link) in sample.links.iter().enumerate() {
        let p = DVec3::from(link.position);
        let patch = sample.patches.iter().find(|v| v.link_id == link.id);
        let sum = patch
            .into_iter()
            .flat_map(|v| &v.points)
            .map(|v| v.normal_impulse)
            .sum::<f64>();
        let color = if i == 19 {
            [0.97, 0.34, 0.12, 1.]
        } else {
            [0.12, 0.59, 0.55, 1.]
        };
        frame.add(
            &format!("body_{}", link.id),
            world(p, 0.79),
            rotation * DQuat::from_array(link.orientation),
            json!({"kind":"box","size":[0.12,0.12,0.12]}),
            color,
        );
        let xy = DVec3::new((i % 8) as f64 * 0.16, (i / 8) as f64 * 0.16, 0.);
        // 1 N = 10 mm; native impulse is measured across exactly one millisecond.
        frame.add(
            &format!("support_N_{}", link.id),
            world(xy + DVec3::Z * (10. * sum + 0.012), -0.79),
            rotation,
            json!({"kind":"sphere","radius":0.024}),
            color,
        );
        if i % 4 == 3 {
            let mut corners = patch
                .into_iter()
                .flat_map(|v| &v.points)
                .collect::<Vec<_>>();
            corners.sort_by(|a, b| {
                a.plane_witness[0]
                    .total_cmp(&b.plane_witness[0])
                    .then_with(|| a.plane_witness[1].total_cmp(&b.plane_witness[1]))
            });
            for j in 0..4 {
                let point = corners
                    .get(j)
                    .map(|c| {
                        let mut w = DVec3::from(c.plane_witness);
                        w.z = 40. * c.normal_impulse + 0.008;
                        w
                    })
                    .unwrap_or_else(|| {
                        xy + DVec3::new(
                            if j & 2 == 0 { -0.06 } else { 0.06 },
                            if j & 1 == 0 { -0.06 } else { 0.06 },
                            0.008,
                        )
                    });
                frame.add(
                    &format!("corner_{}_{}", link.id, j),
                    world(point, -0.79),
                    rotation,
                    json!({"kind":"sphere","radius":0.011}),
                    [0.98, 0.72, 0.19, 1.],
                );
            }
        }
    }
    frame
}
fn main() -> Result<(), Box<dyn Error>> {
    let mut a = env::args_os().skip(1);
    let input = a.next().ok_or("provide measured timeline.jsonl")?;
    let output = a.next().ok_or("provide fresh output directory")?;
    let exe = a.next().ok_or("provide real physics_worker executable")?;
    let xml = grounded_box_array::boxes(32);
    let spec = gz_sim_scene::distributed_hinge::distributed_grounded_robot_spec(
        &xml,
        "array.sdf",
        "default",
    )?;
    let temporary = tempfile::tempdir()?;
    let cold = gz_physics_distributed::grounded_robot::GroundedRobotPool::create(
        Path::new(&exe),
        temporary.path(),
        2,
        spec,
    )?;
    let initial = Sample {
        tick: 0,
        links: cold.observation().links.clone(),
        patches: cold.ground_patches().to_vec(),
    };
    let mut frames = vec![project(&initial).finish(Duration::ZERO, 1)];
    drop(cold);
    for line in BufReader::new(File::open(input)?).lines() {
        let s: Sample = serde_json::from_str(&line?)?;
        if s.links.len() != 32 {
            return Err("incomplete measured array".into());
        }
        frames.push(project(&s).finish(Duration::from_millis(s.tick), frames.len() + 1));
    }
    if frames.len() != 161 {
        return Err("incomplete capacity trajectory".into());
    }
    gz_sim_cli::export_archive(Path::new(&output), BTreeMap::new(), frames)?;
    Ok(())
}
