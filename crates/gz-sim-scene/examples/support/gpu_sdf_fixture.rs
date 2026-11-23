// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use std::fmt::Write as _;
const EXAMPLE: &str = include_str!("../../../../examples/gpu_mixed_world.sdf");
const GROUPS: [(&str, &[&str]); 3] = [
    ("sphere-plane", &["rolling", "incline"]),
    ("sphere-pair", &["free_a", "free_b"]),
    (
        "supported-pair",
        &["supported_a", "supported_b", "supported_plane"],
    ),
];
/// Replicate each authored SDF model into a separate cell while retaining its actual
/// visible geometry, mass and collision. All coordinates are in the ordinary world.
pub fn fixture(count: usize, every: u64) -> String {
    fixture_from(EXAMPLE, count, every)
}
/// Use authored coordinates and geometry from this world before translating the cells.
pub fn fixture_from(authored_world: &str, count: usize, every: u64) -> String {
    let doc = roxmltree::Document::parse(authored_world).unwrap();
    let world = doc.descendants().find(|n| n.has_tag_name("world")).unwrap();
    let mut groups = String::new();
    let mut models = String::new();
    for id in 0..count {
        let (profile, names) = GROUPS[id % 3];
        let node = world
            .descendants()
            .find(|n| n.has_tag_name("environment") && n.attribute("profile") == Some(profile))
            .unwrap();
        let mut group = authored_world[node.range()].replace(
            &format!("id=\"{}\"", node.attribute("id").unwrap()),
            &format!("id=\"{id}\""),
        );
        let x = 3. * (id % 48) as f64;
        let y = 5. * (id / 48) as f64;
        for name in names {
            let unique = format!("cell_{id}_{name}");
            group = group.replace(&format!("link=\"{name}::"), &format!("link=\"{unique}::"));
            let model = world
                .children()
                .find(|n| n.has_tag_name("model") && n.attribute("name") == Some(*name))
                .unwrap();
            let pose = model.children().find(|n| n.has_tag_name("pose")).unwrap();
            let mut xyzrpy: Vec<f64> = pose
                .text()
                .unwrap()
                .split_whitespace()
                .map(|v| v.parse().unwrap())
                .collect();
            xyzrpy[0] += x;
            xyzrpy[1] += y;
            let pose_text = xyzrpy
                .iter()
                .map(f64::to_string)
                .collect::<Vec<_>>()
                .join(" ");
            let authored = authored_world[model.range()]
                .replacen(
                    &format!("name=\"{name}\""),
                    &format!("name=\"{unique}\""),
                    1,
                )
                .replacen(
                    &authored_world[pose.range()],
                    &format!("<pose>{pose_text}</pose>"),
                    1,
                );
            models.push_str(&authored);
        }
        groups.push_str(&group);
    }
    if every > 0 {
        write!(groups, "<observe every=\"{every}\"/>").unwrap();
    }
    format!(
        r#"<sdf version="1.12"><world name="default"><gravity>0 0 -1.2</gravity><physics name="gpu" type="ignored"><max_step_size>0.001</max_step_size></physics><plugin name="gz::sim::systems::GpuBatch" filename="gz-sim-gpu-batch-system">{groups}</plugin>{models}</world></sdf>"#
    )
}
