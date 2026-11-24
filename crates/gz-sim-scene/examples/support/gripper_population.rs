// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Generate ordinary independent SDF models from the qualified gripper and free-sphere geometry.
const WORLD: &str = include_str!("../../../../examples/gpu_parallel_gripper.sdf");
pub const CASES: [(f64, f64, bool, u64, u64); 8] = [
    (0.08, 0.6, false, 201, 651),
    (0.2, 0.6, false, 211, 686),
    (0.35, 0.6, false, 225, 671),
    (0.08, 0., false, 201, 651),
    (0.08, 0.6, true, 201, 651),
    (0.2, 0.3, false, 201, 651),
    (0.08, 0.6, false, 0, 651),
    (0.08, 0.6, false, 245, 801),
];
fn lift_command(i: usize) -> (u64, u64, f64) {
    let (_, _, _, lift, release) = CASES[i % CASES.len()];
    let stagger = (i / CASES.len() % 16) as u64;
    let height = if lift == 0 {
        0.
    } else {
        0.122 + (i / CASES.len()) as f64 * 0.00001
    };
    (lift.max(201) + stagger, release + stagger, height)
}
pub fn action(i: usize, tick: u64) -> [f64; 3] {
    let (lift, release, height) = lift_command(i);
    let jaw = if CASES[i % 8].2 || tick >= release {
        0.
    } else {
        0.028
    };
    [if tick >= lift { height } else { 0. }, jaw, -jaw]
}
pub fn streamed(xml: &str) -> String {
    let tree = roxmltree::Document::parse(xml).unwrap();
    let mut source = xml.to_owned();
    for range in tree
        .descendants()
        .filter(|n| n.has_tag_name("target"))
        .map(|n| n.range())
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
    {
        source.replace_range(range, "");
    }
    source
}
pub fn xml(count: usize) -> String {
    let tree = roxmltree::Document::parse(WORLD).unwrap();
    let node = |tag, name: Option<&str>| {
        tree.descendants()
            .find(|n| n.has_tag_name(tag) && name.is_none_or(|v| n.attribute("name") == Some(v)))
            .unwrap()
    };
    let env = node("environment", None);
    let hand = node("model", Some("gripper"));
    let ball = node("model", Some("parcel"));
    let table = node("model", Some("table"));
    let block = |n: roxmltree::Node<'_, '_>| &WORLD[n.range()];
    let mut envs = String::new();
    let mut models = String::new();
    for i in 0..count {
        let (mass, mu, open, _, _) = CASES[i % CASES.len()];
        let (lift, release, height) = lift_command(i);
        let x = (i % 4) as f64 * 0.16;
        let y = i as f64 * 0.08;
        let hand_name = format!("gripper_{i:04}");
        let obj_name = format!("parcel_{i:04}");
        let mut e = block(env)
            .replace("id=\"12\"", &format!("id=\"{i}\""))
            .replace("model=\"gripper\"", &format!("model=\"{hand_name}\""))
            .replace("object=\"parcel\"", &format!("object=\"{obj_name}\""))
            .replace("tick=\"201\"", &format!("tick=\"{}\"", lift))
            .replace("tick=\"651\"", &format!("tick=\"{}\"", release))
            .replace("lift=\"0.122\"", &format!("lift=\"{height}\""));
        if open {
            e = e
                .replace("\"0.028\"", "\"0\"")
                .replace("\"-0.028\"", "\"0\"");
        }
        envs.push_str(&e);
        let publisher = "<plugin name=\"gz::sim::systems::JointStatePublisher\" filename=\"gz-sim-joint-state-publisher-system\"><joint_name>lift</joint_name><joint_name>left_slider</joint_name><joint_name>right_slider</joint_name></plugin>";
        models.push_str(
            &block(hand)
                .replace(
                    "<model name=\"gripper\">",
                    &format!("<model name=\"{hand_name}\"><pose>{x} {y} 0 0 0 0</pose>"),
                )
                .replace(
                    "<mu>0.6</mu><mu2>0.6</mu2>",
                    &format!("<mu>{mu}</mu><mu2>{mu}</mu2>"),
                )
                .replace("</model>", &format!("{publisher}</model>")),
        );
        let inert = 0.4 * mass * 0.022 * 0.022;
        models.push_str(
            &block(ball)
                .replace("name=\"parcel\"", &format!("name=\"{obj_name}\""))
                .replace("0 0 0.022 0 0 0", &format!("{x} {y} 0.022 0 0 0"))
                .replace("<mass>0.08</mass>", &format!("<mass>{mass}</mass>"))
                .replace("0.000015488", &inert.to_string()),
        );
    }
    let floor = block(table)
        .replace("0.32 0.12 0.018", "0.90 0.85 0.018")
        .replace("0 0 -0.009", "0.24 0.20 -0.009");
    format!(
        "{}{}{}{}{}{}",
        &WORLD[..env.range().start],
        envs,
        &WORLD[env.range().end..table.range().start],
        floor,
        models,
        &WORLD[ball.range().end..]
    )
}
