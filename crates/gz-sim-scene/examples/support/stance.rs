// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Sole geometry and committed visual sampling for the stance qualification.
use bevy_ecs::entity::Entity;
use glam::{DQuat, DVec2, DVec3};
use gz_sim_core::Simulation;
use gz_sim_scene::WorldPose;

pub const FLOOR: &str = r#"<model name="floor"><static>true</static><link name="body"><pose>0 0 -0.06 0 0 0</pose><collision name="ground"><geometry><box><size>2 1.5 0.12</size></box></geometry></collision><visual name="pad"><geometry><box><size>2 1.5 0.12</size></box></geometry></visual></link></model>"#;

/// Signed horizontal distance from COM to the convex hull of both actual foot
/// soles. Positive means inside the geometric double-support polygon. This
/// measure has physical meaning only while both feet carry contact.
pub fn support(sim: &Simulation, feet: [Entity; 2], center: DVec3) -> (f64, f64) {
    let mut points = [DVec2::ZERO; 8];
    let mut depth = 0_f64;
    for (i, entity) in feet.into_iter().enumerate() {
        let pose = sim.world().get::<WorldPose>(entity).unwrap();
        let origin = DVec3::from_array(pose.translation);
        let rotation = DQuat::from_array(pose.rotation);
        for (corner, (x, y)) in [(-0.12, -0.05), (-0.12, 0.05), (0.12, -0.05), (0.12, 0.05)]
            .into_iter()
            .enumerate()
        {
            let p = origin + rotation * DVec3::new(x, y, -0.035);
            points[4 * i + corner] = p.truncate();
            depth = depth.max(-p.z);
        }
    }
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    let mut hull = [DVec2::ZERO; 16];
    let mut len = 0;
    for upper in [false, true] {
        let start = len;
        for i in 0..8 {
            let p = points[if upper { 7 - i } else { i }];
            while len >= start + 2
                && (hull[len - 1] - hull[len - 2]).perp_dot(p - hull[len - 2]) <= 0.
            {
                len -= 1;
            }
            hull[len] = p;
            len += 1;
        }
        len -= 1;
    }
    let mut margin = f64::INFINITY;
    for i in 0..len {
        let edge = hull[(i + 1) % len] - hull[i];
        margin = margin.min(edge.normalize().perp_dot(center.truncate() - hull[i]));
    }
    (margin, depth)
}

pub fn paint(name: &str) -> [f64; 3] {
    if name.contains("::floor::") {
        [0.24, 0.30, 0.37]
    } else if name.contains("_hand::") {
        [0.95, 0.64, 0.13]
    } else if name.contains("left_") {
        [0.04, 0.67, 0.57]
    } else if name.contains("right_") {
        [0.18, 0.48, 0.75]
    } else {
        [0.72, 0.78, 0.81]
    }
}
