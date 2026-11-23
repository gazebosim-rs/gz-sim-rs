// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::primitive_replay_view::View;
use gz_physics_distributed::moving::{MovingObservation, MovingSpec};
use rapier3d_f64::math::{Rotation, Vector};
use serde_json::json;

pub(super) const BLUE: [f64; 4] = [0.13, 0.57, 0.93, 1.];
pub(super) const ORANGE: [f64; 4] = [0.95, 0.42, 0.14, 1.];
pub(super) const GRAY: [f64; 4] = [0.20, 0.27, 0.33, 1.];
pub(super) const GOLD: [f64; 4] = [0.99, 0.76, 0.16, 1.];
pub(super) const ID: [f64; 4] = [0., 0., 0., 1.];
const SCALE: f64 = 2.4;
pub(super) const MAGENTA: [f64; 4] = [0.84, 0.19, 0.58, 1.];

#[path = "hinge_rod.rs"]
mod hinge_rod;
use super::primitive_replay_view;
use hinge_rod::rod;

/// Draw the three authored spheres, the revolute anchor and the last normal impulse.
pub(super) fn draw(
    v: &mut View,
    spec: &MovingSpec,
    s: &MovingObservation,
    label: &str,
    base: u64,
    y: f64,
) {
    let origin =
        (Vector::from(spec.hinge.links[0].position) + Vector::from(spec.sphere.position)) * 0.5;
    let project = |p: Vector| (p - origin) * SCALE + Vector::new(0., y, 0.7);
    for (i, (rest, body, name, color)) in [
        (spec.hinge.links[0], s.hinge.links[0], "base", BLUE),
        (spec.hinge.links[1], s.hinge.links[1], "tool", ORANGE),
        (spec.sphere, s.sphere, "free_sphere", MAGENTA),
    ]
    .into_iter()
    .enumerate()
    {
        let id = base + i as u64 * 6;
        let p = Vector::from(body.position);
        v.add(
            id,
            &format!("{label}_{name}"),
            project(p).to_array(),
            body.orientation,
            json!({"kind":"sphere","radius":rest.radius*SCALE}),
            color,
        );
        if i < 2 {
            let to_joint = Rotation::from_array(rest.orientation).inverse()
                * (Vector::from(spec.hinge.anchor) - Vector::from(rest.position));
            let a = p + Rotation::from_array(body.orientation) * to_joint;
            rod(
                v,
                id + 2,
                &format!("{label}_{name}_to_hinge"),
                project(p),
                project(a),
                to_joint.length() * SCALE,
                color,
            );
            if i == 0 {
                v.add(
                    id + 4,
                    &format!("{label}_hinge"),
                    project(a).to_array(),
                    ID,
                    json!({"kind":"sphere","radius":0.082}),
                    GOLD,
                );
            }
        }
    }
    // Display contact magnitude on a fixed scale even when the loaded interval lasts one tick.
    let normal = s.reactions.iter().map(|r| r.normal_impulse).sum::<f64>();
    let x = 2.22;
    v.cuboid(
        base + 30,
        &format!("{label}_impact_scale"),
        [x, y, 0.65],
        [0.05, 0.05, 1.22],
        GRAY,
    );
    v.add(
        base + 32,
        &format!("{label}_impact_impulse"),
        [x, y, 0.04 + 1.22 * normal / (normal + 0.10)],
        ID,
        json!({"kind":"sphere","radius":0.095}),
        GOLD,
    );
}
