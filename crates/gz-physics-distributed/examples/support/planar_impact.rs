// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent continuum reference for two smooth homogeneous spheres on a horizontal floor.
use gz_physics_distributed::{
    physics::{Body, BodyWrench, PendingImpulse, Primitive},
    pool::PoolFrame,
};
use serde::Serialize;
#[derive(Clone, Copy, Debug, Serialize)]
pub struct Budget {
    pub angular_z_nms: f64,
    pub kinetic_horizontal_j: f64,
}
impl Budget {
    pub fn read(frame: &PoolFrame) -> Self {
        let mut angular_z_nms = 0.;
        let mut kinetic_horizontal_j = 0.;
        for id in [1, 2] {
            let b = frame.body(id).unwrap().1;
            assert_eq!(b.primitive, Primitive::Sphere);
            let radius = b.size[0] / 2.;
            let inertia = 0.4 * b.mass * radius * radius;
            angular_z_nms += b.mass * moment(b, b.linear) + inertia * b.angular[2];
            kinetic_horizontal_j += 0.5 * b.mass * (b.linear[0].powi(2) + b.linear[1].powi(2))
                + 0.5 * inertia * b.angular.into_iter().map(|x| x * x).sum::<f64>();
        }
        Self {
            angular_z_nms,
            kinetic_horizontal_j,
        }
    }
}
fn moment(b: &Body, vector: [f64; 3]) -> f64 {
    b.position[0] * vector[1] - b.position[1] * vector[0]
}
/// The two ordered inputs and the single held horizontal force precede
/// collision. During that force there is no sideways velocity, so its moment
/// arm for the vertical-axis torque integral is constant throughout the step.
pub fn external(frame: &PoolFrame, loads: &[BodyWrench], queued: &[PendingImpulse]) -> f64 {
    let dt = frame.step().duration().as_secs_f64();
    queued
        .iter()
        .map(|p| moment(frame.body(p.id).unwrap().1, p.impulse))
        .sum::<f64>()
        + loads
            .iter()
            .map(|p| {
                let b = frame.body(p.id).unwrap().1;
                assert_eq!(b.linear[1], 0.);
                (moment(b, p.force) + p.torque[2]) * dt
            })
            .sum::<f64>()
}
#[derive(Serialize)]
pub struct Prediction {
    pub normal_xy: [f64; 2],
    pub impact_ns: f64,
    pub horizontal_m_s: [[f64; 2]; 2],
    pub dissipated_j: f64,
}
/// Predict from the cold seeds and the prescribed pre-impact loading schedule.
pub fn collision([a, b]: &[Body; 2]) -> Prediction {
    let velocity_a = a.linear[0] + (0.25 + 6. * 0.01) / a.mass;
    let velocity_b = b.linear[0] + 0.15 / b.mass;
    let distance = (a.size[0] + b.size[0]) * 0.5;
    let ny = (b.position[1] - a.position[1]) / distance;
    let nx = (1. - ny.powi(2)).sqrt();
    assert!(
        nx.is_finite() && a.linear[1] == 0. && b.linear[1] == 0. && a.position[0] < b.position[0]
    );
    let approach = (velocity_a - velocity_b) * nx;
    let reduced_mass = (1. / a.mass + 1. / b.mass).recip();
    let impulse = reduced_mass * approach;
    Prediction {
        normal_xy: [nx, ny],
        impact_ns: impulse,
        horizontal_m_s: [
            [velocity_a - impulse * nx / a.mass, -impulse * ny / a.mass],
            [velocity_b + impulse * nx / b.mass, impulse * ny / b.mass],
        ],
        dissipated_j: 0.5 * reduced_mass * approach * approach,
    }
}
