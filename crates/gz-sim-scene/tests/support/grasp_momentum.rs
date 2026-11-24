// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DMat3, DQuat, DVec3};
use gz_physics::BodyState;
use gz_physics_distributed::articulation::ArticulationLinkSpec;

pub fn momenta(spec: &ArticulationLinkSpec, state: BodyState) -> (DVec3, DVec3) {
    let q = DQuat::from_array(state.orientation);
    let center = DVec3::from_array(state.position) + q * DVec3::from_array(spec.inertia_center);
    let frame = q * DQuat::from_array(spec.inertia_orientation);
    let [xx, yy, zz, xy, xz, yz] = spec.inertia_tensor;
    let inertia = DMat3::from_cols(
        DVec3::new(xx, xy, xz),
        DVec3::new(xy, yy, yz),
        DVec3::new(xz, yz, zz),
    );
    let omega = DVec3::from_array(state.angular_velocity);
    let p = spec.mass * DVec3::from_array(state.velocity);
    (
        p,
        center.cross(p) + frame * (inertia * (frame.conjugate() * omega)),
    )
}
