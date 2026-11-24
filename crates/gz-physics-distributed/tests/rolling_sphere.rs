// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "private/rolling_control.rs"]
mod rolling_control;

#[test]
fn rolling_sphere_meets_a_loaded_box_without_discontinuity_and_casts_curved_rays()
-> Result<(), Box<dyn std::error::Error>> {
    rolling_control::qualify(
        include_str!("../../../examples/distributed_rolling_sphere.sdf"),
        gz_physics_distributed::physics::Primitive::Sphere,
    )
}
