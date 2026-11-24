// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "private/rolling_control.rs"]
mod rolling_control;

#[test]
fn rolling_cylinder_retains_contacts_and_casts_barrel_and_flat_endcap_rays()
-> Result<(), Box<dyn std::error::Error>> {
    rolling_control::qualify(
        include_str!("../../../examples/distributed_rolling_cylinder.sdf"),
        gz_physics_distributed::physics::Primitive::Cylinder,
    )
}
