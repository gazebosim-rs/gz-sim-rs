// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use sdformat::NoResources;
use std::time::Duration;
#[test]
fn exactly_authored_integer_nanoseconds_survive_seconds_conversion() {
    for (decimal, ns) in [
        ("0.000000060000", 60),
        ("0.000000059999999999", 59),
        ("0.00000006075", 60),
        ("0.000000128", 128),
        ("0.000500000", 500000),
    ] {
        let xml = format!(
            r#"<sdf version="1.12"><world name="clock"><physics name="p" type="ignored"><max_step_size>{decimal}</max_step_size></physics></world></sdf>"#
        );
        let mut sim = load_world_with_sdf_step(
            &xml,
            "clock.sdf",
            "clock",
            &NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        sim.step_exact(8).unwrap();
        assert_eq!(sim.elapsed(), Duration::from_nanos(8 * ns));
    }
}
