// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use naga::valid::{Capabilities, ValidationFlags, Validator};

#[test]
fn vertical_step_validates_with_f64_and_no_optional_arithmetic_features() {
    let shader = module();
    let entry = &shader.entry_points[0];
    assert_eq!(entry.name, "step");
    assert_eq!(entry.workgroup_size, [64, 1, 1]);
    Validator::new(ValidationFlags::all(), Capabilities::FLOAT64)
        .validate(&shader)
        .unwrap();
    assert!(
        Validator::new(ValidationFlags::all(), Capabilities::empty())
            .validate(&shader)
            .is_err()
    );
}

#[test]
fn sphere_plane_step_needs_only_float64_capability() {
    let graph = crate::sphere_plane::module();
    Validator::new(ValidationFlags::all(), Capabilities::FLOAT64)
        .validate(&graph)
        .unwrap();
    assert_eq!(graph.entry_points[0].workgroup_size, [64, 1, 1]);
}

#[test]
fn resident_graph_uses_a_bounded_runtime_loop_and_f64_only() {
    let graph = crate::sphere_block::module();
    Validator::new(ValidationFlags::all(), Capabilities::FLOAT64)
        .validate(&graph)
        .unwrap();
    assert!(
        graph.entry_points[0]
            .function
            .body
            .iter()
            .any(|s| matches!(s, Statement::Loop { .. }))
    );
    assert_eq!(graph.global_variables.len(), 2);
}
