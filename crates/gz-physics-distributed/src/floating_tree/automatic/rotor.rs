// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

/// Construction of a spherical passive receiver carried by one fixed-world revolute.
/// The axis and anchor use world coordinates. The public link's local inertia is explicit.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RotorPadSpec {
    pub model_id: u64,
    pub link: ArticulationLinkSpec,
    pub joint_id: u64,
    pub anchor: [f64; 3],
    pub axis: [f64; 3],
    pub lower: f64,
    pub upper: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AutomaticRotorMaterialSpec {
    pub pad: RotorPadSpec,
    pub material_name: String,
    pub step_ns: u64,
    pub restitution: f64,
}
