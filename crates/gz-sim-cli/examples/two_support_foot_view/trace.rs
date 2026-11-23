// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Recorded native boxes, clocks and step impulses shared by the paired support replays.
use serde::Deserialize;
#[derive(Deserialize, PartialEq)]
pub(super) struct Trace {
    pub(super) dt_s: f64,
    pub(super) geometry: Geometry,
    pub(super) frames: Vec<Sample>,
}
#[derive(Deserialize, PartialEq)]
pub(super) struct Geometry {
    pub(super) support_a_center: [f64; 3],
    pub(super) support_b_center: [f64; 3],
    pub(super) support_a_half_extents: [f64; 3],
    pub(super) support_b_half_extents: [f64; 3],
    pub(super) foot_half_extents: [f64; 3],
    pub(super) upper_half_extents: [f64; 3],
}
#[derive(Deserialize, PartialEq)]
pub(super) struct Sample {
    pub(super) tick: u64,
    pub(super) time_s: f64,
    pub(super) switched: Lane,
    pub(super) reference: Lane,
    pub(super) support_b_present: bool,
    pub(super) prescribed: bool,
}
#[derive(Deserialize, PartialEq)]
pub(super) struct Lane {
    pub(super) foot: Pose,
    pub(super) upper: Pose,
    pub(super) normal_impulses_ns: [f64; 2],
}
#[derive(Deserialize, PartialEq)]
pub(super) struct Pose {
    pub(super) position: [f64; 3],
    pub(super) orientation: [f64; 4],
}
#[derive(Deserialize, PartialEq)]
pub(super) struct Events {
    pub(super) demotion_tick: u64,
    pub(super) loss_tick: u64,
    pub(super) promotion_tick: u64,
    pub(super) load_transfer_tick: u64,
}
