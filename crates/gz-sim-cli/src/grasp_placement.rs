// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_scene::distributed_hinge::{GraspFleetLayout, GraspPlacement};

/// Parse the native placement policy used by the tree command and viewer.
pub fn parse_grasp_placement(value: &str) -> Result<GraspPlacement, String> {
    if value == "together" {
        return Ok(GraspPlacement::KeepTogether);
    }
    let Some(dwell) = value.strip_prefix("adaptive=") else {
        return Err("expected together or adaptive=N, with N consecutive clear intervals".into());
    };
    let dwell = dwell
        .parse::<std::num::NonZeroU16>()
        .map_err(|_| "expected a dwell of 1–65535 clear intervals")?;
    Ok(GraspPlacement::Adaptive {
        release_dwell: dwell.get(),
    })
}

/// Place coupled gripper assemblies in one worker or spread them across the pool.
pub fn parse_grasp_fleet_placement(value: &str) -> Result<GraspFleetLayout, String> {
    if let Some(raw) = value.strip_prefix("replay=") {
        let (intervals, events) = raw
            .split_once(',')
            .ok_or("expected replay=INTERVALS,EVENTS")?;
        let number = |v: &str| {
            v.parse::<std::num::NonZeroU32>()
                .map(|n| n.get())
                .map_err(|_| "replay limits must be positive integers")
        };
        return Ok(GraspFleetLayout::Replay {
            max_intervals: number(intervals)?,
            max_events: number(events)?,
        });
    }
    match value {
        "warm" => Ok(GraspFleetLayout::Warm),
        "adaptive" => Ok(GraspFleetLayout::Adaptive),
        "coupled" => Ok(GraspFleetLayout::Coupled),
        "packed" => Ok(GraspFleetLayout::Packed),
        "round-robin" => Ok(GraspFleetLayout::RoundRobin),
        _ => Err(
            "expected packed, round-robin, coupled, adaptive, warm, or replay=INTERVALS,EVENTS"
                .into(),
        ),
    }
}
