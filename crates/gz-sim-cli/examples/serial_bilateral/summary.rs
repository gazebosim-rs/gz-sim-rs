// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[derive(Serialize)]
pub(super) struct Hold {
    pub(super) jaw_normal_n: [f64; 2],
    pub(super) jaw_force_n: [[f64; 3]; 2],
    pub(super) jaw_torque_nm: [[f64; 3]; 2],
}
fn hold(window: &[Sample]) -> Hold {
    let seconds = window.len() as f64 * 0.001;
    let mut normal = [0.; 2];
    let mut force = [DVec3::ZERO; 2];
    let mut torque = [DVec3::ZERO; 2];
    for sample in window {
        for (i, jaw) in [&sample.contacts.left, &sample.contacts.right]
            .into_iter()
            .enumerate()
        {
            normal[i] += jaw.normal_ns;
            force[i] += DVec3::from(jaw.impulse_ns);
            torque[i] += DVec3::from(jaw.angular_nms);
        }
    }
    Hold {
        jaw_normal_n: normal.map(|v| v / seconds),
        jaw_force_n: force.map(|v| (v / seconds).into()),
        jaw_torque_nm: torque.map(|v| (v / seconds).into()),
    }
}
#[derive(Serialize)]
pub(super) struct Report {
    pub(super) loaded_101_to_140_ms: Hold,
    pub(super) loaded_246_to_280_ms: Hold,
    pub(super) first_grip_ms: Option<u32>,
    pub(super) first_clear_ms: Option<u32>,
    pub(super) first_pad_return_ms: [Option<u32>; 2],
    pub(super) first_regrip_ms: Option<u32>,
    pub(super) first_tray_ms: Option<u32>,
    pub(super) peak_linear_residual_ns: f64,
    pub(super) peak_angular_residual_nms: f64,
    pub(super) max_drop_mm: f64,
}
pub(super) fn report(rows: &[Sample]) -> Report {
    let both = |s: &Sample| s.contacts.left.normal_ns > 0. && s.contacts.right.normal_ns > 0.;
    let first = |a: &[Sample], p: &dyn Fn(&Sample) -> bool| a.iter().find(|s| p(s)).map(|s| s.tick);
    let open = &rows[140..];
    Report {
        loaded_101_to_140_ms: hold(&rows[100..140]),
        loaded_246_to_280_ms: hold(&rows[245..280]),
        first_grip_ms: first(rows, &both),
        first_clear_ms: first(open, &|s| {
            s.contacts.left.normal_ns == 0. && s.contacts.right.normal_ns == 0.
        }),
        first_pad_return_ms: [
            first(&rows[164..], &|s| s.contacts.left.normal_ns > 0.),
            first(&rows[164..], &|s| s.contacts.right.normal_ns > 0.),
        ],
        first_regrip_ms: first(&rows[164..], &both),
        first_tray_ms: first(rows, &|s| s.contacts.tray.normal_ns > 0.),
        peak_linear_residual_ns: rows
            .iter()
            .map(|s| DVec3::from(s.linear_residual_ns).length())
            .fold(0., f64::max),
        peak_angular_residual_nms: rows
            .iter()
            .map(|s| DVec3::from(s.angular_residual_nms).length())
            .fold(0., f64::max),
        max_drop_mm: rows
            .iter()
            .map(|s| 1000. * (0.556 - s.xyz_m[2]))
            .fold(0., f64::max),
    }
}
