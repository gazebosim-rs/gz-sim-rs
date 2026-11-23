// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{
    ArticulationObservation, ArticulationSpec, Result, control, primitive_replay_view::View,
};
use rapier3d_f64::math::{Rotation, Vector};
use serde_json::{Value, json};

const GRAY: [f64; 4] = [0.17, 0.25, 0.34, 1.];
const GOLD: [f64; 4] = [0.99, 0.68, 0.12, 1.];
const GREEN: [f64; 4] = [0.11, 0.80, 0.60, 1.];
const BLUE: [f64; 4] = [0.14, 0.50, 0.87, 1.];
const ORANGE: [f64; 4] = [0.96, 0.39, 0.12, 1.];
const SCALE: f64 = 2.8;
const IDENTITY: [f64; 4] = [0., 0., 0., 1.];

fn segment(
    view: &mut View,
    id: u64,
    label: &str,
    start: Vector,
    end: Vector,
    length: f64,
    color: [f64; 4],
) {
    let unit = (end - start).normalize();
    let half = ((1. + unit.x) * 0.5).sqrt();
    let orientation = if half < 1e-10 {
        [0., 0., 1., 0.]
    } else {
        [0., -unit.z / (2. * half), unit.y / (2. * half), half]
    };
    view.add(
        id,
        label,
        ((start + end) * 0.5).to_array(),
        orientation,
        json!({"kind":"box","size":[length,0.065,0.065]}),
        color,
    );
}

pub(super) struct Authority<'a> {
    pub owner: u8,
    pub width: u8,
    pub events: &'a [Value],
}
pub(super) fn draw(
    spec: &ArticulationSpec,
    actual: &ArticulationObservation,
    control: &control::Observation,
    authority: Authority,
    efforts: &[f64],
    ordinal: u64,
) -> Result<((String, String), f64)> {
    let Authority {
        owner,
        width,
        events,
    } = authority;
    let mut view = View::default();
    let mut max_gap: f64 = 0.;
    let origin = Vector::from(spec.links[0].position);
    for (lane, label) in ["uninterrupted_SDF", "distributed_worker"]
        .into_iter()
        .enumerate()
    {
        let shift = Vector::new(-0.85, if lane == 0 { 1.15 } else { -1.15 }, 1.);
        let project = |point: Vector| (point - origin) * SCALE + shift;
        let mut anchors = [Vector::ZERO; 2];
        let id = 10 + lane as u64 * 30;
        for (i, (link, anchor_out)) in spec.links.iter().zip(&mut anchors).enumerate() {
            let (position, orientation) = if lane == 0 {
                (control.links[i].position, control.links[i].orientation)
            } else {
                (actual.links[i].position, actual.links[i].orientation)
            };
            let pos = Vector::from(position);
            let rot = Rotation::from_array(orientation);
            let rest_rot = Rotation::from_array(link.orientation);
            let local_anchor =
                rest_rot.inverse() * (Vector::from(spec.anchor) - Vector::from(link.position));
            let anchor = pos + rot * local_anchor;
            *anchor_out = anchor;
            let link_name = if i == 0 { "base" } else { "tool" };
            let color = if i == 0 { BLUE } else { ORANGE };
            let idx = id + 8 * i as u64;
            view.add(
                idx,
                &format!("{label}_{link_name}"),
                project(pos).to_array(),
                orientation,
                json!({"kind":"sphere","radius":link.radius*SCALE}),
                color,
            );
            segment(
                &mut view,
                idx + 2,
                &format!("{label}_{link_name}_to_joint"),
                project(pos),
                project(anchor),
                local_anchor.length() * SCALE,
                color,
            );
        }
        let gap = (anchors[0] - anchors[1]).length();
        if !gap.is_finite() || gap > 1e-9 {
            return Err(format!("{label} hinge opened by {gap:e} m").into());
        }
        max_gap = max_gap.max(gap);
        view.add(
            id + 16,
            &format!("{label}_hinge"),
            project(anchors[0]).to_array(),
            IDENTITY,
            json!({"kind":"sphere","radius":0.1}),
            GOLD,
        );
        view.cuboid(
            id + 18,
            &format!("{label}_lane"),
            [0., shift.y, 0.08],
            [3.9, 1.8, 0.08],
            GRAY,
        );
    }
    let x_pos = |worker: u8| (f64::from(worker) - 0.5 * f64::from(width - 1)) * 0.42;
    for worker in 0..width {
        view.cuboid(
            100 + u64::from(worker) * 2,
            &format!("worker_{worker}"),
            [x_pos(worker), -2.45, 0.12],
            [0.31, 0.28, 0.13],
            GRAY,
        );
    }
    view.add(
        130,
        "active_worker",
        [x_pos(owner), -2.45, 0.31],
        IDENTITY,
        json!({"kind":"sphere","radius":0.135}),
        GREEN,
    );
    if let Some(event) = events.last() {
        let kind = event["kind"]
            .as_str()
            .ok_or("missing measured worker event")?;
        let color = match kind {
            "transfer" => GOLD,
            "owner_sigkill" => [0.97, 0.12, 0.09, 1.],
            "owner_replaced" => BLUE,
            "disk_resume" => [0.69, 0.38, 0.94, 1.],
            _ => return Err("unexpected ownership event".into()),
        };
        view.cuboid(
            1000 + actual.tick * 2,
            &format!("{kind}_at_{}_ms", actual.tick),
            [x_pos(owner), -2.8, 0.25],
            [0.22, 0.22, 0.32],
            color,
        );
    }
    let held = efforts.iter().sum::<f64>();
    view.cuboid(
        140,
        "effort_zero",
        [0., 2.55, 0.12],
        [0.06, 0.36, 0.17],
        GRAY,
    );
    view.add(
        142,
        "pending_effort_Nm",
        [0.35 * held, 2.55, 0.24],
        IDENTITY,
        json!({"kind":"sphere","radius":0.10}),
        GOLD,
    );
    Ok((view.finish_at(ordinal, actual.tick * 1_000_000), max_gap))
}
