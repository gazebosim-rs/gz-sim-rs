// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use primitive_replay_view::View;
use rapier3d_f64::math::{Rotation, Vector};

const BLUE: [f64; 4] = [0.13, 0.57, 0.93, 1.];
const ORANGE: [f64; 4] = [0.95, 0.42, 0.14, 1.];
const GRAY: [f64; 4] = [0.20, 0.27, 0.33, 1.];
const GREEN: [f64; 4] = [0.15, 0.87, 0.56, 1.];
const GOLD: [f64; 4] = [0.99, 0.76, 0.16, 1.];
const ID: [f64; 4] = [0., 0., 0., 1.];
const SCALE: f64 = 3.;

fn rod(
    v: &mut View,
    id: u64,
    name: &str,
    start: Vector,
    end: Vector,
    length: f64,
    color: [f64; 4],
) {
    let displacement = end - start;
    let dir = displacement.normalize();
    let w = ((1. + dir.x) * 0.5).sqrt();
    let rotation = if w < 1e-10 {
        [0., 0., 1., 0.]
    } else {
        [0., -dir.z / (2. * w), dir.y / (2. * w), w]
    };
    v.add(
        id,
        name,
        ((start + end) * 0.5).to_array(),
        rotation,
        json!({"kind":"box","size":[length,0.06,0.06]}),
        color,
    );
}

pub(super) fn draw(
    spec: &ArticulationSpec,
    control: &ArticulationObservation,
    worker: &ArticulationObservation,
    owner: u8,
    width: u8,
    event: Option<&str>,
    ordinal: u64,
) -> Result<(String, String)> {
    let mut v = View::default();
    let ground = spec.ground.ok_or("ground floor required")?;
    let weight = -ground.gravity_z * spec.links.iter().map(|l| l.mass).sum::<f64>();
    let origin = Vector::new(
        spec.links[0].position[0],
        spec.links[0].position[1],
        ground.height,
    );
    for (lane, (label, state)) in [
        ("uninterrupted_SDF", control),
        ("distributed_worker", worker),
    ]
    .into_iter()
    .enumerate()
    {
        let y = if lane == 0 { 1.1 } else { -1.1 };
        let start = 10 + lane as u64 * 40;
        let project = |p: Vector| (p - origin) * SCALE + Vector::new(-0.8, y, 0.);
        let mut anchor = [Vector::ZERO; 2];
        let contacts = state.ground_reactions.ok_or("ground reaction missing")?;
        v.cuboid(
            start,
            &format!("{label}_floor"),
            [0., y, -0.06],
            [4.5, 1.9, 0.12],
            GRAY,
        );
        for i in 0..2 {
            let name = if i == 0 { "base" } else { "tool" };
            let id = start + 2 + i as u64 * 10;
            let body = &state.links[i];
            let rest = &spec.links[i];
            let color = if i == 0 { BLUE } else { ORANGE };
            let p = Vector::from(body.position);
            let local_anchor = Rotation::from_array(rest.orientation).inverse()
                * (Vector::from(spec.anchor) - Vector::from(rest.position));
            anchor[i] = p + Rotation::from_array(body.orientation) * local_anchor;
            let center = project(p);
            v.add(
                id,
                &format!("{label}_{name}"),
                center.to_array(),
                body.orientation,
                json!({"kind":"sphere","radius":rest.radius*SCALE}),
                color,
            );
            rod(
                &mut v,
                id + 2,
                &format!("{label}_{name}_to_hinge"),
                center,
                project(anchor[i]),
                local_anchor.length() * SCALE,
                color,
            );
            // Two fixed gauges indicate per-link mean normal force over this 1 ms step.
            // F/(F+weight) preserves the impact and static support on one finite scale.
            let force = contacts[i].normal_impulse * 1000.;
            let x = -1.98 + i as f64 * 0.30;
            v.cuboid(
                id + 4,
                &format!("{label}_{name}_reaction_scale"),
                [x, y + 0.65, 0.61],
                [0.045, 0.045, 1.2],
                GRAY,
            );
            v.add(
                id + 6,
                &format!("{label}_{name}_normal_force"),
                [x, y + 0.65, 0.01 + 1.2 * force / (weight + force)],
                ID,
                json!({"kind":"sphere","radius":0.085}),
                color,
            );
        }
        if (anchor[0] - anchor[1]).length() > 1e-8 {
            return Err(format!("hinge anchor separation at {}", state.tick).into());
        }
        v.add(
            start + 26,
            &format!("{label}_hinge"),
            project(anchor[0]).to_array(),
            ID,
            json!({"kind":"sphere","radius":0.08}),
            GOLD,
        );
    }
    let x = |slot: u8| (f64::from(slot) - f64::from(width - 1) * 0.5) * 0.38;
    for slot in 0..width {
        v.cuboid(
            140 + 2 * u64::from(slot),
            &format!("worker_{slot}"),
            [x(slot), -2.45, 0.08],
            [0.29, 0.22, 0.13],
            GRAY,
        );
    }
    v.add(
        160,
        "active_worker",
        [x(owner), -2.45, 0.25],
        ID,
        json!({"kind":"sphere","radius":0.12}),
        GREEN,
    );
    if let Some(kind) = event {
        let color = match kind {
            "transfer" => GOLD,
            "kill" => [0.95, 0.12, 0.09, 1.],
            "replaced" => BLUE,
            "resume" => [0.69, 0.38, 0.94, 1.],
            _ => return Err("unknown event".into()),
        };
        v.cuboid(
            1000 + worker.tick * 2,
            &format!("{kind}_at_{}_ms", worker.tick),
            [x(owner), -2.76, 0.18],
            [0.19, 0.19, 0.29],
            color,
        );
    }
    Ok(v.finish_at(ordinal, worker.tick * 1_000_000))
}
