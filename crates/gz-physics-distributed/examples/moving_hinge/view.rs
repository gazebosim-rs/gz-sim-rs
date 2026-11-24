// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::hinge_rig::{self, BLUE, GOLD, GRAY, ID, MAGENTA, ORANGE};
use super::*;
use primitive_replay_view::View;
const GREEN: [f64; 4] = [0.15, 0.87, 0.56, 1.];

pub(super) fn draw(
    spec: &MovingSpec,
    direct: &MovingObservation,
    worker: &MovingObservation,
    owners: [u8; 2],
    width: u8,
    event: Option<&str>,
    ordinal: u64,
) -> Result<(String, String)> {
    let mut v = View::default();
    hinge_rig::draw(&mut v, spec, direct, "uninterrupted_SDF", 10, 1.18);
    hinge_rig::draw(&mut v, spec, worker, "distributed", 60, -1.18);
    let x = |n: u8| (f64::from(n) - f64::from(width - 1) * 0.5) * 0.45;
    for n in 0..width {
        v.cuboid(
            150 + u64::from(n) * 2,
            &format!("worker_{n}"),
            [x(n), -2.6, 0.12],
            [0.33, 0.58, 0.14],
            GRAY,
        );
    }
    for (i, (owner, label, color)) in [
        (owners[0], "articulation_owner", BLUE),
        (owners[1], "sphere_owner", MAGENTA),
    ]
    .into_iter()
    .enumerate()
    {
        v.add(
            178 + 2 * i as u64,
            label,
            [x(owner), -2.74 + i as f64 * 0.28, 0.30],
            ID,
            json!({"kind":"sphere","radius":0.115}),
            color,
        );
    }
    if let Some(e) = event {
        let color = match e {
            "join" => GREEN,
            "split" => BLUE,
            "loaded_handoff" => GOLD,
            "released_resume" => [0.70, 0.35, 0.95, 1.],
            "owner_kill" => [0.9, 0.15, 0.1, 1.],
            _ => ORANGE,
        };
        v.cuboid(
            1000 + 2 * worker.hinge.tick,
            &format!("{e}_at_{}_ms", worker.hinge.tick),
            [x(owners[0]), -3.12, 0.16],
            [0.2, 0.2, 0.24],
            color,
        );
    }
    Ok(v.finish_at(ordinal, worker.hinge.tick * 1_000_000))
}
