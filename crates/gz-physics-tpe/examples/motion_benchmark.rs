// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::BodyState;
use gz_physics_tpe::{Body, MotionError};
use std::{
    hint::black_box,
    time::{Duration, Instant},
};

fn main() -> Result<(), MotionError> {
    let mut bodies = (0..1000)
        .map(|i| {
            Body::try_from(BodyState {
                position: [i as f64 * 0.001, 2.0, 3.0],
                orientation: [0.0, 0.0, 0.0, 1.0],
                velocity: [1.0, 0.5, 0.25],
                angular_velocity: [0.1, 0.2, 0.3],
            })
        })
        .collect::<Result<Vec<_>, _>>()?;
    let start = Instant::now();
    for _ in 0..1000 {
        for body in black_box(&mut bodies) {
            body.advance(Duration::from_millis(1))?;
        }
    }
    let nanos = start.elapsed().as_nanos();
    let mut checksum = 0.0;
    for body in &bodies {
        for value in body.state().position {
            checksum += value;
        }
        checksum += body.state().orientation[3];
    }
    println!(
        "{{\"nanos\":{nanos},\"checksum\":{checksum:.17},\"position\":{:?},\"orientation\":{:?}}}",
        bodies[0].state().position,
        bodies[0].state().orientation
    );
    Ok(())
}
