// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isolate the pinned native solver's response to distant, uncoupled contacts.
use super::grid_control::{self, Control, Pairs, Supports};
use gz_physics_distributed::physics::{Body, fixture};
use std::{error::Error, time::Duration};

pub(super) fn run(ticks: usize) -> Result<(), Box<dyn Error>> {
    let h = Duration::from_millis(5);
    let target = vec![
        Body {
            id: 0,
            position: [0., 0., 0.64],
            size: [0.55, 0.55, 0.4],
            linear: [0.; 3],
            angular: [0.; 3],
            mass: 1.,
            friction: 1.,
            ..fixture()[0].promoted()
        },
        Body {
            id: 1,
            position: [0., 0., 2.14],
            size: [0.55, 0.55, 0.4],
            linear: [0.; 3],
            angular: [0.; 3],
            mass: 2.,
            friction: 1.,
            ..fixture()[0].promoted()
        },
    ];
    println!(
        "remote_boxes,remote_in_contact,first_target_difference,ticks_exact,peak_position_m,peak_velocity_m_s,peak_angle_rad,peak_spin_rad_s,peak_floor_Ns,peak_pair_Ns"
    );
    for quiet in [false, true] {
        for n in [
            0, 30, 60, 100, 120, 122, 123, 124, 125, 126, 127, 128, 129, 150, 254,
        ] {
            let mut world = target.clone();
            for i in 0..n {
                world.push(Body {
                    id: (i + 2) as u64,
                    position: [
                        8. + 8. * (i % 16) as f64,
                        8. * (i / 16) as f64,
                        if quiet { 10000. } else { 0.64 },
                    ],
                    ..target[0].clone()
                });
            }
            let mut observed = Control::new(&world, h)?;
            let mut reference = Control::new(&target, h)?;
            let mut first = 0;
            let mut matches = 0;
            let mut peak = [0f64; 6];
            for tick in 1..=ticks {
                observed.step(h)?;
                reference.step(h)?;
                let mut exact = true;
                for (i, seed) in target.iter().enumerate() {
                    let state = observed.state(i);
                    let b = Body {
                        position: state.position,
                        orientation: state.orientation,
                        linear: state.velocity,
                        angular: state.angular_velocity,
                        ..seed.clone()
                    };
                    let c = reference.state(i);
                    exact &= grid_control::exact(&b, c);
                    for (p, e) in peak.iter_mut().zip(grid_control::state_error(&b, c)) {
                        *p = p.max(e);
                    }
                }
                let mut native_pairs = Pairs::new();
                let mut native_ground = Supports::new();
                let mut reference_pairs = Pairs::new();
                let mut reference_ground = Supports::new();
                observed.contacts(&mut native_pairs, &mut native_ground);
                reference.contacts(&mut reference_pairs, &mut reference_ground);
                native_ground.retain(|id, _| *id < 2);
                assert!(native_pairs.keys().all(|k| k == &(0, 1)));
                for (p, (same, e)) in peak[4..].iter_mut().zip([
                    grid_control::impulses(&native_ground, &reference_ground),
                    grid_control::impulses(&native_pairs, &reference_pairs),
                ]) {
                    exact &= same;
                    *p = p.max(e);
                }
                if exact {
                    matches += 1;
                } else if first == 0 {
                    first = tick;
                }
            }
            println!(
                "{n},{},{first},{matches},{:.10e},{:.10e},{:.10e},{:.10e},{:.10e},{:.10e}",
                !quiet, peak[0], peak[1], peak[2], peak[3], peak[4], peak[5]
            );
        }
    }
    Ok(())
}
