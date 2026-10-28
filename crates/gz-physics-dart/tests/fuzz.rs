// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Deterministic search for inputs that abort the packaged DART. Each seeded
//! trial runs in its own child process, so an engine abort fails this test with
//! the seed instead of killing the runner. Trials combine rotated, anisotropic,
//! offset bodies at and beyond the admission limits, resting on a floor and on
//! each other, optional hinges, and random wrenches, velocities, impulses and
//! step sizes. Every admitted input must step without aborting.

use gz_physics::{BodyId, BodySpec, DynamicBackend, Inertia, RotationSpec, Shape};
use gz_physics_dart::DartBackend;
use std::{process::Command, thread, time::Duration};

const TRIALS: u64 = 500;
const WORKERS: u64 = 8;
const SEED: &str = "GZ_PHYSICS_DART_FUZZ_SEED";
const K: f64 = 1e5;

/// xorshift64, so every seed reproduces the same trial.
struct Random(u64);

impl Random {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn pick<T: Copy>(&mut self, values: &[T]) -> T {
        values[(self.next() % values.len() as u64) as usize]
    }

    fn unit(&mut self) -> [f64; 3] {
        loop {
            let v = [0, 1, 2].map(|_| (self.next() % 2001) as f64 / 1000.0 - 1.0);
            let length = v.iter().map(|x| x * x).sum::<f64>().sqrt();
            if length > 0.1 {
                return v.map(|x| x / length);
            }
        }
    }

    fn vector(&mut self, magnitudes: &[f64]) -> [f64; 3] {
        let magnitude = self.pick(magnitudes);
        self.unit().map(|v| v * magnitude)
    }

    fn quaternion(&mut self) -> [f64; 4] {
        let axis = self.unit();
        let angle = (self.next() % 628) as f64 / 100.0;
        let s = (angle / 2.0).sin();
        [axis[0] * s, axis[1] * s, axis[2] * s, (angle / 2.0).cos()]
    }
}

/// A shape and the distance from its origin to its farthest point.
fn shape(random: &mut Random) -> (Shape, f64) {
    let size = [1e-3, 0.05, 0.5, 2.0, 50.0];
    let (a, b, c) = (random.pick(&size), random.pick(&size), random.pick(&size));
    match random.next() % 4 {
        0 => (Shape::Sphere { radius: a }, a),
        1 => (
            Shape::Box {
                half_extents: [a, b, c],
            },
            (a * a + b * b + c * c).sqrt(),
        ),
        2 => (
            Shape::Cylinder {
                radius: a,
                half_height: b,
            },
            a.hypot(b),
        ),
        _ => (
            Shape::Capsule {
                radius: a,
                half_length: b,
            },
            a + b,
        ),
    }
}

#[derive(Default)]
struct Outcome {
    admitted: usize,
    hinges: usize,
    steps: usize,
    refused: usize,
    /// Accepted steps of trials with a body placed on the floor or a stack.
    resting_steps: usize,
}

fn trial(seed: u64) -> Outcome {
    let mut random = Random(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
    let mut outcome = Outcome::default();
    let gravity = random.vector(&[0.0, 9.81, 9.81, 1e3, 1e30, 0.999_999e60]);
    let gravity = if random.next().is_multiple_of(2) {
        [0.0, 0.0, -9.81]
    } else {
        gravity
    };
    let mut backend = DartBackend::new(gravity).unwrap();
    let floor = !random.next().is_multiple_of(3);
    if floor {
        let ground = BodySpec {
            shape: Shape::Box {
                half_extents: [100.0, 100.0, 0.5],
            },
            position: [0.0, 0.0, -0.5],
            velocity: [0.0; 3],
            mass: None,
        };
        backend
            .spawn_link(ground, RotationSpec::default(), None)
            .unwrap();
    }
    let mut ids: Vec<BodyId> = Vec::new();
    let mut top = 0.0;
    let mut resting = false;
    for index in 0..1 + random.next() % 4 {
        let (shape, reach) = shape(&mut random);
        let center = random.vector(&[0.0, 0.0, 1e-3, 0.1, 1.0, 30.0, 1e3]);
        let lever = center.iter().map(|v| v * v).sum::<f64>().sqrt() + reach;
        // Moments from about the conditioning limit upward, with anisotropy
        // on both sides of the 1e8 cap; triangle-consistent.
        let smallest =
            3.0 * lever * lever / K * random.pick(&[1.0, 1.0, 1.5, 10.0, 1e3, 1e6, 1e-3]);
        let ratio = random.pick(&[1.0, 1.0, 10.0, 1e3, 1e7, 0.99999e8, 1.00001e8, 1e60]);
        let middle = smallest * ratio;
        let largest = smallest.max(middle);
        // A quarter of bodies take shape-derived inertia, slender ones included.
        let derived = random.next().is_multiple_of(4);
        let mass = random.pick(&[
            4.0 / K,
            4.0 / K,
            1e-4,
            0.1,
            1.0,
            10.0,
            1e3,
            1e30,
            1e60,
            0.5 / K,
        ]);
        // Rest on the floor, stack on the previous body, or sit beside it.
        let placement = random.next() % 3;
        let position = match placement {
            0 => [
                index as f64 * reach * 1.9,
                0.0,
                reach * random.pick(&[0.9, 1.0, 1.5]),
            ],
            1 => {
                let z = top + reach;
                [0.01 * index as f64, 0.0, z]
            }
            _ => [index as f64 * 0.3, 0.0, 2.0 + index as f64],
        };
        let velocity = random.vector(&[0.0, 0.0, 1.0, 1e3, 1e30, 1e59]);
        let angular = random.vector(&[0.0, 0.0, 1.0, 1e3, 1e30, 1e59]);
        let spawned = backend.spawn_link(
            BodySpec {
                shape,
                position,
                velocity,
                mass: Some(mass),
            },
            RotationSpec {
                orientation: random.quaternion(),
                angular_velocity: angular,
            },
            (!derived).then(|| Inertia {
                center,
                orientation: random.quaternion(),
                tensor: [smallest, middle, largest, 0.0, 0.0, 0.0],
            }),
        );
        if let Ok(id) = spawned {
            ids.push(id);
            resting |= floor && placement < 2;
            top = position[2] + reach;
            outcome.admitted += 1;
            if random.next().is_multiple_of(3) {
                let offset = random.vector(&[0.0, reach, 3.0 * reach, 1e3]);
                let anchor = [0, 1, 2].map(|i| position[i] + offset[i]);
                let axis = random.unit();
                if backend.add_world_revolute(id, anchor, axis).is_ok() {
                    outcome.hinges += 1;
                }
            }
        }
    }
    let steps = [
        Duration::from_millis(1),
        Duration::from_millis(1),
        Duration::from_millis(1),
        Duration::from_millis(1),
        Duration::from_millis(10),
        Duration::from_secs(1),
        Duration::from_secs(1_000_000),
        Duration::from_secs(u64::MAX),
    ];
    let magnitudes = [0.0, 1.0, 1e3, 1e30, 1e60];
    for _ in 0..40 {
        for &id in &ids {
            let (linear, angular) = (random.vector(&magnitudes), random.vector(&magnitudes));
            // Refused commands leave the body unchanged; the step must survive.
            let _ = match random.next() % 5 {
                0 => backend.add_world_wrench(id, linear, angular),
                1 => backend.set_world_velocity(id, linear, angular),
                2 => backend.apply_impulse(id, linear, angular),
                _ => Ok(()),
            };
        }
        match backend.step(random.pick(&steps)) {
            Ok(()) => {
                outcome.steps += 1;
                outcome.resting_steps += usize::from(resting);
            }
            Err(_) => outcome.refused += 1,
        }
        for &id in &ids {
            let state = backend.state(id).unwrap();
            assert!(
                state
                    .position
                    .iter()
                    .chain(&state.velocity)
                    .chain(&state.orientation)
                    .chain(&state.angular_velocity)
                    .all(|v| v.is_finite())
            );
        }
    }
    outcome
}

/// Runs one seed when invoked as a child; otherwise does nothing.
#[test]
fn fuzz_child_trial() {
    if let Some(seed) = std::env::var_os(SEED) {
        let seed: u64 = seed.to_str().unwrap().parse().unwrap();
        let outcome = trial(seed);
        println!(
            "OUTCOME {} {} {} {} {}",
            outcome.admitted, outcome.hinges, outcome.steps, outcome.refused, outcome.resting_steps
        );
    }
}

#[test]
fn admitted_random_inputs_never_abort_the_engine() {
    let exe = std::env::current_exe().unwrap();
    let workers: Vec<_> = (0..WORKERS)
        .map(|worker| {
            let exe = exe.clone();
            thread::spawn(move || {
                let mut failures = Vec::new();
                let mut totals = [0usize; 5];
                for seed in (worker..TRIALS).step_by(WORKERS as usize) {
                    let output = Command::new(&exe)
                        .args(["--exact", "fuzz_child_trial", "--nocapture"])
                        .env(SEED, seed.to_string())
                        .output()
                        .unwrap();
                    let stdout = String::from_utf8_lossy(&output.stdout);
                    match stdout.lines().find_map(|l| l.strip_prefix("OUTCOME ")) {
                        Some(line) if output.status.success() => {
                            for (total, value) in totals.iter_mut().zip(line.split(' ')) {
                                *total += value.parse::<usize>().unwrap();
                            }
                        }
                        _ => {
                            let stderr = String::from_utf8_lossy(&output.stderr);
                            let tail: String =
                                stderr.lines().rev().take(3).collect::<Vec<_>>().join(" | ");
                            failures.push(format!("seed {seed}: {:?} {tail}", output.status));
                        }
                    }
                }
                (failures, totals)
            })
        })
        .collect();
    let mut failures = Vec::new();
    let mut totals = [0usize; 5];
    for worker in workers {
        let (more, counts) = worker.join().unwrap();
        failures.extend(more);
        for (total, count) in totals.iter_mut().zip(counts) {
            *total += count;
        }
    }
    let [admitted, hinges, steps, refused, resting] = totals;
    println!(
        "{TRIALS} trials: {admitted} bodies, {hinges} hinges, {steps} steps \
         ({resting} with bodies on the floor), {refused} refused"
    );
    assert!(failures.is_empty(), "{failures:#?}");
    // The search must exercise the engine, not only admission: contact steps
    // with bodies on the floor must succeed, not just free flight.
    assert!(admitted as u64 > TRIALS && hinges > 20 && steps as u64 > TRIALS * 10);
    assert!(resting as u64 > TRIALS, "{resting} resting steps");
}

/// Ordinary scenes: 0.1-10 kg spheres, boxes, cylinders and capsules with
/// shape-derived inertia resting in rows and stacks on a floor, nudged gently.
/// In-bound scenes like these must never be refused.
#[test]
fn ordinary_resting_scenes_are_never_refused() {
    let mut steps = 0;
    for seed in 1..=40u64 {
        let mut random = Random(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
        let mut backend = DartBackend::new([0.0, 0.0, -9.81]).unwrap();
        let ground = BodySpec {
            shape: Shape::Box {
                half_extents: [50.0, 50.0, 0.5],
            },
            position: [0.0, 0.0, -0.5],
            velocity: [0.0; 3],
            mass: None,
        };
        backend
            .spawn_link(ground, RotationSpec::default(), None)
            .unwrap();
        let mut top = 0.0;
        let mut ids = Vec::new();
        for index in 0..2 + random.next() % 5 {
            let size = [0.05, 0.1, 0.2, 0.3].map(|v| v + 0.01 * (seed % 5) as f64);
            let (a, b) = (random.pick(&size), random.pick(&size));
            let (shape, height) = match random.next() % 4 {
                0 => (Shape::Sphere { radius: a }, a),
                1 => (
                    Shape::Box {
                        half_extents: [a, b, a],
                    },
                    a,
                ),
                2 => (
                    Shape::Cylinder {
                        radius: a,
                        half_height: b,
                    },
                    b,
                ),
                _ => (
                    Shape::Capsule {
                        radius: a,
                        half_length: b,
                    },
                    a + b,
                ),
            };
            let stacked = random.next().is_multiple_of(2);
            let position = if stacked {
                [0.01 * index as f64, 0.0, top + height + 0.001]
            } else {
                [index as f64 * 1.5, 3.0, height + 0.001]
            };
            if stacked {
                top = position[2] + height;
            }
            let id = backend
                .spawn_link(
                    BodySpec {
                        shape,
                        position,
                        velocity: random.vector(&[0.0, 0.2, 0.5]),
                        mass: Some(random.pick(&[0.1, 0.5, 1.0, 3.0, 10.0])),
                    },
                    RotationSpec {
                        orientation: [0.0, 0.0, 0.0, 1.0],
                        angular_velocity: random.vector(&[0.0, 0.5, 1.0]),
                    },
                    None,
                )
                .unwrap();
            ids.push(id);
        }
        for step in 0..1000 {
            let result = backend.step(Duration::from_millis(1));
            assert!(result.is_ok(), "seed {seed} step {step}: {result:?}");
            steps += 1;
        }
    }
    assert_eq!(steps, 40_000);
}
