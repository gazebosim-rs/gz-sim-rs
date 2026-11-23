// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Grazing contacts previously missed while the two native controls agreed bitwise.
#[path = "../examples/support/seeded.rs"]
mod seeded;
#[path = "../examples/support/train_fixture.rs"]
mod train_fixture;

use gz_physics_distributed::{
    coordinator::Coordinator,
    physics::{Body, Island, SimulationStep, Snapshot},
    transport::WorkerProcess,
};
use rapier3d_f64::prelude::Vector;
use std::path::Path;

fn left_momentum(bodies: &[Body]) -> Vector {
    bodies
        .iter()
        .filter(|b| b.id <= 2)
        .map(|b| Vector::from(b.linear) * b.mass)
        .sum()
}

fn check_encounter(seed: u64, miss_track: bool) {
    let (mut bodies, loads) = train_fixture::train_fixture(seed);
    if miss_track {
        // Displace the entire right train to a distinct parallel track.
        for body in &mut bodies[2..] {
            body.position[1] += 12.;
        }
    }
    let worker = Path::new(env!("CARGO_BIN_EXE_physics_worker"));
    let step = SimulationStep::Ms1;
    let dt = step.duration().as_secs_f64();
    let mut last_momentum = left_momentum(&bodies);
    let peers = [
        WorkerProcess::spawn_with_step(worker, seed, 0, &bodies[..2], step).unwrap(),
        WorkerProcess::spawn_with_step(worker, seed, 1, &bodies[2..], step).unwrap(),
    ];
    let mut pool = Coordinator::new(seed, peers, 1024).unwrap();
    pool.enable_rolling_journals();
    let mut native = Island::restore(&Snapshot {
        bodies,
        step,
        ..Default::default()
    })
    .unwrap();
    let mut peak_exchange = 0f64;
    let mut peak_report = 0f64;
    for tick in 1..=2800 {
        let distributed = pool.advance_in_field_overlapped([0.; 3], &loads).unwrap();
        native.step_with_wrenches(&loads).unwrap();
        let reference = native.snapshot();
        for body in &reference.bodies {
            assert_eq!(
                distributed.body(body.id).unwrap().1,
                body,
                "seed {seed}, tick {tick}"
            );
        }
        let mut pairs: Vec<_> = distributed
            .workers
            .iter()
            .flat_map(|w| &w.snapshot.impulses)
            .collect();
        pairs.sort_by_key(|p| (p.a, p.b));
        assert!(
            pairs.iter().copied().eq(&reference.impulses),
            "seed {seed}, tick {tick}"
        );
        let p = left_momentum(&reference.bodies);
        let exchange = (p - last_momentum - Vector::from(loads[0].force) * dt).length();
        peak_exchange = peak_exchange.max(exchange);
        last_momentum = p;
        for pair in pairs.iter().filter(|p| p.a <= 2 && p.b >= 3) {
            peak_report = peak_report.max(pair.normal_impulse);
        }
    }
    if miss_track {
        assert_eq!(peak_report, 0.);
        assert!(peak_exchange < 1e-12, "{seed}: {peak_exchange}");
    } else {
        // Each regression has a measurable frictionless impact above 0.5 N s.
        assert!(peak_report > 0.5, "{seed}: {peak_report}");
        assert!(
            (peak_exchange - peak_report).abs() < 1e-12,
            "{seed}: {peak_exchange} vs {peak_report}"
        );
    }
}

#[test]
fn millisecond_workers_resolve_the_three_grazing_encounters() {
    for seed in [20261161424, 20261184268, 20261199941] {
        check_encounter(seed, false);
    }
}

#[test]
fn displaced_trains_exchange_no_cross_pair_momentum() {
    check_encounter(20261161424, true);
}

#[test]
fn box_forecast_refuses_a_sphere_even_with_cubic_bounds() {
    let mut bodies = gz_physics_distributed::physics::fixture();
    bodies[1].primitive = gz_physics_distributed::physics::Primitive::Sphere;
    for (a, b) in [(0, 1), (1, 0)] {
        assert!(matches!(
            bodies[a].predicts_box_contact(&bodies[b], SimulationStep::Ms10),
            Err(gz_physics::PhysicsError::Unsupported { .. })
        ));
    }
}
