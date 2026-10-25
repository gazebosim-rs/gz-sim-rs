// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sensors::ImuReading;
use gz_sim_core::{Simulation, StepError};
use gz_sim_scene::{ImuSample, PhysicsEngine, load_world_with_engine, reset_loaded_world};
use sdformat::NoResources;
use std::time::Duration;

fn sample(sim: &Simulation) -> Option<(Duration, u64, ImuReading)> {
    sim.world().iter_entities().find_map(|entity| {
        let value = entity.get::<ImuSample>()?;
        Some((value.time, value.epoch, *value.reading.as_ref().unwrap()))
    })
}

#[test]
fn native_rates_preserve_phase_derivative_pause_and_reset_for_both_engines() {
    let fixture = include_str!("fixtures/fall.sdf");
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for (rate, expected) in [
            (0, (1..=100).collect::<Vec<_>>()),
            (10, vec![1, 100]),
            (60, vec![1, 17, 34, 50, 67, 84, 100]),
            (1000, (1..=100).collect()),
            (2000, (1..=100).collect()),
        ] {
            let xml = fixture.replace(
                "<update_rate>1000</update_rate>",
                &format!("<update_rate>{rate}</update_rate>"),
            );
            let mut sim = load_world_with_engine(
                &xml,
                "schedule.sdf",
                "default",
                Duration::from_millis(1),
                &NoResources,
                engine,
            )
            .unwrap();
            let mut observed = Vec::new();
            let mut previous = None;
            for _ in 0..100 {
                sim.step_exact(1).unwrap();
                let (time, epoch, reading) = sample(&sim).unwrap();
                if previous != Some(time) {
                    observed.push(time.as_millis());
                    assert_eq!(epoch, 0);
                    let expected_force = if engine == PhysicsEngine::Tpe {
                        9.81
                    } else {
                        0.0
                    };
                    assert!((reading.specific_force[2] - expected_force).abs() < 1e-10);
                    assert_eq!(reading.angular_velocity, [0.0; 3]);
                }
                previous = Some(time);
            }
            assert_eq!(observed, expected, "{engine:?} {rate}Hz");
            let before = sample(&sim);
            sim.set_paused(true);
            sim.tick().unwrap();
            assert_eq!(sample(&sim), before);
            assert_eq!(sim.elapsed(), Duration::from_millis(100));
            sim.set_paused(false);
            reset_loaded_world(&mut sim, |_| ()).unwrap();
            assert!(sample(&sim).is_none());
            sim.step_exact(1).unwrap();
            let (time, epoch, _) = sample(&sim).unwrap();
            assert_eq!(time, Duration::from_millis(1));
            assert_eq!(epoch, 1);
            sim.step_exact(1).unwrap();
            assert_eq!(
                sample(&sim).unwrap().0,
                Duration::from_millis(if rate == 10 || rate == 60 { 1 } else { 2 })
            );
        }
    }
}

#[test]
fn unrepresentable_next_sample_is_a_terminal_fault_without_a_new_sample() {
    let mut sim = load_world_with_engine(
        include_str!("fixtures/fall.sdf"),
        "overflow.sdf",
        "default",
        Duration::MAX,
        &NoResources,
        PhysicsEngine::Tpe,
    )
    .unwrap();
    let error = sim.step_exact(1).unwrap_err();
    assert_eq!(error, StepError::Faulted);
    assert!(
        sim.fault()
            .unwrap()
            .to_string()
            .contains("sensor sampling time overflow")
    );
    assert!(sample(&sim).is_none());
    assert_eq!(sim.elapsed(), Duration::ZERO);
    assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
}
