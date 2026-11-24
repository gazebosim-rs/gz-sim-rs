// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::time::Instant;

/// Each row starts with a fresh SDF world. Durations surround step_exact only.
/// Comparisons, recording, scene loading and schedule initialization are untimed.
#[derive(Serialize)]
pub(super) struct Timing<'a> {
    trial: &'a Trial,
    native_calls_per_tick: Vec<u32>,
    step_ns_by_run: Vec<Vec<u128>>,
}

fn repeat(reference: &Run, xml: &str) -> Result<Vec<u128>> {
    let trial = &reference.trial;
    let (mut sim, bodies, _) = load_robot(trial, xml)?;
    sim.initialize_schedules()?;
    let calls = trial.substeps;
    let first = trial.first_native_steps.unwrap_or(calls);
    if (calls > 1 || first > 1) && !trial.automatic {
        // Match the observed fine reference without collecting every native
        // manifold for analysis inside its clocked interval.
        assert_eq!(
            sim.remove_systems_in_set(Phase::PhysicsStep, Phase::PhysicsStep)?,
            1
        );
        sim.add_systems(
            Phase::PhysicsStep,
            move |clock: Res<UpdateInfo>,
                  mut backend: ResMut<PhysicsBackend>,
                  fault: Res<StepFault>| {
                let native = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
                if let Err(error) = native.step_contact_interval(
                    clock.dt,
                    if clock.start_time.is_zero() {
                        first
                    } else {
                        calls
                    },
                    native.ordinary_contact_iterations(),
                ) {
                    fault.report(error);
                }
            },
        );
        sim.initialize_schedules()?;
    }
    assert_eq!(states(&sim, &bodies), reference.incoming);
    let mut times = Vec::with_capacity(reference.samples.len());
    for expected in &reference.samples {
        let start = Instant::now();
        sim.step_exact(1)?;
        times.push(start.elapsed().as_nanos());
        assert_eq!(sim.elapsed(), Duration::from_millis(u64::from(expected.ms)));
        assert_eq!(states(&sim, &bodies), expected.bodies);
        if trial.automatic {
            let actual = gz_sim_scene::primary_contact::last_refinement(&sim)
                .filter(|r| r.sim_time == sim.elapsed())
                .map_or(1, |r| r.interval.native_steps);
            assert_eq!(actual, expected.native_steps);
        }
    }
    Ok(times)
}

pub(super) fn measure(reference: &Run, count: u32) -> Result<Timing<'_>> {
    let xml = fixture(&reference.trial);
    repeat(reference, &xml)?;
    let step_ns_by_run = (0..count)
        .map(|_| repeat(reference, &xml))
        .collect::<Result<_>>()?;
    Ok(Timing {
        trial: &reference.trial,
        native_calls_per_tick: reference.samples.iter().map(|s| s.native_steps).collect(),
        step_ns_by_run,
    })
}
