// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "landing-audit")]
use gz_physics::{BodyId, BodySpec, BodyState, DynamicBackend, PhysicsError, Shape};
use gz_physics_multifidelity::{LandingRowOrder, MultiFidelityBackend, ReturnFeatureMode};
use std::time::Duration;

const DT: Duration = Duration::from_millis(1);
fn cube(z: f64, half: [f64; 3], vz: f64) -> BodySpec {
    BodySpec {
        shape: Shape::Box { half_extents: half },
        mass: Some(1.),
        position: [0., 0., z],
        velocity: [0., 0., vz],
    }
}

fn approaching(oriented: bool) -> Result<(MultiFidelityBackend, BodyId, BodyId), PhysicsError> {
    let mut w = MultiFidelityBackend::new([0., 0., -9.81])?;
    let foot = w.spawn_articulation_link(
        cube(0.123, [0.12, 0.08, 0.1], -0.5),
        Default::default(),
        None,
    )?;
    let upper = w.spawn_articulation_link(cube(0.38, [0.08; 3], -0.5), Default::default(), None)?;
    w.add_limited_revolute(Some(foot), upper, [0., 0., 0.25], [0., 1., 0.], [0.; 2])?;
    w.enable_exact_revolute_locks(foot)?;
    let surface = cube(-0.1, [0.6, 0.6, 0.1], 0.05);
    let pad = if oriented {
        w.spawn_ideal_landing_surface(surface, Default::default())?
    } else {
        w.spawn_ideal_landing_pad(surface)?
    };
    w.step(DT)?;
    Ok((w, foot, pad))
}

fn assert_motion_close(actual: BodyState, warm: BodyState) {
    for (a, b) in actual
        .position
        .into_iter()
        .chain(actual.velocity)
        .chain(actual.orientation)
        .chain(actual.angular_velocity)
        .zip(
            warm.position
                .into_iter()
                .chain(warm.velocity)
                .chain(warm.orientation)
                .chain(warm.angular_velocity),
        )
    {
        assert!((a - b).abs() < 1e-8);
    }
}

#[test]
fn detached_landing_audits_preserve_the_live_cut_and_predict_positive_ground_reaction()
-> Result<(), PhysicsError> {
    let (w, foot, pad) = approaching(false)?;
    let initial = w.airborne_restart_state(foot, pad)?;
    assert!(initial.history.iter().all(|p| p.absolute_seeds == [0.; 3]));
    let mobility = w.landing_normal_mobility(foot, [[0.; 3]; 4])?;
    assert!((mobility[0][0] - 0.5).abs() < 1e-8);
    let mut donor = w.copy_airborne_landing(foot, pad, false)?;
    let mut cold = w.copy_airborne_landing(foot, pad, true)?;
    let mut fork = w.landing_fork(Some(&donor), foot, pad, DT)?;
    let prediction = w.landing_trial(Some(&donor), foot, pad, 64, DT, 0.)?;
    assert_eq!(prediction.initial, initial.links);
    let mut impulse = 0.;
    for _ in 0..64 {
        fork.step_outer(1, None, |f| {
            impulse += f.normal_impulse();
        })?;
        cold.step(DT)?;
        donor.step(DT)?;
    }
    assert!(impulse > 0.8);
    let warm = donor.state(foot).unwrap();
    assert_motion_close(cold.state(foot).unwrap(), warm);
    assert_eq!(impulse, prediction.impulses.iter().sum::<f64>());
    assert_eq!(fork.states(), prediction.final_states);
    assert_motion_close(fork.states()[0], warm);
    assert_eq!(w.state(foot), Some(initial.links[0]));
    assert!(w.landing_fork(None, foot, pad, Duration::ZERO).is_err());
    Ok(())
}

#[test]
fn observed_and_timed_controls_step_identical_airborne_motion_and_keep_the_solver_budget()
-> Result<(), PhysicsError> {
    let (mut a, foot, pad) = approaching(true)?;
    let mut b = a.copy_airborne_landing(foot, pad, false)?;
    let it = a.ordinary_contact_iterations();
    let pair = (foot, pad);
    let (receipt, _) =
        a.observe_landing_row_order(pair, DT, 2, it, LandingRowOrder::Unchanged, |_| {})?;
    let (timed, _) = b.step_landing_row_order(pair, DT, 2, it, LandingRowOrder::Unchanged)?;
    assert_eq!(receipt.native_steps, timed.native_steps);
    assert_eq!(a.state(foot), b.state(foot));
    let pg = a.contact_pgs_passes();
    a.step_landing_with_relaxation(DT, 2)?;
    a.step_landing_with_iterations(DT, 6)?;
    a.step_landing_response_control(DT, 1, false, false, false)?;
    a.step_landing_with_fresh_box_pairs(DT, 1, &[pair])?;
    a.step_returning_box_features(pair, DT, 2, it, ReturnFeatureMode::FirstStale, None)?;
    assert_eq!(pg, a.contact_pgs_passes());
    for _ in 0..100 {
        a.step(DT)?;
    }
    assert!(!a.landing_solver_points(foot, pad)?.is_empty());
    assert!(a.state(foot).unwrap().position[2] > 0.09);
    Ok(())
}
