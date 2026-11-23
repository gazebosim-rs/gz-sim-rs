// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/inclined_sheet.rs"]
mod sheet;
#[allow(dead_code)]
#[path = "../examples/support/slip_transition.rs"]
mod transition;
use sheet::Incline;
use transition::{Run, run};
fn fixture() -> Run {
    Run {
        speed: -0.42,
        mu: 0.15,
        width: 250000,
        transition: None,
        yaw: 0.,
        extra_mass: 0.,
    }
}
#[test]
fn local_transition_subdivision_tracks_the_analytic_kinetic_reversal_in_both_world_directions() {
    for (speed, mu, yaw) in [
        (-0.42, 0.15, 0.),
        (-0.42, 0.15, std::f64::consts::PI),
        (-0.37, 0.12, -0.8),
        (-0.2, 0.23, 1.1),
    ] {
        let source = Run {
            speed,
            mu,
            yaw,
            ..fixture()
        };
        let base = run(source, 330).unwrap();
        let fine = run(
            Run {
                transition: Some(2000),
                ..source
            },
            330,
        )
        .unwrap();
        let (lo, hi) = fine.crossing.unwrap();
        let exact = -speed / sheet::acceleration(mu, -1.) * 1e9;
        assert!(
            hi - lo <= 2000 && lo as f64 <= exact && hi as f64 >= exact,
            "actual {lo}–{hi}, analytic {exact}"
        );
        assert!(fine.worst_v < base.worst_v / 12. && fine.worst_x < base.worst_x / 12.);
        assert!(
            fine.slip_refinements > 0 && fine.slip_refinements < 100 && fine.cuts <= base.cuts + 10
        );
        assert!(fine.p < 2e-13 && fine.l < 2e-13);
        if speed == -0.42 {
            assert_eq!(
                fine.slip_refinements, 1,
                "coherent incoming sites should supply one reusable time proposal"
            );
            assert!(hi - lo <= 1000);
        }
        println!(
            "v={speed} μ={mu} yaw={yaw} radians, coarse x/v errors {:e} / {:e}, adaptive {:e} / {:e}, analytic {exact} ns in [{lo},{hi}], {} / {} accepted cuts, {} observed rejected turn trials",
            base.worst_x,
            base.worst_v,
            fine.worst_x,
            fine.worst_v,
            base.cuts,
            fine.cuts,
            fine.slip_refinements
        );
    }
}
#[test]
fn sustained_stick_and_unidirectional_kinetic_slide_do_not_trigger_the_reversal_detector() {
    for (speed, mu, ms) in [(0., 0.45, 160), (0.05, 0.2, 160), (0.32, 0.45, 160)] {
        let cfg = Run {
            speed,
            mu,
            ..fixture()
        };
        let mut baseline = Incline::new(&cfg.sdf()).unwrap();
        let mut refined = Incline::new(
            &Run {
                transition: Some(2000),
                ..cfg
            }
            .sdf(),
        )
        .unwrap();
        for _ in 0..ms {
            baseline.step().unwrap();
            refined.step().unwrap();
            assert_eq!(baseline.states(), refined.states());
            assert_eq!(
                baseline.material().particles(),
                refined.material().particles()
            );
            assert!(
                refined
                    .material()
                    .support_this_step()
                    .iter()
                    .all(|r| r.slip_transition_refinements == 0)
            );
        }
    }
    let before = run(
        Run {
            speed: 0.32,
            mu: 0.45,
            ..fixture()
        },
        300,
    )
    .unwrap();
    let after = run(
        Run {
            speed: 0.32,
            mu: 0.45,
            transition: Some(2000),
            ..fixture()
        },
        300,
    )
    .unwrap();
    assert!(
        after.worst_x < before.worst_x / 2. && after.slip_refinements > 0 && after.v.abs() < 2e-9
    );
}
#[test]
fn moving_pad_reaction_closes_momentum_and_has_smaller_slip_error_against_a_uniformly_fine_reference()
 {
    let cfg = Run {
        extra_mass: 0.002,
        ..fixture()
    };
    let coarse = run(cfg, 135).unwrap();
    let adaptive = run(
        Run {
            transition: Some(2000),
            ..cfg
        },
        135,
    )
    .unwrap();
    let fine = run(Run { width: 4000, ..cfg }, 135).unwrap();
    let err = |(x, v): (f64, f64)| ((x - fine.x).abs(), (v - fine.v).abs());
    let before = err((coarse.x, coarse.v));
    let after = err((adaptive.x, adaptive.v));
    println!(
        "135 ms, moving native pads, actual spin {:e} rad/s, 4 µs reference x/v {:e} m/{:e} m/s, 250 µs errors {before:?}, locally refined {after:?}, {} rejected trial(s)",
        adaptive.spin, fine.x, fine.v, adaptive.slip_refinements
    );
    assert!(
        adaptive.spin > 1e-4
            && adaptive.slip_refinements > 0
            && adaptive.p < 2e-13
            && adaptive.l < 2e-13
    );
    assert!(after.0 < before.0 * 0.65 && after.1 < before.1 * 0.65);
}
#[test]
fn an_authored_turn_cap_requires_frictional_persistent_contact_and_a_representable_recursive_step()
{
    let cfg = Run {
        transition: Some(2000),
        ..fixture()
    };
    for invalid in [
        cfg.sdf().replace(
            "max_slip_transition_step_ns=\"2000\"",
            "max_slip_transition_step_ns=\"250001\"",
        ),
        cfg.sdf().replace(
            "max_slip_transition_step_ns=\"2000\"",
            "max_slip_transition_step_ns=\"0\"",
        ),
        cfg.sdf().replace("persistent_contact", "recurrent_contact"),
        cfg.sdf().replace("friction=\"0.15\"", "friction=\"0\""),
    ] {
        assert!(Incline::new(&invalid).is_err());
    }
}

#[test]
fn kinetic_turn_and_native_reaction_transform_with_a_tilted_translated_world() {
    use glam::{DQuat, DVec3, EulerRot};
    let cfg = Run {
        transition: Some(2000),
        extra_mass: 0.002,
        ..fixture()
    };
    let source = cfg.sdf();
    let rpy = [0.41, -0.53, 1.04];
    let d = DVec3::new(2.13, -1.51, 0.94);
    let rot = DQuat::from_euler(EulerRot::ZYX, rpy[2], rpy[1], rpy[0]);
    let mut a = Incline::new(&source).unwrap();
    let mut b = Incline::new(&transition::reexpress(&source, rpy, d)).unwrap();
    let mut worst_x = 0f64;
    let mut worst_v = 0f64;
    let mut angle_axis = 0f64;
    let mut impulse = (DVec3::ZERO, DVec3::ZERO);
    let mut turns = [0u64; 2];
    for _ in 0..160 {
        a.step().unwrap();
        b.step().unwrap();
        for (p, q) in a
            .material()
            .particles()
            .iter()
            .zip(b.material().particles())
        {
            worst_x = worst_x.max((d + rot * DVec3::from(p.position)).distance(q.position.into()));
            worst_v = worst_v.max((rot * DVec3::from(p.velocity)).distance(q.velocity.into()));
        }
        for (p, q) in a.states().into_iter().zip(b.states()) {
            worst_x = worst_x.max((d + rot * DVec3::from(p.position)).distance(q.position.into()));
            angle_axis = angle_axis
                .max((rot * DVec3::from(p.angular_velocity)).distance(q.angular_velocity.into()));
        }
        for (i, w) in [&a, &b].iter().enumerate() {
            for r in w.material().support_this_step() {
                turns[i] += u64::from(r.slip_transition_refinements);
                for load in r.beginning.iter().chain(&r.end).flat_map(|a| &a.material) {
                    let j = DVec3::from(load.on_rigid);
                    if i == 0 {
                        impulse.0 += j;
                    } else {
                        impulse.1 += j;
                    }
                }
            }
        }
    }
    let j = (rot * impulse.0).distance(impulse.1);
    println!(
        "Oblique gravity/world pivot, true supported particle/native frame discrepancy position {worst_x:e} m, particle speed {worst_v:e} m/s, native angular velocity {angle_axis:e} rad/s, cumulative native impulse {j:e} N s, rejected physical transitions {turns:?}."
    );
    assert!(
        turns[0] > 0
            && turns[1] > 0
            && worst_x < 1e-7
            && worst_v < 2e-6
            && angle_axis < 2e-6
            && j < 2e-6
    );
}

#[test]
fn projected_component_sign_change_away_from_rest_is_not_a_coulomb_turn() {
    let base = Run {
        extra_mass: 0.01,
        ..fixture()
    };
    let a = run(base, 180).unwrap();
    let b = run(
        Run {
            transition: Some(2000),
            ..base
        },
        180,
    )
    .unwrap();
    assert!(a.crossing.is_some() && a.spin > 0.001);
    assert_eq!(b.slip_refinements, 0);
    assert_eq!(
        (a.x, a.v, a.spin, a.cuts, a.trials),
        (b.x, b.v, b.spin, b.cuts, b.trials)
    );
}
