// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[allow(dead_code)]
#[path = "../examples/support/force_aware.rs"]
mod force;
#[allow(dead_code)]
#[path = "../examples/support/capsule_grasp.rs"]
mod grasp;
use force::*;
use glam::DVec3;
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step, reset_loaded_world};
#[test]
fn a_known_varying_force_stays_resolved_at_every_substep_and_its_complete_history_resets() {
    for dt in [0.001, 0.004, 0.008] {
        for n in [1, 2, 4, 8] {
            for aware in [false, true] {
                let mut p = Platen::new(aware, n, dt).unwrap();
                let mut history = vec![];
                let mut peak_speed = 0f64;
                let mut sum = 0.;
                let mut ncontact = 0;
                let mut max_error = 0f64;
                for tick in 0..125 {
                    let f = load(tick as f64 * dt);
                    p.step(f).unwrap();
                    let s = p.cloth();
                    for sample in s.last_capsule_contact_samples() {
                        let h = sample.interval[1] - sample.interval[0];
                        max_error = max_error.max((sample.normal_impulse / h - f).abs());
                        sum += sample.normal_impulse;
                        ncontact += 1;
                    }
                    if aware {
                        let [forcing] = s.capsule_forcing() else {
                            panic!()
                        };
                        assert!(forcing.used && forcing.boundary == 0);
                        assert!(
                            (DVec3::from(forcing.acceleration.unwrap()).z - f / 0.4).abs() < 1e-10
                        );
                        assert_eq!(s.last_capsule_contact_samples().len(), n as usize);
                    } else {
                        assert!(s.capsule_forcing().is_empty());
                    }
                    let b = p.state();
                    peak_speed = peak_speed.max(b.velocity[2].abs());
                    assert!((b.position[2] - 0.4).abs() < 1e-10);
                    if aware && dt == 0.004 && n == 4 {
                        history.push((
                            b,
                            s.last_capsule_contact_samples().to_vec(),
                            s.capsule_forcing().to_vec(),
                        ));
                    }
                }
                println!(
                    "platen dt={dt} n={n} aware={aware}: {ncontact} supported substeps, normal impulse {sum:.9} Ns, max sample force error {max_error:e} N, max native residual speed {peak_speed:e} m/s"
                );
                if aware {
                    assert!(max_error < 2e-8 && peak_speed < 2e-12);
                } else if n == 4 {
                    assert!(peak_speed > 0.001 && ncontact < 500);
                }
                if !history.is_empty() {
                    reset_loaded_world(&mut p.sim, |_| ()).unwrap();
                    assert!(
                        p.cloth().last_capsule_contact_samples().is_empty()
                            && p.cloth().capsule_forcing().is_empty()
                    );
                    for (t, (b, s, f)) in history.into_iter().enumerate() {
                        p.step(load(t as f64 * dt)).unwrap();
                        assert_eq!(p.state(), b);
                        assert_eq!(p.cloth().last_capsule_contact_samples(), s);
                        assert_eq!(p.cloth().capsule_forcing(), f);
                    }
                }
            }
        }
    }
}
#[test]
fn constant_force_motion_and_contact_history_require_eligible_soft_and_native_ownership() {
    for xml in [
        PLATEN
            .replace(
                "capsule_contact=\"material\"",
                "capsule_contact=\"particles\"",
            )
            .replace("capsule_contact_history=\"true\"", ""),
        PLATEN.replace("friction=\"0.65\"", "friction=\"0\""),
        PLATEN.replace(
            "linear_trajectory=\"force_aware\"",
            "linear_trajectory=\"typo\"",
        ),
        PLATEN.replace(
            "capsule_contact_history=\"true\"",
            "capsule_contact_history=\"typo\"",
        ),
    ] {
        assert!(
            load_world_with_sdf_step(
                &xml,
                "invalid-force.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}

#[test]
fn a_shared_open_loop_wrench_history_preserves_rigid_soft_momentum_and_full_endpoint_diagnostics() {
    check_wrench_history(false);
}
#[test]
fn the_previous_jaw_inertial_frame_reproduces_the_earlier_force_window() {
    check_wrench_history(true);
}
fn check_wrench_history(previous_frame: bool) {
    let dt = 0.001;
    let case = CarryInput::Balanced;
    let fixture = |aware| {
        if !previous_frame {
            return pair(aware, 4, dt, case).unwrap();
        }
        // The old native eigensystem put the smallest principal moment on link
        // X and delivered this exact principal frame to the soft receiver.
        let xml = pair_description(aware, 4, dt, case, 0.0625)
            .replace(
                "<inertial><mass>0.010",
                r#"<inertial><pose rotation_format="quat_xyzw">0 0 0 1 0 0 0</pose><mass>0.010"#,
            )
            .replace(
                "<ixx>0.00000115</ixx><iyy>0.00000115</iyy><izz>0.00000030</izz>",
                "<ixx>0.00000030</ixx><iyy>0.00000115</iyy><izz>0.00000115</izz>",
            );
        grasp::Grasp::new(grasp::Mode::Closed, xml, grasp::SQUEEZE).unwrap()
    };
    let mut chord = fixture(false);
    let mut aware = fixture(true);
    let mut windows = [Window::default(), Window::default()];
    let mut expected = DVec3::ZERO;
    let mut max_p = 0f64;
    let mut trace = vec![];
    for tick in 1..=1050 {
        let input = command(case, &chord, &chord);
        assert_eq!(input, command(case, &aware, &aware));
        chord.step_wrenches(input).unwrap();
        aware.step_wrenches(input).unwrap();
        expected += dt
            * (input[0].0 + input[1].0
                - DVec3::Z * (2. * grasp::JAW_MASS + grasp::CLOTH_MASS) * grasp::GRAVITY);
        for (i, g) in [&chord, &aware].into_iter().enumerate() {
            let sample = g.cloth();
            for point in sample.last_capsule_contact_samples() {
                let tangent = DVec3::from(point.tangent_impulse);
                let slip = DVec3::from(point.slip_velocity);
                assert!(
                    point.normal_impulse > 0.
                        && tangent.length() <= 0.65 * point.normal_impulse + 2e-13
                );
                assert!(tangent.dot(slip) <= 2e-13 * (1. + slip.length()));
                assert!((point.interval[0]..=point.interval[1]).contains(&point.first_time));
            }
            let rigid = g
                .jaws()
                .into_iter()
                .map(|b| grasp::JAW_MASS * DVec3::from(b.velocity))
                .sum::<DVec3>();
            let cloth = sample
                .particles()
                .iter()
                .map(|p| DVec3::from(p.velocity) / p.inverse_mass)
                .sum::<DVec3>();
            max_p = max_p.max((rigid + cloth - expected).length());
            assert_eq!(sample.last_boundary_impulses()[2].linear, [0.; 3]);
            if (570..=690).contains(&tick) {
                windows[i].add(sample, dt, 2, grasp::SQUEEZE);
            }
        }
        let s = aware.cloth();
        assert_eq!(s.capsule_forcing().len(), 2);
        assert!(s.capsule_forcing().iter().all(|f| f.used));
        if [1, 210, 610, 1050].contains(&tick) {
            trace.push((
                tick,
                aware.jaws(),
                s.particles().to_vec(),
                s.last_capsule_contact_samples().to_vec(),
                s.capsule_forcing().to_vec(),
            ));
        }
    }
    let [chord, known] = windows;
    assert!(max_p < 2e-11);
    assert!(chord.duty / chord.duration < 0.70);
    assert!(known.weighted_slip() < 0.6 * chord.weighted_slip());
    if previous_frame {
        assert!(known.duty / known.duration > 0.97);
        assert!(known.rms_variation() < 0.65 * chord.rms_variation());
    } else {
        assert!(known.duty / known.duration > chord.duty / chord.duration);
    }
    println!(
        "prior frame {previous_frame}, chord/force-aware duty {:.6}/{:.6}, RMS {:.6}/{:.6}, slip {:.9}/{:.9}",
        chord.duty / chord.duration,
        known.duty / known.duration,
        chord.rms_variation(),
        known.rms_variation(),
        chord.weighted_slip(),
        known.weighted_slip()
    );
    reset_loaded_world(&mut aware.sim, |_| ()).unwrap();
    for tick in 1..=1050 {
        let input = command(case, &aware, &aware);
        aware.step_wrenches(input).unwrap();
        if let Some((_, jaws, particles, samples, forcing)) = trace.iter().find(|t| t.0 == tick) {
            assert_eq!(&aware.jaws(), jaws);
            assert_eq!(aware.cloth().particles(), particles);
            assert_eq!(aware.cloth().last_capsule_contact_samples(), samples);
            assert_eq!(aware.cloth().capsule_forcing(), forcing);
        }
    }
    println!(
        "1050 identical jaw commands, exact reset at four checkpoints, max aggregate P residual {max_p:e} N s"
    );
}
