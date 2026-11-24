// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "../../../examples/support/free_top.rs"]
mod free_top;
use free_top::*;
use glam::{DQuat, DVec3};
use gz_sim_scene::reset_loaded_world;
#[test]
fn tri_axial_rigid_body_matches_independent_torque_free_and_driven_reference() {
    for ms in [1, 2, 5, 10] {
        let dt = f64::from(ms) * 0.001;
        for driven in [false, true] {
            let mut runs = [Run::new(false, ms).unwrap(), Run::new(true, ms).unwrap()];
            let mut reference = Reference::new(runs[0].state());
            let mut fine = reference;
            let mut histories = vec![];
            let mut errors = [[0f64; 4]; 2];
            let mut reference_angle = 0f64;
            let mut reference_energy = 0f64;
            for start in (0..6000).step_by(ms as usize) {
                let tau = torque(start, driven);
                reference.advance(dt, tau, 8);
                fine.advance(dt, tau, 32);
                reference_angle =
                    reference_angle.max(attitude_error(reference.attitude(), fine.attitude()));
                reference_energy = reference_energy.max((reference.energy() - fine.energy()).abs());
                for (i, run) in runs.iter_mut().enumerate() {
                    run.advance(tau).unwrap();
                    let s = run.state();
                    let (l, e) = spatial(s);
                    let values = [
                        l.distance(fine.momentum),
                        (e - fine.energy()).abs(),
                        attitude_error(DQuat::from_array(s.orientation), fine.attitude()),
                        DVec3::from(s.angular_velocity).distance(fine.spin()),
                    ];
                    for (m, x) in errors[i].iter_mut().zip(values) {
                        *m = m.max(x);
                    }
                    if i == 1 && ms == 10 {
                        histories.push(s);
                    }
                }
            }
            println!(
                "asymmetric {ms} ms driven={driven}: max |dL|, |dE|, attitude rad, spin rad/s: native={:?} isolated={:?}, RK4 8 vs 32 subdivisions attitude={reference_angle:e}, energy={reference_energy:e}",
                errors[0], errors[1]
            );
            assert!(reference_angle < 3e-8 && reference_energy < 2e-9);
            assert!(errors[1][0] < 1e-10 && errors[0][0] > errors[1][0] * 1e6);
            assert!(errors[1][1] < if driven { 1e-5 } else { 1e-9 });
            assert!(errors[1][2] < 0.0002 && errors[1][3] < 0.0006);
            if ms == 10 {
                reset_loaded_world(&mut runs[1].sim, |_| ()).unwrap();
                for (tick, state) in histories.into_iter().enumerate() {
                    runs[1].advance(torque(tick as u32 * ms, driven)).unwrap();
                    assert_eq!(runs[1].state(), state);
                }
            }
        }
    }
}
#[test]
fn invalid_free_rotation_configuration_is_rejected() {
    for invalid in [
        "<link>missing</link>",
        "<link a='1'>default::top::body</link>",
        "<link><x/></link>",
        "<x>default::top::body</x>",
        "<link>default::top::body</link><link/>",
    ] {
        let xml = XML.replace("<link>default::top::body</link>", invalid);
        let e = gz_sim_scene::load_world_with_sdf_step(
            &xml,
            "test.sdf",
            "default",
            &sdformat::NoResources,
            gz_sim_scene::PhysicsEngine::MultiFidelity,
        )
        .err()
        .unwrap();
        assert!(e.to_string().contains("FreeRotation"), "{e}");
    }
}
