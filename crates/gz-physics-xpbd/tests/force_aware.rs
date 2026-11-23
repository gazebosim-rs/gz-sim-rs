// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, Particle, PrincipalRigidMotion, RigidMobility, SoftBody, SolverConfig,
    step_coupled_rigid,
};
use std::time::Duration;
const M: f64 = 0.4;
const R: f64 = 0.08;
const H: f64 = 0.12;
const SKIN: f64 = 0.015;
fn mobility(v: DVec3, a: DVec3, known: bool) -> RigidMobility {
    let m = PrincipalRigidMotion {
        inverse_mass: M.recip(),
        inverse_principal_inertia: [27., 27., 80.],
        principal_orientation: DQuat::IDENTITY.to_array(),
        start_orientation: DQuat::IDENTITY.to_array(),
        end_orientation: DQuat::IDENTITY.to_array(),
        start_angular_velocity: [0.; 3],
    };
    if known {
        RigidMobility::ForceAware {
            rotation: m,
            start_velocity: v.to_array(),
            acceleration: a.to_array(),
        }
    } else {
        RigidMobility::Principal(m)
    }
}
fn sheet(n: u32, z: f64, free: bool, gravity: DVec3) -> SoftBody {
    let data =
        [(-0.45, -0.25), (0.35, -0.25), (-0.45, 0.40), (0.35, 0.40)].map(|(x, y)| Particle {
            position: [x, y, z],
            velocity: [0.; 3],
            inverse_mass: if free { 4. } else { 0. },
        });
    let cfg = SolverConfig {
        gravity: gravity.to_array(),
        particle_radius: SKIN,
        substeps: n,
        iterations: 6,
        friction: 0.65,
        self_contact: false,
    };
    SoftBody::cloth(data.to_vec(), 2, 1e-4, 1e-4, 1e-3, cfg)
        .unwrap()
        .with_capsule_material_contact()
        .unwrap()
        .with_capsule_contact_history()
        .unwrap()
}
fn cap(chord: DVec3) -> Boundary {
    Boundary::DynamicCapsule {
        center: [0.; 3],
        radius: R,
        half_length: H,
        velocity: chord.to_array(),
        angular_velocity: [0.; 3],
    }
}
#[test]
fn constant_force_against_a_fixed_shell_produces_the_analytic_force_in_every_uniform_or_unequal_substep()
 {
    let a = DVec3::Z * 12.5;
    for ns in [4_000_000, 4_000_003] {
        let dt = Duration::from_nanos(ns);
        let d = dt.as_secs_f64();
        let v = a * d;
        let chord = 0.625 * v; // Four native semi-implicit steps integrated independently.
        for n in [1, 2, 4, 8, 12] {
            let mut cloth = sheet(n, H + R + SKIN, false, DVec3::ZERO);
            let m = step_coupled_rigid(
                &mut [&mut cloth],
                dt,
                &[cap(chord)],
                &[Some(mobility(DVec3::ZERO, a, true))],
            )
            .unwrap()[0]
                .unwrap();
            assert!((v + DVec3::from(m.linear_velocity_correction)).length() < 1e-10);
            assert!((d * chord + DVec3::from(m.translation_correction)).length() < 1e-13);
            let samples = cloth.last_capsule_contact_samples();
            assert_eq!(samples.len(), n as usize);
            for s in samples {
                let h = s.interval[1] - s.interval[0];
                assert!((s.normal_impulse / h - 5.).abs() < 5e-10, "{n} {s:?}");
            }
            let j = DVec3::from(cloth.last_boundary_impulses()[0].linear);
            assert!((j + M * v).length() < 1e-11);
            println!(
                "{ns} ns / {n} soft: {} substep records, total native impulse {:?}, final speed {:e} m/s",
                samples.len(),
                j,
                (v + DVec3::from(m.linear_velocity_correction)).length()
            );
        }
    }
    let d = 0.004;
    let dt = Duration::from_millis(4);
    let chord = 0.625 * a * d;
    let mut previous = sheet(4, H + R + SKIN, false, DVec3::ZERO);
    let m = step_coupled_rigid(
        &mut [&mut previous],
        dt,
        &[cap(chord)],
        &[Some(mobility(DVec3::ZERO, a, false))],
    )
    .unwrap()[0]
        .unwrap();
    let residual = a * d + DVec3::from(m.linear_velocity_correction);
    assert!((residual.z - 0.01875).abs() < 1e-10);
    println!(
        "Identical native inputs, affine trajectory retained {:.6} m/s normal velocity vs zero analytical static support.",
        residual.z
    );
}

#[test]
fn unloaded_flight_and_equal_acceleration_preserve_native_end_state_without_false_impulses() {
    let d = 0.007;
    let dt = Duration::from_millis(7);
    let a = -9.81 * DVec3::Z;
    for n in [1, 3, 8] {
        for distance in [0., 0.5] {
            let mut cloth = sheet(n, H + R + SKIN + distance, true, a);
            let m = step_coupled_rigid(
                &mut [&mut cloth],
                dt,
                &[cap(0.625 * d * a)],
                &[Some(mobility(DVec3::ZERO, a, true))],
            )
            .unwrap()[0]
                .unwrap();
            assert_eq!(m.translation_correction, [0.; 3]);
            assert_eq!(m.linear_velocity_correction, [0.; 3]);
            assert!(cloth.last_capsule_contact_samples().is_empty());
            assert_eq!(cloth.last_boundary_impulses()[0].linear, [0.; 3]);
        }
    }
}
