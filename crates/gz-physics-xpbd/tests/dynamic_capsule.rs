// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, Particle, PrincipalRigidMotion, ReceiverMotion, RigidMobility, SoftBody,
    SolverConfig, step_coupled_rigid,
};
use std::time::Duration;

const H: f64 = 0.52;
const R: f64 = 0.20;
const SKIN: f64 = 0.025;
const M: f64 = 0.4;
const POINT_MASS: f64 = 0.01;
const DT: f64 = 0.02;
const INERTIA: DVec3 = DVec3::new(0.055, 0.055, 0.009);

fn point(x: DVec3, v: DVec3, substeps: u32) -> SoftBody {
    SoftBody::new(
        vec![Particle {
            position: x.to_array(),
            velocity: v.to_array(),
            inverse_mass: POINT_MASS.recip(),
        }],
        vec![],
        SolverConfig {
            gravity: [0.; 3],
            substeps,
            iterations: 16,
            particle_radius: SKIN,
            friction: 0.,
            self_contact: false,
        },
    )
    .unwrap()
}
fn integrate(p: &mut SoftBody, q0: DQuat, q1: DQuat, omega: DVec3, axes: DQuat) -> ReceiverMotion {
    let b = Boundary::DynamicCapsule {
        center: [0.; 3],
        radius: R,
        half_length: H,
        velocity: [0.; 3],
        angular_velocity: omega.to_array(),
    };
    let inertia = RigidMobility::Principal(PrincipalRigidMotion {
        inverse_mass: M.recip(),
        inverse_principal_inertia: INERTIA.recip().to_array(),
        start_orientation: q0.to_array(),
        end_orientation: q1.to_array(),
        principal_orientation: axes.to_array(),
        start_angular_velocity: omega.to_array(),
    });
    step_coupled_rigid(&mut [p], Duration::from_millis(20), &[b], &[Some(inertia)]).unwrap()[0]
        .unwrap()
}
fn spin_momentum(omega: DVec3, frame: DQuat) -> DVec3 {
    frame * (INERTIA * (frame.conjugate() * omega))
}

#[test]
fn last_instant_endcap_barrel_and_seam_match_mass_matrix_in_a_rotated_inertia_frame() {
    let q = DQuat::from_euler(glam::EulerRot::XYZ, 0.9, -0.71, 0.32);
    let axes = DQuat::from_rotation_y(0.49) * DQuat::from_rotation_z(-0.29);
    let frame = q * axes;
    for (feature, z, n) in [
        ("barrel", 0.28, DVec3::X),
        ("seam", H, DVec3::X),
        ("cap", H, DVec3::new(0.8, 0., 0.6)),
        ("axis", H, DVec3::Z),
        ("negative cap", -H, DVec3::new(-0.6, 0., -0.8)),
    ] {
        let arm = q * (z * DVec3::Z + n * (R + SKIN));
        let normal = q * n;
        let incoming = -2.3 * normal;
        let start = arm - incoming * DT;
        let torque = arm.cross(normal);
        let inverse = frame * (INERTIA.recip() * (frame.conjugate() * torque));
        let impulse = 2.3 / (M.recip() + POINT_MASS.recip() + torque.dot(inverse));
        let expected = incoming + impulse * normal / POINT_MASS;
        for count in [1, 2, 4, 16] {
            let mut p = point(start, incoming, count);
            let m = integrate(&mut p, q, q, DVec3::ZERO, axes);
            let v = DVec3::from(p.particles()[0].velocity);
            let linear = DVec3::from(m.linear_velocity_correction);
            let spin = DVec3::from(m.angular_velocity_correction);
            println!(
                "{feature} n={count} |v-oracle|={:e} |omega-oracle|={:e}",
                v.distance(expected),
                spin.distance(-impulse * inverse)
            );
            assert!(
                v.distance(expected) < 2e-10
                    && linear.distance(-impulse * normal / M) < 2e-11
                    && spin.distance(-impulse * inverse) < 2e-10
            );
            assert!(DVec3::from(m.translation_correction).length() < 1e-12);
            let p1 = DVec3::from(p.particles()[0].position);
            assert!(
                (p1.cross(POINT_MASS * v) + spin_momentum(spin, frame)
                    - start.cross(POINT_MASS * incoming))
                .length()
                    < 1e-11
            );
        }
    }
}

// Independent planar time stepping. Advance the free rotor and point, test the
// endpoint distance to the finite axis, then project their penetration and
// closing speed through the scalar planar mass matrix. This reference uses
// neither XPBD, sweeps, nor a contact tangent transported from first impact.
fn reference(s: f64, steps: usize) -> DVec3 {
    let h = DT / steps as f64;
    let mut theta = -0.72;
    let mut omega = 1.44 / DT;
    let mut center = DVec3::ZERO;
    let mut linear = DVec3::ZERO;
    let mut pos = DVec3::Z * s;
    let mut vel = DVec3::ZERO;
    for _ in 0..steps {
        theta += omega * h;
        center += linear * h;
        pos += vel * h;
        let arm = pos - center;
        let axis = DQuat::from_rotation_y(theta) * DVec3::Z;
        let d = arm - axis * arm.dot(axis).clamp(-H, H);
        let gap = d.length() - (R + SKIN);
        if gap >= 0. {
            continue;
        }
        let n = d.normalize();
        let lever = arm.cross(n).y;
        let inverse = M.recip() + POINT_MASS.recip() + lever * lever / INERTIA.y;
        let vn = (vel - linear).dot(n) - omega * lever;
        let j = (-vn / inverse).max(0.);
        vel += n * j / POINT_MASS;
        linear -= n * j / M;
        omega -= lever * j / INERTIA.y;
        let dx = -gap / inverse;
        pos += n * dx / POINT_MASS;
        center -= n * dx / M;
        theta -= lever * dx / INERTIA.y;
    }
    vel
}
#[test]
fn tumbling_barrel_and_tip_pass_through_the_point_between_two_clear_endpoints() {
    let angle = 0.72_f64;
    let omega = DVec3::Y * (2. * angle / DT);
    let q0 = DQuat::from_rotation_y(-angle);
    let q1 = DQuat::from_rotation_y(angle);
    for (kind, s) in [("barrel", 0.42_f64), ("cap", 0.73)] {
        let start = DVec3::Z * s;
        let r = R + SKIN;
        let theta = if kind == "barrel" {
            -(r / s).asin()
        } else {
            -((s * s + H * H - r * r) / (2. * s * H)).acos()
        };
        let axis = DQuat::from_rotation_y(theta) * DVec3::Z;
        let n = (start - axis * start.dot(axis).clamp(-H, H)).normalize();
        let lever = start.cross(n).y;
        let impulse =
            omega.y * lever / (M.recip() + POINT_MASS.recip() + lever * lever / INERTIA.y);
        let expected = n * (impulse / POINT_MASS);
        for q in [q0, q1] {
            let axis = q * DVec3::Z;
            assert!((start - axis * start.dot(axis).clamp(-H, H)).length() > r);
        }
        let reference_100k = reference(s, 100_000);
        let reference = reference(s, 200_000);
        let reference_change = reference.distance(reference_100k);
        println!(
            "reference {kind} final v={reference:?} dv(100k,200k)={reference_change:e}; first instantaneous velocity {expected:?}"
        );
        assert!(reference_change < 0.002);
        for count in [1, 4, 16, 64, 256] {
            let mut p = point(start, DVec3::ZERO, count);
            let out = integrate(&mut p, q0, q1, omega, DQuat::IDENTITY);
            let v = DVec3::from(p.particles()[0].velocity);
            let linear = DVec3::from(out.linear_velocity_correction);
            let spin = omega + DVec3::from(out.angular_velocity_correction);
            let pose = DQuat::from_array(out.rotation_correction) * q1;
            let p0 = M * linear + POINT_MASS * v;
            let l = spin_momentum(spin, pose)
                + DVec3::from(out.translation_correction).cross(M * linear)
                + DVec3::from(p.particles()[0].position).cross(POINT_MASS * v);
            let dl = l.distance(spin_momentum(omega, q0));
            let energy = 0.5
                * (M * linear.length_squared()
                    + POINT_MASS * v.length_squared()
                    + spin.dot(spin_momentum(spin, pose)));
            let initial = 0.5 * omega.dot(spin_momentum(omega, q0));
            let err = v.distance(reference);
            println!(
                "{kind} n={count} analytic t_hit={} ms dv={err:e} dP={:e} dL={dl:e} energy_loss={:e}",
                ((theta + angle) / omega.y) * 1e3,
                p0.length(),
                initial - energy
            );
            assert!(p0.length() < 1e-11 && dl < 1e-10 && energy < initial);
            assert!(
                err < 0.25,
                "{kind} n={count} v={v:?} reference={reference:?}"
            );
        }
    }
}

#[test]
fn exhausted_refinements_and_incompatible_mobility_leave_published_particles_and_reactions_intact()
{
    use gz_physics_xpbd::{SphereMobility, XpbdError};
    let mut p = point(DVec3::ZERO, DVec3::ZERO, 1);
    let plane = Boundary::Plane {
        point: [0., 0., -2.],
        normal: [0., 0., 1.],
        velocity: [0.; 3],
    };
    p.step(Duration::from_millis(1), &[plane]).unwrap();
    let start = p.particles().to_vec();
    let old = p.last_boundary_impulses().to_vec();
    let b = Boundary::DynamicCapsule {
        center: [0.; 3],
        radius: R,
        half_length: H,
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    };
    let m = RigidMobility::Principal(PrincipalRigidMotion {
        inverse_mass: M.recip(),
        inverse_principal_inertia: INERTIA.recip().to_array(),
        principal_orientation: DQuat::IDENTITY.to_array(),
        start_orientation: DQuat::IDENTITY.to_array(),
        end_orientation: DQuat::IDENTITY.to_array(),
        start_angular_velocity: [0.; 3],
    });
    for (ms, mob, wanted) in [
        (20_000_000, Some(m), XpbdError::UnresolvedSweep),
        (1, Some(m), XpbdError::UnresolvedSweep),
        (20_000_000, None, XpbdError::InvalidBoundary),
        (
            20_000_000,
            Some(RigidMobility::Isotropic(SphereMobility {
                inverse_mass: 1.,
                inverse_inertia: 1.,
            })),
            XpbdError::InvalidBoundary,
        ),
    ] {
        assert_eq!(
            step_coupled_rigid(&mut [&mut p], Duration::from_nanos(ms), &[b], &[mob]).unwrap_err(),
            wanted
        );
        assert_eq!(p.particles(), start);
        assert_eq!(p.last_boundary_impulses(), old);
        assert_eq!(p.last_sweep_refinements(), 0);
    }
}
