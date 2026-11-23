// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DMat3, DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, CapsuleRegion, MaterialFeature, Particle, PrincipalRigidMotion, RigidMobility,
    SoftBody, SolverConfig, step_coupled_rigid,
};
use std::time::Duration;
const H: f64 = 0.2;
const R: f64 = 0.08;
const SKIN: f64 = 0.02;
const M: f64 = 0.4;
const NODE: f64 = 0.2;
const EDGE: f64 = 0.6;
const INERTIA: DVec3 = DVec3::new(0.045, 0.067, 0.090);
const VN: f64 = 1.5;
const DT: f64 = 0.002;
fn near(a: DVec3, b: DVec3, e: f64) {
    assert!(
        a.distance(b) < e,
        "{a:?} vs {b:?}; error {:e}",
        a.distance(b)
    );
}
fn sheet(v: DVec3, mu: f64, substeps: u32) -> SoftBody {
    let a = DVec3::new(-0.5, -0.5 / 3f64.sqrt(), 0.) * EDGE;
    let b = DVec3::new(0.5, -0.5 / 3f64.sqrt(), 0.) * EDGE;
    let c = DVec3::new(0., 1. / 3f64.sqrt(), 0.) * EDGE;
    let particles = [a, b, a + c - b, c]
        .map(|q| Particle {
            position: (DVec3::Z * (H + R + SKIN) + q - DT * v).to_array(),
            velocity: v.to_array(),
            inverse_mass: NODE.recip(),
        })
        .to_vec();
    let config = SolverConfig {
        gravity: [0.; 3],
        particle_radius: SKIN,
        substeps,
        iterations: 4,
        friction: mu,
        self_contact: false,
    };
    SoftBody::cloth(particles, 2, 1e-5, 1e-5, 1e-5, config)
        .unwrap()
        .with_capsule_material_contact()
        .unwrap()
}
fn capsule() -> Boundary {
    Boundary::DynamicCapsule {
        center: [0.; 3],
        radius: R,
        half_length: H,
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    }
}
fn mobility(q: DQuat) -> RigidMobility {
    RigidMobility::Principal(PrincipalRigidMotion {
        inverse_mass: M.recip(),
        inverse_principal_inertia: INERTIA.recip().to_array(),
        principal_orientation: q.to_array(),
        start_orientation: DQuat::IDENTITY.to_array(),
        end_orientation: DQuat::IDENTITY.to_array(),
        start_angular_velocity: [0.; 3],
    })
}

#[test]
fn endcap_friction_matches_full_analytical_skin_and_tensor_for_sticking_and_sliding() {
    let drive = DVec3::new(1.2, -0.75, 0.);
    let q = DQuat::from_rotation_z(0.57);
    let native_inv = |j: DVec3| q * ((q.inverse() * j) / INERTIA);
    let n = DVec3::Z;
    let point = n * (H + R);
    let v = drive - VN * n;
    let normal = VN / ((3. * NODE).recip() + M.recip());
    let in_plane_inertia = NODE * EDGE * EDGE / 2.;
    let response = |j: DVec3| {
        j * ((3. * NODE).recip() + SKIN * SKIN / in_plane_inertia + M.recip())
            + native_inv(point.cross(j)).cross(point)
    };
    let w = DMat3::from_cols(response(DVec3::X), response(DVec3::Y), DVec3::Z);
    let stick = -w.inverse() * drive;
    for mu in [0.15, 0.6, 1.8] {
        let limit = mu * normal;
        // On a sliding disk the optimum is the minimum energy angle. Golden
        // section uses only the continuum kinetic-energy quadratic.
        let expected = if stick.length() <= limit {
            stick
        } else {
            let objective = |a: f64| {
                let j = limit * DVec3::new(a.cos(), a.sin(), 0.);
                j.dot(drive) + 0.5 * j.dot(response(j))
            };
            let center = (-drive.y).atan2(-drive.x);
            let (mut l, mut r) = (center - 1.5, center + 1.5);
            for _ in 0..80 {
                let a = l + (r - l) * 0.38;
                let b = l + (r - l) * 0.62;
                if objective(a) < objective(b) {
                    r = b
                } else {
                    l = a
                }
            }
            let a = 0.5 * (l + r);
            limit * DVec3::new(a.cos(), a.sin(), 0.)
        };
        for steps in [1, 2, 4, 8] {
            let mut body = sheet(v, mu, steps);
            let initial = body.particles().to_vec();
            let rigid = step_coupled_rigid(
                &mut [&mut body],
                Duration::from_millis(2),
                &[capsule()],
                &[Some(mobility(q))],
            )
            .unwrap()[0]
                .unwrap();
            let [load] = body.last_capsule_friction_loads() else {
                panic!("{:?}", body.last_capsule_friction_loads())
            };
            let [nl] = body.last_capsule_material_loads() else {
                panic!()
            };
            assert!(
                load.two_way
                    && load.feature == MaterialFeature::Triangle([0, 1, 3])
                    && load.region == CapsuleRegion::PositiveCap
            );
            near(nl.linear.into(), -normal * n, 1e-11);
            near(load.linear.into(), -expected, 1.8e-8);
            near(load.angular.into(), point.cross(-expected), 1e-8);
            assert!(
                (load.normal_impulse_sum - normal).abs() < 1e-11
                    && (load.tangent_impulse_sum - expected.length()).abs() < 1e-8
            );
            let mut spatial = DVec3::ZERO;
            let mut net = DVec3::ZERO;
            for &(id, j) in &load.nodal {
                let j = DVec3::from(j);
                net += j;
                spatial += DVec3::from(body.particles()[id].position).cross(j);
            }
            near(net, expected, 1.8e-8);
            near(spatial, point.cross(expected), 1e-8);
            let j = -normal * n - expected;
            near(rigid.linear_velocity_correction.into(), j / M, 6e-8);
            near(
                rigid.angular_velocity_correction.into(),
                native_inv(point.cross(j)),
                1.5e-7,
            );
            let e0 = initial
                .iter()
                .map(|p| 0.5 * NODE * DVec3::from(p.velocity).length_squared())
                .sum::<f64>();
            let spin = q.inverse() * DVec3::from(rigid.angular_velocity_correction);
            let e = body
                .particles()
                .iter()
                .map(|p| 0.5 * NODE * DVec3::from(p.velocity).length_squared())
                .sum::<f64>()
                + 0.5
                    * (M * DVec3::from(rigid.linear_velocity_correction).length_squared()
                        + spin.dot(INERTIA * spin));
            let exact_loss =
                0.5 * normal * VN - expected.dot(drive) - 0.5 * expected.dot(response(expected));
            assert!(((e0 - e) - exact_loss).abs() < 1e-11);
            let slip = (drive + response(expected)).length();
            println!(
                "mu={mu} steps={steps} true normal={normal} tangent={expected:?} postslip={slip} measured kinetic loss={} J",
                e0 - e
            );
        }
    }
}

#[test]
fn tilted_endcap_tangent_and_normal_converge_to_the_same_three_dimensional_sticking_solution() {
    let n = DVec3::new(0.54, 0.18, (1. - 0.54f64.powi(2) - 0.18f64.powi(2)).sqrt());
    let u = DVec3::Z.cross(n).normalize();
    let v = n.cross(u);
    let drive = 1.5 * u - 1.1 * v - 1.7 * n;
    let at = DVec3::Z * H + R * n;
    let a = EDGE * (-0.5 * u - 0.5 / 3f64.sqrt() * v);
    let b = EDGE * (0.5 * u - 0.5 / 3f64.sqrt() * v);
    let c = EDGE / 3f64.sqrt() * v;
    let p = [a, b, a + c - b, c].map(|d| Particle {
        position: (at + SKIN * n + d - DT * drive).to_array(),
        velocity: drive.to_array(),
        inverse_mass: NODE.recip(),
    });
    let q = DQuat::from_rotation_x(-0.40) * DQuat::from_rotation_y(0.23);
    let inv = |j: DVec3| q * ((q.inverse() * j) / INERTIA);
    let soft_i = NODE * EDGE * EDGE / 2.;
    let w = |j: DVec3| {
        (((3. * NODE).recip() + M.recip()) * j)
            + (SKIN * SKIN / soft_i) * (j - n * j.dot(n))
            + inv(at.cross(j)).cross(at)
    };
    let mass = DMat3::from_cols(w(DVec3::X), w(DVec3::Y), w(DVec3::Z));
    let expected = mass.inverse() * (-drive);
    let normal = n * expected.dot(n);
    let tangent = expected - normal;
    let frictionless = -drive.dot(n) / n.dot(w(n));
    assert!((expected.dot(n) - frictionless).abs() > 0.02);
    for substeps in [1, 4, 8] {
        let cfg = SolverConfig {
            gravity: [0.; 3],
            particle_radius: SKIN,
            substeps,
            iterations: 6,
            friction: 2.,
            self_contact: false,
        };
        let mut cloth = SoftBody::cloth(p.to_vec(), 2, 1e-5, 1e-5, 1e-5, cfg)
            .unwrap()
            .with_capsule_material_contact()
            .unwrap();
        step_coupled_rigid(
            &mut [&mut cloth],
            Duration::from_millis(2),
            &[capsule()],
            &[Some(mobility(q))],
        )
        .unwrap();
        let [nl] = cloth.last_capsule_material_loads() else {
            panic!()
        };
        let [tl] = cloth.last_capsule_friction_loads() else {
            panic!()
        };
        near(nl.linear.into(), -normal, 1e-10);
        near(tl.linear.into(), -tangent, 1e-10);
        near(
            cloth.last_boundary_impulses()[0].linear.into(),
            -expected,
            1e-10,
        );
        println!(
            "coupled oblique endcap n={substeps}: full normal {:.9} N s vs frictionless {:.9}, tangent {:.9} N s",
            normal.length(),
            frictionless,
            tangent.length()
        );
    }
}

#[test]
fn shared_cloth_edge_transports_the_skin_couple_through_all_four_incident_vertices() {
    let mu = 0.32;
    let v = DVec3::new(0.95, 0.2, -1.5);
    let square = [(-0.3, -0.3), (0.3, -0.3), (-0.3, 0.3), (0.3, 0.3)];
    let p = square.map(|(x, y)| Particle {
        position: (DVec3::new(x, y, H + R + SKIN) - DT * v).to_array(),
        velocity: v.to_array(),
        inverse_mass: NODE.recip(),
    });
    let cfg = SolverConfig {
        gravity: [0.; 3],
        particle_radius: SKIN,
        substeps: 4,
        iterations: 8,
        friction: mu,
        self_contact: false,
    };
    let mut cloth = SoftBody::cloth(p.to_vec(), 2, 1e-4, 1e-4, 1e-4, cfg)
        .unwrap()
        .with_capsule_material_contact()
        .unwrap();
    step_coupled_rigid(
        &mut [&mut cloth],
        Duration::from_millis(2),
        &[capsule()],
        &[Some(mobility(DQuat::IDENTITY))],
    )
    .unwrap();
    let [f] = cloth.last_capsule_friction_loads() else {
        panic!("{:?}", cloth.last_capsule_friction_loads())
    };
    assert_eq!(f.feature, MaterialFeature::Segment([0, 3]));
    assert_eq!(
        f.nodal.iter().map(|(i, _)| *i).collect::<Vec<_>>(),
        vec![0, 1, 2, 3]
    );
    let mut force = DVec3::ZERO;
    let mut torque = DVec3::ZERO;
    for &(i, j) in &f.nodal {
        let j = DVec3::from(j);
        force += j;
        torque += DVec3::from(cloth.particles()[i].position).cross(j);
        if i == 1 || i == 2 {
            assert!(j.length() > 1e-5, "off-edge skin couple {i}: {j:?}");
        }
    }
    near(force, -DVec3::from(f.linear), 1e-12);
    near(torque, -DVec3::from(f.angular), 1e-12);
    assert!((f.tangent_impulse_sum - mu * f.normal_impulse_sum).abs() < 1e-11);
}

#[test]
fn wallward_soft_impulse_cannot_drive_one_finite_jaw_through_another_between_samples() {
    let points = [-0.4, 0.4].map(|x| Particle {
        position: [x, 0., R + SKIN + 0.001],
        velocity: [0., 0., -2.],
        inverse_mass: 2.,
    });
    let cfg = SolverConfig {
        gravity: [0.; 3],
        particle_radius: SKIN,
        substeps: 4,
        iterations: 8,
        friction: 0.,
        self_contact: false,
    };
    let mut cable = SoftBody::cable(points.to_vec(), 0., 0., cfg)
        .unwrap()
        .with_capsule_material_contact()
        .unwrap();
    let other = Boundary::DynamicCapsule {
        center: [0., 0., -2. * R - 0.001],
        radius: R,
        half_length: H,
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    };
    let q = DQuat::from_rotation_x(-std::f64::consts::FRAC_PI_2).to_array();
    let rigid = RigidMobility::Principal(PrincipalRigidMotion {
        inverse_mass: M.recip(),
        inverse_principal_inertia: INERTIA.recip().to_array(),
        principal_orientation: DQuat::IDENTITY.to_array(),
        start_orientation: q,
        end_orientation: q,
        start_angular_velocity: [0.; 3],
    });
    let err = step_coupled_rigid(
        &mut [&mut cable],
        Duration::from_millis(12),
        &[capsule(), other],
        &[Some(rigid); 2],
    )
    .unwrap_err();
    assert_eq!(err, gz_physics_xpbd::XpbdError::SimultaneousCapsules);
    assert_eq!(cable.particles(), points);
    assert!(
        cable.last_capsule_material_loads().is_empty()
            && cable.last_capsule_friction_loads().is_empty()
            && cable.last_boundary_impulses().is_empty()
    );
}
