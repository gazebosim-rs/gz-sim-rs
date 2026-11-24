// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_xpbd::{
    MaterialFeature, Particle, SoftBody, SolverConfig, articulated_impact::ImpactSphere,
};
const R: f64 = 0.2;
const SKIN: f64 = 0.01;
fn sheet(v: [f64; 3], w: f64) -> SoftBody {
    let particles = [
        [-1., -1., R + SKIN],
        [1., -1., R + SKIN],
        [-1., 1., R + SKIN],
        [1., 1., R + SKIN],
    ]
    .map(|position| Particle {
        position,
        velocity: v,
        inverse_mass: w,
    })
    .to_vec();
    SoftBody::cloth(
        particles,
        2,
        2e-3,
        2e-3,
        3e-3,
        SolverConfig {
            gravity: [0.; 3],
            particle_radius: SKIN,
            substeps: 2,
            iterations: 8,
            friction: 0.,
            self_contact: false,
        },
    )
    .unwrap()
}
fn sphere(x: f64, y: f64, z: f64) -> ImpactSphere {
    ImpactSphere {
        center: [x, y, z],
        radius: R,
    }
}
fn diag(z: f64) -> [[f64; 3]; 3] {
    [[0.; 3], [0.; 3], [0., 0., z]]
}
#[test]
fn face_edge_and_cable_barrier_impulses_match_the_independent_two_mass_oracle() {
    for (x, y, ids, weights) in [
        (0.2, -0.2, vec![0, 1, 3], vec![0.4, 0.2, 0.4]),
        (0., 0., vec![0, 3], vec![0.5, 0.5]),
    ] {
        for e in [0., 0.35, 1.] {
            let mut body = sheet([0.12, 0., -0.8], 10.);
            let impact = body.sphere_impact_at_barrier(&[sphere(x, y, 0.)]).unwrap();
            assert_eq!(impact.sites().len(), 1);
            let site = impact.sites()[0];
            assert_eq!(site.feature.vertices(), ids);
            let w = 10. * weights.iter().map(|w| w * w).sum::<f64>() + 0.30;
            let j = (1. + e) * 0.8 / w;
            let report = impact
                .prepare(&[[0.; 3]], |_, _| diag(0.30), e)
                .unwrap()
                .commit();
            let load = &report.loads[0];
            assert!((load.on_rigid[2] + j).abs() < 2e-14);
            assert!((load.final_normal_velocity - e * 0.8).abs() < 2e-14);
            assert!(
                (report.kinetic_energy_change + 0.5 * (1. - e * e) * 0.8 * 0.8 / w).abs() < 2e-14
            );
            for (i, p) in body.particles().iter().enumerate() {
                let expected = -0.8
                    + ids
                        .iter()
                        .position(|&id| id == i)
                        .map_or(0., |k| 10. * weights[k] * j);
                assert!((p.velocity[2] - expected).abs() < 1e-14);
                assert!((p.velocity[0] - 0.12).abs() < 2e-14);
            }
        }
    }
    let mut string = SoftBody::cable(
        vec![
            Particle {
                position: [-0.3, 0., R + SKIN],
                velocity: [0., 0., -1.],
                inverse_mass: 1.,
            },
            Particle {
                position: [0.7, 0., R + SKIN],
                velocity: [0., 0., -1.],
                inverse_mass: 0.5,
            },
        ],
        0.01,
        0.01,
        SolverConfig {
            gravity: [0.; 3],
            particle_radius: SKIN,
            substeps: 1,
            iterations: 4,
            friction: 0.,
            self_contact: false,
        },
    )
    .unwrap();
    let impact = string
        .sphere_impact_at_barrier(&[sphere(0., 0., 0.)])
        .unwrap();
    assert_eq!(impact.sites()[0].feature, MaterialFeature::Segment([0, 1]));
    let report = impact
        .prepare(&[[0.; 3]], |_, _| diag(0.2), 0.4)
        .unwrap()
        .commit();
    assert!(
        (report.loads[0].on_rigid[2] + 1.4 / (0.7 * 0.7 + 0.5 * 0.3 * 0.3 + 0.2)).abs() < 1e-14
    );
}
#[test]
fn two_fingertips_share_both_native_and_material_mobility_and_prescribed_power_is_accounted() {
    let b0 = [0.40, 0.2, 0., 0.4];
    let b1 = [0.4, 0., 0.2, 0.40];
    let mut body = sheet([0., 0., -1.], 4.);
    let impact = body
        .sphere_impact_at_barrier(&[sphere(0.2, -0.2, 0.), sphere(-0.2, 0.2, 0.)])
        .unwrap();
    assert_eq!(impact.sites().len(), 2);
    let mat = [[0.5, 0.12], [0.12, 0.7]];
    let kernel =
        |a: [f64; 4], b: [f64; 4]| a.into_iter().zip(b).map(|(a, b)| 4. * a * b).sum::<f64>();
    let (a, b, d) = (
        kernel(b0, b0) + mat[0][0],
        kernel(b0, b1) + mat[0][1],
        kernel(b1, b1) + mat[1][1],
    );
    let j0 = 1.3 * (d - b) / (a * d - b * b);
    let j1 = 1.3 * (a - b) / (a * d - b * b);
    let report = impact
        .prepare(&[[0.; 3]; 2], |i, j| diag(mat[i][j]), 0.3)
        .unwrap()
        .commit();
    assert!(
        (report.loads[0].on_rigid[2] + j0).abs() < 1e-14
            && (report.loads[1].on_rigid[2] + j1).abs() < 1e-14
    );
    assert!((report.dissipated_energy - 0.35 * (j0 + j1)).abs() < 1e-13);
    let mut pinned = sheet([0., 0., -1.], 0.);
    let impact = pinned
        .sphere_impact_at_barrier(&[sphere(0.2, -0.2, 0.)])
        .unwrap();
    let report = impact
        .prepare(&[[0.; 3]], |_, _| diag(0.5), 0.)
        .unwrap()
        .commit();
    assert!(
        (report.prescribed_support_work - 2.).abs() < 1e-14
            && (report.dissipated_energy - 1.).abs() < 1e-14
            && (report.kinetic_energy_change - 1.).abs() < 1e-14
    );
}
#[test]
fn separated_separating_penetrating_and_invalid_inputs_preserve_particles() {
    let mut body = sheet([0., 0., 1.], 1.0);
    let saved = body.particles().to_vec();
    let clear = body
        .sphere_impact_at_barrier(&[sphere(0.2, -0.2, -0.01)])
        .unwrap();
    assert!(clear.sites().is_empty());
    drop(clear);
    assert!(
        body.sphere_impact_at_barrier(&[sphere(0.2, -0.2, 0.01)])
            .is_err()
    );
    assert!(
        body.sphere_impact_at_barrier(&[sphere(0.2, -0.2, 0.)])
            .unwrap()
            .prepare(&[[0.; 3]], |_, _| diag(f64::INFINITY), 0.)
            .is_err()
    );
    let result = body
        .sphere_impact_at_barrier(&[sphere(0., 0., 0.)])
        .unwrap()
        .prepare(&[[0.; 3]], |_, _| diag(0.3), 0.2)
        .unwrap()
        .commit();
    assert_eq!(result.loads[0].on_rigid, [0.; 3]);
    assert_eq!(body.particles(), saved);
}

#[test]
fn thick_shared_edge_coulomb_threshold_and_skin_offset_couple_match_closed_form() {
    use glam::DVec3;
    let vel = DVec3::new(0.56, -0.42, -0.8);
    let rigid = 0.3;
    let inverse = 4.;
    let restitution = 0.45;
    let normal = (1. + restitution) * (-vel.z) / (0.5 * inverse + rigid);
    // Least-squares angular inertia of four corners (+/-1,+/-1) is (4,4,8).
    let tangent_inverse = inverse * (0.5 + SKIN.powi(2) / 4.) + rigid;
    let critical = 0.7 / (tangent_inverse * normal);
    for mu in [
        0.10 * critical,
        critical * (1. - 1e-5),
        critical,
        critical * (1. + 1e-5),
        2. * critical,
    ] {
        let mut raw = sheet(vel.to_array(), inverse);
        let mut cfg = raw.config();
        cfg.friction = mu;
        let mut body = SoftBody::cloth(raw.particles().to_vec(), 2, 0.01, 0.02, 0.03, cfg).unwrap();
        let impact = body
            .sphere_impact_at_barrier(&[sphere(0., 0., 0.)])
            .unwrap();
        let report = impact
            .prepare(
                &[[0.; 3]],
                |_, _| [[rigid, 0., 0.], [0., rigid, 0.], [0., 0., rigid]],
                restitution,
            )
            .unwrap()
            .commit();
        let load = &report.loads[0];
        let friction = load.friction.as_ref().unwrap();
        let jt = (0.7 / tangent_inverse).min(mu * normal);
        let tangent = DVec3::new(0.8, -0.6, 0.);
        let expected = normal * DVec3::Z - jt * tangent;
        assert!(DVec3::from(load.on_rigid).distance(-expected) < 3e-12);
        let slip = (0.7 - jt * tangent_inverse).max(0.) * tangent;
        assert!(DVec3::from(friction.final_slip).distance(slip) < 3e-12);
        let (mut linear, mut moment) = (DVec3::ZERO, DVec3::ZERO);
        let before = raw.particles();
        for &(id, j) in &load.on_particles {
            let change =
                DVec3::from(body.particles()[id].velocity) - DVec3::from(before[id].velocity);
            assert!(change.distance(DVec3::from(j) * inverse) < 2e-13);
            linear += DVec3::from(j);
            moment += DVec3::from(before[id].position).cross(DVec3::from(j));
        }
        assert_eq!(load.on_particles.len(), 4);
        assert!(linear.distance(expected) < 2e-12);
        assert!(moment.distance(DVec3::from(load.site.point).cross(expected)) < 2e-12);
        let loss = 0.5 * normal * (0.8 - 0.8 * restitution) + 0.5 * jt * (0.7 + slip.length());
        assert!((report.dissipated_energy - loss).abs() < 2e-12);
        assert!(raw.sphere_impact_at_barrier(&[sphere(0., 0., 0.)]).is_ok());
    }
}
