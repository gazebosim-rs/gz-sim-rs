// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, Particle, RigidMobility, SoftBody, SolverConfig, XpbdError, step_coupled_rigid,
};
use std::time::Duration;

const DT: Duration = Duration::from_millis(1);
const MASSES: [f64; 2] = [0.02, 0.06];
const MU: f64 = 0.2;

fn material(substeps: u32) -> SoftBody {
    SoftBody::new(
        MASSES
            .iter()
            .enumerate()
            .map(|(i, mass)| Particle {
                position: [0., 0., 0.04 * i as f64],
                velocity: [0.; 3],
                inverse_mass: 1. / mass,
            })
            .collect(),
        vec![],
        SolverConfig {
            substeps,
            iterations: 12,
            gravity: [0., 0., -9.81],
            friction: MU,
            particle_radius: 0.01,
            self_contact: false,
        },
    )
    .unwrap()
}
fn pad(x: f64) -> Boundary {
    Boundary::Cuboid {
        center: [x, 0., 0.],
        half_extents: [0.03, 0.1, 0.1],
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    }
}
fn solve(body: &mut SoftBody, centers: [f64; 2]) -> Result<(), XpbdError> {
    let mobility = RigidMobility::PrescribedCuboid {
        normal_compliance: 0.001,
        start_orientation: DQuat::IDENTITY.to_array(),
        end_orientation: DQuat::IDENTITY.to_array(),
        start_angular_velocity: [0.; 3],
    };
    step_coupled_rigid(&mut [body], DT, &centers.map(pad), &[Some(mobility); 2]).map(|_| ())
}
#[test]
fn two_layers_report_each_pads_normal_and_tangent_during_sticking_and_sliding() {
    let h = DT.as_secs_f64();
    for substeps in [1, 2, 4] {
        let mut quiet = material(substeps);
        let mut observed = quiet.clone().with_pad_loads();
        for _ in 0..12 {
            // A 1 mm indentation at 0.001 m/N gives 1 N per pad per particle.
            // The first layer sticks; the heavier second layer saturates friction.
            for body in [&mut quiet, &mut observed] {
                solve(body, [-0.039, 0.039]).unwrap();
            }
            assert_eq!(quiet.particles(), observed.particles());
            assert_eq!(
                quiet.last_boundary_impulses(),
                observed.last_boundary_impulses()
            );
            assert!(quiet.last_pad_loads().is_empty());
            let loads = observed.last_pad_loads();
            assert_eq!(loads.len(), 4);
            for (boundary, rows) in loads.chunks_exact(2).enumerate() {
                let side = if boundary == 0 { -1. } else { 1. };
                let mut total = DVec3::ZERO;
                for (i, row) in rows.iter().enumerate() {
                    assert_eq!((row.boundary, row.particle), (boundary, i));
                    let normal = DVec3::from(row.normal);
                    let tangent = DVec3::from(row.tangent);
                    let support = (0.5 * MASSES[i] * 9.81).min(MU);
                    assert!((normal - side * h * DVec3::X).length() < 2e-11);
                    assert!((tangent + support * h * DVec3::Z).length() < 2e-11);
                    total += normal + tangent;
                }
                assert!(
                    total.distance(DVec3::from(
                        observed.last_boundary_impulses()[boundary].linear
                    )) < 1e-15
                );
            }
        }
        let committed = observed.last_pad_loads().to_vec();
        // An impossible core intersection follows an already loaded interval.
        assert_eq!(
            solve(&mut observed, [-0.039, 0.02]),
            Err(XpbdError::UnresolvedSweep)
        );
        assert_eq!(observed.last_pad_loads(), committed);
        solve(&mut observed, [-0.2, 0.2]).unwrap();
        assert!(observed.last_pad_loads().is_empty());
    }
}

#[cfg(feature = "one-way-checkpoint")]
#[test]
fn restoring_one_way_reactions_clears_old_articulated_pad_readings() {
    let mut observed = material(2).with_pad_loads();
    solve(&mut observed, [-0.039, 0.039]).unwrap();
    assert!(!observed.last_pad_loads().is_empty());
    let mut one_way = material(2);
    one_way
        .step(
            DT,
            &[Boundary::Plane {
                point: [0., 0., -1.],
                normal: [0., 0., 1.],
                velocity: [0.; 3],
            }],
        )
        .unwrap();
    observed
        .install_one_way_state(one_way.one_way_state().unwrap())
        .unwrap();
    assert_eq!(
        observed.last_boundary_impulses(),
        one_way.last_boundary_impulses()
    );
    assert!(observed.last_pad_loads().is_empty());
}
