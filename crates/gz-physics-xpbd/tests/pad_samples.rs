// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics_xpbd::{
    Boundary, PadSample, Particle, RigidMobility, SoftBody, SolverConfig, XpbdError,
    step_coupled_rigid,
};
use std::time::Duration;

const DT: Duration = Duration::from_millis(1);
const SAMPLE: PadSample = PadSample {
    particles: [0, 1, 3],
    weights: [0.2, 0.5, 0.3],
    stiffness_weight: 1.,
};
fn cloth(gravity: f64, iterations: u32) -> SoftBody {
    let points = [(0., 0.), (0.06, 0.), (0., 0.06), (0.06, 0.06)]
        .into_iter()
        .map(|(y, z)| Particle {
            position: [0., y, z],
            velocity: [0.; 3],
            inverse_mass: 2.,
        })
        .collect();
    SoftBody::cloth(
        points,
        2,
        0.1,
        0.1,
        1.,
        SolverConfig {
            gravity: [0., 0., gravity],
            substeps: 1,
            iterations,
            particle_radius: 0.01,
            friction: 0.3,
            self_contact: false,
        },
    )
    .unwrap()
}
fn pad(center: [f64; 3], half: [f64; 3]) -> Boundary {
    Boundary::Cuboid {
        center,
        half_extents: half,
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    }
}
fn run(body: &mut SoftBody, jaws: &[Boundary], c: f64) -> Result<(), XpbdError> {
    let m = RigidMobility::PrescribedCuboid {
        normal_compliance: c,
        start_orientation: [0., 0., 0., 1.],
        end_orientation: [0., 0., 0., 1.],
        start_angular_velocity: [0.; 3],
    };
    step_coupled_rigid(&mut [body], DT, jaws, &vec![Some(m); jaws.len()]).map(|_| ())
}
#[test]
fn an_interior_contact_balances_the_free_cloths_linear_and_angular_momentum() {
    let mut b = cloth(-4., 1)
        .with_pad_samples(vec![SAMPLE])
        .unwrap()
        .with_pad_loads();
    let start = b.particles().to_vec();
    let center = DVec3::new(-0.039, 0.048, 0.018);
    let jaw = pad(center.to_array(), [0.03, 0.003, 0.003]);
    let mut ordinary = cloth(-4., 1);
    run(&mut ordinary, &[jaw], 0.001).unwrap();
    assert_eq!(ordinary.last_boundary_impulses()[0].linear, [0.; 3]);
    run(&mut b, &[jaw], 0.001).unwrap();
    let reaction = b.last_boundary_impulses()[0];
    let force = DVec3::from(reaction.linear) / DT.as_secs_f64();
    let expected_normal =
        0.001 / (0.001 + 1e-6 * 2. * (0.2_f64.powi(2) + 0.5_f64.powi(2) + 0.3_f64.powi(2)));
    assert!((force.x + expected_normal).abs() < 1e-11);
    assert!((force.z + 0.3 * expected_normal).abs() < 1e-11);
    let mut delta_p = DVec3::ZERO;
    let mut delta_l = DVec3::ZERO;
    for (old, p) in start.iter().zip(b.particles()) {
        let measured = (DVec3::from(p.velocity) + 0.004 * DVec3::Z) / p.inverse_mass;
        delta_p += measured;
        delta_l += (DVec3::from(old.position) - center).cross(measured);
    }
    assert!((delta_p + DVec3::from(reaction.linear)).length() < 1e-14);
    assert!((delta_l + DVec3::from(reaction.angular)).length() < 1e-14);
    for ((row, n), w) in b
        .last_pad_loads()
        .iter()
        .zip(SAMPLE.particles)
        .zip(SAMPLE.weights)
    {
        assert_eq!(row.particle, n);
        assert!(
            (DVec3::from(row.normal) + DVec3::from(row.tangent) - DVec3::from(reaction.linear) * w)
                .length()
                < 1e-14
        );
    }
    assert_eq!(b.last_pad_loads().len(), 3);
    let committed = b.last_pad_loads().to_vec();
    let state = b.particles().to_vec();
    // A non-sampled vertex crosses a small jaw core while the sample remains clear.
    let bad = pad([0., 0., 0.06], [0.001; 3]);
    assert_eq!(run(&mut b, &[bad], 0.001), Err(XpbdError::UnresolvedSweep));
    assert_eq!(b.last_pad_loads(), committed);
    assert_eq!(b.particles(), state);
}
#[test]
fn equal_indentation_produces_the_same_flat_patch_stiffness_and_torque_after_subdivision() {
    let jaws = [
        pad([-0.039, 0.03, 0.03], [0.03, 0.1, 0.1]),
        pad([0.039, 0.03, 0.03], [0.03, 0.1, 0.1]),
    ];
    let split = vec![
        PadSample {
            weights: [0.4, 0.4, 0.2],
            stiffness_weight: 0.5,
            ..SAMPLE
        },
        PadSample {
            weights: [0.0, 0.6, 0.4],
            stiffness_weight: 0.5,
            ..SAMPLE
        },
    ];
    for samples in [vec![SAMPLE], split] {
        let mut b = cloth(0., 8).with_pad_samples(samples).unwrap();
        run(&mut b, &jaws, 0.001).unwrap();
        for (k, r) in b.last_boundary_impulses().iter().enumerate() {
            let j = if k == 0 { -0.001 } else { 0.001 };
            let torque = DVec3::new(0., 0.048 - 0.03, 0.018 - 0.03).cross(j * DVec3::X);
            assert!((DVec3::from(r.linear) - j * DVec3::X).length() < 1e-14);
            assert!((DVec3::from(r.angular) - torque).length() < 1e-14);
        }
    }
}
#[test]
fn sample_construction_requires_a_material_face_and_finite_normalized_weights() {
    let c = cloth(0., 1);
    assert!(c.clone().with_pad_samples(vec![]).is_err());
    for s in [
        PadSample {
            particles: [0, 1, 2],
            ..SAMPLE
        },
        PadSample {
            particles: [0, 3, 3],
            ..SAMPLE
        },
        PadSample {
            particles: [0, 3, usize::MAX],
            ..SAMPLE
        },
        PadSample {
            weights: [0.8, -0.1, 0.3],
            ..SAMPLE
        },
        PadSample {
            weights: [f64::NAN, 0.5, 0.5],
            ..SAMPLE
        },
        PadSample {
            weights: [0.2, 0.2, 0.2],
            ..SAMPLE
        },
        PadSample {
            stiffness_weight: 0.,
            ..SAMPLE
        },
        PadSample {
            stiffness_weight: f64::INFINITY,
            ..SAMPLE
        },
    ] {
        assert!(matches!(
            c.clone().with_pad_samples(vec![s]),
            Err(XpbdError::InvalidConstraint)
        ));
    }
    let points = SoftBody::new(c.particles().to_vec(), vec![], c.config()).unwrap();
    assert!(matches!(
        points.with_pad_samples(vec![SAMPLE]),
        Err(XpbdError::InvalidConstraint)
    ));
    // The checkpoint schema stores the original material law, not the sampled one.
    #[cfg(feature = "one-way-checkpoint")]
    {
        assert!(c.one_way_identity().is_ok());
        let custom = c.with_pad_samples(vec![SAMPLE]).unwrap();
        assert!(matches!(
            custom.one_way_identity(),
            Err(XpbdError::MaterialCheckpoint)
        ));
        assert!(custom.one_way_state().is_err());
    }
}

#[test]
fn prescribed_material_points_transmit_load_and_sliding_friction_without_accelerating() {
    let original = cloth(0., 4);
    for speed in [0., 1.] {
        let mut nodes = original.particles().to_vec();
        for p in &mut nodes {
            p.inverse_mass = 0.;
            p.velocity = [0., speed, 0.];
        }
        let mut b = SoftBody::cloth(nodes.clone(), 2, 0.1, 0.1, 1., original.config())
            .unwrap()
            .with_pad_samples(vec![SAMPLE])
            .unwrap()
            .with_pad_loads();
        let jaw = pad([-0.039, 0.42, 0.42], [0.03, 0.5, 0.5]);
        run(&mut b, &[jaw], 0.001).unwrap();
        let expected = DVec3::new(-0.001, 0.0003 * speed, 0.);
        assert!(expected.distance(b.last_boundary_impulses()[0].linear.into()) < 1e-14);
        for p in b.particles() {
            assert_eq!(p.velocity, [0., speed, 0.]);
        }
    }
}

#[test]
fn unrepresentable_quadrature_compliance_refuses_without_committing_state() {
    let mut b = cloth(0., 1)
        .with_pad_samples(vec![PadSample {
            stiffness_weight: f64::MIN_POSITIVE,
            ..SAMPLE
        }])
        .unwrap();
    let state = b.particles().to_vec();
    let jaw = pad([-0.039, 0., 0.], [0.03, 0.2, 0.2]);
    assert_eq!(run(&mut b, &[jaw], 100.), Err(XpbdError::Numerical));
    assert_eq!(state, b.particles());
}
