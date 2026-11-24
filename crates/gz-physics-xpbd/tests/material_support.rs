// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_xpbd::{Particle, SoftBody, SolverConfig, articulated_impact::ImpactSphere};
fn cloth(v: f64) -> SoftBody {
    let nodes = [
        [-1., -1., 0.21],
        [1., -1., 0.21],
        [-1., 1., 0.21],
        [1., 1., 0.21],
    ]
    .map(|position| Particle {
        position,
        velocity: [v, 0., -0.05],
        inverse_mass: 2.,
    })
    .to_vec();
    SoftBody::cloth(
        nodes,
        2,
        1e-3,
        2e-3,
        3e-3,
        SolverConfig {
            gravity: [0.; 3],
            particle_radius: 0.01,
            substeps: 2,
            iterations: 4,
            friction: 0.32,
            self_contact: false,
        },
    )
    .unwrap()
}
#[test]
fn reused_skin_response_checks_material_velocity_geometry_native_motion_and_each_cross_block_before_application()
 {
    let sources = [
        ImpactSphere {
            center: [0.52, -0.34, 0.],
            radius: 0.2,
        },
        ImpactSphere {
            center: [-0.51, 0.33, 0.],
            radius: 0.2,
        },
    ];
    let source = cloth(0.24);
    let mut first = source.clone();
    let k = |i: usize, j: usize| {
        let a = if i == j { 0.5 } else { 0.125 };
        [[a, 0., 0.], [0., a, 0.], [0., 0., a]]
    };
    let v = [[0.; 3]; 2];
    let f = first.sphere_support_witnesses(&sources, 2e-9).unwrap();
    let basis = f.support_response(&v, k).unwrap();
    let j = basis.solve(basis.speeds()).unwrap();
    let live = f.apply_support(&basis, &j, &v, k).unwrap();
    assert!(
        live.iter()
            .all(|r| r.friction.is_some() && r.on_rigid[2] < 0.)
    );
    for changed in 0..4 {
        let mut s = if changed == 0 {
            cloth(0.25)
        } else {
            source.clone()
        };
        let prior = s.particles().to_vec();
        let mut spheres = sources;
        if changed == 1 {
            spheres[1].center[1] += 1e-6;
        }
        let mut vel = v;
        if changed == 2 {
            vel[0][0] += 1e-6;
        }
        let rows = s.sphere_support_witnesses(&spheres, 2e-9).unwrap();
        assert!(
            rows.apply_support(&basis, &j, &vel, |i, b| {
                let mut w = k(i, b);
                if changed == 3 && i == 1 && b == 0 {
                    w[0][1] += 1e-9;
                }
                w
            })
            .is_err()
        );
        assert_eq!(prior, s.particles());
    }
    let mut again = source.clone();
    assert_eq!(
        again
            .sphere_support_witnesses(&sources, 2e-9)
            .unwrap()
            .apply_support(&basis, &j, &v, k)
            .unwrap(),
        live
    );
    assert_eq!(again.particles(), first.particles());
}

#[test]
fn a_separate_closer_fold_cannot_supply_clearance_to_the_selected_face() {
    use gz_physics_xpbd::MaterialFeature;
    let skin = 0.006;
    let radius = 0.12;
    let ridge = (radius + skin) * 1.25_f64.sqrt();
    let particles = [
        [-0.31, 0., ridge],
        [0., -0.24, ridge - 0.12],
        [0., 0.24, ridge - 0.12],
        [0.31, 0., ridge],
    ]
    .map(|position| Particle {
        position,
        velocity: [0.; 3],
        inverse_mass: 10.,
    })
    .to_vec();
    let soft = SoftBody::cloth(
        particles,
        2,
        0.001,
        0.001,
        0.0008,
        SolverConfig {
            gravity: [0., 0., -9.81],
            particle_radius: skin,
            substeps: 2,
            iterations: 6,
            friction: 0.,
            self_contact: false,
        },
    )
    .unwrap();
    let contact = [
        (0, MaterialFeature::Triangle([0, 1, 3])),
        (0, MaterialFeature::Triangle([0, 2, 3])),
    ];
    let delta = 0.015;
    let expected = delta * 0.5 / 1.25_f64.sqrt();
    let cut = soft
        .articulated_separations(&[ImpactSphere {
            center: [0., delta, 0.],
            radius,
        }])
        .unwrap();
    let (both, out) = cut.support_clearances(&contact).unwrap();
    assert!(
        (both[0] - expected).abs() < 2e-15
            && (both[1] + expected).abs() < 2e-15
            && out.is_infinite()
    );
    let (one, out) = cut.support_clearances(&contact[..1]).unwrap();
    assert_eq!(one, [both[0]]);
    assert_eq!(out, both[1]);
    assert!(
        cut.support_clearances(&[(0, MaterialFeature::Segment([1, 2]))])
            .is_err()
    );
}
