// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_xpbd::SolverConfig;

fn material(x: [f64; 3], v: [f64; 3]) -> SoftBody {
    SoftBody::new(
        vec![Particle {
            position: x,
            velocity: v,
            inverse_mass: 1.,
        }],
        vec![],
        SolverConfig {
            gravity: [0., 0., -9.81],
            substeps: 1,
            iterations: 2,
            particle_radius: 0.01,
            self_contact: false,
            friction: 0.,
        },
    )
    .unwrap()
}
fn deck() -> Boundary {
    Boundary::StaticCuboid {
        center: [0., 0., -0.1],
        half_extents: [0.1; 3],
        orientation: [0., 0., 0., 1.],
    }
}
fn surfaces() -> [Surface; 2] {
    [
        Surface::Rigid {
            link: 1,
            body: BodyId::from_raw(1),
            shape: ReceiverShape::Cuboid([0.1; 3]),
            mobility: None,
            one_way_box: Some(BoxMotion::Fixed(Transform::default())),
        },
        Surface::Rigid {
            link: 2,
            body: BodyId::from_raw(2),
            shape: ReceiverShape::Cuboid([0.05; 3]),
            mobility: Some(ReceiverMass::ArticulatedWrench {
                normal_compliance: 0.001,
                normal_damping: 1.,
            }),
            one_way_box: None,
        },
    ]
}
#[test]
fn only_incident_unheld_particles_inside_the_declared_face_request_fine_time() {
    let dt = 0.001;
    let hit = material([0., 0., 0.0107], [0., 0., -1.]);
    assert!(approaching(&hit, deck(), dt));
    for (x, v) in [
        ([0., 0., 0.02], [0., 0., -1.]),
        ([0., 0., 0.009], [0., 0., -1.]),
        ([0., 0., 0.01005], [0., 0., -0.05]),
        ([0.091, 0., 0.0107], [0., 0., -1.]),
        ([0., -0.091, 0.0107], [0., 0., -1.]),
        ([0.089, 0., 0.0107], [2., 0., -1.]),
        ([0., 0., 0.0107], [0., 0., 1.]),
    ] {
        assert!(!approaching(&material(x, v), deck(), dt));
    }
    let axis = DQuat::from_rotation_y(0.6);
    let center = DVec3::new(1., 2., 3.);
    let v = DVec3::new(0.25, -0.4, 0.);
    let tilted = Boundary::TranslatingCuboid {
        center: center.to_array(),
        half_extents: [0.1; 3],
        orientation: axis.to_array(),
        velocity: v.to_array(),
    };
    let x = center + axis * DVec3::Z * 0.1107;
    assert!(approaching(
        &material(x.to_array(), (v - axis * DVec3::Z).to_array()),
        tilted,
        dt
    ));
    let sphere = Boundary::Plane {
        point: [0.; 3],
        normal: [0., 0., 1.],
        velocity: [0.; 3],
    };
    assert!(!approaching(&hit, sphere, dt));
}

#[test]
fn missed_or_retired_geometry_and_a_failed_solve_preserve_the_original_resolution() {
    let mut p = Policy {
        receiver: 0,
        surfaces: 1,
        target_ns: 5_000,
        remaining: Duration::ZERO,
        last: Default::default(),
    };
    let mut b = material([0., 0., 0.01005], [0., 0., -1.]);
    let dt = Duration::from_micros(100);
    let start = b.particles().to_vec();
    assert!(p.solve(&mut b, dt, &[0], &[deck()], &[], &[]).is_err());
    assert_eq!(b.particles(), &start);
    assert_eq!(b.config().substeps, 1);
    assert_eq!(p.last, FiniteLandingStep::default());
    p.solve(&mut b, dt, &[0], &[deck()], &[None], &[]).unwrap();
    assert_eq!(
        p.last,
        FiniteLandingStep {
            triggered: true,
            refined: true,
            material_substeps: 20
        }
    );
    assert_eq!(b.config().substeps, 1);
    assert!(!p.remaining.is_zero());
    p.solve(&mut b, dt, &[], &[], &[], &[]).unwrap();
    assert_eq!(
        p.last,
        FiniteLandingStep {
            triggered: false,
            refined: false,
            material_substeps: 1
        }
    );
    assert!(p.remaining.is_zero());
    p.reset();
    assert_eq!(p.last, FiniteLandingStep::default());
}

#[test]
fn one_enrolled_deck_enforces_local_clock_and_budget_before_stepping() {
    let bodies = BTreeMap::from([("sheet".to_owned(), material([0.; 3], [0.; 3]))]);
    let surfaces = surfaces();
    let check = |attr: &str, s: &[Surface], e: u32| {
        let xml = format!("<plugin {attr}/>");
        let d = roxmltree::Document::parse(&xml).unwrap();
        Policy::admit(d.root_element(), &bodies, s, Duration::from_millis(1), e)
    };
    assert!(check("", &surfaces, 1).unwrap().is_none());
    assert!(
        check("landing_step_ns='1000'", &surfaces, 1)
            .unwrap()
            .is_some()
    );
    for attr in [
        "landing_step_ns='x'",
        "landing_step_ns='0'",
        "landing_step_ns='1'",
        "landing_step_ns='1000' worker_processes='2'",
    ] {
        assert!(check(attr, &surfaces, 1).is_err());
    }
    assert!(check("landing_step_ns='1000'", &surfaces, 2).is_err());
    assert!(check("landing_step_ns='1000'", &surfaces[..1], 1).is_err());
    assert!(check("landing_step_ns='1000'", &surfaces[1..], 1).is_err());
    let invalid = [
        surfaces[0],
        surfaces[1],
        Surface::Plane {
            point: [0.; 3],
            normal: [0., 0., 1.],
        },
    ];
    assert!(check("landing_step_ns='1000'", &invalid, 1).is_err());
}
