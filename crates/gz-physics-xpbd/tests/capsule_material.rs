// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3};
use gz_physics_xpbd::{
    Boundary, CapsuleRegion, MaterialFeature, Particle, SoftBody, SolverConfig, XpbdError,
};
use std::time::Duration;
fn particle(q: DVec3, v: DVec3, m: f64) -> Particle {
    Particle {
        position: q.to_array(),
        velocity: v.to_array(),
        inverse_mass: m.recip(),
    }
}
fn cfg(n: u32) -> SolverConfig {
    SolverConfig {
        substeps: n,
        iterations: 1,
        particle_radius: 0.02,
        gravity: [0.; 3],
        friction: 0.,
        self_contact: false,
    }
}
fn guide(center: DVec3, axis: DVec3, v: DVec3) -> Boundary {
    Boundary::Capsule {
        center: center.to_array(),
        axis: axis.to_array(),
        radius: 0.08,
        half_length: 0.2,
        velocity: v.to_array(),
        angular_speed: 0.,
    }
}
fn momentum(b: &SoftBody) -> (DVec3, DVec3) {
    let mut p = DVec3::ZERO;
    let mut l = DVec3::ZERO;
    for a in b.particles() {
        let mv = DVec3::from(a.velocity) / a.inverse_mass;
        p += mv;
        l += DVec3::from(a.position).cross(mv);
    }
    (p, l)
}
fn close(a: DVec3, b: DVec3, tol: f64) {
    assert!(a.distance(b) <= tol, "{a:?} vs {b:?} tol={tol}");
}
#[test]
fn cable_barrel_load_has_the_analytic_time_and_the_two_nodal_impulses() {
    let turn = DQuat::from_rotation_y(-0.22) * DQuat::from_rotation_x(0.63);
    let place = DVec3::new(3.1, -0.7, 0.4);
    let drift = DVec3::new(0.13, 0.24, -0.17);
    for n in [1, 2, 4, 8] {
        let c = cfg(n);
        let base = SoftBody::cable(
            [(-0.6, 0.25), (0.2, 0.75)]
                .into_iter()
                .map(|(x, m)| {
                    particle(
                        place + turn * DVec3::new(x, 0.10, 0.3),
                        drift - turn * DVec3::Z * 2.,
                        m,
                    )
                })
                .collect(),
            0.,
            0.,
            c,
        )
        .unwrap();
        let g = guide(place, turn * DVec3::Y, drift);
        let mut control = base.clone();
        control.step(Duration::from_millis(200), &[g]).unwrap();
        assert_eq!(control.last_boundary_impulses()[0].linear, [0.; 3]);
        let mut body = base.with_capsule_material_contact().unwrap();
        let (p0, l0) = momentum(&body);
        body.step(Duration::from_millis(200), &[g]).unwrap();
        let (p1, l1) = momentum(&body);
        let reaction = body.last_boundary_impulses()[0];
        close(p1 - p0, -DVec3::from(reaction.linear), 1e-12);
        close(
            l1 - l0,
            -DVec3::from(reaction.angular) - place.cross(reaction.linear.into()),
            1e-12,
        );
        let [load] = body.last_capsule_material_loads() else {
            panic!("{:?}", body.last_capsule_material_loads());
        };
        assert_eq!(load.feature, MaterialFeature::Segment([0, 1]));
        assert_eq!(load.region, CapsuleRegion::Barrel);
        assert!((load.first_time - 0.1).abs() < 1e-10);
        close(load.linear.into(), -2. * (turn * DVec3::Z), 1e-10);
        for (i, m) in [0.25, 0.75].into_iter().enumerate() {
            close(body.particles()[i].velocity.into(), drift, 1e-10);
            close(load.nodal[i].into(), 2. * m * (turn * DVec3::Z), 1e-10);
        }
        println!(
            "cable {n} substeps: time={} s, support={:?}, angular closure={:e}",
            load.first_time,
            load.linear,
            (l1 - l0 + DVec3::from(reaction.angular) + place.cross(reaction.linear.into()))
                .length()
        );
    }
}
fn cloth(n: u32) -> SoftBody {
    let f = [0.20, 0.35, 0.9, 0.45];
    let p = [(-0.48, -0.38), (0.48, -0.38), (-0.48, 0.38), (0.48, 0.38)]
        .into_iter()
        .zip(f)
        .map(|((x, y), m)| particle(DVec3::new(x, y, 0.3), -2. * DVec3::Z, m))
        .collect();
    SoftBody::cloth(p, 2, 1e8, 1e8, 1e8, cfg(n)).unwrap()
}
#[test]
fn cap_first_hits_face_interior_and_transfers_the_barycentric_load() {
    let center = DVec3::new(0.288, -0.038, -0.2);
    let g = guide(center, DVec3::Z, DVec3::ZERO);
    for n in [1, 2, 4, 8] {
        let base = cloth(n);
        let mut control = base.clone();
        control.step(Duration::from_millis(180), &[g]).unwrap();
        assert_eq!(control.last_boundary_impulses()[0].linear, [0.; 3]);
        let mut body = base.with_capsule_material_contact().unwrap();
        let (p0, l0) = momentum(&body);
        body.step(Duration::from_millis(180), &[g]).unwrap();
        let (p, l) = momentum(&body);
        let loads = body.last_capsule_material_loads();
        let [load] = loads else {
            panic!("{loads:?}");
        };
        assert_eq!(load.feature, MaterialFeature::Triangle([0, 1, 3]));
        assert_eq!(load.region, CapsuleRegion::PositiveCap);
        assert!((load.first_time - 0.1).abs() < 1e-9);
        close(load.linear.into(), -DVec3::Z * 2., 1e-5);
        for (i, m) in [0.20, 0.35, 0.45].into_iter().enumerate() {
            close(load.nodal[i].into(), DVec3::Z * 2. * m, 1e-5);
        }
        close(p - p0, -DVec3::from(load.linear), 1e-12);
        close(
            l - l0,
            -DVec3::from(load.angular) - center.cross(load.linear.into()),
            1e-10,
        );
        println!(
            "cloth {n} substeps: time={} s, support={:?} triangle loads={:?}",
            load.first_time, load.linear, load.nodal
        );
    }
}
#[test]
fn malformed_or_unsupported_material_contact_and_pierced_faces_refuse_atomically() {
    let mut b = cloth(1).with_capsule_material_contact().unwrap();
    let ok = guide(DVec3::new(0.288, -0.038, -0.2), DVec3::Z, DVec3::ZERO);
    b.step(Duration::from_millis(150), &[ok]).unwrap();
    let state = b.particles().to_vec();
    let loads = b.last_capsule_material_loads().to_vec();
    let reactions = b.last_boundary_impulses().to_vec();
    let bad = guide(DVec3::new(0.288, -0.038, 0.1), DVec3::Z, DVec3::ZERO);
    assert_eq!(
        b.step(Duration::from_millis(15), &[bad]),
        Err(XpbdError::UnresolvedSurface)
    );
    assert_eq!(b.particles(), state);
    assert_eq!(b.last_capsule_material_loads(), loads);
    assert_eq!(b.last_boundary_impulses(), reactions);
    b.step(Duration::ZERO, &[ok]).unwrap();
    assert!(b.last_capsule_material_loads().is_empty());
    let p = particle(DVec3::ZERO, DVec3::ZERO, 1.);
    assert!(
        SoftBody::new(vec![p], vec![], cfg(1))
            .unwrap()
            .with_capsule_material_contact()
            .is_err()
    );
    let mut c = cfg(1);
    c.friction = 0.2;
    assert!(
        SoftBody::cable(vec![p, particle(DVec3::X, DVec3::ZERO, 1.)], 0., 0., c)
            .unwrap()
            .with_capsule_material_contact()
            .is_err()
    );
}

#[test]
fn parallel_cable_has_symmetric_support_across_its_contact_interval() {
    for span in [0.12, 0.4] {
        let pts = [-span, span]
            .map(|x| particle(DVec3::new(x, 0., 0.3), -2. * DVec3::Z, 0.5))
            .to_vec();
        let mut b = SoftBody::cable(pts, 0., 0., cfg(1))
            .unwrap()
            .with_capsule_material_contact()
            .unwrap();
        b.step(
            Duration::from_millis(180),
            &[guide(DVec3::ZERO, DVec3::X, DVec3::ZERO)],
        )
        .unwrap();
        let load = b.last_boundary_impulses()[0];
        println!(
            "parallel span {span}: loads {:?}",
            b.last_capsule_material_loads()
        );
        close(load.linear.into(), -DVec3::Z * 2., 1e-10);
        close(load.angular.into(), DVec3::ZERO, 1e-10);
        for (p, x) in b.particles().iter().zip([-span, span]) {
            close(p.position.into(), DVec3::new(x, 0., 0.1), 1e-10);
            close(p.velocity.into(), DVec3::ZERO, 1e-10);
        }
    }
}

#[test]
fn prescribed_degrees_of_freedom_and_separating_capsule_are_handled() {
    let guide_still = guide(DVec3::ZERO, DVec3::Y, DVec3::ZERO);
    let mut p = [-0.4, 0.4]
        .map(|x| particle(DVec3::new(x, 0., 0.3), -2. * DVec3::Z, 1.))
        .to_vec();
    for v in &mut p {
        v.inverse_mass = 0.;
    }
    let mut fixed = SoftBody::cable(p, 0., 0., cfg(1))
        .unwrap()
        .with_capsule_material_contact()
        .unwrap();
    let initial = fixed.particles().to_vec();
    assert_eq!(
        fixed.step(Duration::from_millis(180), &[guide_still]),
        Err(XpbdError::UnresolvedSurface)
    );
    assert_eq!(fixed.particles(), initial);
    assert!(fixed.last_capsule_material_loads().is_empty());
    for z in [0.10, 0.100001] {
        let p = [-0.4, 0.4]
            .map(|x| particle(DVec3::new(x, 0., z), DVec3::Z, 1.))
            .to_vec();
        let mut b = SoftBody::cable(p, 0., 0., cfg(1))
            .unwrap()
            .with_capsule_material_contact()
            .unwrap();
        b.step(Duration::from_millis(180), &[guide_still]).unwrap();
        assert_eq!(b.last_boundary_impulses()[0].linear, [0.; 3]);
        assert!(b.last_capsule_material_loads().is_empty());
    }
}
