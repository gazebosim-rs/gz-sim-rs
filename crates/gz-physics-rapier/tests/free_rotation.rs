// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, Inertia, RotationSpec, Shape};
use gz_physics_rapier::RapierBackend;
use rapier3d_f64::prelude::{Rotation, Vector};
use std::time::Duration;
fn body(b: &mut RapierBackend, x: f64, v: f64, w: Vector, inertia: Vector) -> BodyId {
    b.spawn_link(
        BodySpec {
            position: [x, 0., 0.],
            velocity: [v, 0., 0.],
            mass: Some(1.4),
            shape: Shape::Sphere { radius: 0.2 },
        },
        RotationSpec {
            orientation: Rotation::IDENTITY.to_array(),
            angular_velocity: w.to_array(),
        },
        Some(Inertia {
            center: [0.; 3],
            orientation: Rotation::IDENTITY.to_array(),
            tensor: [inertia.x, inertia.y, inertia.z, 0., 0., 0.],
        }),
    )
    .unwrap()
}
fn spatial(b: &RapierBackend, id: BodyId, inertia: Vector) -> (Vector, f64) {
    let s = b.state(id).unwrap();
    let q = Rotation::from_array(s.orientation);
    let w = Vector::from_array(s.angular_velocity);
    let l = q * (inertia * (q.inverse() * w));
    (l, 0.5 * l.dot(w))
}
#[test]
fn symmetric_top_tracks_its_closed_form_attitude_and_free_precession() {
    let i = Vector::new(0.055, 0.055, 0.009);
    let initial = Vector::new(4.1, -2.7, 7.3);
    let l0 = i * initial;
    for step_ms in [1, 2, 4] {
        let mut b = RapierBackend::new([0.; 3]).unwrap();
        let id = body(&mut b, 0., 0., initial, i);
        b.enable_momentum_preserving_rotation(id).unwrap();
        let mut err = 0f64;
        let mut dl = 0f64;
        let mut de = 0f64;
        for step in 1..=6000 / step_ms {
            b.step(Duration::from_millis(step_ms)).unwrap();
            let t = step as f64 * step_ms as f64 * 0.001;
            let q = Rotation::from_axis_angle(l0.normalize(), l0.length() * t / i.x)
                * Rotation::from_rotation_z((1. - i.z / i.x) * initial.z * t);
            let (l, e) = spatial(&b, id, i);
            let s = b.state(id).unwrap();
            err = err.max(q.angle_between(Rotation::from_array(s.orientation)).abs());
            dl = dl.max(l.distance(l0));
            de = de.max((e - 0.5 * l0.dot(initial)).abs());
        }
        println!(
            "symmetric top {step_ms} ms over 6 s: max attitude error={err:e} rad vector dL={dl:e} dE={de:e}"
        );
        assert!(err < 0.009 && dl < 1e-10 && de < 1e-9);
    }
}
#[test]
fn a_queued_world_torque_survives_pause_and_a_refused_step_then_applies_once() {
    let mut b = RapierBackend::new([0.; 3]).unwrap();
    let i = Vector::new(0.045, 0.067, 0.09);
    let id = body(&mut b, 0., 0., Vector::ZERO, i);
    b.enable_momentum_preserving_rotation(id).unwrap();
    b.add_world_wrench(id, [1., 0., 0.], [0., 0., 1e8]).unwrap();
    let initial = b.state(id).unwrap();
    b.step(Duration::ZERO).unwrap();
    assert_eq!(b.state(id).unwrap(), initial);
    assert!(b.step(Duration::from_secs(1)).is_err());
    assert_eq!(b.state(id).unwrap(), initial);
    b.step(Duration::from_nanos(1)).unwrap();
    let (l, _) = spatial(&b, id, i);
    assert!(l.distance(Vector::Z * 0.1) < 1e-13);
    for _ in 0..15 {
        b.step(Duration::from_millis(2)).unwrap();
    }
    assert!(spatial(&b, id, i).0.distance(l) < 1e-13);
    assert!((b.state(id).unwrap().velocity[0] - 1e-9 / 1.4).abs() < 1e-15);
    b.apply_impulse(id, [0.; 3], [0.12, -0.03, 0.04]).unwrap();
    b.step(Duration::from_millis(2)).unwrap();
    assert!(
        spatial(&b, id, i)
            .0
            .distance(l + Vector::new(0.12, -0.03, 0.04))
            < 1e-12
    );
}
#[test]
fn another_collider_restores_native_contact_and_removal_resumes_isolated_integration() {
    let i = Vector::new(0.045, 0.067, 0.09);
    let construct = |opt| {
        let mut b = RapierBackend::new([0.; 3]).unwrap();
        let a = body(&mut b, -0.4, 1.2, Vector::new(2., 3., 4.), i);
        if opt {
            b.enable_momentum_preserving_rotation(a).unwrap();
        }
        let hit = body(&mut b, 0.4, -1.2, Vector::ZERO, i);
        assert!(b.enable_momentum_preserving_rotation(hit).is_err());
        (b, a, hit)
    };
    let (mut opt, a, hit) = construct(true);
    let (mut native, c, d) = construct(false);
    for _ in 0..200 {
        opt.step(Duration::from_millis(2)).unwrap();
        native.step(Duration::from_millis(2)).unwrap();
        assert_eq!(opt.state(a), native.state(c));
        assert_eq!(opt.state(hit), native.state(d));
    }
    assert!(opt.state(a).unwrap().velocity[0] < 0.4);
    assert!(opt.remove(hit));
    assert!(native.remove(d));
    let l = spatial(&opt, a, i).0;
    for _ in 0..200 {
        opt.step(Duration::from_millis(4)).unwrap();
        native.step(Duration::from_millis(4)).unwrap();
    }
    assert!(spatial(&opt, a, i).0.distance(l) < 1e-11);
    assert!(spatial(&native, c, i).0.distance(l) > 1e-7);
}

#[cfg(feature = "partition-checkpoint")]
#[test]
fn partition_checkpoint_preserves_its_integration_contract() {
    let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let make = |x| BodySpec {
        position: [x, 0., 0.],
        velocity: [0.; 3],
        mass: Some(1.),
        shape: Shape::Box {
            half_extents: [0.4, 0.3, 0.2],
        },
    };
    let id = b
        .spawn_partition_box(make(0.), RotationSpec::default(), false, 0.)
        .unwrap();
    b.checkpoint_solver().unwrap();
    b.enable_momentum_preserving_rotation(id).unwrap();
    assert!(b.checkpoint_solver().is_err());
    // Removing the enrolled identity restores the ordinary serialization contract.
    assert!(b.remove(id));
    let replacement = b
        .spawn_partition_box(make(0.), RotationSpec::default(), false, 0.)
        .unwrap();
    assert_ne!(id, replacement);
    b.checkpoint_solver().unwrap();
    assert!(b.enable_momentum_preserving_rotation(id).is_err());
}
