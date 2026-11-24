// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::{IsotropicFriction, MaterialLinkMotion};

fn belt(end: PathEnd) -> (RapierBackend, BodyId, BodyId) {
    let mut w = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
    let mass = Some(0.5);
    let ball = w
        .spawn_link(
            BodySpec {
                position: [0., 0., 0.25],
                velocity: [0.; 3],
                mass,
                shape: Shape::Sphere { radius: 0.04 },
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.; 3],
                orientation: [0., 0., 0., 1.],
                tensor: [0.0004, 0.0004, 0.0004, 0., 0., 0.],
            }),
        )
        .unwrap();
    let kinematic = w
        .spawn_material_link(
            BodySpec {
                position: [0., 0., 0.],
                velocity: [0.; 3],
                mass: None,
                shape: Shape::Box {
                    half_extents: [0.5, 0.4, 0.04],
                },
            },
            RotationSpec::default(),
            None,
            MaterialLinkMotion::Prescribed,
            IsotropicFriction::new(0.5).unwrap(),
        )
        .unwrap();
    let spline = TranslationSpline::new(
        &[(0, 0.), (20, 0.002), (40, 0.)].map(|(ms, x)| Waypoint {
            time: Duration::from_millis(ms),
            displacement: [x, 0., 0.],
            velocity: [0.; 3],
        }),
        end,
    )
    .unwrap();
    w.set_initial_translation_path(kinematic, spline).unwrap();
    w.prepare_trajectory_owner().unwrap();
    (w, ball, kinematic)
}
fn seal(bytes: &mut [u8]) {
    let digest = Sha256::digest(&bytes[40..]);
    bytes[8..40].copy_from_slice(&digest);
}

#[test]
fn rebuilt_belt_distinguishes_hold_and_repeat_and_supports_exact_warm_resume() {
    for end in [PathEnd::Hold, PathEnd::Repeat] {
        let (mut w, ball, k) = belt(end);
        let cold = w.trajectory_construction().unwrap();
        let mut other = RapierBackend::from_trajectory_construction(&cold).unwrap();
        assert_eq!(other.trajectory_construction().unwrap(), cold);
        let dt = Duration::from_millis(1);
        for _ in 0..20 {
            w.step(dt).unwrap();
        }
        let snap = w.checkpoint_trajectory().unwrap();
        other.restore_trajectory(&snap).unwrap();
        for _ in 20..60 {
            w.step(dt).unwrap();
            other.step(dt).unwrap();
            assert_eq!(other.state(k), w.state(k));
            assert_eq!(other.state(ball), w.state(ball));
        }
        let x = w.state(k).unwrap().position[0];
        assert!((x - if end == PathEnd::Hold { 0. } else { 0.002 }).abs() < 1e-14);
    }
}

#[test]
fn clocked_chord_bounds_the_midpoint_and_rejects_unrepresentable_deadlines() {
    let (mut w, ball, k) = belt(PathEnd::Hold);
    let dt = Duration::from_millis(10);
    let (v, r) = w.curves.chord(k, dt).unwrap().unwrap();
    let mid = 0.5 * dt.as_secs_f64();
    let p = w.curves.tracks[&k].path.sample(dt / 2);
    assert!(v.x > 0. && r > 0.);
    assert!((Vector::from(p.displacement) - v * mid).length() <= r);
    assert!(w.curves.chord(ball, dt).unwrap().is_none());
    assert!(w.curves.chord(k, Duration::ZERO).is_err());
    w.curves.time = Duration::MAX;
    assert!(w.curves.chord(k, dt).is_err());
    assert!(w.curves.plan(dt, 4).is_err());
    w.curves.override_link(k);
    assert!(w.curves.chord(k, dt).unwrap().is_none());
    assert!(w.curves.plan(dt, 4).unwrap().is_empty());
}

#[test]
fn checksum_valid_but_inconsistent_cold_construction_is_rejected() {
    let (w, _, _) = belt(PathEnd::Repeat);
    let seed = w.trajectory_construction().unwrap();
    type Header = (Vector, IntegrationParameters, u64, u32);
    let rejected = |edit: fn(&mut Header, &mut Row), trailer: Option<(u32, bool)>| {
        let mut src = &seed[72..];
        let mut header: Header = read(&mut src).unwrap();
        let mut row: Row = read(&mut src).unwrap();
        let knots: (u32, bool) = read(&mut src).unwrap();
        edit(&mut header, &mut row);
        let mut bytes = seed[..72].to_vec();
        append(&mut bytes, &header).unwrap();
        append(&mut bytes, &row).unwrap();
        append(&mut bytes, &trailer.unwrap_or(knots)).unwrap();
        bytes.extend_from_slice(src);
        seal(&mut bytes);
        assert!(RapierBackend::from_trajectory_construction(&bytes).is_err());
    };
    rejected(|h, _| h.3 = 0, None);
    rejected(|h, _| h.0.x = f64::INFINITY, None);
    rejected(|h, r| r.id = h.2, None);
    rejected(|_, r| r.mass = Some(0.), None);
    rejected(|_, r| r.orientation = [0.; 4], None);
    rejected(|_, r| r.inertia.as_mut().unwrap().2[0] = -1., None);
    rejected(|_, _| {}, Some((0, true)));
    rejected(|_, _| {}, Some((u32::MAX, false)));
    let mut trailing = seed;
    trailing.push(0);
    seal(&mut trailing);
    assert!(RapierBackend::from_trajectory_construction(&trailing).is_err());
}

#[test]
fn rejected_warm_snapshot_leaves_the_native_body_graph_and_clock_untouched() {
    let (mut w, ball, k) = belt(PathEnd::Hold);
    let start = w.checkpoint_trajectory().unwrap();
    let mut source = &start[72..];
    type Header = (Duration, bool, u64, u64, u32, u32);
    let h: Header = read(&mut source).unwrap();
    assert_eq!(h.5, 0);
    let install = |w: &mut RapierBackend, header: Header, data: &[u8]| {
        let mut image = start[..72].to_vec();
        append(&mut image, &header).unwrap();
        image.extend_from_slice(data);
        seal(&mut image);
        assert!(w.install_trajectory(&image).is_err());
        assert_eq!(w.checkpoint_trajectory().unwrap(), start);
    };
    for edit in [0, 1, 2, 3, 4, 5, 6] {
        let mut header = h;
        match edit {
            0 => header.0 = Duration::from_nanos(5),
            1 => header.2 = 1u64 << 63,
            2 => header.4 = u32::MAX,
            3 => header.5 = u32::MAX,
            4 => header.2 |= 1u64 << ball.raw(),
            5 => header.3 |= 1u64 << k.raw(),
            _ => header.3 = 1u64 << 63,
        }
        install(&mut w, header, source);
    }
    for edit in 0..6 {
        let mut data = source;
        let mut graph: PhysicsWorld = read(&mut data).unwrap();
        let ball = w.bodies[&ball].0;
        let belt = w.bodies[&k].0;
        match edit {
            0 => graph.gravity.x = 0.11,
            1 => graph.bodies[ball].set_enabled(false),
            2 => graph.bodies[belt].set_linvel(Vector::Z, true),
            3 => graph.bodies[ball].add_force(Vector::X, true),
            4 => graph.bodies[ball].set_body_type(RigidBodyType::Fixed, true),
            _ => graph.colliders[graph.bodies[belt].colliders()[0]].set_enabled(false),
        }
        let mut bytes = vec![];
        append(&mut bytes, &graph).unwrap();
        install(&mut w, h, &bytes);
    }
    w.step(Duration::from_millis(1)).unwrap();
    assert!(w.restore_trajectory(&start).is_err());
}
