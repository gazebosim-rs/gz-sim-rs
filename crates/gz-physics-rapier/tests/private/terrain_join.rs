// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn floor(h: f64) -> RapierBackend {
    let mut world = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    world.set_partition_ground(h).unwrap();
    world
}
fn place(world: &mut RapierBackend, x: f64, yaw: f64, mass: f64) -> BodyId {
    world
        .spawn_awake_frictionless_box(
            BodySpec {
                shape: Shape::Box {
                    half_extents: [0.31, 0.22, 0.18],
                },
                position: [x, 0., 0.57],
                velocity: [0.045, 0., 0.],
                mass: Some(mass),
            },
            RotationSpec {
                orientation: [0., 0., (yaw / 2.).sin(), (yaw / 2.).cos()],
                angular_velocity: [0.; 3],
            },
            false,
        )
        .unwrap()
}
fn advance(world: &mut RapierBackend) {
    world.set_partition_gravity([0., 0., -9.81]).unwrap();
    world.step(Duration::from_millis(5)).unwrap();
    world.set_partition_gravity([0.; 3]).unwrap();
}
/// Canonical box-local point, retained normal warm start and last total impulse.
fn points(s: &RapierBackend, id: BodyId) -> Vec<([u64; 3], u64, u64)> {
    let g = ground_of(&s.world).unwrap().unwrap().0;
    let c = s.world.bodies[s.bodies[&id].0].colliders()[0];
    let p = s.world.narrow_phase.contact_pair(g, c).unwrap();
    let mut points: Vec<_> = p
        .solver_manifolds()
        .iter()
        .flat_map(|m| &m.points)
        .map(|v| {
            let local = if p.collider1 == c {
                v.local_p1
            } else {
                v.local_p2
            };
            (
                local.to_array().map(f64::to_bits),
                v.data.warmstart_impulse.to_bits(),
                v.data.impulse.to_bits(),
            )
        })
        .collect();
    points.sort_unstable();
    points
}
#[test]
fn all_donor_ground_points_and_warm_starts_survive_one_shared_plane() {
    let mut resident = floor(0.25);
    let mut donor = floor(0.25);
    let mut control = floor(0.25);
    let a = place(&mut resident, -5., -0.4, 0.75);
    let b = place(&mut donor, 5., 0.53, 1.6);
    let ca = place(&mut control, -5., -0.4, 0.75);
    let cb = place(&mut control, 5., 0.53, 1.6);
    for _ in 0..85 {
        advance(&mut resident);
        advance(&mut donor);
        advance(&mut control);
    }
    for (s, id, c) in [(&resident, a, ca), (&donor, b, cb)] {
        let measured = points(s, id);
        assert_eq!(measured.len(), 4);
        assert!(measured.iter().all(|p| f64::from_bits(p.1) > 0.));
        assert_eq!(measured, points(&control, c));
    }
    let a_before = resident.checkpoint_solver().unwrap();
    let b_before = donor.checkpoint_solver().unwrap();
    let changed = floor(0.2);
    assert!(resident.join_box_solvers(&[&changed]).is_err());
    let no_floor = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    assert!(resident.join_box_solvers(&[&no_floor]).is_err());
    assert_eq!(resident.checkpoint_solver().unwrap(), a_before);
    assert_eq!(donor.checkpoint_solver().unwrap(), b_before);
    let (mut joined, maps) = resident.join_box_solvers(&[&donor]).unwrap();
    assert_eq!(joined.world.colliders.len(), 3);
    assert!(joined.checkpoint_solver().is_err());
    for tick in 1..=45 {
        advance(&mut joined);
        advance(&mut control);
        for (actual, reference) in [(a, ca), (maps[0][&b], cb)] {
            assert_eq!(
                points(&joined, actual),
                points(&control, reference),
                "step {tick}"
            );
            assert_eq!(joined.state(actual), control.state(reference));
        }
        if tick == 5 {
            joined = RapierBackend::restore_trusted_solver(&joined.checkpoint_solver().unwrap())
                .unwrap();
        }
    }
}

#[test]
fn direct_native_reset_and_captured_initial_state_preserve_their_own_ground() {
    let mut solver = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let empty = solver.initial_state();
    solver.set_partition_ground(0.25).unwrap();
    let body = place(&mut solver, 0., 0., 1.);
    let initial = solver.initial_state();
    let start = solver.state(body).unwrap();
    for _ in 0..80 {
        advance(&mut solver);
    }
    assert!(!solver.partition_ground_contacts().is_empty());
    solver.reset().unwrap();
    assert_eq!(solver.partition_ground_height().unwrap(), Some(0.25));
    assert_eq!(solver.state(body), Some(start));
    assert!(solver.partition_ground_contacts().is_empty());
    let clear = solver.prepare_initial(&empty).unwrap();
    solver.apply_initial(clear).unwrap();
    assert_eq!(solver.partition_ground_height().unwrap(), None);
    let seed = solver.prepare_initial(&initial).unwrap();
    solver.apply_initial(seed).unwrap();
    assert_eq!(solver.partition_ground_height().unwrap(), Some(0.25));
    assert_eq!(solver.state(body), Some(start));
}

#[test]
fn batched_readback_reuses_buffers_and_matches_rigid_and_ground_queries() {
    let mut rigid = Vec::new();
    let mut supports = Vec::new();
    let rigid_bits = |items: &[RigidContactObservation]| {
        items
            .iter()
            .map(|p| (p.a, p.b, p.active, p.normal_impulse.to_bits()))
            .collect::<Vec<_>>()
    };
    let ground_bits = |items: &[NativeGroundContact]| {
        items
            .iter()
            .map(|p| (p.body, p.active, p.normal_impulse.to_bits()))
            .collect::<Vec<_>>()
    };
    for additional in [0, 7] {
        let mut solver = floor(0.);
        solver.read_partition_contacts(&mut rigid, Some(&mut supports));
        assert!(rigid.is_empty() && supports.is_empty());
        for i in 0..2 + additional {
            let (x, z) = match i {
                0 => (0., 0.12),
                1 => (0., 0.4),
                n => (n as f64, 1.5),
            };
            solver
                .spawn_partition_primitive(
                    BodySpec {
                        shape: Shape::Sphere { radius: 0.10 },
                        position: [x, 0., z],
                        velocity: [0.; 3],
                        mass: Some(1.),
                    },
                    RotationSpec {
                        orientation: [0., 0., 0., 1.],
                        angular_velocity: [0.; 3],
                    },
                    false,
                    0.,
                )
                .unwrap();
        }
        solver.set_partition_gravity([0., 0., -9.81]).unwrap();
        for _ in 0..250 {
            solver.step(Duration::from_millis(1)).unwrap();
            solver.read_partition_contacts(&mut rigid, Some(&mut supports));
            assert_eq!(
                rigid_bits(&rigid),
                rigid_bits(&solver.rigid_contact_observations())
            );
            assert_eq!(
                ground_bits(&supports),
                ground_bits(&solver.partition_ground_contacts())
            );
            solver.read_partition_contacts(&mut rigid, None);
            assert_eq!(
                rigid_bits(&rigid),
                rigid_bits(&solver.rigid_contact_observations())
            );
        }
        assert!(rigid.iter().any(|p| p.active && p.normal_impulse > 0.));
        assert!(supports.iter().any(|p| p.active && p.normal_impulse > 0.));
    }
}
