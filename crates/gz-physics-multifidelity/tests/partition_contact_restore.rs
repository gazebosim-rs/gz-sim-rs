// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#![cfg(feature = "partition-checkpoint")]
use gz_physics::{BodySpec, DynamicBackend, PhysicsError, Shape};
use gz_physics_multifidelity::{MultiFidelityBackend, NativeSolverProfile};
use std::time::Duration;

const COARSE: Duration = Duration::from_millis(10);
const FINE: Duration = Duration::from_millis(1);

fn box_at(x: f64, z: f64) -> BodySpec {
    BodySpec {
        shape: Shape::Box {
            half_extents: [0.1; 3],
        },
        mass: Some(1.),
        position: [x, 0., z],
        velocity: [0.; 3],
    }
}
#[test]
fn ground_reaction_and_warm_geometry_survive_restoring_a_loaded_partition()
-> Result<(), PhysicsError> {
    let profile = NativeSolverProfile::Standard32;
    let mut w = MultiFidelityBackend::new_with_profile([0.; 3], profile)?;
    w.set_partition_step(FINE)?;
    w.set_partition_ground_material(0., 0.4)?;
    let lo = w.spawn_partition_box(box_at(0., 0.101), Default::default(), false, 0.4)?;
    let hi = w.spawn_partition_primitive(box_at(0., 0.330), Default::default(), false, 0.4)?;
    assert_eq!(w.partition_box(lo), Some(([0.2; 3], 1.)));
    let Some((Shape::Box { half_extents }, mass)) = w.partition_shape(hi) else {
        panic!("lost box geometry")
    };
    assert_eq!((half_extents, mass), ([0.1; 3], 1.));
    assert_eq!(w.partition_friction(hi), Some(0.4));
    assert_eq!(w.partition_ground_height()?, Some(0.));
    assert_eq!(w.partition_ground_material()?, Some((0., 0.4)));
    w.set_partition_gravity([0., 0., -9.81])?;
    for _ in 0..200 {
        w.step(FINE)?;
    }
    let mut pairs = Vec::new();
    let mut support = Vec::new();
    w.read_partition_contacts(&mut pairs, Some(&mut support));
    assert_eq!(pairs.len(), 1);
    assert_eq!(support.len(), 1);
    assert_eq!(support[0].body, lo);
    let vertical = support[0].normal_impulse;
    assert!((vertical - 2. * 9.81 * FINE.as_secs_f64()).abs() < 0.0002);
    let patch = w.partition_ground_patches();
    assert_eq!(patch.len(), 1);
    assert_eq!(patch[0].body, lo);
    assert!(
        (patch[0]
            .points
            .iter()
            .map(|p| p.normal_impulse)
            .sum::<f64>()
            - vertical)
            .abs()
            < 1e-10
    );
    assert_eq!(w.partition_ground_contacts()[0].body, lo);
    w.set_partition_gravity([0.; 3])?;
    w.finish_refined_partition_step();
    assert!(w.partition_step_matches(COARSE));
    let image = w.checkpoint_boxes()?;
    assert_eq!(image, w.checkpoint_solver()?);
    let mut restored = MultiFidelityBackend::restore_trusted_boxes(&image)?;
    assert_eq!(restored.partition_box_count(), 2);
    assert_eq!(restored.partition_ground_patches(), patch);
    assert_eq!(restored.partition_warmstart(), w.partition_warmstart());
    for b in [&mut w, &mut restored] {
        b.set_partition_gravity([0., 0., -9.81])?;
        b.step(COARSE)?;
    }
    assert_eq!(w.state(lo), restored.state(lo));
    assert_eq!(w.state(hi), restored.state(hi));
    assert_eq!(
        w.partition_ground_patches(),
        restored.partition_ground_patches()
    );
    Ok(())
}

#[test]
fn join_preserves_each_donors_shape_and_remaps_its_local_id() -> Result<(), PhysicsError> {
    for profile in [
        NativeSolverProfile::Standard32,
        NativeSolverProfile::Extended64PerPoint,
    ] {
        let build = |x| -> Result<_, PhysicsError> {
            let mut w = MultiFidelityBackend::new_with_profile([0.; 3], profile)?;
            w.set_partition_step(COARSE)?;
            w.set_partition_plane([0.; 3], [0., 0., 1.], 0.)?;
            let id = w.spawn_awake_frictionless_box(box_at(x, 0.5), Default::default(), false)?;
            Ok((w, id))
        };
        let (resident, a) = build(0.)?;
        let (donor, b) = build(1.)?;
        assert_eq!(a, b);
        let image = donor.checkpoint_solver_with_profile(profile)?;
        let copy = MultiFidelityBackend::restore_solver_with_profile(&image, profile)?;
        assert_eq!(copy.state(b), donor.state(b));
        if profile == NativeSolverProfile::Standard32 {
            assert_eq!(
                MultiFidelityBackend::restore_trusted_solver(&image)?.state(b),
                copy.state(b)
            );
        }
        let (mut joined, ids) = if profile == NativeSolverProfile::Standard32 {
            resident.join_box_solvers(&[&donor])?
        } else {
            resident.join_box_solvers_with_profile(&[&donor], profile)?
        };
        let peer = ids[0][&b];
        assert_ne!(a, peer);
        assert_eq!(joined.state(a), resident.state(a));
        assert_eq!(joined.state(peer), donor.state(b));
        assert_eq!(joined.partition_plane()?.unwrap().normal, [0., 0., 1.]);
        joined.step(COARSE)?;
        assert_eq!(joined.active_contact_pairs(), 0);
        assert!(
            resident
                .join_box_solvers_with_profile(&[&resident], profile)
                .is_err()
        );
    }
    Ok(())
}
