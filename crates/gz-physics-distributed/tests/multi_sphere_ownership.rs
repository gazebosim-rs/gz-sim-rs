// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    articulation::ArticulationPool,
    moving::{MovingPool, MovingSpec},
    multi_sphere::{MultiSpherePool, MultiSphereSpec},
};
use sha2::{Digest, Sha256};
use std::{fs, path::Path};

type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;
const SOURCE: &str = "distributed_hinge_two_spheres.sdf";
const XML: &str = include_str!("../../../examples/distributed_hinge_two_spheres.sdf");
const WORKER: &str = env!("CARGO_BIN_EXE_physics_worker");
fn spec() -> Result<MultiSphereSpec> {
    Ok(gz_sim_scene::distributed_hinge::distributed_multi_sphere_spec(XML, SOURCE, "default")?)
}

#[test]
fn a_contact_on_the_first_tick_and_queued_cancellation_recover_exactly() -> Result<()> {
    let d = tempfile::tempdir()?;
    let mut s = spec()?;
    s.spheres[0].position = s.hinge.links[1].position;
    s.spheres[0].position[0] += s.hinge.links[1].radius + s.spheres[0].radius - 0.0001;
    s.spheres[1].position[0] += 8.;
    let exe = Path::new(WORKER);
    let path = d.path().join("barrier.gzmultisphere");
    let mut p = MultiSpherePool::create(exe, d.path(), 3, s)?;
    p.queue_effort(s.hinge.joint_id, f64::MAX)?;
    let accepted = fs::read(&path)?;
    assert!(p.queue_effort(s.hinge.joint_id, f64::MAX).is_err());
    assert_eq!(p.pending_joint_effort(), Some(f64::MAX));
    assert_eq!(fs::read(&path)?, accepted);
    p.queue_effort(s.hinge.joint_id, -f64::MAX)?;
    drop(p);
    let mut p = MultiSpherePool::resume(exe, d.path())?;
    assert_eq!(p.pending_joint_effort(), Some(0.));
    p.advance()?;
    let [h, a, b] = p.owners();
    assert_eq!(h, a);
    assert_ne!(h, b);
    assert!(p.observation().reactions[0].iter().any(|r| r.active));
    assert_eq!(p.pending_joint_effort(), None);
    let snapshot = *p.observation();
    let image = p.image().unwrap().to_vec();
    // Moving the hinge component to an occupied independent slot atomically swaps its owner.
    p.transfer(b)?;
    assert_eq!(p.owners(), [b, b, h]);
    assert_eq!(p.image(), Some(image.as_slice()));
    assert_eq!(p.observation(), &snapshot);
    drop(p);
    let mut p = MultiSpherePool::resume(exe, d.path())?;
    assert_eq!(p.image(), Some(image.as_slice()));
    let independent = tempfile::tempdir()?;
    let reference_cut = independent.path().join("barrier.gzmultisphere");
    fs::copy(&path, &reference_cut)?;
    let mut reference = MultiSpherePool::resume(exe, independent.path())?;
    for _ in 0..4 {
        p.queue_effort(s.hinge.joint_id, 0.01)?;
        reference.queue_effort(s.hinge.joint_id, 0.01)?;
        // Lose the acknowledgement of the eligibility check that stages the hinge step.
        // Both processes must consume that private integration exactly once.
        p.drop_next_acknowledgement(b)?;
        p.advance()?;
        reference.advance()?;
        assert_eq!(p.owners(), [b, b, h]);
        assert_eq!(p.observation(), reference.observation());
        assert_eq!(p.worker_steps(b), reference.worker_steps(b));
        assert_eq!(fs::read(&path)?, fs::read(&reference_cut)?);
    }
    p.reset()?;
    assert_eq!(p.epoch(), 1);
    assert_eq!(p.observation().hinge.tick, 0);
    p.advance()?;
    assert_eq!(p.observation(), &snapshot);
    Ok(())
}

#[test]
fn profile_and_recipe_conflicts_leave_existing_archives_unchanged() -> Result<()> {
    let s = spec()?;
    let exe = Path::new(WORKER);
    let d = tempfile::tempdir()?;
    let p = MultiSpherePool::create(exe, d.path(), 3, s)?;
    assert!(MultiSpherePool::resume(exe, d.path()).is_err());
    drop(p);
    let path = d.path().join("barrier.gzmultisphere");
    let before = fs::read(&path)?;
    let single = MovingSpec {
        hinge: s.hinge,
        sphere: s.spheres[0],
    };
    assert!(MovingPool::open_scene(exe, d.path(), None, single).is_err());
    assert!(ArticulationPool::open_scene(exe, d.path(), None, s.hinge).is_err());
    assert!(MultiSpherePool::open_scene(exe, d.path(), Some(4), s).is_err());
    let mut mismatch = s;
    mismatch.spheres[1].radius *= 2.;
    assert!(MultiSpherePool::open_scene(exe, d.path(), None, mismatch).is_err());
    assert_eq!(fs::read(&path)?, before);
    fs::write(&path, &before[..before.len() - 1])?;
    assert!(MultiSpherePool::resume(exe, d.path()).is_err());
    let other = tempfile::tempdir()?;
    drop(MovingPool::create(exe, other.path(), 2, single)?);
    assert!(MultiSpherePool::open_scene(exe, other.path(), None, s).is_err());
    assert!(!other.path().join("barrier.gzmultisphere").exists());
    Ok(())
}

#[test]
fn correctly_checksummed_conflicting_receipts_fail_admission() -> Result<()> {
    let d = tempfile::tempdir()?;
    let dir = d.path();
    let path = dir.join("barrier.gzmultisphere");
    let exe = Path::new(WORKER);
    let mut s = spec()?;
    s.spheres[0].position = s.hinge.links[1].position;
    s.spheres[1].position[0] += 8.;
    let mut pool = MultiSpherePool::create(exe, dir, 3, s)?;
    pool.advance()?;
    pool.queue_effort(s.hinge.joint_id, 0.)?;
    let [hinge, a, b] = pool.owners();
    assert_eq!(hinge, a);
    assert_ne!(hinge, b);
    let spheres = pool.observation().spheres;
    drop(pool);
    let clean = fs::read(&path)?;
    // The fixed-width v1 archive ends with both public role states and the
    // pending scalar. Preserve the native graph and regenerate the outer hash.
    let tail = bincode::serialize(&(spheres, Some(0_f64)))?;
    assert!(clean.ends_with(&tail));
    let mut conflicting_resident = spheres;
    conflicting_resident[0].position[1] += 1.;
    let mut invalid_motion = spheres;
    invalid_motion[1].velocity[0] = f64::NAN;
    let mut invalid_rotation = spheres;
    invalid_rotation[1].orientation = [0.; 4];
    for (case, states, pending) in [
        ("resident copy", conflicting_resident, Some(0.0)),
        ("loose motion", invalid_motion, Some(0.0)),
        ("loose rotation", invalid_rotation, Some(0.0)),
        ("effort sign", spheres, Some(-0.0)),
    ] {
        let mut damaged = clean.clone();
        let offset = damaged.len() - tail.len();
        damaged[offset..].copy_from_slice(&bincode::serialize(&(states, pending))?);
        let digest = Sha256::digest(&damaged[40..]);
        damaged[8..40].copy_from_slice(&digest);
        fs::write(&path, &damaged)?;
        assert!(MultiSpherePool::resume(exe, dir).is_err(), "{case}");
        assert_eq!(fs::read(&path)?, damaged, "{case}");
    }
    fs::write(&path, &clean)?;
    MultiSpherePool::resume(exe, dir)?.advance()?;
    Ok(())
}

#[test]
fn oblique_sphere_island_recovers_and_returns_to_two_independent_spheres() -> Result<()> {
    let dir = tempfile::tempdir()?;
    let archived = dir.path().join("barrier.gzmultisphere");
    let worker = Path::new(WORKER);
    let mut s = spec()?;
    s.spheres[0].position = [4., 0., 0.5];
    s.spheres[1].position = [4.35, 0.17, 0.5];
    s.spheres[0].velocity = [3., 0., 0.];
    s.spheres[1].velocity = [-3., 0., 0.];
    let mut p = MultiSpherePool::create(worker, dir.path(), 3, s)?;
    assert!(p.transfer_pair(2).is_err());
    for _ in 0..90 {
        p.advance()?;
        if p.observation().sphere_reaction.normal_impulse > 0. {
            break;
        }
    }
    let [h, a, b] = p.owners();
    assert_ne!(h, a);
    assert_eq!(a, b);
    assert!(p.observation().sphere_reaction.normal_impulse > 0.);
    assert!(
        p.observation()
            .sphere_reaction
            .tangent_impulse
            .iter()
            .any(|v| *v != 0.)
    );
    p.queue_effort(s.hinge.joint_id, 0.01)?;
    let image = p.pair_image().unwrap().to_vec();
    let obs = *p.observation();
    assert!(p.transfer_pair(a).is_err());
    assert!(p.transfer_pair(3).is_err());
    p.transfer_pair(h)?;
    assert_eq!(p.owners(), [a, h, h]);
    assert_eq!(p.pair_image(), Some(image.as_slice()));
    assert_eq!(p.observation(), &obs);
    let clean = fs::read(&archived)?;
    assert_eq!(&clean[..8], b"GZ2SCU02");
    drop(p);

    // Change just the duplicate drained flag while keeping its checksum valid.
    // Only the native pair image may authorize a release.
    let mut forged = clean.clone();
    *forged.last_mut().unwrap() ^= 1;
    let digest = Sha256::digest(&forged[40..]);
    forged[8..40].copy_from_slice(&digest);
    fs::write(&archived, &forged)?;
    assert!(MultiSpherePool::resume(worker, dir.path()).is_err());
    assert_eq!(fs::read(&archived)?, forged);
    fs::write(&archived, &clean)?;
    let copy = tempfile::tempdir()?;
    let ref_path = copy.path().join("barrier.gzmultisphere");
    fs::write(&ref_path, &clean)?;
    let mut expected = MultiSpherePool::resume(worker, copy.path())?;
    let mut p = MultiSpherePool::resume(worker, dir.path())?;
    assert_eq!(p.pending_joint_effort(), Some(0.01));
    // Preview ACK loss must not integrate the pair a second time on Step.
    p.drop_next_acknowledgement(h)?;
    for _ in 0..160 {
        expected.advance()?;
        p.advance()?;
        assert_eq!(p.observation(), expected.observation());
        assert_eq!(p.worker_steps(h), expected.worker_steps(h));
        assert_eq!(fs::read(&archived)?, fs::read(&ref_path)?);
        if p.pair_image().is_none() {
            break;
        }
    }
    let [h, a, b] = p.owners();
    assert!(h != a && h != b && a != b);
    assert!(p.pair_image().is_none());
    assert_eq!(p.observation().sphere_reaction.normal_impulse, 0.);
    assert_eq!(&fs::read(&archived)?[..8], b"GZ2SCU01");
    Ok(())
}

#[test]
fn an_oblique_impulse_redirects_a_stationary_sphere_into_the_hinge_join_margin() -> Result<()> {
    let mut s = spec()?;
    s.spheres[0].position = [0.45, 0.52, 0.5];
    s.spheres[0].velocity = [50., 0., 0.];
    s.spheres[1].position = [0.65, 0.3, 0.5];
    s.spheres[1].velocity = [0.; 3];
    let clearance = |position: [f64; 3]| {
        glam::DVec3::from(position).distance(glam::DVec3::from(s.hinge.links[1].position))
            - s.spheres[1].radius
            - s.hinge.links[1].radius
    };
    // B's incoming segment is a point 30 mm clear of the child. A moves
    // along X above B, too far in Y to reach either hinge link directly.
    assert!(clearance(s.spheres[1].position) > 0.025);
    let dir = tempfile::tempdir()?;
    let mut p = MultiSpherePool::create(Path::new(WORKER), dir.path(), 3, s)?;
    assert_eq!(p.owners(), [0, 1, 2]);
    p.advance()?;
    assert_eq!(p.owners(), [0; 3]);
    assert!(p.observation().sphere_reaction.normal_impulse > 0.);
    assert!(clearance(p.observation().spheres[1].position) < 0.025);
    // The discarded pair preview holds no committed step on its former owner.
    assert_eq!(p.worker_steps(0), Some(1));
    assert_eq!(p.worker_steps(1), Some(0));
    assert_eq!(p.worker_steps(2), Some(0));
    Ok(())
}
