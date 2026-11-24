// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use sha2::{Digest, Sha256};

const IMAGE: &[u8] = include_bytes!("../../tests/fixtures/pair-warm-v1-tick-1292.gzmultisphere");

fn read_original() -> Result<(MultiSphereSpec, Cut)> {
    let dir = tempfile::tempdir()?;
    std::fs::write(dir.path().join("barrier.gzmultisphere"), IMAGE)?;
    disk::Disk::lock(dir.path(), false)?.read()
}

#[test]
fn original_pair_restores_and_follows_the_archived_monolithic_solve() -> Result<()> {
    let checked = inspect_restart_cut(IMAGE)?;
    assert_eq!(
        checked.sha256,
        "7bdbbe2133701d371b99008a012924a4dd5605b3dde04edaae34f4095ba25017"
    );
    let (spec, cut) = read_original()?;
    let mut hinge = Hinge::load(&spec, Some(&cut.hinge.image))?;
    // The original four/states.jsonl (SHA-256 b9a501233e6a997377f364ca68db48aeeb59561cd70580d7432e9be492e276ff)
    // recorded a separate, uninterrupted monolithic solver in its "direct" column.
    let oracle: [MultiSphereObservation; 4] = serde_json::from_slice(include_bytes!(
        "../../tests/fixtures/pair-warm-v1-direct-1292-1295.json"
    ))
    .unwrap();
    let torque = 0.01;
    assert_eq!(cut.pending, Some(torque));
    for (i, reference) in oracle.into_iter().enumerate() {
        if i > 0 {
            // The saved native graph already holds any accepted input for the first step.
            if i > 1 {
                let joint = hinge.joint;
                hinge.edit(|b| b.add_joint_effort(joint, torque))?;
            }
            hinge.step()?;
        }
        let observed = hinge.sample(&spec)?;
        assert_eq!(reference.hinge.tick, cut.tick + i as u64);
        assert_eq!(observed.links, reference.hinge.links);
        assert_eq!(observed.spheres, reference.spheres);
        assert_eq!(observed.q.to_bits(), reference.hinge.q.to_bits());
        assert_eq!(observed.qdot.to_bits(), reference.hinge.qdot.to_bits());
        assert_eq!(observed.reactions, reference.reactions);
        assert_eq!(observed.sphere_reaction, reference.sphere_reaction);
    }
    Ok(())
}

#[test]
fn original_pair_rejects_a_neighboring_unrecorded_inertia() -> Result<()> {
    let (spec, cut) = read_original()?;
    let mut graph = cut.hinge.image;
    // Identify the first link's serialized principal moments, independently
    // of the graph offsets. Modify just one inverse moment by one ULP.
    let moments: Vec<u8> = [8.333333333333334_f64, 8.33333333333333, 8.33333333333333]
        .into_iter()
        .flat_map(f64::to_le_bytes)
        .collect();
    let locations: Vec<_> = graph
        .windows(moments.len())
        .enumerate()
        .filter_map(|(i, w)| (w == moments).then_some(i))
        .collect();
    assert_eq!(locations.len(), 1);
    let start = locations[0] + 8;
    graph[start..start + 8].copy_from_slice(&8.33333333333333_f64.next_up().to_le_bytes());
    let digest = Sha256::digest(&graph[40..]);
    graph[8..40].copy_from_slice(&digest);
    assert!(Hinge::load(&spec, Some(&graph)).is_err());
    Ok(())
}
