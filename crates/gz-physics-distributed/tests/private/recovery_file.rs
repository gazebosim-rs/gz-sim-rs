// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_distributed::coordinator::{RecoveryError, RecoveryPlan};
use std::{fs, path::Path};

fn fresh(run: u64, id: u8, bodies: &[Body]) -> io::Result<LocalPeer> {
    let seed = Snapshot {
        bodies: bodies.to_vec(),
        ..Snapshot::default()
    };
    Ok(LocalPeer {
        id,
        worker: Worker::new(run, id, seed.clone()).map_err(io::Error::other)?,
        seed,
        link: Rc::new(Link::default()),
    })
}

fn resume(file: &Path) -> Coordinator<LocalPeer> {
    let plan = RecoveryPlan::open(file).unwrap();
    assert_eq!(plan.run_id(), RUN);
    plan.resume(fresh).unwrap()
}

#[test]
fn a_new_coordinator_rebuilds_contact_history_inputs_generations_and_receipts_from_disk() {
    let (peers, links) = peers(initial());
    let mut c = Coordinator::new_recorded(RUN, peers, 256).unwrap();
    let dir = tempfile::tempdir().unwrap();
    // Promotion at tick zero reaches the boundary ahead of first contact.
    c.transfer(1, true).unwrap();
    while c.frame().workers[1].snapshot.contacts == 0 {
        assert!(c.frame().tick < 140);
        c.advance().unwrap();
    }
    c.queue_impulse(1, [1.2, -0., 0.]).unwrap();
    let before = dir.path().join("before.gzr");
    c.save_recovery(&before).unwrap();
    let sequence = links[0].requests.borrow().last().unwrap().sequence + 2;
    links[0].next_fault.set(Some((sequence, Fault::LoseReply)));
    assert!(matches!(c.transfer_island(1), Err(Error::Transport(_))));
    let interrupted = dir.path().join("interrupted.gzr");
    c.save_recovery(&interrupted).unwrap();
    assert_eq!(fs::read(&before).unwrap(), fs::read(&interrupted).unwrap());
    let mut rebuilt = resume(&interrupted);
    assert_eq!(rebuilt.frame(), c.frame());
    assert_eq!(
        rebuilt.transfer_island(1).unwrap(),
        c.transfer_island(1).unwrap()
    );
    let published = dir.path().join("published.gzr");
    rebuilt.save_recovery(&published).unwrap();
    let mut after = resume(&published);
    for _ in 0..30 {
        assert_eq!(after.advance().unwrap(), c.advance().unwrap());
    }
    for w in [0, 1] {
        after.recover(w).unwrap();
    }
    assert_eq!(after.advance().unwrap(), c.advance().unwrap());
}

#[test]
fn a_saved_prefix_is_immutable_and_divergent_replacements_are_refused() {
    let (peers, _) = peers(initial());
    let mut c = Coordinator::new_recorded(RUN, peers, 20).unwrap();
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("seed.gzr");
    c.save_recovery(&path).unwrap();
    let bytes = fs::read(&path).unwrap();
    c.advance().unwrap();
    assert!(
        matches!(c.save_recovery(&path), Err(RecoveryError::Io(e)) if e.kind() == io::ErrorKind::AlreadyExists)
    );
    assert_eq!(fs::read(&path).unwrap(), bytes);
    assert_eq!(resume(&path).frame().tick, 0);
    let error = RecoveryPlan::open(&path)
        .unwrap()
        .resume(|run, id, bodies| {
            if id == 0 {
                let mut altered = bodies.to_vec();
                altered[0].angular[2] = -0.;
                fresh(run, id, &altered)
            } else {
                fresh(run, id, bodies)
            }
        });
    assert!(matches!(error, Err(RecoveryError::Divergence)));
    assert!(matches!(
        RecoveryPlan::open(&path).unwrap().resume(|run, id, seed| {
            if id == 1 {
                Err(io::Error::other("worker failed to start"))
            } else {
                fresh(run, id, seed)
            }
        }),
        Err(RecoveryError::Io(_))
    ));
    let (regular, _) = group(8);
    assert!(matches!(
        regular.save_recovery(dir.path().join("disabled")),
        Err(RecoveryError::Format(_))
    ));
}

#[test]
fn damaged_files_and_inconsistent_action_counts_fail_before_running_physics() {
    use sha2::{Digest, Sha256};
    let (peers, _) = peers(initial());
    let mut c = Coordinator::new_recorded(RUN, peers, 8).unwrap();
    c.advance().unwrap();
    let dir = tempfile::tempdir().unwrap();
    let original = dir.path().join("good");
    let path = dir.path().join("bad");
    c.save_recovery(&original).unwrap();
    let bytes = fs::read(original).unwrap();
    for cut in [0, 8, 39, 70, bytes.len() - 1] {
        fs::write(&path, &bytes[..cut]).unwrap();
        assert!(RecoveryPlan::open(&path).is_err());
    }
    for index in [0, 8, bytes.len() - 3] {
        let mut bad = bytes.clone();
        bad[index] ^= 1;
        fs::write(&path, bad).unwrap();
        assert!(RecoveryPlan::open(&path).is_err());
    }
    let good = std::str::from_utf8(&bytes[40..]).unwrap();
    let write_payload = |payload: &str| {
        let mut file = bytes[..8].to_vec();
        file.extend_from_slice(&Sha256::digest(payload.as_bytes()));
        file.extend_from_slice(payload.as_bytes());
        fs::write(&path, file).unwrap();
    };
    for bad in [
        good.trim_end().to_owned(),
        good.replace("\"action_count\":1", "\"action_count\":2"),
        good.replace("\"advance\"", "\"bogus\""),
        good.replace("\"limit\":8", "\"limit\":0"),
    ] {
        write_payload(&bad);
        assert!(RecoveryPlan::open(&path).is_err());
    }
}
