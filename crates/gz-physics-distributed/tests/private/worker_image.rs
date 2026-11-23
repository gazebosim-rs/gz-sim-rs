// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::physics::fixture;

fn worker() -> Worker {
    Worker::new(
        43,
        0,
        Snapshot {
            bodies: fixture(),
            ..Default::default()
        },
    )
    .unwrap()
}
fn perform(worker: &mut Worker, sequence: u64, command: Command) -> Result<Reply, ProtocolError> {
    worker.handle(Request {
        run_id: 43,
        worker_id: 0,
        sequence,
        command,
    })
}

#[test]
fn native_images_require_the_committed_tick_sequence_and_no_active_transactions() {
    let mut w = worker();
    perform(&mut w, 1, Command::Observe).unwrap();
    let state = w.recovery_image(1, 0).unwrap();
    assert!(matches!(
        w.recovery_image(0, 0),
        Err(ProtocolError::Sequence { .. })
    ));
    assert!(w.recovery_image(1, 1).is_err());
    for (run, id) in [(44, 0), (43, 1), (43, 2)] {
        assert!(matches!(
            Worker::from_trusted_image(run, id, &state),
            Err(ProtocolError::Address)
        ));
    }
    let mut rebuilt = Worker::from_trusted_image(43, 0, &state).unwrap();
    assert!(matches!(
        perform(&mut rebuilt, 1, Command::Observe),
        Err(ProtocolError::Sequence { .. })
    ));
    for i in 2..=7 {
        let cmd = Command::Step { tick: i - 2 };
        assert_eq!(
            perform(&mut w, i, cmd.clone()),
            perform(&mut rebuilt, i, cmd)
        );
    }
    let t = Ticket {
        run_id: 43,
        serial: 1,
        source: 0,
        destination: 1,
        body_id: 1,
        generation: 1,
        tick: 6,
        promote: false,
    };
    perform(&mut rebuilt, 8, Command::Export { ticket: t }).unwrap();
    assert!(matches!(
        rebuilt.recovery_image(8, 6),
        Err(ProtocolError::Pending)
    ));
    w.terminal = Some("native step failed".into());
    assert!(matches!(
        w.recovery_image(7, 6),
        Err(ProtocolError::Terminal(_))
    ));
}

#[test]
fn native_authority_membership_tombstones_commands_and_digest_are_validated_before_restore() {
    let image = worker().recovery_image(0, 0).unwrap();
    let mut damaged = image.clone();
    damaged.digest[2] ^= 1;
    assert!(matches!(
        Worker::from_trusted_image(43, 0, &damaged),
        Err(ProtocolError::Receipt)
    ));
    let invalid = |mut i: WorkerImage| {
        i.digest = i.checksum(2).unwrap();
        assert!(Worker::from_trusted_image(43, 0, &i).is_err());
    };
    let mut i = image.clone();
    i.generations.remove(&2);
    invalid(i);
    let mut i = image.clone();
    i.retired.insert(1, 1);
    invalid(i);
    let mut i = image.clone();
    i.retired.insert(5, 0);
    invalid(i);
    let mut i = image.clone();
    i.retired
        .extend((4..=MAX_BODY_IDS as u64 + 4).map(|id| (id, 1)));
    invalid(i);
    let mut i = image.clone();
    i.partition.pending.push(PendingImpulse {
        id: 1,
        impulse: [1., 0., 0.],
    });
    invalid(i);
    let mut i = image;
    i.partition.tick += 1;
    invalid(i);
}

#[test]
fn process_work_probes_are_idempotent_and_require_a_complete_unchanged_window() {
    let mut w = worker();
    assert!(w.start_probe(0, 0).is_err());
    assert!(w.start_probe(1, 2).is_err());
    assert!(w.read_probe(0, 2).is_none());
    w.start_probe(0, 2).unwrap();
    w.start_probe(0, 2).unwrap();
    perform(&mut w, 1, Command::Step { tick: 0 }).unwrap();
    assert!(w.read_probe(0, 2).is_none());
    perform(&mut w, 2, Command::Step { tick: 1 }).unwrap();
    let rows = w.read_probe(0, 2).unwrap();
    assert!(rows.iter().all(|r| r.elapsed_ns > 0));
    assert_eq!(
        rows.iter()
            .flat_map(|r| &r.members)
            .copied()
            .collect::<Vec<_>>(),
        [1, 2]
    );
    assert!(w.read_probe(1, 1).is_none());
    w.start_probe(2, 1).unwrap();
    perform(
        &mut w,
        3,
        Command::RetireBody {
            tick: 2,
            id: 1,
            owner: 0,
            generation: 0,
        },
    )
    .unwrap();
    perform(&mut w, 4, Command::Step { tick: 2 }).unwrap();
    assert!(w.read_probe(2, 1).is_none());
}

#[test]
fn pool_image_binds_its_worker_count_even_when_the_endpoint_id_exists_in_both_pools() {
    let state = crate::physics::Snapshot::default();
    let w = Worker::with_width(25, 1, state, 4).unwrap();
    let image = w.recovery_image(0, 0).unwrap();
    assert!(Worker::from_trusted_image(25, 1, &image).is_err());
    assert!(Worker::from_scoped_image(25, 1, &image, 3).is_err());
    assert!(Worker::from_scoped_image(25, 1, &image, 4).is_ok());
    for (id, width) in [(2, 2), (3, 3), (0, 9), (0, 1)] {
        assert!(Worker::with_width(25, id, crate::physics::Snapshot::default(), width).is_err());
    }
}
