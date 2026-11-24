// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_distributed::{coordinator::Frame, physics::Tier};

fn touching() -> (Coordinator<LocalPeer>, [Rc<Link>; 2]) {
    let mut boxes = fixture();
    boxes[0].tier = Tier::Dynamic;
    boxes[0].position[0] = -0.35;
    boxes[1].position[0] = 0.35;
    let (workers, links) = peers([boxes, vec![]]);
    let mut c = Coordinator::new(RUN, workers, 100).unwrap();
    while c.frame().workers[0].snapshot.contacts == 0 {
        assert!(c.frame().tick < 20);
        c.advance().unwrap();
    }
    (c, links)
}

fn assert_physics(a: &Frame, b: &Frame) {
    assert_eq!(a.tick, b.tick);
    let owner_a = a.body(1).unwrap().0 as usize;
    let owner_b = b.body(1).unwrap().0 as usize;
    assert_eq!(a.workers[owner_a].snapshot, b.workers[owner_b].snapshot);
}

fn inject(link: &Link, ordinal: u64, fault: Fault) {
    link.next_fault.set(Some((
        link.requests.borrow().last().unwrap().sequence + ordinal,
        fault,
    )));
}

#[test]
fn lost_reply_and_replacement_at_every_warm_phase_match_uninterrupted_solver() {
    for (peer, ordinal) in [(0, 1), (1, 1), (0, 2), (1, 2), (0, 3)] {
        let (mut c, links) = touching();
        let (mut control, _) = touching();
        for world in [&mut c, &mut control] {
            world.queue_impulse(1, [1.2, 0.0, 0.0]).unwrap();
        }
        let published = c.frame().clone();
        inject(&links[peer], ordinal, Fault::LoseReply);
        assert!(matches!(c.transfer_island(0), Err(Error::Transport(_))));
        assert_eq!(c.frame(), &published);
        assert!(matches!(c.advance(), Err(Error::Pending)));
        c.recover(peer as u8).unwrap();
        c.transfer_island(0).unwrap();
        assert_eq!(c.frame().body(1).unwrap().0, 1);
        assert_eq!(
            c.frame().workers[1]
                .generations
                .values()
                .copied()
                .collect::<Vec<_>>(),
            [1, 1]
        );
        assert_physics(c.frame(), control.frame());
        for _ in 0..3 {
            assert_physics(c.advance().unwrap(), control.advance().unwrap());
        }
        c.transfer_island(1).unwrap();
        c.recover(0).unwrap();
        assert_eq!(c.frame().workers[0].generations[&1], 2);
        assert_physics(c.advance().unwrap(), control.advance().unwrap());
    }
}

#[test]
fn pending_impulses_follow_rollback_and_are_consumed_once_in_order() {
    let (mut c, links) = touching();
    let (mut control, _) = touching();
    inject(&links[0], 1, Fault::LoseReply);
    assert!(matches!(
        c.queue_impulse(1, [0.9, 0., 0.]),
        Err(Error::Transport(_))
    ));
    assert!(matches!(
        c.queue_impulse(1, [0.9, -0., 0.]),
        Err(Error::Pending)
    ));
    assert!(matches!(c.transfer_island(0), Err(Error::Pending)));
    c.abort_pending().unwrap();
    for world in [&mut c, &mut control] {
        for (id, input) in [(1, [0.7, 0., 0.]), (2, [-0.1, 0., 0.]), (1, [0.7, 0., 0.])] {
            world.queue_impulse(id, input).unwrap();
        }
        assert!(world.transfer(1, false).is_err());
    }
    inject(&links[1], 2, Fault::LoseReply);
    assert!(c.transfer_island(0).is_err());
    c.abort_pending().unwrap();
    assert_physics(c.advance().unwrap(), control.advance().unwrap());
    assert_physics(c.advance().unwrap(), control.advance().unwrap());
}

fn corrupt_warm_export(o: &mut Outcome) {
    let Outcome::WarmExported(h) = o else {
        panic!("expected export")
    };
    h.checkpoint.pending.clear();
}
fn corrupt_digest(o: &mut Outcome) {
    let Outcome::WarmExported(h) = o else {
        panic!("expected export")
    };
    h.receipt.digest[0] ^= 1;
}
fn corrupt_active(o: &mut Outcome) {
    let Outcome::WarmActivated(obs) = o else {
        panic!("expected activation")
    };
    obs.snapshot.impulses.clear();
}
fn corrupt_queue(o: &mut Outcome) {
    let Outcome::State(obs) = o else {
        panic!("expected observation")
    };
    obs.snapshot.tick += 1;
}
fn wrong_phase(o: &mut Outcome) {
    *o = Outcome::State(Observation {
        components: Vec::new(),
        snapshot: Snapshot::default(),
        generations: Default::default(),
    });
}
#[test]
fn changed_input_ledger_digest_phase_or_contact_receipt_never_publishes() {
    for (peer, ordinal, edit) in [
        (0, 1, corrupt_warm_export as fn(&mut Outcome)),
        (0, 1, corrupt_digest),
        (0, 1, wrong_phase),
        (1, 1, wrong_phase),
        (0, 2, wrong_phase),
        (1, 2, wrong_phase),
        (1, 2, corrupt_active),
    ] {
        let (mut c, links) = touching();
        c.queue_impulse(1, [0.2, 0., 0.]).unwrap();
        let before = c.frame().clone();
        inject(&links[peer], ordinal, Fault::Corrupt(edit));
        assert!(matches!(c.transfer_island(0), Err(Error::Invalid(_))));
        assert_eq!(c.frame(), &before);
        c.abort_pending().unwrap();
        c.transfer_island(0).unwrap();
    }
    let (mut c, links) = touching();
    let before = c.frame().clone();
    inject(&links[0], 1, Fault::Corrupt(corrupt_queue));
    assert!(matches!(
        c.queue_impulse(1, [0.2, 0., 0.]),
        Err(Error::Invalid(_))
    ));
    assert_eq!(c.frame(), &before);
    c.abort_pending().unwrap();
    let (mut control, _) = touching();
    assert_physics(c.advance().unwrap(), control.advance().unwrap());
}

#[test]
fn admission_rejects_split_owners_unknown_bodies_and_unrepresentable_commands() {
    let (mut split, _) = group(10);
    assert!(split.queue_impulse(1, [0.; 3]).is_err()); // Prescribed source.
    assert!(split.queue_impulse(2, [0.; 3]).is_ok()); // Dynamic singleton.
    assert!(split.transfer_island(0).is_err());
    let (mut c, _) = touching();
    for worker in [1, 2] {
        assert!(c.transfer_island(worker).is_err());
    }
    assert!(c.queue_impulse(19, [0.; 3]).is_err());
    assert!(c.queue_impulse(1, [f64::NAN, 0., 0.]).is_err());
    c.advance().unwrap();
}
