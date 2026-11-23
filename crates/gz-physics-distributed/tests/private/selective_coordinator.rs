// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_distributed::{coordinator::Frame, physics::MAX_PENDING_IMPULSES};

fn four_boxes() -> [Vec<Body>; 2] {
    let mut pair = fixture();
    pair[0] = pair[0].promoted();
    pair[0].position[0] = -0.35;
    pair[1].position[0] = 0.35;
    let bystander = |id, height, speed| Body {
        id,
        position: [0., 0., height],
        linear: [speed, 0., 0.],
        ..pair[0].clone()
    };
    let dest = bystander(4, 12., -0.45);
    pair.push(bystander(3, -12., 0.25));
    [pair, vec![dest]]
}

fn contact() -> (Coordinator<LocalPeer>, [Rc<Link>; 2]) {
    let (peers, links) = peers(four_boxes());
    let mut world = Coordinator::new(RUN, peers, 128).unwrap();
    while world.frame().workers[0].snapshot.contacts == 0 {
        assert!(world.frame().tick < 20);
        world.advance().unwrap();
    }
    (world, links)
}

fn assert_same_physics(a: &Frame, b: &Frame) {
    assert_eq!(a.tick, b.tick);
    for id in 1..=4 {
        assert_eq!(a.body(id).unwrap().1, b.body(id).unwrap().1);
    }
    let pair = |f: &Frame| {
        f.workers[f.body(1).unwrap().0 as usize]
            .snapshot
            .impulses
            .clone()
    };
    assert_eq!(pair(a), pair(b));
}

fn queued(world: &mut Coordinator<LocalPeer>) {
    for (id, magnitude) in [(3, -0.3), (2, 0.2), (4, 0.95), (1, 0.7), (3, 0.25)] {
        world.queue_impulse(id, [magnitude, 0., 0.]).unwrap();
    }
}

fn interrupt(link: &Link, offset: u64, fault: Fault) {
    let sequence = link.requests.borrow().last().unwrap().sequence + offset;
    link.next_fault.set(Some((sequence, fault)));
}

#[test]
fn either_member_can_retry_after_replacement_and_return_to_the_populated_source() {
    for (worker, offset) in [(0, 1), (1, 1), (0, 2), (1, 2), (0, 3)] {
        let (mut c, links) = contact();
        let (mut reference, _) = contact();
        queued(&mut c);
        queued(&mut reference);
        let before = c.frame().clone();
        interrupt(&links[worker], offset, Fault::LoseReply);
        assert!(matches!(c.transfer_group(2), Err(Error::Transport(_))));
        assert_eq!(c.frame(), &before);
        assert!(matches!(c.advance(), Err(Error::Pending)));
        c.recover(worker as u8).unwrap();
        let arrived = c.transfer_group(1).unwrap();
        assert_eq!(arrived.workers[0].generations, [(3, 0)].into());
        assert_eq!(
            arrived.workers[1].generations,
            [(1, 1), (2, 1), (4, 0)].into()
        );
        assert_same_physics(arrived, &before);
        let returned = c.transfer_group(2).unwrap();
        assert_eq!(
            returned.workers[0].generations,
            [(1, 2), (2, 2), (3, 0)].into()
        );
        assert_eq!(returned.workers[1].generations, [(4, 0)].into());
        c.recover(0).unwrap();
        c.recover(1).unwrap();
        for _ in 0..8 {
            assert_same_physics(c.advance().unwrap(), reference.advance().unwrap());
        }
    }
}

#[test]
fn selective_activation_verifies_both_bystanders_before_publication() {
    fn changed_destination(o: &mut Outcome) {
        let Outcome::WarmActivated(obs) = o else {
            panic!("expected activation")
        };
        obs.generations.insert(4, 1);
    }
    fn changed_source(o: &mut Outcome) {
        state(o).snapshot.bodies[0].angular[2] = -0.;
    }
    for (worker, offset, edit) in [
        (1, 2, changed_destination as fn(&mut Outcome)),
        (0, 3, changed_source),
    ] {
        let (mut c, links) = contact();
        let (mut reference, _) = contact();
        queued(&mut c);
        queued(&mut reference);
        let before = c.frame().clone();
        interrupt(&links[worker], offset, Fault::Corrupt(edit));
        assert!(matches!(c.transfer_group(1), Err(Error::Invalid(_))));
        assert_eq!(c.frame(), &before);
        c.abort_pending().unwrap();
        assert_eq!(c.advance().unwrap(), reference.advance().unwrap());
    }
}

#[test]
fn destination_input_capacity_and_member_eligibility_are_checked_before_export() {
    let (mut c, links) = contact();
    for _ in 0..MAX_PENDING_IMPULSES {
        c.queue_impulse(4, [0.01, 0., 0.]).unwrap();
    }
    c.queue_impulse(1, [0.1, 0., 0.]).unwrap();
    let lengths = || links.each_ref().map(|l| l.requests.borrow().len());
    let before = lengths();
    assert!(matches!(c.transfer_group(1), Err(Error::Capacity)));
    for id in [3, 4, 5] {
        assert!(matches!(c.transfer_group(id), Err(Error::Invalid(_))));
    }
    assert_eq!(before, lengths());
    assert!(c.advance().is_ok());
}

#[test]
fn a_later_bystanders_impulse_cannot_create_an_unresolved_cross_worker_collision() {
    let [mut s, mut d] = four_boxes();
    s[2].position = [0., 0., 12.];
    d[0].position[0] = 2.5;
    let (peers, links) = peers([s, d]);
    let mut c = Coordinator::new(RUN, peers, 32).unwrap();
    assert!(matches!(
        c.queue_impulse(3, [300., 0., 0.]),
        Err(Error::Colocation)
    ));
    assert_eq!(links[0].requests.borrow().len(), 1);
    let before = c.frame().body(3).unwrap().1.linear;
    let next = c.advance().unwrap();
    assert_eq!(next.body(3).unwrap().1.linear, before);
}

#[test]
fn an_inconsistent_native_partition_is_rejected_at_the_barrier() {
    fn empty(o: &mut Outcome) {
        state(o).components[0].ids.clear();
    }
    fn unknown(o: &mut Outcome) {
        state(o).components.last_mut().unwrap().ids.push(9);
    }
    fn reorder(o: &mut Outcome) {
        state(o).components.reverse();
    }
    fn history(o: &mut Outcome) {
        state(o).snapshot.contact_history = true;
    }
    for edit in [empty as fn(&mut Outcome), unknown, reorder, history] {
        let (peers, links) = peers(four_boxes());
        links[0].next_fault.set(Some((1, Fault::Corrupt(edit))));
        assert!(matches!(
            Coordinator::new(RUN, peers, 8),
            Err(Error::Invalid(_))
        ));
    }
}

#[test]
fn capacity_includes_retired_ids_but_a_former_owner_can_receive_the_same_body_again() {
    let [source, _] = four_boxes();
    let dest = (0..gz_physics_distributed::physics::MAX_PARTITION_BODIES as u64)
        .map(|i| Body {
            id: 10 + i,
            position: [0., 0., 20. + 10. * i as f64],
            ..source[2].clone()
        })
        .collect();
    let (peers, links) = peers([source, dest]);
    let mut c = Coordinator::new(RUN, peers, 64).unwrap();
    c.transfer(10, false).unwrap();
    let lengths = || links.each_ref().map(|l| l.requests.borrow().len());
    let before = lengths();
    assert!(matches!(c.transfer(3, false), Err(Error::Capacity)));
    assert_eq!(lengths(), before);
    c.transfer(10, false).unwrap();
    while c.frame().workers[0].snapshot.contacts == 0 {
        assert!(c.frame().tick < 20);
        c.advance().unwrap();
    }
    let before = lengths();
    assert!(matches!(c.transfer_group(2), Err(Error::Capacity)));
    assert_eq!(lengths(), before);
}

fn chain_world(with_bystanders: bool) -> (Coordinator<LocalPeer>, [Rc<Link>; 2]) {
    let mut bodies: Vec<_> = (0..3)
        .map(|i| Body {
            id: [2, 7, 23][i],
            position: [(i as f64 - 1.) * 0.63, 0., 0.],
            linear: [1. - i as f64, 0., 0.],
            ..fixture()[0].promoted()
        })
        .collect();
    let mut dest = Vec::new();
    if with_bystanders {
        for (id, y, owner) in [(4, 15., 0), (11, -15., 1)] {
            let b = Body {
                id,
                position: [0., y, 0.],
                ..bodies[0].clone()
            };
            if owner == 0 {
                bodies.push(b);
            } else {
                dest.push(b);
            }
        }
    }
    let (peers, links) = peers([bodies, dest]);
    let mut world = Coordinator::new(RUN, peers, 128).unwrap();
    while world.frame().workers[0].snapshot.impulses.len() != 2 {
        assert!(world.frame().tick < 20);
        world.advance().unwrap();
    }
    (world, links)
}

#[test]
fn connected_chain_reverse_and_recovery_preserve_all_generations_and_ordered_inputs() {
    let (mut c, links) = chain_world(true);
    let (mut reference, _) = chain_world(true);
    for w in [&mut c, &mut reference] {
        for (id, impulse) in [
            (2, 0.2),
            (4, 0.3),
            (23, -0.4),
            (11, -0.8),
            (7, 0.5),
            (2, 0.1),
        ] {
            w.queue_impulse(id, [impulse, 0., 0.]).unwrap();
        }
    }
    let before = c.frame().clone();
    interrupt(&links[1], 2, Fault::LoseReply);
    assert!(matches!(c.transfer_group(7), Err(Error::Transport(_))));
    assert_eq!(c.frame(), &before);
    c.recover(1).unwrap();
    let moved = c.transfer_group(23).unwrap();
    assert_eq!(moved.workers[0].generations, [(4, 0)].into());
    assert_eq!(
        moved.workers[1].generations,
        [(2, 1), (7, 1), (11, 0), (23, 1)].into()
    );
    assert_eq!(
        moved.workers[1].snapshot.impulses,
        before.workers[0].snapshot.impulses
    );
    let again = c.transfer_group(2).unwrap();
    assert_eq!(
        again.workers[0].generations,
        [(2, 2), (4, 0), (7, 2), (23, 2)].into()
    );
    c.recover(0).unwrap();
    for _ in 0..20 {
        let a = c.advance().unwrap();
        let b = reference.advance().unwrap();
        for (x, y) in a.workers.iter().zip(&b.workers) {
            assert_eq!(x.snapshot, y.snapshot);
            assert_eq!(x.components, y.components);
        }
    }
}

#[test]
fn whole_connected_chain_can_move_to_an_empty_worker_and_back() {
    let (mut c, _) = chain_world(false);
    let before = c.frame().clone();
    let moved = c.transfer_island(0).unwrap();
    assert!(moved.workers[0].snapshot.bodies.is_empty());
    assert_eq!(moved.workers[1].snapshot, before.workers[0].snapshot);
    let back = c.transfer_island(1).unwrap();
    assert_eq!(back.workers[0].snapshot, before.workers[0].snapshot);
}

#[test]
fn a_cold_bystander_can_move_both_directions_while_its_neighbours_retain_contacts_and_inputs() {
    let (mut c, _) = contact();
    let (mut reference, _) = contact();
    for w in [&mut c, &mut reference] {
        w.queue_impulse(1, [0.6, 0., 0.]).unwrap();
        w.queue_impulse(4, [-0.4, 0., 0.]).unwrap();
    }
    let before = c.frame().clone();
    let moved = c.transfer(3, false).unwrap();
    assert_eq!(
        moved.workers[0].snapshot.impulses,
        before.workers[0].snapshot.impulses
    );
    assert_eq!(moved.body(3), Some((1, before.body(3).unwrap().1)));
    assert!(c.transfer(1, false).is_err());
    assert!(c.transfer(4, false).is_err());
    let returned = c.transfer(3, false).unwrap();
    assert_eq!(returned.workers[0].generations[&3], 2);
    c.recover(0).unwrap();
    c.recover(1).unwrap();
    for _ in 0..12 {
        assert_same_physics(c.advance().unwrap(), reference.advance().unwrap());
    }
}
