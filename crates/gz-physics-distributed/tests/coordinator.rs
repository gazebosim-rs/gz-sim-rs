// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_distributed::{
    coordinator::{Coordinator, Error, Peer},
    physics::{Body, Snapshot, fixture},
    protocol::{Observation, Outcome, Reply, Request, Worker},
    transport::TransportError,
};
use std::{
    cell::{Cell, RefCell},
    io,
    rc::Rc,
};

const RUN: u64 = 16;

#[derive(Clone, Copy)]
enum Fault {
    LoseReply,
    WrongRun,
    WrongWorker,
    OldSequence,
    Corrupt(fn(&mut Outcome)),
}

#[derive(Default)]
struct Link {
    next_fault: Cell<Option<(u64, Fault)>>,
    fail_restart: Cell<bool>,
    restarts: Cell<usize>,
    requests: RefCell<Vec<Request>>,
}

// Native Bevy/Rapier Worker with a controllable delivery boundary. A dropped
// reply happens after native execution, exactly like loss of a transport ACK.
struct LocalPeer {
    id: u8,
    seed: Snapshot,
    worker: Worker,
    link: Rc<Link>,
}
impl Peer for LocalPeer {
    fn exchange(&mut self, request: &Request) -> Result<Reply, TransportError> {
        self.link.requests.borrow_mut().push(request.clone());
        let mut reply = self.worker.handle(request.clone())?;
        if let Some((sequence, fault)) = self.link.next_fault.get()
            && sequence == request.sequence
        {
            self.link.next_fault.set(None);
            match fault {
                Fault::LoseReply => {
                    return Err(TransportError::Unavailable(
                        "lost committed reply".to_owned(),
                    ));
                }
                Fault::WrongRun => reply.run_id += 1,
                Fault::WrongWorker => reply.worker_id = 1 - reply.worker_id,
                Fault::OldSequence => reply.sequence -= 1,
                Fault::Corrupt(edit) => edit(&mut reply.outcome),
            }
        }
        Ok(reply)
    }
    fn restart(&mut self) -> io::Result<()> {
        if self.link.fail_restart.replace(false) {
            return Err(io::Error::other("injected restart failure"));
        }
        self.worker = Worker::new(RUN, self.id, self.seed.clone()).map_err(io::Error::other)?;
        self.link.restarts.set(self.link.restarts.get() + 1);
        Ok(())
    }
}

fn peers(seeds: [Vec<Body>; 2]) -> ([LocalPeer; 2], [Rc<Link>; 2]) {
    let links = [Rc::new(Link::default()), Rc::new(Link::default())];
    let mut seeds = seeds.into_iter();
    let peers = std::array::from_fn(|id| {
        let seed = Snapshot {
            bodies: seeds.next().unwrap(),
            ..Snapshot::default()
        };
        LocalPeer {
            id: id as u8,
            worker: Worker::new(RUN, id as u8, seed.clone()).unwrap(),
            seed,
            link: links[id].clone(),
        }
    });
    (peers, links)
}
fn initial() -> [Vec<Body>; 2] {
    let [a, b]: [Body; 2] = fixture().try_into().unwrap();
    [vec![a], vec![b]]
}
fn group(limit: usize) -> (Coordinator<LocalPeer>, [Rc<Link>; 2]) {
    let (peers, links) = peers(initial());
    (Coordinator::new(RUN, peers, limit).unwrap(), links)
}

#[test]
fn loss_after_step_and_after_release_never_publishes_a_partial_barrier() {
    let (mut group, links) = group(64);
    let initial = group.frame().clone();
    links[1].next_fault.set(Some((2, Fault::LoseReply)));
    assert!(matches!(group.advance(), Err(Error::Transport(_))));
    assert_eq!(group.frame(), &initial);
    assert!(matches!(group.transfer(1, true), Err(Error::Pending)));
    let advanced = group.advance().unwrap().clone();
    assert_eq!(advanced.tick, 1);
    assert_eq!(links[0].requests.borrow().len(), 2);
    assert_eq!(links[1].requests.borrow()[1], links[1].requests.borrow()[2]);
    links[0].next_fault.set(Some((4, Fault::LoseReply)));
    assert!(matches!(group.transfer(1, true), Err(Error::Transport(_))));
    assert_eq!(group.frame(), &advanced);
    assert!(matches!(group.advance(), Err(Error::Pending)));
    assert!(matches!(group.transfer(1, false), Err(Error::Pending)));
    let transferred = group.transfer(1, true).unwrap().clone();
    assert_eq!(transferred.tick, advanced.tick);
    assert_eq!(transferred.body(1).unwrap().0, 1);
    assert_eq!(
        transferred.body(1).unwrap().1,
        &advanced.body(1).unwrap().1.promoted()
    );
    assert_eq!(transferred.workers[1].generations[&1], 1);
    let (mut control, _) = self::group(64);
    control.advance().unwrap();
    assert_eq!(control.transfer(1, true).unwrap(), &transferred);
    assert_eq!(group.advance().unwrap(), control.advance().unwrap());
}

#[test]
fn stale_and_misaddressed_replies_never_enter_the_acknowledged_journal() {
    for fault in [Fault::WrongRun, Fault::WrongWorker, Fault::OldSequence] {
        let (mut coordinator, links) = group(6);
        let previous = coordinator.frame().clone();
        links[1].next_fault.set(Some((2, fault)));
        assert!(matches!(coordinator.advance(), Err(Error::Invalid(_))));
        assert_eq!(coordinator.frame(), &previous);
        assert_eq!(coordinator.advance().unwrap().tick, 1);
        let log = links[1].requests.borrow();
        assert_eq!(log[1], log[2]);
    }
}

#[test]
fn capacity_is_reserved_for_the_entire_transfer_before_any_packet_is_sent() {
    let (mut c, links) = group(3);
    assert!(matches!(c.transfer(1, true), Err(Error::Capacity)));
    for link in &links {
        assert_eq!(link.requests.borrow().len(), 1);
    }
    c.advance().unwrap();
    c.advance().unwrap();
    let full = c.frame().clone();
    assert!(matches!(c.advance(), Err(Error::Capacity)));
    assert_eq!(c.frame(), &full);
    for link in links {
        assert_eq!(link.requests.borrow().len(), 3);
    }
    let (peers, links) = peers(initial());
    assert!(matches!(
        Coordinator::new(RUN, peers, 0),
        Err(Error::Capacity)
    ));
    for link in links {
        assert!(link.requests.borrow().is_empty());
    }
}

#[test]
fn reject_duplicate_ids_and_protect_fast_cross_partition_trajectories() {
    let [a, mut b] = initial();
    b[0].id = a[0].id;
    let (same, _) = peers([a, b]);
    assert!(matches!(
        Coordinator::new(RUN, same, 8),
        Err(Error::Invalid(_))
    ));
    let [mut a, mut b] = initial();
    a[0].position = [-2., 0., 0.];
    a[0].linear = [300., 0., 0.];
    b[0].position = [2., 0., 0.];
    b[0].linear = [-300., 0., 0.];
    let (crossing, links) = peers([a.clone(), b.clone()]);
    let mut c = Coordinator::new(RUN, crossing, 16).unwrap();
    // Both endpoint center distances exceed the sum of the enclosing radii.
    // The swept segments intersect in between; no local solver should advance.
    assert!(matches!(c.advance(), Err(Error::Colocation)));
    assert_eq!(c.frame().tick, 0);
    for link in links {
        assert_eq!(link.requests.borrow().len(), 1);
    }
    b[0].position[1] = 3.;
    let (parallel, _) = peers([a, b]);
    assert_eq!(
        Coordinator::new(RUN, parallel, 16)
            .unwrap()
            .advance()
            .unwrap()
            .tick,
        1
    );
}

#[test]
fn divergent_recovery_leaves_worker_offline_until_a_verified_replay_succeeds() {
    let (mut c, links) = group(64);
    c.advance().unwrap();
    let checkpoint = c.frame().clone();
    assert!(matches!(c.recover(2), Err(Error::Invalid(_))));
    links[0].fail_restart.set(true);
    assert!(matches!(c.recover(0), Err(Error::Restart(_))));
    assert!(matches!(c.advance(), Err(Error::Offline(0))));
    links[0].next_fault.set(Some((
        2,
        Fault::Corrupt(|reply| {
            state(reply).snapshot.bodies[0].position[1] += 0.01;
        }),
    )));
    assert!(matches!(c.recover(0), Err(Error::Diverged(0, 2))));
    assert_eq!(c.frame(), &checkpoint);
    assert!(matches!(c.advance(), Err(Error::Offline(0))));
    c.recover(0).unwrap();
    assert_eq!(c.frame(), &checkpoint);
    let (mut expected, _) = group(64);
    expected.advance().unwrap();
    assert_eq!(c.advance().unwrap(), expected.advance().unwrap());
}

#[test]
fn abort_after_source_retirement_fences_and_replays_both_published_owners() {
    let (mut c, links) = group(64);
    c.advance().unwrap();
    let before = c.frame().clone();
    // Export and prepare have been acknowledged. The source physically removes
    // its body, then its release receipt is lost. Neither worker can be trusted
    // as the published authority until fenced replacements replay the checkpoint.
    links[0].next_fault.set(Some((4, Fault::LoseReply)));
    assert!(matches!(c.transfer(1, true), Err(Error::Transport(_))));
    assert_eq!(c.frame(), &before);
    c.abort_pending().unwrap();
    assert_eq!(c.frame(), &before);
    for link in links {
        assert_eq!(link.restarts.get(), 1);
    }
    let (mut control, _) = group(64);
    control.advance().unwrap();
    assert_eq!(c.advance().unwrap(), control.advance().unwrap());
    assert_eq!(
        c.transfer(1, true).unwrap(),
        control.transfer(1, true).unwrap()
    );
}

#[test]
fn interruption_during_fenced_rollback_requires_another_abort_before_use() {
    let (mut c, links) = group(32);
    c.abort_pending().unwrap();
    assert_eq!(links[0].restarts.get(), 0);
    let initial = c.frame().clone();
    links[1].next_fault.set(Some((2, Fault::LoseReply)));
    assert!(matches!(c.advance(), Err(Error::Transport(_))));
    links[1].fail_restart.set(true);
    assert!(matches!(c.abort_pending(), Err(Error::Restart(_))));
    assert_eq!(links[0].restarts.get(), 1);
    assert_eq!(links[1].restarts.get(), 0);
    assert_eq!(c.frame(), &initial);
    assert!(matches!(c.advance(), Err(Error::Pending)));
    assert!(matches!(c.recover(1), Err(Error::Pending)));
    c.abort_pending().unwrap();
    assert_eq!(links[0].restarts.get(), 2);
    assert_eq!(links[1].restarts.get(), 1);
    let (mut control, _) = group(32);
    assert_eq!(c.advance().unwrap(), control.advance().unwrap());
}

fn state(reply: &mut Outcome) -> &mut Observation {
    let Outcome::State(state) = reply else {
        panic!("expected state reply")
    };
    state
}

fn activated(reply: &mut Outcome) -> &mut Observation {
    let Outcome::Activated(state) = reply else {
        panic!("expected activation reply")
    };
    state
}

fn substitute_observation(reply: &mut Outcome) {
    *reply = Outcome::State(Observation {
        components: Vec::new(),
        snapshot: Snapshot::default(),
        generations: Default::default(),
    });
}

fn assert_transfer_refusal(worker: usize, sequence: u64, edit: fn(&mut Outcome), message: &str) {
    let (mut c, links) = group(32);
    let checkpoint = c.frame().clone();
    links[worker]
        .next_fault
        .set(Some((sequence, Fault::Corrupt(edit))));
    assert!(matches!(c.transfer(1, true), Err(Error::Invalid(reason)) if reason == message));
    assert_eq!(c.frame(), &checkpoint);
    c.abort_pending().unwrap();
    let frame = c.transfer(1, true).unwrap();
    assert_eq!(
        frame.body(1),
        Some((1, &checkpoint.body(1).unwrap().1.promoted()))
    );
    assert_eq!(frame.body(2), checkpoint.body(2));
}

#[test]
fn transfer_acknowledgements_require_the_correct_phase_and_the_original_handoff() {
    for (worker, sequence, message) in [
        (0, 2, "export acknowledgement"),
        (1, 2, "prepare acknowledgement"),
        (0, 3, "release acknowledgement"),
        (1, 3, "activation acknowledgement"),
    ] {
        assert_transfer_refusal(worker, sequence, substitute_observation, message);
    }
    assert_transfer_refusal(
        0,
        2,
        |reply| {
            let Outcome::Exported(h) = reply else {
                panic!("expected export")
            };
            h.ticket.destination = h.ticket.source;
        },
        "export changed identity or state",
    );
    assert_transfer_refusal(
        0,
        2,
        |reply| {
            let Outcome::Exported(h) = reply else {
                panic!("expected export")
            };
            h.body.linear[0] += 0.01;
        },
        "export changed identity or state",
    );
    assert_transfer_refusal(
        1,
        2,
        |reply| {
            let Outcome::Prepared(p) = reply else {
                panic!("expected prepare")
            };
            p.handoff.ticket.promote = false;
        },
        "prepare changed transfer",
    );
    assert_transfer_refusal(
        0,
        3,
        |reply| {
            let Outcome::Released(r) = reply else {
                panic!("expected release")
            };
            r.handoff.body.mass += 0.5;
        },
        "release changed transfer",
    );
}

#[test]
fn activation_must_preserve_both_bodies_and_only_advance_the_migrating_generation() {
    assert_transfer_refusal(
        1,
        3,
        |reply| {
            activated(reply)
                .snapshot
                .bodies
                .iter_mut()
                .find(|b| b.id == 1)
                .unwrap()
                .position[0] += 0.5;
        },
        "transfer discontinuity",
    );
    assert_transfer_refusal(
        1,
        3,
        |reply| {
            activated(reply).generations.insert(1, 0);
        },
        "transfer discontinuity",
    );
    assert_transfer_refusal(
        1,
        3,
        |reply| {
            activated(reply)
                .snapshot
                .bodies
                .iter_mut()
                .find(|b| b.id == 2)
                .unwrap()
                .linear[0] += 0.01;
        },
        "unrelated body changed during transfer",
    );
    assert_transfer_refusal(
        1,
        3,
        |reply| {
            activated(reply).generations.insert(2, 1);
        },
        "unrelated body changed during transfer",
    );
}

fn flip_zero(scalar: &mut f64) {
    assert_eq!(*scalar, 0.);
    *scalar = -*scalar;
}

#[test]
fn migration_requires_bitwise_equality_for_exported_activated_and_unrelated_bodies() {
    assert_transfer_refusal(
        0,
        2,
        |reply| {
            let Outcome::Exported(h) = reply else {
                panic!("expected export")
            };
            flip_zero(&mut h.body.angular[2]);
        },
        "export changed identity or state",
    );
    fn flip_angular_z(reply: &mut Outcome, id: u64) {
        let body = activated(reply)
            .snapshot
            .bodies
            .iter_mut()
            .find(|b| b.id == id)
            .unwrap();
        flip_zero(&mut body.angular[2]);
    }
    assert_transfer_refusal(1, 3, |r| flip_angular_z(r, 1), "transfer discontinuity");
    assert_transfer_refusal(
        1,
        3,
        |r| flip_angular_z(r, 2),
        "unrelated body changed during transfer",
    );
}

#[test]
fn step_requires_state_at_target_tick_with_matching_owned_ids_and_generations() {
    fn refuse(edit: fn(&mut Outcome), message: &str) {
        let (mut c, links) = group(8);
        let checkpoint = c.frame().clone();
        links[1].next_fault.set(Some((2, Fault::Corrupt(edit))));
        assert!(matches!(c.advance(), Err(Error::Invalid(reason)) if reason == message));
        assert_eq!(c.frame(), &checkpoint);
        c.abort_pending().unwrap();
        assert_eq!(c.advance().unwrap().tick, 1);
    }
    refuse(
        |reply| *reply = Outcome::Activated(state(reply).clone()),
        "expected worker observation",
    );
    refuse(
        |reply| state(reply).snapshot.tick = 0,
        "observation tick or generation count",
    );
    refuse(
        |reply| state(reply).generations.clear(),
        "observation tick or generation count",
    );
    refuse(
        |reply| {
            state(reply).generations.remove(&2);
            state(reply).generations.insert(3, 0);
        },
        "duplicate or unowned global ID",
    );
    refuse(
        |reply| {
            state(reply).generations.insert(2, 1);
        },
        "step changed ownership",
    );
    let (one, _) = peers([initial()[0].clone(), vec![]]);
    assert!(matches!(
        Coordinator::new(RUN, one, 8),
        Err(Error::Invalid(
            "the distributed world requires at least two boxes"
        ))
    ));
}

#[test]
fn signed_zero_replay_divergence_keeps_the_worker_offline() {
    let (mut c, links) = group(12);
    let checkpoint = c.frame().clone();
    links[0].next_fault.set(Some((
        1,
        Fault::Corrupt(|reply| {
            flip_zero(&mut state(reply).snapshot.bodies[0].angular[2]);
        }),
    )));
    assert!(matches!(c.recover(0), Err(Error::Diverged(0, 1))));
    assert!(matches!(c.advance(), Err(Error::Offline(0))));
    assert_eq!(c.frame(), &checkpoint);
    c.recover(0).unwrap();
    assert_eq!(c.advance().unwrap().tick, 1);
}

#[test]
fn bidirectional_transfers_swap_observations_and_keep_generation_monotonic() {
    let (mut c, _) = group(32);
    let original = c.frame().clone();
    assert!(matches!(
        c.transfer(3, false),
        Err(Error::Invalid("unknown transfer body"))
    ));
    let on_first = c.transfer(2, false).unwrap();
    assert_eq!(on_first.body(2), Some((0, original.body(2).unwrap().1)));
    assert_eq!(on_first.workers[0].generations[&2], 1);
    let returned = c.transfer(2, false).unwrap();
    assert_eq!(returned.body(2), original.body(2));
    assert_eq!(returned.workers[1].generations[&2], 2);
}

#[test]
fn replacement_after_loss_of_each_handoff_reply_recovers_the_same_world() {
    let (mut baseline, _) = group(16);
    let expected = baseline.transfer(1, true).unwrap().clone();
    let after_step = baseline.advance().unwrap().clone();
    // Exercise loss after each native mutation, including the final observation
    // when the destination already owns the body but no new frame was published.
    for (peer, sequence) in [(0, 2), (1, 2), (0, 3), (1, 3), (0, 4)] {
        let (mut world, links) = group(16);
        let before = world.frame().clone();
        links[peer]
            .next_fault
            .set(Some((sequence, Fault::LoseReply)));
        assert!(matches!(world.transfer(1, true), Err(Error::Transport(_))));
        assert_eq!(world.frame(), &before);
        world.recover(peer as u8).unwrap();
        assert_eq!(world.transfer(1, true).unwrap(), &expected);
        assert_eq!(world.advance().unwrap(), &after_step);
    }
}

#[test]
fn observe_and_replay_require_a_response_from_the_requested_worker() {
    fn wrong_kind(reply: &mut Outcome) {
        let Outcome::State(obs) = reply else {
            panic!("expected observation")
        };
        *reply = Outcome::Activated(obs.clone());
    }
    for id in [0, 1] {
        for fault in [Fault::LoseReply, Fault::Corrupt(wrong_kind)] {
            let (peers, links) = peers(initial());
            links[id].next_fault.set(Some((1, fault)));
            assert!(Coordinator::new(RUN, peers, 16).is_err());
        }
    }
    let (mut world, links) = group(16);
    let before = world.frame().clone();
    links[0].next_fault.set(Some((2, Fault::LoseReply)));
    assert!(matches!(world.advance(), Err(Error::Transport(_))));
    links[0].next_fault.set(Some((1, Fault::LoseReply)));
    assert!(matches!(world.recover(0), Err(Error::Transport(_))));
    assert_eq!(world.frame(), &before);
    assert!(matches!(world.advance(), Err(Error::Offline(0))));
    world.recover(0).unwrap();
    assert_eq!(world.advance().unwrap().tick, 1);
}

#[test]
fn coordinator_refuses_to_move_native_contact_history_before_any_transfer_request() {
    let (mut world, links) = group(180);
    world.transfer(1, true).unwrap();
    for _ in 0..120 {
        world.advance().unwrap();
    }
    assert!(world.frame().workers[1].snapshot.contact_history);
    let previous = world.frame().clone();
    let sent = links.each_ref().map(|l| l.requests.borrow().len());
    assert!(matches!(world.transfer(1, false), Err(Error::Invalid(_))));
    assert_eq!(world.frame(), &previous);
    assert_eq!(sent, links.each_ref().map(|l| l.requests.borrow().len()));
    assert_eq!(world.advance().unwrap().tick, 121);
}

#[path = "private/warm_coordinator.rs"]
mod warm;

#[path = "private/selective_coordinator.rs"]
mod selective;

#[path = "private/recovery_file.rs"]
mod recovery_file;

#[path = "private/routing.rs"]
mod routing;

#[path = "private/overlap.rs"]
mod overlap;

#[path = "private/compact.rs"]
mod compact;

#[test]
fn interrupted_spatial_edits_forced_step_and_retirement_preserve_the_last_frame() {
    use gz_physics_distributed::physics::BodyWrench;
    let [mut a, mut b]: [Body; 2] = fixture().try_into().unwrap();
    a = a.promoted();
    a.position[1] = -20.;
    b.position[1] = 20.;
    let seeds = [vec![a], vec![b]];
    let make = || {
        let (ps, links) = peers(seeds.clone());
        (Coordinator::new(RUN, ps, 128).unwrap(), links)
    };
    let (mut live, links) = make();
    let (mut control, _) = make();
    let lose_next = |w: usize| {
        let seq = links[w].requests.borrow().last().unwrap().sequence + 1;
        links[w].next_fault.set(Some((seq, Fault::LoseReply)));
    };
    let at = live.frame().clone();
    lose_next(0);
    assert!(matches!(
        live.impulse_now(1, [0.15, 0., -0.], [0., 0.02, 0.]),
        Err(Error::Transport(_))
    ));
    assert_eq!(live.frame(), &at);
    assert!(matches!(
        live.impulse_now(1, [0.15, 0., 0.], [0., 0.02, 0.]),
        Err(Error::Pending)
    ));
    live.recover(0).unwrap();
    assert_eq!(
        live.impulse_now(1, [0.15, 0., -0.], [0., 0.02, 0.])
            .unwrap(),
        control
            .impulse_now(1, [0.15, 0., -0.], [0., 0.02, 0.])
            .unwrap()
    );
    lose_next(1);
    assert!(matches!(
        live.set_velocity(2, [0.21, 0., 0.], [0., 0., -0.3]),
        Err(Error::Transport(_))
    ));
    live.abort_pending().unwrap();
    assert_eq!(live.frame(), control.frame());
    let w = BodyWrench {
        id: 2,
        force: [0.01, 0., -0.0],
        torque: [0., 0., 0.02],
    };
    lose_next(1);
    assert!(matches!(
        live.advance_routed_with_wrenches(std::slice::from_ref(&w)),
        Err(Error::Transport(_))
    ));
    let mut changed = w.clone();
    changed.force[2] = 0.;
    assert!(matches!(
        live.advance_routed_with_wrenches(&[changed]),
        Err(Error::Pending)
    ));
    assert_eq!(live.frame(), control.frame());
    assert_eq!(
        live.advance_routed_with_wrenches(std::slice::from_ref(&w))
            .unwrap(),
        control.advance_routed_with_wrenches(&[w]).unwrap()
    );
    live.queue_impulse_routed(2, [0.1, 0., 0.]).unwrap();
    control.queue_impulse_routed(2, [0.1, 0., 0.]).unwrap();
    lose_next(1);
    assert!(matches!(live.retire(2), Err(Error::Transport(_))));
    assert_eq!(live.frame(), control.frame());
    live.recover(1).unwrap();
    assert_eq!(live.retire(2).unwrap(), control.retire(2).unwrap());
    assert_eq!(live.advance().unwrap(), control.advance().unwrap());
}

#[test]
fn insert_and_pose_edit_reuse_the_exact_delivered_request_after_ack_loss() {
    let (mut c, links) = group(42);
    let fresh = Body {
        id: 3,
        position: [0., 7., 0.],
        ..fixture()[1].clone()
    };
    links[0].next_fault.set(Some((2, Fault::LoseReply)));
    assert!(matches!(
        c.insert_body(fresh.clone()),
        Err(Error::Transport(_))
    ));
    assert!(c.frame().body(3).is_none());
    let changed = Body {
        angular: [-0., 0., 0.],
        ..fresh.clone()
    };
    assert!(matches!(c.insert_body(changed), Err(Error::Pending)));
    c.insert_body(fresh).unwrap();
    let original = c.frame().clone();
    let mut state = gz_physics::BodyState {
        position: [1., 8., 0.],
        velocity: [0., 0., 0.],
        orientation: [0., 0., 0., 1.],
        angular_velocity: [0.1, 0.2, 0.],
    };
    links[0].next_fault.set(Some((3, Fault::LoseReply)));
    assert!(matches!(
        c.replace_state(3, state),
        Err(Error::Transport(_))
    ));
    assert_eq!(c.frame(), &original);
    state.velocity[2] = -0.;
    assert!(matches!(c.replace_state(3, state), Err(Error::Pending)));
    state.velocity[2] = 0.;
    c.recover(0).unwrap();
    c.replace_state(3, state).unwrap();
    assert_eq!(c.frame().body(3).unwrap().1.position, state.position);
    c.advance_routed().unwrap();
}
