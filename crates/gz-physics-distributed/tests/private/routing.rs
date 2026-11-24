// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_distributed::{coordinator::BodyRoute, physics::Island};

fn cube(id: u64, x: f64, lane: f64, vx: f64) -> Body {
    Body {
        id,
        position: [x, lane, 0.],
        linear: [vx, 0., 0.],
        ..fixture()[0].promoted()
    }
}

#[test]
fn predicted_contacts_are_routed_before_the_first_impact_and_match_native_dynamics() {
    let first = vec![cube(3, -1.5, 0., 1.5), cube(4, 0., 15., 0.3)];
    let second = vec![cube(1, 1.5, 0., -1.5), cube(5, 0., -15., -0.2)];
    let mut all = first.clone();
    all.extend_from_slice(&second);
    let mut native = Island::restore(&Snapshot {
        bodies: all,
        ..Snapshot::default()
    })
    .unwrap();
    let (peers, links) = peers([first, second]);
    let mut c = Coordinator::new(RUN, peers, 256).unwrap();
    let mut contact = false;
    let mut handoffs = 0;
    for _ in 0..140 {
        let routes = c.contact_routes().unwrap();
        if !routes.is_empty() {
            assert_eq!(
                routes,
                [BodyRoute {
                    body: 1,
                    destination: 0
                }]
            );
            let old = c.frame().clone();
            assert!(matches!(c.advance(), Err(Error::Colocation)));
            let seq = links[0].requests.borrow().last().unwrap().sequence + 2;
            links[0].next_fault.set(Some((seq, Fault::LoseReply)));
            assert!(matches!(c.advance_routed(), Err(Error::Transport(_))));
            assert_eq!(c.frame(), &old);
            c.recover(0).unwrap();
            handoffs += 1;
        }
        let frame = c.advance_routed().unwrap();
        native.step().unwrap();
        let oracle = native.snapshot();
        for b in &oracle.bodies {
            assert_eq!(frame.body(b.id).unwrap().1, b);
        }
        let contacts: Vec<_> = frame
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.iter().cloned())
            .collect();
        assert_eq!(contacts, oracle.impulses);
        contact |= !contacts.is_empty();
    }
    assert!(contact);
    assert_eq!(handoffs, 1);
    assert_eq!(c.frame().workers[0].generations[&1], 1);
}

#[test]
fn earliest_complete_component_and_capacity_are_considered_for_all_prospective_contacts() {
    // Two lanes prefer worker 0 purely by stable-ID tie-breaking, but it has
    // one unused lifetime ID slot. The other pair must choose worker 1.
    let mut a = vec![cube(40, -0.36, 0., 1.), cube(41, -0.36, 20., 1.)];
    let b = vec![cube(1, 0.36, 0., -1.), cube(2, 0.36, 20., -1.)];
    a.extend(
        (0..gz_physics_distributed::physics::MAX_PARTITION_BODIES as u64 - 3)
            .map(|i| cube(100 + i, 0., 40. + 10. * i as f64, 0.)),
    );
    let (peers, _) = peers([a, b]);
    let mut c = Coordinator::new(RUN, peers, 128).unwrap();
    assert_eq!(
        c.contact_routes().unwrap(),
        [
            BodyRoute {
                body: 1,
                destination: 0
            },
            BodyRoute {
                body: 41,
                destination: 1
            }
        ]
    );
    c.advance_routed().unwrap();
    assert_eq!(c.frame().body(40).unwrap().0, 0);
    assert_eq!(c.frame().body(41).unwrap().0, 1);
    assert!(c.contact_routes().unwrap().is_empty());
}

#[test]
fn routed_journal_capacity_is_reserved_for_all_handoffs_and_the_step() {
    let bodies = [vec![cube(1, -0.36, 0., 1.)], vec![cube(2, 0.36, 0., -1.)]];
    let (peers, links) = peers(bodies);
    let mut c = Coordinator::new(RUN, peers, 4).unwrap();
    assert_eq!(c.contact_routes().unwrap().len(), 1);
    assert!(matches!(c.advance_routed(), Err(Error::Capacity)));
    assert_eq!(c.frame().tick, 0);
    assert!(links.iter().all(|l| l.requests.borrow().len() == 1));
}

#[test]
fn routing_a_mixed_native_candidate_uses_the_majority_owner() {
    let bodies = [
        vec![cube(9, -0.75, 0., 1.)],
        vec![cube(2, 0., 0., 0.), cube(6, 0.75, 0., -1.)],
    ];
    let (peers, _) = peers(bodies);
    let mut c = Coordinator::new(RUN, peers, 64).unwrap();
    assert_eq!(
        c.contact_routes().unwrap(),
        [BodyRoute {
            body: 9,
            destination: 1
        }]
    );
    c.advance_routed().unwrap();
    assert_eq!(c.frame().workers[1].generations.len(), 3);
}

#[test]
fn impulse_routing_precedes_command_acceptance_and_resumes_after_lost_input_reply() {
    let bodies = [vec![cube(1, -2., 0., 0.)], vec![cube(2, 0., 0., 0.)]];
    let mut native = Island::restore(&Snapshot {
        bodies: bodies.concat(),
        ..Snapshot::default()
    })
    .unwrap();
    let (peers, links) = peers(bodies);
    let mut c = Coordinator::new_recorded(RUN, peers, 64).unwrap();
    let impulse = [150., 0., 0.];
    assert!(matches!(
        c.queue_impulse(1, impulse),
        Err(Error::Colocation)
    ));
    assert!(c.contact_routes().unwrap().is_empty());
    // The new owner receives prepare, activation and finally the input.
    links[1].next_fault.set(Some((4, Fault::LoseReply)));
    assert!(matches!(
        c.queue_impulse_routed(1, impulse),
        Err(Error::Transport(_))
    ));
    assert_eq!(c.frame().body(1).unwrap().0, 1);
    assert_eq!(c.frame().body(1).unwrap().1.linear[0], 0.);
    c.recover(1).unwrap();
    c.queue_impulse_routed(1, impulse).unwrap();
    native.queue_impulse(1, impulse).unwrap();
    for _ in 0..8 {
        let f = c.advance_routed().unwrap();
        native.step().unwrap();
        assert_eq!(f.workers[1].snapshot, native.snapshot());
    }
}

#[test]
fn a_pending_bystander_anchors_a_route_and_a_full_queue_forces_an_alternative_destination() {
    use gz_physics_distributed::physics::MAX_PENDING_IMPULSES;
    let bodies = [
        vec![cube(1, -2., 0., 0.)],
        vec![cube(2, 0., 0., 0.), cube(3, 0., 20., 0.)],
    ];
    let (peers, _) = peers(bodies);
    let mut c = Coordinator::new(RUN, peers, 128).unwrap();
    for _ in 0..MAX_PENDING_IMPULSES {
        c.queue_impulse(3, [0.01, 0., 0.]).unwrap();
    }
    c.queue_impulse_routed(1, [150., 0., 0.]).unwrap();
    assert_eq!(c.frame().body(1).unwrap().0, 0);
    assert_eq!(c.frame().body(2).unwrap().0, 0);
    assert_eq!(c.frame().body(3).unwrap().0, 1);
    assert!(c.advance_routed().unwrap().body(3).unwrap().1.linear[0] > 0.31);
}

#[test]
fn directional_held_force_routes_a_reachable_contact_but_leaves_transverse_workers_independent() {
    use gz_physics_distributed::physics::BodyWrench;
    let a = cube(1, 0., 0., 0.);
    let b = cube(2, 2., 0., 0.);
    let make = || {
        let (p, links) = peers([vec![a.clone()], vec![b.clone()]]);
        (Coordinator::new(RUN, p, 128).unwrap(), links)
    };
    let wrench = |force| BodyWrench {
        id: 1,
        force,
        torque: [0., 0., 0.5],
    };
    for force in [[0., 15_000., 0.], [-22_000., 0., 0.], [0.; 3]] {
        let (mut c, _) = make();
        assert!(c.wrench_routes(&[wrench(force)]).unwrap().is_empty());
        c.advance_routed_with_wrenches(&[wrench(force)]).unwrap();
        assert_ne!(c.frame().body(1).unwrap().0, c.frame().body(2).unwrap().0);
    }
    let (mut c, links) = make();
    let command = wrench([24_000., 0., 0.]);
    let routes = c.wrench_routes(std::slice::from_ref(&command)).unwrap();
    assert_eq!(routes.len(), 1);
    assert!(links.iter().all(|l| l.requests.borrow().len() == 1));
    assert!(
        c.wrench_routes(&[wrench([24_000., 0., 0.]), wrench([-24_000., 0., 0.])])
            .unwrap()
            .is_empty()
    );
    let mut native = Island::restore(&Snapshot {
        bodies: vec![a, b],
        ..Default::default()
    })
    .unwrap();
    let mut hit = false;
    for tick in 0..12 {
        let inputs = if tick == 0 {
            std::slice::from_ref(&command)
        } else {
            &[]
        };
        c.advance_routed_with_wrenches(inputs).unwrap();
        native.step_with_wrenches(inputs).unwrap();
        let oracle = native.snapshot();
        for b in &oracle.bodies {
            assert_eq!(c.frame().body(b.id).unwrap().1, b);
        }
        let impulses = c
            .frame()
            .workers
            .iter()
            .flat_map(|w| w.snapshot.impulses.clone())
            .collect::<Vec<_>>();
        assert_eq!(impulses, oracle.impulses);
        hit |= !impulses.is_empty();
    }
    assert!(hit);
}

#[test]
fn a_new_wrench_refuses_invalid_contributions_without_touching_the_backend() {
    use gz_physics_distributed::physics::{BodyWrench, MAX_PENDING_IMPULSES};
    let (mut c, links) = group(64);
    let bad = |id, force, torque| BodyWrench { id, force, torque };
    let zero = bad(2, [0.; 3], [0.; 3]);
    let cases = vec![
        vec![bad(7, [0.; 3], [0.; 3])],
        vec![bad(1, [1.; 3], [0.; 3])],
        vec![bad(2, [f64::INFINITY; 3], [0.; 3])],
        vec![bad(2, [0.; 3], [f64::NAN; 3])],
        vec![bad(2, [0.; 3], [1e308; 3]); 2],
        vec![zero.clone(); MAX_PENDING_IMPULSES + 1],
    ];
    for inputs in cases {
        assert!(c.wrench_routes(&inputs).is_err());
        assert!(c.advance_routed_with_wrenches(&inputs).is_err());
        assert!(links.iter().all(|l| l.requests.borrow().len() == 1));
    }
    c.advance_routed_with_wrenches(&[zero]).unwrap();
}

#[test]
fn a_fast_arrival_can_route_two_warm_groups_carrying_interleaved_accepted_commands() {
    let a = vec![
        cube(1, 0., 0., 8.),
        cube(2, 0.65, 0., -8.),
        cube(3, 9., 0., 0.15),
    ];
    let b = vec![cube(4, 17.3, 0., 8.), cube(5, 17.95, 0., -8.)];
    let mut native = Island::restore(&Snapshot {
        bodies: [a.clone(), b.clone()].concat(),
        ..Default::default()
    })
    .unwrap();
    let (peers, links) = peers([a, b]);
    let mut c = Coordinator::new(RUN, peers, 64).unwrap();
    for _ in 0..2 {
        c.advance().unwrap();
        native.step().unwrap();
    }
    assert!(c.frame().workers.iter().all(|w| w.snapshot.contacts == 1));
    for id in [4, 2, 5, 1] {
        let j = [0.004, -0., 0.];
        c.queue_impulse(id, j).unwrap();
        native.queue_impulse(id, j).unwrap();
    }
    let j = [-375., 0., 0.];
    c.queue_impulse_routed(3, j).unwrap();
    native.queue_impulse(3, j).unwrap();
    assert_eq!(c.frame().body(1).unwrap().0, c.frame().body(5).unwrap().0);
    assert!(
        links
            .iter()
            .any(|l| l.requests.borrow().iter().any(|r| matches!(
                r.command,
                gz_physics_distributed::protocol::Command::PrepareOrdered { .. }
            )))
    );
    for _ in 0..10 {
        c.advance_routed().unwrap();
        native.step().unwrap();
        for b in &native.snapshot().bodies {
            assert_eq!(c.frame().body(b.id).unwrap().1, b);
        }
    }
}
