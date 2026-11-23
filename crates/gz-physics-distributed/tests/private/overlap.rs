// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_distributed::coordinator::OverlappingPeers;
impl OverlappingPeers for LocalPeer {
    fn exchange_steps(
        peers: &mut [Self; 2],
        requests: &[Request; 2],
    ) -> [Result<Reply, TransportError>; 2] {
        [
            peers[0].exchange(&requests[0]),
            peers[1].exchange(&requests[1]),
        ]
    }
}

#[test]
fn success_on_either_peer_is_retained_even_if_its_counterpart_loses_a_reply() {
    for failed in 0..2 {
        for finish_overlapped in [false, true] {
            let (mut c, l) = group(8);
            let (mut reference, _) = group(8);
            let before = c.frame().clone();
            l[failed].next_fault.set(Some((2, Fault::LoseReply)));
            assert!(matches!(c.advance_overlapped(), Err(Error::Transport(_))));
            assert_eq!(c.frame(), &before);
            assert!(l.iter().all(|l| l.requests.borrow().len() == 2));
            c.recover(failed as u8).unwrap();
            let result = if finish_overlapped {
                c.advance_overlapped()
            } else {
                c.advance()
            };
            assert_eq!(result.unwrap(), reference.advance().unwrap());
            assert_eq!(l[1 - failed].requests.borrow().len(), 2);
            assert_eq!(
                c.advance_overlapped().unwrap(),
                reference.advance().unwrap()
            );
        }
    }
}

#[test]
fn independent_failures_roll_back_to_the_previous_verified_frame() {
    let (mut c, l) = group(8);
    let before = c.frame().clone();
    l[0].next_fault.set(Some((2, Fault::WrongRun)));
    l[1].next_fault.set(Some((2, Fault::LoseReply)));
    assert!(matches!(
        c.advance_overlapped(),
        Err(Error::Invalid("reply envelope"))
    ));
    assert_eq!(c.frame(), &before);
    c.abort_pending().unwrap();
    assert_eq!(c.advance_overlapped().unwrap().tick, 1);
    for link in l {
        assert_eq!(link.restarts.get(), 1);
    }
}

#[test]
fn field_ack_loss_on_either_worker_keeps_the_other_native_step_and_exact_input() {
    use gz_physics_distributed::physics::BodyWrench;
    for failed in 0..2 {
        for overlap_retry in [false, true] {
            let (mut c, links) = group(16);
            let (mut reference, _) = group(16);
            let id = c
                .frame()
                .workers
                .iter()
                .flat_map(|w| &w.snapshot.bodies)
                .find(|b| b.tier == gz_physics_distributed::physics::Tier::Dynamic)
                .unwrap()
                .id;
            let fields = [BodyWrench {
                id,
                force: [2.5, -0., -0.4],
                torque: [0.1, 0., 0.4],
            }];
            let gravity = [0.3, 0., -9.81];
            let old = c.frame().clone();
            links[failed].next_fault.set(Some((2, Fault::LoseReply)));
            assert!(matches!(
                c.advance_in_field_overlapped(gravity, &fields),
                Err(Error::Transport(_))
            ));
            assert_eq!(c.frame(), &old);
            c.recover(failed as u8).unwrap();
            let mut changed = fields.clone();
            changed[0].force[1] = 0.;
            assert!(matches!(
                c.advance_in_field_overlapped(gravity, &changed),
                Err(Error::Pending)
            ));
            let recovered = if overlap_retry {
                c.advance_in_field_overlapped(gravity, &fields)
            } else {
                c.advance_in_field(gravity, &fields)
            }
            .unwrap();
            assert_eq!(
                recovered,
                reference.advance_in_field(gravity, &fields).unwrap()
            );
            assert_eq!(links[1 - failed].requests.borrow().len(), 2);
        }
    }
}
