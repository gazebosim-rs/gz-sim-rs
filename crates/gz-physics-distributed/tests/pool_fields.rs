// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodySpec, DynamicBackend, RotationSpec};
use gz_physics_distributed::{
    coordinator::Error,
    physics::{Body, BodyWrench, Primitive, SimulationStep, Tier, fixture},
    pool::WorkerPool,
};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::{collections::BTreeMap, path::Path};
fn worker() -> &'static Path {
    Path::new(env!("CARGO_BIN_EXE_physics_worker"))
}
fn body(id: u64, position: [f64; 3]) -> Body {
    Body {
        id,
        tier: Tier::Dynamic,
        mass: 1.,
        size: [0.5; 3],
        primitive: Primitive::Sphere,
        position,
        linear: [0.; 3],
        angular: [0.; 3],
        ..fixture()[1].clone()
    }
}
fn force(id: u64, x: f64) -> BodyWrench {
    BodyWrench {
        id,
        force: [x, 0., 0.],
        torque: [0.; 3],
    }
}

#[test]
fn a_held_force_and_pending_impulse_that_only_jointly_reach_another_worker_are_admitted_together() {
    let a = body(1, [0., 0., 0.]);
    let b = body(2, [0.8, 0., 0.]);
    let mut p = WorkerPool::spawn(
        worker(),
        72213,
        vec![vec![b], vec![], vec![a]],
        SimulationStep::Ms10,
        None,
        2,
    )
    .unwrap();
    let before = p.frame().clone();
    let loads = [force(1, 1500.), force(1, 500.)];
    // Either effect alone is below the 0.3 m clearance. Their conservative
    // joint reach requires a transfer which this deliberately small budget cannot hold.
    assert!(p.validate_field([0.; 3], &loads).is_ok());
    assert!(matches!(
        p.queue_impulse_in_field(1, [20., 0., 0.], [0.; 3], &loads),
        Err(Error::Capacity)
    ));
    assert_eq!(p.frame(), &before);
    p.queue_impulse(1, [20., 0., 0.]).unwrap();
    assert!(p.validate_next([0.; 3]).is_ok());
    assert!(matches!(
        p.validate_field([0.; 3], &loads),
        Err(Error::Capacity)
    ));
    p.advance().unwrap();
    assert_eq!(p.frame().body(1).unwrap().1.linear, [20., 0., 0.]);
}

#[test]
fn force_torque_gravity_and_contacts_match_unpartitioned_native_after_a_routed_lost_barrier() {
    let step = SimulationStep::Ms10;
    let gravity = [0., 0., -2.];
    let a = body(1, [0., 0., 0.]);
    let b = body(2, [0.8, 0., 0.]);
    let c = Body {
        primitive: Primitive::Capsule,
        size: [0.3, 0.3, 1.1],
        ..body(3, [0., 16., 4.])
    };
    let d = Body {
        primitive: Primitive::Box,
        size: [0.3, 0.5, 0.8],
        ..body(4, [0., 32., 4.])
    };
    let mut reference = MultiFidelityBackend::new(gravity).unwrap();
    reference.set_partition_step(step.duration()).unwrap();
    let mut handles = BTreeMap::new();
    for initial in [&a, &b, &c, &d] {
        let shape = initial.shape().unwrap();
        handles.insert(
            initial.id,
            reference
                .spawn_partition_primitive(
                    BodySpec {
                        shape,
                        position: initial.position,
                        mass: Some(initial.mass),
                        velocity: initial.linear,
                    },
                    RotationSpec {
                        orientation: initial.orientation,
                        angular_velocity: initial.angular,
                    },
                    false,
                    0.,
                )
                .unwrap(),
        );
    }
    let mut p = WorkerPool::spawn(
        worker(),
        72214,
        vec![vec![c], vec![b], vec![d], vec![a]],
        step,
        None,
        9,
    )
    .unwrap();
    let mut peak = 0f64;
    let mut exact = 0;
    for tick in 1..=40 {
        let mut held = Vec::new();
        if tick == 1 {
            held.extend([force(1, 1500.), force(1, 500.)]);
        }
        if (3..=19).contains(&tick) {
            held.push(BodyWrench {
                id: 3,
                force: [0.15, 0.42, -0.],
                torque: [1.135, -0.43, 0.28],
            });
            held.push(BodyWrench {
                id: 4,
                force: [-0.36, 0.17, 0.24],
                torque: [0.71, 0.17, -0.34],
            });
            held.push(BodyWrench {
                id: 3,
                force: [0.03, -0.2, 0.],
                torque: [0.15, 0.105, -0.033],
            });
        }
        if tick == 1 {
            p.queue_impulse_in_field(1, [20., 0., 0.], gravity, &held)
                .unwrap();
            reference
                .apply_impulse(handles[&1], [20., 0., 0.], [0.; 3])
                .unwrap();
            let original = p.frame().clone();
            p.drop_next_acknowledgement(2).unwrap();
            assert!(p.advance_in_field(gravity, &held).is_err());
            assert_eq!(p.frame(), &original);
            assert_eq!(p.frame().body(1).unwrap().0, 3);
        }
        if tick == 14 {
            p.compact().unwrap();
            p.recover(0).unwrap();
        }
        for load in &held {
            reference
                .add_world_wrench(handles[&load.id], load.force, load.torque)
                .unwrap();
        }
        reference.step(step.duration()).unwrap();
        let scene = p.advance_in_field(gravity, &held).unwrap();
        assert_eq!(scene.body(1).unwrap().0, scene.body(2).unwrap().0);
        for (&id, &native) in &handles {
            let obs = scene.body(id).unwrap().1;
            let n = reference.state(native).unwrap();
            assert_eq!(
                obs.position.map(f64::to_bits),
                n.position.map(f64::to_bits),
                "origin at {tick}, body {id}"
            );
            assert_eq!(obs.linear.map(f64::to_bits), n.velocity.map(f64::to_bits));
            assert_eq!(
                obs.orientation.map(f64::to_bits),
                n.orientation.map(f64::to_bits)
            );
            assert_eq!(
                obs.angular.map(f64::to_bits),
                n.angular_velocity.map(f64::to_bits)
            );
            exact += 1;
        }
        for pair in scene.workers.iter().flat_map(|w| &w.snapshot.impulses) {
            let native = reference
                .rigid_contact_observations()
                .into_iter()
                .find(|o| {
                    (o.a == handles[&pair.a] && o.b == handles[&pair.b])
                        || (o.b == handles[&pair.a] && o.a == handles[&pair.b])
                })
                .unwrap();
            assert_eq!(
                native.normal_impulse.to_bits(),
                pair.normal_impulse.to_bits()
            );
            peak = peak.max(pair.normal_impulse);
        }
    }
    assert!(peak > 10.);
    assert!(
        p.frame()
            .body(3)
            .unwrap()
            .1
            .angular
            .iter()
            .any(|v| v.abs() > 0.5)
    );
    println!("{exact} bitwise complete body samples, {peak:.10} N s peak normal impulse");
}

#[test]
fn malformed_or_unrealizable_field_leaves_the_accepted_impulse_and_published_states_intact() {
    let a = body(1, [0., 0., 0.]);
    let mut fixed = body(2, [5., 0., 0.]);
    fixed.tier = Tier::Prescribed;
    let mut p = WorkerPool::spawn(
        worker(),
        72215,
        vec![vec![a], vec![fixed], vec![]],
        SimulationStep::Ms1,
        None,
        10,
    )
    .unwrap();
    p.queue_impulse(1, [0.19, 0., -0.]).unwrap();
    let old = p.frame().clone();
    for held in [
        vec![force(44, 1.)],
        vec![force(2, 1.)],
        vec![force(1, f64::NAN)],
        vec![BodyWrench {
            id: 1,
            force: [0.; 3],
            torque: [f64::INFINITY, 0., 0.],
        }],
        vec![force(1, 0.); 33],
    ] {
        assert!(p.advance_in_field([0.; 3], &held).is_err());
        assert!(p.validate_field([0.; 3], &held).is_err());
        assert_eq!(p.frame(), &old);
    }
    p.advance().unwrap();
    assert_eq!(p.frame().body(1).unwrap().1.linear, [0.19, 0., 0.]);
}

#[test]
fn a_rejected_immediate_spatial_impulse_never_writes_the_worker_and_pending_loads_remain_intact() {
    let a = body(1, [0., 0., 0.]);
    let b = body(2, [0.8, 0., 0.]);
    let held = [force(1, 1000.)];
    let mut p = WorkerPool::spawn(
        worker(),
        72420,
        vec![vec![b], vec![], vec![a]],
        SimulationStep::Ms10,
        None,
        2,
    )
    .unwrap();
    p.queue_impulse_in_field(1, [10., 0., 0.], [0.; 3], &held)
        .unwrap();
    let before = p.frame().clone();
    let traffic = p.traffic();
    let old_pid = p.worker_pid(2);
    assert!(matches!(
        p.apply_impulse_in_field(1, [15., 0., 0.], [0.; 3], [0.; 3], &held),
        Err(Error::Capacity)
    ));
    assert_eq!(p.frame(), &before);
    assert_eq!(
        p.traffic()[2].requests - traffic[2].requests,
        1,
        "one read-only native preview"
    );
    assert_eq!(p.worker_pid(2), old_pid);
    p.advance_in_field([0.; 3], &held).unwrap();
    assert!((p.frame().body(1).unwrap().1.linear[0] - 20.).abs() < 1e-13);
}

#[test]
fn lost_spatial_impulse_ack_restores_the_old_twist_and_wakes_only_the_committed_edit() {
    let mut a = body(11, [0., 0., 0.]);
    a.linear = [-0., 0., 0.];
    let mut p = WorkerPool::spawn(
        worker(),
        72421,
        vec![vec![], vec![], vec![a.clone()]],
        SimulationStep::Ms5,
        None,
        8,
    )
    .unwrap();
    // Zero linear impulse is a native no-op, including when angular motion changes.
    p.apply_impulse(a.id, [0., -0., 0.], [0., 0., 0.05])
        .unwrap();
    let prior = p.frame().clone();
    assert_eq!(
        prior.body(a.id).unwrap().1.linear.map(f64::to_bits),
        a.linear.map(f64::to_bits)
    );
    let pid = p.worker_pid(2);
    p.drop_next_acknowledgement(2).unwrap();
    assert!(
        p.apply_impulse(a.id, [0.75, 0., 0.], [0.03, 0.02, 0.])
            .is_err()
    );
    assert_eq!(&prior, p.frame());
    p.compact().unwrap();
    assert_ne!(pid, p.worker_pid(2));
    assert!(p.apply_impulse(a.id, [f64::NAN, 0., 0.], [0.; 3]).is_err());
    assert!(p.apply_impulse(a.id, [0.; 3], [f64::MAX; 3]).is_err());
    assert_eq!(&prior, p.frame());
    p.apply_impulse(a.id, [-0.25, 0., 0.], [0.; 3]).unwrap();
    let final_state = p.frame().body(a.id).unwrap().1;
    assert_eq!(final_state.linear, [-0.25, 0., 0.]);
    assert_eq!(final_state.angular, prior.body(a.id).unwrap().1.angular);
}
