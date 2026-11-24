// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn seed(step: SimulationStep) -> Snapshot {
    let mut left = fixture()[0].promoted();
    left.position = [-0.34, 0., 0.34];
    left.linear = [11., 0., -1.];
    let mut right = left.clone();
    right.id = 2;
    right.position[0] = 0.34;
    right.linear[0] = -11.;
    Snapshot {
        bodies: vec![left, right],
        ground: Some(GroundPlane::new(0.).unwrap()),
        step,
        ..Default::default()
    }
}

#[test]
fn one_public_receipt_matches_ten_forced_native_solves_including_support_load_and_restart() {
    let mut actual = Partition::restore(&seed(SimulationStep::Ms10)).unwrap();
    let mut oracle = Island::restore(&seed(SimulationStep::Ms1)).unwrap();
    let loads = [
        BodyWrench {
            id: 1,
            force: [1.5, 0., 0.],
            torque: [0., 0.2, 0.],
        },
        BodyWrench {
            id: 1,
            force: [0.5, 0., 0.],
            torque: [0., -0.1, 0.],
        },
        BodyWrench {
            id: 2,
            force: [-2., 0., 0.],
            torque: [0.; 3],
        },
    ];
    let g = [0., 0., -9.81];
    let kick = [0.125, 0., 0.];
    actual.queue_impulse(1, kick).unwrap();
    oracle.queue_impulse(1, kick).unwrap();
    let mut saw_pair = false;
    let mut saw_support = false;
    for tick in 1..=18 {
        let mut pairs = BTreeMap::<(u64, u64), f64>::new();
        let mut supports = BTreeMap::<u64, f64>::new();
        for _ in 0..10 {
            oracle.step_refined_in_field(g, &loads, 1).unwrap();
            let frame = oracle.snapshot();
            for p in frame.impulses {
                *pairs.entry((p.a, p.b)).or_default() += p.normal_impulse;
            }
            for p in frame.supports {
                *supports.entry(p.body).or_default() += p.normal_impulse;
            }
        }
        actual.step_refined_in_field(g, &loads, 10).unwrap();
        let frame = actual.snapshot();
        assert_eq!(frame.tick, tick);
        assert_eq!(frame.bodies, oracle.snapshot().bodies);
        assert_eq!(
            frame.impulses,
            pairs
                .into_iter()
                .map(|((a, b), normal_impulse)| ContactImpulse {
                    a,
                    b,
                    normal_impulse
                })
                .collect::<Vec<_>>()
        );
        assert_eq!(
            frame.supports,
            supports
                .into_iter()
                .map(|(body, normal_impulse)| GroundSupport {
                    body,
                    normal_impulse
                })
                .collect::<Vec<_>>()
        );
        saw_pair |= frame.impulses.iter().any(|p| p.normal_impulse > 0.);
        saw_support |= frame.supports.iter().any(|p| p.normal_impulse > 0.);
        if tick == 1 || tick == 8 {
            let copy = Partition::from_image(&actual.image().unwrap()).unwrap();
            assert_eq!(copy.snapshot(), frame);
            actual = copy;
        }
    }
    assert!(saw_pair && saw_support);
}

#[test]
fn invalid_substeps_or_schedule_fault_never_publish_a_partial_refined_tick() {
    let mut actual = Partition::restore(&seed(SimulationStep::Ms10)).unwrap();
    actual.queue_impulse(1, [0.25, 0., 0.]).unwrap();
    let initial = actual.image().unwrap();
    for bad in [0, 2, 9, 11] {
        assert!(actual.step_refined_in_field([0.; 3], &[], bad).is_err());
        assert_eq!(actual.image().unwrap(), initial);
    }
    let mut wrong_dt = Partition::restore(&seed(SimulationStep::Ms5)).unwrap();
    assert!(wrong_dt.step_refined_in_field([0.; 3], &[], 10).is_err());
    assert_eq!(wrong_dt.snapshot().tick, 0);
    let published = actual.snapshot();
    actual.inject_commit_fault();
    assert!(actual.step_refined_in_field([0.; 3], &[], 10).is_err());
    assert_eq!(actual.snapshot(), published);
    assert!(actual.step().is_err());
}

#[test]
fn same_tick_velocity_edit_keeps_the_impulse_from_the_entire_refined_interval() {
    let mut p = Partition::restore(&seed(SimulationStep::Ms10)).unwrap();
    p.step_refined_in_field([0.; 3], &[], 10).unwrap();
    let before = p.snapshot();
    assert!(before.impulses.iter().any(|v| v.normal_impulse > 1.));
    p.set_velocity(1, [0.; 3], [0.; 3]).unwrap();
    assert_eq!(p.snapshot().impulses, before.impulses);
    assert_eq!(
        Partition::from_image(&p.image().unwrap())
            .unwrap()
            .snapshot(),
        p.snapshot()
    );
    p.retire(2).unwrap();
    assert!(p.snapshot().impulses.is_empty());
    assert_eq!(
        Partition::from_image(&p.image().unwrap())
            .unwrap()
            .snapshot(),
        p.snapshot()
    );
}

#[test]
fn an_impact_that_ends_before_the_tenth_solve_survives_a_warm_transfer() {
    let mut driver = fixture()[0].promoted();
    driver.primitive = Primitive::Sphere;
    driver.size = [0.1; 3];
    driver.position = [-0.16, 0.08, 0.];
    driver.linear = [100., 0., 0.];
    let mut target = driver.clone();
    target.id = 2;
    target.position = [0.; 3];
    target.linear = [0.; 3];
    let mut native = Island::restore(&Snapshot {
        bodies: vec![driver, target],
        ..Default::default()
    })
    .unwrap();
    native.step_refined_with_wrenches(&[], 10).unwrap();
    let receipt = native.snapshot();
    assert!(receipt.impulses.iter().any(|p| p.normal_impulse > 1.));
    let backend = native
        .sim
        .world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .unwrap();
    assert!(
        backend
            .rigid_contact_observations()
            .iter()
            .all(|p| !p.active)
    );
    let archived = native.checkpoint().unwrap();
    let bytes = serde_json::to_vec(&archived).unwrap();
    let mut replaced = Island::restore_warm(&serde_json::from_slice(&bytes).unwrap()).unwrap();
    assert_eq!(replaced.snapshot(), receipt);
    for steps in [1, 10, 1] {
        replaced.step_refined_with_wrenches(&[], steps).unwrap();
        native.step_refined_with_wrenches(&[], steps).unwrap();
        assert_eq!(native.snapshot(), replaced.snapshot());
        assert!(native.snapshot().impulses.is_empty());
    }
}

#[test]
fn split_planner_defers_a_transient_cross_fragment_receipt_for_both_coordinator_and_worker() {
    let mut p = seed(SimulationStep::Ms10);
    p.bodies[0].position[0] = -20.;
    p.bodies[1].position[0] = 20.;
    let component = ComponentInfo {
        ids: vec![1, 2],
        contact_history: true,
    };
    assert_eq!(separated_groups(&p, &component, &[]).unwrap().len(), 2);
    p.impulses.push(ContactImpulse {
        a: 1,
        b: 2,
        normal_impulse: 2.,
    });
    p.contacts = 1;
    p.contact_history = true;
    assert_eq!(
        separated_groups(&p, &component, &[]).unwrap(),
        vec![component]
    );
}

#[test]
fn a_nonrepresentable_normal_impulse_is_rejected_before_the_barrier_can_commit() {
    let half = f64::MAX / 2.;
    let mut loads = BTreeMap::new();
    checked_impulse(&mut loads, (1, 2), half, false).unwrap();
    for _ in 1..9 {
        checked_impulse(&mut loads, (1, 2), 0., false).unwrap();
    }
    // Exactly representable at the boundary; the tenth contribution remains
    // in the native end receipt rather than being counted in the prefix.
    checked_impulse(&mut loads, (1, 2), half, true).unwrap();
    assert_eq!(loads[&(1, 2)], half);
    for (impulse, final_solve) in [
        (f64::MAX, true),
        (f64::MAX, false),
        (f64::INFINITY, true),
        (-1., false),
    ] {
        assert!(checked_impulse(&mut loads, (1, 2), impulse, final_solve).is_err());
        assert_eq!(loads[&(1, 2)], half);
    }
}
