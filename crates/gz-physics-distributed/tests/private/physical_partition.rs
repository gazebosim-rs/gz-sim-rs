// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn body(id: u64, x: f64, y: f64, vx: f64) -> Body {
    Body {
        primitive: Default::default(),
        friction: 0.,
        id,
        tier: Tier::Dynamic,
        size: [0.6; 3],
        mass: 1.,
        position: [x, y, 0.],
        orientation: [0., 0., 0., 1.],
        linear: [vx, 0., 0.],
        angular: [0.; 3],
    }
}
fn seed(bodies: Vec<Body>) -> Snapshot {
    Snapshot {
        bodies,
        ..Snapshot::default()
    }
}

fn pair(a: u64, b: u64, y: f64) -> Vec<Body> {
    let mut second = body(b, 0.35, y + 0.12, -0.2);
    second.mass = 2.;
    second.linear[1] = 0.8;
    vec![body(a, -0.35, y, 2.), second]
}

fn combined(first: &Partition, second: &Partition) -> Snapshot {
    let mut all = first.snapshot();
    let second = second.snapshot();
    assert_eq!(all.tick, second.tick);
    all.contacts += second.contacts;
    all.contact_history |= second.contact_history;
    all.bodies.extend(second.bodies);
    all.impulses.extend(second.impulses);
    all.bodies.sort_by_key(|b| b.id);
    all.impulses.sort_by_key(|i| (i.a, i.b));
    all
}

#[test]
fn selective_handoff_preserves_two_unrelated_warm_solvers_and_matches_monolithic_control() {
    let mut initial = pair(10, 60, 0.);
    initial.extend(pair(20, 50, 16.));
    initial.extend(pair(30, 40, 32.));
    let mut source = Partition::restore(&seed(initial[..4].to_vec())).unwrap();
    let mut receiver = Partition::restore(&seed(initial[4..].to_vec())).unwrap();
    let mut control = Island::restore(&seed(initial)).unwrap();
    while control.snapshot().contacts != 3
        || !source
            .checkpoint_pair([10, 60])
            .unwrap()
            .warmstart_normal_impulses()
            .any(|v| v > 0.)
    {
        source.step().unwrap();
        receiver.step().unwrap();
        control.step().unwrap();
        assert_eq!(combined(&source, &receiver), control.snapshot());
        assert!(control.snapshot().tick < 20);
    }
    let moved = [10, 60];
    let local = [20, 50];
    let peer = [30, 40];
    for (world, id, impulse) in [
        (&mut source, local[0], [0.03, 0., 0.]),
        (&mut receiver, peer[1], [-0.025, 0., 0.]),
    ] {
        world.queue_impulse(id, impulse).unwrap();
        control.queue_impulse(id, impulse).unwrap();
    }
    source.queue_impulse(moved[0], [0.05, 0., 0.]).unwrap();
    control.queue_impulse(moved[0], [0.05, 0., 0.]).unwrap();
    assert!(source.checkpoint_pair([10, 50]).is_err());
    let stay = source.checkpoint_pair(local).unwrap();
    let occupied = receiver.checkpoint_pair(peer).unwrap();
    let moving = source.checkpoint_pair(moved).unwrap();
    assert!(moving.warmstart_normal_impulses().any(|x| x > 0.));
    let record = Island::restore_warm(&moving).unwrap();
    receiver.check_insert(&record).unwrap();
    source.release_pair(moved).unwrap();
    receiver.insert_validated(record);
    // Native bytes include manifold caches, solver island graphs, forces and pending inputs.
    assert_eq!(source.checkpoint_pair(local).unwrap(), stay);
    assert_eq!(receiver.checkpoint_pair(peer).unwrap(), occupied);
    assert_eq!(receiver.checkpoint_pair(moved).unwrap(), moving);
    for _ in 0..120 {
        source.step().unwrap();
        receiver.step().unwrap();
        control.step().unwrap();
        assert_eq!(combined(&source, &receiver), control.snapshot());
    }
    eprintln!(
        "6 bodies, 3 glancing contacts. Handoff at tick {}: {} manifold(s), max warm start {} N s; 120 later barriers bitwise match.",
        moving.snapshot.tick,
        moving.manifold_count,
        moving.warmstart_normal_impulses().fold(0_f64, f64::max)
    );
}

#[test]
fn pending_impulses_trigger_cold_merger_before_contact_without_group_tags() {
    let a = body(1, -0.85, 0., 1.);
    let b = body(2, 0.85, 0., -1.);
    let seed = seed(vec![a, b, body(3, 0., 20., 0.)]);
    let mut part = Partition::restore(&seed).unwrap();
    let mut control = Island::restore(&seed).unwrap();
    assert_eq!(part.topology().len(), 3);
    for w in [1, 2] {
        let impulse = [if w == 1 { 34. } else { -34. }, 0., 0.];
        part.queue_impulse(w, impulse).unwrap();
        control.queue_impulse(w, impulse).unwrap();
    }
    let mut saw_impact = false;
    for _ in 0..8 {
        part.check_step().unwrap();
        part.step().unwrap();
        control.step().unwrap();
        assert_eq!(part.snapshot(), control.snapshot());
        saw_impact |= part
            .snapshot()
            .impulses
            .iter()
            .any(|p| p.normal_impulse > 1.);
    }
    assert!(saw_impact);
    assert_eq!(part.topology()[0].ids, vec![1, 2]);
    assert_eq!(part.topology()[1].ids, vec![3]);
}

#[test]
fn bounds_include_secondary_deflections_and_preflight_preserves_the_live_warm_solver() {
    let mut part = Partition::restore(&seed(pair(1, 2, 0.))).unwrap();
    while part.snapshot().contacts == 0 {
        part.step().unwrap();
    }
    let warm = part.checkpoint_pair([1, 2]).unwrap();
    let resident = |y| {
        Partition::restore(&Snapshot {
            tick: warm.snapshot.tick,
            ..seed(vec![body(3, 0., y, 0.)])
        })
        .unwrap()
    };
    let near = resident(3.5);
    assert!(partitions_may_interact(
        &part.snapshot(),
        &part.topology(),
        &near.snapshot(),
        &near.topology(),
        &[]
    ));
    near.check_insert(&Island::restore_warm(&warm).unwrap())
        .unwrap();
    assert!(matches!(
        resident(0.).check_insert(&Island::restore_warm(&warm).unwrap()),
        Err(WorldError::CrossIsland)
    ));
    // Both cold components can reach the warm pair in one tick. Planning one
    // staged append must leave the current native cache and inputs untouched.
    let mut far = Partition::restore(&Snapshot {
        tick: warm.snapshot.tick,
        ..seed(vec![body(3, 0., 7., 0.), body(4, 0., -7., 0.)])
    })
    .unwrap();
    assert!(!partitions_may_interact(
        &part.snapshot(),
        &part.topology(),
        &far.snapshot(),
        &far.topology(),
        &[]
    ));
    far.check_insert(&Island::restore_warm(&warm).unwrap())
        .unwrap();
    far.insert_validated(Island::restore_warm(&warm).unwrap());
    far.queue_impulse(3, [0., -300., 0.]).unwrap();
    far.queue_impulse(4, [0., 300., 0.]).unwrap();
    let before = far.snapshot();
    let selected = far.checkpoint_pair([1, 2]).unwrap();
    far.check_step().unwrap();
    assert_eq!(far.snapshot(), before);
    assert_eq!(far.checkpoint_pair([1, 2]).unwrap(), selected);
    far.step().unwrap();
    assert_eq!(far.topology().len(), 1);
    assert!(far.pending.is_empty());
}

#[test]
fn rotating_free_boxes_merge_before_glancing_contact_and_continue_against_native_control() {
    let mut a = body(1, -1.3, 0., 2.);
    let mut b = body(2, 1.3, 0.10, -0.4);
    a.size = [0.64, 0.46, 0.38];
    b.size = [0.52, 0.40, 0.66];
    a.orientation = [0., 0., 0.16_f64.sin(), 0.16_f64.cos()];
    b.orientation = [0.21_f64.sin(), 0., 0., 0.21_f64.cos()];
    a.angular = [0.18, 0.09, 0.7];
    b.angular = [0.06, -0.03, -0.4];
    a.linear[1] = 0.12;
    b.linear[2] = 0.02;
    let scene = seed(vec![a, b, body(3, 0., 25., 0.)]);
    let mut parted = Partition::restore(&scene).unwrap();
    let mut native = Island::restore(&scene).unwrap();
    let mut max_position = 0.0_f64;
    let mut max_quaternion = 0.0_f64;
    let mut max_linear = 0.0_f64;
    let mut max_angular = 0.0_f64;
    let mut max_impulse = 0.0_f64;
    let mut first_merge = None;
    let mut impact = None;
    let mut peak = 0.0_f64;
    for tick in 1..=220 {
        parted.check_step().unwrap();
        parted.step().unwrap();
        native.step().unwrap();
        if parted.topology().len() == 2 {
            first_merge.get_or_insert(tick);
        }
        let actual = parted.snapshot();
        let expected = native.snapshot();
        assert_eq!(actual.contacts, expected.contacts);
        for (a, b) in actual.bodies.iter().zip(&expected.bodies) {
            let err = |a: &[f64], b: &[f64]| {
                a.iter()
                    .zip(b)
                    .map(|(x, y)| (x - y).abs())
                    .fold(0.0_f64, f64::max)
            };
            max_position = max_position.max(err(&a.position, &b.position));
            max_quaternion = max_quaternion.max(err(&a.orientation, &b.orientation));
            max_linear = max_linear.max(err(&a.linear, &b.linear));
            max_angular = max_angular.max(err(&a.angular, &b.angular));
        }
        for (p, q) in actual.impulses.iter().zip(expected.impulses) {
            assert_eq!((p.a, p.b), (q.a, q.b));
            max_impulse = max_impulse.max((p.normal_impulse - q.normal_impulse).abs());
            peak = peak.max(q.normal_impulse);
            if q.normal_impulse > 1e-4 {
                impact.get_or_insert(tick);
            }
        }
    }
    eprintln!(
        "rotating cold merge tick {first_merge:?}, glancing impact tick {impact:?}, peak {peak:.9} N s, max errors [m, quat, m/s, rad/s, N s] = [{max_position:e},{max_quaternion:e},{max_linear:e},{max_angular:e},{max_impulse:e}]"
    );
    assert!(first_merge.unwrap() < impact.unwrap() && peak > 0.1);
    assert!(
        [
            max_position,
            max_quaternion,
            max_linear,
            max_angular,
            max_impulse
        ]
        .into_iter()
        .all(|x| x < 1e-10)
    );
}

#[test]
fn failed_tick_keeps_the_last_complete_component_topology_and_public_state() {
    let mut part = Partition::restore(&seed(vec![
        body(1, -0.85, 0., 1.),
        body(2, 0.85, 0., -1.),
        body(3, 0., 20., 0.),
    ]))
    .unwrap();
    part.queue_impulse(1, [34., 0., 0.]).unwrap();
    part.queue_impulse(2, [-34., 0., 0.]).unwrap();
    let owner = part.owner(3).unwrap();
    part.solvers[owner].inject_commit_fault();
    let frame = part.snapshot();
    let topology = part.topology();
    assert_eq!(topology.len(), 3);
    assert!(matches!(part.step(), Err(WorldError::Fault(_))));
    assert_eq!(part.snapshot(), frame);
    assert_eq!(part.topology(), topology);
    assert!(matches!(
        part.queue_impulse(1, [0.; 3]),
        Err(WorldError::Fault(_))
    ));
}

#[test]
fn failed_local_solver_keeps_premerge_topology_and_last_complete_barrier() {
    let scene = seed(vec![
        body(1, 0., -20., 1.),
        body(2, 0., 20., -1.),
        body(3, -0.85, 0., 1.),
        body(4, 0.85, 0., -1.),
    ]);
    let mut part = Partition::restore(&scene).unwrap();
    let before = part.snapshot();
    let topology = part.topology();
    assert_eq!(topology.len(), 4);
    part.queue_impulse(3, [34., 0., 0.]).unwrap();
    part.queue_impulse(4, [-34., 0., 0.]).unwrap();
    // The next tick creates a new solver for 3 and 4 and integrates solver 1
    // before the injected failure on 2. Neither change can be published.
    part.solvers[1].inject_commit_fault();
    assert!(matches!(part.step(), Err(WorldError::Fault(_))));
    assert_eq!(part.snapshot(), before);
    assert_eq!(part.topology(), topology);
    assert!(matches!(part.step(), Err(WorldError::Fault(_))));
    assert!(matches!(part.export(1), Err(WorldError::Fault(_))));
}

#[test]
fn cold_import_checks_native_geometry_and_leaves_the_resident_queue_intact_on_refusal() {
    let a = body(1, -2., 0., 0.);
    let b = body(2, 2., 0., 0.);
    let mut part = Partition::restore(&seed(vec![a.clone()])).unwrap();
    let initial = part.snapshot();
    assert!(matches!(
        part.validate_import(&a),
        Err(WorldError::Duplicate(1))
    ));
    let mut overlap = b.clone();
    overlap.position = a.position;
    assert!(part.import(overlap).is_err());
    assert_eq!(part.snapshot(), initial);
    part.validate_import(&b).unwrap();
    part.queue_impulse(1, [0.5, 0., 0.]).unwrap();
    part.import(b).unwrap();
    assert!(matches!(part.remove(1), Err(WorldError::Pending)));
    part.step().unwrap();
    assert_eq!(part.snapshot().bodies[0].linear[0], 0.5);
    assert_eq!(part.topology().len(), 2);
}

#[test]
fn warm_admission_requires_a_complete_current_two_body_image() {
    let incoming = Island::restore(&seed(pair(1, 2, 0.))).unwrap();
    let mut empty = Partition::restore(&Snapshot::default()).unwrap();
    empty.check_insert(&incoming).unwrap();
    let solo = Island::restore(&seed(vec![body(1, 0., 0., 0.)])).unwrap();
    assert!(matches!(
        empty.check_insert(&solo),
        Err(WorldError::Checkpoint)
    ));
    empty.step().unwrap();
    assert!(matches!(
        empty.check_insert(&incoming),
        Err(WorldError::Checkpoint)
    ));

    let mut warm = Partition::restore(&seed(pair(1, 2, 0.))).unwrap();
    while warm.snapshot().contacts == 0 {
        warm.step().unwrap();
        assert!(warm.snapshot().tick < 20);
    }
    let duplicate = Island::restore_warm(&warm.checkpoint_pair([1, 2]).unwrap()).unwrap();
    assert!(matches!(
        warm.check_insert(&duplicate),
        Err(WorldError::Duplicate(1))
    ));
    assert!(warm.release_pair([2, 1]).is_err());
    assert_eq!(warm.snapshot(), duplicate.snapshot());
}

fn converging_row(count: usize) -> Vec<Body> {
    (0..count)
        .map(|i| {
            let mut b = body(
                10 + i as u64,
                0.625 * i as f64,
                0.,
                (count - 1) as f64 * 0.5 - i as f64,
            );
            b.size = [0.5; 3];
            b
        })
        .collect()
}

#[test]
fn connected_triple_moves_with_ordered_inputs_and_other_workers_keep_their_solvers() {
    let mut source_bodies = converging_row(3);
    let ids = [10, 11, 12];
    source_bodies.extend(pair(20, 21, 18.));
    let target_bodies = pair(30, 31, 36.);
    let mut all = source_bodies.clone();
    all.extend_from_slice(&target_bodies);
    let mut src = Partition::restore(&seed(source_bodies)).unwrap();
    let mut dst = Partition::restore(&seed(target_bodies)).unwrap();
    let mut reference = Island::restore(&seed(all)).unwrap();
    assert!(src.checkpoint_group(&ids).is_err());
    // Drive both ends inward so genuine cached normal impulses persist.
    for _ in 0..60 {
        for (id, impulse) in [(10, [0.04, 0., 0.]), (12, [-0.04, 0., 0.])] {
            src.queue_impulse(id, impulse).unwrap();
            reference.queue_impulse(id, impulse).unwrap();
        }
        src.step().unwrap();
        dst.step().unwrap();
        reference.step().unwrap();
        assert_eq!(combined(&src, &dst), reference.snapshot());
        if let Ok(c) = src.checkpoint_group(&ids)
            && c.warmstart_normal_impulses().any(|i| i > 1e-6)
        {
            break;
        }
    }
    assert!(active_contact_group(&src.snapshot(), &ids));
    for proposed in [
        &ids[..2],
        &[10, 12],
        &[10, 11, 21],
        &[11, 10, 12],
        &[11, 11],
    ] {
        assert!(!active_contact_group(&src.snapshot(), proposed));
        assert!(src.checkpoint_group(proposed).is_err());
        assert!(src.release_group(proposed).is_err());
    }
    for (id, impulse) in [
        (12, [-0.05, 0., 0.]),
        (20, [0.01, 0., 0.]),
        (11, [0., 0.02, 0.]),
        (10, [0.07, 0., 0.]),
    ] {
        src.queue_impulse(id, impulse).unwrap();
        reference.queue_impulse(id, impulse).unwrap();
    }
    let before_local = src.checkpoint_pair([20, 21]).unwrap();
    let before_remote = dst.checkpoint_pair([30, 31]).unwrap();
    let moving = src.checkpoint_group(&ids).unwrap();
    assert_eq!(
        moving.pending.iter().map(|c| c.id).collect::<Vec<_>>(),
        [12, 11, 10]
    );
    assert_eq!(moving.manifold_count, 2);
    assert!(moving.warmstart_normal_impulses().any(|v| v > 1e-6));
    let restored = Island::restore_warm(&moving).unwrap();
    dst.check_insert(&restored).unwrap();
    src.release_group(&ids).unwrap();
    dst.insert_validated(restored);
    assert_eq!(src.checkpoint_pair([20, 21]).unwrap(), before_local);
    assert_eq!(dst.checkpoint_pair([30, 31]).unwrap(), before_remote);
    assert_eq!(dst.checkpoint_group(&ids).unwrap(), moving);
    for _ in 0..100 {
        src.step().unwrap();
        dst.step().unwrap();
        reference.step().unwrap();
        assert_eq!(combined(&src, &dst), reference.snapshot());
    }
    eprintln!(
        "triple at tick {} native={} B, {} manifolds, {} point caches, max warmstart {:.12} N s, 3 ordered commands, 100 subsequent bitwise matching barriers",
        moving.snapshot.tick,
        moving.native_bytes(),
        moving.manifold_count,
        moving.warmstart_normal_impulses().count(),
        moving.warmstart_normal_impulses().fold(0_f64, f64::max)
    );
}

#[test]
fn maximum_connected_body_count_restores_every_cache_and_state_bit() {
    let count = MAX_CHECKPOINT_BODIES;
    let scene = seed(converging_row(count));
    let ids: Vec<_> = scene.bodies.iter().map(|b| b.id).collect();
    let mut partition = Partition::restore(&scene).unwrap();
    let mut reference = Island::restore(&scene).unwrap();
    let mut archive = None;
    for _ in 0..40 {
        partition.step().unwrap();
        reference.step().unwrap();
        assert_eq!(partition.snapshot(), reference.snapshot());
        if let Ok(candidate) = partition.checkpoint_group(&ids)
            && candidate.warmstart_normal_impulses().any(|i| i > 1e-6)
        {
            archive = Some(candidate);
            break;
        }
    }
    let archive = archive.expect("simultaneous active contacts along the full row");
    let native_len = archive.native_bytes();
    let json = serde_json::to_vec(&archive).unwrap();
    let wire: WarmCheckpoint = serde_json::from_slice(&json).unwrap();
    assert_eq!(wire, archive);
    let mut copy = Island::restore_warm(&wire).unwrap();
    assert_eq!(copy.checkpoint().unwrap(), archive);
    assert_eq!(copy.snapshot().contacts, count - 1);
    for _ in 0..15 {
        copy.step().unwrap();
        reference.step().unwrap();
        assert_eq!(copy.snapshot(), reference.snapshot());
    }
    eprintln!(
        "{count} contacting boxes, {} manifolds, {} point caches, native {native_len} B, wire checkpoint {} B, tick {}",
        archive.manifold_count,
        archive.warmstart_normal_impulses().count(),
        json.len(),
        archive.snapshot.tick
    );
}

/// Insert directly into the already-running native control, independently of the
/// partition planner, checkpoint serialization and staged solver replacement.
fn append_control(island: &mut Island, b: Body) {
    let id = b.id;
    let native = {
        let mut backend = island.sim.world_mut().resource_mut::<PhysicsBackend>();
        spawn_box(
            backend.downcast_mut::<MultiFidelityBackend>().unwrap(),
            &b,
            SimulationStep::default(),
        )
        .unwrap()
    };
    let entity = spawn_binding(&mut island.sim, id, native).unwrap();
    island.index.insert(id, entity);
    island.committed.bodies.push(b);
    island.committed.bodies.sort_by_key(|b| b.id);
}

#[test]
fn moving_cold_box_joins_loaded_warm_pair_preserving_cached_contact_and_resident_ecs() {
    for ids in [[1, 2, 3], [2, 3, 1]] {
        let [left, right, incoming] = ids;
        let mut a = body(left, -0.316, 0., 0.5);
        let mut b = body(right, 0.316, 0., -0.5);
        let mut c = body(incoming, 5.62, 0., -8.);
        for b in [&mut a, &mut b, &mut c] {
            b.size = [0.6; 3];
        }
        let world = seed(vec![a.clone(), b.clone(), c.clone()]);
        let mut parted = Partition::restore(&world).unwrap();
        let mut mono = Island::restore(&world).unwrap();
        let mut control = Island::restore(&seed(vec![a, b])).unwrap();
        let mut free = Some(Island::restore(&seed(vec![c])).unwrap());
        let mut joined = None;
        let mut impact = None;
        let mut peak_diff = 0.0_f64;
        let mut peak_impulse_diff = 0.0_f64;
        let mut retained = 0.0;
        let mut simultaneous = false;
        for tick in 1..=130 {
            let commands = [(left, [0.015, 0., 0.]), (right, [-0.015, 0., 0.])];
            for (id, impulse) in commands {
                parted.queue_impulse(id, impulse).unwrap();
                mono.queue_impulse(id, impulse).unwrap();
                control.queue_impulse(id, impulse).unwrap();
            }
            let mut resident_identity = None;
            if free.is_some()
                && potential_contact_groups(&parted.committed, &parted.components, &parted.pending)
                    .unwrap()
                    .iter()
                    .any(|g| g.len() == 2)
            {
                let i = parted.owner(left).unwrap();
                let warm = &parted.solvers[i];
                let eid = warm.index[&left];
                let stable = warm
                    .sim
                    .world()
                    .get::<gz_sim_core::EntityId>(eid)
                    .unwrap()
                    .get();
                let original_world = warm.sim.world().id();
                let old_handle = warm.sim.world().get::<Binding>(eid).unwrap().native;
                retained = warm
                    .checkpoint()
                    .unwrap()
                    .warmstart_normal_impulses()
                    .fold(0_f64, f64::max);
                let before = warm.committed.impulses.clone();
                assert!(!before.is_empty());
                let cold = free.take().unwrap().snapshot().bodies.pop().unwrap();
                append_control(&mut control, cold);
                let plan = parted.plan_merges().unwrap();
                assert_eq!(plan.appends.len(), 1);
                let witness = plan.appends[0].backend.partition_warmstart();
                let live = warm
                    .sim
                    .world()
                    .resource::<PhysicsBackend>()
                    .downcast_ref::<MultiFidelityBackend>()
                    .unwrap();
                assert_eq!(witness, live.partition_warmstart());
                resident_identity = Some((original_world, eid, stable, old_handle));
                joined = Some(tick);
            }
            parted.step().unwrap();
            if let Some((world, entity, stable, native)) = resident_identity {
                let s = &parted.solvers[parted.owner(left).unwrap()];
                assert_eq!(s.sim.world().id(), world);
                assert_eq!(s.index[&left], entity);
                assert_eq!(
                    s.sim
                        .world()
                        .get::<gz_sim_core::EntityId>(entity)
                        .unwrap()
                        .get(),
                    stable
                );
                assert_eq!(s.sim.world().get::<Binding>(entity).unwrap().native, native);
            }
            mono.step().unwrap();
            control.step().unwrap();
            if let Some(free) = free.as_mut() {
                free.step().unwrap();
            } else {
                assert_eq!(parted.committed, control.committed);
            }
            let actual = parted.snapshot();
            let expected = mono.snapshot();
            for (a, b) in actual.bodies.iter().zip(&expected.bodies) {
                let values = |b: &Body| {
                    b.position
                        .into_iter()
                        .chain(b.orientation)
                        .chain(b.linear)
                        .chain(b.angular)
                };
                for (x, y) in values(a).zip(values(b)) {
                    peak_diff = peak_diff.max((x - y).abs());
                }
            }
            for p in &actual.impulses {
                let reference = expected
                    .impulses
                    .iter()
                    .find(|q| (q.a, q.b) == (p.a, p.b))
                    .map_or(0., |q| q.normal_impulse);
                peak_impulse_diff = peak_impulse_diff.max((p.normal_impulse - reference).abs());
            }
            if actual
                .impulses
                .iter()
                .any(|p| (p.a == incoming || p.b == incoming) && p.normal_impulse > 1e-3)
            {
                impact.get_or_insert(tick);
                simultaneous |= actual.contacts >= 2;
            }
        }
        eprintln!(
            "warm+cold global ids {ids:?}, merge {joined:?}, new impact {impact:?}, retained {retained:e} N s, simultaneous={simultaneous}, 130 ticks, same-event native control bitwise, max monolithic state {peak_diff:e} / impulse {peak_impulse_diff:e}"
        );
        assert!(joined.unwrap() < impact.unwrap() && simultaneous && retained > 1e-4);
        assert!(peak_diff < 1e-8 && peak_impulse_diff < 1e-8);
    }
}

#[test]
fn warm_append_accepts_a_cold_component_and_resumes_boxes_with_different_insertion_epochs() {
    let first = body(1, -0.316, 0., 2.5);
    let second = body(2, 0.316, 0., 1.5);
    let mut source = Island::restore(&seed(vec![first.clone(), second])).unwrap();
    for _ in 0..200 {
        source.queue_impulse(1, [0.015, 0., 0.]).unwrap();
        source.queue_impulse(2, [-0.015, 0., 0.]).unwrap();
        source.step().unwrap();
    }
    assert!(active_contact_group(&source.snapshot(), &[1, 2]));
    // This newly admitted cold box is at the warm body's *construction* pose;
    // the warm body is now roughly four metres away at its current pose.
    let cold = seed(vec![
        body(7, first.position[0], 0.8, 0.),
        body(5, first.position[0], 0., 0.),
    ]);
    let mut partition = Partition::restore(&Snapshot { tick: 200, ..cold }).unwrap();
    assert_eq!(partition.topology()[0].ids, vec![5, 7]);
    partition.check_insert(&source).unwrap();
    partition.insert_validated(source);
    let plan = partition.plan_merges().unwrap();
    assert_eq!(plan.appends.len(), 1);
    assert_eq!(
        plan.appends[0]
            .new_bodies
            .iter()
            .map(|(b, _)| b.id)
            .collect::<Vec<_>>(),
        [5, 7]
    );
    partition.step().unwrap();
    assert_eq!(partition.topology()[0].ids, vec![1, 2, 5, 7]);
    let merged = &partition.solvers[0];
    let saved = merged.checkpoint().unwrap();
    let mut resume = Island::restore_warm(&saved).unwrap();
    for _ in 0..20 {
        partition.step().unwrap();
        resume.step().unwrap();
        assert_eq!(partition.snapshot(), resume.snapshot());
    }
}

#[test]
fn incompatible_warm_joins_preserve_both_warm_caches_and_command_ledgers() {
    let loaded = |ids: [u64; 2], y| {
        let mut island = Island::restore(&seed(pair(ids[0], ids[1], y))).unwrap();
        for _ in 0..5 {
            island.step().unwrap();
        }
        island
    };
    let a = loaded([1, 2], 0.);
    // The remote members already overlap the resident geometry at this barrier.
    let b = loaded([3, 4], 0.15);
    let mut local = Partition::restore(&Snapshot {
        tick: 5,
        ..Snapshot::default()
    })
    .unwrap();
    local.insert_validated(a);
    local.queue_impulse(1, [0.05, 0., 0.]).unwrap();
    let snapshot = local.snapshot();
    let before = local.checkpoint_pair([1, 2]).unwrap();
    let remote = b.checkpoint().unwrap();
    assert!(matches!(
        local.check_insert(&b),
        Err(WorldError::CrossIsland)
    ));
    assert_eq!(local.snapshot(), snapshot);
    assert_eq!(local.checkpoint_pair([1, 2]).unwrap(), before);
    assert_eq!(b.checkpoint().unwrap(), remote);
    let warm = local.topology();
    let mut crowded = snapshot.clone();
    let mut members = Vec::new();
    for id in 10..41 {
        crowded
            .bodies
            .push(body(id, 0., 3. + 0.001 * id as f64, 0.));
        members.push(id);
    }
    let mut groups = warm.clone();
    groups.push(ComponentInfo {
        ids: members,
        contact_history: false,
    });
    assert!(!partition_can_step(&crowded, &groups, &[]));
    let mut mixed = snapshot;
    let mut prescribed = body(9, 0., 3., 0.);
    prescribed.tier = Tier::Prescribed;
    mixed.bodies.push(prescribed);
    let mut groups = warm;
    groups.push(ComponentInfo {
        ids: vec![9],
        contact_history: false,
    });
    assert!(partition_can_step(&mixed, &groups, &[]));
}

#[test]
fn simultaneous_append_of_two_cold_solvers_preserves_the_native_resident_and_input_order() {
    let seeds = seed(vec![body(2, -0.316, 0., 0.5), body(3, 0.316, 0., -0.5)]);
    let mut parted = Partition::restore(&seeds).unwrap();
    let mut direct = Island::restore(&seeds).unwrap();
    let impulse = loop {
        for (id, j) in [(2, 0.015), (3, -0.015)] {
            parted.queue_impulse(id, [j, 0., 0.]).unwrap();
            direct.queue_impulse(id, [j, 0., 0.]).unwrap();
        }
        parted.step().unwrap();
        direct.step().unwrap();
        let j = parted
            .checkpoint_pair([2, 3])
            .unwrap()
            .warmstart_normal_impulses()
            .fold(0., f64::max);
        if j > 1.0e-6 {
            break j;
        }
        assert!(
            parted.snapshot().tick < 30,
            "positive normal impulse prior to join"
        );
    };
    // Arrival order interleaves the resident global IDs. The staged native
    // append always assigns incoming handles in global-ID order.
    let left = body(1, -1.45, 0., 2.);
    let right = body(4, 1.45, 0., -2.);
    parted.import(right.clone()).unwrap();
    parted.import(left.clone()).unwrap();
    let components = parted.topology();
    assert_eq!(components.len(), 3);
    let plan = parted.plan_merges().unwrap();
    assert_eq!(plan.appends.len(), 1);
    assert_eq!(
        plan.appends[0]
            .new_bodies
            .iter()
            .map(|(b, _)| b.id)
            .collect::<Vec<_>>(),
        [1, 4]
    );
    assert_eq!(plan.retired.len(), 2);
    append_control(&mut direct, left);
    append_control(&mut direct, right);
    let mut active = 0;
    for _ in 0..130 {
        for (id, j) in [(4, -0.01), (1, 0.01), (4, -0.025), (1, 0.025)] {
            parted.queue_impulse(id, [j, 0., 0.]).unwrap();
            direct.queue_impulse(id, [j, 0., 0.]).unwrap();
        }
        parted.step().unwrap();
        direct.step().unwrap();
        assert_eq!(parted.snapshot(), direct.snapshot());
        active = active.max(parted.snapshot().contacts);
    }
    assert_eq!(active, 3);
    eprintln!(
        "multi-append retained {impulse:.9} N s, {active} simultaneous pairs, 130 exact ticks"
    );
}

#[test]
fn cold_admission_bounds_match_native_insertion_including_rounded_prediction_edges() {
    // These are public barrier states; the current poses can differ from
    // construction. Use native materialization as the independent control.
    for angle in [0., 0.07_f64, 0.61, 1.3] {
        let mut a = body(1, 0., 0., 0.);
        let mut b = body(2, 0., 0., 0.);
        a.size = [1.25, 0.4, 0.8];
        b.size = [0.5, 0.7, 0.6];
        a.orientation = [0., 0., angle.sin(), angle.cos()];
        b.orientation = [0., 0., (-angle * 0.8).sin(), (-angle * 0.8).cos()];
        let resident = Partition::restore(&seed(vec![a.clone()])).unwrap();
        let mut outcomes = [0; 2];
        for i in 0..120 {
            b.position[0] = 0.4 + (i as f64) / 100.;
            let actual = resident.validate_import(&b);
            let control = materialize(
                &[a.clone(), b.clone()],
                SimulationStep::default(),
                None,
                NativeSolverProfile::Standard32,
            );
            assert_eq!(
                actual.is_ok(),
                control.is_ok(),
                "angle {angle}, x {}",
                b.position[0]
            );
            outcomes[usize::from(actual.is_ok())] += 1;
        }
        assert!(outcomes.iter().all(|&n| n > 1));
    }
    let a = body(1, 0., 0., 0.);
    let mut b = body(2, 0., 0., 0.);
    let resident = Partition::restore(&seed(vec![a.clone()])).unwrap();
    let edge = 0.6 + gz_physics_multifidelity::partition_prediction_distance();
    for x in [edge.next_down(), edge, edge.next_up()] {
        b.position[0] = x;
        assert_eq!(
            resident.validate_import(&b).is_ok(),
            materialize(
                &[a.clone(), b.clone()],
                SimulationStep::default(),
                None,
                NativeSolverProfile::Standard32
            )
            .is_ok()
        );
    }
}

#[test]
fn stationary_prescribed_singleton_remains_checkpointable_at_every_barrier() {
    let mut b = body(4, 0., 12., 0.);
    b.tier = Tier::Prescribed;
    let mut part = Partition::restore(&seed(vec![b])).unwrap();
    for _ in 0..75 {
        let image = part.checkpoint_component(&[4]).unwrap();
        assert_eq!(
            Island::restore_solver_with_profile(&image, NativeSolverProfile::Standard32)
                .unwrap()
                .snapshot(),
            part.snapshot()
        );
        part.step().unwrap();
    }
}

#[test]
fn worker_local_helper_respects_probe_window_and_failed_barrier() {
    let initial = seed(vec![
        body(1, 0., 0., 0.),
        body(2, 20., 0., 0.),
        body(3, 40., 0., 0.),
    ]);
    let mut parallel = Partition::restore(&initial).unwrap();
    let mut serial = Partition::restore(&initial).unwrap();
    parallel.helper_allowed = true;
    parallel.start_probe(0, 1).unwrap();
    let schedule = SolveSchedule::Islands(&[1, 3]);
    for tick in 1..=3 {
        for world in [&mut parallel, &mut serial] {
            world.queue_impulse(2, [0., 0.11, 0.]).unwrap();
            world.queue_impulse(3, [0., -0.12, 0.]).unwrap();
            world
                .step_scheduled([0., 0., -9.81], &[], schedule)
                .unwrap();
        }
        assert_eq!(parallel.snapshot(), serial.snapshot());
        assert_eq!(parallel.helper.is_some(), tick > 1);
        if tick == 1 {
            assert_eq!(parallel.read_probe(0, 1).unwrap().len(), 3);
        }
    }
    let before = parallel.snapshot();
    for (index, reason) in [(1, "first batch failed"), (2, "second batch failed")] {
        parallel.solvers[index].sim.add_systems(
            gz_sim_core::Phase::PhysicsCommit,
            move |fault: bevy_ecs::prelude::Res<gz_sim_core::StepFault>| {
                fault.report(std::io::Error::other(reason));
            },
        );
    }
    let error = parallel.step_scheduled([0.; 3], &[], schedule).unwrap_err();
    assert!(matches!(error, WorldError::Fault(reason) if reason.contains("first batch failed")));
    assert_eq!(parallel.snapshot(), before);
    assert!(matches!(parallel.step(), Err(WorldError::Fault(_))));
}

#[test]
fn shared_queue_preserves_first_error_in_canonical_world_order() {
    let initial = seed(
        (1..=4)
            .map(|id| body(id, 20. * id as f64, 0., 0.))
            .collect(),
    );
    for failing_indices in [[0, 1], [1, 2]] {
        let mut partition = Partition::restore(&initial).unwrap();
        // Use one larger native world between two one-body worlds. All are
        // independent and use the same native timestep.
        let middle = Island::restore(&seed(initial.bodies[1..3].to_vec())).unwrap();
        partition.solvers.splice(1..3, [middle]);
        partition.publish(0);
        partition.helper_allowed = true;
        let schedule = SolveSchedule::Uniform(10);
        assert_eq!(
            lanes::plan(&partition.solvers, schedule),
            Some(lanes::Plan::Shared)
        );
        partition.start_probe_in_mode(0, 1, true).unwrap();
        let mut control = Partition::restore(&initial).unwrap();
        let middle = Island::restore(&seed(initial.bodies[1..3].to_vec())).unwrap();
        control.solvers.splice(1..3, [middle]);
        control.publish(0);
        control.step_scheduled([0.; 3], &[], schedule).unwrap();
        partition.step_scheduled([0.; 3], &[], schedule).unwrap();
        assert_eq!(partition.snapshot(), control.snapshot());
        let work = partition.read_probe(0, 1).unwrap();
        assert_eq!(
            work.iter()
                .map(|row| row.members.as_slice())
                .collect::<Vec<_>>(),
            [&[1][..], &[2, 3][..], &[4][..]]
        );
        assert!(work.iter().all(|row| row.elapsed_ns > 0));
        partition.start_probe_in_mode(1, 1, true).unwrap();
        for i in failing_indices {
            partition.solvers[i].sim.add_systems(
                gz_sim_core::Phase::PhysicsCommit,
                move |fault: bevy_ecs::prelude::Res<gz_sim_core::StepFault>| {
                    fault.report(std::io::Error::other(format!("failed native world {i}")));
                },
            );
        }
        let before = partition.snapshot();
        let error = partition
            .step_scheduled([0.; 3], &[], schedule)
            .unwrap_err();
        let expected = format!("failed native world {}", failing_indices[0]);
        assert!(matches!(error, WorldError::Fault(reason) if reason.contains(&expected)));
        assert!(partition.helper.is_some());
        assert_eq!(partition.snapshot(), before);
    }
}
