// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{
    BodyId, BodySpec, BodyState, DynamicBackend, Shape,
    trajectory::{PathEnd, TranslationSpline, Waypoint},
};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::time::Duration;

fn track() -> TranslationSpline {
    TranslationSpline::new(
        &[
            Waypoint {
                time: Duration::ZERO,
                displacement: [0.; 3],
                velocity: [0.; 3],
            },
            Waypoint {
                time: Duration::from_millis(100),
                displacement: [1., 0., 0.],
                velocity: [0.; 3],
            },
            Waypoint {
                time: Duration::from_millis(200),
                displacement: [0.; 3],
                velocity: [0.; 3],
            },
        ],
        PathEnd::Repeat,
    )
    .unwrap()
}
fn spec(x: f64, z: f64) -> BodySpec {
    BodySpec {
        position: [x, 0., z],
        velocity: [0.; 3],
        mass: Some(1.),
        shape: Shape::Box {
            half_extents: [0.45, 0.3, 0.1],
        },
    }
}
fn setup() -> (MultiFidelityBackend, [BodyId; 2]) {
    let mut backend = MultiFidelityBackend::new([0., 0., -9.81]).unwrap();
    let ids = [0., 5.].map(|x| {
        let id = backend
            .spawn_kinematic_link(spec(x, 0.), Default::default(), None)
            .unwrap();
        backend.set_initial_translation_path(id, track()).unwrap();
        id
    });
    (backend, ids)
}
fn published(b: &MultiFidelityBackend, ids: [BodyId; 2]) -> [BodyState; 2] {
    let states = ids.map(|id| b.state(id).unwrap());
    b.read_states(&mut |lookup| {
        for i in [1, 0, 1] {
            assert_eq!(lookup.state(ids[i]), Some(states[i]));
        }
    });
    states
}
#[test]
fn pure_curves_materialize_at_the_same_position_and_tangent_for_rays_edits_and_contact() {
    let (mut b, ids) = setup();
    let dt = Duration::from_millis(50);
    b.step(dt).unwrap();
    let s = published(&b, ids);
    assert_eq!(s[0].position, [0.5, 0., 0.]);
    assert!((s[0].velocity[0] - 15.).abs() < 1e-12);
    assert_eq!(s[1].position, [5.5, 0., 0.]);
    assert_eq!(b.translation_interval().0, 0);
    assert!(b.translation_interval().1.is_empty());
    assert_eq!(
        b.cast_ray([0.5, 0., 1.], [0., 0., -1.], 2., None).unwrap(),
        Some(0.9)
    );
    assert!(
        b.set_world_velocity(ids[0], [f64::NAN; 3], [0.; 3])
            .is_err()
    );
    b.set_world_velocity(ids[0], [1., 0., 0.], [0.; 3]).unwrap();
    b.step(dt).unwrap();
    let s = published(&b, ids);
    assert_eq!(s[0].position, [0.55, 0., 0.]);
    assert_eq!(s[1].position, [6., 0., 0.]);
    let payload = b
        .spawn_link(spec(6., 0.20001), Default::default(), None)
        .unwrap();
    assert_eq!(published(&b, ids), s);
    let start = b.state(payload).unwrap();
    b.add_world_wrench(payload, [0., 0., 3.], [0.; 3]).unwrap();
    assert!(b.step(Duration::from_secs(10)).is_err());
    let before = published(&b, ids);
    let h = Duration::from_millis(5);
    b.step(h).unwrap();
    assert_ne!(published(&b, ids), before);
    let (cuts, hits) = b.translation_interval();
    assert_eq!(cuts, 10);
    let expected = b.state(payload).unwrap().velocity[2] - start.velocity[2] + (9.81 - 3.) * 0.005;
    assert!((hits.iter().map(|h| h.normal_impulse).sum::<f64>() - expected).abs() < 1e-9);
    b.step(h).unwrap();
    b.remove(payload);
    b.reset().unwrap();
    b.step(dt).unwrap();
    assert_eq!(published(&b, ids)[1].position, [5.5, 0., 0.]);
    assert_eq!(b.translation_interval().0, 0);
}
#[test]
fn pure_curve_reads_survive_large_time_jumps_removal_handle_reuse_and_loaded_reset() {
    let (mut b, ids) = setup();
    let initial = b.initial_state();
    let t = Duration::new(10_000_000_000, 50_000_000);
    b.step(t).unwrap();
    let same = published(&b, ids);
    assert_eq!(same[0].position, [0.5, 0., 0.]);
    assert!(b.remove(ids[0]));
    assert!(b.state(ids[0]).is_none());
    let late = b
        .spawn_kinematic_link(spec(-4., 0.), Default::default(), None)
        .unwrap();
    b.step(Duration::from_millis(50)).unwrap();
    assert_eq!(b.state(late).unwrap().position, [-4., 0., 0.]);
    assert_eq!(b.state(ids[1]).unwrap().position, [6., 0., 0.]);
    assert!(b.set_initial_translation_path(late, track()).is_err());
    let prepared = b.prepare_initial(&initial).unwrap();
    b.apply_initial(prepared).unwrap();
    assert!(b.state(late).is_none());
    b.step(t).unwrap();
    assert_eq!(published(&b, ids), same);
    assert_eq!(
        b.cast_ray([5.5, 0., 1.], [0., 0., -1.], 2., None).unwrap(),
        Some(0.9)
    );
}

#[cfg(feature = "partition-checkpoint")]
#[test]
fn native_warm_trajectory_preserves_pending_work_edits_and_future_contact_graph() {
    fn make(phase: bool) -> (MultiFidelityBackend, BodyId, BodyId) {
        let mut b = MultiFidelityBackend::new([0., 0., -9.81]).unwrap();
        let deck = b
            .spawn_kinematic_link(spec(0., 0.), Default::default(), None)
            .unwrap();
        let path = TranslationSpline::new(
            &[
                Waypoint {
                    time: Duration::ZERO,
                    displacement: [0.; 3],
                    velocity: [0.; 3],
                },
                Waypoint {
                    time: Duration::from_nanos(310250000),
                    displacement: [0.02, 0., if phase { 0.01 } else { 0.012 }],
                    velocity: [0.; 3],
                },
                Waypoint {
                    time: Duration::from_millis(700),
                    displacement: [0.; 3],
                    velocity: [0.; 3],
                },
            ],
            PathEnd::Repeat,
        )
        .unwrap();
        b.set_initial_translation_path(deck, path).unwrap();
        let cargo = b
            .spawn_link(spec(0., 0.205), Default::default(), None)
            .unwrap();
        (b, deck, cargo)
    }
    let (mut b, d, c) = make(false);
    let dt = Duration::from_millis(5);
    let cold = b.trajectory_construction().unwrap();
    let mut cuts = Vec::new();
    for i in 0..200 {
        if i == 70 {
            b.set_world_velocity(d, [0.01, 0., 0.], [0.; 3]).unwrap();
        }
        if [5, 32, 87].contains(&i) {
            b.add_world_wrench(c, [1.2, 0.1, 0.3], [0.02, 0., -0.01])
                .unwrap();
        }
        assert_eq!(b.trajectory_construction().unwrap(), cold);
        cuts.push(b.checkpoint_trajectory().unwrap());
        b.step(dt).unwrap();
    }
    for start in [0, 5, 6, 31, 32, 63, 70, 87, 119] {
        let bytes = &cuts[start];
        let mut restored = MultiFidelityBackend::from_trajectory_construction(&cold).unwrap();
        assert_eq!(restored.trajectory_construction().unwrap(), cold);
        let (deck, cargo) = (d, c);
        let (mut wrong, ..) = make(true);
        let initial = wrong.checkpoint_trajectory().unwrap();
        assert!(wrong.restore_trajectory(bytes).is_err());
        assert_eq!(wrong.checkpoint_trajectory().unwrap(), initial);
        for off in [7, 24, 67, bytes.len() - 1] {
            let mut bad = bytes.clone();
            bad[off] ^= 32;
            assert!(restored.restore_trajectory(&bad).is_err());
        }
        restored.restore_trajectory(bytes).unwrap();
        if restored.translation_interval().0 > 0 {
            assert_eq!(restored.normal_reaction_on(cargo, deck).unwrap(), None);
        }
        for (i, cut) in cuts.iter().enumerate().skip(start) {
            if i != start {
                if [5, 32, 87].contains(&i) {
                    restored
                        .add_world_wrench(cargo, [1.2, 0.1, 0.3], [0.02, 0., -0.01])
                        .unwrap();
                }
                if i == 70 {
                    restored
                        .set_world_velocity(deck, [0.01, 0., 0.], [0.; 3])
                        .unwrap();
                }
            }
            assert!(
                restored.checkpoint_trajectory().unwrap() == *cut,
                "diverged from {start} at {i}"
            );
            restored.step(dt).unwrap();
            assert!(restored.normal_reaction_on(cargo, deck).unwrap().is_some());
        }
    }
}

#[test]
fn grouped_publication_preserves_sparse_raw_ids_distinct_tangents_and_repeat_seams() {
    let mut engine = MultiFidelityBackend::new([0.; 3]).unwrap();
    let mut ids = Vec::new();
    let mut expected = Vec::new();
    for i in 0..470 {
        let id = engine
            .spawn_kinematic_link(spec(i as f64 * 2., 0.), Default::default(), None)
            .unwrap();
        let curve = (i % 5 != 0).then(|| {
            let d = (i % 7) as f64 / 32.;
            let v = (i % 3) as f64 / 15.;
            TranslationSpline::new(
                &[
                    Waypoint {
                        time: Duration::ZERO,
                        displacement: [0.; 3],
                        velocity: [0.; 3],
                    },
                    Waypoint {
                        time: Duration::from_nanos(71340001),
                        displacement: [d, 0., 0.],
                        velocity: [0., v, if i % 2 == 0 { -0.0 } else { 0.0 }],
                    },
                    Waypoint {
                        time: Duration::from_millis(150),
                        displacement: [0.; 3],
                        velocity: [0.; 3],
                    },
                ],
                PathEnd::Repeat,
            )
            .unwrap()
        });
        if let Some(path) = &curve {
            engine
                .set_initial_translation_path(id, path.clone())
                .unwrap();
        }
        expected.push(curve);
        ids.push(id);
    }
    let mut elapsed = Duration::ZERO;
    for step in [
        0,
        173_000,
        71_340_001,
        149_999_999,
        1,
        75_431_567,
        8_000_000,
    ] {
        let dt = Duration::from_nanos(step);
        if !dt.is_zero() {
            engine.step(dt).unwrap();
            elapsed += dt;
        }
        let mut order: Vec<_> = (0..ids.len()).collect();
        order.extend((0..ids.len()).rev());
        order.extend((0..2000).map(|i| i * 271 % ids.len()));
        engine.read_states(&mut |lookup| {
            for &i in &order {
                let p = lookup.state(ids[i]).unwrap();
                let analytic = expected[i].as_ref().map(|c| c.sample(elapsed));
                let shift = analytic.map_or([0.; 3], |s| s.displacement);
                let vel = analytic.map_or([0.; 3], |s| s.velocity);
                assert_eq!(p.position, [2. * i as f64 + shift[0], shift[1], shift[2]]);
                assert_eq!(p.velocity.map(f64::to_bits), vel.map(f64::to_bits));
                assert_eq!(p, engine.state(ids[i]).unwrap());
            }
        });
        if step == 149_999_999 {
            // Deactivate precisely one member of a shared path at the seam of the read traversal;
            // the equal siblings must keep their group. Its new pose remains an ordinary native row.
            let last = ids.len() - 1;
            let state = BodyState {
                position: [2. * last as f64, 0., 0.],
                velocity: [0.; 3],
                ..engine.state(ids[last]).unwrap()
            };
            engine.set_body_state(ids[last], state).unwrap();
            engine
                .set_world_velocity(ids[last], [0.; 3], [0.; 3])
                .unwrap();
            expected[last] = None;
        }
    }
}
