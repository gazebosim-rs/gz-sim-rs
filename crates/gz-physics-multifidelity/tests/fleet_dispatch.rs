// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::{BodyId, BodySpec, DynamicBackend, PhysicsError, RotationSpec, Shape};
use gz_physics_multifidelity::MultiFidelityBackend;
use std::time::Duration;

const DT: Duration = Duration::from_millis(10);
fn cuboid(x: f64, vx: f64, half: [f64; 3]) -> BodySpec {
    BodySpec {
        position: [x, 0., 0.],
        velocity: [vx, 0., 0.],
        shape: Shape::Box { half_extents: half },
        mass: Some(1.),
    }
}
struct Fleet {
    p: MultiFidelityBackend,
    decks: Vec<BodyId>,
    payload: BodyId,
}
impl Fleet {
    fn new(automatic: bool) -> Self {
        let mut p = MultiFidelityBackend::new([0.; 3]).unwrap();
        if !automatic {
            p.disable_fleet_isolation();
        }
        let decks = (0..256)
            .map(|i| {
                p.spawn_kinematic_link(
                    cuboid(i as f64 * 2., 0.25, [0.5; 3]),
                    Default::default(),
                    None,
                )
                .unwrap()
            })
            .collect();
        let payload = p
            .spawn_link(cuboid(-7., 0., [0.1; 3]), Default::default(), None)
            .unwrap();
        Self { p, decks, payload }
    }
}

#[test]
fn a_later_impulse_wakes_distant_decks_before_contact_and_preserves_impulses() {
    let mut auto = Fleet::new(true);
    let mut native = Fleet::new(false);
    for f in [&mut auto, &mut native] {
        f.p.step(DT).unwrap();
        f.p.apply_impulse(f.payload, [5., 0., 0.], [0., 0.02, 0.])
            .unwrap();
    }
    let start = auto.p.fleet_isolation_stats().unwrap();
    assert_eq!(start.analytic_decks, 256);
    let mut native_before_contact = false;
    let mut impulse = 0.0_f64;
    for _ in 0..170 {
        auto.p.step(DT).unwrap();
        native.p.step(DT).unwrap();
        let witness = auto.p.fleet_isolation_stats().unwrap();
        let a = auto.p.rigid_contact_observations();
        let b = native.p.rigid_contact_observations();
        assert_eq!(a.len(), b.len());
        for (left, right) in a.iter().zip(&b) {
            assert_eq!(
                (left.a, left.b, left.active),
                (right.a, right.b, right.active)
            );
            assert!((left.normal_impulse - right.normal_impulse).abs() < 1e-9);
            impulse = impulse.max(left.normal_impulse);
        }
        if a.is_empty() && witness.native_decks > 0 {
            native_before_contact = true;
        }
        let s = auto.p.state(auto.payload).unwrap();
        let c = native.p.state(native.payload).unwrap();
        for (x, y) in s
            .position
            .into_iter()
            .chain(s.velocity)
            .chain(s.orientation)
            .chain(s.angular_velocity)
            .zip(
                c.position
                    .into_iter()
                    .chain(c.velocity)
                    .chain(c.orientation)
                    .chain(c.angular_velocity),
            )
        {
            assert!((x - y).abs() < 1e-8);
        }
    }
    assert!(native_before_contact && impulse > 0.1);
    assert!(auto.p.fleet_isolation_stats().unwrap().analytic_decks > 250);
}

#[test]
fn cold_ray_edits_and_lifecycle_remain_consistent_across_native_rejoin() {
    let mut f = Fleet::new(true);
    let initial = f.p.initial_state();
    f.p.step(DT).unwrap();
    let last = f.decks[255];
    let s = f.p.state(last).unwrap();
    assert_eq!(s.position, [510.0025, 0., 0.]);
    let hit = |p: &MultiFidelityBackend, except| {
        p.cast_ray([509., 0., 0.], [1., 0., 0.], 1.5, except)
            .unwrap()
    };
    assert!((hit(&f.p, None).unwrap() - 0.5025).abs() < 1e-12);
    assert_eq!(hit(&f.p, Some(last)), None);
    let invalid = f.p.set_world_velocity(last, [f64::NAN, 0., 0.], [0.; 3]);
    assert_eq!(invalid, Err(PhysicsError::InvalidValue));
    assert!(f.p.fleet_isolation_stats().is_some());
    f.p.set_world_velocity(last, [-0.5, 0., 0.], [0.; 3])
        .unwrap();
    assert_eq!(f.p.fleet_isolation_stats().unwrap().analytic_decks, 255);
    assert_eq!(f.p.fleet_deck_is_native(last), Some(true));
    assert_eq!(f.p.state(last).unwrap().position, s.position);
    f.p.step(DT).unwrap();
    assert!((f.p.state(last).unwrap().position[0] - 509.9975).abs() < 1e-10);
    assert!(f.p.remove(last));
    assert!(f.p.state(last).is_none());
    let replacement =
        f.p.spawn_kinematic_link(cuboid(800., 0.25, [0.5; 3]), Default::default(), None)
            .unwrap();
    assert_ne!(replacement, last);
    let reset = f.p.prepare_initial(&initial).unwrap();
    f.p.apply_initial(reset).unwrap();
    assert!(f.p.state(replacement).is_none());
    f.p.step(DT).unwrap();
    assert_eq!(f.p.fleet_isolation_stats().unwrap().analytic_decks, 256);
    assert_eq!(f.p.state(last).unwrap(), s);
    f.p.add_world_wrench(f.payload, [2., 0., 0.], [0.; 3])
        .unwrap();
    f.p.step(Duration::ZERO).unwrap();
    assert_eq!(f.p.state(f.payload).unwrap().velocity, [0.; 3]);
    f.p.step(DT).unwrap();
    assert!((f.p.state(f.payload).unwrap().velocity[0] - 0.02).abs() < 1e-10);
}
#[test]
fn a_rotating_or_differently_translating_deck_keeps_the_whole_world_native() {
    for (v, w) in [(0.25, 1.), (-0.25, 0.)] {
        let mut f = Fleet::new(true);
        f.p.spawn_kinematic_link(
            cuboid(900., v, [0.5; 3]),
            RotationSpec {
                angular_velocity: [0., 0., w],
                ..Default::default()
            },
            None,
        )
        .unwrap();
        f.p.step(DT).unwrap();
        assert_eq!(f.p.fleet_isolation_stats(), None);
    }
}

#[test]
fn overlapping_prescribed_boxes_preserve_even_inactive_pair_observations() {
    let mut auto = Fleet::new(true);
    let mut native = Fleet::new(false);
    for f in [&mut auto, &mut native] {
        f.p.spawn_kinematic_link(cuboid(510., 0.25, [0.5; 3]), Default::default(), None)
            .unwrap();
        f.p.step(DT).unwrap();
    }
    let convert = |p: &MultiFidelityBackend| {
        p.rigid_contact_observations()
            .into_iter()
            .map(|r| (r.a, r.b, r.active, r.normal_impulse.to_bits()))
            .collect::<Vec<_>>()
    };
    assert_eq!(convert(&auto.p), convert(&native.p));
}

#[test]
fn a_larger_step_expands_the_guard_before_integrating_a_fast_rotating_payload() {
    let mut auto = Fleet::new(true);
    let mut native = Fleet::new(false);
    for f in [&mut auto, &mut native] {
        let half = std::f64::consts::FRAC_1_SQRT_2;
        f.p.spawn_link(
            cuboid(-9., 395., [3., 0.03, 0.03]),
            RotationSpec {
                orientation: [0., 0., half, half],
                angular_velocity: [0., 0., 4.],
            },
            None,
        )
        .unwrap();
        f.p.step(Duration::from_millis(1)).unwrap();
    }
    assert_eq!(auto.p.fleet_isolation_stats().unwrap().native_decks, 0);
    auto.p.step(Duration::from_millis(25)).unwrap();
    native.p.step(Duration::from_millis(25)).unwrap();
    assert!(auto.p.fleet_isolation_stats().unwrap().native_decks > 0);
    for _ in 0..5 {
        auto.p.step(DT).unwrap();
        native.p.step(DT).unwrap();
    }
    for k in 0..4 {
        let s = auto.p.state(auto.decks[k]).unwrap();
        let n = native.p.state(native.decks[k]).unwrap();
        assert!((s.position[0] - n.position[0]).abs() < 1e-10);
        assert_eq!(s.velocity, n.velocity);
    }
    assert_eq!(
        auto.p.active_contact_pairs(),
        native.p.active_contact_pairs()
    );
}

#[test]
fn topology_changes_materialize_surviving_decks_and_admit_new_contact() {
    let mut removed = Fleet::new(true);
    removed.p.step(DT).unwrap();
    let last = removed.decks[255];
    assert!(removed.p.remove(last));
    assert!(removed.p.fleet_isolation_stats().is_none());
    assert!(removed.p.state(last).is_none());
    assert_eq!(
        removed.p.state(removed.decks[254]).unwrap().position[0],
        508.0025
    );
    let mut added = Fleet::new(true);
    added.p.step(DT).unwrap();
    let deck = added.decks[255];
    let center = added.p.state(deck).unwrap().position;
    let mut spec = cuboid(center[0], 0., [0.1; 3]);
    spec.position[2] = 0.599;
    let fresh = added.p.spawn_link(spec, Default::default(), None).unwrap();
    assert!(added.p.fleet_isolation_stats().is_none());
    assert_eq!(added.p.state(deck).unwrap().position, center);
    added.p.step(DT).unwrap();
    assert!(
        added
            .p
            .rigid_contact_observations()
            .iter()
            .any(|r| r.a == deck && r.b == fresh && r.active)
    );
}

#[test]
fn scoped_observations_cover_analytic_native_reordered_and_unknown_ids() {
    let mut f = Fleet::new(true);
    f.p.step(DT).unwrap();
    let missing = BodyId::from_raw(u64::MAX);
    for dt in [Duration::from_nanos(7_000_003), Duration::from_secs(2)] {
        let ids = [
            f.decks[255],
            f.payload,
            missing,
            f.decks[0],
            f.decks[1],
            f.decks[255],
        ];
        let bits = |s: gz_physics::BodyState| {
            s.position
                .into_iter()
                .chain(s.velocity)
                .chain(s.orientation)
                .chain(s.angular_velocity)
                .map(f64::to_bits)
                .collect::<Vec<_>>()
        };
        let expected = ids.map(|id| f.p.state(id).map(bits));
        f.p.read_states(&mut |lookup| {
            assert_eq!(ids.map(|id| lookup.state(id).map(bits)), expected)
        });
        f.p.step(dt).unwrap();
    }
    // A 2 s guard reaches every deck in this compact fixture.
    assert!(f.p.fleet_isolation_stats().is_none());
    assert!(
        (f.p.state(f.decks[255]).unwrap().position[0] - (510. + 0.25 * 2.017000003)).abs() < 1e-11
    );
}

#[test]
fn a_global_floor_or_small_deck_population_stays_in_the_original_solver() {
    let mut floor = Fleet::new(true);
    let mut spec = cuboid(2000., 0., [0.5; 3]);
    spec.mass = None;
    floor.p.spawn_link(spec, Default::default(), None).unwrap();
    floor.p.step(DT).unwrap();
    assert!(floor.p.fleet_isolation_stats().is_none());
    let mut short = Fleet::new(true);
    assert!(short.p.remove(short.decks[0]));
    short
        .p
        .spawn_link(cuboid(2000., 0., [0.5; 3]), Default::default(), None)
        .unwrap();
    short.p.step(DT).unwrap();
    assert!(short.p.fleet_isolation_stats().is_none());
}

#[test]
fn native_speed_ceiling_bounds_the_first_tick_after_an_overspeed_command() {
    let mut auto = Fleet::new(true);
    let mut native = Fleet::new(false);
    for f in [&mut auto, &mut native] {
        f.p.set_world_velocity(f.payload, [1000., 0., 0.], [0.; 3])
            .unwrap();
        f.p.step(DT).unwrap();
        let state = f.p.state(f.payload).unwrap();
        assert!((state.position[0] + 3.).abs() < 1e-10, "{state:?}");
        assert_eq!(state.velocity, [400., 0., 0.]);
    }
    assert_eq!(auto.p.fleet_isolation_stats().unwrap().native_decks, 0);
    auto.p.step(DT).unwrap();
    native.p.step(DT).unwrap();
    assert!(auto.p.fleet_isolation_stats().unwrap().native_decks > 0);
    assert_eq!(
        auto.p.state(auto.payload).unwrap(),
        native.p.state(native.payload).unwrap()
    );
}

#[test]
fn detached_deck_crosses_shared_rows_and_hits_a_payload_after_several_surveys() {
    let mut auto = Fleet::new(true);
    let mut stock = Fleet::new(false);
    let mut hits = 0;
    let mut impulse = 0.;
    for tick in 1..=320 {
        for f in [&mut auto, &mut stock] {
            if tick == 4 || tick == 150 {
                let speed = if tick == 4 { -6. } else { 5. };
                f.p.set_world_velocity(f.decks[0], [speed, 0., 0.], [0.; 3])
                    .unwrap();
            }
            f.p.step(DT).unwrap();
        }
        let stats = auto.p.fleet_isolation_stats().unwrap();
        assert!(stats.analytic_decks >= 250);
        if tick >= 4 {
            assert_eq!(auto.p.fleet_deck_is_native(auto.decks[0]), Some(true));
        }
        let a = auto.p.rigid_contact_observations();
        let b = stock.p.rigid_contact_observations();
        assert_eq!(a.len(), b.len(), "tick {tick}");
        for (x, y) in a.iter().zip(&b) {
            assert_eq!((x.a, x.b, x.active), (y.a, y.b, y.active));
            assert!((x.normal_impulse - y.normal_impulse).abs() < 1e-9);
            if x.normal_impulse > 0. {
                hits += 1;
                impulse += x.normal_impulse;
            }
        }
        for id in [auto.payload, auto.decks[0], auto.decks[1], auto.decks[200]] {
            let a = auto.p.state(id).unwrap();
            let b = stock.p.state(id).unwrap();
            assert!(
                a.position
                    .into_iter()
                    .zip(b.position)
                    .all(|(x, y)| (x - y).abs() < 1e-9),
                "tick {tick} id {id:?}: {a:?} vs {b:?}"
            );
            assert!(
                a.velocity
                    .into_iter()
                    .zip(b.velocity)
                    .all(|(x, y)| (x - y).abs() < 1e-9),
                "tick {tick} id {id:?}: {a:?} vs {b:?}"
            );
        }
    }
    assert!(hits > 0 && impulse > 0.1);
}

#[test]
fn full_rejoin_preserves_the_public_edit_barrier_and_native_successor() {
    let mut auto = Fleet::new(true);
    let mut ordinary = Fleet::new(false);
    for _ in 0..73 {
        auto.p.step(DT).unwrap();
        ordinary.p.step(DT).unwrap();
    }
    let unchanged = auto.decks[253];
    let edited = auto.decks[254];
    let public = auto.p.state(unchanged).unwrap();
    let native = ordinary.p.state(ordinary.decks[253]).unwrap();
    assert_ne!(
        public.position, native.position,
        "fixture must accumulate native roundoff"
    );
    auto.p.disable_fleet_isolation();
    auto.p.step(Duration::ZERO).unwrap();
    assert_eq!(auto.p.state(unchanged).unwrap(), public);
    let mut requested = auto.p.state(edited).unwrap();
    requested.position = [700., 3., 0.];
    requested.velocity = [0.; 3];
    auto.p.set_body_state(edited, requested).unwrap();
    auto.p.step(DT).unwrap();
    ordinary.p.step(DT).unwrap();
    assert_eq!(
        auto.p.state(unchanged),
        ordinary.p.state(ordinary.decks[253])
    );
    assert_eq!(auto.p.state(edited).unwrap(), requested);
}
