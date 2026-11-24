// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
const DT: Duration = Duration::from_millis(1);
fn scene(z: f64, y: f64, speed: f64, gravity: f64) -> (RapierBackend, [BodyId; 2]) {
    ordered_scene(z, y, speed, gravity, false)
}
fn ordered_scene(
    z: f64,
    y: f64,
    speed: f64,
    gravity: f64,
    fixed_first: bool,
) -> (RapierBackend, [BodyId; 2]) {
    let mut engine = RapierBackend::new_multifidelity([0., 0., gravity]).unwrap();
    let rot = Rotation::from_array([-0.0007481, -0.00013354, 0.71759124, 0.69646409]).normalize();
    let build = |half_extents, position, velocity, mass| BodySpec {
        shape: Shape::Box { half_extents },
        position,
        velocity,
        mass,
    };
    let fixed = |engine: &mut RapierBackend| {
        engine
            .spawn_link(
                build([0.13, 0.012, 0.0535], [0., 0.23, 0.5535], [0.; 3], None),
                Default::default(),
                None,
            )
            .unwrap()
    };
    let first = fixed_first.then(|| fixed(&mut engine));
    let parcel = engine
        .spawn_link(
            build(
                [0.03, 0.0275, 0.035],
                [0., y, z],
                [0., speed, 0.],
                Some(1.24),
            ),
            RotationSpec {
                orientation: rot.to_array(),
                ..Default::default()
            },
            None,
        )
        .unwrap();
    let bar = first.unwrap_or_else(|| fixed(&mut engine));
    (engine, [parcel, bar])
}

#[derive(Default, Debug, PartialEq)]
struct Trace {
    checks: usize,
    first_force: Option<usize>,
    normal_ns: f64,
    sustained: usize,
    last: [f64; 3],
    history: Vec<(BodyState, NormalReaction, ContactWarmstart)>,
}
fn coast(mut engine: RapierBackend, [parcel, bar]: [BodyId; 2], check: bool) -> Trace {
    let mut out = Trace::default();
    for tick in 1..=550 {
        if check && engine.recheck_separated_boxes(parcel, bar, DT).unwrap() {
            out.checks += 1;
        }
        engine.step(DT).unwrap();
        let reaction = engine.normal_reaction_on(parcel, bar).unwrap().unwrap();
        let j = reaction.magnitude;
        out.history.push((
            engine.state(parcel).unwrap(),
            reaction,
            engine.native_contact_warmstart(parcel, bar).unwrap(),
        ));
        if j > 0. {
            out.first_force.get_or_insert(tick);
            if tick > 400 {
                out.sustained += 1;
            }
        }
        out.normal_ns += j;
    }
    out.last = engine.state(parcel).unwrap().position;
    out
}

#[test]
fn an_unloaded_frozen_edge_cannot_impulse_a_slab_that_passes_clear() {
    let run = |check| {
        let (e, ids) = scene(0.6528474, 0.133, 0.2208225, 0.);
        coast(e, ids, check)
    };
    let old = run(false);
    let fresh = run(true);
    assert!(old.normal_ns > 0. && old.first_force.is_some(), "{old:?}");
    assert!(
        fresh.checks > 0 && fresh.normal_ns == 0. && fresh.first_force.is_none(),
        "{fresh:?}"
    );
    assert!((fresh.last[1] - (0.133 + 0.2208225 * 0.55)).abs() < 1e-12);
}

#[test]
fn fixed_first_box_order_can_also_pass_clear() {
    let (e, ids) = ordered_scene(0.6528474, 0.133, 0.2208225, 0., true);
    let trace = coast(e, ids, true);
    assert_eq!(trace.normal_ns, 0.);
    assert!((trace.last[1] - (0.133 + 0.2208225 * 0.55)).abs() < 1e-12);
}

#[test]
fn genuinely_intersecting_separating_and_loaded_boxes_preserve_their_history() {
    for (z, y, vy, g) in [
        (0.60, 0.15, 0.22, 0.),
        (0.6528474, 0.176, -0.22, 0.),
        (0.642, 0.23, 0., -9.81),
    ] {
        let run = |check| {
            let (e, ids) = scene(z, y, vy, g);
            coast(e, ids, check)
        };
        let raw = run(false);
        let observer = run(true);
        assert_eq!(raw, observer);
        if g < 0. {
            assert_eq!(raw.sustained, 150);
        }
        if vy > 0. {
            assert!(raw.normal_ns > 0.);
        }
        if vy < 0. {
            assert_eq!(raw.normal_ns, 0.);
        }
    }
}

#[test]
fn invalid_body_pairs_are_rejected_before_mutating_contact_rows() {
    let (mut e, [m, f]) = scene(0.642, 0.23, 0., -9.81);
    assert!(e.recheck_separated_boxes(m, f, Duration::ZERO).is_err());
    assert!(e.recheck_separated_boxes(f, m, DT).is_err());
    let unknown = BodyId::from_raw(100);
    assert!(e.recheck_separated_boxes(unknown, f, DT).is_err());
    assert!(e.recheck_separated_boxes(m, unknown, DT).is_err());
    let ball = e
        .spawn(BodySpec {
            shape: Shape::Sphere { radius: 0.01 },
            position: [0.; 3],
            velocity: [0.; 3],
            mass: Some(1.),
        })
        .unwrap();
    assert!(e.recheck_separated_boxes(ball, f, DT).is_err());
    let h = e.bodies[&m].0;
    e.world.colliders.insert_with_parent(
        ColliderBuilder::cuboid(0.01, 0.01, 0.01),
        h,
        &mut e.world.bodies,
    );
    assert!(e.recheck_separated_boxes(m, f, DT).is_err());
}
