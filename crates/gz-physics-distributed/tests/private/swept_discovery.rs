// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::collections::BTreeSet;
#[test]
fn candidate_discovery_matches_all_pairs_through_deflection_closure_and_final_overlap() {
    let mut rng = 0x62726f6164706861_u64;
    let mut random = || {
        rng = rng
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((rng >> 11) as f64) / (1_u64 << 53) as f64
    };
    let mut sparse = 0;
    let mut dense = 0;
    let mut new_deflections = 0;
    for case in 0..320 {
        let n = 33 + case % 88;
        let scale = [0.3, 3., 11., 43.][case % 4];
        let step = [
            SimulationStep::Ms1,
            SimulationStep::Ms2,
            SimulationStep::Ms5,
            SimulationStep::Ms10,
        ][(case / 4) % 4];
        let mut seed = Snapshot {
            step,
            ground: (case % 5 == 0).then(|| GroundPlane::new(0.).unwrap()),
            ..Default::default()
        };
        let mut components = Vec::new();
        let mut forced = Vec::new();
        let mut inputs = Vec::new();
        for i in 0..n {
            let id = i as u64;
            let v = [1., 6., 16., 1200.][i % 4];
            seed.bodies.push(Body {
                id,
                position: std::array::from_fn(|_| (random() - 0.5) * scale),
                size: std::array::from_fn(|_| 0.2 + random()),
                linear: std::array::from_fn(|_| (random() - 0.5) * v),
                tier: if i % 13 == 0 {
                    Tier::Prescribed
                } else {
                    Tier::Dynamic
                },
                ..fixture()[0].promoted()
            });
            components.push(ComponentInfo {
                ids: vec![id],
                contact_history: i % 16 == 0,
            });
            if i % 9 == 0 {
                forced.push(ForceMotion {
                    id,
                    has_torque: i % 4 == 0,
                    delta_velocity: std::array::from_fn(|_| (random() - 0.5) * 35.),
                });
            }
            if i % 11 == 0 {
                inputs.push(PendingImpulse {
                    id,
                    impulse: [random(), -0., -3.],
                });
            }
        }
        if case % 3 == 1 {
            seed.bodies.reverse();
        }
        let (fast, pairs) = motions(&seed, &components, &inputs, &forced).unwrap();
        let (oracle, all) = motions_with::<false>(&seed, &components, &inputs, &forced, |m| {
            Candidates::All(m.len())
        })
        .unwrap();
        match pairs {
            Candidates::Sparse(_) => sparse += 1,
            Candidates::All(_) => dense += 1,
        }
        assert_eq!(
            fast.iter().map(|m| m.deflects).collect::<Vec<_>>(),
            oracle.iter().map(|m| m.deflects).collect::<Vec<_>>()
        );
        new_deflections += fast
            .iter()
            .filter(|m| m.deflects && !components[m.component].contact_history)
            .count();
        let mut a = BTreeSet::new();
        let mut b = BTreeSet::new();
        pairs.visit(|i, j| {
            if overlaps(&fast[i], &fast[j]) {
                a.insert((i, j));
            }
        });
        all.visit(|i, j| {
            if overlaps(&oracle[i], &oracle[j]) {
                b.insert((i, j));
            }
        });
        assert_eq!(a, b, "exact pair graph at seed {case}");
    }
    assert!(sparse > 0 && dense > 0 && new_deflections > 100);
}

#[test]
fn parallel_capsule_lanes_separate_only_while_orientation_is_certified() {
    let mut b = Body {
        primitive: Primitive::Capsule,
        size: [0.2, 0.2, 4.],
        position: [0.; 3],
        orientation: [0., 0., 0., 1.],
        angular: [0.; 3],
        linear: [0.; 3],
        ..fixture()[0].promoted()
    };
    b.id = 1;
    let sphere = Body {
        id: 2,
        primitive: Primitive::Sphere,
        size: [0.2; 3],
        position: [1.25, 0., 0.],
        ..b.clone()
    };
    let components = [
        ComponentInfo {
            ids: vec![1],
            contact_history: false,
        },
        ComponentInfo {
            ids: vec![2],
            contact_history: false,
        },
    ];
    let mut s = Snapshot {
        step: SimulationStep::Ms5,
        bodies: vec![b, sphere],
        ..Default::default()
    };
    let connected = |s: &Snapshot, force: &[BodyWrench]| {
        let inputs = validate_wrenches(s.step, &s.bodies, force).unwrap();
        forced_contact_groups(s, &components, &[], &inputs)
            .unwrap()
            .len()
            == 1
    };
    assert!(!connected(&s, &[]));
    let torque = BodyWrench {
        id: 1,
        force: [0.; 3],
        torque: [0., 10000., 0.],
    };
    assert!(connected(&s, std::slice::from_ref(&torque)));
    assert!(!connected(
        &s,
        &[
            torque.clone(),
            BodyWrench {
                torque: [0., -10000., 0.],
                ..torque
            }
        ]
    ));
    s.bodies[0].angular = [0., 0.3, 0.];
    assert!(connected(&s, &[]));
    s.bodies[0].tier = Tier::Prescribed;
    assert!(!connected(&s, &[]));
    s.bodies[0].angular = [0., std::f64::consts::PI / 0.005, 0.];
    assert!(connected(&s, &[]));
    s.bodies[0].angular = [0.; 3];
    s.bodies[0].tier = Tier::Dynamic;
    // The capsule's long arm already grazes a separate support. Any contact
    // can redirect its center and rotation before it approaches its neighbor.
    s.ground = Some(GroundPlane::new(-2.1).unwrap());
    assert!(connected(&s, &[]));
}

#[test]
fn prescribed_axial_sweeps_enclose_sampled_continuous_support() {
    let mut s = 0x6b51792d_u64;
    let mut rnd = || {
        s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
        (s >> 11) as f64 / (1u64 << 53) as f64
    };
    let unit = |mut v: [f64; 3]| {
        let n = v[0].hypot(v[1]).hypot(v[2]);
        for x in &mut v {
            *x /= n;
        }
        v
    };
    let cross = |a: [f64; 3], b: [f64; 3]| {
        std::array::from_fn::<_, 3, _>(|k| {
            a[(k + 1) % 3] * b[(k + 2) % 3] - a[(k + 2) % 3] * b[(k + 1) % 3]
        })
    };
    let dot = |a: [f64; 3], b: [f64; 3]| a.into_iter().zip(b).map(|(x, y)| x * y).sum::<f64>();
    let mut tight = 0;
    for _ in 0..4096 {
        let tilt = unit(std::array::from_fn(|_| rnd() - 0.5));
        let theta = (rnd() - 0.5) * 6.;
        let spin = unit(std::array::from_fn(|_| rnd() - 0.5));
        let speed = 1400. * rnd().powi(3);
        let n = unit(std::array::from_fn(|_| rnd() - 0.5));
        let dt = [0.001, 0.002, 0.005, 0.01][(4. * rnd()) as usize];
        let (si, co) = (0.5 * theta).sin_cos();
        let b = Body {
            primitive: Primitive::Capsule,
            size: [0.10, 0.10, 6.],
            tier: Tier::Prescribed,
            orientation: [si * tilt[0], si * tilt[1], si * tilt[2], co],
            angular: spin.map(|x| x * speed),
            ..fixture()[0].clone()
        };
        let rotate = |axis: [f64; 3], v: [f64; 3], a: f64| {
            let (sin, cos) = a.sin_cos();
            let t = cross(axis, v);
            let d = dot(axis, v);
            std::array::from_fn(|k| v[k] * cos + t[k] * sin + axis[k] * d * (1. - cos))
        };
        let start = rotate(tilt, [0., 0., 1.], theta);
        let bound = FreeHull::new(&b, 3.05, dt, false).radius(n);
        let (r, h) = if rnd() > 0.5 {
            (0.05_f64, 3.0_f64)
        } else {
            (2.0, 0.035)
        };
        let flat = Body {
            primitive: Primitive::Cylinder,
            size: [2. * r, 2. * r, 2. * h],
            ..b.clone()
        };
        let flat_bound = FreeHull::new(&flat, r.hypot(h) * (1. + 8e-12), dt, false).radius(n);
        for k in 0..=12 {
            let z = rotate(spin, start, speed * dt * (k as f64) / 12.);
            let exact = 0.05 + 3. * dot(z, n).abs();
            assert!(bound + 1e-14 >= exact, "bound {bound}, support {exact}");
            let perpendicular = cross(z, n);
            let flat_exact = h * dot(z, n).abs()
                + r * perpendicular[0]
                    .hypot(perpendicular[1])
                    .hypot(perpendicular[2]);
            assert!(
                flat_bound + 1e-14 >= flat_exact,
                "cylinder bound {flat_bound}, native support {flat_exact}"
            );
        }
        tight += usize::from(bound < 2.9);
    }
    assert!(tight > 800);
    eprintln!(
        "53248 capsule and 53248 cylinder continuous support samples within the directional bound, {tight} materially tighter directions"
    );
}

#[test]
fn tilted_flat_faces_admit_quiet_lanes_and_capture_crossings_rotation_and_contact_deflection() {
    let tilt = 0.56_f64;
    let (s, c) = (tilt / 2.).sin_cos();
    let normal = [tilt.sin(), 0., tilt.cos()];
    let first = Body {
        id: 1,
        primitive: Primitive::Cylinder,
        size: [1.04, 1.04, 0.055],
        position: [4., 2., 5.],
        orientation: [0., s, 0., c],
        angular: [0.; 3],
        linear: [0.; 3],
        ..fixture()[1].clone()
    };
    let second = Body {
        id: 2,
        position: std::array::from_fn(|i| first.position[i] + 0.15 * normal[i]),
        ..first.clone()
    };
    let mut world = Snapshot {
        step: SimulationStep::Ms5,
        bodies: vec![first, second],
        ..Default::default()
    };
    let mut membership = [
        ComponentInfo {
            ids: vec![1],
            contact_history: false,
        },
        ComponentInfo {
            ids: vec![2],
            contact_history: false,
        },
    ];
    let joined =
        |w: &Snapshot, g: &[ComponentInfo]| potential_contact_groups(w, g, &[]).unwrap().len() == 1;
    assert!(!joined(&world, &membership));
    // The thin solids exchange sides during one tick; their endpoint volumes miss.
    world.bodies[0].linear = normal.map(|v| v * 65.);
    assert!(joined(&world, &membership));
    world.bodies[0].linear = [0.; 3];
    world.bodies[0].tier = Tier::Prescribed;
    world.bodies[0].angular = [0., 120., 0.];
    assert!(joined(&world, &membership));
    world.bodies[0].angular = [0.; 3];
    world.bodies[0].tier = Tier::Dynamic;
    membership[0].contact_history = true;
    // A separated singleton carries history through cloning but has no finite
    // partner to redirect it. The same body near a plane can still deflect.
    assert!(!joined(&world, &membership));
    world.ground = Some(GroundPlane::new(world.bodies[0].position[2] - 0.1).unwrap());
    assert!(joined(&world, &membership));
}

#[test]
fn deflected_reach_excludes_cube_corners_but_includes_crossings_and_secondary_contacts() {
    let sphere = Body {
        primitive: Primitive::Sphere,
        size: [0.2; 3],
        position: [0.; 3],
        orientation: [0., 0., 0., 1.],
        angular: [0.; 3],
        linear: [0.; 3],
        tier: Tier::Dynamic,
        ..fixture()[0].clone()
    };
    let mut scene = Snapshot {
        step: SimulationStep::Ms10,
        bodies: [[0., 0., 0.], [-0.2, 0., 0.], [3.1, 3.1, 0.], [6.2, 6.2, 0.]]
            .into_iter()
            .enumerate()
            .map(|(i, position)| Body {
                id: i as u64,
                position,
                ..sphere.clone()
            })
            .collect(),
        ..Default::default()
    };
    let groups = [vec![0, 1], vec![2], vec![3]].map(|ids| ComponentInfo {
        ids,
        contact_history: true,
    });
    let sweep = |s: &Snapshot, forced: &[ForceMotion]| {
        forced_contact_groups(s, &groups, &[], forced).unwrap()
    };
    assert_eq!(sweep(&scene, &[]), [vec![0], vec![1], vec![2]]);
    // The third sphere sweeps into the loaded pair's four-metre reach, although
    // both of its endpoint positions are outside that ball.
    scene.bodies[2].position = [3.9, 1.8, 0.];
    scene.bodies[2].linear = [0., -360., 0.];
    assert_eq!(sweep(&scene, &[])[0], [0, 1]);
    // An inward held force on the third body brings its conservative trajectory
    // inside reach, enabling another deflection toward the fourth sphere.
    scene.bodies[2].position = [3.1, 3.1, 0.];
    scene.bodies[2].linear = [0.; 3];
    scene.bodies[3].position = [5.9, 5.9, 0.];
    let force = ForceMotion {
        id: 2,
        has_torque: false,
        delta_velocity: [-25., -25., 0.],
    };
    assert_eq!(sweep(&scene, &[force]), [vec![0, 1, 2]]);
    // A freely crossing pair must be colocated before their encounter even after
    // they each carry native history from a different solver.
    scene.bodies[2].position = [12., 12., 0.];
    scene.bodies[2].linear = [160., 0., 0.];
    scene.bodies[3].position = [13., 12., 0.];
    assert_eq!(sweep(&scene, &[]), [vec![0], vec![1, 2]]);
}

#[test]
fn smooth_plane_candidate_discovery_matches_all_pairs_for_rotating_grounded_spheres() {
    let mut rng = 1_u64;
    let mut choose = || {
        rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
        ((rng >> 11) as f64) / (1_u64 << 53) as f64
    };
    let mut sparse = 0;
    let mut eligible = 0;
    let mut promoted = 0;
    for sample in 0..220 {
        let mut world = Snapshot {
            ground: Some(GroundPlane::new(0.).unwrap()),
            ..Default::default()
        };
        let n = 33 + sample % 96;
        let span = [2., 18., 58., 97.][sample % 4];
        for id in 0..n {
            let r = 0.08 + choose() * 0.18;
            world.bodies.push(Body {
                id: id as u64,
                tier: Tier::Dynamic,
                primitive: Primitive::Sphere,
                friction: if id % 14 == 0 { 0.4 } else { 0. },
                size: [r * 2.; 3],
                position: [(choose() - 0.5) * span, (choose() - 0.5) * span, r + 0.001],
                linear: [(choose() - 0.5) * 24., (choose() - 0.5) * 24., -0.5],
                angular: [choose() * 250., 17., -32.],
                ..fixture()[0].clone()
            });
        }
        let c: Vec<_> = world
            .bodies
            .iter()
            .map(|b| ComponentInfo {
                ids: vec![b.id],
                contact_history: true,
            })
            .collect();
        let (fast, f) = motions_with::<true>(&world, &c, &[], &[], Candidates::new).unwrap();
        let (full, g) =
            motions_with::<true>(&world, &c, &[], &[], |m| Candidates::All(m.len())).unwrap();
        sparse += usize::from(matches!(f, Candidates::Sparse(_)));
        let state = |m: &Motion| (m.deflects, m.plane_only);
        assert!(fast.iter().map(state).eq(full.iter().map(state)));
        eligible += fast.iter().filter(|m| m.plane_only).count();
        promoted += fast
            .iter()
            .filter(|m| !m.plane_only && m.body.friction == 0.)
            .count();
        let graph = |motions: &[Motion], c: Candidates| {
            let mut edges = BTreeSet::new();
            c.visit(|a, b| {
                if overlaps_bound::<true>(&motions[a], &motions[b]) {
                    edges.insert((a, b));
                }
            });
            edges
        };
        assert_eq!(graph(&fast, f), graph(&full, g));
    }
    assert!(sparse > 0 && eligible > 0 && promoted > 0);
}

#[test]
fn smooth_plane_scope_keeps_speed_clamp_and_high_force_fallbacks() {
    let mut b = fixture()[0].promoted();
    b.primitive = Primitive::Sphere;
    b.id = 1;
    b.size = [0.4; 3];
    b.position = [0., 0., 0.205];
    b.friction = 0.;
    b.linear = [0.; 3];
    let group = [ComponentInfo {
        ids: vec![1],
        contact_history: true,
    }];
    let mut s = Snapshot {
        ground: Some(GroundPlane::new(0.).unwrap()),
        bodies: vec![b],
        ..Default::default()
    };
    let eligible = |s: &Snapshot, forced: &[ForceMotion]| {
        motions_with::<true>(s, &group, &[], forced, Candidates::new)
            .unwrap()
            .0[0]
            .plane_only
    };
    assert!(eligible(&s, &[]));
    s.bodies[0].linear[2] = -partition_linear_speed_limit();
    assert!(!eligible(&s, &[]));
    s.bodies[0].linear = [0.; 3];
    assert!(!eligible(
        &s,
        &[ForceMotion {
            id: 1,
            delta_velocity: [0., 0., -2. * partition_linear_speed_limit()],
            has_torque: false
        }]
    ));
}
