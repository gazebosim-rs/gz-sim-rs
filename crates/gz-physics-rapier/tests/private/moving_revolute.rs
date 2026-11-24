// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn hinge() -> (RapierBackend, [BodyId; 2], JointId) {
    let mut w = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let links = [
        (0., 0.16, 2., [0.07, -0.03, 0.], 0.12),
        (0.65, 0.12, 0.8, [-0.04, 0.02, 0.], 0.036),
    ]
    .map(|(x, radius, mass, center, i)| {
        let id = w
            .spawn_articulation_link(
                BodySpec {
                    position: [x, 0., 0.5],
                    velocity: [0.; 3],
                    mass: Some(mass),
                    shape: Shape::Sphere { radius },
                },
                RotationSpec::default(),
                Some(Inertia {
                    center,
                    orientation: [0., 0., 0., 1.],
                    tensor: [i, i, i, 0., 0., 0.],
                }),
            )
            .unwrap();
        w.configure_moving_body(id).unwrap();
        id
    });
    let j = w
        .add_reduced_revolute(Some(links[0]), links[1], [0.33, 0., 0.5], [0., 0., 1.])
        .unwrap();
    (w, links, j)
}

fn add_sphere(w: &mut RapierBackend, mass: f64) -> BodyId {
    let id = w
        .spawn_link(
            BodySpec {
                position: [1.25, 0.139, 0.5],
                velocity: [-1.8, 0., 0.],
                mass: Some(mass),
                shape: Shape::Sphere { radius: 0.15 },
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.; 3],
                orientation: [0., 0., 0., 1.],
                tensor: [0.009, 0.009, 0.009, 0., 0., 0.],
            }),
        )
        .unwrap();
    w.configure_moving_body(id).unwrap();
    id
}

fn coupled() -> (RapierBackend, [BodyId; 2], JointId, BodyId) {
    let (mut w, links, joint) = hinge();
    let sphere = add_sphere(&mut w, 1.);
    (w, links, joint, sphere)
}

#[test]
fn loaded_moving_graph_and_pending_effort_continue_exactly() {
    let (mut reference, links, joint, sphere) = coupled();
    let (mut trial, ..) = coupled();
    let mut loaded_cuts = 0;
    let mut peak_tangent = 0.0_f64;
    let mut peak = 0.0_f64;
    let mut max_residual = 0.0_f64;
    for tick in 1..=750 {
        let before = Vector::from(reference.state(sphere).unwrap().velocity);
        reference.step(STEP).unwrap();
        trial.step(STEP).unwrap();
        let after = Vector::from(reference.state(sphere).unwrap().velocity);
        let mut total = Vector::ZERO;
        let mut loaded = false;
        for link in links {
            assert_eq!(reference.state(link), trial.state(link), "tick {tick}");
            let r = reference.moving_reaction(link, sphere).unwrap();
            assert_eq!(r, trial.moving_reaction(link, sphere).unwrap());
            total += Vector::from(r.normal) * r.normal_impulse + Vector::from(r.tangent_impulse);
            if r.normal_impulse > 0. && r.warmstart_normal_impulse > 0. {
                loaded = true;
                assert!(reference.detached_hinge_image(sphere).is_err());
            }
            peak = peak.max(r.normal_impulse);
            peak_tangent = peak_tangent.max(Vector::from(r.tangent_impulse).length());
        }
        max_residual = max_residual.max((after - before + total).length());
        assert_eq!(reference.state(sphere), trial.state(sphere));
        assert_eq!(reference.joint_state(joint), trial.joint_state(joint));
        if tick % 71 == 0 || (loaded && loaded_cuts < 7) {
            if loaded {
                for w in [&mut reference, &mut trial] {
                    w.add_joint_effort(joint, 0.15).unwrap();
                }
            }
            let image = trial.checkpoint_moving_revolute().unwrap();
            assert_eq!(
                reference.checkpoint_moving_revolute().unwrap(),
                image,
                "at {tick}"
            );
            let (mut dest, ..) = coupled();
            dest.restore_moving_revolute(&image).unwrap();
            assert_eq!(dest.checkpoint_moving_revolute().unwrap(), image);
            trial = dest;
            loaded_cuts += usize::from(loaded);
        }
    }
    println!(
        "moving impact, peak {peak} N·s, peak tangent {peak_tangent} N·s, {loaded_cuts} warm cuts, sphere impulse residual {max_residual} N·s"
    );
    assert!(loaded_cuts > 0 && peak > 0.1 && peak_tangent > 0.01 && max_residual < 1e-10);
}

#[test]
fn detached_workers_join_then_release_with_the_permanently_coupled_trajectory() {
    let (mut reference, links, j, sphere) = coupled();
    let (mut detached, ..) = hinge();
    let mut solo = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let single = add_sphere(&mut solo, 1.);
    let mut together = false;
    let mut loaded = false;
    let mut split = None;
    for tick in 1..=900 {
        let effort = if tick < 450 { 0.06 } else { -0.1 };
        detached.add_joint_effort(j, effort).unwrap();
        reference.add_joint_effort(j, effort).unwrap();
        if tick == 121 {
            let motion = detached.checkpoint_floating_revolute().unwrap();
            let state = solo.state(single).unwrap();
            detached = hinge().0;
            detached.restore_floating_revolute(&motion).unwrap();
            assert_eq!(add_sphere(&mut detached, 1.), sphere);
            detached.place_detached_sphere(sphere, state).unwrap();
            // Allow the new owner to die before the first coupled solve.
            let joined_cut = detached.checkpoint_moving_revolute().unwrap();
            detached = coupled().0;
            detached.restore_moving_revolute(&joined_cut).unwrap();
            together = true;
        }
        if together
            && loaded
            && tick > 400
            && let Ok(motion) = detached.detached_hinge_image(sphere)
        {
            let state = detached.state(sphere).unwrap();
            detached = hinge().0;
            detached.restore_floating_revolute(&motion).unwrap();
            solo = RapierBackend::new_multifidelity([0.; 3]).unwrap();
            assert_eq!(add_sphere(&mut solo, 1.), single);
            solo.place_detached_sphere(single, state).unwrap();
            together = false;
            split = Some(tick);
        }
        reference.step(STEP).unwrap();
        detached.step(STEP).unwrap();
        let state = if together {
            for link in links {
                let r = reference.moving_reaction(link, sphere).unwrap();
                let observed = detached.moving_reaction(link, sphere).unwrap();
                assert_eq!(r, observed, "tick {tick}");
                loaded |= r.normal_impulse > 0.;
            }
            detached.state(sphere)
        } else {
            solo.step(STEP).unwrap();
            for link in links {
                assert_eq!(
                    reference.moving_reaction(link, sphere).unwrap(),
                    MovingReaction::default(),
                    "detached reaction at tick {tick}"
                );
            }
            solo.state(single)
        };
        assert_eq!(reference.state(sphere), state, "sphere at tick {tick}");
        assert_eq!(
            reference.joint_state(j),
            detached.joint_state(j),
            "joint at tick {tick}"
        );
        for link in links {
            assert_eq!(
                reference.state(link),
                detached.state(link),
                "link {link:?} at tick {tick}"
            );
        }
    }
    assert!(loaded && split.is_some());
    println!(
        "separate, merged on tick 121, detached again on tick {}, exact through tick 900",
        split.unwrap()
    );
}

#[test]
fn moving_image_rejects_wrong_recipe_and_digest_without_mutation() {
    let (mut src, ..) = coupled();
    src.step(STEP).unwrap();
    let mut bytes = src.checkpoint_moving_revolute().unwrap();
    let (mut wrong, links, _) = hinge();
    let sphere = add_sphere(&mut wrong, 2.);
    let before = [links[0], links[1], sphere].map(|id| wrong.state(id));
    assert!(wrong.restore_moving_revolute(&bytes).is_err());
    assert_eq!(
        before,
        [links[0], links[1], sphere].map(|id| wrong.state(id))
    );
    bytes[40] ^= 1;
    let (mut dest, ..) = coupled();
    assert!(dest.restore_moving_revolute(&bytes).is_err());
    assert!(dest.joints.open);
}

#[test]
fn release_requires_geometric_clearance_even_before_first_pair_update() {
    let (mut w, links, _, sphere) = coupled();
    w.step(STEP).unwrap();
    assert!(w.detached_hinge_image(sphere).is_ok());
    let mut ball = w.state(sphere).unwrap();
    ball.position = w.state(links[1]).unwrap().position;
    w.place_detached_sphere(sphere, ball).unwrap();
    assert!(!w.moving_reaction(links[1], sphere).unwrap().active);
    assert!(w.detached_hinge_image(sphere).is_err());
}

#[test]
fn routing_reads_pending_torque_from_compact_and_coupled_cuts() {
    for joined in [false, true] {
        let (mut world, _, joint) = hinge();
        if joined {
            add_sphere(&mut world, 1.);
        }
        let encode = |world: &RapierBackend| {
            if joined {
                world.checkpoint_moving_revolute().unwrap()
            } else {
                world.checkpoint_floating_revolute().unwrap()
            }
        };
        world.step(STEP).unwrap();
        assert_eq!(
            RapierBackend::moving_checkpoint_effort(&encode(&world)).unwrap(),
            None
        );
        for (torque, expected) in [(0.125, 0.125), (-0.375, -0.25)] {
            world.add_joint_effort(joint, torque).unwrap();
            let mut bytes = encode(&world);
            assert_eq!(
                RapierBackend::moving_checkpoint_effort(&bytes).unwrap(),
                Some(expected)
            );
            bytes[40] ^= 1;
            assert!(RapierBackend::moving_checkpoint_effort(&bytes).is_err());
        }
        world.step(STEP).unwrap();
        assert_eq!(
            RapierBackend::moving_checkpoint_effort(&encode(&world)).unwrap(),
            None
        );
    }
}

fn companion_at(w: &mut RapierBackend, p: [f64; 3], v: [f64; 3]) -> BodyId {
    let id = w
        .spawn_link(
            BodySpec {
                position: p,
                velocity: v,
                mass: Some(1.0),
                shape: Shape::Sphere { radius: 0.15 },
            },
            RotationSpec::default(),
            Some(Inertia {
                center: [0.; 3],
                orientation: [0., 0., 0., 1.],
                tensor: [0.009, 0.009, 0.009, 0., 0., 0.],
            }),
        )
        .unwrap();
    w.configure_moving_body(id).unwrap();
    id
}

fn four() -> (RapierBackend, [BodyId; 2], JointId, [BodyId; 2]) {
    let (mut w, links, joint, first) = coupled();
    let second = companion_at(&mut w, [2., 1.5, 0.5], [0.125, -0.25, 0.]);
    w.configure_pair_revolute().unwrap();
    (w, links, joint, [first, second])
}

#[test]
fn pair_graph_restores_cold_activation_and_loaded_partial_changes() {
    let (mut reference, links, joint, [a, b]) = four();
    let (mut trial, ..) = four();
    trial.set_pair_sphere_active(b, false, None).unwrap();
    for w in [&mut reference, &mut trial] {
        w.add_joint_effort(joint, -0.125).unwrap();
    }
    let roundtrip = |w: &mut RapierBackend| {
        let image = w.checkpoint_pair_revolute().unwrap();
        let mut dest = four().0;
        dest.restore_pair_revolute(&image).unwrap();
        assert_eq!(dest.checkpoint_pair_revolute().unwrap(), image);
        *w = dest;
    };
    roundtrip(&mut trial);
    assert!(!trial.pair_sphere_active(b).unwrap());
    let mut joined = false;
    let mut released = false;
    for tick in 1..700 {
        for w in [&mut reference, &mut trial] {
            w.step(STEP).unwrap();
        }
        for link in links {
            assert_eq!(
                trial.state(link),
                reference.state(link),
                "link {link:?} at {tick}"
            );
            assert_eq!(
                trial.pair_reaction(link, a).unwrap(),
                reference.pair_reaction(link, a).unwrap(),
                "reaction at {tick}"
            );
        }
        assert_eq!(trial.state(a), reference.state(a));
        assert_eq!(trial.joint_state(joint), reference.joint_state(joint));
        let loaded = links
            .iter()
            .any(|&link| trial.pair_reaction(link, a).unwrap().normal_impulse > 0.);
        if loaded && !joined {
            let before = trial.checkpoint_pair_revolute().unwrap();
            assert!(trial.set_pair_sphere_active(a, false, None).is_err());
            assert_eq!(trial.checkpoint_pair_revolute().unwrap(), before);
            trial
                .set_pair_sphere_active(b, true, Some(reference.state(b).unwrap()))
                .unwrap();
            for w in [&mut reference, &mut trial] {
                w.add_joint_effort(joint, 0.375).unwrap();
            }
            roundtrip(&mut trial);
            joined = true;
        } else if loaded && joined && !released {
            assert_eq!(trial.state(b), reference.state(b));
            trial.set_pair_sphere_active(b, false, None).unwrap();
            roundtrip(&mut trial);
            released = true;
        }
    }
    assert!(
        released,
        "expected to change the unloaded companion during two loaded solves"
    );
}

#[test]
fn pair_reaction_resolves_oblique_sphere_impulse_with_opposite_signs() {
    let (mut w, _, _) = hinge();
    let first = companion_at(&mut w, [2., 3., 0.5], [1., 0.2, 0.]);
    let second = companion_at(&mut w, [2.6, 3.08, 0.5], [-1., 0., 0.]);
    w.configure_pair_revolute().unwrap();
    let mut peak = 0.0_f64;
    let mut friction = 0.0_f64;
    for _ in 0..350 {
        let before = w.state(first).unwrap().velocity;
        let other_before = w.state(second).unwrap().velocity;
        w.step(STEP).unwrap();
        let after = w.state(first).unwrap().velocity;
        let a = w.pair_reaction(first, second).unwrap();
        let b = w.pair_reaction(second, first).unwrap();
        assert_eq!(a.normal_impulse, b.normal_impulse);
        let impulse = Vector::from(a.normal) * a.normal_impulse + Vector::from(a.tangent_impulse);
        assert!(
            (impulse + Vector::from(b.normal) * b.normal_impulse + Vector::from(b.tangent_impulse))
                .length()
                < 1e-12
        );
        assert!((Vector::from(after) - Vector::from(before) - impulse).length() < 1e-11);
        let other_after = Vector::from(w.state(second).unwrap().velocity);
        assert!((other_after - Vector::from(other_before) + impulse).length() < 1e-11);
        peak = peak.max(a.normal_impulse);
        friction = friction.max(Vector::from(a.tangent_impulse).length());
        if a.normal_impulse > 0. && peak == a.normal_impulse {
            let image = w.checkpoint_pair_revolute().unwrap();
            let (mut dest, _, _) = hinge();
            companion_at(&mut dest, [2., 3., 0.5], [1., 0.2, 0.]);
            companion_at(&mut dest, [2.6, 3.08, 0.5], [-1., 0., 0.]);
            dest.configure_pair_revolute().unwrap();
            dest.restore_pair_revolute(&image).unwrap();
            assert_eq!(dest.pair_reaction(first, second).unwrap(), a);
            assert_eq!(dest.checkpoint_pair_revolute().unwrap(), image);
            w = dest;
        }
    }
    assert!(
        peak > 0.5 && friction > 0.01,
        "normal {peak}, tangent {friction}"
    );
}

#[test]
fn pair_observer_follows_authored_reset_and_rejects_wrong_archives() {
    let (mut w, _, _, [a, _]) = four();
    let authored = w.initial_state();
    w.step(STEP).unwrap();
    let mut bytes = w.checkpoint_pair_revolute().unwrap();
    let (mut destination, ..) = four();
    let before = destination.checkpoint_pair_revolute().unwrap();
    bytes[40] ^= 1;
    assert!(destination.restore_pair_revolute(&bytes).is_err());
    let (mut wrong, _, _, _) = coupled();
    companion_at(&mut wrong, [2.1, 1.5, 0.5], [0.125, -0.25, 0.]);
    wrong.configure_pair_revolute().unwrap();
    assert!(
        destination
            .restore_pair_revolute(&wrong.checkpoint_pair_revolute().unwrap())
            .is_err()
    );
    assert_eq!(destination.checkpoint_pair_revolute().unwrap(), before);

    assert!(w.remove(a));
    w.step(STEP).unwrap();
    assert!(w.checkpoint_pair_revolute().is_err());
    let prepared = w.prepare_initial(&authored).unwrap();
    w.apply_initial(prepared).unwrap();
    assert!(w.pair_sphere_active(a).unwrap());
    w.step(STEP).unwrap();
    assert!(w.checkpoint_pair_revolute().is_ok());
}

#[test]
fn independently_solved_pair_keeps_the_entire_loaded_contact_during_warm_merge() {
    fn authored() -> (RapierBackend, [BodyId; 2], JointId, [BodyId; 2]) {
        let (mut w, links, joint) = hinge();
        let a = companion_at(&mut w, [2., 3., 0.5], [1., 0.2, 0.]);
        let b = companion_at(&mut w, [2.6, 3.08, 0.5], [-1., 0., 0.]);
        w.configure_pair_revolute().unwrap();
        (w, links, joint, [a, b])
    }
    fn cache(w: &RapierBackend, [a, b]: [BodyId; 2]) -> Vec<u8> {
        let handle = |id| w.world.bodies[w.bodies[&id].0].colliders()[0];
        bincode::serialize(w.world.contact_pair(handle(a), handle(b)).unwrap()).unwrap()
    }
    let (mut reference, links, j, balls) = authored();
    let mut pair = authored().0;
    let mut hinge = authored().0;
    pair.extract_sphere_island(true).unwrap();
    hinge.extract_sphere_island(false).unwrap();
    let mut merged = false;
    for tick in 1..=360 {
        for w in [&mut reference, &mut hinge] {
            w.add_joint_effort(j, 0.01).unwrap();
            w.step(STEP).unwrap();
        }
        if !merged {
            pair.step(STEP).unwrap();
        }
        let ball_world = if merged { &hinge } else { &pair };
        for id in links {
            assert_eq!(
                reference.state(id),
                hinge.state(id),
                "hinge state at {tick}"
            );
        }
        for id in balls {
            assert_eq!(
                reference.state(id),
                ball_world.state(id),
                "sphere state at {tick}"
            );
        }
        let r = reference.pair_reaction(balls[0], balls[1]).unwrap();
        assert_eq!(
            ball_world.pair_reaction(balls[0], balls[1]).unwrap(),
            r,
            "reaction at {tick}"
        );
        if !merged && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0. {
            assert_eq!(pair.world.multibody_joints.iter().count(), 0);
            assert!(
                links
                    .iter()
                    .all(|id| !pair.world.bodies[pair.bodies[id].0].is_enabled())
            );
            let image = pair.checkpoint_sphere_island().unwrap();
            let advanced_bytes = image.len();
            let raw = cache(&pair, balls);
            let mut restored = authored().0;
            restored.restore_sphere_island(&image).unwrap();
            assert_eq!(restored.checkpoint_sphere_island().unwrap(), image);
            assert_eq!(cache(&restored, balls), raw);
            hinge.merge_sphere_island(&image).unwrap();
            assert_eq!(hinge.pair_reaction(balls[0], balls[1]).unwrap(), r);
            assert_eq!(cache(&hinge, balls), raw);
            let full = hinge.checkpoint_pair_revolute().unwrap();
            let mut split = authored().0;
            split.restore_pair_revolute(&full).unwrap();
            split.extract_sphere_island(true).unwrap();
            assert_eq!(cache(&split, balls), raw);
            let image = split.checkpoint_sphere_island().unwrap();
            let mut restored = authored().0;
            restored.restore_sphere_island(&image).unwrap();
            assert_eq!(cache(&restored, balls), raw);
            let fresh_bytes = image.len();
            assert_eq!(restored.world.multibody_joints.iter().count(), 1);
            restored.step(STEP).unwrap();
            pair.step(STEP).unwrap();
            assert_eq!(restored.world.multibody_joints.iter().count(), 0);
            for id in balls {
                assert_eq!(restored.state(id), pair.state(id));
            }
            assert_eq!(cache(&restored, balls), cache(&pair, balls));
            println!(
                "pair image in the oblique fixture: fresh extraction {fresh_bytes} bytes, advanced pair {advanced_bytes} bytes"
            );
            println!(
                "preserved {} native contact bytes, applied {} and cached {} N·s at tick {tick}",
                raw.len(),
                r.normal_impulse,
                r.warmstart_normal_impulse
            );
            merged = true;
        }
    }
    assert!(merged);
}

#[test]
fn detached_pair_images_reject_wrong_shape_layout_digest_and_live_cross_contacts() {
    let (mut world, links, _, balls) = four();
    world.extract_sphere_island(true).unwrap();
    let image = world.checkpoint_sphere_island().unwrap();
    let (mut destination, ..) = four();
    let before = destination.checkpoint_pair_revolute().unwrap();
    assert!(destination.merge_sphere_island(&image).is_err());
    for bytes in [&[][..], &before[..], &image[..image.len() - 1]] {
        assert!(destination.restore_sphere_island(bytes).is_err());
        assert_eq!(destination.checkpoint_pair_revolute().unwrap(), before);
    }
    let mut corrupted = image.clone();
    corrupted[40] ^= 1;
    assert!(destination.restore_sphere_island(&corrupted).is_err());
    let (mut wrong, ..) = coupled();
    companion_at(&mut wrong, [3., 2., 0.5], [0.; 3]);
    wrong.configure_pair_revolute().unwrap();
    assert!(wrong.restore_sphere_island(&image).is_err());
    let mut checked = false;
    for _ in 0..500 {
        destination.step(STEP).unwrap();
        if links.iter().any(|link| {
            destination
                .pair_reaction(*link, balls[0])
                .unwrap()
                .normal_impulse
                > 0.
        }) {
            let coupled = destination.checkpoint_pair_revolute().unwrap();
            for keep_pair in [true, false] {
                assert!(destination.extract_sphere_island(keep_pair).is_err());
                assert_eq!(destination.checkpoint_pair_revolute().unwrap(), coupled);
            }
            checked = true;
            break;
        }
    }
    assert!(checked);
}
