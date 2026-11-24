// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn make(parents: &[usize], group: Option<u64>) -> (RapierBackend, Vec<BodyId>, Vec<JointId>) {
    make_with_stops(parents, group, &[])
}
fn make_with_stops(
    parents: &[usize],
    group: Option<u64>,
    limits: &[Option<[f64; 2]>],
) -> (RapierBackend, Vec<BodyId>, Vec<JointId>) {
    let mut b = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let xyz = |i: usize| {
        [
            0.21 * (i as f64),
            0.04 * ((i % 3) as f64),
            0.11 + 0.05 * ((i % 2) as f64),
        ]
    };
    let mut links = Vec::new();
    for i in 0..=parents.len() {
        let body = b
            .spawn_articulation_link(
                BodySpec {
                    position: xyz(i),
                    velocity: [0.08, -0.06, 0.04],
                    mass: Some(1.2 + (i % 3) as f64 * 0.1),
                    shape: Shape::Sphere { radius: 0.18 },
                },
                RotationSpec {
                    orientation: Rotation::from_rotation_z(i as f64 * 0.067).to_array(),
                    angular_velocity: [0.; 3],
                },
                Some(Inertia {
                    center: [0.025, -0.016, 0.009],
                    orientation: Rotation::from_rotation_y(0.27).to_array(),
                    tensor: [0.025, 0.027, 0.03, 0.002, -0.001, 0.],
                }),
            )
            .unwrap();
        b.set_self_collision_group(body, group).unwrap();
        links.push(body);
    }
    let axes = [[0.1, 0.1, 1.], [0.2, 1., -0.05], [1., -0.12, 0.18]];
    let joints = parents
        .iter()
        .enumerate()
        .map(|(k, &p)| {
            let i = k + 1;
            let anchor = std::array::from_fn(|j| (xyz(i)[j] + xyz(p)[j]) * 0.5);
            match limits.get(k).copied().flatten() {
                Some(range) => {
                    b.add_limited_revolute(Some(links[p]), links[i], anchor, axes[k % 3], range)
                }
                None => b.add_reduced_revolute(Some(links[p]), links[i], anchor, axes[k % 3]),
            }
            .unwrap()
        })
        .collect();
    (b, links, joints)
}

fn compare(a: &RapierBackend, b: &RapierBackend, links: &[BodyId], joints: &[JointId]) {
    for &link in links {
        let a = a.state(link).unwrap();
        let b = b.state(link).unwrap();
        assert_eq!(a.position.map(f64::to_bits), b.position.map(f64::to_bits));
        assert_eq!(a.velocity.map(f64::to_bits), b.velocity.map(f64::to_bits));
        assert_eq!(
            a.orientation.map(f64::to_bits),
            b.orientation.map(f64::to_bits)
        );
        assert_eq!(
            a.angular_velocity.map(f64::to_bits),
            b.angular_velocity.map(f64::to_bits)
        );
    }
    for &joint in joints {
        let p = a.joint_state(joint).unwrap();
        let q = b.joint_state(joint).unwrap();
        assert_eq!(
            (p.position.to_bits(), p.velocity.to_bits()),
            (q.position.to_bits(), q.velocity.to_bits())
        );
        assert_eq!(
            a.joints.efforts.get(&joint).map(|v| v.to_bits()),
            b.joints.efforts.get(&joint).map(|v| v.to_bits())
        );
    }
}

#[test]
fn branched_and_long_serial_roundtrips_match_every_native_state_and_consume_pending_torques() {
    for parents in [vec![0, 1, 1, 2, 0], (0..15).collect(), vec![0, 0]] {
        let (mut control, links, joints) = make(&parents, Some(11));
        let (mut journey, _, _) = make(&parents, Some(11));
        for tick in 1..=300 {
            if tick % 3 == 1 {
                for (i, &id) in joints.iter().enumerate() {
                    let e = if (i + tick) % 2 == 0 { 0.24 } else { -0.31 };
                    control.add_joint_effort(id, e).unwrap();
                    journey.add_joint_effort(id, e).unwrap();
                }
            }
            if [2, 37, 63, 139, 290].contains(&tick) {
                let image = journey.checkpoint_floating_revolute_tree().unwrap();
                let (mut next, _, _) = make(&parents, Some(11));
                next.restore_floating_revolute_tree(&image).unwrap();
                assert_eq!(next.checkpoint_floating_revolute_tree().unwrap(), image);
                compare(&control, &next, &links, &joints);
                journey = next;
            }
            control.step(STEP).unwrap();
            journey.step(STEP).unwrap();
            compare(&control, &journey, &links, &joints);
            assert_eq!(
                journey.checkpoint_floating_revolute_tree().unwrap(),
                control.checkpoint_floating_revolute_tree().unwrap()
            );
        }
    }
}

#[test]
fn bounded_wire_is_checked_against_fresh_construction_and_committed_link_motion() {
    let parents = [0, 0, 2];
    let (mut source, links, joints) = make(&parents, Some(7));
    assert!(source.checkpoint_floating_revolute_tree().is_err());
    source.add_joint_effort(joints[0], 0.37).unwrap();
    source.step(STEP).unwrap();
    source.add_joint_effort(joints[0], 0.).unwrap();
    source.add_joint_effort(joints[2], -0.21).unwrap();
    let image = source.checkpoint_floating_revolute_tree().unwrap();
    assert_eq!(image.len(), FIXED_BYTES + 17 * 3 + 16);
    let (mut dest, _, _) = make(&parents, Some(7));
    let mut bads = vec![
        vec![],
        vec![0; FLOATING_TREE_IMAGE_LIMIT + 1],
        image[..image.len() - 1].to_vec(),
    ];
    for (index, value) in [
        (0, 0x88),
        (30, 0x3a),
        (44, 0xff),
        (91, 0xee),
        (FIXED_BYTES + 16, 19),
    ] {
        let mut b = image.clone();
        b[index] = value;
        if index >= 40 {
            let d = Sha256::digest(&b[40..]);
            b[8..40].copy_from_slice(&d);
        }
        bads.push(b);
    }
    for (index, value) in [
        (104, f64::MAX),
        (128, 2.0),
        (160, f64::NAN),
        (FIXED_BYTES, f64::NAN),
        (FIXED_BYTES + 8, f64::INFINITY),
        (FIXED_BYTES + 17, -0.0),
    ] {
        let mut b = image.clone();
        b[index..index + 8].copy_from_slice(&value.to_le_bytes());
        let d = Sha256::digest(&b[40..]);
        b[8..40].copy_from_slice(&d);
        bads.push(b);
    }
    let mut extra = image.clone();
    extra.push(0);
    let d = Sha256::digest(&extra[40..]);
    extra[8..40].copy_from_slice(&d);
    bads.push(extra);
    for bad in bads {
        let (reference, _, _) = make(&parents, Some(7));
        assert!(dest.restore_floating_revolute_tree(&bad).is_err());
        compare(&dest, &reference, &links, &joints);
    }
    for (top, group) in [
        (vec![0, 1, 2], Some(7)),
        (parents.to_vec(), Some(8)),
        (parents.to_vec(), None),
    ] {
        let (mut other, _, _) = make(&top, group);
        assert!(other.restore_floating_revolute_tree(&image).is_err());
    }
    dest.add_joint_effort(joints[1], 0.11).unwrap();
    assert!(dest.restore_floating_revolute_tree(&image).is_err());
    assert_eq!(dest.joints.efforts[&joints[1]], 0.11);
    dest.reset().unwrap();
    dest.restore_floating_revolute_tree(&image).unwrap();
    assert!(dest.restore_floating_revolute_tree(&image).is_err());
    compare(&source, &dest, &links, &joints);
    let handle = source.bodies[&links[1]].0;
    source.world.bodies[handle].set_linvel(Vector::Z, false);
    assert!(source.checkpoint_floating_revolute_tree().is_err());
}

#[test]
fn maximum_profile_roundtrips_and_one_additional_link_is_refused() {
    let parents = vec![0; MAX_FLOATING_TREE_LINKS - 1];
    let (mut source, links, joints) = make(&parents, Some(5));
    let (mut dest, _, _) = make(&parents, Some(5));
    source.step(STEP).unwrap();
    for &joint in &joints {
        source.add_joint_effort(joint, -0.008).unwrap();
    }
    let image = source.checkpoint_floating_revolute_tree().unwrap();
    assert_eq!(image.len(), FLOATING_TREE_IMAGE_LIMIT);
    dest.restore_floating_revolute_tree(&image).unwrap();
    source.step(STEP).unwrap();
    dest.step(STEP).unwrap();
    compare(&source, &dest, &links, &joints);
    let (mut oversized, _, _) = make(&vec![0; MAX_FLOATING_TREE_LINKS], Some(5));
    oversized.step(STEP).unwrap();
    assert!(oversized.checkpoint_floating_revolute_tree().is_err());
}

#[test]
fn finite_stops_hold_release_and_reverse_across_repeated_active_native_restores() {
    let parents = [0, 1, 1, 2];
    let limits = [
        Some([-0.017, 0.023]),
        Some([-0.033, 0.014]),
        None,
        Some([0., 0.]),
    ];
    let build = || make_with_stops(&parents, Some(19), &limits);
    let (mut reference, links, joints) = build();
    let (mut trip, _, _) = build();
    let mut minima = [f64::INFINITY; 4];
    let mut maxima = [f64::NEG_INFINITY; 4];
    let mut active_cuts = [0; 2];
    let mut restored = 0;
    for tick in 0..1200 {
        for (i, &j) in joints.iter().enumerate() {
            let sign = if (tick / 150 + i) % 2 == 0 { 1. } else { -1. };
            let effort = sign * if i == 2 { 0.02 } else { 0.65 };
            reference.add_joint_effort(j, effort).unwrap();
            trip.add_joint_effort(j, effort).unwrap();
        }
        if tick > 0 && (tick % 53 == 0 || tick % 150 == 0 || tick % 150 == 1) {
            let bytes = trip.checkpoint_floating_revolute_tree().unwrap();
            assert_eq!(&bytes[..8], STOPPED_MAGIC);
            let (mut next, _, _) = build();
            next.restore_floating_revolute_tree(&bytes).unwrap();
            compare(&reference, &next, &links, &joints);
            let q = next.joint_state(joints[0]).unwrap().position;
            if q <= limits[0].unwrap()[0] {
                active_cuts[0] += 1
            }
            if q >= limits[0].unwrap()[1] {
                active_cuts[1] += 1
            }
            trip = next;
            restored += 1;
        }
        reference.step(STEP).unwrap();
        trip.step(STEP).unwrap();
        compare(&reference, &trip, &links, &joints);
        assert_eq!(
            reference.checkpoint_floating_revolute_tree().unwrap(),
            trip.checkpoint_floating_revolute_tree().unwrap()
        );
        for (i, &j) in joints.iter().enumerate() {
            let p = reference.joint_state(j).unwrap().position;
            minima[i] = minima[i].min(p);
            maxima[i] = maxima[i].max(p);
        }
    }
    assert!(
        active_cuts.into_iter().all(|n| n > 0),
        "{active_cuts:?}, {minima:?}, {maxima:?}"
    );
    for (i, range) in limits.iter().enumerate() {
        if let Some([lo, hi]) = range {
            assert!(
                minima[i] <= *lo
                    && maxima[i] >= *hi
                    && minima[i] > *lo - 0.0005
                    && maxima[i] < *hi + 0.0005,
                "{minima:?} {maxima:?}"
            );
        }
    }
    assert!(restored >= 30);
    eprintln!(
        "restored={restored} active_lower_upper={active_cuts:?} observed_min={minima:?} observed_max={maxima:?}"
    );
    let image = trip.checkpoint_floating_revolute_tree().unwrap();
    let mut wrong = limits;
    wrong[0] = Some([-0.018, 0.023]);
    let (mut wrong, _, _) = make_with_stops(&parents, Some(19), &wrong);
    assert!(wrong.restore_floating_revolute_tree(&image).is_err());
    let (mut unbounded, _, _) = make(&parents, Some(19));
    assert!(unbounded.restore_floating_revolute_tree(&image).is_err());
    let mut wrong_magic = image;
    wrong_magic[..8].copy_from_slice(TREE_MAGIC);
    let (mut fresh, _, _) = build();
    assert!(fresh.restore_floating_revolute_tree(&wrong_magic).is_err());
}

#[test]
fn persistent_velocity_motors_restore_under_stops_and_clear_only_on_request_or_reset() {
    let limits = [Some([-0.025, 0.04]), Some([-0.035, 0.03]), None, None];
    let make = || make_with_stops(&[0, 0, 1, 1], Some(4), &limits);
    let (mut local, links, joints) = make();
    let (mut trip, _, _) = make();
    let mut motor = Some(JointVelocityMotor {
        target_velocity: 1.6,
        max_force: 0.75,
    });
    for tick in 0..1200 {
        if [0, 280, 510, 640, 870].contains(&tick) {
            motor = match tick {
                0 => motor,
                280 => Some(JointVelocityMotor {
                    target_velocity: -1.2,
                    max_force: 0.4,
                }),
                510 => None,
                640 => Some(JointVelocityMotor {
                    target_velocity: 0.0,
                    max_force: 0.62,
                }),
                _ => Some(JointVelocityMotor {
                    target_velocity: 1.1,
                    max_force: 0.0,
                }),
            };
            for b in [&mut local, &mut trip] {
                b.set_joint_velocity_motor(joints[0], motor).unwrap();
                b.set_joint_velocity_motor(
                    joints[3],
                    Some(JointVelocityMotor {
                        target_velocity: -1.7,
                        max_force: 0.32,
                    }),
                )
                .unwrap();
            }
        }
        for b in [&mut local, &mut trip] {
            if tick % 9 == 0 {
                b.add_joint_effort(joints[1], 0.18).unwrap();
            }
            if tick % 7 == 0 {
                b.add_joint_effort(joints[2], -0.06).unwrap();
            }
        }
        if tick > 0 && (tick % 77 == 0 || [280, 510, 640, 870].contains(&tick)) {
            let image = trip.checkpoint_motor_revolute_tree().unwrap();
            assert_eq!(&image[..8], MOTOR_MAGIC);
            assert!(trip.checkpoint_floating_revolute_tree().is_err());
            let (mut restored, _, _) = make();
            assert!(restored.restore_floating_revolute_tree(&image).is_err());
            restored.restore_motor_revolute_tree(&image).unwrap();
            compare(&local, &restored, &links, &joints);
            assert_eq!(restored.checkpoint_motor_revolute_tree().unwrap(), image);
            trip = restored;
        }
        assert_eq!(trip.motor_revolute_velocity(joints[0]).unwrap(), motor);
        assert_eq!(local.motor_revolute_velocity(joints[0]).unwrap(), motor);
        local.step(STEP).unwrap();
        trip.step(STEP).unwrap();
        compare(&local, &trip, &links, &joints);
        assert_eq!(
            trip.checkpoint_motor_revolute_tree().unwrap(),
            local.checkpoint_motor_revolute_tree().unwrap()
        );
    }
    let image = trip.checkpoint_motor_revolute_tree().unwrap();
    let (mut fresh, _, _) = make();
    fresh
        .set_joint_velocity_motor(
            joints[1],
            Some(JointVelocityMotor {
                target_velocity: 0.0,
                max_force: 1.0,
            }),
        )
        .unwrap();
    assert!(fresh.restore_motor_revolute_tree(&image).is_err());
    fresh.reset().unwrap();
    fresh.restore_motor_revolute_tree(&image).unwrap();
    fresh
        .set_joint_velocity_servo(
            joints[0],
            Some(JointVelocityMotor {
                target_velocity: 0.4,
                max_force: 0.1,
            }),
        )
        .unwrap();
    assert!(fresh.checkpoint_motor_revolute_tree().is_err());
    assert!(fresh.motor_revolute_velocity(joints[0]).is_err());
    fresh.reset().unwrap();
    assert!(
        joints
            .iter()
            .all(|&j| fresh.motor_revolute_velocity(j).unwrap().is_none())
    );
    let mut corrupt = image;
    // The last joint is actuated; the final eight bytes hold its maximum torque.
    let n = corrupt.len();
    corrupt[n - 8..].copy_from_slice(&(-0.02f64).to_le_bytes());
    let digest = Sha256::digest(&corrupt[40..]);
    corrupt[8..40].copy_from_slice(&digest);
    assert!(fresh.restore_motor_revolute_tree(&corrupt).is_err());
    assert!(
        joints
            .iter()
            .all(|&j| fresh.motor_revolute_velocity(j).unwrap().is_none())
    );
}

#[test]
fn explicit_no_sleep_keeps_a_released_slow_body_coasting_after_reset() {
    let make = || {
        let mut world = RapierBackend::new_multifidelity([0.; 3]).unwrap();
        let body = world
            .spawn_link(
                BodySpec {
                    position: [0.; 3],
                    velocity: [0.025, 0., 0.],
                    mass: Some(0.72),
                    shape: Shape::Sphere { radius: 0.16 },
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [0.; 3],
                    orientation: [0., 0., 0., 1.],
                    tensor: [0.01, 0.01, 0.01, 0., 0., 0.],
                }),
            )
            .unwrap();
        (world, body)
    };
    let (mut awake, id) = make();
    let params = awake.world.integration_parameters;
    awake.keep_body_awake(id).unwrap();
    assert_eq!(awake.world.integration_parameters, params);
    let (mut default, other) = make();
    for _ in 0..1600 {
        awake.step(STEP).unwrap();
        default.step(STEP).unwrap();
    }
    assert_eq!(awake.state(id).unwrap().velocity, [0.025, 0., 0.]);
    assert_eq!(default.state(other).unwrap().velocity, [0.; 3]);
    assert!(awake.state(id).unwrap().position[0] > 0.0399);
    awake.reset().unwrap();
    for _ in 0..1600 {
        awake.step(STEP).unwrap();
    }
    assert_eq!(awake.state(id).unwrap().velocity, [0.025, 0., 0.]);
}

#[test]
fn timed_native_images_bind_the_real_solver_duration_and_exclude_fixed_step_recipes() {
    use sha2::{Digest, Sha256};
    use std::time::Duration;
    let (mut direct, links, joints) = make(&[0, 0, 2], Some(7));
    let (mut recovered, _, _) = make(&[0, 0, 2], Some(7));
    let mut fixed = None;
    for nanos in [
        1_000_000u64,
        10_000_000,
        2_371_250,
        7_628_750,
        1,
        350_123,
        10_000_000,
    ] {
        let dt = Duration::from_nanos(nanos);
        for b in [&mut direct, &mut recovered] {
            b.add_joint_effort(joints[0], 0.23).unwrap();
            b.step(dt).unwrap();
        }
        compare(&direct, &recovered, &links, &joints);
        let image = direct.checkpoint_timed_revolute_tree(dt).unwrap();
        if nanos == 1_000_000 {
            fixed = Some(direct.checkpoint_floating_revolute_tree().unwrap());
        } else {
            assert!(direct.checkpoint_floating_revolute_tree().is_err());
        }
        assert!(
            direct
                .checkpoint_timed_revolute_tree(Duration::ZERO)
                .is_err()
        );
        assert!(
            direct
                .checkpoint_timed_revolute_tree(Duration::from_nanos(nanos + 1))
                .is_err()
        );
        let (mut staged, _, _) = make(&[0, 0, 2], Some(7));
        let mut false_duration = image.clone();
        false_duration[40..48].copy_from_slice(
            &if nanos == 5_000_000 {
                2_000_000u64
            } else {
                5_000_000u64
            }
            .to_le_bytes(),
        );
        let hash = Sha256::digest(&false_duration[40..]);
        false_duration[8..40].copy_from_slice(&hash);
        assert!(staged.restore_timed_revolute_tree(&false_duration).is_err());
        assert!(staged.restore_floating_revolute_tree(&image[48..]).is_err());
        let mut legacy = image[..48].to_vec();
        legacy.extend(fixed.as_ref().unwrap());
        let hash = Sha256::digest(&legacy[40..]);
        legacy[8..40].copy_from_slice(&hash);
        assert!(staged.restore_timed_revolute_tree(&legacy).is_err());
        assert_eq!(staged.restore_timed_revolute_tree(&image).unwrap(), dt);
        compare(&direct, &staged, &links, &joints);
        assert_eq!(staged.checkpoint_timed_revolute_tree(dt).unwrap(), image);
        recovered = staged;
    }
}
