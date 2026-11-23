// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn make(height: f64, ground_first: bool) -> (RapierBackend, [BodyId; 3], BodyId, [JointId; 2]) {
    let mut b = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
    let plane = |b: &mut RapierBackend| {
        b.spawn_horizontal_halfspace(gz_physics::PlaneSpec {
            position: [0., 0., height],
            orientation: [0., 0., 0., 1.],
            size: [5., 6.],
        })
        .unwrap()
    };
    let plane_id = ground_first.then(|| plane(&mut b));
    let coords = [[0., 0., 0.53], [-0.24, 0., 0.24], [0.23, 0.03, 0.25]];
    let links = coords.map(|xyz| {
        let h = b
            .spawn_articulation_link(
                BodySpec {
                    position: xyz,
                    velocity: [0.18, -0.05, 0.],
                    mass: Some(1.5),
                    shape: Shape::Sphere { radius: 0.12 },
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [0.008, -0.005, 0.012],
                    orientation: [0., 0., 0., 1.],
                    tensor: [0.036, 0.04, 0.042, 0., 0., 0.],
                }),
            )
            .unwrap();
        b.set_self_collision_group(h, Some(5)).unwrap();
        h
    });
    let joints = [1, 2].map(|c| {
        let anchor = std::array::from_fn(|i| (coords[0][i] + coords[c][i]) * 0.5);
        b.add_limited_revolute(
            Some(links[0]),
            links[c],
            anchor,
            [0., 1., 0.],
            [-0.33, 0.36],
        )
        .unwrap()
    });
    let floor = plane_id.unwrap_or_else(|| plane(&mut b));
    (b, links, floor, joints)
}

#[test]
fn grounded_tree_preserves_solved_and_warm_reactions_through_both_ground_insertion_orders() {
    for first in [true, false] {
        let build = || make(0., first);
        let (mut base, links, floor, joints) = build();
        let (mut trip, _, _, _) = build();
        let mut loaded = 0;
        let mut tangential = 0;
        let mut air = 0;
        let mut warm_max = 0f64;
        for tick in 1..=600 {
            for b in [&mut base, &mut trip] {
                let sign = if tick < 350 { 1. } else { -1. };
                b.add_joint_effort(joints[0], sign * 0.5).unwrap();
                b.add_joint_effort(joints[1], -sign * 0.4).unwrap();
                b.step(STEP).unwrap();
                if tick % 23 == 0 {
                    b.add_joint_effort(joints[1], -0.07).unwrap();
                }
            }
            if tick % 23 == 0 {
                let image = trip.checkpoint_grounded_revolute_tree().unwrap();
                let (mut dest, _, _, _) = build();
                assert!(dest.restore_floating_revolute_tree(&image).is_err());
                let (mut other, _, _, _) = make(0.015, first);
                assert!(other.restore_grounded_revolute_tree(&image).is_err());
                let mut corrupt = image.clone();
                corrupt[72..80].copy_from_slice(&0.12f64.to_le_bytes());
                let d = Sha256::digest(&corrupt[40..]);
                corrupt[8..40].copy_from_slice(&d);
                assert!(dest.restore_grounded_revolute_tree(&corrupt).is_err());
                dest.restore_grounded_revolute_tree(&image).unwrap();
                assert_eq!(image, dest.checkpoint_grounded_revolute_tree().unwrap());
                let all = links.map(|l| dest.ground_reaction(l, floor).unwrap());
                loaded +=
                    usize::from(all.iter().any(|r| {
                        r.active && r.normal_impulse > 0. && r.warmstart_normal_impulse > 0.
                    }));
                tangential += usize::from(
                    all.iter()
                        .any(|r| Vector::from_array(r.tangent_impulse).length() > 1e-7),
                );
                air += usize::from(all.iter().all(|r| !r.active));
                trip = dest;
            }
            for l in links {
                assert_eq!(base.state(l), trip.state(l));
                let r = base.ground_reaction(l, floor).unwrap();
                warm_max = warm_max.max(r.warmstart_normal_impulse);
                assert_eq!(r, trip.ground_reaction(l, floor).unwrap());
            }
            for j in joints {
                assert_eq!(base.joint_state(j), trip.joint_state(j));
            }
            assert_eq!(
                base.checkpoint_grounded_revolute_tree().unwrap(),
                trip.checkpoint_grounded_revolute_tree().unwrap()
            );
        }
        assert!(loaded >= 10 && tangential >= 5 && air >= 2 && warm_max > 0.02);
        println!(
            "ground_first={first} warm_cuts={loaded} tangential_cuts={tangential} airborne_cuts={air} peak_warm={warm_max}"
        );
    }
}
#[test]
fn external_plane_impulses_balance_ordinary_three_link_momentum() {
    let mut errors = Vec::new();
    for us in [1000, 500, 250] {
        let dt = Duration::from_micros(us);
        let (mut b, links, floor, joints) = make(0., true);
        let momentum = |b: &RapierBackend| {
            links.iter().fold(Vector::ZERO, |p, l| {
                p + 1.5 * b.world.bodies[b.bodies[l].0].linvel()
            })
        };
        let start = momentum(&b);
        let mut contacts = Vector::ZERO;
        let mut worst = 0f64;
        let mut peak_tangent = 0f64;
        let mut peak_normal = 0f64;
        for i in 1..=600000 / us {
            let sign = if i * us < 350000 { 1. } else { -1. };
            b.add_joint_effort(joints[0], sign * 0.5).unwrap();
            b.add_joint_effort(joints[1], -sign * 0.4).unwrap();
            b.step(dt).unwrap();
            for l in links {
                let r = b.ground_reaction(l, floor).unwrap();
                let tangent = Vector::from_array(r.tangent_impulse);
                peak_tangent = peak_tangent.max(tangent.length());
                peak_normal = peak_normal.max(r.normal_impulse);
                contacts += Vector::Z * r.normal_impulse + tangent;
            }
            let expected =
                start + contacts + Vector::Z * (-9.81 * 4.5 * i as f64 * dt.as_secs_f64());
            worst = worst.max((momentum(&b) - expected).length());
        }
        println!(
            "dt_us={us} max_linear_momentum_residual={worst} peak_tangent_impulse={peak_tangent} peak_normal_impulse={peak_normal}"
        );
        assert!(peak_normal > 0.5 && peak_tangent > 0.01 && worst < 0.08);
        errors.push(worst);
    }
    assert!(errors[2] < errors[0]);
}
