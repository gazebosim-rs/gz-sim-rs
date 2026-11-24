// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn world(height: f64, ground_first: bool) -> (RapierBackend, [BodyId; 2], BodyId, JointId) {
    let mut backend = RapierBackend::new_multifidelity([0., 0., -9.81]).unwrap();
    let insert_ground = |b: &mut RapierBackend| {
        b.spawn_horizontal_halfspace(gz_physics::PlaneSpec {
            position: [0., 0., height],
            orientation: [0., 0., 0., 1.],
            size: [10., 10.],
        })
        .unwrap()
    };
    let ground = ground_first.then(|| insert_ground(&mut backend));
    let links = [(-0.25, 0.2), (0.35, 0.25)].map(|(x, z)| {
        backend
            .spawn_articulation_link(
                BodySpec {
                    position: [x, 0., z],
                    velocity: [0.12, 0.02, 0.],
                    shape: Shape::Sphere { radius: 0.11 },
                    mass: Some(1.5),
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [0.015, -0.008, 0.],
                    orientation: [0., 0., 0., 1.],
                    tensor: [0.034, 0.038, 0.035, 0., 0., 0.],
                }),
            )
            .unwrap()
    });
    let joint = backend
        .add_reduced_revolute(Some(links[0]), links[1], [0., 0., 0.24], [0., 1., 0.])
        .unwrap();
    let ground = ground.unwrap_or_else(|| insert_ground(&mut backend));
    (backend, links, ground, joint)
}

#[test]
fn grounded_hinge_restarts_with_loaded_manifolds_and_queued_torque() {
    for first in [true, false] {
        let (mut source, links, ground, joint) = world(0., first);
        let (mut trial, ..) = world(0., first);
        let mut active_cuts = 0;
        let mut peak = 0.0_f64;
        for i in 1..=430 {
            source.step(STEP).unwrap();
            trial.step(STEP).unwrap();
            for link in links {
                assert_eq!(source.state(link), trial.state(link), "tick {i}");
                let s = source.ground_reaction(link, ground).unwrap();
                assert_eq!(s, trial.ground_reaction(link, ground).unwrap(), "tick {i}");
                peak = peak.max(s.normal_impulse);
            }
            if i % 13 == 1 {
                for w in [&mut source, &mut trial] {
                    w.add_joint_effort(joint, if i < 200 { 0.8 } else { -0.35 })
                        .unwrap();
                }
                let bytes = source.checkpoint_grounded_revolute().unwrap();
                assert_eq!(bytes, trial.checkpoint_grounded_revolute().unwrap());
                let (mut replacement, ..) = world(0., first);
                replacement
                    .restore_grounded_revolute(&bytes)
                    .unwrap_or_else(|e| panic!("restore at {i}, {} bytes: {e}", bytes.len()));
                assert_eq!(
                    replacement.checkpoint_grounded_revolute().unwrap(),
                    bytes,
                    "restored bytes at {i}"
                );
                assert_eq!(source.joint_state(joint), replacement.joint_state(joint));
                for link in links {
                    let a = source.ground_reaction(link, ground).unwrap();
                    assert_eq!(a, replacement.ground_reaction(link, ground).unwrap());
                    active_cuts +=
                        usize::from(a.normal_impulse > 0. && a.warmstart_normal_impulse > 0.);
                }
                trial = replacement;
            }
        }
        assert!(
            peak > 1. && active_cuts >= 10,
            "peak={peak} cuts={active_cuts}"
        );
    }
}

#[test]
fn ground_impulses_close_the_linear_momentum_budget_in_world_axes() {
    let mut residuals = vec![];
    for microseconds in [1000_u64, 500, 250] {
        let dt = Duration::from_micros(microseconds);
        let (mut body, links, ground, joint) = world(0., true);
        let momentum = |w: &RapierBackend| {
            links
                .into_iter()
                .map(|id| 1.5 * w.world.bodies[w.bodies[&id].0].linvel())
                .sum::<Vector>()
        };
        let initial = momentum(&body);
        let mut impulses = Vector::ZERO;
        let mut max_residual = 0.0_f64;
        let mut impact = 0.0_f64;
        let mut peak_tangent = 0.0_f64;
        for tick in 1..=(600_000 / microseconds) {
            body.add_joint_effort(
                joint,
                if (tick - 1) * microseconds < 349_000 {
                    1.4
                } else {
                    -0.8
                },
            )
            .unwrap();
            body.step(dt).unwrap();
            for id in links {
                let r = body.ground_reaction(id, ground).unwrap();
                impulses += Vector::Z * r.normal_impulse + Vector::from(r.tangent_impulse);
                impact = impact.max(r.normal_impulse);
                peak_tangent = peak_tangent.max(Vector::from(r.tangent_impulse).length());
            }
            let expected =
                initial + Vector::Z * (-9.81 * 3. * tick as f64 * dt.as_secs_f64()) + impulses;
            max_residual = max_residual.max((momentum(&body) - expected).length());
        }
        println!(
            "600ms check at {microseconds} us per step: max linear momentum residual {max_residual} N·s, peak normal {impact} N·s, peak tangent {peak_tangent} N·s"
        );
        assert!(impact > 1. && peak_tangent > 0.02 && max_residual < 0.02);
        residuals.push(max_residual);
    }
    assert!(residuals[2] < residuals[1] && residuals[1] < residuals[0]);
}

#[test]
fn resealed_changes_to_the_plane_mass_material_or_contact_are_rejected_atomically() {
    let (mut source, links, plane, joint) = world(0., true);
    for _ in 0..235 {
        source.step(STEP).unwrap();
    }
    source.add_joint_effort(joint, 0.75).unwrap();
    let live = source.checkpoint_grounded_revolute().unwrap();
    let point = source
        .world
        .narrow_phase
        .contact_pairs()
        .flat_map(|p| p.solver_manifolds())
        .flat_map(|m| &m.points)
        .find(|p| p.data.impulse > 0.)
        .unwrap();
    let pattern = graph_codec().serialize(&point.data).unwrap();
    let offset = live[40..]
        .windows(pattern.len())
        .position(|v| v == pattern)
        .unwrap()
        + 40;
    let mut nonfinite = live.clone();
    nonfinite[offset..offset + 8].copy_from_slice(&f64::NAN.to_le_bytes());
    let digest = Sha256::digest(&nonfinite[40..]);
    nonfinite[8..40].copy_from_slice(&digest);
    let (mut fresh, ..) = world(0., true);
    let before = links.map(|id| fresh.state(id));
    assert!(fresh.restore_grounded_revolute(&nonfinite).is_err());

    let ground_handle = source.bodies[&plane].0;
    let root = source.bodies[&links[0]].0;
    for change in 0..4 {
        let (signature, q, pending, mut graph): ([u8; 32], f64, Option<f64>, PhysicsWorld) =
            graph_codec().deserialize(&live[40..]).unwrap();
        match change {
            0 => graph.bodies[ground_handle].set_translation(Vector::Z, false),
            1 => graph.bodies[ground_handle].set_rotation(Rotation::from_rotation_x(0.3), false),
            2 => {
                let collider = graph.bodies[root].colliders()[0];
                graph.colliders[collider].set_friction(0.);
            }
            _ => graph.bodies[root].set_additional_mass(13., false),
        }
        let payload = graph_codec()
            .serialize(&(signature, q, pending, graph))
            .unwrap();
        let mut bytes = MAGIC.to_vec();
        bytes.extend_from_slice(&Sha256::digest(&payload));
        bytes.extend_from_slice(&payload);
        assert!(
            fresh.restore_grounded_revolute(&bytes).is_err(),
            "variant {change}"
        );
        assert_eq!(before, links.map(|id| fresh.state(id)));
        assert!(fresh.joints.open && fresh.joints.efforts.is_empty());
    }
    let (mut raised_ground, ..) = world(0.02, true);
    assert!(raised_ground.restore_grounded_revolute(&live).is_err());
    fresh.restore_grounded_revolute(&live).unwrap();
    assert_eq!(fresh.checkpoint_grounded_revolute().unwrap(), live);
}

#[test]
fn legacy_pair_link_requires_authored_inertia_and_an_attached_collider() {
    let collider = ColliderBuilder::ball(0.1).density(0.).build();
    let unconfigured = RigidBodyBuilder::dynamic().build();
    assert!(!legacy_pair_link_mass(
        &unconfigured,
        &collider,
        &unconfigured,
        &collider
    ));
    let authored = RigidBodyBuilder::dynamic()
        .additional_mass_properties(MassProperties::new(Vector::ZERO, 1., Vector::ONE))
        .build();
    assert!(!legacy_pair_link_mass(
        &authored, &collider, &authored, &collider
    ));
}
