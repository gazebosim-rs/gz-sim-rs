// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn setup(radius: f64) -> (RapierBackend, [BodyId; 2], JointId) {
    let mut world = RapierBackend::new_multifidelity([0.; 3]).unwrap();
    let mut body = |x, offset| {
        world
            .spawn_articulation_link(
                BodySpec {
                    position: [x, 0., 1.5],
                    velocity: [0.25, -0.04, 0.06],
                    shape: Shape::Sphere { radius },
                    mass: Some(1.),
                },
                RotationSpec::default(),
                Some(Inertia {
                    center: [offset, 0.01, 0.],
                    orientation: [0., 0., 0., 1.],
                    tensor: [0.032, 0.031, 0.03, 0., 0., 0.],
                }),
            )
            .unwrap()
    };
    let root = body(-0.3, 0.027);
    let tip = body(0.4, -0.013);
    let j = world
        .add_reduced_revolute(Some(root), tip, [0.03, 0.06, 1.5], [0., 1., 0.])
        .unwrap();
    (world, [root, tip], j)
}
fn moving() -> (RapierBackend, [BodyId; 2], JointId) {
    let (mut w, links, j) = setup(0.12);
    for _ in 0..19 {
        w.add_joint_effort(j, 0.75).unwrap();
        w.step(STEP).unwrap();
    }
    (w, links, j)
}
fn replace(bytes: &[u8], change: impl FnOnce(&mut Image)) -> Vec<u8> {
    let mut read = &bytes[40..];
    let mut data: Image = codec().deserialize_from(&mut read).unwrap();
    change(&mut data);
    let payload = codec().serialize(&data).unwrap();
    let mut out = Vec::from(*MAGIC);
    out.extend_from_slice(&Sha256::digest(&payload));
    out.extend(payload);
    out
}
#[test]
fn malformed_images_leave_construction_native_state_and_queued_effort_unchanged() {
    let (mut source, links, j) = moving();
    source.add_joint_effort(j, -0.25).unwrap();
    let good = source.checkpoint_floating_revolute().unwrap();
    assert_eq!(good.len(), EMPTY_BYTES + 8);
    let mut bad_magic = good.clone();
    bad_magic[3] ^= 1;
    let mut bad_digest = good.clone();
    bad_digest[10] ^= 1;
    let mut trailing = good.clone();
    trailing.push(0);
    let mut unknown_option = good.clone();
    let option_tag = good.len() - 17;
    unknown_option[option_tag] = 2;
    let digest = Sha256::digest(&unknown_option[40..]);
    unknown_option[8..40].copy_from_slice(&digest);
    let cases = [
        vec![],
        good[..40].to_vec(),
        good[..good.len() - 1].to_vec(),
        trailing,
        bad_magic,
        bad_digest,
        unknown_option,
        replace(&good, |m| m.effort = Some(-0.0)),
        replace(&good, |m| m.links[0].linear[0] = f64::MAX),
        replace(&good, |m| m.construction[0] ^= 1),
        replace(&good, |m| m.angle = f64::NAN),
        replace(&good, |m| m.effort = Some(f64::INFINITY)),
        replace(&good, |m| m.links[0].position[0] = f64::INFINITY),
        replace(&good, |m| m.links[1].orientation[1] = 1.2),
        replace(&good, |m| m.links[1].position[1] += 0.01),
        replace(&good, |m| m.links[1].linear[0] += 0.1),
        replace(&good, |m| m.velocities[6] += 0.1),
        replace(&good, |m| m.root_rotation_coordinates[0] = f64::MAX),
    ];
    let (mut fresh, _, _) = setup(0.12);
    let before = links.map(|i| fresh.state(i));
    for (case, data) in cases.into_iter().enumerate() {
        assert!(
            fresh.restore_floating_revolute(&data).is_err(),
            "case {case}"
        );
        assert_eq!(before, links.map(|i| fresh.state(i)));
        assert!(fresh.joints.construction_open() && fresh.joints.efforts.is_empty());
    }
    fresh.add_joint_effort(j, 0.375).unwrap();
    assert!(fresh.restore_floating_revolute(&good).is_err());
    assert_eq!(fresh.joints.efforts.get(&j), Some(&0.375));
    fresh.reset().unwrap();
    fresh.restore_floating_revolute(&good).unwrap();
    assert_eq!(fresh.checkpoint_floating_revolute().unwrap(), good);
    let (mut other, _, _) = setup(0.13);
    assert!(other.restore_floating_revolute(&good).is_err());
}
#[test]
fn external_inputs_prescribed_tiers_extra_geometry_and_other_native_formats_are_distinct() {
    let (mut src, links, j) = moving();
    let bytes = src.checkpoint_floating_revolute().unwrap();
    assert!(src.checkpoint_solver().is_err());
    assert!(RapierBackend::restore_trusted_solver(&bytes).is_err());
    let native_body = src.bodies[&links[0]].0;
    src.world.bodies[native_body].add_force(Vector::X, false);
    assert!(src.checkpoint_floating_revolute().is_err());
    src.world.bodies[native_body].reset_forces(false);
    src.set_joint_velocity_motor(
        j,
        Some(JointVelocityMotor {
            target_velocity: 0.4,
            max_force: 1.,
        }),
    )
    .unwrap();
    assert!(src.checkpoint_floating_revolute().is_err());
    src.set_joint_velocity_motor(j, None).unwrap();
    assert_eq!(src.checkpoint_floating_revolute().unwrap(), bytes);
    src.add_world_wrench(links[0], [0.3, 0., 0.], [0.; 3])
        .unwrap();
    assert!(src.checkpoint_floating_revolute().is_err());
    src.step(STEP).unwrap();
    assert!(src.checkpoint_floating_revolute().is_ok());
    src.step(Duration::from_millis(2)).unwrap();
    assert!(src.checkpoint_floating_revolute().is_err());
    let (mut cold, ids, _) = setup(0.12);
    assert!(cold.checkpoint_floating_revolute().is_err());
    cold.step(STEP).unwrap();
    cold.switch_articulation(ids[0], ArticulationTier::Prescribed, STEP)
        .unwrap();
    assert!(cold.checkpoint_floating_revolute().is_err());
    let extra = BodySpec {
        position: [4., 0., 0.],
        velocity: [0.; 3],
        mass: Some(1.),
        shape: Shape::Sphere { radius: 0.15 },
    };
    src.spawn_link(extra, Default::default(), None).unwrap();
    assert!(src.checkpoint_floating_revolute().is_err());
    assert!(src.joint_state(j).is_some());
}

#[test]
fn local_articulation_images_cannot_erase_a_negotiated_worker_identity() {
    let (mut free, _, _) = setup(0.12);
    free.step(STEP).unwrap();
    let valid = free.checkpoint_floating_revolute().unwrap();
    let (mut destination, _, _) = setup(0.12);
    destination.native_profile = Some(NativeSolverProfile::Extended64);
    assert!(destination.restore_floating_revolute(&valid).is_err());
    free.native_profile = Some(NativeSolverProfile::Standard32);
    assert!(free.checkpoint_floating_revolute().is_err());
}

#[test]
fn free_root_roundoff_in_a_legacy_cut_is_preserved_until_the_next_interval() {
    let (mut source, links, j) = moving();
    let Handle::Reduced(handle) = source.joints.records[&j].handle else {
        unreachable!()
    };
    // Reconstruct a previously legal, spinning root near the admission limit.
    // Regenerate every dependent pose from the unchanged joint coordinates.
    let root = source.bodies[&links[0]].0;
    let rb = &mut source.world.bodies[root];
    let q = *rb.rotation() * (1. + 4.5e-13);
    rb.set_rotation(q, false);
    let (tree, _) = source.world.multibody_joints.get_mut(handle).unwrap();
    tree.forward_kinematics(&source.world.bodies, true);
    for link in tree.links() {
        source.world.bodies[link.rigid_body_handle()].set_position(*link.local_to_world(), false);
    }
    source.joints.finish_velocities(&mut source.world, false);
    source.add_joint_effort(j, -0.25).unwrap();
    let old = source.checkpoint_floating_revolute().unwrap();
    let (mut restored, _, _) = setup(0.12);
    restored.restore_floating_revolute(&old).unwrap();
    assert_eq!(restored.checkpoint_floating_revolute().unwrap(), old);
    for tick in 0..128 {
        source.step(STEP).unwrap();
        restored.step(STEP).unwrap();
        assert_eq!(
            source.checkpoint_floating_revolute().unwrap(),
            restored.checkpoint_floating_revolute().unwrap()
        );
        for id in links {
            let body = &source.world.bodies[source.bodies[&id].0];
            let tol = if tick == 0 { 2e-15 } else { 5.1e-13 };
            assert!((body.rotation().length_squared() - 1.).abs() < tol);
            let collider = &source.world.colliders[body.colliders()[0]];
            assert_eq!(
                *collider.position(),
                body.position() * collider.position_wrt_parent().unwrap()
            );
        }
    }
}
