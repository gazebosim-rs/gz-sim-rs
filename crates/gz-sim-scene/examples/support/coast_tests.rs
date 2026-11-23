// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodySpec, JointVelocityMotor, PhysicsBackend, PhysicsError, RotationSpec, Shape};
use gz_sim_scene::{FidelityError, remove_model, reset_loaded_world, set_model_fidelity};

fn deferred(e: FidelityError) -> &'static str {
    match e {
        FidelityError::Physics(PhysicsError::TransitionDeferred { reason }) => reason,
        _ => panic!("unexpected refusal {e:?}"),
    }
}
#[test]
fn skew_hinge_requires_centrifugal_torque_even_with_an_aggregate_principal_spin() {
    // Only the hinge line moves. The rigid mass distribution and aggregate
    // principal axis are unchanged. At this speed, joint motion after the
    // initial kick is below 2.1e-13 rad/s.
    let skew = ARRAY.replace("-0.6 0 0.2 0 0 0", "-0.6 0.09 0.2 0 0 0");
    assert_ne!(skew, ARRAY);
    let mut sim = load(&skew).unwrap();
    let rig = Mechanism::bind(&sim, &skew, "satellite");
    rig.prepare(&mut sim, 1e-5, 0, |_, _| {}).unwrap();
    let initial = rig.sample(&sim);
    assert!(initial.joints.iter().all(|j| j.velocity.abs() < 2.1e-13));
    assert_eq!(
        deferred(begin_model_free_coast(&mut sim, rig.model).unwrap_err()),
        "passive hinge requires torque to hold its coast configuration"
    );
    assert_eq!(rig.sample(&sim), initial);
    assert_eq!(
        model_fidelity(&sim, rig.model).unwrap(),
        ArticulationTier::Dynamic
    );
    // Manually prescribing the same posture remains available to a caller
    // that intentionally supplies the constraint torque.
    set_model_fidelity(&mut sim, rig.model, ArticulationTier::Prescribed).unwrap();
}
#[test]
fn active_motors_moving_joints_and_pending_effort_defer_without_mutation() {
    let mut sim = load(ARRAY).unwrap();
    let rig = Mechanism::bind(&sim, ARRAY, "satellite");
    sim.step_exact(1).unwrap();
    let (id, j) = rig.joints[3];
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_servo(
            j,
            Some(JointVelocityMotor {
                target_velocity: 1.,
                max_force: 10.,
            }),
        )
        .unwrap();
    assert!(deferred(begin_model_free_coast(&mut sim, rig.model).unwrap_err()).contains("motor"));
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .set_joint_velocity_servo(j, None)
        .unwrap();
    queue_joint_effort(&mut sim, id, 12.).unwrap();
    let pre = rig.sample(&sim);
    assert!(deferred(begin_model_free_coast(&mut sim, rig.model).unwrap_err()).contains("effort"));
    assert_eq!(pre, rig.sample(&sim));
    sim.step_exact(1).unwrap();
    assert!(rig.sample(&sim).joints[3].velocity.abs() > 1e-3);
    assert!(
        deferred(begin_model_free_coast(&mut sim, rig.model).unwrap_err()).contains("rigid twist")
    );
}

#[test]
fn arrival_resumes_the_tree_before_collision_and_limit_resumes_on_its_stable_handle() {
    let bounded = ARRAY.replacen(
        "</axis>",
        "<limit><lower>-0.03</lower><upper>0.03</upper></limit></axis>",
        1,
    );
    let mut a = load(&bounded).unwrap();
    let mut reference = load(&bounded).unwrap();
    let rig = Mechanism::bind(&a, &bounded, "satellite");
    for s in [&mut a, &mut reference] {
        s.step_exact(1).unwrap();
    }
    begin_model_free_coast(&mut a, rig.model).unwrap();
    assert!(queue_joint_effort(&mut a, rig.joints[0].0, 10.).is_err());
    for s in [&mut a, &mut reference] {
        // A normal native insertion into the shared contact domain, already
        // moving toward the right panel.
        s.world_mut()
            .resource_mut::<PhysicsBackend>()
            .spawn_link(
                BodySpec {
                    position: [1.13, 0., 0.2],
                    velocity: [-2., 0., 0.],
                    shape: Shape::Sphere { radius: 0.09 },
                    mass: Some(0.8),
                },
                RotationSpec::default(),
                None,
            )
            .unwrap();
    }
    assert!(
        deferred(begin_model_free_coast(&mut a, rig.model).unwrap_err()).contains("disturbance")
    );
    for s in [&mut a, &mut reference] {
        s.step_exact(1).unwrap();
        assert_eq!(
            model_fidelity(s, rig.model).unwrap(),
            ArticulationTier::Dynamic
        );
        s.world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_joint_velocity_servo(
                rig.joints[0].1,
                Some(JointVelocityMotor {
                    target_velocity: 1.,
                    max_force: 20.,
                }),
            )
            .unwrap();
        s.step_exact(150).unwrap();
    }
    let current = rig.sample(&a);
    assert_eq!(current, rig.sample(&reference));
    assert!(current.joints[0].position > 0.025 && current.joints[0].position < 0.031);
    reset_loaded_world(&mut a, |_| ()).unwrap();
    assert_eq!(Mechanism::bind(&a, &bounded, "satellite"), rig);
    a.step_exact(1).unwrap();
    begin_model_free_coast(&mut a, rig.model).unwrap();
    remove_model(&mut a, rig.model).unwrap();
    a.step_exact(1).unwrap();
}

#[test]
fn full_spinning_branch_and_translating_humanoid_match_native_link_and_joint_trajectories() {
    for (name, xml, spin, brake) in [
        ("satellite", ARRAY.to_owned(), 1., 1000),
        ("humanoid", humanoid::Humanoid::new().world_sdf(""), 0., 0),
    ] {
        let mut coast = load(&xml).unwrap();
        let mut native = load(&xml).unwrap();
        let rig = Mechanism::bind(&coast, &xml, name);
        assert_eq!(rig, Mechanism::bind(&native, &xml, name));
        for s in [&mut coast, &mut native] {
            rig.prepare(s, spin, brake, |_, _| {}).unwrap();
        }
        let original = rig.sample(&coast);
        begin_model_free_coast(&mut coast, rig.model).unwrap();
        assert_eq!(original, rig.sample(&coast));
        let mut errors = [0_f64; 6];
        for i in 0..2200 {
            if i == 2000 {
                for s in [&mut coast, &mut native] {
                    queue_link_world_wrench(s, rig.links[0].0, [0., 80., 0.], [0.; 3]).unwrap();
                }
                coast.tick().unwrap();
                assert_eq!(
                    model_fidelity(&coast, rig.model).unwrap(),
                    ArticulationTier::Prescribed
                );
                assert!(
                    deferred(begin_model_free_coast(&mut coast, rig.model).unwrap_err())
                        .contains("disturbance")
                );
            }
            if i == 2001 {
                for s in [&mut coast, &mut native] {
                    queue_joint_effort(s, rig.joints.last().unwrap().0, 8.).unwrap();
                }
            }
            coast.step_exact(1).unwrap();
            native.step_exact(1).unwrap();
            let (a, b) = (rig.sample(&coast), rig.sample(&native));
            for (l, r) in a.links.iter().zip(b.links.iter()) {
                let v = DVec3::from_array;
                let (u, q) = (
                    DQuat::from_array(l.orientation),
                    DQuat::from_array(r.orientation),
                );
                for (k, diff) in [
                    v(l.position).distance(v(r.position)),
                    (u - q * u.dot(q).signum()).length(),
                    v(l.velocity).distance(v(r.velocity)),
                    v(l.angular_velocity).distance(v(r.angular_velocity)),
                ]
                .into_iter()
                .enumerate()
                {
                    errors[k] = errors[k].max(diff);
                }
            }
            for (l, r) in a.joints.iter().zip(&b.joints) {
                errors[4] = errors[4].max((l.position - r.position).abs());
                errors[5] = errors[5].max((l.velocity - r.velocity).abs());
            }
        }
        eprintln!(
            "{name} trajectory max |position, quaternion, COM velocity, angular velocity, q, qdot| deltas: {errors:?}"
        );
        assert!(errors.into_iter().all(|e| e < 1e-9));
        assert_eq!(
            model_fidelity(&coast, rig.model).unwrap(),
            ArticulationTier::Dynamic
        );
        assert_eq!(rig, Mechanism::bind(&coast, &xml, name));
    }
}

#[test]
fn only_committed_isolated_weightless_dynamics_can_begin_and_reset_cancels_coast() {
    let xml = ARRAY.replace("<gravity>0 0 0", "<gravity>0 0 -9.81");
    let mut falling = load(&xml).unwrap();
    let rig = Mechanism::bind(&falling, &xml, "satellite");
    falling.step_exact(1).unwrap();
    let before = rig.sample(&falling);
    assert!(
        deferred(begin_model_free_coast(&mut falling, rig.model).unwrap_err())
            .contains("weightless")
    );
    assert_eq!(rig.sample(&falling), before);

    let mut sim = load(ARRAY).unwrap();
    let rig = Mechanism::bind(&sim, ARRAY, "satellite");
    assert!(
        deferred(begin_model_free_coast(&mut sim, rig.model).unwrap_err()).contains("committed")
    );
    sim.step_exact(1).unwrap();
    set_model_fidelity(&mut sim, rig.model, ArticulationTier::Prescribed).unwrap();
    assert!(deferred(begin_model_free_coast(&mut sim, rig.model).unwrap_err()).contains("dynamic"));
    set_model_fidelity(&mut sim, rig.model, ArticulationTier::Dynamic).unwrap();
    begin_model_free_coast(&mut sim, rig.model).unwrap();
    begin_model_free_coast(&mut sim, rig.model).unwrap();
    queue_link_world_wrench(&mut sim, rig.links[0].0, [800., 0., 0.], [0.; 3]).unwrap();
    sim.reset().unwrap();
    sim.step_exact(1).unwrap();
    let initial = rig.sample(&sim);
    assert_eq!(
        model_fidelity(&sim, rig.model).unwrap(),
        ArticulationTier::Dynamic
    );
    assert_eq!(initial.momentum, DVec3::ZERO);
    begin_model_free_coast(&mut sim, rig.model).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(
        model_fidelity(&sim, rig.model).unwrap(),
        ArticulationTier::Dynamic
    );
}
