// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodyId, BodySpec, BodyState, JointId, JointState, JointVelocityMotor, Shape};

struct Construction {
    sim: Simulation,
    model: Model,
    owner: Entity,
    plan: Plan,
    backend: PhysicsBackend,
    joints: Vec<(Entity, JointId)>,
    bodies: Vec<BodyId>,
}

fn construction() -> Construction {
    let mut sim = load(XML);
    let model = sdformat::load_str(XML, "m9.sdf", &sdformat::NoResources)
        .unwrap()
        .worlds
        .remove(0)
        .models
        .into_iter()
        .find(|model| model.name == "arm")
        .unwrap();
    let plan = parse(&model).unwrap().unwrap();
    let owner = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == model.scoped_name)
                && entity.get::<EntityKind>() == Some(&EntityKind::Model)
        })
        .unwrap()
        .id();
    let joints: Vec<_> = controllers(&sim)
        .into_iter()
        .map(|(entity, _)| (entity, sim.world().get::<JointHandle>(entity).unwrap().id()))
        .collect();
    let bodies = sim
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<BodyHandle>().map(BodyHandle::id))
        .collect();
    for (entity, _) in &joints {
        sim.world_mut().entity_mut(*entity).remove::<Controller>();
    }
    // Constructor preflight receives the same backend borrowed by the real loader,
    // outside its World resource slot; it must never step or replace native targets.
    let mut backend = sim.world_mut().remove_resource::<PhysicsBackend>().unwrap();
    for (_, joint) in &joints {
        backend
            .set_joint_velocity_motor(
                *joint,
                Some(JointVelocityMotor {
                    target_velocity: 1.0,
                    max_force: 0.2,
                }),
            )
            .unwrap();
    }
    let root = backend.joint_child(joints[0].1).unwrap();
    backend.add_joint_effort(joints[0].1, 0.1).unwrap();
    backend
        .add_world_wrench(root, [0.0; 3], [0.0, 0.0, 0.2])
        .unwrap();
    Construction {
        sim,
        model,
        owner,
        plan,
        backend,
        joints,
        bodies,
    }
}

fn native(case: &Construction) -> (Vec<Option<BodyState>>, Vec<Option<JointState>>) {
    (
        case.bodies
            .iter()
            .map(|id| case.backend.state(*id))
            .collect(),
        case.joints
            .iter()
            .map(|(_, id)| case.backend.joint_state(*id))
            .collect(),
    )
}

fn identities(case: &Construction) -> Vec<(Entity, u64)> {
    let mut identities: Vec<_> = case
        .sim
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<EntityId>().map(|id| (entity.id(), id.get())))
        .collect();
    identities.sort_by_key(|(_, id)| *id);
    identities
}

fn publications(case: &Construction) -> Vec<(Entity, JointControllerConfiguration)> {
    case.sim
        .world()
        .iter_entities()
        .filter_map(|entity| {
            entity
                .get::<JointControllerConfiguration>()
                .map(|value| (entity.id(), value.clone()))
        })
        .collect()
}

fn allocation_and_pending_work_match(case: &mut Construction, control: &mut Construction) {
    // Observe both native allocators after construction, rather than inspecting
    // private backend counters or assuming an ID on every ECS entity.
    let spec = BodySpec {
        shape: Shape::Sphere { radius: 0.1 },
        position: [100.0, 0.0, 1.0],
        velocity: [0.0; 3],
        mass: Some(1.0),
    };
    let body = case
        .backend
        .spawn_link(spec, gz_physics::RotationSpec::default(), None)
        .unwrap();
    let reference = control
        .backend
        .spawn_link(spec, gz_physics::RotationSpec::default(), None)
        .unwrap();
    assert_eq!(body, reference);
    let joint = case
        .backend
        .add_reduced_revolute(None, body, spec.position, [0.0, 0.0, 1.0])
        .unwrap();
    let reference_joint = control
        .backend
        .add_reduced_revolute(None, reference, spec.position, [0.0, 0.0, 1.0])
        .unwrap();
    assert_eq!(joint, reference_joint);
    // The control retains the same native motor modes and queued torque/wrench,
    // but never calls build. Equality after real stepping detects consumed work
    // or an early native target write, including after first-controller preflight.
    let before = native(case);
    case.backend.step(Duration::from_millis(1)).unwrap();
    control.backend.step(Duration::from_millis(1)).unwrap();
    assert_eq!(native(case), native(control));
    assert_ne!(
        native(case),
        before,
        "the queued work/native motors must actually advance physics"
    );
    case.backend.step(Duration::from_millis(1)).unwrap();
    control.backend.step(Duration::from_millis(1)).unwrap();
    assert_eq!(native(case), native(control));
}

#[derive(Clone, Copy)]
enum Damage {
    MissingName,
    Parent,
    Kind,
    Handle,
    NativeJoint,
    InitialTarget,
    Speed,
    Torque,
}

fn damage(case: &mut Construction, damage: Damage) {
    let (second, joint) = case.joints[1];
    match damage {
        Damage::MissingName => case.plan.0[1].joint = "missing".into(),
        Damage::Parent => {
            case.sim
                .world_mut()
                .entity_mut(second)
                .insert(ChildOf(case.joints[0].0));
        }
        Damage::Kind => {
            case.sim
                .world_mut()
                .entity_mut(second)
                .insert(EntityKind::Link);
        }
        Damage::Handle => {
            case.sim
                .world_mut()
                .entity_mut(second)
                .remove::<JointHandle>();
        }
        Damage::NativeJoint => {
            assert!(case.backend.remove_joint(joint));
        }
        Damage::InitialTarget => case.plan.0[1].initial_target = f64::NAN,
        Damage::Speed => case.plan.0[1].max_velocity = 0.0,
        Damage::Torque => case.plan.0[1].max_force = f64::NAN,
    }
}

#[test]
fn second_controller_refusal_preserves_constructor_atomicity_and_native_work() {
    for damaged in [
        Damage::MissingName,
        Damage::Parent,
        Damage::Kind,
        Damage::Handle,
        Damage::NativeJoint,
        Damage::InitialTarget,
        Damage::Speed,
        Damage::Torque,
    ] {
        let mut case = construction();
        let mut control = construction();
        // The first controller is valid and observably different from the retained
        // native velocity mode; every failure belongs to the second controller.
        case.plan.0[0].initial_target = 0.7;
        damage(&mut case, damaged);
        damage(&mut control, damaged);
        let before = native(&case);
        let ids = identities(&case);
        let metadata = publications(&case);
        let error = build(
            &mut case.sim,
            &case.model,
            case.owner,
            &case.backend,
            &case.plan,
        )
        .unwrap_err();
        let expected = match damaged {
            Damage::MissingName => Some(("default::arm::missing", "missing controller joint")),
            Damage::Parent | Damage::Kind => {
                Some(("default::arm::tip_hinge", "controller joint ownership"))
            }
            Damage::Handle => Some((
                "default::arm::tip_hinge",
                "missing controller joint binding",
            )),
            Damage::NativeJoint => {
                Some(("default::arm::tip_hinge", "missing controller joint state"))
            }
            Damage::InitialTarget | Damage::Speed => Some((
                "default::arm::tip_hinge",
                "unrepresentable controller initialization",
            )),
            Damage::Torque => None,
        };
        if let Some((name, expected)) = expected {
            assert!(matches!(error, LoadError::Unsupported { entity, feature }
                if entity == name && feature == expected));
        } else {
            assert!(matches!(
                error,
                LoadError::Physics(PhysicsError::InvalidValue)
            ));
        }
        assert!(
            controllers(&case.sim).is_empty(),
            "no first-controller partial commit"
        );
        assert_eq!(native(&case), before);
        assert_eq!(identities(&case), ids);
        assert_eq!(publications(&case), metadata);
        assert_eq!(case.sim.elapsed(), Duration::ZERO);
        allocation_and_pending_work_match(&mut case, &mut control);
    }
}

#[test]
fn successful_constructor_commits_both_pending_targets_without_native_mutation() {
    let mut case = construction();
    let mut control = construction();
    case.plan.0[0].initial_target = 0.7;
    case.plan.0[1].initial_target = -0.4;
    let before = native(&case);
    let ids = identities(&case);
    let metadata = publications(&case);
    build(
        &mut case.sim,
        &case.model,
        case.owner,
        &case.backend,
        &case.plan,
    )
    .unwrap();
    assert_eq!(controllers(&case.sim).len(), 2);
    for ((entity, _), target) in case.joints.iter().zip([0.7, -0.4]) {
        let controller = case.sim.world().get::<Controller>(*entity).unwrap();
        assert_eq!(controller.target, target);
        assert_eq!(controller.initial_target, target);
        assert_eq!(
            case.sim
                .world()
                .get::<JointControllerConfiguration>(*entity),
            Some(&controller.publication)
        );
    }
    assert_eq!(native(&case), before);
    assert_eq!(identities(&case), ids);
    assert_eq!(publications(&case), metadata);
    allocation_and_pending_work_match(&mut case, &mut control);
}
