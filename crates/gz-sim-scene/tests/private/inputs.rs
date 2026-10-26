// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::{Name, PhysicsEngine, Velocity, load_world_with_sdf_step};
use gz_sim_core::{EntityId, StepError};
use sdformat::NoResources;
use std::sync::Arc;

fn scene(engine: PhysicsEngine) -> (Simulation, Entity, Entity, u64) {
    let sim = load_world_with_sdf_step("<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><model name='robot'><link name='body'><collision name='shape'><geometry><sphere><radius>1</radius></sphere></geometry></collision></link></model></world></sdf>","input-boundaries","default",&NoResources,engine).unwrap();
    let named = |name| {
        sim.world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap()
            .id()
    };
    let model = named("default::robot");
    let link = named("default::robot::body");
    let id = sim.world().get::<EntityId>(link).unwrap().get();
    (sim, model, link, id)
}

#[test]
fn admission_rejects_broken_bindings_before_inserting_any_pending_input() {
    type Mutation = fn(&mut World, Entity, Entity);
    let common: &[Mutation] = &[
        |w, _, _| {
            w.remove_resource::<InputsInstalled>();
        },
        |w, _, l| {
            w.entity_mut(l).remove::<ChildOf>();
        },
        |w, m, _| {
            w.entity_mut(m).remove::<EntityKind>();
        },
        |w, m, _| {
            w.entity_mut(m).remove::<ChildOf>();
        },
        |w, _, l| {
            w.entity_mut(l).insert(EntityKind::Visual);
        },
        |w, m, _| {
            w.spawn((EntityKind::Link, ChildOf(m)));
        },
    ];
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        for mutate in common {
            let (mut sim, model, link, id) = scene(engine);
            mutate(sim.world_mut(), model, link);
            assert!(matches!(
                queue_link_world_velocity(&mut sim, id, [1.0; 3], [0.0; 3]),
                Err(InputError::InvalidPhysics | InputError::NotFreeLink(_))
            ));
            assert!(sim.world().get::<PendingInput>(link).is_none());
        }
    }
    let rapier: &[Mutation] = &[
        |w, _, _| {
            w.remove_resource::<RapierBackend>();
        },
        |w, _, _| {
            w.insert_resource(tpe::TpeWorld(Arc::new(())));
        },
        |w, _, l| {
            w.entity_mut(l).remove::<BodyHandle>();
        },
        |w, _, l| {
            let id = w.get::<BodyHandle>(l).unwrap().id();
            assert!(w.resource_mut::<RapierBackend>().remove(id));
        },
    ];
    for mutate in rapier {
        let (mut sim, model, link, id) = scene(PhysicsEngine::Rapier);
        mutate(sim.world_mut(), model, link);
        assert!(matches!(
            queue_link_world_velocity(&mut sim, id, [1.0; 3], [0.0; 3]),
            Err(InputError::InvalidPhysics)
        ));
        assert!(sim.world().get::<PendingInput>(link).is_none());
    }
    let tpe: &[Mutation] = &[
        |w, m, _| {
            w.entity_mut(m).remove::<ModelBounds>();
        },
        |w, _, l| {
            w.entity_mut(l).remove::<LocalPose>();
        },
        |w, m, _| {
            w.entity_mut(m).remove::<WorldPose>();
        },
        |w, m, _| {
            w.entity_mut(m).remove::<Motion>();
        },
        |w, _, l| {
            w.entity_mut(l).remove::<tpe::TpeLink>();
        },
    ];
    for mutate in tpe {
        let (mut sim, model, link, id) = scene(PhysicsEngine::Tpe);
        mutate(sim.world_mut(), model, link);
        assert!(matches!(
            queue_link_world_velocity(&mut sim, id, [1.0; 3], [0.0; 3]),
            Err(InputError::InvalidPhysics)
        ));
        assert!(sim.world().get::<PendingInput>(link).is_none());
    }
}

#[test]
fn capabilities_and_accumulation_are_explicit_and_atomic() {
    for change in [0, 1, 2] {
        let (mut sim, model, link, id) = scene(PhysicsEngine::Tpe);
        match change {
            0 => {
                sim.world_mut()
                    .entity_mut(model)
                    .insert(ModelBounds::new([-1.0; 3], [1.0; 3], true, u16::MAX).unwrap());
            }
            1 => {
                sim.world_mut()
                    .get_mut::<LocalPose>(link)
                    .unwrap()
                    .0
                    .rotation = glam::DQuat::from_rotation_z(1.0).to_array()
            }
            _ => {
                sim.world_mut()
                    .get_mut::<WorldPose>(model)
                    .unwrap()
                    .rotation = glam::DQuat::from_rotation_z(1.0).to_array()
            }
        }
        assert!(matches!(
            queue_link_world_wrench(&mut sim, id, [1.0; 3], [0.0; 3]),
            Err(InputError::Unsupported(_))
        ));
        assert!(sim.world().get::<PendingInput>(link).is_none());
    }
    let (mut sim, model, link, id) = scene(PhysicsEngine::Rapier);
    let model_id = sim.world().get::<EntityId>(model).unwrap().get();
    assert!(matches!(
        queue_link_world_velocity(&mut sim, model_id, [1.0; 3], [0.0; 3]),
        Err(InputError::NotFreeLink(_))
    ));
    assert!(matches!(
        queue_link_world_velocity(&mut sim, u64::MAX, [1.0; 3], [0.0; 3]),
        Err(InputError::MissingEntity(_))
    ));
    for (force, torque) in [([1e308, 0.0, 0.0], [0.0; 3]), ([0.0; 3], [0.0, 0.0, 1e308])] {
        queue_link_world_wrench(&mut sim, id, force, torque).unwrap();
        assert!(matches!(
            queue_link_world_wrench(&mut sim, id, force, torque),
            Err(InputError::InvalidValue)
        ));
        queue_link_world_wrench(&mut sim, id, force.map(|v| -v), torque.map(|v| -v)).unwrap();
    }
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().get::<Velocity>(link).unwrap().linear, [0.0; 3]);
    assert_eq!(sim.world().get::<Velocity>(link).unwrap().angular, [0.0; 3]);
}

#[test]
fn broken_admitted_inputs_fault_before_step_commit_and_reject_further_admission() {
    for corruption in [0, 1, 2, 3, 4] {
        let engine = if corruption >= 3 {
            PhysicsEngine::Tpe
        } else {
            PhysicsEngine::Rapier
        };
        let (mut sim, _, link, id) = scene(engine);
        queue_link_world_velocity(&mut sim, id, [1.0; 3], [0.0; 3]).unwrap();
        match corruption {
            0 => {
                let body = sim.world().get::<BodyHandle>(link).unwrap().id();
                sim.world_mut().resource_mut::<RapierBackend>().remove(body);
            }
            1 => {
                sim.world_mut().entity_mut(link).remove::<EntityId>();
            }
            2 => {
                sim.world_mut()
                    .get_mut::<PendingInput>(link)
                    .unwrap()
                    .velocity = Some(([f64::NAN; 3], [0.0; 3]))
            }
            3 => {
                sim.world_mut()
                    .get_mut::<PendingInput>(link)
                    .unwrap()
                    .wrench = Some(([1.0; 3], [0.0; 3]));
            }
            _ => {
                sim.world_mut()
                    .get_mut::<PendingInput>(link)
                    .unwrap()
                    .velocity = Some(([f64::NAN; 3], [0.0; 3]));
            }
        }
        assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
        assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
        assert!(sim.fault().unwrap().downcast_ref::<InputError>().is_some());
        assert!(matches!(
            queue_link_world_velocity(&mut sim, id, [0.0; 3], [0.0; 3]),
            Err(InputError::Clock(StepError::Faulted))
        ));
        assert!(matches!(
            queue_link_world_wrench(&mut sim, id, [0.0; 3], [0.0; 3]),
            Err(InputError::Clock(StepError::Faulted))
        ));
    }
}

#[test]
fn missing_and_foreign_loaded_owners_cannot_admit_or_execute_inputs() {
    assert!(!crate::initial::matches_physics_owner(&World::new()));
    for admitted in [false, true] {
        let (mut sim, _, _, id) = scene(PhysicsEngine::Tpe);
        if admitted {
            queue_link_world_velocity(&mut sim, id, [1.0; 3], [0.0; 3]).unwrap();
        }
        sim.world_mut().insert_resource(tpe::TpeWorld(Arc::new(())));
        assert!(matches!(
            queue_link_world_velocity(&mut sim, id, [1.0; 3], [0.0; 3]),
            Err(InputError::InvalidPhysics)
        ));
        if admitted {
            assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
            assert_eq!(sim.elapsed(), std::time::Duration::ZERO);
        }
    }
}

#[test]
fn failed_velocity_execution_preserves_attached_imu_history() {
    for engine in [PhysicsEngine::Rapier, PhysicsEngine::Tpe] {
        let mut sim = load_world_with_sdf_step(
            include_str!("../fixtures/headless_imu.sdf"),
            "failed-input-imu",
            "default",
            &NoResources,
            engine,
        )
        .unwrap();
        let link = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|n| n.0 == "default::ball::body")
            })
            .unwrap()
            .id();
        let id = sim.world().get::<EntityId>(link).unwrap().get();
        sim.step_exact(1).unwrap();
        let history = sim
            .world()
            .iter_entities()
            .find_map(|e| e.get::<crate::Imu>())
            .unwrap()
            .previous_velocity;
        queue_link_world_velocity(&mut sim, id, [1.0, 0.0, 0.0], [0.0; 3]).unwrap();
        sim.world_mut().entity_mut(link).remove::<ChildOf>();
        assert_eq!(sim.step_exact(1), Err(StepError::Faulted));
        assert_eq!(
            sim.world()
                .iter_entities()
                .find_map(|e| e.get::<crate::Imu>())
                .unwrap()
                .previous_velocity,
            history
        );
        assert_eq!(sim.elapsed(), std::time::Duration::from_millis(1));
    }
}
