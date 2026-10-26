// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use crate::{
    RemoveError, SceneResetError, load_world_with_engine, remove_model, reset_loaded_world,
};
use gz_sim_core::EntityId;

fn loaded(engine: PhysicsEngine) -> Simulation {
    load_world_with_engine(
        include_str!("../fixtures/fall.sdf"),
        "tpe.sdf",
        "default",
        Duration::from_millis(1),
        &sdformat::NoResources,
        engine,
    )
    .unwrap()
}

fn ball(sim: &Simulation) -> (Entity, u64) {
    let entity = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == "default::ball"))
        .unwrap();
    (entity.id(), entity.get::<EntityId>().unwrap().get())
}

#[test]
fn tpe_reset_preflights_owner_presence_and_runtime_bindings() {
    let mut sim = loaded(PhysicsEngine::Tpe);
    let (entity, id) = ball(&sim);
    sim.step_exact(2).unwrap();
    let owner = sim.world_mut().remove_resource::<TpeWorld>().unwrap();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::MissingPhysics)
    ));
    assert_eq!(remove_model(&mut sim, id), Err(RemoveError::InvalidPhysics));
    sim.world_mut().insert_resource(TpeWorld(Arc::new(())));
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::ForeignPhysics)
    ));
    sim.world_mut().insert_resource(owner);
    sim.world_mut()
        .insert_resource(RapierBackend::new([0.0; 3]).unwrap());
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::ForeignPhysics)
    ));
    assert_eq!(remove_model(&mut sim, id), Err(RemoveError::InvalidPhysics));
    sim.world_mut().remove_resource::<RapierBackend>();
    let link = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<TpeLink>().cloned())
        .unwrap();
    let extra = sim.world_mut().spawn(link).id();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::RuntimePhysics)
    ));
    sim.world_mut().entity_mut(extra).despawn();
    let motion = sim.world().get::<Motion>(entity).unwrap().clone();
    let extra = sim.world_mut().spawn(motion).id();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::RuntimePhysics)
    ));
    sim.world_mut().entity_mut(extra).despawn();
    let bounds = sim
        .world()
        .get::<gz_physics_tpe::ModelBounds>(entity)
        .unwrap()
        .clone();
    let extra = sim.world_mut().spawn(bounds).id();
    assert!(matches!(
        reset_loaded_world(&mut sim, |_| ()),
        Err(SceneResetError::RuntimePhysics)
    ));
    sim.world_mut().entity_mut(extra).despawn();
    assert_eq!(sim.elapsed(), Duration::from_millis(2));
    assert_eq!(ball(&sim), (entity, id));
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(sim.elapsed(), Duration::ZERO);
    let mut rapier = loaded(PhysicsEngine::Rapier);
    rapier.world_mut().insert_resource(TpeWorld(Arc::new(())));
    assert!(matches!(
        reset_loaded_world(&mut rapier, |_| ()),
        Err(SceneResetError::ForeignPhysics)
    ));
}

#[test]
fn tpe_removal_rejects_corrupt_bindings_without_retiring_ids() {
    let mut sim = loaded(PhysicsEngine::Tpe);
    let (entity, id) = ball(&sim);
    let motion = sim.world_mut().entity_mut(entity).take::<Motion>().unwrap();
    assert_eq!(remove_model(&mut sim, id), Err(RemoveError::InvalidPhysics));
    sim.world_mut().entity_mut(entity).insert(motion);
    let link = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<Name>()
                .is_some_and(|n| n.0 == "default::ball::body")
        })
        .unwrap()
        .id();
    let binding = sim.world_mut().entity_mut(link).take::<TpeLink>().unwrap();
    assert_eq!(remove_model(&mut sim, id), Err(RemoveError::InvalidPhysics));
    assert_eq!(ball(&sim), (entity, id));
    sim.world_mut().entity_mut(link).insert(binding);
    assert!(remove_model(&mut sim, id).unwrap().contains(&id));
}

#[test]
fn tpe_construction_rejects_invalid_shapes_and_numeric_poses() {
    for shape in [
        Shape::Sphere { radius: 0.0 },
        Shape::Sphere {
            radius: f64::INFINITY,
        },
        Shape::Box {
            half_extents: [1.0, -1.0, 1.0],
        },
    ] {
        assert!(matches!(
            validate_link(
                BodySpec {
                    shape,
                    position: [0.0; 3],
                    velocity: [0.0; 3],
                    mass: None
                },
                Transform::default()
            ),
            Err(LoadError::Physics(PhysicsError::InvalidValue))
        ));
    }
    let mut pose = Transform::default();
    pose.translation[0] = f64::NAN;
    assert!(matches!(
        validate_link(
            BodySpec {
                shape: Shape::Sphere { radius: 1.0 },
                position: [0.0; 3],
                velocity: [0.0; 3],
                mass: None
            },
            pose
        ),
        Err(LoadError::Tpe(_))
    ));
    let mut sim = Simulation::new(Duration::from_millis(1)).unwrap();
    let entity = sim.world_mut().spawn_empty().id();
    assert!(matches!(
        add_model(&mut sim, entity, pose, false),
        Err(LoadError::Tpe(_))
    ));
    install(&mut sim).unwrap();
    assert!(matches!(install(&mut sim), Err(LoadError::TpeInstall(_))));
}

#[test]
fn cylinder_tpe_bounds_reject_invalid_dimensions() {
    for invalid in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        for shape in [
            Shape::Cylinder {
                radius: invalid,
                half_height: 1.4,
            },
            Shape::Cylinder {
                radius: 0.6,
                half_height: invalid,
            },
        ] {
            assert!(matches!(
                validate_link(
                    BodySpec {
                        shape,
                        position: [0.0; 3],
                        velocity: [0.0; 3],
                        mass: None
                    },
                    Transform::default()
                ),
                Err(LoadError::Physics(PhysicsError::InvalidValue))
            ));
        }
    }
}
