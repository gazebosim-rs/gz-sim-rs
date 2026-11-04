// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics::{BodySpec, BodyState};
use gz_physics_rapier::RapierBackend;
use gz_sim_core::next_entity_id;
use sdformat::NoResources;
use std::{collections::BTreeMap, sync::Arc, time::Duration};

struct Case {
    sim: Simulation,
    world: Entity,
    model: Entity,
    link: Entity,
    collision: Entity,
    visual: Entity,
    other_link: Entity,
    model_id: u64,
    identities: Vec<u64>,
    bodies: [BodyId; 2],
    parked: Option<PhysicsBackend>,
}

fn scene(fixed: bool) -> Case {
    let model = |name: &str, x: u8, geometry: &str| {
        format!(
            "<model name='{name}'><static>{fixed}</static><pose>{x} 0 0 0 0 0</pose><link name='body'><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial><collision name='c'><geometry>{geometry}</geometry></collision><visual name='v'><pose>0.25 0 0 0 0 0</pose><geometry>{geometry}</geometry></visual></link></model>"
        )
    };
    let sim = load_world_with_sdf_step(
        &format!(
            "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity>{}{}</world></sdf>",
            model("a", 0, "<sphere><radius>0.5</radius></sphere>"),
            model("b", 10, "<box><size>1 1 1</size></box>"),
        ),
        "world-edit-boundaries.sdf",
        "w",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap();
    let named = |name: &str| {
        sim.world()
            .iter_entities()
            .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == name))
            .unwrap()
            .id()
    };
    let world = named("w");
    let model = named("w::a");
    let link = named("w::a::body");
    let collision = named("w::a::body::c");
    let visual = named("w::a::body::v");
    let other_link = named("w::b::body");
    let model_id = sim.world().get::<EntityId>(model).unwrap().get();
    let bodies =
        [link, other_link].map(|entity| sim.world().get::<BodyHandle>(entity).unwrap().id());
    let identities = sim
        .world()
        .iter_entities()
        .filter_map(|entity| entity.get::<EntityId>().map(EntityId::get))
        .collect();
    Case {
        sim,
        world,
        model,
        link,
        collision,
        visual,
        other_link,
        model_id,
        identities,
        bodies,
        parked: None,
    }
}

fn advance(case: &mut Case) {
    for (body, velocity) in case
        .bodies
        .into_iter()
        .zip([[0.2, 0.0, 0.0], [-0.1, 0.0, 0.0]])
    {
        case.sim
            .world_mut()
            .resource_mut::<PhysicsBackend>()
            .set_world_velocity(body, velocity, [0.0; 3])
            .unwrap();
    }
    case.sim.step_exact(2).unwrap();
    assert!(case.sim.elapsed() > Duration::ZERO);
}

#[derive(Debug, PartialEq)]
struct Snapshot {
    time: (Duration, u64, bool, bool),
    next_public_id: u64,
    identities: Vec<(u64, Option<Entity>)>,
    entities: BTreeMap<Entity, String>,
    native: Option<[Option<BodyState>; 2]>,
    parked: Option<[Option<BodyState>; 2]>,
}

fn snapshot(case: &Case) -> Snapshot {
    let world = case.sim.world();
    Snapshot {
        time: (
            case.sim.elapsed(),
            case.sim.epoch(),
            case.sim.is_paused(),
            case.sim.is_faulted(),
        ),
        next_public_id: next_entity_id(world).unwrap(),
        identities: case
            .identities
            .iter()
            .map(|&id| (id, entity_by_id(world, id)))
            .collect(),
        entities: world
            .iter_entities()
            .map(|entity| {
                let hierarchy = (
                    entity.get::<Name>().map(|name| &name.0),
                    entity.get::<EntityKind>(),
                    entity.get::<EntityId>().map(EntityId::get),
                    entity.get::<ChildOf>().map(ChildOf::parent),
                    entity
                        .get::<Children>()
                        .map(|children| children.iter().collect::<Vec<_>>()),
                );
                let state = (
                    entity.get::<LocalPose>(),
                    entity.get::<WorldPose>(),
                    entity.get::<Velocity>(),
                    entity.get::<BodyHandle>().map(BodyHandle::id),
                    entity
                        .get::<ModelAnchor>()
                        .map(|anchor| (anchor.link, anchor.link_to_model)),
                    entity.get::<LinkCenterOfMass>(),
                );
                (
                    entity.id(),
                    format!(
                        "{hierarchy:?}/{state:?}/{:?}/{:?}",
                        entity.contains::<AttachedPose>(),
                        entity.get::<VisualGeometry>()
                    ),
                )
            })
            .collect(),
        native: world
            .get_resource::<PhysicsBackend>()
            .map(|backend| case.bodies.map(|body| backend.state(body))),
        parked: case
            .parked
            .as_ref()
            .map(|backend| case.bodies.map(|body| backend.state(body))),
    }
}

#[derive(Clone, Copy, Debug)]
enum Refusal {
    Unsupported,
    InvalidScene,
}

fn expected(error: WorldEditError, refusal: Refusal, label: &str) {
    assert!(
        matches!(
            (refusal, &error),
            (Refusal::Unsupported, WorldEditError::UnsupportedModel)
                | (Refusal::InvalidScene, WorldEditError::InvalidScene)
        ),
        "{label}: {error:?}"
    );
}

fn request() -> ModelState {
    ModelState {
        translation: [4.0, 5.0, 6.0],
        rotation: [0.0, 0.0, 1.0, 0.0],
        linear_velocity: [1.0, 2.0, 3.0],
        angular_velocity: [0.0, 0.0, 2.0],
    }
}

fn refused_without_commit(case: &mut Case, refusal: Refusal, label: &str) {
    let before = snapshot(case);
    expected(
        model_state(&case.sim, case.model_id).unwrap_err(),
        refusal,
        label,
    );
    assert_eq!(snapshot(case), before, "read mutated {label}");
    expected(
        set_model_state(&mut case.sim, case.model_id, request()).unwrap_err(),
        refusal,
        label,
    );
    assert_eq!(snapshot(case), before, "edit mutated {label}");
}

#[test]
fn hierarchy_identity_frame_and_required_component_drift_refuse_without_a_partial_commit() {
    type Mutation = fn(&mut Case);
    use Refusal::{InvalidScene, Unsupported};
    let mutations: &[(&str, Refusal, Mutation)] = &[
        ("missing model parent", Unsupported, |c| {
            c.sim.world_mut().entity_mut(c.model).remove::<ChildOf>();
        }),
        ("model kind", Unsupported, |c| {
            c.sim
                .world_mut()
                .entity_mut(c.model)
                .insert(EntityKind::Link);
        }),
        ("world kind", Unsupported, |c| {
            c.sim
                .world_mut()
                .entity_mut(c.world)
                .insert(EntityKind::Model);
        }),
        ("reparented link", Unsupported, |c| {
            c.sim
                .world_mut()
                .entity_mut(c.link)
                .insert(ChildOf(c.world));
        }),
        ("nested model", Unsupported, |c| {
            c.sim
                .world_mut()
                .spawn((EntityKind::Model, ChildOf(c.model)));
        }),
        ("link kind", Unsupported, |c| {
            c.sim
                .world_mut()
                .entity_mut(c.link)
                .insert(EntityKind::Sensor);
        }),
        ("missing local link pose", Unsupported, |c| {
            c.sim.world_mut().entity_mut(c.link).remove::<LocalPose>();
        }),
        ("link frame translation", Unsupported, |c| {
            c.sim
                .world_mut()
                .get_mut::<LocalPose>(c.link)
                .unwrap()
                .0
                .translation[0] = 0.25;
        }),
        ("link frame orientation", Unsupported, |c| {
            c.sim
                .world_mut()
                .get_mut::<LocalPose>(c.link)
                .unwrap()
                .0
                .rotation = [0.0, 0.0, 1.0, 0.0];
        }),
        ("wrong anchor identity", Unsupported, |c| {
            let id = c.sim.world().get::<EntityId>(c.other_link).unwrap().get();
            c.sim
                .world_mut()
                .get_mut::<ModelAnchor>(c.model)
                .unwrap()
                .link = id;
        }),
        ("anchor translation", Unsupported, |c| {
            c.sim
                .world_mut()
                .get_mut::<ModelAnchor>(c.model)
                .unwrap()
                .link_to_model
                .translation[1] = 0.25;
        }),
        ("anchor rotation", Unsupported, |c| {
            c.sim
                .world_mut()
                .get_mut::<ModelAnchor>(c.model)
                .unwrap()
                .link_to_model
                .rotation = [0.0, 1.0, 0.0, 0.0];
        }),
        ("sensor descendant", Unsupported, |c| {
            c.sim
                .world_mut()
                .spawn((EntityKind::Sensor, ChildOf(c.link)));
        }),
        ("duplicate collision kind", Unsupported, |c| {
            c.sim
                .world_mut()
                .entity_mut(c.visual)
                .insert(EntityKind::Collision);
        }),
        ("missing visual geometry", Unsupported, |c| {
            c.sim
                .world_mut()
                .entity_mut(c.visual)
                .remove::<VisualGeometry>();
        }),
        ("unsupported visual shape", Unsupported, |c| {
            c.sim
                .world_mut()
                .entity_mut(c.visual)
                .insert(VisualGeometry(Arc::new(Geometry::Cylinder {
                    radius: 0.5,
                    length: 1.0,
                })));
        }),
        ("grandchild hierarchy", Unsupported, |c| {
            c.sim
                .world_mut()
                .spawn((EntityKind::Visual, ChildOf(c.visual)));
        }),
        ("missing attachment marker", Unsupported, |c| {
            c.sim
                .world_mut()
                .entity_mut(c.collision)
                .remove::<AttachedPose>();
        }),
        ("missing descendant public ID", InvalidScene, |c| {
            c.sim.world_mut().entity_mut(c.visual).remove::<EntityId>();
        }),
        ("unindexed descendant public ID", InvalidScene, |c| {
            let identity = c
                .sim
                .world_mut()
                .entity_mut(c.collision)
                .take::<EntityId>()
                .unwrap();
            c.sim.world_mut().entity_mut(c.collision).insert(identity);
        }),
        ("missing descendant world pose", InvalidScene, |c| {
            c.sim.world_mut().entity_mut(c.visual).remove::<WorldPose>();
        }),
        ("missing descendant local pose", InvalidScene, |c| {
            c.sim
                .world_mut()
                .entity_mut(c.collision)
                .remove::<LocalPose>();
        }),
        ("missing model local pose", InvalidScene, |c| {
            c.sim.world_mut().entity_mut(c.model).remove::<LocalPose>();
        }),
        ("missing model world pose", InvalidScene, |c| {
            c.sim.world_mut().entity_mut(c.model).remove::<WorldPose>();
        }),
        ("missing world pose", InvalidScene, |c| {
            c.sim.world_mut().entity_mut(c.world).remove::<WorldPose>();
        }),
        ("missing link world pose", InvalidScene, |c| {
            c.sim.world_mut().entity_mut(c.link).remove::<WorldPose>();
        }),
        ("missing link velocity", InvalidScene, |c| {
            c.sim.world_mut().entity_mut(c.link).remove::<Velocity>();
        }),
        ("missing native handle", InvalidScene, |c| {
            c.sim.world_mut().entity_mut(c.link).remove::<BodyHandle>();
        }),
        ("missing link public ID", Unsupported, |c| {
            c.sim.world_mut().entity_mut(c.link).remove::<EntityId>();
        }),
        ("unindexed link public ID", Unsupported, |c| {
            let identity = c
                .sim
                .world_mut()
                .entity_mut(c.link)
                .take::<EntityId>()
                .unwrap();
            c.sim.world_mut().entity_mut(c.link).insert(identity);
        }),
        ("other link native handle", InvalidScene, |c| {
            c.sim
                .world_mut()
                .entity_mut(c.link)
                .insert(BodyHandle(c.bodies[1]));
        }),
        ("duplicate native owner", InvalidScene, |c| {
            c.sim.world_mut().spawn(BodyHandle(c.bodies[0]));
        }),
        ("missing dynamic anchor", InvalidScene, |c| {
            c.sim
                .world_mut()
                .entity_mut(c.model)
                .remove::<ModelAnchor>();
        }),
        ("missing backend", InvalidScene, |c| {
            c.parked = Some(
                c.sim
                    .world_mut()
                    .remove_resource::<PhysicsBackend>()
                    .unwrap(),
            );
        }),
        ("removed native body", Unsupported, |c| {
            assert!(
                c.sim
                    .world_mut()
                    .resource_mut::<PhysicsBackend>()
                    .remove(c.bodies[0])
            );
        }),
    ];
    for &(label, refusal, mutate) in mutations {
        let mut case = scene(false);
        advance(&mut case);
        assert!(
            model_state(&case.sim, case.model_id).is_ok(),
            "invalid baseline: {label}"
        );
        mutate(&mut case);
        refused_without_commit(&mut case, refusal, label);
    }
}

#[test]
fn foreign_native_ownership_and_actual_joint_participation_refuse_before_any_world_edit() {
    let mut foreign = scene(false);
    advance(&mut foreign);
    let original = foreign
        .sim
        .world_mut()
        .remove_resource::<PhysicsBackend>()
        .unwrap();
    let mut backend = RapierBackend::new([0.0; 3]).unwrap();
    for body in foreign.bodies {
        let initial = original.state(body).unwrap();
        let actual = backend
            .spawn(BodySpec {
                shape: Shape::Sphere { radius: 0.5 },
                position: initial.position,
                velocity: initial.velocity,
                mass: Some(1.0),
            })
            .unwrap();
        assert_eq!(actual, body);
    }
    foreign.parked = Some(original);
    foreign
        .sim
        .world_mut()
        .insert_resource(PhysicsBackend::from(backend));
    refused_without_commit(
        &mut foreign,
        Refusal::InvalidScene,
        "foreign backend with coincident IDs",
    );

    for reduced in [false, true] {
        let mut case = scene(false);
        let joint = {
            let mut backend = case.sim.world_mut().resource_mut::<PhysicsBackend>();
            if reduced {
                backend
                    .add_reduced_revolute(None, case.bodies[0], [0.0; 3], [0.0, 0.0, 1.0])
                    .unwrap()
            } else {
                backend
                    .add_world_revolute(case.bodies[0], [0.0; 3], [0.0, 0.0, 1.0])
                    .unwrap()
            }
        };
        case.sim.step_exact(2).unwrap();
        let before = case
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .joint_state(joint);
        refused_without_commit(&mut case, Refusal::Unsupported, "actual joint participant");
        assert_eq!(
            case.sim
                .world()
                .resource::<PhysicsBackend>()
                .joint_state(joint),
            before
        );
    }
}

#[test]
fn fixed_public_edits_require_zero_velocity_and_commit_the_actual_native_and_attached_frames() {
    let mut case = scene(true);
    case.sim.step_exact(2).unwrap();
    let before = snapshot(&case);
    for invalid in [
        ModelState {
            angular_velocity: [0.0; 3],
            ..request()
        },
        ModelState {
            linear_velocity: [0.0; 3],
            ..request()
        },
    ] {
        assert!(matches!(
            set_model_state(&mut case.sim, case.model_id, invalid),
            Err(WorldEditError::Physics(PhysicsError::InvalidValue))
        ));
        assert_eq!(snapshot(&case), before);
    }
    let requested = ModelState {
        linear_velocity: [0.0; 3],
        angular_velocity: [0.0; 3],
        ..request()
    };
    let other = case
        .sim
        .world()
        .resource::<PhysicsBackend>()
        .state(case.bodies[1]);
    assert_eq!(
        set_model_state(&mut case.sim, case.model_id, requested).unwrap(),
        requested
    );
    assert_eq!(model_state(&case.sim, case.model_id).unwrap(), requested);
    let native = case
        .sim
        .world()
        .resource::<PhysicsBackend>()
        .state(case.bodies[0])
        .unwrap();
    assert_eq!(native.position, requested.translation);
    assert_eq!(native.orientation, requested.rotation);
    assert_eq!(
        (native.velocity, native.angular_velocity),
        ([0.0; 3], [0.0; 3])
    );
    assert_eq!(
        case.sim
            .world()
            .get::<WorldPose>(case.visual)
            .unwrap()
            .translation,
        [3.75, 5.0, 6.0]
    );
    assert_eq!(
        case.sim
            .world()
            .resource::<PhysicsBackend>()
            .state(case.bodies[1]),
        other
    );
    assert_eq!(
        next_entity_id(case.sim.world()).unwrap(),
        before.next_public_id
    );
    assert_eq!(
        (
            case.sim.elapsed(),
            case.sim.epoch(),
            case.sim.is_paused(),
            case.sim.is_faulted()
        ),
        before.time
    );
    for (id, entity) in before.identities {
        assert_eq!(entity_by_id(case.sim.world(), id), entity);
    }
}
