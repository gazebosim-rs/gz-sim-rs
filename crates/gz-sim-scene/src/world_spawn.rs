// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Startup-prepared templates: no XML, resource lookup or asset IO during edits.
use super::*;
use gz_sim_core::{entity_by_id, next_entity_id};

enum Collider {
    Primitive(Shape),
    Mesh(gz_assets::mesh::TriangleMesh),
}

/// An immutable copy of one admitted loaded model's construction parameters.
/// Prepare at startup from the same resolved SDF used to construct the scene.
/// The source visual ID binds the CLI's immutable resource alias; browser requests
/// never supply a path, URI, model name, collision shape or inertial parameter.
pub struct ModelTemplate {
    owner: bevy_ecs::world::WorldId,
    world_id: u64,
    source_visual_id: u64,
    collider: Collider,
    collision_pose: WorldPose,
    visual_pose: WorldPose,
    geometry: Arc<Geometry>,
    properties: sdformat::VisualProperties,
    material: Option<Box<sdformat::Material>>,
    link_properties: sdformat::LinkProperties,
    density: Option<f64>,
    mass: Option<f64>,
    inertia: Inertia,
}

impl ModelTemplate {
    pub fn source_visual_id(&self) -> u64 {
        self.source_visual_id
    }
    pub fn visual_mesh(&self) -> Option<&sdformat::Mesh> {
        match self.geometry.as_ref() {
            Geometry::Mesh(mesh) => Some(mesh),
            _ => None,
        }
    }
    /// Project the immutable template's visual before a runtime spawn commits.
    pub fn visual_world_pose(&self, model_pose: WorldPose) -> WorldPose {
        model_pose.compose(self.visual_pose)
    }
}

#[derive(Component)]
#[component(immutable)]
pub(super) struct RuntimeBodyOwner {
    link: u64,
    body: BodyId,
}
impl RuntimeBodyOwner {
    pub(super) fn matches(&self, link: u64, body: BodyId) -> bool {
        self.link == link && self.body == body
    }
}

// Used only by loaded reset: arbitrary runtime physics still refuses. Validate
// the complete owned hierarchy before allowing its nodes through reset preflight.
pub(super) fn reset_nodes(
    world: &World,
) -> Result<std::collections::BTreeSet<Entity>, WorldEditError> {
    let mut result = std::collections::BTreeSet::new();
    for entity in world
        .iter_entities()
        .filter(|entity| entity.contains::<RuntimeBodyOwner>())
    {
        let parent = entity
            .get::<ChildOf>()
            .ok_or(WorldEditError::InvalidScene)?
            .parent();
        let id = world
            .get::<EntityId>(parent)
            .ok_or(WorldEditError::InvalidScene)?
            .get();
        let nodes = super::world_edit::admitted_entities(world, id)?;
        if nodes[1] != entity.id() || nodes.into_iter().any(|node| !result.insert(node)) {
            return Err(WorldEditError::InvalidScene);
        }
    }
    Ok(result)
}

/// Prepare a clone template for a loaded, standalone one-link/one-collision/
/// one-visual model. Existing Scene and native admission remain authoritative.
/// This function is read-only but may decode a declared local glTF collision.
pub fn prepare_model_template(
    sim: &Simulation,
    sdf: &Sdf,
    model: &Model,
) -> Result<ModelTemplate, WorldEditError> {
    let unsupported = || WorldEditError::UnsupportedModel;
    let world = sim.world();
    let root = world
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == model.scoped_name)
        })
        .ok_or_else(unsupported)?;
    let model_id = root.get::<EntityId>().ok_or_else(unsupported)?.get();
    super::world_edit::model_state(sim, model_id)?;
    if model.links.len() != 1
        || !model.joints.is_empty()
        || !model.models.is_empty()
        || !model.plugins.is_empty()
    {
        return Err(unsupported());
    }
    let link = &model.links[0];
    if link.collisions.len() != 1
        || link.visuals.len() != 1
        || !link.sensors.is_empty()
        || !link.plugins.is_empty()
    {
        return Err(unsupported());
    }
    let collision = &link.collisions[0];
    let visual = &link.visuals[0];
    let collider = match &collision.geometry {
        Geometry::Box { size } => Collider::Primitive(Shape::Box {
            half_extents: size.map(|value| value * 0.5),
        }),
        Geometry::Sphere { radius } => Collider::Primitive(Shape::Sphere { radius: *radius }),
        Geometry::Mesh(mesh) => Collider::Mesh(
            gz_assets::mesh::load(mesh).map_err(|_| WorldEditError::UnsupportedModel)?,
        ),
        _ => return Err(unsupported()),
    };
    let resolve = |pose: &sdformat::Pose| {
        sdf.resolve_pose(
            pose,
            &model.scoped_name,
            &link.scoped_name,
            &link.scoped_name,
        )
        .map(WorldPose::from)
        .map_err(|_| WorldEditError::InvalidScene)
    };
    let collision_pose = resolve(&collision.pose)?;
    if matches!(collider, Collider::Primitive(_))
        && (collision_pose.translation != [0.0; 3]
            || collision_pose.rotation != [0.0, 0.0, 0.0, 1.0])
    {
        return Err(unsupported());
    }
    let visual_pose = resolve(&visual.pose)?;
    let inertial_pose = resolve(&link.inertial.pose)?;
    let visual_name = format!("{}::{}", link.scoped_name, visual.name);
    let source_visual = world
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|name| name.0 == visual_name)
        })
        .ok_or_else(unsupported)?;
    let geometry = source_visual
        .get::<VisualGeometry>()
        .ok_or_else(unsupported)?
        .0
        .clone();
    let parent = root.get::<ChildOf>().ok_or_else(unsupported)?.parent();
    let world_id = world.get::<EntityId>(parent).ok_or_else(unsupported)?.get();
    Ok(ModelTemplate {
        owner: world.id(),
        world_id,
        source_visual_id: source_visual
            .get::<EntityId>()
            .ok_or_else(unsupported)?
            .get(),
        collider,
        collision_pose,
        visual_pose,
        geometry,
        properties: visual.properties.clone(),
        material: visual.material.clone(),
        link_properties: link.properties,
        density: collision.density,
        mass: (!model.is_static).then_some(link.inertial.mass),
        inertia: Inertia {
            center: inertial_pose.translation,
            orientation: inertial_pose.rotation,
            tensor: link.inertial.inertia,
        },
    })
}

// The loaded camera admits only a bounded, flat set of model origins. Check
// runtime structural edits before the native body or public allocator changes.
fn preflight_logical_camera_spawn(
    world: &World,
    parent: Entity,
    new_name: &str,
) -> Result<(), WorldEditError> {
    if !world.contains_resource::<crate::logical_camera::Installed>()
        && !world
            .iter_entities()
            .any(|entity| entity.contains::<crate::logical_camera::LogicalCamera>())
    {
        return Ok(());
    }
    let world_name = &world
        .get::<Name>(parent)
        .ok_or(WorldEditError::InvalidScene)?
        .0;
    let prefix = format!("{world_name}::");
    let mut names = std::collections::BTreeSet::new();
    for entity in world
        .iter_entities()
        .filter(|entity| entity.get::<EntityKind>() == Some(&EntityKind::Model))
    {
        if entity.get::<ChildOf>().map(ChildOf::parent) != Some(parent) {
            return Err(WorldEditError::InvalidScene);
        }
        let id = entity
            .get::<EntityId>()
            .ok_or(WorldEditError::InvalidScene)?
            .get();
        if entity_by_id(world, id) != Some(entity.id()) {
            return Err(WorldEditError::InvalidScene);
        }
        let name = entity
            .get::<Name>()
            .ok_or(WorldEditError::InvalidScene)?
            .0
            .strip_prefix(&prefix)
            .ok_or(WorldEditError::InvalidScene)?;
        if !gz_sensors::logical_camera::valid_name(name) || !names.insert(name) {
            return Err(WorldEditError::InvalidScene);
        }
        if names.len() >= 128 {
            return Err(WorldEditError::UnsupportedModel);
        }
    }
    let name = new_name
        .strip_prefix(&prefix)
        .ok_or(WorldEditError::InvalidScene)?;
    if !gz_sensors::logical_camera::valid_name(name) || !names.insert(name) {
        return Err(WorldEditError::UnsupportedModel);
    }
    Ok(())
}

/// Clone one startup template at a validated model origin. All ordinary errors
/// precede construction. The four public IDs are allocated only after the native
/// adapter accepts the body. An arbitrary external ECS observer that corrupts
/// allocator/hierarchy ownership is a terminal invariant failure, not a refusal.
/// The caller preflights its bounded viewer catalog and declared asset alias.
pub fn spawn_model_template(
    sim: &mut Simulation,
    template: &ModelTemplate,
    pose: WorldPose,
) -> Result<u64, WorldEditError> {
    let world = sim.world();
    let parent = entity_by_id(world, template.world_id).ok_or(WorldEditError::InvalidScene)?;
    if world.id() != template.owner
        || world.get::<EntityKind>(parent) != Some(&EntityKind::World)
        || !initial::matches_physics_owner(world)
    {
        return Err(WorldEditError::InvalidScene);
    }
    let backend = world
        .get_resource::<PhysicsBackend>()
        .ok_or(WorldEditError::InvalidScene)?;
    if backend.engine() != "rapier" {
        return Err(WorldEditError::UnsupportedModel);
    }
    let first = next_entity_id(world)?;
    let last = first.checked_add(3).ok_or(IdentityError::Exhausted)?;
    if last > u32::MAX as u64 {
        return Err(WorldEditError::UnsupportedModel);
    }
    let world_name = &world
        .get::<Name>(parent)
        .ok_or(WorldEditError::InvalidScene)?
        .0;
    let name = format!("{world_name}::runtime_{first}");
    if name.len() + "::body::visual".len() > 256
        || world
            .iter_entities()
            .any(|entity| entity.get::<Name>().is_some_and(|other| other.0 == name))
    {
        return Err(WorldEditError::UnsupportedModel);
    }
    preflight_logical_camera_spawn(world, parent, &name)?;
    let local = world
        .get::<WorldPose>(parent)
        .ok_or(WorldEditError::InvalidScene)?
        .inverse()
        .compose(pose);
    let collision_world = pose.compose(template.collision_pose);
    let visual_world = pose.compose(template.visual_pose);
    if [pose, local, collision_world, visual_world]
        .into_iter()
        .any(|pose| !super::world_edit::representable(pose))
    {
        return Err(WorldEditError::InvalidState);
    }
    let rotation = RotationSpec {
        orientation: pose.rotation,
        angular_velocity: [0.0; 3],
    };
    let body = {
        let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
        match &template.collider {
            Collider::Primitive(shape) => backend.spawn_link(
                BodySpec {
                    shape: *shape,
                    position: pose.translation,
                    velocity: [0.0; 3],
                    mass: template.mass,
                },
                rotation,
                Some(template.inertia),
            )?,
            Collider::Mesh(mesh) => backend.spawn_mesh_link(
                MeshBodySpec {
                    vertices: &mesh.vertices,
                    triangles: &mesh.triangles,
                    collision_position: template.collision_pose.translation,
                    collision_orientation: template.collision_pose.rotation,
                    position: pose.translation,
                    velocity: [0.0; 3],
                    mass: template.mass,
                },
                rotation,
                Some(template.inertia),
            )?,
        }
    };
    let world = sim.world_mut();
    let identity = WorldPose {
        translation: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
    };
    let mut node = |parent: Entity, suffix: &str, kind, local, pose| {
        let name = format!("{name}{suffix}");
        let local_name = name.rsplit("::").next().expect("nonempty name").to_owned();
        let entity = world
            .spawn((
                Name(name),
                LocalName(local_name),
                kind,
                LocalPose(local),
                pose,
                ChildOf(parent),
            ))
            .id();
        let id = assign_id(world, entity).expect("exclusive preflighted allocation");
        (entity, id)
    };
    let (model, model_id) = node(parent, "", EntityKind::Model, local, pose);
    let (link, link_id) = node(model, "::body", EntityKind::Link, identity, pose);
    let (collision, collision_id) = node(
        link,
        "::body::collision",
        EntityKind::Collision,
        template.collision_pose,
        collision_world,
    );
    let (visual, visual_id) = node(
        link,
        "::body::visual",
        EntityKind::Visual,
        template.visual_pose,
        visual_world,
    );
    assert_eq!(
        [model_id, link_id, collision_id, visual_id],
        [first, first + 1, first + 2, last],
        "external observer changed preflighted identity allocation"
    );
    world.entity_mut(model).insert(ModelAnchor {
        link: link_id,
        link_to_model: identity,
    });
    world.entity_mut(link).insert((
        BodyHandle(body),
        RuntimeBodyOwner {
            link: link_id,
            body,
        },
        LinkProperties(template.link_properties),
        Velocity {
            linear: [0.0; 3],
            angular: [0.0; 3],
        },
    ));
    if template.inertia.center != [0.0; 3] {
        world
            .entity_mut(link)
            .insert(LinkCenterOfMass(template.inertia.center));
    }
    world.entity_mut(collision).insert(AttachedPose);
    if let Some(density) = template.density {
        world
            .entity_mut(collision)
            .insert(CollisionDensity(density));
    }
    world.entity_mut(visual).insert((
        AttachedPose,
        VisualGeometry(template.geometry.clone()),
        VisualProperties(template.properties.clone()),
    ));
    if let Some(material) = &template.material {
        world
            .entity_mut(visual)
            .insert(VisualMaterial(material.clone()));
    }
    Ok(model_id)
}
