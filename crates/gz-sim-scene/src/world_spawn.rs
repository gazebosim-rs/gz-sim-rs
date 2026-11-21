// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Startup-prepared templates: no XML, resource lookup or asset IO during edits.
use super::*;
use gz_sim_core::{ReservedId, entity_by_id, next_entity_id, reserve_id};

enum Collider {
    Primitive(Shape),
    Mesh(gz_assets::mesh::TriangleMesh),
    Compound {
        collisions: Vec<CollisionSpec>,
        densities: Vec<Option<f64>>,
    },
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

// A compound never receives the singleton RuntimeBodyOwner. Two independent,
// immutable stamp locations retain its original domain after public hierarchy
// edits; neither a live collision count nor caller-owned SDF authorizes edits.
#[derive(Component)]
#[component(immutable)]
struct RuntimeCompoundOrigin(Arc<CompoundOrigin>);

#[derive(Component)]
#[component(immutable)]
struct RuntimeCompoundLink(Arc<CompoundOrigin>);

struct CompoundMember {
    entity: Entity,
    id: ReservedId,
    parent: Option<Entity>,
    kind: EntityKind,
    name: String,
    local_name: String,
    local: WorldPose,
    attached: bool,
    body: Option<BodyId>,
    density: Option<f64>,
    visual: Option<Arc<Geometry>>,
}
impl CompoundMember {
    fn capture(world: &World, entity: Entity) -> Self {
        Self {
            entity,
            id: reserve_id(world, entity).expect("exclusive preflighted runtime identity"),
            parent: world.get::<ChildOf>(entity).map(ChildOf::parent),
            kind: *world
                .get::<EntityKind>(entity)
                .expect("constructed runtime kind"),
            name: world
                .get::<Name>(entity)
                .expect("constructed runtime name")
                .0
                .clone(),
            local_name: world
                .get::<LocalName>(entity)
                .expect("constructed runtime local name")
                .0
                .clone(),
            local: world
                .get::<LocalPose>(entity)
                .expect("preflighted runtime pose")
                .0,
            attached: world.entity(entity).contains::<AttachedPose>(),
            body: world.get::<BodyHandle>(entity).map(BodyHandle::id),
            density: world.get::<CollisionDensity>(entity).map(|v| v.0),
            visual: world.get::<VisualGeometry>(entity).map(|v| v.0.clone()),
        }
    }

    fn matches(&self, world: &World, pin_local: bool) -> bool {
        self.id.validate(world).is_ok()
            && entity_by_id(world, self.id.get()) == Some(self.entity)
            && world.get::<EntityKind>(self.entity) == Some(&self.kind)
            && world.get::<ChildOf>(self.entity).map(ChildOf::parent) == self.parent
            && world
                .get::<Name>(self.entity)
                .is_some_and(|v| v.0 == self.name)
            && world
                .get::<LocalName>(self.entity)
                .is_some_and(|v| v.0 == self.local_name)
            && world.get::<LocalPose>(self.entity).is_some_and(|v| {
                !pin_local
                    || (v.0.translation == self.local.translation
                        && v.0.rotation == self.local.rotation)
            })
            && world.get::<WorldPose>(self.entity).is_some()
            && world.entity(self.entity).contains::<AttachedPose>() == self.attached
            && world.get::<BodyHandle>(self.entity).map(BodyHandle::id) == self.body
            && world.get::<CollisionDensity>(self.entity).map(|v| v.0) == self.density
            && match (&self.visual, world.get::<VisualGeometry>(self.entity)) {
                (Some(original), Some(current)) => Arc::ptr_eq(original, &current.0),
                (None, None) => true,
                _ => false,
            }
    }
}

struct CompoundOrigin {
    world: CompoundMember,
    model: CompoundMember,
    link: CompoundMember,
    members: Vec<CompoundMember>,
    collisions: Vec<CollisionSpec>,
    body: BodyId,
    dynamic: bool,
}

pub(super) fn original_runtime_compound(world: &World, id: u64) -> bool {
    world.iter_entities().any(|entity| {
        entity
            .get::<RuntimeCompoundOrigin>()
            .is_some_and(|v| v.0.model.id.get() == id)
            || entity
                .get::<RuntimeCompoundLink>()
                .is_some_and(|v| v.0.model.id.get() == id)
    })
}

// Return the actual bounded construction nodes, never an inferred live subtree.
// Loaded reset already accepts this set without changing InitialScene's fixed
// singleton API or restoring any runtime body into the loaded blueprint.
pub(super) fn compound_nodes(world: &World, id: u64) -> Result<Vec<Entity>, WorldEditError> {
    let bad = || WorldEditError::UnsupportedModel;
    let origin = world
        .iter_entities()
        .find_map(|entity| {
            entity
                .get::<RuntimeCompoundOrigin>()
                .filter(|v| v.0.model.id.get() == id)
        })
        .ok_or_else(bad)?;
    let stamp = &origin.0;
    let model = stamp.model.entity;
    let link = stamp.link.entity;
    if !(2..=16).contains(&stamp.collisions.len())
        || stamp.members.len() != stamp.collisions.len() + 1
        || !stamp.world.matches(world, false)
        || !stamp.model.matches(world, false)
        || !stamp.link.matches(world, false)
        || stamp.world.kind != EntityKind::World
        || stamp.model.kind != EntityKind::Model
        || stamp.link.kind != EntityKind::Link
        || stamp.model.parent != Some(stamp.world.entity)
        || stamp.link.parent != Some(model)
        || world
            .get::<Children>(model)
            .is_none_or(|v| v.len() != 1 || v[0] != link)
        || !world.get::<LocalPose>(link).is_some_and(|v| {
            identity(Transform {
                translation: v.0.translation,
                rotation: v.0.rotation,
            })
        })
        || world.get::<LinkCenterOfMass>(link).is_some()
        || world.get::<Velocity>(link).is_none()
        || world.get::<ModelAnchor>(model).is_none_or(|v| {
            v.link != stamp.link.id.get()
                || v.link_to_model.translation != [0.; 3]
                || v.link_to_model.rotation != [0., 0., 0., 1.]
        })
        || world
            .get::<RuntimeCompoundOrigin>(model)
            .is_none_or(|v| !Arc::ptr_eq(&v.0, &origin.0))
        || world
            .get::<RuntimeCompoundLink>(link)
            .is_none_or(|v| !Arc::ptr_eq(&v.0, &origin.0))
        || world
            .iter_entities()
            .filter(|entity| {
                entity
                    .get::<RuntimeCompoundOrigin>()
                    .is_some_and(|v| v.0.model.id.get() == id)
            })
            .count()
            != 1
        || world
            .iter_entities()
            .filter(|entity| {
                entity
                    .get::<RuntimeCompoundLink>()
                    .is_some_and(|v| v.0.model.id.get() == id)
            })
            .count()
            != 1
        || !initial::matches_physics_owner(world)
    {
        return Err(bad());
    }
    let backend = world.get_resource::<PhysicsBackend>().ok_or_else(bad)?;
    if backend.engine() != "rapier"
        || backend.state(stamp.body).is_none()
        || stamp.link.body != Some(stamp.body)
        || backend.is_dynamic(stamp.body) != Some(stamp.dynamic)
        || backend.is_jointed(stamp.body) != Some(false)
        || world.iter_entities().any(|entity| {
            entity.id() != link
                && entity
                    .get::<BodyHandle>()
                    .is_some_and(|v| v.id() == stamp.body)
        })
    {
        return Err(bad());
    }
    let children = world.get::<Children>(link).ok_or_else(bad)?;
    if children.len() != stamp.members.len() {
        return Err(bad());
    }
    let mut nodes = vec![model, link];
    let mut collision_index = 0;
    for member in &stamp.members {
        if !member.matches(world, true)
            || member.parent != Some(link)
            || !member.attached
            || !children.contains(&member.entity)
            || nodes.contains(&member.entity)
            || world
                .get::<Children>(member.entity)
                .is_some_and(|v| !v.is_empty())
        {
            return Err(bad());
        }
        match member.kind {
            EntityKind::Collision => {
                let spec = stamp.collisions.get(collision_index).ok_or_else(bad)?;
                if member.local.translation != spec.position
                    || member.local.rotation != spec.orientation
                    || !valid_primitive(spec.shape)
                    || member.visual.is_some()
                {
                    return Err(bad());
                }
                collision_index += 1;
            }
            EntityKind::Visual if member.visual.is_some() => {}
            _ => return Err(bad()),
        }
        nodes.push(member.entity);
    }
    if collision_index != stamp.collisions.len()
        || stamp
            .members
            .iter()
            .filter(|v| v.kind == EntityKind::Visual)
            .count()
            != 1
    {
        return Err(bad());
    }
    Ok(nodes)
}

fn valid_primitive(shape: Shape) -> bool {
    match shape {
        Shape::Sphere { radius } => radius.is_finite() && radius > 0.,
        Shape::Box { half_extents } => half_extents.into_iter().all(|v| v.is_finite() && v > 0.),
        Shape::Cylinder {
            radius,
            half_height,
        } => [radius, half_height]
            .into_iter()
            .all(|v| v.is_finite() && v > 0.),
        Shape::Capsule {
            radius,
            half_length,
        } => {
            [radius, half_length]
                .into_iter()
                .all(|v| v.is_finite() && v > 0.)
                && (radius + half_length).is_finite()
        }
    }
}

// Used only by loaded reset: arbitrary runtime physics still refuses. Validate
// the complete owned hierarchy before allowing its nodes through reset preflight.
pub(super) fn reset_nodes(
    world: &World,
) -> Result<std::collections::BTreeSet<Entity>, WorldEditError> {
    let mut result = std::collections::BTreeSet::new();
    let compounds: std::collections::BTreeSet<_> = world
        .iter_entities()
        .filter_map(|entity| {
            entity
                .get::<RuntimeCompoundOrigin>()
                .map(|v| v.0.model.id.get())
                .or_else(|| {
                    entity
                        .get::<RuntimeCompoundLink>()
                        .map(|v| v.0.model.id.get())
                })
        })
        .collect();
    for id in compounds {
        for node in compound_nodes(world, id)? {
            if !result.insert(node) {
                return Err(WorldEditError::InvalidScene);
            }
        }
    }
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
/// one-visual singleton or passive primitive-compound model. Existing Scene and
/// native admission remain authoritative; supplied resolved SDF parameters are
/// copied, not asserted equal to the loaded body's geometry/inertia.
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
    // A runtime compound is not a loaded template source, even when a caller
    // supplies a different, resolvable singleton DOM for the same public name.
    if original_runtime_compound(world, model_id) {
        return Err(unsupported());
    }
    if model.links.len() != 1
        || !model.joints.is_empty()
        || !model.models.is_empty()
        || !model.plugins.is_empty()
    {
        return Err(unsupported());
    }
    let link = &model.links[0];
    let compound = (2..=16).contains(&link.collisions.len());
    if (link.collisions.len() != 1 && !compound)
        || link.visuals.len() != 1
        || !link.sensors.is_empty()
        || !link.plugins.is_empty()
    {
        return Err(unsupported());
    }
    if compound
        && (!initial::original_compound_model(world, model_id)
            || crate::velocity_control::owns_model(world, model_id))
    {
        return Err(unsupported());
    }
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
    let collision = &link.collisions[0];
    let visual = &link.visuals[0];
    let collider = if compound {
        let local_link = sdf
            .resolve_pose(
                &link.pose,
                &model.scoped_name,
                &model.scoped_name,
                &model.scoped_name,
            )
            .map_err(|_| WorldEditError::InvalidScene)?;
        if !identity(local_link) || link.inertial.fluid_added_mass.is_some() {
            return Err(unsupported());
        }
        let mut collisions = Vec::with_capacity(link.collisions.len());
        let mut densities = Vec::with_capacity(link.collisions.len());
        for child in &link.collisions {
            let shape = match child.geometry {
                Geometry::Box { size } => Shape::Box {
                    half_extents: size.map(|v| v * 0.5),
                },
                Geometry::Sphere { radius } => Shape::Sphere { radius },
                Geometry::Cylinder { radius, length } => Shape::Cylinder {
                    radius,
                    half_height: length * 0.5,
                },
                Geometry::Capsule { radius, length } => Shape::Capsule {
                    radius,
                    half_length: length * 0.5,
                },
                _ => return Err(unsupported()),
            };
            if child.surface.source.is_some() {
                return Err(unsupported());
            }
            let local = resolve(&child.pose)?;
            if !valid_primitive(shape)
                || !super::world_edit::representable(local)
                || child.density.is_some_and(|v| !v.is_finite() || v <= 0.)
            {
                return Err(WorldEditError::InvalidState);
            }
            collisions.push(CollisionSpec {
                shape,
                position: local.translation,
                orientation: local.rotation,
            });
            densities.push(child.density);
        }
        Collider::Compound {
            collisions,
            densities,
        }
    } else {
        match &collision.geometry {
            Geometry::Box { size } => Collider::Primitive(Shape::Box {
                half_extents: size.map(|value| value * 0.5),
            }),
            Geometry::Sphere { radius } => Collider::Primitive(Shape::Sphere { radius: *radius }),
            Geometry::Cylinder { radius, length } => Collider::Primitive(Shape::Cylinder {
                radius: *radius,
                half_height: *length * 0.5,
            }),
            Geometry::Capsule { radius, length } => Collider::Primitive(Shape::Capsule {
                radius: *radius,
                half_length: *length * 0.5,
            }),
            Geometry::Mesh(mesh) => Collider::Mesh(
                gz_assets::mesh::load(mesh).map_err(|_| WorldEditError::UnsupportedModel)?,
            ),
            _ => return Err(unsupported()),
        }
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
    if compound
        && (inertial_pose.translation != [0.; 3]
            || !super::world_edit::representable(inertial_pose))
    {
        return Err(unsupported());
    }
    let visual_name = format!("{}::{}", link.scoped_name, visual.name);
    let source_visual = world
        .iter_entities()
        .find(|entity| {
            entity.get::<EntityKind>() == Some(&EntityKind::Visual)
                && entity
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
/// precede construction. The singleton four or compound N+3 IDs retain the
/// model/link/first-collision/visual prefix and are allocated only after the native
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
    let count = match &template.collider {
        Collider::Compound { collisions, .. } => collisions.len(),
        _ => 1,
    };
    let last = first
        .checked_add(count as u64 + 2)
        .ok_or(IdentityError::Exhausted)?;
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
    if let Collider::Compound {
        collisions,
        densities,
    } = &template.collider
    {
        if !(2..=16).contains(&collisions.len())
            || densities.len() != collisions.len()
            || world.get::<LocalPose>(parent).is_none()
            || world.get::<LocalName>(parent).is_none()
        {
            return Err(WorldEditError::InvalidScene);
        }
        reserve_id(world, parent)?;
        let mut names = vec![
            name.clone(),
            format!("{name}::body"),
            format!("{name}::body::collision"),
            format!("{name}::body::visual"),
        ];
        names.extend((1..count).map(|index| format!("{name}::body::collision_{index}")));
        if names.iter().any(|name| name.len() > 256)
            || world
                .iter_entities()
                .any(|entity| entity.get::<Name>().is_some_and(|v| names.contains(&v.0)))
        {
            return Err(WorldEditError::UnsupportedModel);
        }
        if collisions.iter().any(|child| {
            !valid_primitive(child.shape)
                || !super::world_edit::representable(pose.compose(WorldPose {
                    translation: child.position,
                    rotation: child.orientation,
                }))
        }) || densities
            .iter()
            .any(|v| v.is_some_and(|v| !v.is_finite() || v <= 0.))
        {
            return Err(WorldEditError::InvalidState);
        }
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
            Collider::Compound { collisions, .. } => backend.spawn_compound_link(
                CompoundBodySpec {
                    collisions,
                    position: pose.translation,
                    velocity: [0.; 3],
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
        [first, first + 1, first + 2, first + 3],
        "external observer changed preflighted identity allocation"
    );
    let mut compound_children = vec![collision, visual];
    if let Collider::Compound { collisions, .. } = &template.collider {
        for (index, child) in collisions.iter().enumerate().skip(1) {
            let local = WorldPose {
                translation: child.position,
                rotation: child.orientation,
            };
            let (entity, id) = node(
                link,
                &format!("::body::collision_{index}"),
                EntityKind::Collision,
                local,
                pose.compose(local),
            );
            assert_eq!(
                id,
                first + index as u64 + 3,
                "external observer changed compound allocation"
            );
            compound_children.push(entity);
        }
        assert_eq!(
            next_entity_id(world).expect("preflighted allocator"),
            last + 1
        );
    }
    world.entity_mut(model).insert(ModelAnchor {
        link: link_id,
        link_to_model: identity,
    });
    world.entity_mut(link).insert((
        BodyHandle(body),
        LinkProperties(template.link_properties),
        Velocity {
            linear: [0.0; 3],
            angular: [0.0; 3],
        },
    ));
    if !matches!(&template.collider, Collider::Compound { .. }) {
        world.entity_mut(link).insert(RuntimeBodyOwner {
            link: link_id,
            body,
        });
    }
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
    if let Collider::Compound {
        collisions,
        densities,
    } = &template.collider
    {
        for (index, entity) in compound_children
            .iter()
            .copied()
            .filter(|e| *e != visual)
            .enumerate()
        {
            world.entity_mut(entity).insert(AttachedPose);
            if let Some(density) = densities[index] {
                world.entity_mut(entity).insert(CollisionDensity(density));
            }
        }
        let stamp = Arc::new(CompoundOrigin {
            world: CompoundMember::capture(world, parent),
            model: CompoundMember::capture(world, model),
            link: CompoundMember::capture(world, link),
            members: compound_children
                .iter()
                .map(|entity| CompoundMember::capture(world, *entity))
                .collect(),
            collisions: collisions.clone(),
            body,
            dynamic: template.mass.is_some(),
        });
        world
            .entity_mut(model)
            .insert(RuntimeCompoundOrigin(stamp.clone()));
        world.entity_mut(link).insert(RuntimeCompoundLink(stamp));
    }
    crate::apply_link_wrench::install(sim);
    Ok(model_id)
}

#[cfg(test)]
mod runtime_compound_tests {
    use super::*;

    #[test]
    fn immutable_runtime_compound_markers_and_center_refuse_atomically_without_singleton_fallback()
    {
        let xml = "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='source'><pose>-100 0 10 0 0 0</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial><collision name='a'><pose>2 0 0 0 0 0</pose><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><collision name='b'><pose>5 0 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='view'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>";
        let sdf = sdformat::load_str(xml, "markers", &sdformat::NoResources).unwrap();
        let mut sim = load_world_with_sdf_step(
            xml,
            "markers",
            "w",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
        let pose = WorldPose {
            translation: [10., 0., 10.],
            rotation: [0., 0., 0., 1.],
        };
        let id = spawn_model_template(&mut sim, &template, pose).unwrap();
        let model = entity_by_id(sim.world(), id).unwrap();
        let link = entity_by_id(sim.world(), id + 1).unwrap();
        let child = entity_by_id(sim.world(), id + 4).unwrap();
        let handle = sim.world().get::<BodyHandle>(link).unwrap().id();
        assert!(sim.world().get::<RuntimeBodyOwner>(link).is_none());
        assert_eq!(compound_nodes(sim.world(), id).unwrap().len(), 5);
        let original_model = sim
            .world()
            .iter_entities()
            .find(|v| v.get::<Name>().is_some_and(|v| v.0 == "w::source"))
            .unwrap()
            .get::<EntityId>()
            .unwrap()
            .get();
        assert!(!original_runtime_compound(sim.world(), original_model));
        assert!(model_state(&sim, original_model).is_ok());
        assert!(matches!(
            compound_nodes(sim.world(), original_model),
            Err(WorldEditError::UnsupportedModel)
        ));
        queue_link_world_wrench(&mut sim, id + 1, [4., 0., 0.], [0.; 3]).unwrap();
        let native = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .unwrap();
        let identities = |sim: &Simulation| {
            let mut pairs: Vec<_> = sim
                .world()
                .iter_entities()
                .filter_map(|v| v.get::<EntityId>().map(|id| (id.get(), v.id())))
                .collect();
            pairs.sort_unstable();
            pairs
        };
        let ids = identities(&sim);
        let next = next_entity_id(sim.world()).unwrap();
        let clock = (sim.epoch(), sim.elapsed());
        for row in 0..4 {
            // Loss of either private marker must fail even with all public IDs
            // and native state intact; attached/center drift is independently closed.
            let origin = if row == 0 {
                sim.world_mut()
                    .entity_mut(model)
                    .take::<RuntimeCompoundOrigin>()
            } else {
                None
            };
            let owner = if row == 1 {
                sim.world_mut()
                    .entity_mut(link)
                    .take::<RuntimeCompoundLink>()
            } else {
                None
            };
            let attached = if row == 2 {
                sim.world_mut().entity_mut(child).take::<AttachedPose>()
            } else {
                None
            };
            if row == 3 {
                sim.world_mut()
                    .entity_mut(link)
                    .insert(LinkCenterOfMass([0.25, 0., 0.]));
            }
            assert!(original_runtime_compound(sim.world(), id));
            assert!(matches!(
                model_state(&sim, id),
                Err(WorldEditError::UnsupportedModel)
            ));
            assert!(reset_nodes(sim.world()).is_err());
            assert_eq!(
                sim.world().resource::<PhysicsBackend>().state(handle),
                Some(native)
            );
            assert_eq!(identities(&sim), ids);
            assert_eq!(next_entity_id(sim.world()).unwrap(), next);
            assert_eq!((sim.epoch(), sim.elapsed()), clock);
            if let Some(origin) = origin {
                sim.world_mut().entity_mut(model).insert(origin);
            }
            if let Some(owner) = owner {
                sim.world_mut().entity_mut(link).insert(owner);
            }
            if let Some(attached) = attached {
                sim.world_mut().entity_mut(child).insert(attached);
            }
            if row == 3 {
                sim.world_mut()
                    .entity_mut(link)
                    .remove::<LinkCenterOfMass>();
            }
            assert_eq!(compound_nodes(sim.world(), id).unwrap().len(), 5);
            assert!(model_state(&sim, id).is_ok());
        }
        sim.step_exact(1).unwrap();
        let velocity = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .unwrap()
            .velocity;
        assert!(
            (velocity[0] - 0.002).abs() < 1e-11,
            "pending wrench was lost: {velocity:?}"
        );
    }
}
