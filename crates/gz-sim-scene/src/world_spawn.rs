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

/// An immutable copy of an admitted model's caller-resolved construction parameters.
/// Preparation resolves copied parameters against the caller's SDF graph.
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
    center_of_mass: Option<[f64; 3]>,
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
        || world.get::<LinkCenterOfMass>(link).map(|v| v.0) != stamp.center_of_mass
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

/// Prepare a clone template for an admitted standalone one-link/one-visual
/// singleton or passive loaded/owned-runtime primitive-compound model. Scene and
/// native admission remain authoritative; supplied resolved SDF parameters are
/// copied, not asserted equal to the actual body's geometry/inertia.
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
    // model_state already validated the actual immutable runtime membership.
    // Caller collision count chooses copied parameters, never source ownership.
    let runtime_compound = original_runtime_compound(world, model_id);
    if model.links.len() != 1
        || !model.joints.is_empty()
        || !model.models.is_empty()
        || !model.plugins.is_empty()
    {
        return Err(unsupported());
    }
    let link = &model.links[0];
    let compound = (2..=16).contains(&link.collisions.len());
    let runtime_singleton = runtime_compound && link.collisions.len() == 1;
    if (link.collisions.len() != 1 && !compound)
        || link.visuals.len() != 1
        || !link.sensors.is_empty()
        || !link.plugins.is_empty()
    {
        return Err(unsupported());
    }
    if compound
        && (!(initial::original_compound_model(world, model_id) || runtime_compound)
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
    // Validate the actual source above before caller geometry selects a route.
    // This source route reuses mature Mesh decoding and singleton construction.
    if runtime_singleton
        && (!matches!(
            collision.geometry,
            Geometry::Box { .. }
                | Geometry::Sphere { .. }
                | Geometry::Cylinder { .. }
                | Geometry::Capsule { .. }
                | Geometry::Mesh(_)
        ) || collision.surface.source.is_some()
            || crate::velocity_control::owns_model(world, model_id))
    {
        return Err(unsupported());
    }
    if compound || runtime_singleton {
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
    }
    let collider = if compound {
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
    let posed_primitive = matches!(collider, Collider::Primitive(_))
        && (collision_pose.translation != [0.0; 3]
            || collision_pose.rotation != [0.0, 0.0, 0.0, 1.0]);
    // Only the newly posed route requires an actually passive source. Caller
    // parameters may differ from loaded geometry, but stripping a controller
    // plugin cannot authorize this new branch. Mature identity/mesh policy stays.
    if posed_primitive && crate::velocity_control::owns_model(world, model_id) {
        return Err(unsupported());
    }
    if (posed_primitive || runtime_singleton)
        && (!super::world_edit::representable(collision_pose)
            || collision.density.is_some_and(|v| !v.is_finite() || v <= 0.))
    {
        return Err(WorldEditError::InvalidState);
    }
    let visual_pose = resolve(&visual.pose)?;
    let inertial_pose = resolve(&link.inertial.pose)?;
    let translated_compound = compound && inertial_pose.translation != [0.; 3];
    if translated_compound && inertial_pose.rotation != [0., 0., 0., 1.] {
        // Only translation is new. Existing centered inertia-basis admission is
        // unchanged, while an offset must retain the caller's identity basis.
        return Err(unsupported());
    }
    let translated_primitive = posed_primitive && inertial_pose.translation != [0.; 3];
    if translated_primitive {
        // `runtime_singleton` means a requested singleton from an ACTUAL compound.
        // The new COM route instead requires an actual admitted singleton source;
        // neither caller collision count nor stripped plugins can change ownership.
        if runtime_compound
            || initial::original_compound_model(world, model_id)
            || link.inertial.fluid_added_mass.is_some()
        {
            return Err(unsupported());
        }
        let local_link = sdf
            .resolve_pose(
                &link.pose,
                &model.scoped_name,
                &model.scoped_name,
                &model.scoped_name,
            )
            .map_err(|_| WorldEditError::InvalidScene)?;
        if !identity(local_link) {
            return Err(unsupported());
        }
    }
    if (compound || posed_primitive || runtime_singleton)
        && ((inertial_pose.translation != [0.; 3] && !translated_primitive && !translated_compound)
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
            Collider::Primitive(shape)
                if template.collision_pose.translation != [0.0; 3]
                    || template.collision_pose.rotation != [0.0, 0.0, 0.0, 1.0] =>
            {
                // The local collider pose never moves the link or its inertia.
                // Cylinder's Y-to-Z adaptation is owned by its existing backend.
                match *shape {
                    Shape::Box { half_extents } => backend.spawn_posed_box_link(
                        PosedBoxBodySpec {
                            half_extents,
                            collision_position: template.collision_pose.translation,
                            collision_orientation: template.collision_pose.rotation,
                            position: pose.translation,
                            velocity: [0.; 3],
                            mass: template.mass,
                        },
                        rotation,
                        Some(template.inertia),
                    )?,
                    Shape::Sphere { radius } => backend.spawn_posed_sphere_link(
                        PosedSphereBodySpec {
                            radius,
                            collision_position: template.collision_pose.translation,
                            collision_orientation: template.collision_pose.rotation,
                            position: pose.translation,
                            velocity: [0.; 3],
                            mass: template.mass,
                        },
                        rotation,
                        Some(template.inertia),
                    )?,
                    Shape::Cylinder {
                        radius,
                        half_height,
                    } => backend.spawn_posed_cylinder_link(
                        PosedCylinderBodySpec {
                            radius,
                            half_height,
                            collision_position: template.collision_pose.translation,
                            collision_orientation: template.collision_pose.rotation,
                            position: pose.translation,
                            velocity: [0.; 3],
                            mass: template.mass,
                        },
                        rotation,
                        Some(template.inertia),
                    )?,
                    Shape::Capsule {
                        radius,
                        half_length,
                    } => backend.spawn_posed_capsule_link(
                        PosedCapsuleBodySpec {
                            radius,
                            half_length,
                            collision_position: template.collision_pose.translation,
                            collision_orientation: template.collision_pose.rotation,
                            position: pose.translation,
                            velocity: [0.; 3],
                            mass: template.mass,
                        },
                        rotation,
                        Some(template.inertia),
                    )?,
                }
            }
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
            center_of_mass: world.get::<LinkCenterOfMass>(link).map(|v| v.0),
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

    #[test]
    fn runtime_compound_retemplate_requires_both_original_stamps_and_keeps_pending_input() {
        let xml = "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='source'><pose>-100 0 10 0 0 0</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>3</iyy><izz>4</izz></inertia></inertial><collision name='a'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><collision name='b'><pose>4 0 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='view'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>";
        let sdf = sdformat::load_str(xml, "stamp-retemplate", &sdformat::NoResources).unwrap();
        let mut sim = load_world_with_sdf_step(
            xml,
            "stamp-retemplate",
            "w",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let template = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
        let id = spawn_model_template(
            &mut sim,
            &template,
            WorldPose {
                translation: [10., 0., 10.],
                rotation: [0., 0., 0., 1.],
            },
        )
        .unwrap();
        let runtime_xml = format!(
            "<sdf version='1.12'><world name='w'><model name='runtime_{id}'><link name='body'><collision name='a'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><collision name='b'><pose>4 0 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='visual'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>"
        );
        let requested =
            sdformat::load_str(&runtime_xml, "runtime-stamp", &sdformat::NoResources).unwrap();
        prepare_model_template(&sim, &requested, &requested.worlds[0].models[0]).unwrap();
        let model = entity_by_id(sim.world(), id).unwrap();
        let link = entity_by_id(sim.world(), id + 1).unwrap();
        let handle = sim.world().get::<BodyHandle>(link).unwrap().id();
        queue_link_world_wrench(&mut sim, id + 1, [4., 0., 0.], [0.; 3]).unwrap();
        let observe = |sim: &Simulation| {
            let mut pairs: Vec<_> = sim
                .world()
                .iter_entities()
                .filter_map(|e| e.get::<EntityId>().map(|v| (v.get(), e.id())))
                .collect();
            pairs.sort_unstable();
            let mut bodies: Vec<_> = sim
                .world()
                .iter_entities()
                .filter_map(|e| {
                    e.get::<BodyHandle>().map(|v| {
                        (
                            v.id(),
                            sim.world().resource::<PhysicsBackend>().state(v.id()),
                        )
                    })
                })
                .collect();
            bodies.sort_by_key(|v| v.0);
            (
                pairs,
                bodies,
                next_entity_id(sim.world()).unwrap(),
                sim.world().iter_entities().count(),
                sim.epoch(),
                sim.elapsed(),
            )
        };
        let before = observe(&sim);
        for row in 0..2 {
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
            assert!(original_runtime_compound(sim.world(), id));
            assert!(matches!(
                prepare_model_template(&sim, &requested, &requested.worlds[0].models[0]),
                Err(WorldEditError::UnsupportedModel)
            ));
            assert_eq!(observe(&sim), before);
            if let Some(value) = origin {
                sim.world_mut().entity_mut(model).insert(value);
            }
            if let Some(value) = owner {
                sim.world_mut().entity_mut(link).insert(value);
            }
            prepare_model_template(&sim, &requested, &requested.worlds[0].models[0]).unwrap();
        }
        sim.step_exact(1).unwrap();
        let actual = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .unwrap();
        assert!(
            (actual.velocity[0] - 0.002).abs() < 1e-11,
            "pending wrench lost: {actual:?}"
        );
    }
    #[test]
    fn offset_compound_original_optional_center_and_dual_stamps_refuse_drift_before_inputs() {
        let xml = "<sdf version='1.12'><world name='w'><gravity>0 0 0</gravity><physics name='p' type='ode'><max_step_size>0.001</max_step_size></physics><model name='source'><pose>-100 0 10 0 0 0</pose><link name='body'><inertial><mass>2</mass><inertia><ixx>2</ixx><iyy>2</iyy><izz>2</izz></inertia></inertial><collision name='a'><geometry><sphere><radius>0.5</radius></sphere></geometry></collision><collision name='b'><pose>4 0 0 0 0 0</pose><geometry><box><size>1 1 1</size></box></geometry></collision><visual name='view'><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></world></sdf>";
        let sdf = sdformat::load_str(xml, "offset-stamps", &sdformat::NoResources).unwrap();
        let mut sim = load_world_with_sdf_step(
            xml,
            "offset-stamps",
            "w",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        sim.initialize_schedules().unwrap();
        let mut requested = sdf.worlds[0].models[0].clone();
        requested.links[0].inertial.pose.transform.translation = [0.25, -0.5, 0.75];
        let template = prepare_model_template(&sim, &sdf, &requested).unwrap();
        let pose = WorldPose {
            translation: [10., 20., 30.],
            rotation: [0.5; 4],
        };
        let id = spawn_model_template(&mut sim, &template, pose).unwrap();
        let model = entity_by_id(sim.world(), id).unwrap();
        let link = entity_by_id(sim.world(), id + 1).unwrap();
        let handle = sim.world().get::<BodyHandle>(link).unwrap().id();
        assert_eq!(compound_nodes(sim.world(), id).unwrap().len(), 5);
        assert_eq!(
            sim.world()
                .get::<RuntimeCompoundOrigin>(model)
                .unwrap()
                .0
                .center_of_mass,
            Some([0.25, -0.5, 0.75])
        );
        let original = sim
            .world()
            .get::<RuntimeCompoundOrigin>(model)
            .unwrap()
            .0
            .clone();
        let valid_state = model_state(&sim, id).unwrap();
        let runtime_xml = xml
            .replace("name='source'", &format!("name='runtime_{id}'"))
            .replace("name='view'", "name='visual'");
        let runtime_sdf = sdformat::load_str(
            &runtime_xml,
            "offset-runtime-stamps",
            &sdformat::NoResources,
        )
        .unwrap();
        let mut malformed = runtime_sdf.worlds[0].models[0].clone();
        malformed.links[0].inertial.pose.transform.rotation = [0.; 4];
        queue_link_world_wrench(&mut sim, id + 1, [2., 0., 0.], [0.; 3]).unwrap();
        let observe = |sim: &Simulation| {
            let mut ids: Vec<_> = sim
                .world()
                .iter_entities()
                .filter_map(|e| e.get::<EntityId>().map(|v| (v.get(), e.id())))
                .collect();
            ids.sort_unstable();
            (
                ids,
                sim.world().iter_entities().count(),
                next_entity_id(sim.world()).unwrap(),
                sim.epoch(),
                sim.elapsed(),
                sim.world().resource::<PhysicsBackend>().state(handle),
            )
        };
        for row in 0..6 {
            let com = sim
                .world_mut()
                .entity_mut(link)
                .take::<LinkCenterOfMass>()
                .unwrap();
            if row == 0 {
            } else if row == 1 {
                sim.world_mut()
                    .entity_mut(link)
                    .insert(LinkCenterOfMass([0.; 3]));
            } else {
                sim.world_mut().entity_mut(link).insert(com);
            }
            let origin = if row == 2 {
                sim.world_mut()
                    .entity_mut(model)
                    .take::<RuntimeCompoundOrigin>()
            } else {
                None
            };
            let owner = if row == 3 {
                sim.world_mut()
                    .entity_mut(link)
                    .take::<RuntimeCompoundLink>()
            } else {
                None
            };
            if row >= 4 {
                let stamp = Arc::new(CompoundOrigin {
                    world: CompoundMember::capture(sim.world(), original.world.entity),
                    model: CompoundMember::capture(sim.world(), model),
                    link: CompoundMember::capture(sim.world(), link),
                    members: original
                        .members
                        .iter()
                        .map(|member| CompoundMember::capture(sim.world(), member.entity))
                        .collect(),
                    collisions: original.collisions.clone(),
                    body: handle,
                    dynamic: true,
                    center_of_mass: if row == 4 {
                        Some([9., 0., 0.])
                    } else {
                        original.center_of_mass
                    },
                });
                sim.world_mut()
                    .entity_mut(model)
                    .insert(RuntimeCompoundOrigin(stamp.clone()));
                if row == 4 {
                    sim.world_mut()
                        .entity_mut(link)
                        .insert(RuntimeCompoundLink(stamp));
                }
            }
            let before = observe(&sim);
            assert!(
                matches!(model_state(&sim, id), Err(WorldEditError::UnsupportedModel)),
                "row={row}"
            );
            assert!(
                matches!(
                    compound_nodes(sim.world(), id),
                    Err(WorldEditError::UnsupportedModel)
                ),
                "row={row}"
            );
            assert!(
                matches!(
                    prepare_model_template(&sim, &runtime_sdf, &malformed),
                    Err(WorldEditError::UnsupportedModel)
                ),
                "row={row}"
            );
            assert!(
                matches!(
                    set_model_state(&mut sim, id, valid_state),
                    Err(WorldEditError::UnsupportedModel)
                ),
                "row={row}"
            );
            assert!(
                matches!(
                    remove_model(&mut sim, id),
                    Err(crate::RemoveError::InvalidPhysics)
                ),
                "row={row}"
            );
            assert!(
                matches!(
                    reset_loaded_world(&mut sim, |_| ()),
                    Err(SceneResetError::RuntimePhysics)
                ),
                "row={row}"
            );
            assert_eq!(observe(&sim), before);
            sim.world_mut().entity_mut(link).insert(com);
            if let Some(origin) = origin {
                sim.world_mut().entity_mut(model).insert(origin);
            }
            if let Some(owner) = owner {
                sim.world_mut().entity_mut(link).insert(owner);
            }
            if row >= 4 {
                sim.world_mut()
                    .entity_mut(model)
                    .insert(RuntimeCompoundOrigin(original.clone()));
                sim.world_mut()
                    .entity_mut(link)
                    .insert(RuntimeCompoundLink(original.clone()));
            }
            model_state(&sim, id).unwrap();
        }
        // A centered original's None remains distinct from an added Some(zero).
        let centered = prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
        let centered = spawn_model_template(
            &mut sim,
            &centered,
            WorldPose {
                translation: [100., 20., 30.],
                ..pose
            },
        )
        .unwrap();
        let centered_link = entity_by_id(sim.world(), centered + 1).unwrap();
        sim.world_mut()
            .entity_mut(centered_link)
            .insert(LinkCenterOfMass([0.; 3]));
        assert!(matches!(
            model_state(&sim, centered),
            Err(WorldEditError::UnsupportedModel)
        ));
        sim.world_mut()
            .entity_mut(centered_link)
            .remove::<LinkCenterOfMass>();
        model_state(&sim, centered).unwrap();
        sim.step_exact(1).unwrap();
        let actual = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(handle)
            .unwrap();
        assert!((actual.velocity[0] - 0.001).abs() < 1e-11);
    }
}
