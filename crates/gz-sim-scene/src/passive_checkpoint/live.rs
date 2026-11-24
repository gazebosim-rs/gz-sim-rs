// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Retain provenance while the ordinary server owns the simulation.
use super::*;
use bevy_ecs::{component::ComponentId, resource::IsResource};

impl PassivePoolScene {
    /// Arm a normal passive runtime for local operator checkpoints. Its ordinary
    /// lifecycle continues to own native changes. Only this path installs the tracker.
    pub fn into_checkpointable_runtime(mut self) -> Simulation {
        self.sim
            .world_mut()
            .insert_resource(self.state.templates.clone());
        self.sim.world_mut().insert_resource(self.state);
        self.sim
            .add_systems(Phase::Reset, reset_tracker.after(PhysicsReset));
        let revision = self.sim.schedule_revision();
        let schema = PassiveSchema::new(self.sim.world_mut(), revision);
        self.sim.world_mut().insert_resource(schema);
        self.sim
    }
}

#[derive(Resource)]
struct PassiveSchema {
    revision: u64,
    common: [ComponentId; 7],
    model: [ComponentId; 1],
    link: [ComponentId; 5],
    collision: [ComponentId; 2],
    visual: [ComponentId; 4],
}

impl PassiveSchema {
    fn new(world: &mut World, revision: u64) -> Self {
        Self {
            revision,
            common: [
                world.register_component::<EntityKind>(),
                world.register_component::<EntityId>(),
                world.register_component::<Name>(),
                world.register_component::<LocalName>(),
                world.register_component::<LocalPose>(),
                world.register_component::<WorldPose>(),
                world.register_component::<ChildOf>(),
            ],
            model: [world.register_component::<ModelAnchor>()],
            link: [
                world.register_component::<BodyHandle>(),
                world.register_component::<Velocity>(),
                world.register_component::<LinkProperties>(),
                world.register_component::<LinkCenterOfMass>(),
                world.register_component::<world_spawn::RuntimeBodyOwner>(),
            ],
            collision: [
                world.register_component::<AttachedPose>(),
                world.register_component::<CollisionDensity>(),
            ],
            visual: [
                world.register_component::<AttachedPose>(),
                world.register_component::<VisualGeometry>(),
                world.register_component::<VisualProperties>(),
                world.register_component::<VisualMaterial>(),
            ],
        }
    }
    fn check(&self, world: &World) -> bool {
        let children = world.component_id::<Children>();
        world
            .iter_entities()
            .filter(|e| !e.contains::<IsResource>())
            .all(|e| {
                let extra: &[ComponentId] = match e.get::<EntityKind>() {
                    Some(EntityKind::World) => &[],
                    Some(EntityKind::Model) => &self.model,
                    Some(EntityKind::Link) => &self.link,
                    Some(EntityKind::Collision) => &self.collision,
                    Some(EntityKind::Visual) => &self.visual,
                    _ => return false,
                };
                e.archetype().components().iter().all(|component| {
                    self.common.contains(component)
                        || extra.contains(component)
                        || Some(*component) == children
                })
            })
    }
}

fn reset_tracker(mut state: ResMut<CheckpointState>, backend: Res<PhysicsBackend>) {
    state.changes = Changes::default();
    state.epoch_start_native = backend
        .downcast_ref::<SceneBackend>()
        .expect("passive runtime")
        .next_body_handle();
}

pub(crate) fn same_pose(a: WorldPose, b: WorldPose) -> bool {
    a.translation.map(f64::to_bits) == b.translation.map(f64::to_bits)
        && a.rotation.map(f64::to_bits) == b.rotation.map(f64::to_bits)
}

// The live operator cut follows a normal Bevy physics publication. Refuse an
// unsynchronized caller instead of sealing transient world/local coordinates.
fn check_published(sim: &Simulation) -> Result<(), Error> {
    let world = sim.world();
    let backend = world
        .get_resource::<PhysicsBackend>()
        .ok_or(Error::Domain("missing live backend"))?;
    for e in world.iter_entities().filter(|e| e.contains::<EntityKind>()) {
        let pose = *e
            .get::<WorldPose>()
            .ok_or(Error::Domain("missing published pose"))?;
        if let Some(handle) = e.get::<BodyHandle>() {
            let state = backend
                .state(handle.id())
                .ok_or(Error::Domain("missing published native body"))?;
            let velocity = e
                .get::<Velocity>()
                .ok_or(Error::Domain("missing published link twist"))?;
            let linear = e
                .get::<LinkCenterOfMass>()
                .map_or(state.velocity, |center| {
                    (DVec3::from_array(state.velocity)
                        - center.velocity_shift(state.orientation, state.angular_velocity))
                    .to_array()
                });
            if !same_pose(
                pose,
                WorldPose {
                    translation: state.position,
                    rotation: state.orientation,
                },
            ) || velocity.linear.map(f64::to_bits) != linear.map(f64::to_bits)
                || velocity.angular.map(f64::to_bits) != state.angular_velocity.map(f64::to_bits)
            {
                return Err(Error::Domain(
                    "public link has not published its native state",
                ));
            }
        }
        if let Some(anchor) = e.get::<ModelAnchor>() {
            let link = gz_sim_core::entity_by_id(world, anchor.link)
                .and_then(|id| world.get::<WorldPose>(id))
                .ok_or(Error::Domain("missing model anchor"))?;
            if !same_pose(pose, link.compose(anchor.link_to_model)) {
                return Err(Error::Domain("unpublished model pose"));
            }
        }
        if let Some(parent) = e.get::<ChildOf>() {
            let parent = world
                .get::<WorldPose>(parent.parent())
                .ok_or(Error::Domain("missing published parent"))?;
            let local = e
                .get::<LocalPose>()
                .ok_or(Error::Domain("missing local pose"))?
                .0;
            let matches = if e.contains::<AttachedPose>() {
                same_pose(pose, parent.compose(local))
            } else {
                same_pose(local, parent.inverse().compose(pose))
            };
            if !matches {
                return Err(Error::Domain("public hierarchy awaits physics publication"));
            }
        }
    }
    Ok(())
}

/// Capture the paused, externally synchronized scene after its ordinary physics
/// publication. The enclosing server must fence accepted outstanding commands.
pub fn save_paused_runtime(sim: &mut Simulation, path: &Path) -> Result<(), Error> {
    let tracker = sim
        .world()
        .get_resource::<CheckpointState>()
        .ok_or(Error::Domain(
            "runtime was not armed for live checkpointing",
        ))?;
    let schema = sim
        .world()
        .get_resource::<PassiveSchema>()
        .ok_or(Error::Domain("missing passive live schema"))?;
    if !sim.is_paused() || schema.revision != sim.schedule_revision() {
        return Err(Error::Domain(
            "live capture requires the paused passive schedule",
        ));
    }
    if !schema.check(sim.world()) {
        return Err(Error::Domain(
            "foreign entity, component or uncommitted command in passive scene",
        ));
    }
    if !initial::matches_physics_owner(sim.world())
        || !sim
            .world()
            .get_resource::<initial::InitialScene>()
            .is_some_and(|initial| initial.passive_metadata_matches(sim.world()))
        || !tracker.clone_metadata_matches(sim.world())
    {
        return Err(Error::Domain(
            "live construction metadata differs from the passive SDF or template",
        ));
    }
    sim.committed_update_info()
        .map_err(|_| Error::Domain("public update information differs from the committed clock"))?;
    check_published(sim)?;
    let (manifest, version) = tracker.manifest(sim)?;
    manifest.seal(sim, path, version)
}

pub(crate) fn note_live_birth(
    sim: &mut Simulation,
    template: &ModelTemplate,
    model: u64,
    pose: WorldPose,
) {
    if !sim.world().contains_resource::<CheckpointState>() {
        return;
    }
    let tick = sim.world().resource::<UpdateInfo>().iteration;
    if let Some(mut state) = sim.world_mut().get_resource_mut::<CheckpointState>()
        && let Some(source) = state.source_for_visual(template.source_visual_id())
    {
        state.note_birth(model, source, tick, pose);
    }
}
pub(crate) fn note_live_retirement(sim: &mut Simulation, model: u64) {
    if let Some(mut state) = sim.world_mut().get_resource_mut::<CheckpointState>() {
        state.note_retirement(model);
    }
}
pub(crate) fn note_live_replacement(sim: &mut Simulation, model: u64) {
    if !sim.world().contains_resource::<CheckpointState>() {
        return;
    }
    let tick = sim.world().resource::<UpdateInfo>().iteration;
    if let Some(mut state) = sim.world_mut().get_resource_mut::<CheckpointState>() {
        state.note_replacement(model, tick);
    }
}
