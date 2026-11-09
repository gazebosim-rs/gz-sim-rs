// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded world input state for the admitted free-link ApplyLinkWrench subset.
use super::*;
use gz_sim_core::entity_by_id;
use std::collections::VecDeque;

pub const APPLY_LINK_WRENCH_CAPACITY: usize = 256;

#[derive(Component, Clone, Debug, PartialEq, Eq)]
#[component(immutable)]
pub struct ApplyLinkWrenchConfiguration {
    pub topic: String,
    pub persistent_topic: String,
    pub clear_topic: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ApplyLinkWrenchMode {
    Once,
    Persistent,
}

#[derive(Debug, thiserror::Error)]
pub enum ApplyLinkWrenchError {
    #[error("ApplyLinkWrench is not configured")]
    Unconfigured,
    #[error("ApplyLinkWrench world identity or metadata changed")]
    InvalidConfiguration,
    #[error("ApplyLinkWrench target {0} is not an admitted centred Rapier link")]
    UnsupportedTarget(u64),
    #[error("ApplyLinkWrench buffer is full")]
    Capacity,
    #[error(transparent)]
    Input(#[from] InputError),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Target {
    id: u64,
    entity: Entity,
    body: gz_physics::BodyId,
}
#[derive(Clone, Copy)]
struct Entry {
    target: Target,
    force: [f64; 3],
    torque: [f64; 3],
}
#[derive(Resource)]
pub(super) struct Installed;
#[derive(Resource)]
struct State {
    world_id: u64,
    configuration: ApplyLinkWrenchConfiguration,
    epoch: u64,
    once: VecDeque<Entry>,
    persistent: Vec<Entry>,
    clear: VecDeque<Target>,
}

pub(super) struct Plan(ApplyLinkWrenchConfiguration);

pub(super) fn is_plugin(plugin: &sdformat::Plugin) -> bool {
    plugin.name == "gz::sim::systems::ApplyLinkWrench"
        && plugin.filename == "gz-sim-apply-link-wrench-system"
}
fn valid_topic(topic: &str) -> bool {
    !topic.is_empty()
        && topic.len() <= 65535
        && topic != "/"
        && !topic.chars().any(char::is_control)
        && ![" ", "~", "@", "//", ":="]
            .iter()
            .any(|part| topic.contains(part))
}
fn scalar(xml: &str, node: roxmltree::Node<'_, '_>) -> Option<String> {
    use quick_xml::events::Event;
    if node.attributes().len() != 0 || node.namespaces().next().is_some() {
        return None;
    }
    let mut reader = quick_xml::Reader::from_str(&xml[node.range()]);
    if !matches!(reader.read_event(), Ok(Event::Start(_))) {
        return None;
    }
    let mut content = false;
    loop {
        match reader.read_event().ok()? {
            Event::Text(_) | Event::GeneralRef(_) => content = true,
            Event::End(_) if content => break,
            _ => return None,
        }
    }
    if !matches!(reader.read_event(), Ok(Event::Eof)) {
        return None;
    }
    // Character references split lexical events, but remain one plain scalar.
    // Decode through the already validated XML tree, never by sanitizing topics.
    let text: String = node
        .children()
        .filter(|n| n.is_text())
        .filter_map(|n| n.text())
        .collect();
    Some(text.trim_matches([' ', '\t', '\r', '\n']).to_owned())
}
pub(super) fn parse(world: &sdformat::World) -> Result<Option<Plan>, LoadError> {
    let mut selected = world.plugins.iter().filter(|plugin| is_plugin(plugin));
    let Some(plugin) = selected.next() else {
        return Ok(None);
    };
    require(
        selected.next().is_none(),
        &world.name,
        "duplicate ApplyLinkWrench configuration",
    )?;
    let invalid = || LoadError::Unsupported {
        entity: world.name.clone(),
        feature: "ApplyLinkWrench configuration",
    };
    let document = roxmltree::Document::parse(&plugin.xml).map_err(|_| invalid())?;
    let root = document.root_element();
    if root.tag_name().name() != "plugin"
        || root.namespaces().next().is_some()
        || root.attributes().len() != 2
        || root.attribute("name") != Some(plugin.name.as_str())
        || root.attribute("filename") != Some(plugin.filename.as_str())
    {
        return Err(invalid());
    }
    let mut configuration = ApplyLinkWrenchConfiguration {
        topic: format!("/world/{}/wrench", world.name),
        persistent_topic: format!("/world/{}/wrench/persistent", world.name),
        clear_topic: format!("/world/{}/wrench/clear", world.name),
    };
    let mut seen = std::collections::BTreeSet::new();
    for node in root.children() {
        if node.is_comment()
            || node.is_text() && node.text().is_some_and(|text| text.trim().is_empty())
        {
            continue;
        }
        if !node.is_element() || !seen.insert(node.tag_name().name()) {
            return Err(invalid());
        }
        let value = scalar(&plugin.xml, node)
            .filter(|value| valid_topic(value))
            .ok_or_else(invalid)?;
        match node.tag_name().name() {
            "topic" => configuration.topic = value,
            "topic_persistent" => configuration.persistent_topic = value,
            "topic_clear" => configuration.clear_topic = value,
            _ => return Err(invalid()),
        }
    }
    Ok(Some(Plan(configuration)))
}
impl Plan {
    pub(super) fn validate(
        &self,
        world: &sdformat::World,
        engine: PhysicsEngine,
    ) -> Result<(), LoadError> {
        require(
            engine == PhysicsEngine::Rapier,
            &world.name,
            "ApplyLinkWrench requires Rapier",
        )?;
        let keys = [&self.0.topic, &self.0.persistent_topic, &self.0.clear_topic];
        require(
            keys.iter().all(|key| valid_topic(key))
                && keys
                    .into_iter()
                    .collect::<std::collections::BTreeSet<_>>()
                    .len()
                    == 3,
            &world.name,
            "ApplyLinkWrench distinct topics",
        )
    }
}
pub(super) fn build(plan: Plan, sim: &mut Simulation, world: Entity) -> Result<(), LoadError> {
    require(
        sim.world().get::<EntityKind>(world) == Some(&EntityKind::World)
            && sim.world().get::<EntityId>(world).is_some(),
        "world",
        "ApplyLinkWrench world identity",
    )?;
    sim.world_mut().entity_mut(world).insert(plan.0);
    Ok(())
}
fn configuration(
    world: &World,
) -> Result<Option<(u64, ApplyLinkWrenchConfiguration)>, ApplyLinkWrenchError> {
    let mut result = None;
    for entity in world.iter_entities() {
        let Some(value) = entity.get::<ApplyLinkWrenchConfiguration>() else {
            continue;
        };
        let id = entity
            .get::<EntityId>()
            .ok_or(ApplyLinkWrenchError::InvalidConfiguration)?
            .get();
        if result.is_some()
            || entity.get::<EntityKind>() != Some(&EntityKind::World)
            || entity_by_id(world, id) != Some(entity.id())
        {
            return Err(ApplyLinkWrenchError::InvalidConfiguration);
        }
        result = Some((id, value.clone()));
    }
    Ok(result)
}
fn target(world: &World, id: u64) -> Result<Target, ApplyLinkWrenchError> {
    let entity = entity_by_id(world, id).ok_or(InputError::MissingEntity(id))?;
    if world
        .get_resource::<gz_physics::PhysicsBackend>()
        .is_none_or(|backend| backend.engine() != "rapier")
        || world
            .get::<LinkCenterOfMass>(entity)
            .is_some_and(|center| !center.0.into_iter().all(|v| v.is_finite() && v == 0.0))
    {
        return Err(ApplyLinkWrenchError::UnsupportedTarget(id));
    }
    // Read-only existing admission: never validate by queueing a zero input.
    inputs::validate_world_wrench_batch(world, &[(id, [0.0; 3], [0.0; 3])])?;
    let body = world
        .get::<BodyHandle>(entity)
        .ok_or(InputError::InvalidPhysics)?
        .id();
    Ok(Target { id, entity, body })
}
fn live(world: &World, target: Target) -> bool {
    entity_by_id(world, target.id) == Some(target.entity)
        && world.get::<BodyHandle>(target.entity).map(BodyHandle::id) == Some(target.body)
}
fn eligible(world: &World) -> bool {
    world.iter_entities().any(|entity| {
        entity
            .get::<EntityId>()
            .is_some_and(|id| target(world, id.get()).is_ok())
    })
}
fn ordered(
    state: &State,
    candidate: Option<(ApplyLinkWrenchMode, Entry)>,
) -> Vec<(u64, [f64; 3], [f64; 3])> {
    let once = candidate
        .filter(|(mode, _)| *mode == ApplyLinkWrenchMode::Once)
        .map(|(_, entry)| entry);
    let persistent = candidate
        .filter(|(mode, _)| *mode == ApplyLinkWrenchMode::Persistent)
        .map(|(_, entry)| entry);
    state
        .once
        .iter()
        .copied()
        .chain(once)
        .chain(
            state
                .persistent
                .iter()
                .copied()
                .chain(persistent)
                .filter(|entry| !state.clear.contains(&entry.target)),
        )
        .map(|entry| (entry.target.id, entry.force, entry.torque))
        .collect()
}
fn ensure(sim: &Simulation) -> Result<(), ApplyLinkWrenchError> {
    let (world_id, config) =
        configuration(sim.world())?.ok_or(ApplyLinkWrenchError::Unconfigured)?;
    let Some(state) = sim.world().get_resource::<State>() else {
        return Err(ApplyLinkWrenchError::InvalidConfiguration);
    };
    if state.world_id != world_id || state.configuration != config || state.epoch != sim.epoch() {
        return Err(ApplyLinkWrenchError::InvalidConfiguration);
    }
    Ok(())
}
/// Add a bounded individual command. Validation errors preserve all earlier inputs.
pub fn queue_apply_link_wrench(
    sim: &mut Simulation,
    id: u64,
    force: [f64; 3],
    torque: [f64; 3],
    mode: ApplyLinkWrenchMode,
) -> Result<(), ApplyLinkWrenchError> {
    sim.validate_steps(1).map_err(InputError::Clock)?;
    configuration(sim.world())?.ok_or(ApplyLinkWrenchError::Unconfigured)?;
    let target = target(sim.world(), id)?;
    ensure(sim)?;
    let entry = Entry {
        target,
        force,
        torque,
    };
    let state = sim.world().resource::<State>();
    if match mode {
        ApplyLinkWrenchMode::Once => state.once.len(),
        ApplyLinkWrenchMode::Persistent => state.persistent.len(),
    } >= APPLY_LINK_WRENCH_CAPACITY
    {
        return Err(ApplyLinkWrenchError::Capacity);
    }
    if !force.into_iter().chain(torque).all(f64::is_finite) {
        return Err(InputError::InvalidValue.into());
    }
    // Keep individual entries and existing pending inputs in their exact order.
    // A queued clear removes persistent entries, never queued one-shots.
    let batch = ordered(state, Some((mode, entry)));
    inputs::validate_world_wrench_batch(sim.world(), &batch)?;
    let state = &mut *sim.world_mut().resource_mut::<State>();
    match mode {
        ApplyLinkWrenchMode::Once => state.once.push_back(entry),
        ApplyLinkWrenchMode::Persistent => state.persistent.push(entry),
    }
    Ok(())
}
/// Clear all persistent additions for an existing target on the next maintenance tick.
pub fn clear_apply_link_wrench(sim: &mut Simulation, id: u64) -> Result<(), ApplyLinkWrenchError> {
    sim.validate_steps(0).map_err(InputError::Clock)?;
    configuration(sim.world())?.ok_or(ApplyLinkWrenchError::Unconfigured)?;
    let target = target(sim.world(), id)?;
    ensure(sim)?;
    let mut state = sim.world_mut().resource_mut::<State>();
    if state.clear.len() == APPLY_LINK_WRENCH_CAPACITY {
        return Err(ApplyLinkWrenchError::Capacity);
    }
    state.clear.push_back(target);
    Ok(())
}
pub(super) fn apply(world: &mut World) {
    world.resource_scope(|world, mut state: Mut<State>| {
        let valid = entity_by_id(world, state.world_id).is_some_and(|entity| {
            world.get::<EntityKind>(entity) == Some(&EntityKind::World)
                && world.get::<ApplyLinkWrenchConfiguration>(entity) == Some(&state.configuration)
        });
        if !valid || state.epoch != world.resource::<UpdateInfo>().epoch {
            world
                .resource::<StepFault>()
                .report(ApplyLinkWrenchError::InvalidConfiguration);
            return;
        }
        state.once.retain(|entry| live(world, entry.target));
        state.persistent.retain(|entry| live(world, entry.target));
        while let Some(target) = state.clear.pop_front() {
            state.persistent.retain(|entry| entry.target != target);
        }
        if world.resource::<UpdateInfo>().dt.is_zero() {
            return;
        }
        let batch = ordered(&state, None);
        if let Err(error) = inputs::queue_world_wrench_batch(world, &batch) {
            world.resource::<StepFault>().report(error);
            return;
        }
        state.once.clear();
    });
}
pub(super) fn reset(world: &mut World) {
    let epoch = world.resource::<UpdateInfo>().epoch;
    if let Some(mut state) = world.get_resource_mut::<State>() {
        state.once.clear();
        state.persistent.clear();
        state.clear.clear();
        state.epoch = epoch;
    }
}
pub(super) fn install(sim: &mut Simulation) {
    if sim.world().contains_resource::<Installed>() || !eligible(sim.world()) {
        return;
    }
    let Ok(Some((world_id, configuration))) = configuration(sim.world()) else {
        return;
    };
    let epoch = sim.epoch();
    sim.world_mut().insert_resource(State {
        world_id,
        configuration,
        epoch,
        once: VecDeque::new(),
        persistent: Vec::new(),
        clear: VecDeque::new(),
    });
    sim.add_systems(Phase::PhysicsPrepare, apply.before(inputs::apply));
    sim.add_systems(Phase::Reset, reset);
    sim.world_mut().insert_resource(Installed);
}
pub(super) fn retire_if_unused(
    sim: &mut Simulation,
) -> Result<(), bevy_ecs::schedule::ScheduleError> {
    if !sim.world().contains_resource::<Installed>() {
        return Ok(());
    }
    sim.world_mut()
        .resource_scope(|world, mut state: Mut<State>| {
            state.once.retain(|entry| live(world, entry.target));
            state.persistent.retain(|entry| live(world, entry.target));
            state.clear.retain(|entry| live(world, *entry));
        });
    if eligible(sim.world()) {
        return Ok(());
    }
    sim.initialize_schedules()?;
    sim.remove_systems_in_set(Phase::PhysicsPrepare, apply)?;
    sim.remove_systems_in_set(Phase::Reset, reset)?;
    sim.world_mut().remove_resource::<State>();
    sim.world_mut().remove_resource::<Installed>();
    Ok(())
}

/// Whether this configured system currently owns its admitted Scene hooks.
pub fn apply_link_wrench_active(sim: &Simulation) -> bool {
    sim.world().contains_resource::<Installed>()
}

/// Cancel input state without changing time, physical state or configuration.
pub fn discard_apply_link_wrenches(sim: &mut Simulation) {
    reset(sim.world_mut());
}

pub(super) fn needs_activation(world: &World) -> bool {
    matches!(configuration(world), Ok(Some(_))) && eligible(world)
}

pub(super) fn has_eligible_outside(
    world: &World,
    excluded: &std::collections::BTreeSet<Entity>,
) -> bool {
    matches!(configuration(world), Ok(Some(_)))
        && world.iter_entities().any(|entity| {
            !excluded.contains(&entity.id())
                && entity
                    .get::<EntityId>()
                    .is_some_and(|id| target(world, id.get()).is_ok())
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn translated_com_metadata_refuses_commands_without_mutating_existing_queues() {
        let mut sim = load_world_with_sdf_step(
            include_str!("../../../examples/m35_apply_link_wrench.sdf"),
            "m35.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        )
        .unwrap();
        let entity = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<Name>()
                    .is_some_and(|name| name.0 == "default::rig::body")
            })
            .unwrap()
            .id();
        let id = sim.world().get::<EntityId>(entity).unwrap().get();
        queue_apply_link_wrench(
            &mut sim,
            id,
            [2.0, 0.0, 0.0],
            [0.0; 3],
            ApplyLinkWrenchMode::Once,
        )
        .unwrap();
        queue_apply_link_wrench(
            &mut sim,
            id,
            [3.0, 0.0, 0.0],
            [0.0; 3],
            ApplyLinkWrenchMode::Persistent,
        )
        .unwrap();
        let before = ordered(sim.world().resource::<State>(), None);
        let clock = (sim.epoch(), sim.elapsed());
        // Rapier loading already refuses this configuration. Corrupt only the
        // private metadata to exercise the input boundary; never step this world.
        sim.world_mut()
            .entity_mut(entity)
            .insert(LinkCenterOfMass([0.1, 0.0, 0.0]));
        for mode in [ApplyLinkWrenchMode::Once, ApplyLinkWrenchMode::Persistent] {
            assert!(matches!(
                queue_apply_link_wrench(&mut sim, id, [1.0; 3], [0.0; 3], mode),
                Err(ApplyLinkWrenchError::UnsupportedTarget(target)) if target == id
            ));
        }
        assert!(matches!(
            clear_apply_link_wrench(&mut sim, id),
            Err(ApplyLinkWrenchError::UnsupportedTarget(target)) if target == id
        ));
        let state = sim.world().resource::<State>();
        assert_eq!(ordered(state, None), before);
        assert_eq!(
            (state.once.len(), state.persistent.len(), state.clear.len()),
            (1, 1, 0)
        );
        assert_eq!((sim.epoch(), sim.elapsed()), clock);
        assert_eq!(
            sim.world().get::<LinkCenterOfMass>(entity).unwrap().0,
            [0.1, 0.0, 0.0]
        );
    }
}
