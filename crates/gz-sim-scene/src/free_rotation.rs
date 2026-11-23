// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicit ordinary-world opt-in for an independent rigid rotor.
use super::*;
pub(crate) fn is_plugin(plugin: &sdformat::Plugin) -> bool {
    plugin.name == "gz::sim::systems::FreeRotation"
        && plugin.filename == "gz-sim-free-rotation-system"
}
pub(crate) fn install(
    sim: &Simulation,
    world: &sdformat::World,
    links: &BTreeMap<String, u64>,
    backend: Option<&mut PhysicsBackend>,
) -> Result<(), LoadError> {
    let mut plugins = world.plugins.iter().filter(|plugin| is_plugin(plugin));
    let Some(plugin) = plugins.next() else {
        return Ok(());
    };
    let invalid = || LoadError::Unsupported {
        entity: world.name.clone(),
        feature: "FreeRotation requires a single named link in an ordinary rigid world",
    };
    if plugins.next().is_some() {
        return Err(invalid());
    }
    let xml = roxmltree::Document::parse(&plugin.xml).map_err(|_| invalid())?;
    let root = xml.root_element();
    let children: Vec<_> = root
        .children()
        .filter(|n| !(n.is_comment() || n.is_text() && n.text().unwrap_or("").trim().is_empty()))
        .collect();
    let [link] = children.as_slice() else {
        return Err(invalid());
    };
    if root.tag_name().name() != "plugin"
        || root.attributes().len() != 2
        || root.namespaces().next().is_some()
        || !link.has_tag_name("link")
        || link.attributes().len() != 0
        || link.namespaces().next().is_some()
        || link.children().any(|n| !n.is_text())
    {
        return Err(invalid());
    }
    let name = link.text().unwrap_or("").trim();
    let id = links
        .get(name)
        .and_then(|&id| gz_sim_core::entity_by_id(sim.world(), id))
        .and_then(|e| sim.world().get::<BodyHandle>(e))
        .ok_or_else(invalid)?
        .id();
    let backend = backend.ok_or_else(invalid)?;
    if let Some(native) = backend.downcast_mut::<MultiFidelityBackend>() {
        native.enable_momentum_preserving_rotation(id)?;
    } else if let Some(native) = backend.downcast_mut::<RapierBackend>() {
        native.enable_momentum_preserving_rotation(id)?;
    } else {
        return Err(invalid());
    }
    Ok(())
}
