// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Select fast independent bodies for Rapier's swept collision checks.
use super::*;
use std::collections::BTreeSet;

pub(super) fn is_plugin(p: &sdformat::Plugin) -> bool {
    p.name == "gz::sim::systems::ContinuousCollision"
        && p.filename == "gz-sim-continuous-collision-system"
}

pub(super) fn install(
    sim: &Simulation,
    world: &sdformat::World,
    links: &BTreeMap<String, u64>,
    backend: Option<&mut PhysicsBackend>,
) -> Result<(), LoadError> {
    let mut plugins = world.plugins.iter().filter(|p| is_plugin(p));
    let Some(plugin) = plugins.next() else {
        return Ok(());
    };
    let invalid = || LoadError::Unsupported {
        entity: world.name.clone(),
        feature: "ContinuousCollision requires distinct named dynamic links on the local multi-fidelity engine",
    };
    let engine = backend
        .and_then(|b| b.downcast_mut::<MultiFidelityBackend>())
        .ok_or_else(invalid)?;
    let doc = roxmltree::Document::parse(&plugin.xml).map_err(|_| invalid())?;
    let root = doc.root_element();
    if plugins.next().is_some()
        || root.attributes().len() != 2
        || root.namespaces().next().is_some()
    {
        return Err(invalid());
    }
    let mut linear_slop = None;
    let mut selected = BTreeSet::new();
    for node in root.children() {
        if node.is_comment() || node.is_text() && node.text().unwrap_or("").trim().is_empty() {
            continue;
        }
        if !node.is_element()
            || node.attributes().len() != 0
            || node.namespaces().next().is_some()
            || node.children().any(|n| !n.is_text())
        {
            return Err(invalid());
        }
        if node.has_tag_name("linear_slop") {
            let value = node
                .text()
                .unwrap_or("")
                .trim()
                .parse::<f64>()
                .map_err(|_| invalid())?;
            if !value.is_finite() || value <= 0. || linear_slop.replace(value).is_some() {
                return Err(invalid());
            }
            continue;
        }
        if !node.has_tag_name("link") {
            return Err(invalid());
        }
        let id = links
            .get(node.text().unwrap_or("").trim())
            .and_then(|&id| gz_sim_core::entity_by_id(sim.world(), id))
            .and_then(|entity| sim.world().get::<BodyHandle>(entity))
            .ok_or_else(invalid)?
            .id();
        if !selected.insert(id) {
            return Err(invalid());
        }
    }
    if selected.is_empty() {
        return Err(invalid());
    }
    if let Some(meters) = linear_slop {
        engine.set_contact_linear_slop(meters)?;
    }
    for body in selected {
        engine.enable_continuous_collision(body)?;
    }
    Ok(())
}
