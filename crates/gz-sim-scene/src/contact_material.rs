// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Rigid restitution on named collision links. The pinned SDF producer does
//! not yet expose the standard surface/bounce element.
use crate::{LoadError, PhysicsEngine};
use gz_physics_multifidelity::Restitution;
use sdformat::{Model, Plugin};
use std::collections::BTreeMap;

pub(super) fn is_plugin(plugin: &Plugin) -> bool {
    plugin.name == "gz::sim::systems::ContactMaterial"
        && plugin.filename == "gz-sim-contact-material-system"
}

pub(super) fn parse(
    model: &Model,
    engine: PhysicsEngine,
) -> Result<BTreeMap<String, Restitution>, LoadError> {
    let mut values = BTreeMap::new();
    let mut plugins = model.plugins.iter().filter(|p| is_plugin(p));
    let Some(plugin) = plugins.next() else {
        return Ok(values);
    };
    let invalid = || LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature: "ContactMaterial requires unique collision link names and restitution between zero and one in local multi-fidelity physics",
    };
    if plugins.next().is_some() || engine != PhysicsEngine::MultiFidelity {
        return Err(invalid());
    }
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
    for node in root.children() {
        if node.is_comment() || (node.is_text() && node.text().unwrap_or("").trim().is_empty()) {
            continue;
        }
        if !node.has_tag_name("restitution")
            || node.namespaces().next().is_some()
            || node.attributes().len() != 1
            || !node.children().all(|n| n.is_text())
        {
            return Err(invalid());
        }
        let link = node.attribute("link").ok_or_else(invalid)?;
        let value = node
            .text()
            .unwrap_or("")
            .trim()
            .parse()
            .ok()
            .and_then(|x| Restitution::new(x).ok())
            .ok_or_else(invalid)?;
        if !model
            .links
            .iter()
            .any(|l| l.name == link && !l.collisions.is_empty())
            || values.insert(link.to_owned(), value).is_some()
        {
            return Err(invalid());
        }
    }
    if values.is_empty() {
        return Err(invalid());
    }
    Ok(values)
}
