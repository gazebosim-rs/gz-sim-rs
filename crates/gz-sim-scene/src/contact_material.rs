// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Local material overrides on named collision links.
use crate::{LoadError, PhysicsEngine};
use gz_physics_multifidelity::Restitution;
use sdformat::{Model, Plugin};
use std::collections::BTreeMap;

pub(super) fn is_plugin(plugin: &Plugin) -> bool {
    plugin.name == "gz::sim::systems::ContactMaterial"
        && plugin.filename == "gz-sim-contact-material-system"
}

#[derive(Default)]
pub(super) struct LinkMaterial {
    pub restitution: Option<Restitution>,
    pub normal_stiffness: Option<f64>,
}

pub(super) fn parse(
    model: &Model,
    engine: PhysicsEngine,
) -> Result<BTreeMap<String, LinkMaterial>, LoadError> {
    let mut values = BTreeMap::new();
    let mut plugins = model.plugins.iter().filter(|p| is_plugin(p));
    let Some(plugin) = plugins.next() else {
        return Ok(values);
    };
    let invalid = || LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature: "ContactMaterial requires one value per property and collision link: restitution in [0, 1], or positive normal_stiffness in N/m on a reduced prismatic pad",
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
        if !node.is_element()
            || node.namespaces().next().is_some()
            || node.attributes().len() != 1
            || !node.children().all(|n| n.is_text())
        {
            return Err(invalid());
        }
        let link = node.attribute("link").ok_or_else(invalid)?;
        if !model
            .links
            .iter()
            .any(|l| l.name == link && !l.collisions.is_empty())
        {
            return Err(invalid());
        }
        let value = node
            .text()
            .unwrap_or("")
            .trim()
            .parse::<f64>()
            .map_err(|_| invalid())?;
        let material = values.entry(link.to_owned()).or_default();
        match node.tag_name().name() {
            "restitution" if material.restitution.is_none() => {
                material.restitution = Some(Restitution::new(value).map_err(|_| invalid())?);
            }
            "normal_stiffness"
                if material.normal_stiffness.is_none()
                    && value.is_finite()
                    && value > 0.
                    && model
                        .plugins
                        .iter()
                        .any(crate::joints::is_scalar_tree_plugin) =>
            {
                material.normal_stiffness = Some(value);
            }
            _ => return Err(invalid()),
        }
    }
    if values.is_empty() {
        return Err(invalid());
    }
    Ok(values)
}
