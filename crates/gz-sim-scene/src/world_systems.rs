// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

pub(super) fn validate(world: &sdformat::World) -> Result<(), LoadError> {
    for plugin in &world.plugins {
        require(
            plugin.name == "gz::sim::systems::Physics"
                && plugin.filename == "gz-sim-physics-system",
            &world.name,
            "world plugins",
        )?;
        require(
            world.plugins.len() == 1,
            &world.name,
            "duplicate Physics configuration",
        )?;
        let invalid = || LoadError::Unsupported {
            entity: world.name.clone(),
            feature: "Physics system configuration",
        };
        let document = roxmltree::Document::parse(&plugin.xml).map_err(|_| invalid())?;
        let root = document.root_element();
        if root.tag_name().name() != "plugin"
            || root.namespaces().next().is_some()
            || root
                .attributes()
                .any(|a| a.namespace().is_some() || !matches!(a.name(), "name" | "filename"))
            || root
                .attribute("name")
                .map(|s| s.trim_matches([' ', '\t', '\r', '\n']))
                != Some(plugin.name.as_str())
            || root
                .attribute("filename")
                .map(|s| s.trim_matches([' ', '\t', '\r', '\n']))
                != Some(plugin.filename.as_str())
            || root.children().any(|node| {
                !(node.is_comment()
                    || node.is_text()
                        && node
                            .text()
                            .unwrap_or_default()
                            .trim_matches([' ', '\t', '\r', '\n'])
                            .is_empty())
            })
        {
            return Err(invalid());
        }
        // The selected backend is already installed once by load_world_configured.
        // Legacy metadata does not create a second system or load a library.
    }
    Ok(())
}

#[cfg(test)]
#[path = "../tests/private/world_systems.rs"]
mod tests;
