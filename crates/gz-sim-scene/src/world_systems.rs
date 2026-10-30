// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn content<'a, 'input>(
    node: roxmltree::Node<'a, 'input>,
) -> impl Iterator<Item = roxmltree::Node<'a, 'input>> {
    node.children().filter(|node| {
        !(node.is_comment()
            || node.is_text()
                && node
                    .text()
                    .unwrap_or_default()
                    .trim_matches([' ', '\t', '\r', '\n'])
                    .is_empty())
    })
}

fn engine_metadata(xml: &str, root: roxmltree::Node<'_, '_>) -> bool {
    let mut payload = content(root);
    let Some(engine) = payload.next() else {
        return true;
    };
    if payload.next().is_some()
        || !engine.has_tag_name("engine")
        || engine.attributes().len() != 0
        || engine.namespaces().next().is_some()
    {
        return false;
    }
    let mut payload = content(engine);
    let Some(filename) = payload.next() else {
        return false;
    };
    if payload.next().is_some()
        || !filename.has_tag_name("filename")
        || filename.attributes().len() != 0
        || filename.namespaces().next().is_some()
    {
        return false;
    }
    // Admit one ordinary XML text node; do not concatenate CDATA/comments into
    // a different filename than native TinyXML's first lexical text node.
    use quick_xml::events::Event;
    let mut reader = quick_xml::Reader::from_str(&xml[filename.range()]);
    let plain = matches!(reader.read_event(), Ok(Event::Start(_)))
        && matches!(reader.read_event(), Ok(Event::Text(_)))
        && matches!(reader.read_event(), Ok(Event::End(_)))
        && matches!(reader.read_event(), Ok(Event::Eof));
    plain
        && matches!(
            filename
                .text()
                .unwrap_or_default()
                .trim_matches([' ', '\t', '\r', '\n']),
            "gz-physics-dartsim-plugin" | "gz-physics-mujoco-plugin" | "gz-physics-tpe-plugin"
        )
}

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
            || !engine_metadata(&plugin.xml, root)
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
