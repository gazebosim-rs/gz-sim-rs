// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::collections::{BTreeMap, BTreeSet};

/// Immutable model-local configuration; command state is separate and resets to zero.
#[derive(Component, Clone, Debug, PartialEq, Eq)]
#[component(immutable)]
pub struct JointForceConfiguration {
    model_name: String,
    joint_name: String,
}
impl JointForceConfiguration {
    pub fn model_name(&self) -> &str {
        &self.model_name
    }
    pub fn joint_name(&self) -> &str {
        &self.joint_name
    }
}

// TinyXML GetText uses the first lexical text node; roxmltree merges CDATA.
// Reject split values rather than silently selecting a different joint.
fn text_mode(xml: &str) -> Result<bool, ()> {
    use quick_xml::events::Event;
    let mut reader = quick_xml::Reader::from_str(xml);
    let (mut content, mut text, mut cdata) = (false, false, false);
    loop {
        match reader.read_event().map_err(|_| ())? {
            Event::Text(_) | Event::GeneralRef(_) => {
                if content && !text {
                    return Err(());
                }
                content = true;
                text = true;
            }
            Event::CData(_) => {
                if content {
                    return Err(());
                }
                content = true;
                cdata = true;
                text = false;
            }
            Event::Comment(_) | Event::Start(_) | Event::End(_) => text = false,
            Event::Eof => return if content { Ok(cdata) } else { Err(()) },
            _ => return Err(()),
        }
    }
}

fn joint_text(xml: &str, child: roxmltree::Node<'_, '_>) -> Result<String, ()> {
    let cdata = text_mode(&xml[child.range()])?;
    let name: String = child
        .children()
        .filter(|n| n.is_text())
        .filter_map(|n| n.text())
        .collect();
    Ok(if cdata {
        name.trim_matches([' ', '\t', '\r', '\n']).to_owned()
    } else {
        name.split_ascii_whitespace().collect::<Vec<_>>().join(" ")
    })
}

pub(super) fn parse(model: &Model) -> Result<Vec<JointForceConfiguration>, LoadError> {
    if model.plugins.is_empty() {
        return Ok(Vec::new());
    }
    let mut names = BTreeMap::new();
    for joint in &model.joints {
        *names.entry(joint.name.as_str()).or_insert(0usize) += 1;
    }
    let mut result = Vec::new();
    let mut seen = BTreeSet::new();
    for plugin in &model.plugins {
        require(
            plugin.name == "gz::sim::systems::ApplyJointForce"
                && plugin.filename == "gz-sim-apply-joint-force-system",
            &model.scoped_name,
            "model plugins",
        )?;
        let invalid = || LoadError::Unsupported {
            entity: model.scoped_name.clone(),
            feature: "ApplyJointForce configuration",
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
                .map(|value| value.trim_matches([' ', '\t', '\r', '\n']))
                != Some(plugin.name.as_str())
            || root
                .attribute("filename")
                .map(|value| value.trim_matches([' ', '\t', '\r', '\n']))
                != Some(plugin.filename.as_str())
        {
            return Err(invalid());
        }
        let children: Vec<_> = root
            .children()
            .filter(|node| {
                !(node.is_comment()
                    || node.is_text()
                        && node
                            .text()
                            .unwrap_or_default()
                            .trim_matches([' ', '\t', '\r', '\n'])
                            .is_empty())
            })
            .collect();
        if children.len() != 1 || !children[0].is_element() {
            return Err(invalid());
        }
        let child = children[0];
        if child.tag_name().name() != "joint_name"
            || child.namespaces().next().is_some()
            || child.attributes().len() != 0
            || child.children().any(|n| !n.is_text() && !n.is_comment())
        {
            return Err(invalid());
        }
        let name = joint_text(&plugin.xml, child).map_err(|_| invalid())?;
        if name.is_empty() || names.get(name.as_str()) != Some(&1) {
            return Err(invalid());
        }
        require(
            seen.insert(name.clone()),
            &model.scoped_name,
            "duplicate ApplyJointForce joint",
        )?;
        result.push(JointForceConfiguration {
            model_name: model.name.clone(),
            joint_name: name,
        });
    }
    Ok(result)
}

pub(super) fn install(
    sim: &mut Simulation,
    model: &Model,
    entity: Entity,
    backend: &PhysicsBackend,
    configurations: &[JointForceConfiguration],
) -> Result<(), LoadError> {
    if configurations.is_empty() {
        return Ok(());
    }
    let joints: BTreeMap<_, _> = sim
        .world()
        .get::<Children>(entity)
        .expect("loader constructed model children")
        .iter()
        .filter(|child| sim.world().get::<EntityKind>(*child) == Some(&EntityKind::Joint))
        .map(|child| {
            (
                sim.world()
                    .get::<Name>(child)
                    .expect("loader named joint")
                    .0
                    .clone(),
                child,
            )
        })
        .collect();
    for configuration in configurations {
        let name = format!("{}::{}", model.scoped_name, configuration.joint_name);
        let joint = joints[&name];
        let handle = sim
            .world()
            .get::<JointHandle>(joint)
            .expect("loader constructed joint handle");
        backend.validate_joint_effort(handle.id(), 0.0)?;
        sim.world_mut()
            .entity_mut(joint)
            .insert((configuration.clone(), JointForceCommand::default()));
    }
    Ok(())
}

#[cfg(test)]
#[path = "../tests/private/joint_forces.rs"]
mod tests;
