// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Closed native velocity/ABS-position plugin records for one revolute chain.
//! Field meanings and default topics follow gz-sim 71c60db3 JointController.cc
//! and JointPositionController.cc. PID, actuator and multi-DOF modes are deferred.
use crate::{LoadError, require};
use sdformat::{JointKind, Model, Plugin, Transform};
use std::collections::{BTreeMap, BTreeSet};

use super::{JointControllerConfiguration, JointControllerMode};

#[derive(Clone)]
pub(super) struct Parameters {
    pub joint: String,
    pub max_velocity: f64,
    pub max_force: f64,
    pub initial_target: f64,
    pub publication: JointControllerConfiguration,
}

pub(super) fn is_plugin(plugin: &Plugin) -> bool {
    (plugin.name == "gz::sim::systems::JointController"
        && plugin.filename == "gz-sim-joint-controller-system")
        || (plugin.name == "gz::sim::systems::JointPositionController"
            && plugin.filename == "gz-sim-joint-position-controller-system")
}

fn error(model: &Model, feature: &'static str) -> LoadError {
    LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature,
    }
}

fn fields(plugin: &Plugin, mode: JointControllerMode) -> Result<BTreeMap<&str, String>, ()> {
    let document = roxmltree::Document::parse(&plugin.xml).map_err(|_| ())?;
    let root = document.root_element();
    if root.tag_name().name() != "plugin"
        || root.namespaces().next().is_some()
        || root.attributes().any(|attribute| {
            attribute.namespace().is_some() || !matches!(attribute.name(), "name" | "filename")
        })
        || root.attribute("name") != Some(plugin.name.as_str())
        || root.attribute("filename") != Some(plugin.filename.as_str())
    {
        return Err(());
    }
    let mut fields = BTreeMap::new();
    for child in root.children() {
        if child.is_comment()
            || child.is_text() && child.text().unwrap_or_default().trim().is_empty()
        {
            continue;
        }
        if !child.is_element() || child.namespaces().next().is_some() || child.attributes().len() != 0
            || child.children().any(|node| !node.is_text())
            // TinyXML selects the first lexical text node; roxmltree merges CDATA.
            // Keep the initial subset to ordinary, unsplit scalar text.
            || plugin.xml[child.range()].contains("<![CDATA[")
        {
            return Err(());
        }
        let name = child.tag_name().name();
        let allowed = matches!(name, "joint_name" | "topic")
            || match mode {
                JointControllerMode::Velocity => {
                    matches!(name, "use_force_commands" | "initial_velocity")
                }
                JointControllerMode::Position => matches!(
                    name,
                    "use_velocity_commands" | "cmd_max" | "initial_position" | "joint_index"
                ),
            };
        if !allowed {
            return Err(());
        }
        let value = child
            .text()
            .unwrap_or_default()
            .split_ascii_whitespace()
            .collect::<Vec<_>>()
            .join(" ");
        if fields.insert(name, value).is_some() {
            return Err(());
        }
    }
    Ok(fields)
}

fn number(fields: &BTreeMap<&str, String>, name: &str, default: Option<f64>) -> Result<f64, ()> {
    let Some(text) = fields.get(name) else {
        return default.ok_or(());
    };
    let value: f64 = text.parse().map_err(|_| ())?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(())
    }
}

fn valid_topic(topic: &str) -> bool {
    topic.starts_with('/')
        && topic != "/"
        && topic.len() <= 65535
        && !["~", " ", "//", "@", ":="]
            .into_iter()
            .any(|part| topic.contains(part))
}

fn chain(model: &Model) -> Result<(), LoadError> {
    require(
        !model.is_static && model.models.is_empty() && model.raw_namespace.is_none(),
        &model.scoped_name,
        "joint controller model domain",
    )?;
    require(
        model.links.len() == 2
            && model.joints.len() == 2
            && model
                .links
                .iter()
                .map(|link| &link.name)
                .collect::<BTreeSet<_>>()
                .len()
                == 2
            && model
                .joints
                .iter()
                .map(|joint| &joint.name)
                .collect::<BTreeSet<_>>()
                .len()
                == 2,
        &model.scoped_name,
        "joint controller chain",
    )?;
    let mut roots = model.joints.iter().filter(|joint| joint.parent == "world");
    let root = roots
        .next()
        .ok_or_else(|| error(model, "joint controller chain"))?;
    require(
        roots.next().is_none(),
        &model.scoped_name,
        "joint controller chain",
    )?;
    let (canonical, canonical_name) = model.canonical_link_and_relative_name();
    require(
        canonical.is_some() && canonical_name == root.child,
        &model.scoped_name,
        "joint controller chain",
    )?;
    let child = model
        .joints
        .iter()
        .find(|joint| joint.name != root.name)
        .expect("two distinct joint names include exactly one world root");
    require(
        child.parent == root.child && child.child != root.child,
        &model.scoped_name,
        "joint controller chain",
    )?;
    for link in &model.links {
        require(
            !link.name.is_empty()
                && !link.name.contains("::")
                && link.pose.relative_to.is_empty()
                && link.pose.transform.rotation == [0.0, 0.0, 0.0, 1.0]
                && link
                    .pose
                    .transform
                    .translation
                    .into_iter()
                    .all(f64::is_finite),
            &model.scoped_name,
            "joint controller frames",
        )?;
    }
    for joint in &model.joints {
        let axis = joint
            .axis
            .as_ref()
            .ok_or_else(|| error(model, "joint controller frames"))?;
        require(
            !joint.name.is_empty()
                && !joint.name.contains("::")
                && joint.kind == JointKind::Revolute
                && joint.axis2.is_none()
                && model.links.iter().any(|link| link.name == joint.child)
                && joint.pose.transform == Transform::default()
                && (joint.pose.relative_to.is_empty() || joint.pose.relative_to == joint.child)
                && axis.xyz == [0.0, 0.0, 1.0]
                && axis.expressed_in.is_empty(),
            &model.scoped_name,
            "joint controller frames",
        )?;
    }
    Ok(())
}

pub(super) fn parse(model: &Model) -> Result<Option<Vec<Parameters>>, LoadError> {
    let mut plugins = model.plugins.iter().filter(|plugin| is_plugin(plugin));
    let Some(first) = plugins.next() else {
        return Ok(None);
    };
    let second = plugins.next();
    require(
        plugins.next().is_none(),
        &model.scoped_name,
        "joint controller count",
    )?;
    chain(model)?;
    let mut names = BTreeSet::new();
    let mut topics = BTreeSet::new();
    let mut result = Vec::with_capacity(1 + usize::from(second.is_some()));
    for plugin in std::iter::once(first).chain(second) {
        let mode = if plugin.name == "gz::sim::systems::JointController" {
            JointControllerMode::Velocity
        } else {
            JointControllerMode::Position
        };
        let fields =
            fields(plugin, mode).map_err(|()| error(model, "joint controller configuration"))?;
        let name = fields
            .get("joint_name")
            .ok_or_else(|| error(model, "joint controller joint name"))?;
        let joint = model
            .joints
            .iter()
            .find(|joint| joint.name == *name)
            .ok_or_else(|| error(model, "joint controller joint name"))?;
        require(
            names.insert(name.clone()),
            &model.scoped_name,
            "overlapping joint controllers",
        )?;
        let axis = joint
            .axis
            .as_ref()
            .expect("chain admission requires a revolute axis");
        require(
            [axis.effort, axis.velocity]
                .into_iter()
                .all(|value| value.is_finite() && value > 0.0),
            &model.scoped_name,
            "joint controller limits",
        )?;
        let (initial_target, max_velocity, suffix) = match mode {
            JointControllerMode::Velocity => {
                require(
                    fields
                        .get("use_force_commands")
                        .is_none_or(|value| matches!(value.as_str(), "false" | "0")),
                    &model.scoped_name,
                    "joint controller mode",
                )?;
                let initial = number(&fields, "initial_velocity", Some(0.0))
                    .map_err(|()| error(model, "initial joint controller target"))?;
                require(
                    initial.abs() <= axis.velocity,
                    &model.scoped_name,
                    "initial joint controller target",
                )?;
                (initial, axis.velocity, "cmd_vel")
            }
            JointControllerMode::Position => {
                require(
                    fields
                        .get("use_velocity_commands")
                        .is_some_and(|value| matches!(value.as_str(), "true" | "1"))
                        && fields
                            .get("joint_index")
                            .is_none_or(|value| value.parse::<u32>() == Ok(0)),
                    &model.scoped_name,
                    "joint controller mode",
                )?;
                let cap = number(&fields, "cmd_max", None)
                    .map_err(|()| error(model, "joint controller limits"))?;
                require(cap > 0.0, &model.scoped_name, "joint controller limits")?;
                let initial = number(&fields, "initial_position", Some(0.0))
                    .map_err(|()| error(model, "initial joint controller target"))?;
                (initial, cap.min(axis.velocity), "0/cmd_pos")
            }
        };
        let topic = fields
            .get("topic")
            .cloned()
            .unwrap_or_else(|| format!("/model/{}/joint/{name}/{suffix}", model.name));
        require(
            valid_topic(&topic) && topics.insert(topic.clone()),
            &model.scoped_name,
            "joint controller topics",
        )?;
        result.push(Parameters {
            joint: name.clone(),
            max_velocity,
            max_force: axis.effort,
            initial_target,
            publication: JointControllerConfiguration {
                model_name: model.name.clone(),
                joint_name: name.clone(),
                topic,
                mode,
            },
        });
    }
    Ok(Some(result))
}

#[cfg(test)]
#[path = "../../tests/private/joint_controller_config.rs"]
mod tests;
