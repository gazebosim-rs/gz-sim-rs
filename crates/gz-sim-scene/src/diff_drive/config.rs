// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Closed one-pair configuration for the built-in DiffDrive system.
use crate::{LoadError, require};
use sdformat::{JointKind, Model, Plugin, Transform};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone)]
pub(super) struct Parameters {
    pub left: String,
    pub right: String,
    pub radius: f64,
    pub separation: f64,
    pub rate: f64,
    pub linear: [f64; 2],
    pub angular: [f64; 2],
    pub wheel_speed: [f64; 2],
    pub wheel_force: [f64; 2],
    pub publication: super::DiffDriveConfiguration,
}

pub(super) fn is_plugin(plugin: &Plugin) -> bool {
    plugin.name == "gz::sim::systems::DiffDrive" && plugin.filename == "gz-sim-diff-drive-system"
}

fn error(model: &Model, feature: &'static str) -> LoadError {
    LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature,
    }
}

fn fields(plugin: &Plugin) -> Result<BTreeMap<&str, String>, ()> {
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
    let mut result = BTreeMap::new();
    for child in root.children() {
        if child.is_comment()
            || child.is_text() && child.text().unwrap_or_default().trim().is_empty()
        {
            continue;
        }
        if !child.is_element() || child.namespaces().next().is_some() || child.attributes().len() != 0
            || child.children().any(|node| !node.is_text())
            // TinyXML selects the first lexical text node; roxmltree merges CDATA.
            // This bounded configuration admits ordinary text, not lexical splits.
            || plugin.xml[child.range()].contains("<![CDATA[")
        {
            return Err(());
        }
        let name = child.tag_name().name();
        if !matches!(
            name,
            "left_joint"
                | "right_joint"
                | "wheel_radius"
                | "wheel_separation"
                | "odom_publish_frequency"
                | "topic"
                | "odom_topic"
                | "frame_id"
                | "child_frame_id"
                | "min_velocity"
                | "max_velocity"
                | "min_linear_velocity"
                | "max_linear_velocity"
                | "min_angular_velocity"
                | "max_angular_velocity"
        ) {
            return Err(());
        }
        let value = child
            .text()
            .unwrap_or_default()
            .split_ascii_whitespace()
            .collect::<Vec<_>>()
            .join(" ");
        if result.insert(name, value).is_some() {
            return Err(());
        }
    }
    Ok(result)
}

fn number(fields: &BTreeMap<&str, String>, name: &str, default: f64) -> Result<f64, ()> {
    let Some(text) = fields.get(name) else {
        return Ok(default);
    };
    let value: f64 = text.parse().map_err(|_| ())?;
    if value.is_finite() {
        Ok(value)
    } else {
        Err(())
    }
}

fn topic(value: &str) -> bool {
    value.starts_with('/')
        && value != "/"
        && value.len() <= 65535
        && !["~", " ", "//", "@", ":="]
            .into_iter()
            .any(|part| value.contains(part))
}

pub(super) fn parse(model: &Model) -> Result<Option<Parameters>, LoadError> {
    let mut plugins = model.plugins.iter().filter(|plugin| is_plugin(plugin));
    let Some(plugin) = plugins.next() else {
        return Ok(None);
    };
    require(
        plugins.next().is_none(),
        &model.scoped_name,
        "duplicate DiffDrive system",
    )?;
    require(
        !model.is_static && model.raw_namespace.is_none() && model.models.is_empty(),
        &model.scoped_name,
        "DiffDrive model domain",
    )?;
    let fields = fields(plugin).map_err(|()| error(model, "DiffDrive configuration"))?;
    let left = fields
        .get("left_joint")
        .ok_or_else(|| error(model, "DiffDrive wheel pair"))?;
    let right = fields
        .get("right_joint")
        .ok_or_else(|| error(model, "DiffDrive wheel pair"))?;
    require(
        !left.is_empty()
            && !right.is_empty()
            && left != right
            && !left.contains("::")
            && !right.contains("::")
            && model.links.len() == 3
            && model.joints.len() == 2
            && model
                .links
                .iter()
                .map(|link| &link.name)
                .collect::<BTreeSet<_>>()
                .len()
                == 3
            && model
                .joints
                .iter()
                .map(|joint| &joint.name)
                .collect::<BTreeSet<_>>()
                .len()
                == 2,
        &model.scoped_name,
        "DiffDrive wheel pair",
    )?;
    let (parent, parent_name) = model.canonical_link_and_relative_name();
    let parent = parent.ok_or_else(|| error(model, "DiffDrive wheel pair"))?;
    require(
        !parent_name.contains("::")
            && parent.pose.relative_to.is_empty()
            && parent.pose.transform == Transform::default(),
        &model.scoped_name,
        "DiffDrive wheel pair",
    )?;
    let mut positions = [[0.0; 3]; 2];
    let mut children = BTreeSet::new();
    let mut wheel_speed = [0.0; 2];
    let mut wheel_force = [0.0; 2];
    for (index, name) in [left, right].into_iter().enumerate() {
        let joint = model
            .joints
            .iter()
            .find(|joint| joint.name == *name)
            .ok_or_else(|| error(model, "DiffDrive wheel pair"))?;
        let child = model
            .links
            .iter()
            .find(|link| link.name == joint.child)
            .ok_or_else(|| error(model, "DiffDrive wheel pair"))?;
        require(
            joint.kind == JointKind::Revolute
                && joint.parent == parent_name
                && joint.child != parent_name
                && children.insert(&joint.child)
                && joint.axis2.is_none()
                && joint.pose.transform == Transform::default()
                && (joint.pose.relative_to.is_empty() || joint.pose.relative_to == joint.child)
                && child.pose.relative_to.is_empty()
                && child.pose.transform.rotation == [0.0, 0.0, 0.0, 1.0]
                && child
                    .pose
                    .transform
                    .translation
                    .into_iter()
                    .all(f64::is_finite),
            &model.scoped_name,
            "DiffDrive wheel pair",
        )?;
        let axis = joint
            .axis
            .as_ref()
            .ok_or_else(|| error(model, "DiffDrive wheel pair"))?;
        require(
            axis.xyz == [0.0, 1.0, 0.0]
                && axis.expressed_in.is_empty()
                && [axis.effort, axis.velocity]
                    .into_iter()
                    .all(|value| value.is_finite() && value > 0.0),
            &model.scoped_name,
            "DiffDrive wheel limits",
        )?;
        positions[index] = child.pose.transform.translation;
        wheel_speed[index] = axis.velocity;
        wheel_force[index] = axis.effort;
    }
    require(
        positions[0][0] == positions[1][0]
            && positions[0][2] == positions[1][2]
            && positions[0][1] > 0.0
            && positions[0][1] == -positions[1][1],
        &model.scoped_name,
        "DiffDrive wheel pair",
    )?;
    let radius =
        number(&fields, "wheel_radius", 0.2).map_err(|()| error(model, "DiffDrive geometry"))?;
    let separation = number(&fields, "wheel_separation", 1.0)
        .map_err(|()| error(model, "DiffDrive geometry"))?;
    require(
        super::model::Geometry::new(radius, separation).is_ok(),
        &model.scoped_name,
        "DiffDrive geometry",
    )?;
    let rate = number(&fields, "odom_publish_frequency", 50.0)
        .map_err(|()| error(model, "DiffDrive rate"))?;
    require(rate > 0.0, &model.scoped_name, "DiffDrive rate")?;
    let minimum = number(&fields, "min_velocity", f64::NEG_INFINITY)
        .map_err(|()| error(model, "DiffDrive velocity limits"))?;
    let maximum = number(&fields, "max_velocity", f64::INFINITY)
        .map_err(|()| error(model, "DiffDrive velocity limits"))?;
    let linear = [
        number(&fields, "min_linear_velocity", minimum),
        number(&fields, "max_linear_velocity", maximum),
    ];
    let angular = [
        number(&fields, "min_angular_velocity", minimum),
        number(&fields, "max_angular_velocity", maximum),
    ];
    let linear = [
        linear[0].map_err(|()| error(model, "DiffDrive velocity limits"))?,
        linear[1].map_err(|()| error(model, "DiffDrive velocity limits"))?,
    ];
    let angular = [
        angular[0].map_err(|()| error(model, "DiffDrive velocity limits"))?,
        angular[1].map_err(|()| error(model, "DiffDrive velocity limits"))?,
    ];
    // Native allows positive-only limits; M8 deliberately keeps zero/reset command stationary.
    require(
        [linear, angular]
            .into_iter()
            .all(|range| range[0] <= 0.0 && range[1] >= 0.0),
        &model.scoped_name,
        "DiffDrive velocity limits",
    )?;
    let command = fields
        .get("topic")
        .cloned()
        .unwrap_or_else(|| format!("/model/{}/cmd_vel", model.name));
    let odometry = fields
        .get("odom_topic")
        .cloned()
        .unwrap_or_else(|| format!("/model/{}/odometry", model.name));
    require(
        topic(&command) && topic(&odometry) && command != odometry,
        &model.scoped_name,
        "DiffDrive topics",
    )?;
    let frame = |key: &str, default: String| {
        fields
            .get(key)
            .filter(|value| !value.is_empty())
            .cloned()
            .unwrap_or(default)
    };
    Ok(Some(Parameters {
        left: left.clone(),
        right: right.clone(),
        radius,
        separation,
        rate,
        linear,
        angular,
        wheel_speed,
        wheel_force,
        publication: super::DiffDriveConfiguration {
            model_name: model.name.clone(),
            topic: command,
            odom_topic: odometry,
            frame_id: frame("frame_id", format!("{}/odom", model.name)),
            child_frame_id: frame("child_frame_id", format!("{}/{parent_name}", model.name)),
        },
    }))
}

#[cfg(test)]
#[path = "../../tests/private/diff_drive_config.rs"]
mod tests;
