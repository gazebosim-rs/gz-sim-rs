// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::collections::BTreeMap;

pub(crate) struct Plan {
    pub topic: String,
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}

pub(crate) fn is_plugin(plugin: &sdformat::Plugin) -> bool {
    plugin.name == "gz::sim::systems::VelocityControl"
        && plugin.filename == "gz-sim-velocity-control-system"
}

fn fields(plugin: &sdformat::Plugin) -> Result<BTreeMap<String, String>, ()> {
    if plugin.xml.len() > 65536 {
        return Err(());
    }
    let document = roxmltree::Document::parse(&plugin.xml).map_err(|_| ())?;
    let root = document.root_element();
    if root.tag_name().name() != "plugin"
        || root.namespaces().next().is_some()
        || root
            .attributes()
            .any(|a| a.namespace().is_some() || !matches!(a.name(), "name" | "filename"))
        || root.attribute("name") != Some(plugin.name.as_str())
        || root.attribute("filename") != Some(plugin.filename.as_str())
    {
        return Err(());
    }
    let mut values = BTreeMap::new();
    for child in root.children() {
        if child.is_comment()
            || child.is_text() && child.text().unwrap_or_default().trim().is_empty()
        {
            continue;
        }
        if !child.is_element()
            || child.namespaces().next().is_some()
            || child.attributes().len() != 0
            || child.children().any(|n| !n.is_text())
            || plugin.xml[child.range()].contains("<![CDATA[")
        {
            return Err(());
        }
        let name = child.tag_name().name();
        if !matches!(name, "topic" | "initial_linear" | "initial_angular") {
            return Err(());
        }
        let value = child.text().unwrap_or_default().trim();
        if value.is_empty() || values.insert(name.to_owned(), value.to_owned()).is_some() {
            return Err(());
        }
    }
    Ok(values)
}

fn vector(value: Option<&String>) -> Result<[f64; 3], ()> {
    let Some(value) = value else {
        return Ok([0.; 3]);
    };
    let values: Vec<_> = value.split_whitespace().map(str::parse::<f64>).collect();
    if values.len() != 3 {
        return Err(());
    }
    let mut result = [0.; 3];
    for (out, value) in result.iter_mut().zip(values) {
        *out = value.map_err(|_| ())?;
        if !out.is_finite() {
            return Err(());
        }
    }
    Ok(result)
}

pub(crate) fn parse(model: &Model) -> Result<Option<Plan>, LoadError> {
    let mut selected = model.plugins.iter().filter(|p| is_plugin(p));
    let Some(plugin) = selected.next() else {
        return Ok(None);
    };
    require(
        selected.next().is_none(),
        &model.scoped_name,
        "duplicate VelocityControl system",
    )?;
    let error = || LoadError::Unsupported {
        entity: model.scoped_name.clone(),
        feature: "VelocityControl configuration",
    };
    let fields = fields(plugin).map_err(|_| error())?;
    let topic = fields
        .get("topic")
        .cloned()
        .unwrap_or_else(|| format!("/model/{}/cmd_vel", model.name));
    // Keep the existing native-compatible topic subset; Server additionally
    // preflights canonical transport keys against every other family before IO.
    require(
        topic.starts_with('/')
            && topic != "/"
            && topic.len() <= 65535
            && !topic.chars().any(char::is_whitespace)
            && !["~", "//", "@", ":=", "*", "?", "#", "$"]
                .into_iter()
                .any(|p| topic.contains(p)),
        &model.scoped_name,
        "VelocityControl topic",
    )?;
    Ok(Some(Plan {
        topic,
        linear: vector(fields.get("initial_linear")).map_err(|_| error())?,
        angular: vector(fields.get("initial_angular")).map_err(|_| error())?,
    }))
}

impl Plan {
    pub(crate) fn validate(
        &self,
        sdf: &Sdf,
        model: &Model,
        sim: &Simulation,
        engine: Option<&str>,
        (top_level, is_static): (bool, bool),
        pose: Transform,
    ) -> Result<(), LoadError> {
        require(
            top_level
                && !is_static
                && model.links.len() == 1
                && model.models.is_empty()
                && model.joints.is_empty(),
            &model.scoped_name,
            "VelocityControl requires a top-level dynamic free single link",
        )?;
        require(
            sim.world()
                .iter_entities()
                .filter(|e| e.contains::<Controller>())
                .count()
                < MAX_CONTROLLERS,
            &model.scoped_name,
            "at most eight VelocityControl models",
        )?;
        require(
            !model
                .plugins
                .iter()
                .any(|p| crate::diff_drive::is_plugin(p) || crate::joint_controller::is_plugin(p)),
            &model.scoped_name,
            "conflicting VelocityControl owner",
        )?;
        let link = &model.links[0];
        let local = sdf.resolve_pose(
            &link.pose,
            &model.scoped_name,
            &model.scoped_name,
            &model.scoped_name,
        )?;
        let inertial = sdf.resolve_pose(
            &link.inertial.pose,
            &model.scoped_name,
            &link.scoped_name,
            &link.scoped_name,
        )?;
        require(
            identity(local)
                && inertial
                    .translation
                    .into_iter()
                    .all(|x| x.abs() <= POSE_TOLERANCE)
                && (link.collisions.len() == 1
                    || (engine == Some("rapier") && (2..=16).contains(&link.collisions.len()))),
            &model.scoped_name,
            "VelocityControl centered link",
        )?;
        for collision in &link.collisions {
            let local = sdf.resolve_pose(
                &collision.pose,
                &model.scoped_name,
                &link.scoped_name,
                &link.scoped_name,
            )?;
            require(
                (identity(local) || engine == Some("rapier"))
                    && matches!(
                        &collision.geometry,
                        Geometry::Box { .. }
                            | Geometry::Sphere { .. }
                            | Geometry::Cylinder { .. }
                            | Geometry::Capsule { .. }
                    ),
                &model.scoped_name,
                "VelocityControl centered primitive",
            )?;
        }
        require(
            world_vectors(pose.rotation, self.linear, self.angular).is_some(),
            &model.scoped_name,
            "VelocityControl unrepresentable initial command",
        )?;
        Ok(())
    }
}
