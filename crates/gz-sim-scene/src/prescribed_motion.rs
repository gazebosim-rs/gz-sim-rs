// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! World-frame motion at the center of mass of a primitive or switchable assembly.
use crate::{LoadError, PhysicsEngine, require};
use sdformat::{Geometry, Model, Plugin};
mod trajectory;
pub(super) struct BoundaryMotion {
    pub twist: PrescribedMotion,
    pub path: Option<gz_physics::trajectory::TranslationSpline>,
}

#[derive(Clone, Copy, Default)]
pub(super) struct PrescribedMotion {
    pub linear_at_com: [f64; 3],
    pub angular: [f64; 3],
}

pub(super) fn is_plugin(plugin: &Plugin) -> bool {
    plugin.name == "gz::sim::systems::PrescribedMotion"
        && plugin.filename == "gz-sim-prescribed-motion-system"
}

pub(super) fn is_initial_velocity(plugin: &Plugin) -> bool {
    plugin.name == "gz::sim::systems::InitialVelocity"
        && plugin.filename == "gz-sim-initial-velocity-system"
}

/// A one-shot world-frame twist at the whole model center of mass. It seeds the native
/// construction and loaded reset, so contact impulses can change its velocity.
pub(super) fn initial_velocity(
    model: &Model,
    engine: PhysicsEngine,
    top_level: bool,
    coated_tree: bool,
) -> Result<Option<PrescribedMotion>, LoadError> {
    let Some(plugin) = model.plugins.iter().find(|p| is_initial_velocity(p)) else {
        return Ok(None);
    };
    let motion_plugin_count = model
        .plugins
        .iter()
        .filter(|p| !crate::contact_material::is_plugin(p))
        .count();
    let primitive = model.joints.is_empty()
        && model.links.len() == 1
        && motion_plugin_count == 1
        && !model.links[0].collisions.is_empty()
        && model.links[0].collisions.iter().all(|c| {
            matches!(
                c.geometry,
                Geometry::Sphere { .. }
                    | Geometry::Box { .. }
                    | Geometry::Cylinder { .. }
                    | Geometry::Capsule { .. }
            )
        });
    // The Rapier constructor independently validates both the angular axis
    // and zero anchor velocity for the prescribed initial spatial twist.
    let rotor = matches!(model.links.as_slice(), [l] if matches!(l.collisions.as_slice(),[c] if matches!(c.geometry,Geometry::Sphere{..})))
        && matches!(model.joints.as_slice(),[j] if j.parent=="world" && j.kind==sdformat::JointKind::Revolute)
        && motion_plugin_count == 1;
    let tree = !model.joints.is_empty()
        && model.plugins.iter().any(crate::articulation::is_plugin)
        && model
            .plugins
            .iter()
            .filter(|p| is_initial_velocity(p))
            .count()
            == 1;
    let weld = !model.joints.is_empty()
        && motion_plugin_count == 1
        && model
            .joints
            .iter()
            .all(|j| j.kind == sdformat::JointKind::Fixed && j.parent != "world");
    require(
        engine == PhysicsEngine::MultiFidelity
            && top_level
            && !model.is_static
            && model.models.is_empty()
            && (primitive || tree || rotor || weld || coated_tree && motion_plugin_count == 1)
            && model.links.iter().all(|l| !l.properties.kinematic),
        &model.scoped_name,
        "InitialVelocity requires a free multifidelity primitive, welded or switchable tree, or a single world-revolute sphere",
    )?;
    parse_twist(plugin)
        .map(Some)
        .ok_or_else(|| LoadError::Unsupported {
            entity: model.scoped_name.clone(),
            feature: "InitialVelocity configuration",
        })
}

pub(super) fn parse(
    model: &Model,
    engine: PhysicsEngine,
    top_level: bool,
) -> Result<Option<BoundaryMotion>, LoadError> {
    let mut plugins = model.plugins.iter().filter(|plugin| is_plugin(plugin));
    let Some(plugin) = plugins.next() else {
        return Ok(None);
    };
    require(
        engine == PhysicsEngine::MultiFidelity || engine.is_distributed(),
        &model.scoped_name,
        "PrescribedMotion requires the multifidelity or distributed engine",
    )?;
    require(
        top_level
            && !model.is_static
            && model.links.len() == 1
            && model.joints.is_empty()
            && model.models.is_empty(),
        &model.scoped_name,
        "PrescribedMotion requires one free top-level link",
    )?;
    require(
        plugins.next().is_none(),
        &model.scoped_name,
        "duplicate PrescribedMotion",
    )?;
    let collisions = &model.links[0].collisions;
    require(
        collisions.len() == 1
            && matches!(
                collisions[0].geometry,
                Geometry::Box { .. }
                    | Geometry::Sphere { .. }
                    | Geometry::Cylinder { .. }
                    | Geometry::Capsule { .. }
            ),
        &model.scoped_name,
        "PrescribedMotion requires one primitive collider",
    )?;
    if let Some(twist) = parse_twist(plugin) {
        return Ok(Some(BoundaryMotion { twist, path: None }));
    }
    let has_trajectory = roxmltree::Document::parse(&plugin.xml).is_ok_and(|doc| {
        doc.root_element()
            .children()
            .any(|node| node.has_tag_name("trajectory"))
    });
    if !has_trajectory {
        return Err(LoadError::Unsupported {
            entity: model.scoped_name.clone(),
            feature: "PrescribedMotion configuration",
        });
    }
    let invalid = |reason: &str| LoadError::TranslationPath {
        entity: model.scoped_name.clone(),
        reason: reason.to_owned(),
    };
    if engine != PhysicsEngine::MultiFidelity {
        return Err(invalid("trajectory needs the multifidelity engine"));
    }
    let path = trajectory::parse(plugin).map_err(invalid)?;
    Ok(Some(BoundaryMotion {
        twist: PrescribedMotion {
            linear_at_com: path.sample(std::time::Duration::ZERO).velocity,
            angular: [0.; 3],
        },
        path: Some(path),
    }))
}

fn parse_twist(plugin: &Plugin) -> Option<PrescribedMotion> {
    let document = roxmltree::Document::parse(&plugin.xml).ok()?;
    let root = document.root_element();
    if root.tag_name().name() != "plugin"
        || root.namespaces().next().is_some()
        || root.attributes().len() != 2
        || root.attribute("name") != Some(plugin.name.as_str())
        || root.attribute("filename") != Some(plugin.filename.as_str())
    {
        return None;
    }
    let mut linear = None;
    let mut angular = None;
    for child in root.children() {
        if child.is_comment() || child.is_text() && child.text()?.trim().is_empty() {
            continue;
        }
        if !child.is_element()
            || child.namespaces().next().is_some()
            || child.attributes().len() != 0
            || child.children().any(|node| !node.is_text())
        {
            return None;
        }
        let target = match child.tag_name().name() {
            "linear_velocity" => &mut linear,
            "angular_velocity" => &mut angular,
            _ => return None,
        };
        if target.replace(triple(child.text()?)?).is_some() {
            return None;
        }
    }
    Some(PrescribedMotion {
        linear_at_com: linear?,
        angular: angular?,
    })
}

fn triple(text: &str) -> Option<[f64; 3]> {
    let mut numbers = text.split_ascii_whitespace();
    let mut vector = [0.0_f64; 3];
    for component in &mut vector {
        *component = numbers.next()?.parse().ok()?;
        if !component.is_finite() {
            return None;
        }
    }
    numbers.next().is_none().then_some(vector)
}

/// Mass-weighted center including rotated link-frame inertial offsets. Use the
/// first center as a local origin so distant assemblies preserve small offsets.
pub(super) fn assembly_center(
    sdf: &sdformat::Sdf,
    model: &Model,
    world: &str,
) -> Result<glam::DVec3, LoadError> {
    use glam::{DQuat, DVec3};
    let mut total = 0.;
    let mut origin = None;
    let mut weighted = DVec3::ZERO;
    for link in &model.links {
        let name = &link.scoped_name;
        let pose = sdf.pose_of(name, world)?;
        let inertial = sdf.resolve_pose(&link.inertial.pose, &model.scoped_name, name, name)?;
        let center = DVec3::from(pose.translation)
            + DQuat::from_array(pose.rotation) * DVec3::from(inertial.translation);
        let mass = link.inertial.mass;
        require(
            mass.is_finite() && mass > 0. && center.is_finite(),
            name,
            "articulation mass and center",
        )?;
        weighted += (center - *origin.get_or_insert(center)) * mass;
        total += mass;
    }
    let center = origin.unwrap_or_default() + weighted / total;
    require(
        total.is_finite() && center.is_finite(),
        &model.scoped_name,
        "unrepresentable articulation center of mass",
    )?;
    Ok(center)
}
