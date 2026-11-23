// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Shared validation, identifiers and spherical link construction for distributed SDF profiles.
use super::*;

pub(crate) fn validate_xml(xml: &str, world_name: &str, tree: bool) -> Result<(), LoadError> {
    validate_document(xml, world_name, tree, false, false)
}
pub(super) fn validate_fleet_xml(xml: &str, world_name: &str) -> Result<(), LoadError> {
    validate_document(xml, world_name, true, true, false)
}
pub(super) fn validate_grounded_robot_xml(xml: &str, world_name: &str) -> Result<(), LoadError> {
    validate_document(xml, world_name, true, false, true)
}
fn validate_document(
    xml: &str,
    world_name: &str,
    tree: bool,
    fleet: bool,
    robot: bool,
) -> Result<(), LoadError> {
    require(
        xml.len() <= 1024 * 1024,
        world_name,
        "articulation SDF exceeds 1 MiB",
    )?;
    let doc = roxmltree::Document::parse(xml).map_err(|_| LoadError::Unsupported {
        entity: world_name.into(),
        feature: "invalid articulation XML",
    })?;
    // Admit authored inputs that the durable worker recipe can completely represent.
    require(
        doc.descendants().filter(|n| n.is_element()).all(|n| {
            n.tag_name().namespace().is_none()
                && ((tree
                    && match n.tag_name().name() {
                        "assembly" if fleet => n.parent_element().is_some_and(|p| {
                            p.has_tag_name("plugin")
                                && p.attribute("name") == Some("gz::sim::systems::GraspFleet")
                        }),
                        "footsteps" if robot => n.parent_element().is_some_and(|p| {
                            p.has_tag_name("plugin")
                                && p.attribute("name") == Some("gz::sim::systems::JointPosture")
                        }),
                        "force" if robot => n.parent_element().is_some_and(|p| {
                            p.has_tag_name("plugin")
                                && p.attribute("name")
                                    == Some("gz::sim::systems::ScheduledLinkForce")
                        }),
                        "balance" | "actuator" => n.parent_element().is_some_and(|p| {
                            p.has_tag_name("plugin")
                                && p.attribute("name") == Some("gz::sim::systems::JointPosture")
                        }),
                        "self_collide" => {
                            n.parent_element().is_some_and(|p| p.has_tag_name("model"))
                        }
                        "exact_revolute_locks" => {
                            n.parent_element().is_some_and(|p| p.has_tag_name("plugin"))
                        }
                        "limit" => {
                            n.parent_element().is_some_and(|p| {
                                p.has_tag_name("axis")
                                    && p.children().filter(|x| x.has_tag_name("limit")).count() == 1
                            }) && ["lower", "upper"]
                                .iter()
                                .all(|s| n.children().filter(|x| x.has_tag_name(*s)).count() == 1)
                        }
                        "lower" | "upper" => {
                            n.parent_element().is_some_and(|p| p.has_tag_name("limit"))
                        }
                        _ => false,
                    })
                    || matches!(
                        n.tag_name().name(),
                        "sdf"
                            | "world"
                            | "model"
                            | "link"
                            | "linear_velocity"
                            | "angular_velocity"
                            | "allow_auto_disable"
                            | "joint"
                            | "visual"
                            | "material"
                            | "ambient"
                            | "diffuse"
                            | "specular"
                            | "emissive"
                            | "collision"
                            | "geometry"
                            | "static"
                            | "plane"
                            | "normal"
                            | "size"
                            | "box"
                            | "sphere"
                            | "radius"
                            | "physics"
                            | "max_step_size"
                            | "real_time_factor"
                            | "gravity"
                            | "inertial"
                            | "mass"
                            | "inertia"
                            | "ixx"
                            | "iyy"
                            | "izz"
                            | "ixy"
                            | "ixz"
                            | "iyz"
                            | "pose"
                            | "parent"
                            | "child"
                            | "axis"
                            | "xyz"
                            | "plugin"
                            | "frame"
                    ))
        }),
        world_name,
        "unsupported element in isolated articulation SDF",
    )?;
    Ok(())
}

pub(crate) fn public_id(sim: &Simulation, name: &str) -> Result<u64, LoadError> {
    sim.world()
        .iter_entities()
        .find_map(|entity| {
            (entity.get::<Name>()?.0 == name)
                .then(|| entity.get::<EntityId>().map(|id| id.get()))
                .flatten()
        })
        .ok_or_else(|| LoadError::Unsupported {
            entity: name.into(),
            feature: "missing authored entity identity",
        })
}

pub(crate) fn spherical_link(
    sdf: &Sdf,
    model: &Model,
    name: &str,
    world_name: &str,
    sim: &Simulation,
) -> Result<ArticulationLinkSpec, LoadError> {
    let invalid = |feature| LoadError::Unsupported {
        entity: name.into(),
        feature,
    };
    let name = sdf.resolve_name(&model.scoped_name, name)?;
    let link = model
        .links
        .iter()
        .find(|l| l.scoped_name == name)
        .ok_or_else(|| invalid("expected a floating link endpoint"))?;
    let [collision] = link.collisions.as_slice() else {
        return Err(invalid("expected one sphere per link"));
    };
    let Geometry::Sphere { radius } = collision.geometry else {
        return Err(invalid("expected spherical link geometry"));
    };
    require(
        identity(sdf.resolve_pose(&collision.pose, &model.scoped_name, &name, &name)?),
        &name,
        "expected a centered sphere",
    )?;
    let pose = sdf.pose_of(&name, world_name)?;
    let inertia = sdf.resolve_pose(&link.inertial.pose, &model.scoped_name, &name, &name)?;
    Ok(ArticulationLinkSpec {
        id: public_id(sim, &name)?,
        mass: link.inertial.mass,
        radius,
        position: pose.translation,
        orientation: pose.rotation,
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
        inertia_center: inertia.translation,
        inertia_orientation: inertia.rotation,
        inertia_tensor: link.inertial.inertia,
    })
}

/// Resolve the sole authored fixed horizontal plane and its world gravity.
pub(super) fn ground(
    sdf: &Sdf,
    world: &sdformat::World,
    floor: &Model,
    world_name: &str,
    sim: &Simulation,
) -> Result<ArticulationGroundSpec, LoadError> {
    let invalid = |feature| LoadError::Unsupported {
        entity: floor.scoped_name.clone(),
        feature,
    };
    let [link] = floor.links.as_slice() else {
        return Err(invalid("expected one fixed ground link"));
    };
    require(
        floor.models.is_empty()
            && floor.joints.is_empty()
            && floor.plugins.is_empty()
            && world.gravity[0] == 0.
            && world.gravity[1] == 0.
            && world.gravity[2].is_finite()
            && world.gravity[2] < 0.,
        &floor.scoped_name,
        "expected one fixed ground plane and vertical downward gravity",
    )?;
    let [collision] = link.collisions.as_slice() else {
        return Err(invalid("expected one fixed plane collision"));
    };
    let Geometry::Plane { normal, size } = collision.geometry else {
        return Err(invalid("expected a fixed plane collision"));
    };
    let pose = sdf.pose_of(&link.scoped_name, world_name)?;
    require(
        floor.is_static
            && identity(sdf.resolve_pose(
                &collision.pose,
                &floor.scoped_name,
                &link.scoped_name,
                &link.scoped_name,
            )?)
            && normal == [0., 0., 1.]
            && pose.rotation == [0., 0., 0., 1.]
            && pose.translation[0] == 0.
            && pose.translation[1] == 0.
            && pose.translation[2].is_finite()
            && size.iter().all(|v| v.is_finite() && *v > 0.),
        &link.scoped_name,
        "expected a horizontal +Z plane at (0, 0, height) with positive finite size",
    )?;
    Ok(ArticulationGroundSpec {
        link_id: public_id(sim, &link.scoped_name)?,
        height: pose.translation[2],
        gravity_z: world.gravity[2],
        size,
    })
}
