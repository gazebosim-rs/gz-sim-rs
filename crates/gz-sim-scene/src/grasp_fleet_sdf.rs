// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Authored pairing of articulated grippers and free payload models for worker placement.
use crate::{LoadError, require};
use sdformat::{Model, Plugin, World};
use std::collections::BTreeSet;

pub(super) fn is_plugin(plugin: &Plugin) -> bool {
    plugin.name == "gz::sim::systems::GraspFleet" && plugin.filename == "gz-sim-grasp-fleet-system"
}

/// Resolve each name against the same world used for ordinary physics. Pairing supplies
/// worker routing metadata. It does not filter contacts or change either body's dynamics.
pub(super) fn models(world: &World) -> Result<Option<Vec<(&Model, &Model)>>, LoadError> {
    let mut plugins = world.plugins.iter().filter(|p| is_plugin(p));
    let Some(plugin) = plugins.next() else {
        return Ok(None);
    };
    let invalid = |feature| LoadError::Unsupported {
        entity: world.name.clone(),
        feature,
    };
    require(
        plugins.next().is_none(),
        &world.name,
        "duplicate GraspFleet configuration",
    )?;
    let doc =
        roxmltree::Document::parse(&plugin.xml).map_err(|_| invalid("invalid GraspFleet XML"))?;
    let root = doc.root_element();
    require(
        root.has_tag_name("plugin")
            && root.namespaces().next().is_none()
            && root.attributes().len() == 2
            && root.attribute("name") == Some(&plugin.name)
            && root.attribute("filename") == Some(&plugin.filename),
        &world.name,
        "invalid GraspFleet attributes",
    )?;
    let mut used = BTreeSet::new();
    let mut pairs = Vec::new();
    let whitespace = |n: roxmltree::Node<'_, '_>| {
        n.is_comment() || n.is_text() && n.text().unwrap_or_default().trim().is_empty()
    };
    for entry in root.children().filter(|n| !whitespace(*n)) {
        require(
            entry.is_element()
                && entry.has_tag_name("assembly")
                && entry.namespaces().next().is_none()
                && entry.attributes().len() == 2
                && entry.children().all(whitespace),
            &world.name,
            "expected an empty GraspFleet assembly with gripper and payload model names",
        )?;
        let mut find = |attr| -> Result<&Model, LoadError> {
            let name = entry
                .attribute(attr)
                .ok_or_else(|| invalid("missing gripper or payload model name"))?;
            let m = world
                .models
                .iter()
                .find(|m| m.name == name)
                .ok_or_else(|| invalid("unknown GraspFleet model name"))?;
            require(
                used.insert(m.name.as_str()),
                &world.name,
                "GraspFleet model assigned more than once",
            )?;
            Ok(m)
        };
        let tree = find("gripper")?;
        let part = find("payload")?;
        require(
            !tree.is_static
                && tree.joints.len() >= 2
                && !part.is_static
                && part.joints.is_empty()
                && part.links.len() == 1,
            &world.name,
            "expected a gripper tree and a separate dynamic payload link",
        )?;
        pairs.push((tree, part));
    }
    require(
        (2..=8).contains(&pairs.len()),
        &world.name,
        "expected two to eight GraspFleet assemblies",
    )?;
    Ok(Some(pairs))
}

/// Two independently moving spheres use native rigid twist friction. Its total
/// transverse impulse is recovered from pre-solve momentum and post-solve contact receipts.
#[cfg(feature="distributed-physics")]
pub(super) fn install_observer(
    sim: &gz_sim_core::Simulation,
    backend: Option<&mut gz_physics::PhysicsBackend>,
    world: &World,
) -> Result<(), LoadError> {
    let Some(groups) = models(world)? else {
        return Ok(());
    };
    let [first, second] = groups.as_slice() else {
        return Ok(());
    };
    let Some(native) =
        backend.and_then(|b| b.downcast_mut::<gz_physics_multifidelity::MultiFidelityBackend>())
    else {
        return Ok(());
    };
    let find = |m: &Model| {
        sim.world()
            .iter_entities()
            .find_map(|e| {
                (e.get::<crate::Name>()?.0 == m.links[0].scoped_name)
                    .then(|| e.get::<crate::BodyHandle>().map(crate::BodyHandle::id))
                    .flatten()
            })
            .ok_or(gz_physics::PhysicsError::MissingBody)
    };
    let parts = [find(first.1)?, find(second.1)?];
    // Extra unassigned bodies remain legal in ordinary Gazebo; that wider world
    // cannot attribute all rigid-rigid shear through two scalar momentum balances.
    if world.models.len() == 4 && world.gravity == [0.; 3] {
        native.configure_grasp_payload_pair(parts)?;
    }
    Ok(())
}
