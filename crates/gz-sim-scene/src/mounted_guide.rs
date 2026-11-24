// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Authored stabilization frequency of an individual native mounted slider.
use super::*;

pub(super) fn is_plugin(p: &sdformat::Plugin) -> bool {
    p.name == "gz::sim::systems::MountedPrismaticGuide"
        && p.filename == "gz-sim-mounted-prismatic-guide-system"
}

pub(super) fn install(
    sim: &Simulation,
    world: &sdformat::World,
    backend: Option<&mut PhysicsBackend>,
) -> Result<(), LoadError> {
    let mut selected = world.plugins.iter().filter(|p| is_plugin(p));
    let Some(p) = selected.next() else {
        return Ok(());
    };
    let bad = || LoadError::Unsupported {
        entity: world.name.clone(),
        feature: "MountedPrismaticGuide requires unique named world-mounted prismatic robot joints and a finite positive natural_frequency_hz up to the native hard-joint frequency",
    };
    let engine = backend
        .and_then(|b| b.downcast_mut::<MultiFidelityBackend>())
        .ok_or_else(bad)?;
    let doc = roxmltree::Document::parse(&p.xml).map_err(|_| bad())?;
    let root = doc.root_element();
    if selected.next().is_some()
        || root.attributes().len() != 2
        || root.namespaces().next().is_some()
    {
        return Err(bad());
    }
    #[allow(deprecated)]
    let joints: BTreeMap<_, _> = sim
        .world()
        .iter_entities()
        .filter_map(|e| Some((e.get::<Name>()?.0.as_str(), e.get::<JointHandle>()?.id())))
        .collect();
    let mut requested = BTreeMap::new();
    for node in world_systems::content(root) {
        if !node.has_tag_name("natural_frequency_hz")
            || node.attributes().len() != 1
            || node.namespaces().next().is_some()
            || !node.children().all(|c| c.is_text())
        {
            return Err(bad());
        }
        let name = node.attribute("joint").ok_or_else(bad)?;
        let id = *joints.get(name).ok_or_else(bad)?;
        let hz = node
            .text()
            .unwrap_or("")
            .trim()
            .parse::<f64>()
            .map_err(|_| bad())?;
        if requested.insert(id, hz).is_some() {
            return Err(bad());
        }
    }
    if requested.is_empty() {
        return Err(bad());
    }
    for (joint, hz) in requested {
        engine.set_mounted_guide_frequency(joint, hz)?;
    }
    Ok(())
}
