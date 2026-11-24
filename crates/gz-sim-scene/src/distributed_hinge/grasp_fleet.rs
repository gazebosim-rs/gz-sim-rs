// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resolve globally identified grippers from a single ordinary Bevy world.
use super::*;

pub fn distributed_grasp_fleet_specs(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<Vec<GraspTreeSpec>, LoadError> {
    Ok(fleet_and_sim(xml, source, world_name)?.0)
}

pub(super) fn fleet_and_sim(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<(Vec<GraspTreeSpec>, Simulation), LoadError> {
    construction::validate_fleet_xml(xml, world_name)?;
    let sdf = sdformat::load_str(xml, source, &sdformat::NoResources)?;
    let invalid = |feature| LoadError::Unsupported {
        entity: world_name.to_owned(),
        feature,
    };
    let [world] = sdf.worlds.as_slice() else {
        return Err(invalid("expected one grasp-fleet world"));
    };
    let pairs = crate::grasp_fleet_sdf::models(world)?
        .ok_or_else(|| invalid("expected a GraspFleet world plugin"))?;
    require(
        world.name == world_name
            && world.gravity == [0.; 3]
            && world.joints.is_empty()
            && world.models.len() == pairs.len() * 2
            && world.plugins.len() == 1,
        world_name,
        "distributed grasp fleet requires zero gravity and every model assigned exactly once",
    )?;
    let sim = load_world_with_sdf_step(
        xml,
        source,
        world_name,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    require(
        sim.step_duration() == Duration::from_millis(1),
        world_name,
        "expected a 1 ms grasp-fleet step",
    )?;
    let mut specs = Vec::with_capacity(pairs.len());
    for (model, part) in pairs {
        let recipe =
            floating_tree::resolve_recipe(&sdf, world, model, None, Some(part), world_name, &sim)?;
        let FloatingTreeRecipe::Payload(payload) = recipe else {
            return Err(invalid("missing assigned payload"));
        };
        specs.push(GraspTreeSpec { payload });
    }
    specs.sort_by_key(|s| s.payload.tree.model_id);
    Ok((specs, sim))
}
