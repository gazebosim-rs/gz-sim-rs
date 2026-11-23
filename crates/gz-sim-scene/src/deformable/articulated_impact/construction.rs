// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Construct independent rigid and material owners from one ordinary authored SDF.
use super::*;
use crate::distributed_hinge::{FloatingTreeRecipe, tree_and_sim_for_step};
use gz_physics_distributed::floating_tree::{FloatingTreeJointLimit, FloatingTreeSpec};
use gz_physics_xpbd::articulated_impact::TranslatingSphere;

#[derive(Clone, Debug, PartialEq)]
pub struct ArticulatedImpactBarrier {
    pub time: Duration,
    pub links: Vec<u64>,
    pub restitution: f64,
}
pub struct ArticulatedMaterialConstruction {
    pub tree: FloatingTreeSpec,
    pub limits: Vec<FloatingTreeJointLimit>,
    pub material_name: String,
    pub material: SoftBody,
    pub step: Duration,
    pub barriers: Vec<ArticulatedImpactBarrier>,
}
pub struct AutomaticArticulatedMaterialConstruction {
    pub tree: FloatingTreeSpec,
    pub limits: Vec<FloatingTreeJointLimit>,
    pub material_name: String,
    /// Immutable authored seed for reconstructing undisturbed free flight.
    pub material: SoftBody,
    pub step: Duration,
    pub links: Vec<u64>,
    pub restitution: f64,
}

/// Parse the intact ordinary scene and extract its exact controller-owned
/// cold material and worker-owned rigid tree. Stable ECS identities must agree.
pub fn distributed_material_contact_tree_spec(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<ArticulatedMaterialConstruction, LoadError> {
    let (result, automatic) = construction(xml, source, world_name, false)?;
    if automatic.is_some() || result.barriers.is_empty() {
        return Err(LoadError::Unsupported {
            entity: world_name.into(),
            feature: "scheduled distributed material requires authored public barriers",
        });
    }
    Ok(result)
}

/// Bind a single continuous, zero-gravity first impact and the actual public
/// step to the same ordinary SDF construction used by both physical owners.
pub fn distributed_automatic_material_contact_tree_spec(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<AutomaticArticulatedMaterialConstruction, LoadError> {
    let (r, automatic) = construction(xml, source, world_name, true)?;
    let automatic = automatic.ok_or_else(|| LoadError::Unsupported {
        entity: world_name.into(),
        feature: "expected detection=first_impact for the distributed material tree",
    })?;
    Ok(AutomaticArticulatedMaterialConstruction {
        tree: r.tree,
        limits: r.limits,
        material_name: r.material_name,
        material: r.material,
        step: r.step,
        links: automatic.links,
        restitution: automatic.restitution,
    })
}
fn construction(
    xml: &str,
    source: &str,
    world_name: &str,
    continuous: bool,
) -> Result<(ArticulatedMaterialConstruction, Option<ScheduledImpact>), LoadError> {
    let invalid = |feature| LoadError::Unsupported {
        entity: world_name.into(),
        feature,
    };
    let (rigid, _) = isolated_rigid_xml(xml, world_name)?;
    let intact = crate::load_world_with_sdf_step(
        xml,
        source,
        world_name,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let step = if continuous {
        intact.step_duration()
    } else {
        Duration::from_millis(1)
    };
    let (recipe, rigid_sim) = tree_and_sim_for_step(&rigid, source, world_name, step)?;
    let (tree, limits) = match recipe {
        FloatingTreeRecipe::Unlimited(s) => (s, vec![]),
        FloatingTreeRecipe::Stopped(s) => (s.tree, s.limits),
        _ => return Err(invalid("material coupling requires one isolated free tree")),
    };
    if tree.links.len() > 32 {
        return Err(invalid(
            "material contact mobility supports up to 32 rigid links",
        ));
    }
    for id in std::iter::once(tree.model_id)
        .chain(tree.links.iter().map(|l| l.id))
        .chain(tree.joints.iter().map(|j| j.joint_id))
    {
        let identity = |sim: &Simulation| {
            entity_by_id(sim.world(), id).and_then(|e| {
                Some((
                    sim.world().get::<Name>(e)?.0.clone(),
                    *sim.world().get::<EntityKind>(e)?,
                ))
            })
        };
        let a = identity(&rigid_sim);
        if a.is_none() || a != identity(&intact) {
            return Err(invalid(
                "rigid public identities differ in the ordinary material world",
            ));
        }
    }
    let soft = intact
        .world()
        .get_resource::<Deformables>()
        .ok_or_else(|| invalid("missing admitted material resource"))?;
    let (name, material) = soft
        .initial
        .first_key_value()
        .ok_or_else(|| invalid("missing authored material"))?;
    if soft.initial.len() != 1
        || !soft.surfaces.is_empty()
        || (continuous && (!soft.impact_barriers.is_empty() || soft.automatic_impact.is_none()))
        || (!continuous && (soft.impact_barriers.is_empty() || soft.automatic_impact.is_some()))
        || material.checkpoint_articulated_material().is_err()
    {
        return Err(invalid(
            "expected one isolated checkpointable material with the selected impact timing",
        ));
    }
    let mut barriers = vec![];
    for (time, event) in &soft.impact_barriers {
        if event.material != *name
            || event
                .links
                .iter()
                .any(|&id| !tree.links.iter().any(|l| l.id == id))
        {
            return Err(invalid(
                "authored impact targets must belong to the admitted tree",
            ));
        }
        barriers.push(ArticulatedImpactBarrier {
            time: *time,
            links: event.links.clone(),
            restitution: event.restitution,
        });
    }
    if let Some(event) = &soft.automatic_impact {
        if event.returning_step.is_some()
            || event.material != *name
            || event
                .links
                .iter()
                .any(|id| !tree.links.iter().any(|l| l.id == *id))
        {
            return Err(invalid(
                "automatic impact targets must belong to the admitted material tree",
            ));
        }
        let world = intact.world();
        let owners = event
            .links
            .iter()
            .map(|&id| loaded_link(world, id).map_err(|_| invalid("invalid native material site")))
            .collect::<Result<Vec<_>, _>>()?;
        let backend = world
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or_else(|| invalid("expected native multi-fidelity solver"))?;
        let velocity = backend.articulated_translation(&owners, step)?;
        let shapes = owners
            .iter()
            .map(|&id| {
                let (center, radius) = backend.articulated_sphere_geometry(id)?;
                Ok(TranslatingSphere {
                    sphere: ImpactSphere { center, radius },
                    velocity,
                })
            })
            .collect::<Result<Vec<_>, PhysicsError>>()?;
        material
            .first_translating_sphere_impact(&shapes, step)
            .map_err(|e| LoadError::Deformable {
                world: world_name.into(),
                reason: e.to_string(),
            })?;
    }
    Ok((
        ArticulatedMaterialConstruction {
            tree,
            limits,
            material_name: name.clone(),
            material: material.clone(),
            step: intact.step_duration(),
            barriers,
        },
        soft.automatic_impact.clone(),
    ))
}
pub(super) fn isolated_rigid_xml(xml: &str, world_name: &str) -> Result<(String, bool), LoadError> {
    let invalid = |feature| LoadError::Unsupported {
        entity: world_name.into(),
        feature,
    };
    if xml.len() > 1024 * 1024 {
        return Err(invalid("material articulation SDF exceeds 1 MiB"));
    }
    let doc = roxmltree::Document::parse(xml)
        .map_err(|_| invalid("invalid material articulation XML"))?;
    let mut plugins = doc.descendants().filter(|n| {
        n.has_tag_name("plugin")
            && n.parent_element()
                .is_some_and(|w| w.has_tag_name("world") && w.attribute("name") == Some(world_name))
            && n.attribute("name") == Some("gz::sim::systems::Deformables")
            && n.attribute("filename") == Some("gz-sim-deformables-system")
    });
    let plugin = plugins
        .next()
        .ok_or_else(|| invalid("expected one world Deformables plugin"))?;
    if plugins.next().is_some() {
        return Err(invalid("duplicate Deformables world plugin"));
    }
    let mut rigid = String::with_capacity(xml.len() - plugin.range().len());
    rigid.push_str(&xml[..plugin.range().start]);
    rigid.push_str(&xml[plugin.range().end..]);
    let material = plugin
        .children()
        .any(|node| node.has_tag_name("cable") || node.has_tag_name("cloth"));
    Ok((rigid, material))
}
