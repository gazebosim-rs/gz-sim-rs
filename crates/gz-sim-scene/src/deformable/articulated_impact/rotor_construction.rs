// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A finite spinning native pad and independently checkpointable material.
use super::*;
use crate::distributed_hinge::construction::{public_id, spherical_link, validate_xml};
use gz_physics_distributed::floating_tree::RotorPadSpec;
use gz_physics_multifidelity::ArticulatedSweep;
use gz_physics_xpbd::articulated_impact::{SphereMotion, SweptSphere};

pub struct RotorMaterialConstruction {
    pub pad: RotorPadSpec,
    pub material_name: String,
    pub material: SoftBody,
    pub step: Duration,
    pub restitution: f64,
}
/// Construct the worker seed from the actual ordinary SDF/Bevy owners. The
/// cold twist is read after the native world revolute and centered sphere have
/// been constructed, and the same native swept-motion certificate is required.
pub fn distributed_rotor_material_contact_spec(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<RotorMaterialConstruction, LoadError> {
    let invalid = |feature| LoadError::Unsupported {
        entity: world_name.into(),
        feature,
    };
    let (rigid, material) = super::construction::isolated_rigid_xml(xml, world_name)?;
    if !material {
        return Err(invalid("managed rotor material requires a cable or cloth"));
    }
    validate_xml(&rigid, world_name, true)?;
    let sdf = sdformat::load_str(&rigid, source, &sdformat::NoResources)?;
    let [world] = sdf.worlds.as_slice() else {
        return Err(invalid("expected one isolated rotor world"));
    };
    let [model] = world.models.as_slice() else {
        return Err(invalid("expected one rotor model"));
    };
    let [link] = model.links.as_slice() else {
        return Err(invalid("expected one rotor sphere link"));
    };
    let [joint] = model.joints.as_slice() else {
        return Err(invalid("expected one rotor joint"));
    };
    if world.name != world_name
        || world.gravity != [0.; 3]
        || !world.joints.is_empty()
        || !world.plugins.is_empty()
        || model.is_static
        || !model.models.is_empty()
        || !matches!(model.plugins.as_slice(),[p] if crate::prescribed_motion::is_initial_velocity(p))
        || link.properties.kinematic
        || joint.kind != sdformat::JointKind::Revolute
        || joint.parent != "world"
        || sdf.resolve_name(&model.scoped_name, &joint.child)? != link.scoped_name
    {
        return Err(invalid(
            "rotor material requires one zero-gravity spherical dynamic link on a passive world revolute with initial velocity",
        ));
    }
    let ax = joint
        .axis
        .as_ref()
        .ok_or_else(|| invalid("missing rotor axis"))?;
    if !(ax.lower.is_finite() && ax.upper.is_finite() && ax.lower < 0. && ax.upper > 0.) {
        return Err(invalid(
            "rotor requires finite revolute stops containing its cold position",
        ));
    }
    let sim = crate::load_world_with_sdf_step(
        xml,
        source,
        world_name,
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )?;
    let joint_name = format!("{}::{}", model.scoped_name, joint.name);
    let mut pad_link = spherical_link(&sdf, model, &link.name, world_name, &sim)?;
    let world = sim.world();
    let id = loaded_link(world, pad_link.id).map_err(|_| invalid("missing native rotor sphere"))?;
    let physics = world.resource::<PhysicsBackend>();
    let native = physics
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or_else(|| invalid("expected multi-fidelity rotor owner"))?;
    let cold = physics
        .state(id)
        .ok_or_else(|| invalid("missing native rotor state"))?;
    pad_link.position = cold.position;
    pad_link.orientation = cold.orientation;
    pad_link.velocity = cold.velocity;
    pad_link.angular_velocity = cold.angular_velocity;
    let step = sim.step_duration();
    let ArticulatedSweep::Revolution {
        pivot,
        axis,
        angular_speed,
    } = native.articulated_sweep(&[id], step)?
    else {
        return Err(invalid("rotor requires nonzero certified angular motion"));
    };
    let (center, radius) = native.articulated_sphere_geometry(id)?;
    let pad = RotorPadSpec {
        model_id: public_id(&sim, &model.scoped_name)?,
        link: pad_link,
        joint_id: public_id(&sim, &joint_name)?,
        anchor: sdf.pose_of(&joint_name, world_name)?.translation,
        axis: sdf.resolve_axis(ax, &joint_name, world_name)?,
        lower: ax.lower,
        upper: ax.upper,
    };
    let soft = world
        .get_resource::<Deformables>()
        .ok_or_else(|| invalid("missing ordinary material owner"))?;
    let (name, material) = soft
        .initial
        .first_key_value()
        .ok_or_else(|| invalid("missing ordinary material"))?;
    let event = soft
        .automatic_impact
        .as_ref()
        .ok_or_else(|| invalid("rotor requires first_impact detection"))?;
    if soft.initial.len() != 1
        || !soft.surfaces.is_empty()
        || !soft.impact_barriers.is_empty()
        || event.material != *name
        || event.returning_step.is_some()
        || event.links.as_slice() != [pad.link.id]
        || material.config().friction != 0.
        || material.config().gravity != [0.; 3]
        || material.checkpoint_articulated_material().is_err()
    {
        return Err(invalid(
            "rotor requires one checkpointable frictionless cable or cloth with a single automatic first impact",
        ));
    }
    material
        .first_swept_sphere_impact(
            &[SweptSphere {
                sphere: ImpactSphere { center, radius },
                motion: SphereMotion::Revolve {
                    pivot,
                    axis,
                    angular_speed,
                },
            }],
            step,
        )
        .map_err(|e| LoadError::Deformable {
            world: world_name.into(),
            reason: e.to_string(),
        })?;
    Ok(RotorMaterialConstruction {
        pad,
        material_name: name.clone(),
        material: material.clone(),
        step,
        restitution: event.restitution,
    })
}
