// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resolve a passive SDF articulation into the same typed identities used by local workers.
mod backend;
pub use backend::{
    FencedHingeError, HingeWorkerOptions, load_distributed_hinge_world_with_options,
    load_distributed_hinge_world_with_transport, preview_selected_hinge_world,
    resume_fenced_hinge_world, resume_selected_hinge_world,
};
pub use gz_physics_distributed::moving::WorkerTransport;
pub(crate) mod construction;
mod grounded_robot;
pub(crate) use backend::remember_robot_program;
pub use grounded_robot::distributed_grounded_robot_spec;
pub use gz_physics_distributed::grounded_robot::GroundedRobotPool;
mod floating_tree;
pub(crate) use floating_tree::tree_and_sim_for_step;
mod grasp_fleet;
pub use backend::{
    DistributedFloatingTreeBackend, DistributedGraspFleetBackend, DistributedGraspTreeBackend,
    DistributedGroundedRobot, DistributedGroundedTreeBackend, DistributedHingeBackend,
    DistributedMotorTreeBackend, DistributedMovingHingeBackend, DistributedMultiSphereBackend,
    DistributedPayloadTreeBackend, DistributedSplitGraspBackend, DistributedStoppedTreeBackend,
    DistributedTreeBackend, GraspFleetLayout, LocalGroundedRobotSession, ManagedGraspFleet,
    SessionBackend, TreeSceneSession, WorkerTreeBackend, checkpoint_returned_grounded_robot,
    handoff_grounded_robot_to_workers, load_distributed_articulation_world,
    load_distributed_articulation_world_with_step, load_distributed_floating_tree_world,
    load_distributed_grasp_fleet_world, load_distributed_grasp_tree_world,
    load_distributed_grounded_robot_world, load_distributed_grounded_tree_world,
    load_distributed_hinge_world, load_distributed_motor_tree_world,
    load_distributed_payload_tree_world, load_distributed_split_grasp_world,
    load_distributed_stopped_tree_world, load_distributed_tree_world,
    load_returned_grounded_robot_world, rejoin_grounded_robot_workers,
    return_grounded_robot_to_local,
};
pub use floating_tree::{
    FloatingTreeRecipe, distributed_floating_tree_spec, distributed_grounded_tree_spec,
    distributed_motor_tree_spec, distributed_payload_tree_spec, distributed_stopped_tree_spec,
    distributed_tree_recipe,
};
pub use grasp_fleet::distributed_grasp_fleet_specs;
pub use gz_physics_distributed::floating_tree::{
    FloatingTreePool, FloatingTreeSpec, GraspFleetPlacement, GraspFleetPool, GraspFleetSpec,
    GraspPlacement, GraspTreePool, GraspTreeSpec, GroundedTreePool, GroundedTreeSpec,
    MotorTreePool, MotorTreeSpec, PayloadTreePool, PayloadTreeSpec, SplitGraspPool,
    StoppedTreePool, StoppedTreeSpec, TreeConstruction, TreePool,
};
pub use gz_physics_distributed::moving::{MovingPool, MovingSpec};
pub use gz_physics_distributed::multi_sphere::{
    ColdOrigin, ColdSelection, FenceReceipt, InspectedCut, MultiSpherePool, MultiSphereSpec,
    RecoveryMonitor, RecoveryPhase, RecoveryStatus, inspect_published_cut, inspect_published_path,
    inspect_restart_cut,
};

use super::*;
pub use gz_physics_distributed::articulation::{
    ArticulationError, ArticulationPool, ArticulationSpec,
};
use gz_physics_distributed::articulation::{ArticulationGroundSpec, ArticulationLinkSpec};

/// Admit a floating two-sphere hinge, optionally above a fixed horizontal
/// plane under downward gravity, with a 1 ms timestep. Stable link and joint IDs come from the ordinary Bevy loader.
pub fn distributed_hinge_spec(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<ArticulationSpec, LoadError> {
    match distributed_hinge_recipe(xml, source, world_name)? {
        HingeRecipe::Tree(spec) => Ok(*spec),
        HingeRecipe::Moving(_) | HingeRecipe::MultiSphere(_) => Err(LoadError::Unsupported {
            entity: world_name.into(),
            feature: "moving sphere requires a multi-owner hinge session",
        }),
    }
}

/// Construction-only description of an admitted local worker session.
pub enum HingeRecipe {
    Tree(Box<ArticulationSpec>),
    Moving(Box<MovingSpec>),
    MultiSphere(Box<MultiSphereSpec>),
}

/// Resolve the worker profile using the ordinary SDF construction and stable public IDs.
pub fn distributed_hinge_recipe(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<HingeRecipe, LoadError> {
    Ok(spec_and_sim(xml, source, world_name)?.0)
}

/// Resolve the floating hinge and its independently moving counterpart.
pub fn distributed_moving_hinge_spec(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<MovingSpec, LoadError> {
    match distributed_hinge_recipe(xml, source, world_name)? {
        HingeRecipe::Moving(spec) => Ok(*spec),
        HingeRecipe::Tree(_) | HingeRecipe::MultiSphere(_) => Err(LoadError::Unsupported {
            entity: world_name.into(),
            feature: "expected a hinge and a free dynamic sphere",
        }),
    }
}

/// Resolve a floating hinge and two independent spherical models, in authored order.
pub fn distributed_multi_sphere_spec(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<MultiSphereSpec, LoadError> {
    match distributed_hinge_recipe(xml, source, world_name)? {
        HingeRecipe::MultiSphere(spec) => Ok(*spec),
        _ => Err(LoadError::Unsupported {
            entity: world_name.into(),
            feature: "expected a hinge and two free dynamic spheres",
        }),
    }
}

fn spec_and_sim(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<(HingeRecipe, Simulation), LoadError> {
    construction::validate_xml(xml, world_name, false)?;
    let sdf = sdformat::load_str(xml, source, &sdformat::NoResources)?;
    let invalid = |feature| LoadError::Unsupported {
        entity: world_name.to_owned(),
        feature,
    };
    let [world] = sdf.worlds.as_slice() else {
        return Err(invalid("expected one world"));
    };
    require(
        world.name == world_name && world.plugins.is_empty() && world.joints.is_empty(),
        world_name,
        "expected the selected world without world joints or plugins",
    )?;
    let (model, floor, moving) = match world.models.as_slice() {
        [hinge] if !hinge.is_static && world.gravity == [0.; 3] => (hinge, None, &[][..]),
        [floor, hinge] if floor.is_static && !hinge.is_static => (hinge, Some(floor), &[][..]),
        [hinge, floor] if floor.is_static && !hinge.is_static => (hinge, Some(floor), &[][..]),
        [hinge, spheres @ ..]
            if (1..=2).contains(&spheres.len()) && articulation::moving_contact_profile(world) =>
        {
            (hinge, None, spheres)
        }
        _ => {
            return Err(invalid(
                "expected one floating hinge, optionally with a ground plane or up to two separate dynamic spheres",
            ));
        }
    };
    let [joint] = model.joints.as_slice() else {
        return Err(invalid("expected one revolute joint"));
    };
    require(
        model.links.len() == 2
            && model.models.is_empty()
            && model.plugins.len() == 1
            && articulation::is_plugin(&model.plugins[0])
            && joint.kind == sdformat::JointKind::Revolute,
        world_name,
        "expected two links and the switchable-articulation model plugin",
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
        "expected a 1 ms articulation step",
    )?;
    let public_id = |name: &str| construction::public_id(&sim, name);
    let joint_name = format!("{}::{}", model.scoped_name, joint.name);
    let spec_for = |model: &Model, name: &str| -> Result<_, LoadError> {
        construction::spherical_link(&sdf, model, name, world_name, &sim)
    };
    let ground = floor
        .map(|floor| construction::ground(&sdf, world, floor, world_name, &sim))
        .transpose()?;
    let spec = ArticulationSpec {
        ground,
        links: [
            spec_for(model, &joint.parent)?,
            spec_for(model, &joint.child)?,
        ],
        model_id: public_id(&model.scoped_name)?,
        joint_id: public_id(&joint_name)?,
        anchor: sdf.pose_of(&joint_name, world_name)?.translation,
        axis: sdf.resolve_axis(
            joint
                .axis
                .as_ref()
                .ok_or_else(|| invalid("missing revolute axis"))?,
            &joint_name,
            world_name,
        )?,
    };
    spec.validate()
        .map_err(|_| invalid("unrepresentable native hinge construction"))?;
    let sphere_spec = |sphere: &Model| -> Result<ArticulationLinkSpec, LoadError> {
        let [link] = sphere.links.as_slice() else {
            return Err(invalid("expected one free sphere link"));
        };
        let mut ball = spec_for(sphere, &link.name)?;
        let handle = gz_sim_core::entity_by_id(sim.world(), ball.id)
            .and_then(|e| sim.world().get::<BodyHandle>(e))
            .ok_or_else(|| invalid("missing ordinary dynamic sphere"))?;
        let state = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(handle.id())
            .ok_or_else(|| invalid("missing native sphere state"))?;
        ball.velocity = state.velocity;
        ball.angular_velocity = state.angular_velocity;
        require(
            ball.inertia_center == [0.; 3] && ball.inertia_orientation == [0., 0., 0., 1.],
            &sphere.scoped_name,
            "expected a centered sphere inertia",
        )?;
        Ok(ball)
    };
    let recipe = match moving {
        [] => {
            require(
                model.allow_auto_disable,
                world_name,
                "isolated hinge uses default activation",
            )?;
            HingeRecipe::Tree(Box::new(spec))
        }
        [sphere] => {
            let moving = MovingSpec {
                hinge: spec,
                sphere: sphere_spec(sphere)?,
            };
            moving
                .validate()
                .map_err(|_| invalid("unrepresentable moving hinge construction"))?;
            HingeRecipe::Moving(Box::new(moving))
        }
        [a, b] => {
            let pair = MultiSphereSpec {
                hinge: spec,
                spheres: [sphere_spec(a)?, sphere_spec(b)?],
            };
            pair.validate()
                .map_err(|_| invalid("unrepresentable two-sphere hinge construction"))?;
            HingeRecipe::MultiSphere(Box::new(pair))
        }
        _ => unreachable!("moving profile admits at most two companions"),
    };
    Ok((recipe, sim))
}
