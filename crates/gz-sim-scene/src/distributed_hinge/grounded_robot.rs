// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_distributed::grounded_robot::{GroundedRobotLinkSpec, GroundedRobotSpec};
use gz_physics_multifidelity::MAX_GROUNDED_ROBOT_LINKS;

/// Resolve a free, fully dynamic, torque-actuated tree of centered boxes on one horizontal plane.
/// Model, links and joints retain the identities and native insertion order of the ordinary load.
pub fn distributed_grounded_robot_spec(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<GroundedRobotSpec, LoadError> {
    grounded_robot_and_sim(xml, source, world_name).map(|(spec, _)| spec)
}
pub(super) fn grounded_robot_and_sim(
    xml: &str,
    source: &str,
    world_name: &str,
) -> Result<(GroundedRobotSpec, Simulation), LoadError> {
    construction::validate_grounded_robot_xml(xml, world_name)?;
    let sdf = sdformat::load_str(xml, source, &sdformat::NoResources)?;
    let error = |feature| LoadError::Unsupported {
        entity: world_name.into(),
        feature,
    };
    let [world] = sdf.worlds.as_slice() else {
        return Err(error("expected one grounded robot world"));
    };
    let [a, b] = world.models.as_slice() else {
        return Err(error("expected an articulated robot and a fixed ground"));
    };
    let (model, floor) = if a.is_static { (b, a) } else { (a, b) };
    require(
        world.name == world_name
            && world.joints.is_empty()
            && world.plugins.iter().all(|p| {
                crate::joint_posture::is_plugin(p) || crate::scheduled_link_force::is_plugin(p)
            })
            && model.plugins.len() == 1
            && articulation::is_plugin(&model.plugins[0])
            && !model.is_static
            && !model.self_collide
            && model.models.is_empty()
            && (3..=MAX_GROUNDED_ROBOT_LINKS).contains(&model.links.len())
            && model.joints.len() + 1 == model.links.len()
            && model
                .joints
                .iter()
                .all(|j| j.kind == sdformat::JointKind::Revolute),
        world_name,
        "expected a free switchable 3–32 link revolute robot, optional torque posture and one fixed plane",
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
        "grounded robot requires a 1 ms step",
    )?;
    let ground = construction::ground(&sdf, world, floor, world_name, &sim)?;
    let mut links = vec![];
    for link in &model.links {
        let name = &link.scoped_name;
        let [collider] = link.collisions.as_slice() else {
            return Err(error("expected one physical box per robot link"));
        };
        let Geometry::Box { size } = collider.geometry else {
            return Err(error("grounded robot requires box links"));
        };
        require(
            !link.properties.kinematic
                && identity(sdf.resolve_pose(&collider.pose, &model.scoped_name, name, name)?)
                && size.iter().all(|v| v.is_finite() && *v > 0.),
            name,
            "expected a centered, massive dynamic box",
        )?;
        let id = construction::public_id(&sim, name)?;
        let e = gz_sim_core::entity_by_id(sim.world(), id)
            .ok_or_else(|| error("missing ordinary link"))?;
        let h = sim
            .world()
            .get::<BodyHandle>(e)
            .ok_or_else(|| error("missing native box"))?;
        let state = sim
            .world()
            .resource::<PhysicsBackend>()
            .state(h.id())
            .ok_or_else(|| error("missing initial momentum"))?;
        let pose = sdf.pose_of(name, world_name)?;
        let inertia = sdf.resolve_pose(&link.inertial.pose, &model.scoped_name, name, name)?;
        links.push(GroundedRobotLinkSpec {
            id,
            size,
            mass: link.inertial.mass,
            position: pose.translation,
            orientation: pose.rotation,
            velocity: state.velocity,
            angular_velocity: state.angular_velocity,
            inertia_center: inertia.translation,
            inertia_orientation: inertia.rotation,
            inertia_tensor: link.inertial.inertia,
        });
    }
    links.sort_by_key(|l| l.id);
    let (joints, limits, exact) = floating_tree::joint_recipe(&sdf, model, world_name, &sim)?;
    require(
        !exact,
        world_name,
        "grounded robot currently requires torque-actuated revolutes",
    )?;
    let spec = GroundedRobotSpec {
        model_id: construction::public_id(&sim, &model.scoped_name)?,
        links,
        joints,
        limits,
        ground,
    };
    Ok((spec, sim))
}
