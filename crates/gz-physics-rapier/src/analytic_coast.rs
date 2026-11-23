// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Closed-form rigid-center motion and linear-time revolute forward kinematics.
//! This path is entered only by the isolated free-coast admission transaction.
use super::*;

#[derive(Clone, Copy)]
struct Motion {
    pose: Pose,
    origin_velocity: Vector,
    angular: Vector,
}
struct Node {
    handle: RigidBodyHandle,
    parent: Option<usize>,
    fraction: f64,
    local_com: Vector,
    rate: f64,
    fixed_pose: Pose,
    next: Motion,
    next_center_velocity: Vector,
}

pub(super) struct FreeCoast {
    pub root: BodyId,
    tree: MultibodyIndex,
    start_center: Vector,
    velocity: Vector,
    root_rotation: Rotation,
    omega: Vector,
    elapsed: Duration,
    nodes: Vec<Node>,
}
impl FreeCoast {
    pub fn len(&self) -> usize {
        self.nodes.len()
    }
    pub fn new(world: &PhysicsWorld, tree_id: MultibodyIndex, root: BodyId) -> Self {
        let tree = world
            .multibody_joints
            .get_multibody(tree_id)
            .expect("admitted tree");
        let base = &world.bodies[tree.root().rigid_body_handle()];
        let origin = base.center_of_mass();
        let mass: f64 = tree
            .links()
            .map(|link| world.bodies[link.rigid_body_handle()].mass())
            .sum();
        let mut center_offset = Vector::ZERO;
        let mut velocity = Vector::ZERO;
        let mut nodes = Vec::with_capacity(tree.num_links());
        for link in tree.links() {
            let body = &world.bodies[link.rigid_body_handle()];
            let fraction = body.mass() / mass;
            center_offset += fraction * (body.center_of_mass() - origin);
            velocity += fraction * body.linvel();
            nodes.push(Node {
                handle: link.rigid_body_handle(),
                parent: link.parent_id(),
                fraction,
                local_com: body.mass_properties().local_mprops.local_com,
                rate: if link.is_root() || link.joint.ndofs() == 0 {
                    0.
                } else {
                    tree.joint_velocity(link)[0]
                },
                fixed_pose: link.joint.body_to_parent(),
                next_center_velocity: Vector::ZERO,
                next: Motion {
                    pose: Pose::IDENTITY,
                    origin_velocity: Vector::ZERO,
                    angular: Vector::ZERO,
                },
            });
        }
        Self {
            root,
            tree: tree_id,
            start_center: origin + center_offset,
            velocity,
            root_rotation: *base.rotation(),
            omega: base.angvel(),
            elapsed: Duration::ZERO,
            nodes,
        }
    }

    pub fn advance(
        &mut self,
        world: &mut PhysicsWorld,
        step: Duration,
    ) -> Result<(), PhysicsError> {
        let elapsed = self
            .elapsed
            .checked_add(step)
            .ok_or(PhysicsError::InvalidValue)?;
        let time = elapsed.as_secs_f64();
        let dt = step.as_secs_f64();
        let center = self.start_center + self.velocity * time;
        let rotation = Rotation::from_scaled_axis(self.omega * time) * self.root_rotation;
        if !center.is_finite() || !rotation.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        let tree = world
            .multibody_joints
            .get_multibody_mut(self.tree)
            .expect("coast tree remains native");
        // First pass expresses all origins and velocities relative to the base
        // origin, in world axes. This also keeps any admitted roundoff-scale
        // internal rates observable. No dense Jacobian is needed to propagate
        // a twist through a revolute bearing.
        let mut relative_center = Vector::ZERO;
        let mut relative_velocity = Vector::ZERO;
        for id in 0..self.nodes.len() {
            let motion = if let Some(parent) = self.nodes[id].parent {
                let previous = self.nodes[parent].next;
                let node = &self.nodes[id];
                let link = tree.link(id).expect("native link");
                let relative = if node.rate == 0. {
                    node.fixed_pose
                } else {
                    let mut joint = link.joint;
                    joint.integrate(dt, &[node.rate]);
                    joint.body_to_parent()
                };
                let pose = previous.pose * relative;
                let bearing = previous.pose * link.joint.data.local_frame1;
                let angular = previous.angular + bearing.rotation * Vector::X * node.rate;
                let origin_velocity = previous.origin_velocity
                    + previous
                        .angular
                        .cross(bearing.translation - previous.pose.translation)
                    + angular.cross(pose.translation - bearing.translation);
                Motion {
                    pose,
                    origin_velocity,
                    angular,
                }
            } else {
                Motion {
                    pose: Pose::from_rotation(rotation),
                    origin_velocity: Vector::ZERO,
                    angular: self.omega,
                }
            };
            let n = &mut self.nodes[id];
            let local_arm = motion.pose.rotation * n.local_com;
            relative_center += n.fraction * (motion.pose.translation + local_arm);
            n.next_center_velocity = motion.origin_velocity + motion.angular.cross(local_arm);
            relative_velocity += n.fraction * n.next_center_velocity;
            n.next = motion;
        }
        let root_position = center - relative_center;
        let root_velocity = self.velocity - relative_velocity;
        // Complete every numerical check before modifying the tree, bodies,
        // collision geometry, or the clock. Scratch is overwritten on retry.
        for node in &mut self.nodes {
            let n = &mut node.next;
            n.pose.translation += root_position;
            node.next_center_velocity += root_velocity;
            if !n.pose.translation.is_finite()
                || !n.pose.rotation.is_finite()
                || !node.next_center_velocity.is_finite()
                || !n.angular.is_finite()
            {
                return Err(PhysicsError::InvalidValue);
            }
        }
        for (id, node) in self.nodes.iter().enumerate() {
            if node.rate != 0. {
                tree.link_mut(id)
                    .expect("native link")
                    .joint
                    .integrate(dt, &[node.rate]);
            }
            let body = &mut world.bodies[node.handle];
            body.set_position(node.next.pose, false);
            body.set_linvel(node.next_center_velocity, false);
            body.set_angvel(node.next.angular, false);
        }
        // Rapier's free root stores COM linear velocity in its first 3 DoFs.
        tree.generalized_velocity_mut()
            .rows_mut(0, 3)
            .copy_from_slice(&self.nodes[0].next_center_velocity.to_array());
        world
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut world.colliders);
        world.integration_parameters.dt = dt;
        self.elapsed = elapsed;
        Ok(())
    }

    /// Rebuild the native Jacobians and root free-joint coordinates from the
    /// already committed body poses before native constraints may respond.
    pub fn materialize(&self, world: &mut PhysicsWorld) {
        world
            .multibody_joints
            .get_multibody_mut(self.tree)
            .expect("coast tree")
            .forward_kinematics(&world.bodies, true);
    }
}
