// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use std::ops::Range;

/// Gravity acting on the joint's downstream rigid bodies, expressed in the
/// parent joint frame. Moment is about that frame's origin. Forces from
/// contacts, actuators, constraints and acceleration are outside this model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointGravityLoad {
    pub force_n: [f64; 3],
    pub torque_nm: [f64; 3],
    /// Orientation (xyzw) of the reporting axes relative to the parent rigid body,
    /// or relative to world for a joint mounted directly to world.
    pub parent_frame_rotation: [f64; 4],
}

/// Each valid subtree occupies a contiguous range in preorder. Topology is
/// built once per joint graph; each query visits only its downstream bodies.
#[derive(Clone, Default)]
pub(super) struct GravityTree {
    bodies: Vec<BodyId>,
    ranges: BTreeMap<JointId, Range<usize>>,
}

#[derive(Default)]
struct Node {
    incoming: Vec<(JointId, Option<BodyId>, bool)>,
    children: Vec<BodyId>,
}

impl GravityTree {
    fn build(records: &BTreeMap<JointId, Joint>) -> Self {
        let mut nodes = BTreeMap::<BodyId, Node>::new();
        for (&id, joint) in records {
            let c = joint.initial;
            nodes
                .entry(c.child)
                .or_default()
                .incoming
                .push((id, c.parent, c.robot));
            if let Some(parent) = c.parent {
                nodes.entry(parent).or_default().children.push(c.child);
            }
        }
        let mut tree = Self::default();
        for (&root, node) in &nodes {
            // A freely floating root has no incoming joint. Separate world
            // mounts never connect independent robots to each other.
            if !matches!(node.incoming.as_slice(), [] | [(_, None, _)]) {
                continue;
            }
            let origin = tree.bodies.len();
            let mut ranges = Vec::new();
            let mut robot_kind = None;
            let mut stack = vec![(root, None)];
            while let Some((body, entering_at)) = stack.pop() {
                let node = &nodes[&body];
                if let Some(start) = entering_at {
                    if let [(id, _, _)] = node.incoming.as_slice() {
                        ranges.push((*id, start..tree.bodies.len()));
                    }
                    continue;
                }
                // A reachable cycle also gives its entry two incoming edges.
                // Pure cycles have no root. Drop the whole affected robot.
                let admitted = match node.incoming.as_slice() {
                    [] => true,
                    [(_, _, robot)] => robot_kind.get_or_insert(*robot) == robot,
                    _ => false,
                };
                if !admitted {
                    ranges.clear();
                    tree.bodies.truncate(origin);
                    break;
                }
                stack.push((body, Some(tree.bodies.len())));
                tree.bodies.push(body);
                stack.extend(node.children.iter().rev().map(|&c| (c, None)));
            }
            tree.ranges.extend(ranges);
        }
        tree
    }
}

impl RapierBackend {
    pub(super) fn parent_joint_pose(&self, c: Construction) -> Pose {
        c.parent.map_or(Pose::IDENTITY, |id| {
            *self.world.bodies[self.bodies[&id].0].position()
        }) * c.frame1
    }

    pub(super) fn gravity_about(
        &self,
        body: RigidBodyHandle,
        pivot: Vector,
    ) -> (Vector, Vector, f64) {
        let rb = &self.world.bodies[body];
        let weight = self.world.gravity * (rb.mass() * rb.gravity_scale());
        let arm = rb.center_of_mass() - pivot;
        (weight, arm.cross(weight), arm.dot(weight))
    }

    /// Sum the native weights and their moments on the child side of a robot joint.
    /// Admits a tree of robot joints or a tree of ordinary native joints. Mixed
    /// external constraints, cycles and multiple parents have no unique robot load.
    pub fn joint_gravity_load(&self, id: JointId) -> Result<JointGravityLoad, PhysicsError> {
        let joint = self
            .joints
            .records
            .get(&id)
            .ok_or(PhysicsError::MissingJoint)?;
        let tree = self
            .joints
            .gravity_tree
            .get_or_init(|| GravityTree::build(&self.joints.records));
        let range = tree.ranges.get(&id).ok_or(PhysicsError::InvalidJoint)?;
        let frame = self.parent_joint_pose(joint.initial);
        let (mut force, mut moment) = (Vector::ZERO, Vector::ZERO);
        for body in &tree.bodies[range.clone()] {
            let (weight, torque, _) = self.gravity_about(self.bodies[body].0, frame.translation);
            force += weight;
            moment += torque;
        }
        let inverse = frame.rotation.inverse();
        let force = inverse * force;
        let moment = inverse * moment;
        if !force.is_finite() || !moment.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(JointGravityLoad {
            force_n: force.to_array(),
            torque_nm: moment.to_array(),
            parent_frame_rotation: joint.initial.frame1.rotation.to_array(),
        })
    }
}

#[cfg(test)]
#[path = "../../tests/private/gravity_load.rs"]
mod tests;
