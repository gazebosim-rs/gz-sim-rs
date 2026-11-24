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

/// Mass, center of mass and rotational inertia of the downstream bodies held at
/// their current relative poses. All coordinates are in the parent joint frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LockedSubtreeInertia {
    pub mass_kg: f64,
    pub center_of_mass_m: [f64; 3],
    /// Columns of the symmetric tensor about the joint origin, in kg·m².
    pub inertia_kg_m2: [[f64; 3]; 3],
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

    fn downstream_bodies(&self, id: JointId) -> Result<(Construction, &[BodyId]), PhysicsError> {
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
        Ok((joint.initial, &tree.bodies[range.clone()]))
    }

    /// Composite rigid-body inertia about the current joint pivot. Descendant
    /// joints contribute at their committed configurations. Free motion of those
    /// joints requires an articulated dynamics calculation.
    pub fn locked_subtree_inertia(
        &self,
        id: JointId,
    ) -> Result<LockedSubtreeInertia, PhysicsError> {
        let (joint, bodies) = self.downstream_bodies(id)?;
        let frame = self.parent_joint_pose(joint);
        let inverse = frame.rotation.inverse();
        let mut mass = 0.;
        let mut first_moment = Vector::ZERO;
        let mut inertia = Matrix::ZERO;
        for body in bodies {
            let rb = &self.world.bodies[self.bodies[body].0];
            let props = &rb.mass_properties().local_mprops;
            let m = props.mass();
            let arm = inverse * (rb.center_of_mass() - frame.translation);
            let axes =
                Matrix::from_quat(inverse * *rb.rotation() * props.principal_inertia_local_frame);
            let at_com = axes * Matrix::from_diagonal(props.principal_inertia()) * axes.transpose();
            // [r]×ᵀ [r]× avoids subtracting two nearly equal squared lengths.
            let cross = Matrix::from_cols(
                arm.cross(Vector::X),
                arm.cross(Vector::Y),
                arm.cross(Vector::Z),
            );
            inertia += at_com + m * cross.transpose() * cross;
            mass += m;
            first_moment += m * arm;
        }
        let center = first_moment / mass;
        if !mass.is_finite() || !center.is_finite() || !inertia.is_finite() {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(LockedSubtreeInertia {
            mass_kg: mass,
            center_of_mass_m: center.to_array(),
            inertia_kg_m2: inertia.to_cols_array_2d(),
            parent_frame_rotation: joint.frame1.rotation.to_array(),
        })
    }

    /// Sum the native weights and their moments on the child side of a robot joint.
    /// Admits a tree of robot joints or a tree of ordinary native joints. Mixed
    /// external constraints, cycles and multiple parents have no unique robot load.
    pub fn joint_gravity_load(&self, id: JointId) -> Result<JointGravityLoad, PhysicsError> {
        let (joint, bodies) = self.downstream_bodies(id)?;
        let frame = self.parent_joint_pose(joint);
        let (mut force, mut moment) = (Vector::ZERO, Vector::ZERO);
        for body in bodies {
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
            parent_frame_rotation: joint.frame1.rotation.to_array(),
        })
    }
}

#[cfg(test)]
#[path = "../../tests/private/gravity_load.rs"]
mod tests;
