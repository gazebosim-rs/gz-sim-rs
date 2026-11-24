// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{MultiFidelityBackend, PhysicsError};
use gz_physics::BodyId;
pub use gz_physics_rapier::{
    CLEAR_GRASP_MEMBER_IMAGE_LIMIT, FLOATING_TREE_IMAGE_LIMIT, GRASP_FLEET_IMAGE_LIMIT,
    GRASP_TREE_IMAGE_LIMIT, GROUNDED_ROBOT_IMAGE_LIMIT, GROUNDED_TREE_IMAGE_LIMIT,
    MAX_FLOATING_TREE_LINKS, MAX_GROUNDED_ROBOT_LINKS, MAX_GROUNDED_TREE_LINKS,
    MAX_PAYLOAD_TREE_LINKS, MOTOR_TREE_IMAGE_LIMIT, PAYLOAD_TREE_IMAGE_LIMIT,
    STOPPED_TREE_IMAGE_LIMIT, WARM_GRASP_MEMBER_IMAGE_LIMIT,
};

impl MultiFidelityBackend {
    /// Capture the native gripper contacts retained by one member.
    pub fn checkpoint_warm_grasp_member(
        &self,
        root: gz_physics::BodyId,
        payload: gz_physics::BodyId,
    ) -> Result<Option<Vec<u8>>, PhysicsError> {
        self.solver.checkpoint_warm_grasp_member(root, payload)
    }
    /// Materialize a shared contact graph from two prior member images.
    pub fn prepare_warm_grasp_fleet(
        &mut self,
        groups: &[(gz_physics::BodyId, gz_physics::BodyId); 2],
        members: [&[u8]; 2],
    ) -> Result<(), PhysicsError> {
        self.solver.prepare_warm_grasp_fleet(groups, members)
    }
    /// Capture an independently solved, fully released tree and workpiece.
    pub fn checkpoint_clear_grasp_member(
        &self,
        root: gz_physics::BodyId,
        payload: gz_physics::BodyId,
    ) -> Result<Option<Vec<u8>>, PhysicsError> {
        self.solver.checkpoint_clear_grasp_member(root, payload)
    }
    /// Import the two cache-drained members into a newly constructed common world.
    pub fn restore_clear_grasp_fleet(
        &mut self,
        groups: &[(gz_physics::BodyId, gz_physics::BodyId); 2],
        members: [&[u8]; 2],
    ) -> Result<(), PhysicsError> {
        self.solver.restore_clear_grasp_fleet(groups, members)
    }
    /// Capture two complete grippers, their separately moving workpieces and all native contacts.
    pub fn checkpoint_grasp_fleet(
        &self,
        groups: &[(gz_physics::BodyId, gz_physics::BodyId); 2],
    ) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_grasp_fleet(groups)
    }
    /// Restore the same-build full graph into two freshly authored assemblies in public model order.
    pub fn restore_grasp_fleet(
        &mut self,
        groups: &[(gz_physics::BodyId, gz_physics::BodyId); 2],
        bytes: &[u8],
    ) -> Result<(), PhysicsError> {
        self.solver.restore_grasp_fleet(groups, bytes)
    }
    /// Observe rigid–rigid shear between this fleet's two independently massed workpieces.
    pub fn configure_grasp_payload_pair(
        &mut self,
        parts: [gz_physics::BodyId; 2],
    ) -> Result<(), PhysicsError> {
        self.solver.configure_grasp_payload_pair(parts)
    }
    /// Last signed native impulse on the first dynamic sphere from the second, in world axes.
    pub fn sphere_pair_reaction(
        &self,
        a: gz_physics::BodyId,
        b: gz_physics::BodyId,
    ) -> Result<gz_physics_rapier::MovingReaction, PhysicsError> {
        self.solver.sphere_pair_reaction(a, b)
    }
    /// Historical raw Rapier writeback used only by the first coupled fleet profile.
    pub fn sphere_pair_reaction_legacy(
        &self,
        a: gz_physics::BodyId,
        b: gz_physics::BodyId,
    ) -> Result<gz_physics_rapier::MovingReaction, PhysicsError> {
        self.solver.sphere_pair_reaction_legacy(a, b)
    }
    /// Native collision and fat-tree envelope for a current 1 ms split proposal.
    pub fn grasp_collision_bounds(
        &self,
        id: gz_physics::BodyId,
        dt: std::time::Duration,
    ) -> Result<([f64; 3], [f64; 3]), PhysicsError> {
        self.solver.grasp_collision_bounds(id, dt)
    }
    /// Release an awake payload and export the contact-free motorized tree's compact image.
    pub fn detached_grasp_tree_image(
        &self,
        payload: gz_physics::BodyId,
    ) -> Result<Vec<u8>, PhysicsError> {
        self.solver.detached_grasp_tree_image(payload)
    }
    /// Capture persistent motors, their linked bodies, the free payload and solved contacts.
    pub fn checkpoint_grasp_revolute_tree(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_grasp_revolute_tree()
    }
    /// Restore a committed native grasp into fresh construction.
    pub fn restore_grasp_revolute_tree(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.solver.restore_grasp_revolute_tree(bytes)
    }
    /// Capture a floating tree interacting with one independently moving sphere.
    pub fn checkpoint_payload_revolute_tree(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_payload_revolute_tree()
    }
    /// Restore the complete same-build contact graph and the separate payload.
    pub fn restore_payload_revolute_tree(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.solver.restore_payload_revolute_tree(bytes)
    }
    /// Contact impulse on the link from the payload, expressed in world axes.
    pub fn payload_tree_reaction(
        &self,
        link: gz_physics::BodyId,
        payload: gz_physics::BodyId,
    ) -> Result<gz_physics_rapier::MovingReaction, PhysicsError> {
        self.solver.payload_tree_reaction(link, payload)
    }

    /// Preserve an externally contacting tree and the complete native contact/island solver.
    pub fn checkpoint_grounded_robot_tree(
        &self,
        root: BodyId,
        ground: BodyId,
    ) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_grounded_robot_tree(root, ground)
    }
    pub fn restore_grounded_robot_tree(
        &mut self,
        root: BodyId,
        ground: BodyId,
        bytes: &[u8],
    ) -> Result<(), PhysicsError> {
        self.solver.restore_grounded_robot_tree(root, ground, bytes)
    }
    pub fn checkpoint_grounded_revolute_tree(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_grounded_revolute_tree()
    }
    /// Restore a committed tree with its fixed floor, cached contacts and held efforts.
    pub fn restore_grounded_revolute_tree(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        self.solver.restore_grounded_revolute_tree(bytes)
    }

    /// Capture a tree whose native solver may have persistent bounded velocity motors.
    pub fn checkpoint_motor_revolute_tree(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_motor_revolute_tree()
    }
    /// Restore the exact native inputs and generalized state into a fresh identical tree.
    pub fn restore_motor_revolute_tree(&mut self, image: &[u8]) -> Result<(), PhysicsError> {
        self.solver.restore_motor_revolute_tree(image)
    }
    /// Read the native velocity-motor policy and its torque cap.
    pub fn motor_revolute_velocity(
        &self,
        id: gz_physics::JointId,
    ) -> Result<Option<gz_physics::JointVelocityMotor>, PhysicsError> {
        self.solver.motor_revolute_velocity(id)
    }

    /// Capture a floating revolute tree and cryptographically bind its actual previous solver interval.
    pub fn checkpoint_timed_revolute_tree(
        &self,
        last: std::time::Duration,
    ) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_timed_revolute_tree(last)
    }
    /// Restore a matching time-bound tree and return the accepted native subinterval.
    pub fn restore_timed_revolute_tree(
        &mut self,
        image: &[u8],
    ) -> Result<std::time::Duration, PhysicsError> {
        self.solver.restore_timed_revolute_tree(image)
    }
    /// Capture one isolated dynamic tree of 3–64 spherical links at the fixed 1 ms interval.
    pub fn checkpoint_floating_revolute_tree(&self) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_floating_revolute_tree()
    }
    /// Restore a committed image into a fresh backend with matching construction.
    pub fn restore_floating_revolute_tree(&mut self, image: &[u8]) -> Result<(), PhysicsError> {
        self.solver.restore_floating_revolute_tree(image)
    }
}

impl MultiFidelityBackend {
    /// Time-bound warm state of one isolated finite spherical receiver on a passive world revolute.
    pub fn checkpoint_timed_world_revolute(
        &self,
        last: std::time::Duration,
    ) -> Result<Vec<u8>, PhysicsError> {
        self.solver.checkpoint_timed_world_revolute(last)
    }
    pub fn restore_timed_world_revolute(
        &mut self,
        image: &[u8],
    ) -> Result<std::time::Duration, PhysicsError> {
        self.solver.restore_timed_world_revolute(image)
    }
}
