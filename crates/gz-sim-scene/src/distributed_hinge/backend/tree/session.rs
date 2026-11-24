// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The Bevy scheduler sees one committed clock and the same public IDs across worker layouts.
use super::*;
use gz_physics::JointVelocityMotor;
use gz_physics_distributed::{
    articulation::ArticulationLinkState, floating_tree::FloatingTreeObservation,
};
type Result<T> = std::result::Result<T, ArticulationError>;

pub trait TreeSceneSession: Send + Sync + 'static {
    fn topology(&self) -> &FloatingTreeSpec;
    fn observation(&self) -> &FloatingTreeObservation;
    fn epoch(&self) -> u64;
    fn ground_id(&self) -> Option<u64>;
    fn payload(&self) -> Option<&ArticulationLinkState>;
    fn motor_supported(&self) -> bool;
    fn motor_locked(&self, joint: u64) -> bool;
    fn supports_payload_force(&self) -> bool;
    fn set_motor(
        &mut self,
        joint: u64,
        motor: Option<JointVelocityMotor>,
    ) -> std::result::Result<(), PhysicsError>;
    fn check_interval(&self, epoch: u64, tick: u64) -> Result<()>;
    fn queue_effort(&mut self, joint: u64, effort: f64) -> Result<()>;
    fn advance_commands(
        &mut self,
        clock: Option<(u64, u64)>,
        efforts: &[(u64, f64)],
        force: [f64; 3],
    ) -> std::result::Result<(), PhysicsError>;
    fn reset(&mut self) -> Result<()>;
    fn transfer(&mut self, destination: u8) -> Result<()>;
    fn replace_worker(&mut self, slot: u8) -> Result<()>;
}

impl<S: TreeConstruction + Send + Sync + 'static> TreeSceneSession for TreePool<S> {
    fn topology(&self) -> &FloatingTreeSpec {
        self.topology()
    }
    fn observation(&self) -> &FloatingTreeObservation {
        self.observation()
    }
    fn epoch(&self) -> u64 {
        self.epoch()
    }
    fn ground_id(&self) -> Option<u64> {
        (self as &dyn Any)
            .downcast_ref::<GroundedTreePool>()
            .map(|floor| floor.spec().ground.link_id)
    }
    fn payload(&self) -> Option<&ArticulationLinkState> {
        self.observed_payload().map(|(p, _)| p)
    }
    fn motor_supported(&self) -> bool {
        self.observed_velocity_motors().is_some()
    }
    fn motor_locked(&self, joint: u64) -> bool {
        (self as &dyn Any)
            .downcast_ref::<GraspTreePool>()
            .is_some_and(|pool| {
                pool.spec().payload.exact_revolute_locks
                    && pool
                        .joint_limits()
                        .iter()
                        .any(|l| l.joint_id == joint && l.lower == 0. && l.upper == 0.)
            })
    }
    fn supports_payload_force(&self) -> bool {
        (self as &dyn Any).is::<GraspTreePool>()
    }
    fn set_motor(
        &mut self,
        joint: u64,
        motor: Option<JointVelocityMotor>,
    ) -> std::result::Result<(), PhysicsError> {
        let pool = self as &mut dyn Any;
        if let Some(grasp) = pool.downcast_mut::<GraspTreePool>() {
            return grasp
                .set_velocity_motor(joint, motor)
                .map_err(physics_error);
        }
        pool.downcast_mut::<MotorTreePool>()
            .ok_or_else(|| unsupported("joint velocity motors"))?
            .set_velocity_motor(joint, motor)
            .map_err(physics_error)
    }
    fn check_interval(&self, epoch: u64, tick: u64) -> Result<()> {
        self.check_interval(epoch, tick)
    }
    fn queue_effort(&mut self, joint: u64, effort: f64) -> Result<()> {
        self.queue_effort(joint, effort)
    }
    fn advance_commands(
        &mut self,
        clock: Option<(u64, u64)>,
        efforts: &[(u64, f64)],
        force: [f64; 3],
    ) -> std::result::Result<(), PhysicsError> {
        if force != [0.; 3] {
            let g = (self as &mut dyn Any)
                .downcast_mut::<GraspTreePool>()
                .ok_or_else(|| unsupported("payload force requires motorized grasp"))?;
            let (e, t) = clock.unwrap_or((g.epoch(), g.observation().tick));
            g.advance_with_payload_force_at(e, t, efforts, force)
                .map_err(physics_error)
        } else if let Some((e, t)) = clock {
            self.advance_with_efforts_at(e, t, efforts)
                .map_err(physics_error)
        } else {
            self.advance().map_err(physics_error)
        }
    }
    fn reset(&mut self) -> Result<()> {
        self.reset()
    }
    fn transfer(&mut self, destination: u8) -> Result<()> {
        self.transfer(destination)
    }
    fn replace_worker(&mut self, slot: u8) -> Result<()> {
        self.replace_worker(slot)
    }
}

impl TreeSceneSession for SplitGraspPool {
    fn topology(&self) -> &FloatingTreeSpec {
        self.topology()
    }
    fn observation(&self) -> &FloatingTreeObservation {
        self.observation()
    }
    fn epoch(&self) -> u64 {
        self.epoch()
    }
    fn ground_id(&self) -> Option<u64> {
        None
    }
    fn payload(&self) -> Option<&ArticulationLinkState> {
        Some(self.payload_state())
    }
    fn motor_supported(&self) -> bool {
        true
    }
    fn motor_locked(&self, joint: u64) -> bool {
        self.spec().payload.exact_revolute_locks
            && self
                .joint_limits()
                .iter()
                .any(|l| l.joint_id == joint && l.lower == 0. && l.upper == 0.)
    }
    fn supports_payload_force(&self) -> bool {
        true
    }
    fn set_motor(
        &mut self,
        joint: u64,
        motor: Option<JointVelocityMotor>,
    ) -> std::result::Result<(), PhysicsError> {
        self.set_velocity_motor(joint, motor).map_err(physics_error)
    }
    fn check_interval(&self, epoch: u64, tick: u64) -> Result<()> {
        self.check_interval(epoch, tick)
    }
    fn queue_effort(&mut self, joint: u64, effort: f64) -> Result<()> {
        self.queue_effort(joint, effort)
    }
    fn advance_commands(
        &mut self,
        clock: Option<(u64, u64)>,
        efforts: &[(u64, f64)],
        force: [f64; 3],
    ) -> std::result::Result<(), PhysicsError> {
        let (e, t) = clock.unwrap_or((self.epoch(), self.tick()));
        self.advance_with_payload_force_at(e, t, efforts, force)
            .map_err(physics_error)
    }
    fn reset(&mut self) -> Result<()> {
        self.reset()
    }
    fn transfer(&mut self, destination: u8) -> Result<()> {
        self.transfer(destination)
    }
    fn replace_worker(&mut self, slot: u8) -> Result<()> {
        self.replace_worker(slot)
    }
}
