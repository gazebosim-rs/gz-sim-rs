// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exclusive owner of one native DART world.

use crate::ffi::bridge::{self, BodyConfig, JointEffort, Readout, Twist, Wrench};
use crate::ffi::native;
use cxx::UniquePtr;
use gz_physics::PhysicsError;
use std::{pin::Pin, sync::Mutex};

/// Every access goes through `&mut self`, so the mutex is never locked: it only
/// makes the `Send` world `Sync`, as `std::sync::Exclusive` would. Shared
/// references never reach native code, whose const getters update caches.
pub(crate) struct NativeWorld(Mutex<UniquePtr<bridge::World>>);

impl NativeWorld {
    pub(crate) fn new(gravity: [f64; 3]) -> Result<Self, PhysicsError> {
        let world = bridge::new_world(
            &Twist {
                linear: gravity,
                angular: [0.0; 3],
            },
            &bridge::Limits {
                magnitude: crate::body::MAGNITUDE_LIMIT,
                position: crate::body::POSITION_LIMIT,
                contact: crate::body::CONTACT_LIMIT,
            },
        )
        .map_err(native)?;
        Ok(Self(Mutex::new(world)))
    }

    fn world(&mut self) -> Pin<&mut bridge::World> {
        // new_world returns std::make_unique's non-null result.
        self.0
            .get_mut()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
            .pin_mut()
    }

    pub(crate) fn add_body(&mut self, config: &BodyConfig) -> Result<usize, PhysicsError> {
        self.world().add_body(config).map_err(native)
    }

    pub(crate) fn remove_body(&mut self, slot: usize) -> Result<(), PhysicsError> {
        self.world().remove_body(slot).map_err(native)
    }

    pub(crate) fn set_velocity(
        &mut self,
        slot: usize,
        velocity: &Twist,
    ) -> Result<bool, PhysicsError> {
        self.world().set_velocity(slot, velocity).map_err(native)
    }

    pub(crate) fn apply_impulse(
        &mut self,
        slot: usize,
        impulse: &Twist,
    ) -> Result<bool, PhysicsError> {
        self.world().apply_impulse(slot, impulse).map_err(native)
    }

    pub(crate) fn add_revolute(
        &mut self,
        slot: usize,
        anchor_axis: &Twist,
    ) -> Result<bool, PhysicsError> {
        self.world().add_revolute(slot, anchor_axis).map_err(native)
    }

    pub(crate) fn remove_revolute(&mut self, slot: usize) -> Result<bool, PhysicsError> {
        self.world().remove_revolute(slot).map_err(native)
    }

    pub(crate) fn add_body_revolute(
        &mut self,
        parent: usize,
        child: usize,
        anchor_axis: &Twist,
    ) -> Result<bool, PhysicsError> {
        self.world()
            .add_body_revolute(parent, child, anchor_axis)
            .map_err(native)
    }

    pub(crate) fn step_with_efforts(
        &mut self,
        dt: f64,
        wrenches: &[Wrench],
        efforts: &[JointEffort],
    ) -> Result<u8, PhysicsError> {
        self.world()
            .step_with_efforts(dt, wrenches, efforts)
            .map_err(native)
    }

    pub(crate) fn step(&mut self, dt: f64, wrenches: &[Wrench]) -> Result<u8, PhysicsError> {
        self.world().step(dt, wrenches).map_err(native)
    }

    pub(crate) fn contacts_conditioned(&mut self) -> Result<bool, PhysicsError> {
        self.world().contacts_conditioned().map_err(native)
    }

    pub(crate) fn library_paths(&mut self) -> Result<Vec<String>, PhysicsError> {
        self.world().library_paths().map_err(native)
    }

    pub(crate) fn read(&mut self, slot: usize) -> Result<Readout, PhysicsError> {
        self.world().read(slot).map_err(native)
    }

    #[cfg(test)]
    pub(crate) fn skeleton_count(&mut self) -> Result<usize, PhysicsError> {
        self.world().skeleton_count().map_err(native)
    }
}

#[cfg(test)]
#[path = "../tests/private/native.rs"]
mod tests;
