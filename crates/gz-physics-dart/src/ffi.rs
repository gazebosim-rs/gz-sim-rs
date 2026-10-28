// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The audited native boundary. Every native function is declared fallible, so
//! cxx wraps it in the `rust::behavior::trycatch` from bridge.h. That catches
//! every C++ exception, std or not, and returns it as an error; none unwinds
//! into Rust.
#![allow(unsafe_code)]

#[cxx::bridge(namespace = "gz_physics_dart")]
pub(crate) mod bridge {
    /// One rigid body. Shape codes: 0 sphere [r], 1 box [full x,y,z],
    /// 2 cylinder [r, length], 3 capsule [r, cylindrical length].
    #[derive(Clone)]
    struct BodyConfig {
        shape: u8,
        dimensions: [f64; 3],
        position: [f64; 3],
        orientation: [f64; 4],
        mobile: bool,
        mass: f64,
        center: [f64; 3],
        /// Body-axis moment about the center of mass: xx, yy, zz, xy, xz, yz.
        moment: [f64; 6],
        /// The same moments' principal values and body-frame axes (column-major
        /// columns), validated in Rust; the contact guard uses these rather than
        /// eigen-solving DART's stored tensor.
        principal: [f64; 3],
        axes: [f64; 9],
        velocity: Twist,
    }

    /// World-axis vector pair: velocity, impulse, wrench, or anchor and axis.
    #[derive(Clone, Copy, Debug, PartialEq)]
    struct Twist {
        linear: [f64; 3],
        angular: [f64; 3],
    }

    /// World force at the center of mass and world torque for one step.
    struct Wrench {
        slot: usize,
        force: [f64; 3],
        torque: [f64; 3],
    }

    /// Admission bounds the native step keeps after each step (see body.rs).
    struct Limits {
        /// Largest generalized velocity entry or impulse result.
        magnitude: f64,
        /// Largest world coordinate of any collision bound, in meters.
        position: f64,
        /// Largest bound on the summed inverse effective mass of a contact.
        contact: f64,
    }

    struct Readout {
        position: [f64; 3],
        orientation: [f64; 4],
        /// Center-of-mass linear and world angular velocity.
        velocity: Twist,
        joint_position: f64,
        joint_velocity: f64,
    }

    unsafe extern "C++" {
        include!("gz-physics-dart/include/bridge.h");

        type World;

        fn new_world(gravity: &Twist, limits: &Limits) -> Result<UniquePtr<World>>;
        fn add_body(self: Pin<&mut World>, config: &BodyConfig) -> Result<usize>;
        fn remove_body(self: Pin<&mut World>, slot: usize) -> Result<()>;
        /// False leaves velocities unchanged: an entry would exceed the limit.
        fn set_velocity(self: Pin<&mut World>, slot: usize, velocity: &Twist) -> Result<bool>;
        /// False leaves velocities unchanged because a result exceeds the limit.
        fn apply_impulse(self: Pin<&mut World>, slot: usize, impulse: &Twist) -> Result<bool>;
        /// False changes nothing: the projected joint rate would exceed the limit.
        fn add_revolute(self: Pin<&mut World>, slot: usize, anchor_axis: &Twist) -> Result<bool>;
        /// False keeps the hinge: freed, the body's contacts would exceed the bound.
        fn remove_revolute(self: Pin<&mut World>, slot: usize) -> Result<bool>;
        /// Applies every wrench or none, then steps. Nonzero restored the
        /// pre-step state: 1 nonfinite or too fast, 2 off the collision grid,
        /// 3 a contact beyond the inverse-mass bound.
        fn step(self: Pin<&mut World>, dt: f64, wrenches: &[Wrench]) -> Result<u8>;
        fn read(self: &World, slot: usize) -> Result<Readout>;
        /// Whether every current contact's inverse effective mass is bounded.
        fn contacts_conditioned(self: Pin<&mut World>) -> Result<bool>;
        /// Files defining the loaded libdart, libdart-collision-ode,
        /// libdart-external-odelcpsolver and libode, in that order.
        fn library_paths(self: &World) -> Result<Vec<String>>;
        /// Skeletons in the DART world, for ownership checks.
        #[allow(dead_code)]
        fn skeleton_count(self: &World) -> Result<usize>;
        /// Throws nothing (0), a std::exception (1) or an int (other values), to
        /// verify exception translation through the real bridge.
        #[allow(dead_code)]
        fn exception_probe(kind: u8) -> Result<()>;
    }
}

/// A C++ exception caught by the bridge, reported with the engine's message.
pub(crate) fn native(error: cxx::Exception) -> gz_physics::PhysicsError {
    gz_physics::PhysicsError::Native {
        engine: crate::ENGINE,
        message: error.what().to_owned(),
    }
}

// SAFETY: A `World` is reachable only through the one `UniquePtr` that owns it;
// no DART object pointer, reference or shared_ptr escapes the bridge, and every
// mutation takes `Pin<&mut World>`. A DART world has no thread affinity. ODE's
// global state is the shared part: the bridge holds one process-wide mutex for
// every call that creates, changes, steps or destroys ODE geometry, and allocates
// ODE's per-thread collider data on the calling thread before each step. Moving
// the owner to another thread is therefore sound (tests/threads.rs).
// `World` is deliberately not `Sync`: const DART getters update mutable caches.
unsafe impl Send for bridge::World {}
