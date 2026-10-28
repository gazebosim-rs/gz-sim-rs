// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! The only unsafe boundary: MuJoCo loading, ownership and array access.
//!
//! Everything here is exposed through safe functions whose preconditions are
//! established locally. Callers in the rest of the crate never see raw pointers.

#[allow(
    non_camel_case_types,
    non_snake_case,
    non_upper_case_globals,
    dead_code,
    clippy::all,
    clippy::undocumented_unsafe_blocks
)]
// Generated declarations with no executable code, kept outside src/ because the
// coverage gate requires every src file to contain instrumented regions.
#[path = "../bindings/sys.rs"]
pub(crate) mod sys;

use std::{
    ffi::{CStr, OsString, c_int},
    io::Write,
    path::PathBuf,
    ptr::NonNull,
    sync::OnceLock,
};

/// Library version this crate's bindings were generated from.
pub(crate) const HEADER_VERSION: c_int = sys::mjVERSION_HEADER as c_int;
const LIBRARY_NAME: &str = "libmujoco.so.3.11.0";

macro_rules! api {
    ($($name:ident),* $(,)?) => {
        const SYMBOLS: &[&str] = &[$(concat!(stringify!($name), "\0")),*];

        /// Binds every symbol after `check_symbols` proved they all exist.
        fn bind(library: libloading::Library) -> sys::MujocoApi {
            sys::MujocoApi {
                $($name: {
                    let name = concat!(stringify!($name), "\0").as_bytes();
                    // SAFETY: the field type is the bindgen-generated signature of this
                    // exact symbol from the 3.11.0 headers; the version is checked
                    // before any call. The pointer stays valid because the library is
                    // stored alongside it and never unloaded.
                    *unsafe { library.get(name) }.expect("symbol presence was checked")
                },)*
                library,
            }
        }
    };
}
api!(
    mj_compile,
    mj_step,
    mj_forward,
    mj_deleteModel,
    mj_makeData,
    mj_deleteData,
    mj_makeSpec,
    mj_deleteSpec,
    mj_kinematics,
    mj_comPos,
    mj_comVel,
    mj_objectVelocity,
    mj_version,
    mju_setLogHandler,
    mjs_getError,
    mjs_addBody,
    mjs_addJoint,
    mjs_addFreeJoint,
    mjs_addGeom,
    mjs_findBody,
);

/// The loaded MuJoCo C API. Lives for the rest of the process once loaded.
pub(crate) struct Api(sys::MujocoApi);

/// Library search order: `MUJOCO_LIBRARY`, `MUJOCO_PREFIX/lib`, then the dynamic
/// loader's normal search path for the exact 3.11.0 soname.
pub(crate) fn library_path(library: Option<OsString>, prefix: Option<OsString>) -> OsString {
    library
        .or_else(|| {
            prefix.map(|prefix| PathBuf::from(prefix).join("lib").join(LIBRARY_NAME).into())
        })
        .unwrap_or_else(|| LIBRARY_NAME.into())
}

pub(crate) fn check_version(found: c_int) -> Result<(), String> {
    if found == HEADER_VERSION {
        Ok(())
    } else {
        Err(format!(
            "loaded MuJoCo version {found} does not match bindings version {HEADER_VERSION}"
        ))
    }
}

/// Loads and version-checks one library, then installs the process log handler.
pub(crate) fn load(path: &OsString) -> Result<Api, String> {
    // SAFETY: loading runs the library's initializers. MuJoCo's initializers only set
    // up its own global defaults; the path is chosen by the process environment.
    let library = unsafe { libloading::Library::new(path) }
        .map_err(|error| format!("cannot load {}: {error}", path.to_string_lossy()))?;
    for symbol in SYMBOLS {
        // SAFETY: only the address is inspected here; nothing is called.
        unsafe { library.get::<*const ()>(symbol.as_bytes()) }
            .map_err(|error| format!("{}: {error}", path.to_string_lossy()))?;
    }
    let api = bind(library);
    // SAFETY: mj_version takes no arguments and only returns a constant.
    check_version(unsafe { (api.mj_version)() })?;
    // SAFETY: the handler is a non-unwinding extern "C" function with the expected
    // signature that stays valid for the life of the process.
    unsafe { (api.mju_setLogHandler)(Some(log_handler)) };
    Ok(Api(api))
}

/// The process-wide MuJoCo library, loaded once.
pub(crate) fn api() -> Result<&'static Api, String> {
    static API: OnceLock<Result<Api, String>> = OnceLock::new();
    API.get_or_init(|| {
        load(&library_path(
            std::env::var_os("MUJOCO_LIBRARY"),
            std::env::var_os("MUJOCO_PREFIX"),
        ))
    })
    .as_ref()
    .map_err(Clone::clone)
}

/// What the log handler does with one message.
#[derive(Debug, PartialEq, Eq)]
pub(crate) enum LogAction {
    Ignore,
    Print(String),
    Abort(String),
}

pub(crate) fn log_action(level: u32, subject: &str) -> LogAction {
    match level {
        sys::mjLOG_ERROR => LogAction::Abort(format!("MuJoCo fatal error: {subject}\n")),
        sys::mjLOG_WARNING => LogAction::Print(format!("MuJoCo warning: {subject}\n")),
        _ => LogAction::Ignore,
    }
}

/// MuJoCo requires error handlers not to return, and Rust cannot longjmp. Fatal
/// engine errors therefore abort like MuJoCo's default handler, but never write a
/// MUJOCO_LOG.TXT file into the working directory. Nothing here can unwind.
unsafe extern "C" fn log_handler(message: *const sys::mjLogMessage) {
    // SAFETY: MuJoCo passes a valid message whose subject is NUL-terminated.
    let message = unsafe { &*message };
    // SAFETY: the subject array is NUL-terminated by vsnprintf.
    let subject = unsafe { CStr::from_ptr(message.subject.as_ptr()) }.to_string_lossy();
    let level = u32::try_from(message.level).unwrap_or(u32::MAX);
    match log_action(level, &subject) {
        LogAction::Ignore => {}
        LogAction::Print(text) => {
            let _ = std::io::stderr().write_all(text.as_bytes());
        }
        LogAction::Abort(text) => {
            let _ = std::io::stderr().write_all(text.as_bytes());
            std::process::abort();
        }
    }
}

/// A native body joint in the body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum BodyJoint {
    Fixed,
    Free,
    Hinge { anchor: [f64; 3], axis: [f64; 3] },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum Geometry {
    Sphere(f64),
    Box([f64; 3]),
    /// Radius and half-length along body Z.
    Cylinder(f64, f64),
    Capsule(f64, f64),
}

/// Explicit inertial: mass, COM in the body frame, principal moments and the
/// principal frame's [w,x,y,z] orientation in the body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct Inertial {
    pub mass: f64,
    pub center: [f64; 3],
    pub principal: [f64; 3],
    pub quaternion: [f64; 4],
}

/// One world child body. Quaternions here are MuJoCo's [w,x,y,z].
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct BodyModel {
    pub position: [f64; 3],
    pub quaternion: [f64; 4],
    pub joint: BodyJoint,
    pub geometry: Geometry,
    pub inertial: Inertial,
}

struct Spec<'a> {
    api: &'a Api,
    raw: NonNull<sys::mjSpec>,
}

impl Drop for Spec<'_> {
    fn drop(&mut self) {
        // SAFETY: the spec was created by mj_makeSpec and is deleted exactly once.
        unsafe { (self.api.0.mj_deleteSpec)(self.raw.as_ptr()) };
    }
}

impl Spec<'_> {
    fn error(&self) -> String {
        // SAFETY: the spec is live; MuJoCo returns a NUL-terminated buffer it owns.
        let text = unsafe { (self.api.0.mjs_getError)(self.raw.as_ptr()) };
        if text.is_null() {
            return "unknown compile error".into();
        }
        // SAFETY: non-null, NUL-terminated and valid until the next spec call.
        unsafe { CStr::from_ptr(text) }
            .to_string_lossy()
            .into_owned()
    }
}

fn non_null<T>(pointer: *mut T, what: &str) -> Result<NonNull<T>, String> {
    NonNull::new(pointer).ok_or_else(|| format!("MuJoCo could not allocate {what}"))
}

/// A compiled model and its data, exclusively owned and freed exactly once.
pub(crate) struct World {
    api: &'static Api,
    data: NonNull<sys::mjData>,
    model: NonNull<sys::mjModel>,
}

// SAFETY: World exclusively owns its heap-allocated mjModel and mjData. MuJoCo
// objects have no thread affinity or thread-local registration, so transferring
// ownership to another thread is sound.
unsafe impl Send for World {}
// SAFETY: every `&self` method only reads model/data memory or calls MuJoCo
// functions that take const model/data pointers and write only caller-owned
// output (mj_objectVelocity). Mutation requires `&mut self`.
unsafe impl Sync for World {}

impl Drop for World {
    fn drop(&mut self) {
        // SAFETY: both were allocated by MuJoCo for this World and are freed once.
        unsafe {
            (self.api.0.mj_deleteData)(self.data.as_ptr());
            (self.api.0.mj_deleteModel)(self.model.as_ptr());
        }
    }
}

fn add_body(api: &Api, world: *mut sys::mjsBody, body: &BodyModel) -> Result<(), String> {
    // SAFETY: `world` is the live worldbody of a live spec; returned elements are
    // owned by that spec and written before it is compiled or dropped.
    unsafe {
        let child = non_null((api.0.mjs_addBody)(world, std::ptr::null()), "a body")?.as_mut();
        child.pos = body.position;
        child.quat = body.quaternion;
        // Diagonal inertia with its frame: MuJoCo then performs no eigen
        // decomposition, and checks exactly these values.
        child.explicitinertial = true;
        child.mass = body.inertial.mass;
        child.ipos = body.inertial.center;
        child.inertia = body.inertial.principal;
        child.iquat = body.inertial.quaternion;
        match body.joint {
            BodyJoint::Fixed => {}
            BodyJoint::Free => {
                non_null((api.0.mjs_addFreeJoint)(child), "a free joint")?;
            }
            BodyJoint::Hinge { anchor, axis } => {
                let joint =
                    non_null((api.0.mjs_addJoint)(child, std::ptr::null()), "a joint")?.as_mut();
                joint.type_ = sys::mjJNT_HINGE;
                joint.pos = anchor;
                joint.axis = axis;
            }
        }
        let geom = non_null((api.0.mjs_addGeom)(child, std::ptr::null()), "a geom")?.as_mut();
        (geom.type_, geom.size) = match body.geometry {
            Geometry::Sphere(radius) => (sys::mjGEOM_SPHERE, [radius, 0.0, 0.0]),
            Geometry::Box(half) => (sys::mjGEOM_BOX, half),
            Geometry::Cylinder(radius, half) => (sys::mjGEOM_CYLINDER, [radius, half, 0.0]),
            Geometry::Capsule(radius, half) => (sys::mjGEOM_CAPSULE, [radius, half, 0.0]),
        };
    }
    Ok(())
}

impl World {
    /// Compiles a fresh model with upstream gz-physics options: implicitfast,
    /// radians, a 1 ms default step and MuJoCo defaults otherwise. Bodies are
    /// world children with compiled body IDs 1..=n in slice order.
    pub(crate) fn compile(gravity: [f64; 3], bodies: &[BodyModel]) -> Result<Self, String> {
        Self::compile_with(gravity, bodies, None)
    }

    /// Compiles with an explicit arena size, which disables the upstream growth.
    #[cfg(test)]
    pub(crate) fn compile_with_arena(
        gravity: [f64; 3],
        bodies: &[BodyModel],
        bytes: i64,
    ) -> Result<Self, String> {
        Self::compile_with(gravity, bodies, Some(bytes))
    }

    fn compile_with(
        gravity: [f64; 3],
        bodies: &[BodyModel],
        arena: Option<i64>,
    ) -> Result<Self, String> {
        let api = api()?;
        // SAFETY: mj_makeSpec has no preconditions; ownership moves into Spec.
        let raw = unsafe { (api.0.mj_makeSpec)() };
        let spec = Spec {
            api,
            raw: non_null(raw, "a spec")?,
        };
        // SAFETY: the spec is live and exclusively owned here.
        let world = unsafe {
            let raw = &mut *spec.raw.as_ptr();
            raw.option.timestep = 0.001;
            raw.option.integrator = sys::mjINT_IMPLICITFAST as c_int;
            raw.option.gravity = gravity;
            raw.compiler.degree = false;
            if let Some(bytes) = arena {
                raw.memory = bytes;
            }
            (api.0.mjs_findBody)(spec.raw.as_ptr(), c"world".as_ptr())
        };
        for body in bodies {
            add_body(api, world, body)?;
        }
        let mut model = Self::compile_spec(&spec)?;
        // gz-physics: broadphase buffers grow with nbody^2 beyond the default arena.
        // SAFETY: the model is live and owned by `model`.
        let (nbody, narena) = unsafe { (model.as_ref().nbody, model.as_ref().narena) };
        let required = nbody.saturating_mul(nbody).saturating_mul(8);
        if arena.is_none() && required > narena {
            // SAFETY: live spec and model; the first model is replaced and freed once.
            unsafe {
                (*spec.raw.as_ptr()).memory = required;
                let larger = Self::compile_spec(&spec);
                (api.0.mj_deleteModel)(model.as_ptr());
                model = larger?;
            }
        }
        // SAFETY: the model is live; mj_makeData returns owned data or null.
        let data = match NonNull::new(unsafe { (api.0.mj_makeData)(model.as_ptr()) }) {
            Some(data) => data,
            None => {
                // SAFETY: allocated above and not yet owned by a World.
                unsafe { (api.0.mj_deleteModel)(model.as_ptr()) };
                return Err("MuJoCo could not allocate data".into());
            }
        };
        let mut world = Self { api, data, model };
        world.forward();
        Ok(world)
    }

    fn compile_spec(spec: &Spec) -> Result<NonNull<sys::mjModel>, String> {
        // SAFETY: live spec; a null VFS is allowed; a returned model is owned.
        NonNull::new(unsafe { (spec.api.0.mj_compile)(spec.raw.as_ptr(), std::ptr::null()) })
            .ok_or_else(|| spec.error())
    }

    fn model(&self) -> &sys::mjModel {
        // SAFETY: the model is live while self exists and only mutated through &mut self.
        unsafe { self.model.as_ref() }
    }

    fn data(&self) -> &sys::mjData {
        // SAFETY: the data is live while self exists and only mutated through &mut self.
        unsafe { self.data.as_ref() }
    }

    fn pointers(&mut self) -> (*const sys::mjModel, *mut sys::mjData) {
        (self.model.as_ptr(), self.data.as_ptr())
    }

    pub(crate) fn nv(&self) -> usize {
        size(self.model().nv)
    }

    fn nbody(&self) -> usize {
        size(self.model().nbody)
    }

    /// A model array with `width` entries per body or degree of freedom.
    fn array<T>(&self, pointer: *const T, count: usize, width: usize) -> &[T] {
        let len = count * width;
        if len == 0 {
            return &[];
        }
        // SAFETY: MuJoCo allocates this array with `count * width` elements in the
        // live model/data buffer; it is borrowed for no longer than `self`.
        unsafe { std::slice::from_raw_parts(pointer, len) }
    }

    fn array_mut<T>(&mut self, pointer: *mut T, count: usize, width: usize) -> &mut [T] {
        let len = count * width;
        if len == 0 {
            return &mut [];
        }
        // SAFETY: as in `array`, with exclusive access through `&mut self`.
        unsafe { std::slice::from_raw_parts_mut(pointer, len) }
    }

    fn row<const N: usize>(values: &[f64], index: usize) -> [f64; N] {
        let mut row = [0.0; N];
        row.copy_from_slice(&values[index * N..(index + 1) * N]);
        row
    }

    pub(crate) fn qpos(&self) -> &[f64] {
        let nq = size(self.model().nq);
        self.array(self.data().qpos, nq, 1)
    }

    pub(crate) fn qpos_mut(&mut self) -> &mut [f64] {
        let nq = size(self.model().nq);
        let pointer = self.data().qpos;
        self.array_mut(pointer, nq, 1)
    }

    pub(crate) fn qvel(&self) -> &[f64] {
        self.array(self.data().qvel, self.nv(), 1)
    }

    pub(crate) fn qvel_mut(&mut self) -> &mut [f64] {
        let (pointer, nv) = (self.data().qvel, self.nv());
        self.array_mut(pointer, nv, 1)
    }

    pub(crate) fn xfrc_applied_mut(&mut self) -> &mut [f64] {
        let (pointer, nbody) = (self.data().xfrc_applied, self.nbody());
        self.array_mut(pointer, nbody, 6)
    }

    /// Joint address of `body`, as (qpos address, dof address), when it has one.
    pub(crate) fn joint_address(&self, body: usize) -> Option<(usize, usize)> {
        let jntadr = self.array(self.model().body_jntadr, self.nbody(), 1)[body];
        let joint = usize::try_from(jntadr).ok()?;
        let njnt = size(self.model().njnt);
        let qpos = self.array(self.model().jnt_qposadr, njnt, 1)[joint];
        let dof = self.array(self.model().jnt_dofadr, njnt, 1)[joint];
        Some((size(qpos), size(dof)))
    }

    /// First degree of freedom and count for `body`; (0, 0) without any, where
    /// MuJoCo stores the address as -1.
    pub(crate) fn dofs(&self, body: usize) -> (usize, usize) {
        let count = size(self.array(self.model().body_dofnum, self.nbody(), 1)[body]);
        if count == 0 {
            return (0, 0);
        }
        (
            size(self.array(self.model().body_dofadr, self.nbody(), 1)[body]),
            count,
        )
    }

    pub(crate) fn body_position(&self, body: usize) -> [f64; 3] {
        Self::row(self.array(self.data().xpos, self.nbody(), 3), body)
    }

    /// World orientation as MuJoCo [w,x,y,z].
    pub(crate) fn body_quaternion(&self, body: usize) -> [f64; 4] {
        Self::row(self.array(self.data().xquat, self.nbody(), 4), body)
    }

    #[cfg(test)]
    pub(crate) fn gravity(&self) -> [f64; 3] {
        self.model().opt.gravity
    }

    #[cfg(test)]
    pub(crate) fn timestep(&self) -> f64 {
        self.model().opt.timestep
    }

    pub(crate) fn time(&self) -> f64 {
        self.data().time
    }

    pub(crate) fn set_time(&mut self, time: f64) {
        // SAFETY: exclusive access to live data.
        unsafe { self.data.as_mut().time = time };
    }

    /// Cumulative engine warning counts for this data, indexed by mjtWarning.
    /// MuJoCo logs a warning only while its count is zero, so counts are kept.
    pub(crate) fn warnings(&self) -> [i32; WARNING_NAMES.len()] {
        self.data().warning.map(|warning| warning.number)
    }

    #[cfg(test)]
    pub(crate) fn inject_warning(&mut self, index: usize) {
        // SAFETY: exclusive access to live data.
        unsafe { self.data.as_mut().warning[index].number += 1 };
    }

    /// Converts a body index already checked against `nbody` for a C call.
    fn body_id(&self, body: usize) -> c_int {
        assert!(
            body < self.nbody(),
            "body {body} outside the compiled model"
        );
        c_int::try_from(body).expect("MuJoCo body counts fit c_int")
    }

    pub(crate) fn set_timestep(&mut self, dt: f64) {
        // SAFETY: exclusive access to the live model's options.
        unsafe { self.model.as_mut().opt.timestep = dt };
    }

    /// World-axis [angular, linear] velocity of the body's center of mass.
    pub(crate) fn com_velocity(&self, body: usize) -> [f64; 6] {
        let mut velocity = [0.0; 6];
        let body = self.body_id(body);
        // SAFETY: live model/data, valid body ID and a six-element output buffer.
        unsafe {
            (self.api.0.mj_objectVelocity)(
                self.model.as_ptr(),
                self.data.as_ptr(),
                sys::mjOBJ_BODY as c_int,
                body,
                velocity.as_mut_ptr(),
                0,
            );
        }
        velocity
    }

    fn forward(&mut self) {
        let (model, data) = self.pointers();
        // SAFETY: live, matching model and data.
        unsafe { (self.api.0.mj_forward)(model, data) };
    }

    /// Recomputes poses, COM quantities and velocities read by queries.
    pub(crate) fn refresh(&mut self) {
        let (model, data) = self.pointers();
        // SAFETY: live, matching model and data.
        unsafe {
            (self.api.0.mj_kinematics)(model, data);
            (self.api.0.mj_comPos)(model, data);
            (self.api.0.mj_comVel)(model, data);
        }
    }

    /// One mj_step of `dt` seconds followed by the gz-physics kinematic refresh.
    pub(crate) fn step(&mut self, dt: f64) {
        self.set_timestep(dt);
        let (model, data) = self.pointers();
        // SAFETY: live, matching model and data.
        unsafe { (self.api.0.mj_step)(model, data) };
        self.refresh();
    }
}

/// MuJoCo sizes and addresses are nonnegative for a compiled model.
fn size<T: TryInto<usize>>(value: T) -> usize {
    value
        .try_into()
        .ok()
        .expect("MuJoCo sizes and addresses are nonnegative")
}

/// Warnings raised since `previous` counts, split into those after which MuJoCo
/// resets its data (bad qpos, qvel or qacc, as upstream treats them) and
/// recoverable ones. MuJoCo's reset zeroes every count before counting its cause,
/// so after a `reset` any nonzero fatal count is new.
pub(crate) fn classify_warnings(
    counts: [i32; WARNING_NAMES.len()],
    previous: [i32; WARNING_NAMES.len()],
    reset: bool,
) -> (Vec<&'static str>, Vec<&'static str>) {
    let fatal_kinds = [
        sys::mjWARN_BADQPOS,
        sys::mjWARN_BADQVEL,
        sys::mjWARN_BADQACC,
    ];
    let mut fatal = Vec::new();
    let mut recoverable = Vec::new();
    for (index, name) in WARNING_NAMES.iter().enumerate() {
        let is_fatal = fatal_kinds.contains(&(index as u32));
        let new = counts[index] > previous[index] || (reset && is_fatal && counts[index] > 0);
        match (new, is_fatal) {
            (true, true) => fatal.push(*name),
            (true, false) => recoverable.push(*name),
            _ => {}
        }
    }
    (fatal, recoverable)
}

/// Names of mjtWarning values, in index order.
pub(crate) const WARNING_NAMES: [&str; sys::mjNWARNING as usize] = [
    "inertia",
    "contact buffer full",
    "constraint buffer full",
    "bad qpos",
    "bad qvel",
    "bad qacc",
    "bad ctrl",
];

#[cfg(test)]
#[path = "../tests/private/native.rs"]
mod tests;
