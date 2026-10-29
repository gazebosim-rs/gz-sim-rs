<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# MuJoCo backend (PHY-04, first slice)

`gz-physics-mujoco` implements `gz_physics::DynamicBackend` on MuJoCo 3.11.0,
the version upstream gz-physics vendors (GIT_TAG b85fdca5). Its scope is what
the Rapier backend supports: free and static bodies with sphere, box, cylinder
and capsule shapes, mass with optional full inertia, gravity, contacts, world
velocity, COM wrench and impulse inputs, world-revolute joints with state
readback, removal, retained reset and initial-state transactions. The coordinator
owns scene and CLI selection (`PhysicsEngine::Mujoco`); this crate is not wired
into the simulator yet.

## Native boundary

MuJoCo is loaded at run time with `libloading`, not linked, so the workspace
builds without MuJoCo and a missing library is a typed `PhysicsError::Native`
from `MujocoBackend::new`. Search order: `MUJOCO_LIBRARY` (a file), then
`MUJOCO_PREFIX/lib/libmujoco.so.3.11.0`, then the loader's normal search for that
exact soname. Every symbol must resolve and `mj_version()` must equal the
header's `mjVERSION_HEADER` (3011000) before any other call.

`bindings/sys.rs` is bindgen 0.72.1 output for only the types and 24 functions
used, checked in and reproducible with `generate-bindings.sh` (header path from
`MUJOCO_INCLUDE`). It holds declarations and compile-time layout assertions
only, so it lives outside `src/`, where the coverage gate requires every file to
contain instrumented code.
`src/native.rs` is the only module allowed unsafe code (the crate denies it;
the workspace's `forbid` cannot be relaxed per crate, so the crate copies the
workspace lints with `deny`). It owns `mjModel`/`mjData` in one `World` freed
exactly once, and `mjSpec` only for the duration of a compile. `World` is `Send`
because MuJoCo objects have no thread affinity, and `Sync` because every `&self`
path only reads memory or calls const-pointer functions (`mj_objectVelocity`,
`mj_jacBodyCom`) that write caller-owned output. Tests share one backend across
reader threads and create, step and drop backends on other threads.

A process-wide `mju_setLogHandler` handler prints warnings to stderr and, for
errors, prints and aborts. MuJoCo requires error handlers not to return and Rust
cannot `longjmp`, so this matches MuJoCo's default (exit) without writing
`MUJOCO_LOG.TXT`. Compiler errors are not routed there; they return through
`mjs_getError` as `PhysicsError::Native`.

## Upstream-matched options

From `EntityManagementFeatures::ConstructEmptyWorld`, `SDFFeatures` and
`SimulationFeatures::WorldForwardStep`: `implicitfast` integrator, radians, a
1 ms default timestep replaced by each step's `dt`, other `mjOption` defaults,
default geom contact parameters, free joints for dynamic links, jointless world
children for static links, explicit inertials, hinge joints at the anchor, `mj_step` followed by `mj_kinematics`, `mj_comPos` and
`mj_comVel`, applied forces cleared after each step, and upstream's arena growth
to `nbody^2 * 8` bytes when that exceeds MuJoCo's default.

## Topology edits

The compiled model is immutable. As in upstream, where edits mark the spec
dirty and `RecompileSpec` runs at the next `WorldForwardStep`, spawn, body
removal and joint construction here update records and mark the model dirty.
One compile happens at the next advancing `step` or velocity/impulse input, or
at a query that cannot be answered without one (below). Building a scene of N
bodies and joints, querying each joint after adding it as the scene does, costs
one compile; a private test counts compiles for 200 bodies and 50 joints.
`reset` is lazy too. `prepare_initial` compiles eagerly so that `apply_initial`
cannot fail. Hinge removal first validates the current velocity in free-joint
coordinates, refusing the edit if MuJoCo would reset it above `mjMAXVAL`. It
settles the current model before removing the hinge, so even an unstepped body
keeps its projected velocity instead of reverting to its original free velocity.
The resulting free model is compiled when next needed. A failed removal keeps
the joint and physical state unchanged.

Queries avoid compiles. A body not stepped and given no input since spawn or
reset reports its construction state from the records: the given pose and its
construction velocity, projected onto its hinge if it has one. That answer does
not change when the model compiles; a private test shows MuJoCo's own readback
agrees within 1e-14. A compiled body whose joint is unchanged answers from the
committed model, because pending edits of other bodies never change it. An
uncompiled joint reports angle zero and the rate its compile writes into
`qvel`, which is exactly what MuJoCo then reports. Only a body whose joint
changed after it was given a velocity input needs a compile to answer.

The compile builds a fresh `mjSpec` from the records with `mj_compile` and
transfers state, replacing the committed world only on success. Upstream calls
`mj_recompile` in place, which deletes the caller's model and data when
compilation fails. State transfer: unchanged joints copy `qpos`/`qvel` exactly;
a body gaining a hinge keeps its construction pose as angle zero (construction
is only open while bodies are at construction poses) and the hinge rate checked
when the joint was added; a body losing its hinge keeps its world pose and
velocity. Time and the last step size are kept. Like `mj_recompile`, constraint
warm-start is not carried over, so contact solutions after an edit can differ
from an unedited run at solver tolerance. Pending wrenches live outside MuJoCo
and survive edits.

## Admission

Errors stay at the edit, before anything changes, and admitted input always
compiles. Every body is lowered with an explicit diagonal inertial (`inertia`
plus `iquat`), so MuJoCo never diagonalizes a tensor or derives inertia from
geoms and checks exactly the doubles this crate computes. Moving bodies use the
user's tensor, rotated into the body frame and diagonalized in Rust, or the
solid shape's moments; fixed bodies get a unit placeholder that no check can
refuse. When a moment exceeds the sum of the other two only within the shared
1e-12 relative slack (or by shape rounding), it is set to that sum so MuJoCo's
strict check passes: a [1, 1, 2] tensor is admitted identically at identity, 45
and 90 degrees of inertial rotation, as on Rapier.

`spawn_link` rejects with `InvalidValue` everything the Rapier backend rejects
plus MuJoCo's rules on the lowered values: the strict triangle inequality as in
`user_objects.cc`, mass and every principal moment at least `mjMINVAL` (1e-15)
as in `CheckBodyMassInertia`, and, for a shape-derived mass, a volume above
`mjEPS` (1e-14 m^3), where MuJoCo stops deriving geom inertia. Edits also
convert velocities in closed form and reject results MuJoCo would not keep:
`spawn_link` converts the construction velocity to free-joint coordinates
(`v - w x (R c)`, `R^T w`) and `add_world_revolute` projects both the
construction and current velocity onto the hinge. Admission limits that keep
MuJoCo from resetting and every later conversion finite:

- free-joint coordinates and hinge rates at most `mjMAXVAL` (1e10), for
  construction velocities and for `set_world_velocity`/`apply_impulse` results;
- a moving body's position components at most 1e10 m (its free-joint `qpos`);
- inertial-center and hinge-anchor body-frame components at most 1e100 m.

A compile failure after admitted edits therefore means an engine fault. It makes
`step` and inputs return `Native` without changing anything; queries that need
the compile return `None` (a private test injects an unadmitted record).

## Load time

Each compile is O(N) in Rust plus MuJoCo's compile and data allocation, which
grow about quadratically with body count once upstream's `nbody^2 * 8` arena
rule applies. Release build, best of five, N spheres one meter apart,
construction plus one 1 ms step, measured on the same machine on 2026-10-27
(`cargo run --release -p gz-physics-mujoco --example load_time -- 100 200 400`
and `mujoco_reference <plugin> --load 100 200 400`, which constructs the
equivalent SDF world through the installed plugin, excluding SDF parsing). The
hinged case adds a world revolute per sphere; the adapter also queries each
joint's state right after adding it, as the scene does.

| Bodies | Free: adapter | Free: plugin | Hinged: adapter | Hinged: plugin |
| ---: | ---: | ---: | ---: | ---: |
| 100 | 5.1 ms | 7.8 ms | 3.5 ms | 8.9 ms |
| 200 | 18.8 ms | 26.3 ms | 12.8 ms | 29.4 ms |
| 400 | 73.3 ms | 96.9 ms | 49.3 ms | 105.5 ms |

The plugin also creates one actuator per joint, which the adapter does not.

## Semantics and conventions

Public quaternions are [x,y,z,w]; MuJoCo stores [w,x,y,z]. `state` reports the
body origin, the COM linear velocity and the world angular velocity
(`mj_objectVelocity` on the inertial frame). Free-body velocity and impulse
inputs are converted analytically: the free joint stores the body-origin
velocity `v - w x (R c)` in world axes and `R^T w` in body axes. Hinged bodies
take the momentum-consistent projection a constraint impulse produces,
`J^T h / (J^T M J)` with J mapping the rate to angular `u` and linear `u x r`,
also in closed form. Upstream's `SetFreeGroupWorldAngularVelocity` copies a
world vector into those local coordinates, and upstream passes `fullinertia`,
which overwrites the inertial-frame rotation (`iquat`) and leaves the eigen
decomposition to MuJoCo; this adapter does neither. Diagonal tensors, including
every parity scenario, lower to identical values either way. Joint angle and
rate are the hinge `qpos` and `qvel`.

## Engine warnings and faults

As in upstream, only MuJoCo's bad qpos, qvel and qacc warnings are fatal. After
them MuJoCo has already reset its data, moving every body to its initial
configuration and simulation time to zero, so `step` returns `Native` (through
`StepFault`) and the run is terminal. The reset also zeroes MuJoCo's warning
counts before counting the cause, so a repeated reset is detected by time going
back. Other warnings (contact or constraint buffer full, near-singular inertia,
bad control) do not fault the run: `step_warnings()` reports those whose count
rose in the last step. Counts are cumulative, so MuJoCo logs only the first
occurrence of each per compiled model through the installed handler. A private
test overflows a real 1 MiB contact arena with sixty overlapping spheres: each
step reports "contact buffer full" and time advances without a reset.
A backend-level regression also runs sixty admitted, overlapping spheres through
`MujocoBackend::step` with an explicitly reduced 1 MiB test arena. Two real
advancing steps expose the warning without resetting time or losing body state;
a zero-duration call retains the last warning, and removal/reset recompiles
with the normal arena and restores a warning-free run. No warning counters are injected in this test. The reduced
arena exercises the actual recovery path; it does not characterize the default
arena's capacity or contact accuracy when contacts are dropped.

Fatal MuJoCo errors, including allocation failure in `mj_makeData` or arena and
stack exhaustion reported through `mju_error`, abort the process.

## Explicit limitations

- A second world joint on one body returns `PhysicsError::Unsupported`
  (`"more than one world joint on a body"`): MuJoCo joints add degrees of
  freedom, while Rapier joints add constraints.
- `remove` returns `false` when the body is absent. `remove_joint` also refuses
  a free velocity beyond MuJoCo's limits or a failed compile of the current model.
- Fixed bodies carry a placeholder inertial instead of MuJoCo's geom-derived
  one; it has no dynamic effect.
- No meshes, planes, contact properties, joint limits, damping, actuators,
  sensors or contact reporting yet. No SDF lowering: the scene owns that.
- The log handler is process-wide and applies to any other MuJoCo user in the
  process.

## Evidence

`cargo test -p gz-physics-mujoco`: 61 tests. Ports of the Rapier physics,
joint, cylinder, capsule and private reset tests keep their assertions; exact
equalities that depend on Rapier arithmetic became 1e-12 bounds where MuJoCo
maps velocities through its mass matrix, and the complete-turn joint test
expects MuJoCo's linear hinge integration. Rapier's sleep-policy tests have no
MuJoCo counterpart. New tests cover compile counts, topology state preservation
(bitwise for free motion), hinge add/remove projection and continuity, exact
compiler-rule admission boundaries, rejection of unrepresentable velocities at
the call, uncompiled answers equal to compiled ones, injected compile faults,
a real contact-buffer overflow, fatal and recoverable warnings, timestep
retention, loader/version errors, arena growth and thread ownership.

Same-engine parity (`tests/parity.rs`) replays
`tests/fixtures/mujoco_reference.json`, produced by
`tests/fixtures/reference/mujoco_reference.cc` through the installed
gz-physics MuJoCo plugin (command and library hashes in its README). Every tenth
1 ms step of a free-falling sphere (1000 steps), a sphere dropped onto a static
box (2000 steps) and a world-revolute pendulum (4000 steps) matches position,
orientation, linear and angular velocity and hinge angle/rate within 1e-14;
the worst observed difference is 8.9e-16. These scenarios have centered
inertia and no off-axis angular velocity, so they do not exercise the
COM-versus-origin or local-versus-world velocity conversions; the analytic
tests in `tests/topology.rs` cover those.

Hosted CI needs `libmujoco.so.3.11.0` (for example the official 3.11.0 release
or the gz-physics vendored build) on the loader path or via `MUJOCO_PREFIX`;
without it every backend test fails at `MujocoBackend::new`. Regenerating the
parity fixture additionally needs gz-physics with its MuJoCo plugin, gz-plugin
and sdformat.
