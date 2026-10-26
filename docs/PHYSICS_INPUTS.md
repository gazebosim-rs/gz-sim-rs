# Headless link inputs

`queue_link_world_velocity` replaces a pending world-frame linear/angular
velocity request. Units are m/s and rad/s. `queue_link_world_wrench` accumulates
world-frame force at the center of mass and world-frame torque, in N and Nm.
Both address public link IDs in a loaded simulation. The initial scope is a
top-level, unjointed model with one physical link. There is no transport input
endpoint or CLI command in this increment.

Admission validates the installed processor, current hierarchy/backend binding and loaded backend ownership,
finite values and healthy future clock before mutation. Accumulated force/torque
overflow leaves both fields unchanged. Fixed Rapier and TPE bodies reject input. Rotated TPE model/link frames accept
world-axis velocity commands under confirmed D23. TPE has no
external wrench feature and returns a typed Unsupported error. There is no
solver fallback or implied force effect.

Each target has at most one private pending Bevy component. Velocity replaces
velocity; wrenches add. No separate general command queue or ECS is introduced.
PhysicsPrepare applies inputs before one advancing step. Paused ticks retain
them; force/torque are cleared after that step, while the resulting velocity
persists. Core reset, loaded reconstruction and removal cancel pending input.
Backend reset also clears forces already admitted to the solver. Constructors
remain the source of reset state; velocity setters do not rewrite them.

Binding corruption after admission produces a typed terminal StepFault before
physics/time publication. The run cannot resume. Earlier input effects in the
same batch are not rolled back. Direct ECS/backend mutation must respect scene
ownership; private corruption tests verify error handling rather than promise
arbitrary transactional recovery. Finite input/storage checks do not guarantee
solver stability for every finite magnitude or timestep.

## Evidence and limits

The author gate passes 182 Rust checks, twelve verifier checks and all nine
existing real C++ peer scenarios: 2733/2767 production lines, 4288/4391 regions,
with every crate strictly above 95% for both metrics. Changed executable lines
are TPE15/15, Rapier56/56 and scene178/179. The native fixture test is included. Independent review and the normal staged
hook remain required before publication. No original assertions, exclusions or
thresholds changed.

Thirty fresh native Server controls use the real Physics system with DART/TPE,
world-velocity-reset components and one-step COM force/torque. Identity, model
rotation and link rotation distinguish world inputs from Link-local setters.
The probe checks pinned source bytes, actual loaded simulator/system/engine
hashes and a stable loaded-library set. Existing native build/installed SDK
provenance remains qualified. Reproduce using `scripts/probe-physics-inputs.py`;
`planning/reference/physics-inputs.json` records commands and hashes.

Rust checks supported velocity/angular observations against these controls.
Ten rotated TPE controls remain native observations. Rust now honors world axes
and explicitly checks the native rotation differences under D23; those controls
are not counted as matching trajectories. [TPE_POLICY.md](TPE_POLICY.md) records
the correction and preserved assertions. Identity TPE wrench requests
also return Unsupported, whereas native TPE ignores them. The retained velocity
is checked after that rejected request; no TPE force implementation is claimed.

DART and Rapier have different integrators. Force-pulse velocities agree within
1e-12. Positions are checked independently against the analytic force-pulse
trajectory with a half-Euler-tick error bound (0.5*a*dt²), explicitly allowing
first-order integration bias. This is not a same-engine trajectory port, an
upstream test retirement, or performance evidence. Selected-fixture P2 still
needs the simple-joint variant and its complete assertions/reference/performance
report. These local APIs cover its bounded free-body inputs when exercised by
the selected fixture; existing real C++ peer interoperability remains required.
Native wrench/velocity transport systems remain wider retained-plan work unless
the selected fixture needs them. Wider physics capabilities remain open.


Independent review found that a replacement Rapier backend could reuse a BodyId
and receive the original scene's input. Admission and execution now compare the
backend's opaque owner against the loaded initial state, without constructing a
solver. TPE's loaded world owner is also checked. The reviewer's exact colliding-ID
countertests now pass, strengthened with fault/time/foreign-state assertions;
additional TPE owner tests cover admission and runtime replacement. No existing
assertion or test was removed.
