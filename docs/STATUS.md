# Implementation status

This is an initial implementation, not a complete rewrite or accepted P2 slice.
All 78 simulator system migrations remain pending. No speedup is claimed.

## Ownership and model routing

The coordinator owns this monorepo and integration. One agent owns each independent
repository: sdformat-rs and gz-transport-rs use gpt-6.1-sol with high reasoning;
gz-msgs-rs uses the same model with medium reasoning. Agents work on bounded
packets and independently review one another's changes. Fuel remains with its
existing agent. Four concurrent slots cover the coordinator and three repo agents.

## Implemented scope

| Crate | Behavior | Remaining boundary |
| --- | --- | --- |
| gz-sim-core | Bevy schedules, deferred commands, exact/pause stepping, integer time/epochs, atomic overflow checks, stable public IDs and cleanup, world/allocator-bound reset reservations, isolated worlds | Persistence and complete lifecycle contract |
| gz-physics | Shared f64 body construction, inertial, rotation and state values | Shared backend capability contract and TPE integration |
| gz-physics-tpe | Validated constant-velocity Body, native stationary/tiny-angle semantics and atomic numeric errors | Bevy adapter, contacts, lifecycle/commands and D23 policy |
| gz-physics-rapier | Full-f64 sphere/box dynamics, inertia tensors, static bodies, world impulses, gravity, backend reset/removal, prepared initial-state restoration and numeric validation | Articulations, forces, common capabilities, DART/MuJoCo/TPE adapters |
| gz-sensors | Noise-free IMU specific force and relative orientation using f64 glam math | Rates, seeds/noise, complete frame configuration and remaining sensor families |
| gz-sim-scene | Atomic SDF construction, Bevy ownership hierarchy, canonical model and local poses, IMU sampling, checked model-subtree removal and loaded-scene restoration | Joints, offset/multiple shapes, asset resolution and scheduling |
| gz-sim-cli | Self-contained SDF input, exact integer steps, sorted physical-link state and typed load errors | Legacy grammar, transport serving, resource resolution, signal handling and packaging |
| gz-sim-server | Bounded clock/pose publication, shared-topic IMU streams, polled pause/step/reset controls, deferred remove services, real C++ clients | Root clock election, run-to controls, scene services and runtime sensor registration |

Rust 1.95.0, Bevy ECS 0.20.0, Rapier3d-f64 0.36.1 and glam 0.33.12 are pinned.
Authoritative state and solver use f64. The submillimeter motion probe at world
coordinate 10,000,000 m passes. The Linux normal dependency closure contains no
Bevy renderer/window or wgpu/winit dependency. Rendering profiles remain pending.

The backend is a capability probe, not the final physics interface. Invalid source
values, derived inertia/AABBs and impulse velocities fail before mutation. This
is not a solver-stability guarantee for arbitrary magnitudes or time steps. Direct
ECS access must preserve installed resources and scene ownership invariants;
system panics invalidate the run rather than rolling back state.

## Current evidence

The current local gate passes 106 Rust tests and nine gate tests, covering 1807/1817
production lines and 2767/2815 regions. Every production crate exceeds 95% for
both metrics. The TPE motion crate covers 48/48 lines and 73/73 regions; all 46 changed
executable lines are covered.
Independent review and the exact staged hook are required before publication.
[HEADLESS.md](HEADLESS.md) records the behavior and remaining integration scope.

Published clock/control, IMU and topic-repair revisions passed clean GitHub CI.
The hierarchy/pose revision d3524f5 passed its exact hosted gate.
[CORE-01 is accepted](acceptance/CORE-01.md); API-01, QLT-01 and the broader phases
remain open. Local model removal is published at 5fd5e49 and remote removal at b39ea419.
The consumer now uses reviewed SDF frame/topic/canonical helpers and rejects
explicit Surface and fluid added-mass physics until implemented; transport includes native publisher
recreation ordering. Producer pins and tests are detailed in HEADLESS.md. The C++ client now
verifies six scenarios using actual clock, local-pose and IMU samples, exact-step
controls, stable IDs, topic repair, deferred model removal and reset/resume. Its
native dependencies are rebuilt from the pinned reference lock in CI.

`python3 scripts/verify.py` runs formatting, Clippy with warnings denied, Rust and
gate tests, and LLVM line/region coverage. Every owned production source must be
instrumented. Per-crate and changed executable-line coverage must exceed 95%;
exactly 95%, empty denominators, missing/duplicate profiles and malformed mappings
fail. The staged hook verifies the exported Git index and checks it is unchanged.
Fresh counts, source hashes, toolchain and logs are written to artifacts/coverage.
Independent repositories retain their own coverage and test obligations.

Feature matrices, build-script input accounting, native/shader coverage, fuzzing,
branch protection and performance qualification remain open under QLT-01 and
TESTING.md. No test retirement or coverage exclusion is implied by these runs.

## Reference scope and next work

The local C++ math build passes 188 CTest entries, including wrappers and Python
tests. Seven original IMU unit and three integration cases pass. The Rust IMU
measurement port retains assertions/tolerances from SensorReadings without retiring
the full original case. An isolated Clang simulator build passes ten ServerConfig
and fourteen SimulationRunner cases; its GCC build encountered an upstream
compiler issue. Selected native case IDs and hashes are in
planning/reference-results.json. These installed dependencies are not yet a
complete reproducible reference environment.

The untouched native UserCommandsTest.Remove case and all 18 parameterized
ResetToWithAddedEntity/DeletedEntity cases pass. Their source/binary/output hashes
and partial or pending Rust mappings are recorded in planning/reference-results.json.
These use the same qualified installed dependency environment.

Additional native fall, nested/canonical-pose and remove/reset probes guide the
headless work. Loaded-scene reset now restores initial logical IDs, nodes and prepared physics,
with native reset-time removal observations and preserved Bevy infrastructure.
The separate core reset remains limited to retained domain state. Remote reset
now composes control ordering, real time and stream reconstruction. SCENE_RESET.md
and SERVER_RESET.md record supported boundaries, cancellation differences and ports.

Next: integrate TPE, local/Fuel resources and sensor scheduling, and complete
the first slice's simple-joint variant and velocity/force inputs. Record the
equal-work reference comparison and performance report for P2. Then
complete the retained backend, sensor/system, rendering, plugin, packaging and
release matrices. CORE-02, full IO-01 and P2 are still open.
