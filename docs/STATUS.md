# Implementation status

This is an initial implementation, not a complete rewrite or accepted P2 slice.
All 78 simulator system migrations remain pending. No speedup is claimed.

## Ownership and model routing

The coordinator owns this monorepo and integration. One agent owns each independent
repository. Current sdformat-rs, gz-transport-rs and gz-msgs-rs owners use
gpt-6.1-sol with high reasoning for implementation and validation. Agents work on bounded
packets and independently review one another's changes. Fuel remains with its
existing agent. Four concurrent slots cover the coordinator and three repo agents.

## Implemented scope

| Crate | Behavior | Remaining boundary |
| --- | --- | --- |
| gz-assets | Bounded local files/configured directories, cache-only Fuel model includes, native lookup precedence, lossless source IDs and structured warnings | Downloads/cancellation, SDK/install/package policy and decoded resource caching |
| gz-sim-core | Bevy schedules, deferred commands, exact/pause stepping, integer time/epochs, atomic overflow checks, stable public IDs and cleanup, typed terminal fault diagnostics, world/allocator-bound reset reservations, isolated worlds | Persistence and complete lifecycle contract |
| gz-physics | Shared f64 body construction, inertial, rotation/state values and exclusive installation identity | Shared backend capability contract and TPE scene integration |
| gz-physics-tpe | Validated constant-velocity Body, Bevy-owned Motion, exclusive phase/reset installation, model overlap contacts and typed faults | Remaining geometry/commands and D23 policy |
| gz-physics-rapier | Full-f64 sphere/box dynamics, inertia tensors, static bodies, world impulses, gravity, world velocity/COM wrench inputs, backend reset/removal, prepared initial-state restoration and numeric validation | Articulations, remaining force frames and common capabilities, DART/MuJoCo/TPE adapters |
| gz-sensors | Noise-free IMU specific force and relative orientation using f64 glam math | Seeds/noise, complete frame configuration and remaining sensor families |
| gz-sim-scene | Atomic SDF construction, Rapier/TPE construction, backend-independent ECS observations, Bevy ownership hierarchy, canonical model and local poses, phase-preserving IMU sampling, checked model-subtree removal and loaded-scene restoration | Joints, offset/multiple shapes, full resource policy and remaining sensor contracts |
| gz-sim-cli | Local SDF/configured world input and include roots, Rapier/TPE selection, exact integer steps, sorted physical-link state and typed load errors | Legacy grammar, transport serving, remote top-level resources, signal handling and packaging |
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

The current local gate passes 182 Rust checks and twelve gate tests, including all
nine actual C++ client scenarios. Coverage is 2733/2767 production lines and
4288/4391 regions. Every production crate exceeds 95% for both metrics. The
input increment covers TPE15/15, Rapier56/56 and scene178/179 changed executable
lines. Scene coverage is 1145/1176 lines and 1778/1854 regions. No coverage
exclusions or thresholds changed. Independent review and the exact staged hook
remain required before this input increment is published.
[HEADLESS.md](HEADLESS.md) records the behavior and remaining integration scope.

Published clock/control, IMU and topic-repair revisions passed clean GitHub CI.
The hierarchy/pose revision d3524f5 passed its exact hosted gate.
[CORE-01 is accepted](acceptance/CORE-01.md); API-01, QLT-01 and the broader phases
remain open. Local model removal is published at 5fd5e49 and remote removal at b39ea419.
The consumer now uses reviewed SDF frame/topic/canonical helpers and rejects
explicit Surface and fluid added-mass physics until implemented; transport includes native publisher
recreation ordering. Producer pins and tests are detailed in HEADLESS.md. The C++ client now
verifies seven scenarios using actual clock, local-pose and IMU samples, exact-step
controls, stable IDs, topic repair, deferred model removal and reset/resume, including explicit TPE kinematics. Its
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

Next: complete the first slice's simple-joint variant and velocity/force inputs,
then its equal-work reference comparison/performance report and selected-fixture
assertions for P2. Full downloads and sensor noise remain wider-plan work, rather
than additional prerequisites for this local-resource, noise-free IMU fixture.
Then complete the retained backend, sensor/system, rendering, plugin, packaging
and release matrices. CORE-02, full IO-01 and P2 are still open.

## Offline Fuel integration

The workspace consumes public Fuel 3747086 and Transport 20320129 with its exact
Messages e2001a3 dependency; using one Messages identity preserves its TypeName
trait boundary. Transport's reviewed socket2 0.6.5 pin fixes the former Fuel/Tokio
resolution conflict. The actual root lock uses Tokio 1.54.1; hyper-util 0.1.15 keeps
its separate socket2 0.5.10 dependency. No constraints were relaxed or producer
source edited. Fuel cache lookup never invokes download/fetch/details, and misses
remain explicit. Both engines load through the cache, step and restore original
logical IDs/poses after the resolver and cached files are removed.

See LOCAL_ASSETS.md for limits, configuration and native-reference qualifications.
This is a model-include consumer, not completion of downloads, cancellation,
remote world/model-file URLs or the complete FUEL-01/P2 contract.

## Worktree hook isolation

The exported staged verifier removes official repository-local Git environment
keys only in its child process. The parent still selects and rechecks the intended
index. A new real-worktree/alternate-index regression checks child normal/bare
Git fixtures without redirecting them into the caller. Main HEAD ancestry, index
bytes, configuration, unstaged content and nonlocal settings are preserved. The
original wrapper fails the new test on leaked keys; the fix passes. This is a
verification-boundary correction; production Rust coverage and all seven native
client scenarios remain required. No root repository corruption is claimed.

## IMU acquisition rates

The scene now supports native-phase sampling rates independently of the physics
step, with checked time overflow, per-physics acceleration and pause/reset state.
SENSOR_SCHEDULING.md records native clock controls, actual C++ cadence/reset
observations and remaining scope. The former Rust-only 10 Hz rejection becomes a
stronger supported-input test; the error table retains a genuinely unrepresentable
1e-300 Hz period. That disposition requires independent assertion review; no
upstream test is retired or numerical tolerance relaxed.

## Exact staged source export

The verifier now exports through a private index and validates all raw Git blobs,
file modes, symlinks, directories and paths before running checks. The regression
that previously failed because export-ignore hid a staged file now passes. A
tracked changed-lines.json also survives unchanged; generated metadata stays
outside the exported source. Checkout conversion and deliberate snapshot mutations
fail closed. Simulator Rust and its native/coverage requirements are unchanged.


## World physics configuration

The CLI uses the first SDF physics profile's timestep when no explicit override
is supplied. Existing Duration APIs remain explicit overrides. Owned WorldPhysics
metadata survives loaded reset without IO. Three fresh native Server clock rows
match both backends; plugins were disabled, so this is clock/configuration
evidence only. Nondefault contact limits fail explicitly, and exact stepping
remains unpaced. Runtime profile changes and backend-specific settings remain
open. See HEADLESS.md and planning/reference/world-physics.json.


## Local physical input increment

A private pending Bevy component per target link provides world-velocity
replacement and one-step COM force/torque accumulation. The author gate passes
182 Rust checks, twelve tooling checks and the existing nine actual C++ scenarios;
2733/2767 production lines and 4288/4391 regions pass all per-crate gates.
Thirty fresh native world-input controls qualify supported velocity/angular
observations and explicit TPE gaps. Independent review and the normal staged hook
remain required before publication. PHYSICS_INPUTS.md records the scope and
analytic force-position accuracy bound. Simple joints and the complete selected-fixture assertions/reference/performance
report remain P2 work. These local APIs cover its bounded free-body input
capability when exercised by the selected fixture; native input transport systems
remain wider retained work unless that fixture requires them. Real C++ peer
interoperability remains required.
