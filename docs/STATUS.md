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
| gz-sim-core | Bevy schedules, deferred commands, exact/pause stepping, integer time/epochs, atomic overflow checks, stable public IDs and cleanup, isolated worlds | Full domain reset, persistence and complete lifecycle contract |
| gz-physics-rapier | Full-f64 sphere/box dynamics, inertia tensors, static bodies, world impulses, gravity, backend reset/removal and numeric validation | Articulations, forces, common capabilities, DART/MuJoCo/TPE adapters |
| gz-sensors | Noise-free IMU specific force and relative orientation using f64 glam math | Rates, seeds/noise, complete frame configuration and remaining sensor families |
| gz-sim-scene | Atomic SDF construction, Bevy ownership hierarchy, canonical model and local poses, IMU sampling, checked model-subtree removal | Full initial-scene reset, joints, offset/multiple shapes, asset resolution and scheduling |
| gz-sim-cli | Self-contained SDF input, exact integer steps, sorted physical-link state and typed load errors | Legacy grammar, transport serving, resource resolution, signal handling and packaging |
| gz-sim-server | Bounded clock/pose/IMU publication, polled pause/step controls, real C++ clients | Root clock election, reset/run-to/remove controls, scene services and sensor lifecycle |

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

The current local gate passes 59 Rust tests and nine gate tests, covering 1222/1232
production lines and 1804/1840 regions. Every production crate exceeds 95% for
both metrics. The removal increment covers 43/43 changed executable lines.
Independent review and the exact staged hook are required before publication.
[HEADLESS.md](HEADLESS.md) records the behavior and remaining integration scope.

Published clock/control, IMU and topic-repair revisions passed clean GitHub CI.
The reviewed hierarchy/pose revision d3524f5 passed its staged gate with 54 Rust
checks; its hosted run is tracked on the [dashboard](https://gazebosim-rs.github.io/rewrite-progress/).
The C++ client verifies all three scenarios using actual clock, local-pose and
IMU samples, exact-step controls, stable IDs, metadata and topic repair. Its
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

Additional native fall, nested/canonical-pose and remove/reset probes guide the
headless work. Full reset restores removed initial logical entities with their
original IDs; the current core reset only restores retained domain state. Remote
reset remains explicitly rejected until that complete contract is implemented.

Next: finish remote lifecycle/reset, TPE, local/Fuel resources and sensor scheduling;
record the equal-work reference comparison and performance report for P2. Then
complete the retained backend, sensor/system, rendering, plugin, packaging and
release matrices. CORE-02, full IO-01 and P2 are still open.
