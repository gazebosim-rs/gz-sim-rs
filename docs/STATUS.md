# Implementation status

This is an initial implementation, not a complete rewrite or accepted P2 slice.
All 78 simulator system migrations remain pending. No speedup is claimed.

## Ownership and model routing

The coordinating agent owns this monorepo and integration reviews. One agent owns
each independent repository: `sdformat-rs` and `gz-transport-rs` use gpt-6.1-sol
with high reasoning; `gz-msgs-rs` uses the same model with medium reasoning.
The harder semantic and concurrency tasks receive more reasoning; routine schema
and conversion work stays at medium. Agents receive bounded packets and share
contracts, results, and review findings rather than entire source listings.
Fuel remains with its existing independent agent. Four concurrent slots cover
the coordinator and the three available repository agents.

## Implemented and verified

| Crate | Behavior | Remaining boundary |
| --- | --- | --- |
| gz-sim-core | Bevy ECS phase order, deferred-command boundary, paused ticks, exact stepping, reset epochs, atomic overflow checks, independent worlds | Domain reset, command ingress, lifecycle IDs, hierarchy, persistence, publication |
| gz-physics-rapier | Full-f64 sphere/box dynamics, static bodies, Z-up gravity, contact probe, stable local IDs, reset/removal, derived numeric validation | Articulations, forces, state exchange through ECS components, capabilities, DART/MuJoCo/TPE adapters |
| gz-sensors | Noise-free IMU specific force and relative orientation with f64 glam math | Rates, seeds/noise, frame configuration, publication and other sensor families |

The Rapier backend is an initial capability probe, not the final physics contract.
Its rigid bodies currently live in an ECS resource; scene entities and batched
component synchronization are pending. Invalid source values and unrepresentable
derived inertia or AABBs fail before insertion. This does not prove solver stability
for arbitrary magnitudes or time steps. Removing a backend resource does not permit
duplicate system registration; callers must preserve installed system resources.
Simulation panics invalidate the run; state rollback is not provided.

Rust 1.95.0, Bevy ECS 0.20.0, Rapier3d-f64 0.36.1 and glam 0.33.12 are pinned with
Cargo.lock. Authoritative state and solver use f64; the submillimeter motion probe
at world coordinate 10,000,000 m passes. Rendering has not been added.

## Evidence and gates

`python3 scripts/verify.py` runs rustfmt, Clippy with warnings denied, Rust tests,
doc tests, gate regression tests, and LLVM line/region coverage. Every owned
production Rust source must be instrumented. Each crate must exceed 95% in both
metrics; empty denominators and duplicate instrumentation fail. The staged hook
exports the Git index and verifies that exact tree, then checks the index is unchanged.
The GitHub workflow runs the same checks; a local pass is not a claim of a hosted run.

The initial three-crate run passed 15 Rust tests and four Python gate tests:
core 124/125 lines and 139/144 regions; Rapier 126/126 and 189/190; sensors 26/26
and 45/45. Fresh machine-readable counts, source hashes, toolchain identity,
and test logs are emitted under `artifacts/coverage`. Counts are observations,
not fixed targets. These gates are the current profile only: release feature
matrices, native/shader coverage, changed-line enforcement, fuzzing, benchmarks,
and branch protection from TESTING.md remain pending. An owned build script
requires gate support for forced execution and object collection before acceptance.

The existing local C++ gz-math build passes all 188 CTest entries (including
result wrappers and Python tests; these are not 188 distinct gtest cases).
Original IMU tests pass: seven unit and three integration cases. The Rust IMU
port covers only the measurement assertions within `ImuSensorTest.SensorReadings`,
preserving the original tolerances; it does not retire the full test.
The GCC simulator build encountered a dependent-template static_assert compiler
issue. An isolated Clang build succeeds without changing upstream source. Native
reference runs and any remaining failures are recorded separately. Existing
local native dependencies are not yet a reproducible reference container.

Independent review by the messages agent covered clock/scheduling and Rapier.
It identified duplicate installation after resource removal and extreme derived
numeric values; both were fixed and re-reviewed. Sensor assertion review is
recorded in the test ledger. No test retirements or coverage exclusions are used.

## Next acceptance sequence

1. Finish reproducible reference execution and assertion inventories (P0/REF-01).
2. Integrate typed SDF, ECS scene construction, physics, IMU, and transport into
   one exact-step world with real legacy clients (SLICE-01).
3. Finish every backend and sensor/system family against the plan's ledgers.
4. Qualify Bevy rendering, plugin ABI, packaging, complete coverage matrices,
   comparative performance and long-running tests. Only then claim rewrite parity.
