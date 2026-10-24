# Reuse policy and dependency evaluation

Use a supported stable Rust toolchain and released dependencies. “Stable library”
means acceptable maintenance, correctness, release discipline, license, and API
cost; a 1.0 version alone proves none of those. Bevy and the physics candidates
are pre-1.0, so upgrades need explicit integration testing and version alignment.
No dependency versions below are a resolved Cargo.lock.

Before adopting a dependency, record the exact release, MSRV, license, enabled
features, native requirements, maintenance/advisory review, public API exposure,
test corpus results, and replacement cost. Prefer std first. Avoid two libraries
for the same job unless a measured semantic or performance need justifies it.

## Avian versus Rapier

Evaluation date: 2026-10-24. The following is a source/documentation assessment;
no comparative Gazebo benchmark has been run in this planning session.

**Owner-approved direction: prioritize Rapier's full-f64 engine, integrated into
the simulation's Bevy schedule through a workspace plugin, and validate it with
probes.** The owner requires full solver precision. Avian remains in the documented
comparison, but 0.8 fails that precision gate. Do not commit to maintaining both as release backends without a
demonstrated need; DART, MuJoCo, and TPE already add substantial validation work.

| Criterion | Avian | Rapier | Implication |
| --- | --- | --- | --- |
| ECS integration | Designed around Bevy ECS | Standalone engine plus official Bevy integration | Avian may reduce state synchronization; measure actual costs |
| Current version pairing | Published table pairs Avian 0.8 with Bevy 0.20 | Official plugin releases target specific Bevy releases | Pin a tested combination; never combine arbitrary latest versions |
| Numerical precision | 0.8 changed f64 to world-space coordinates; most solver math remains f32 | Separate rapier3d-f64 engine is documented | Rapier is the stronger initial fit for full-f64 robotics; widening f32 results does not recover accuracy |
| Independence | Closely follows Bevy | Core engine can be integrated independently of its Bevy adapter | Direct f64 adapter can isolate Bevy upgrades and control stepping |
| Robotics fidelity | Joints/motors exist; no Gazebo parity claim established | Joints/articulation features exist; no Gazebo parity claim established | Probe loops, motors, limits, wrench feedback, and contacts for both |
| Determinism | Deterministic-math feature is available | Documented enhanced-determinism conditions and feature conflicts | Neither makes an uncontrolled ECS/IO schedule deterministic |
| Synchronization | Native components can share the main ECS | Backend structures require mapping and state transfers | Batch and measure Rapier transfers; avoid per-field conversions |
| Performance | Release notes report improvements in their workloads | Parallel/SIMD/profile options exist | No winner until equal-fidelity Gazebo workloads run |
| Maintenance cost | Bevy coupling and recent precision change | Custom f64 Bevy integration may be required | Compare tested adapter surface, upgrade work, and precision policy |

Avian 0.8's precision change is stated by its maintainer; older examples that
describe an entirely f64 solver must not be copied into this design. Older Avian
releases may offer different precision but would need a supported Bevy pairing
and maintenance assessment. [Avian 0.8 release notes](https://joonaa.dev/blog/14/avian-0-8)

The published crate table pairs Bevy 0.20 with Avian 0.8 and 0.19 with Avian 0.7.
Treat that as compatibility information, not certification of our workloads.
[Avian crate documentation](https://docs.rs/crate/avian3d/latest)

Rapier documents separate f64 engine crates. Verify the exact official Bevy
adapter's dependency precision before adopting it; support in the core engine
does not prove the adapter exposes it. A focused Bevy plugin around rapier3d-f64
is the proposed fallback, not a fork of the solver.
[Rapier setup and precision](https://rapier.rs/docs/user_guides/rust/getting_started/),
[Rapier Bevy integration](https://rapier.rs/docs/user_guides/bevy_plugin/getting_started_bevy/)

Determinism claims have explicit prerequisites, including construction order,
compiler/engine versions, initialization math, and feature selections. Record
them in the release profile and test the application, not only the engine.
[Rapier determinism](https://rapier.rs/docs/user_guides/rust/determinism/)

### Required selection probes

Build the Bevy harness for candidates that pass the precision gate. Avian 0.8
can be rejected from the release shortlist on its documented precision; any
mixed-precision experiment is supplementary and cannot qualify it. A full-f64
Avian alternative would first need a supported dependency/maintenance plan.
Use identical physical scenes,
canonical units, target accuracy, and step schedules. Record solver iterations,
substeps, precision, thread count, and synchronization overhead separately.

1. Free fall, pendulum energy/drift, rotating inertia, and large-coordinate
   millimeter motion. Include large mass ratios and small joint motions.
2. Revolute/prismatic/fixed/free joints, limits, damping, motors, mimic behavior,
   closed loops, and force/torque outputs. Unsupported features must fail clearly.
3. Contact friction/restitution, stacks, fast thin-object collision, meshes,
   heightmaps, collision masks, and ray/shape queries.
4. Spawn/remove/reset and repeated topology edits; stable ID mapping; no leaks.
5. Many sleeping bodies, many active robots, camera/lidar load, and low-entity
   latency. Compare accuracy first, then throughput and p99 latency.
6. Repeated runs with fixed seeds and changed worker counts. Document the actual
   determinism boundary; cross-backend bitwise equality is not a requirement.
7. Headless build size/startup and no-render dependency closure. Reject accidental
   default features that pull in windowing or debug rendering.

Selection gates: required precision and tested capability coverage, maintained
release pairing, acceptable accuracy, measurable adapter cost, and reproducible
performance. Do not invent a numeric score in place of missing measurements.

## Math replacement map

The default canonical vector/quaternion representation should align with Bevy's
glam family using f64 types. Use nalgebra for operations it actually supplies
better, such as fixed 6x6/general matrices and decompositions. Keep conversions at
boundaries rather than repeatedly converting inside a step.

| gz-math area | Candidate / approach | Semantics that still require Gazebo tests |
| --- | --- | --- |
| Vector2/3/4, Quaternion, Matrix3/4, poses | glam f64 types; small pose/frame wrapper only if needed | RPY and quaternion conventions, normalization, composition, singularities |
| Matrix6, eigensolvers, decompositions | nalgebra fixed-size/dynamic matrices | Spatial vector ordering, inertia conventions, symmetry and conditioning |
| AABB/OBB, shapes, intersections, mass properties | parry3d-f64 plus engine geometry | Degenerate geometry, tangent cases, signed distances, inertia axes |
| Graph structures/algorithms | petgraph or smaller std-based representation | Scoped frame identity, stable indexing, cycle and disconnected-graph errors |
| RNG/distributions | rand family with explicit reproducible generator selection | Seed/version contract, per-sensor streams, reset, distribution/tolerance tests |
| Semantic versions, time, strings | semver and std | Legacy prerelease parsing and simulation-clock behavior |
| Spherical coordinates/geodesy | geographiclib-rs where covered; evaluate dedicated coordinate-conversion support | ENU/ECEF/NED, ellipsoids, local tangent frames; geodesic distance alone is not enough |
| Splines, polynomials, interpolation, statistics | Evaluate existing numerical crates against each fixture | Boundary extrapolation, continuity, accumulator precision, initialization |
| PID, limiters, odometry | Reuse only if exact semantics fit; otherwise residual gz-math modules | Saturation, anti-windup, derivative conventions, reset, diff/mecanum/swerve kinematics |
| Inertial/MassMatrix3 | Geometry library primitives plus residual domain validation/composition | Positive definiteness, triangle inequalities, parallel-axis theorem, frame rotation |
| Filters/Gauss-Markov noise/rolling statistics | Existing primitives plus concise domain wrappers | Bias evolution, time constants, zero dt, sample windows, reproducibility |
| Volumetric grids/scalar fields/occupancy | Evaluate existing storage/interpolation algorithms; retain domain gaps | Coordinate bounds, interpolation, time-varying fields, environmental data contracts |
| Material/color/temperature/angle | Bevy/color utilities and explicit unit wrappers | Color space, thermal units, angle wrapping, material density tables |

These are reuse decisions to validate, not claims of drop-in equivalence.
Primary API references: [glam](https://docs.rs/glam/latest/glam/),
[nalgebra](https://docs.rs/nalgebra/latest/nalgebra/),
[Parry](https://docs.rs/parry3d-f64/latest/parry3d_f64/),
[petgraph](https://docs.rs/petgraph/latest/petgraph/),
[geographiclib-rs](https://docs.rs/geographiclib-rs/latest/geographiclib_rs/).

For every retained custom algorithm, add a short decision: upstream contract,
libraries evaluated, exact gap, minimal owned implementation, test IDs, and
benchmark impact. “Gazebo has a class for it” is not a reason to port it.

## Common functionality and integration candidates

| Responsibility | Preferred direction | Acceptance conditions |
| --- | --- | --- |
| Files/env/process/time/ownership | std | Match platform errors and lifecycle behavior; no unnecessary wrapper framework |
| CLI | clap | Existing grammar, aliases, help/exit codes, dispatch and signal tests |
| Logging/profiling | tracing, tracing-subscriber, Bevy diagnostics, native profiler bridges | Cheap disabled paths; bounded sinks; preserve useful metadata |
| Serialization/config/errors | serde, serde_json, typed errors such as thiserror | Versioned schemas, precise errors, no silent defaults on malformed input |
| XML | quick-xml or roxmltree as parsing machinery in sdformat-rs | Preserve SDF schema/conversion/frame semantics in owned code |
| Images/meshes | Bevy loaders, image, glTF/STL/OBJ ecosystem candidates | Fixture fidelity, f64 collision geometry where needed, bounded decode |
| Collada/complex animation | Evaluate maintained Rust loader; use a narrow native adapter if gaps remain | Material, skinning, transform, unit, and animation tests; no silent format loss |
| Geospatial/DEM | Rust geospatial crates; optional GDAL adapter when required | CRS, nodata, origin/resolution, terrain and altitude fixtures |
| Video/audio | Maintained Rust bindings to existing media libraries where needed | Encode/decode/timestamps and headless operation; native dependencies documented |
| Concurrency | Bevy tasks plus bounded channels; rayon only where measured | Avoid duplicate global pools, unbounded tasks, and blocking in schedules |
| Fuel/network/archive | Consume gz-fuel-tools-rs; standard HTTP/archive crates remain there | Offline cache, cancellation, errors and Rust API agreed with existing owner |
| Native DART | cxx with minimal C++ bridge | Ownership, thread, exceptions, batching, real native tests |
| Native MuJoCo | Maintained binding or small generated C binding plus safe wrapper | Versioned headers/library alignment and tested callback/error contracts |
| Runtime extensions | libloading with owned C ABI wrapper | ABI negotiation and lifecycle tests; no Rust layout exposure |
| Logs | gz-transport-rs format support; choose rusqlite/compression only after format audit | Existing logs read/write/replay, schema and ordering fidelity |

XML parsing, C++ bridging, and library loading are supplied mechanisms, not the
domain policies this project must implement.
[quick-xml](https://docs.rs/quick-xml/latest/quick_xml/),
[cxx](https://cxx.rs/), [libloading](https://docs.rs/libloading/latest/libloading/).

## Dependency controls

P1 creates a compatibility manifest for Bevy, Rust/MSRV, native engines, protobuf
tools/runtime, math versions, and independent repository revisions. Run cargo
metadata/tree checks to prevent duplicate Bevy versions and forbidden dependency
directions. Pin CI tools/actions and release images. Audit generated/native code
licenses separately from handwritten Rust. Update dependencies in reviewed batches
with parity, precision, performance, and feature-matrix evidence.

Use native dependencies when they provide required functionality that a maintained
Rust library does not yet cover; do not rewrite mature physics or media stacks
merely to remove all non-Rust transitive code. Make any such dependency visible
in package manifests and release documentation.
