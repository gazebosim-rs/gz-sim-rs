<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M23/M24: magnetic and planar CPU-lidar noise

The functional scope below is retained. [M21_M25_DELIVERY.md](M21_M25_DELIVERY.md)
now supersedes its intermediate-parent sequencing: all reviewed M21–M25 source
qualifies in one new candidate over accepted M20, with new full normal/exact CI
and every separate native/functional and fresh M25 distribution criterion before
accepting these phases together. Earlier checkpoint trees and results remain
unchanged historical evidence, not current whole-gate success.

Selected under D30–D32 for parallel source work while M21/M22 qualifies.
Each phase is bounded to five working days including independent review and
mandatory verification. At this Source-review checkpoint M1–M20 are accepted.
The worktrees started from M20 `9a3e81c` and are integrated on M21/M22 review
`36eb202c`, whose full normal hook, native and independent local DATA passed.
Reviewed provisional native capture, its independent DATA and the full normal
hook may overlap parent hosted CI, with local heavy work serialized. Child
retained qualification, acceptance and publication still require the exact
finally accepted parent and every child gate. Parent source corrections require
rebasing and affected review/checks. No acceptance transfers from a parent check.
This frozen source records its checkpoint; actual later acceptance is recorded
separately in release receipts, the tracker and the next metadata increment.

## M23: loaded magnetometer Gaussian noise

Extend the existing single loaded Rapier magnetometer with optional Gaussian and
Gaussian-quantized X/Y/Z noise. Reuse the maintained GaussianNoise/NoiseStream
library unchanged. Preserve the finite constant world field, resolved sensor
pose, ideal model, topic/frame, count/backend/metadata limits and sampling policy.
Apply noise after inverse-rotating the world field into sensor axes, in X/Y/Z
order, matching pinned MagnetometerSensor.cc. Native application has `dt=0`.
Admit finite parameters, nonnegative deviations/precision and signed finite means;
keep `none` default-only. Native warning-only negative precision is outside scope.

Only enabled channels own optional private noise state. Seed from the existing
load seed and public sensor identity, with deterministic Rust reset reconstruction.
Pause, absent demand and non-due acquisition consume no random draws. Distinguish
ideal and noise errors; a nonfinite noisy vector is a fresh invalid acquisition
and cannot republish a prior valid packet. Native global-RNG/reset byte equality
is not claimed.

Use an asymmetric world field, rotated sensor and distinct signed channel means
to detect noise applied in the wrong frame. Retain ideal/refusal controls and add
one-channel/default, rounding, seed/coexisting-stream, zero-dt bias/rate,
demand/pause/reset/reconnect/removal and overflow controls. Observe actual canonical
Magnetometer packets with frame/time/sequence. A new SensorFactory helper supplies
deterministic vectors/rounding and retained upstream stochastic moment checks;
it does not replay an accepted physical trajectory or establish RNG-bit parity.

## M24: loaded planar CPU-lidar white Gaussian noise

Admit only finite Gaussian mean and nonnegative finite standard deviation;
all bias, dynamic-bias and precision fields remain default. Gaussian declarations
also require `range_min == 0`, including inactive near-zero configurations.
Existing default-only noise-free scans retain every accepted nonzero-minimum
behavior. Refuse Gaussian-quantized, noisy nonzero minima and nondefault hidden
fields explicitly. Reproduce the pinned
CpuLidarSensor.cc near-zero activation rule for the admitted mean/stddev subset.
Do not copy the broader generic lidar noise path: CPU activation, skipped beams
and clipping are distinct contracts.

Use one optional model/stream per sensor. Apply zero-elapsed noise to finite raw
physical hit distances in beam order, then clamp finite results to zero/maximum.
Validate physical hits and noisy results before clipping. Positive-infinite misses
remain infinite and consume no random draw. Native rays start at the minimum and
classify out-of-range results before noise; the zero-minimum admission bound keeps
this phase's physical ray origin and finite domain aligned. Do not claim parity
for injected out-of-range native fractions or change accepted ideal clipping.
Preserve the unchanged pure
ideal geometry/model, Rapier queries, 2–128 beams, planar/resolution-one/full-mask
limits, one loaded lidar and metadata/rate/backend restrictions. No GPU lidar,
intensity model, point cloud, per-beam worker or transport change is included.

Keep the existing terminal LidarFault policy. Nonfinite noisy output must fail
closed, retire the previous scan and publish no partial or stale scan; do not
turn overflow into a finite clipped result. Preserve absence/demand/no-due
behavior and reconstruct seeded state on direct/loaded reset.

Tests cover signed mean, lower/upper clipping after noise, refusal of Gaussian
noise with nonzero minimum, unchanged noise-free nonzero-minimum clipping, mixed
finite/miss beam order and skipped draws, activation boundaries,
seed/reset/independent streams, actual analytic Rapier geometry, sample world
pose/count/frame/time/sequence, refusal/overflow, pause and teardown. Keep all
original assertions and bounds. A new CPU SensorFactory helper uses bounded
GenerateRays/SetRaycastResults inputs and the actual provider; it needs no physics
or rendering plugin rebuild and makes no trajectory or performance claim.

## Ownership and execution

| Owner | Worktree | Exclusive files |
| --- | --- | --- |
| Light | `gz-sim-m23-magnetometer-noise` | Scene `src/magnetometer.rs`, `tests/private/magnetometer.rs`, new `tests/magnetometer_noise.rs` and `tests/native_magnetometer_noise.cc`; Server `tests/private/magnetometer.rs`, `magnetometer_io.rs`, `magnetometer_world_io.rs`; new `examples/m23_magnetometer_noise.sdf` and `docs/M23_MAGNETOMETER_NOISE.md`. |
| Messages | `gz-sim-m24-cpu-lidar-noise` | Scene `src/lidar.rs`, `tests/private/lidar.rs`, new `tests/lidar_noise.rs` and `tests/native_cpu_lidar_noise.cc`; Server `tests/private/lidar.rs` and `lidar_io.rs`; new `examples/m24_cpu_lidar_noise.sdf` and `docs/M24_CPU_LIDAR_NOISE.md`. |
| Root | `gz-sim-m23-m24` | Shared exports/attachments only when necessary; integration, canonical plan/docs/tracker, qualification and publication. |
| SDF | Read-only | Independent source/test-intent review and batched findings. |

Authors run no tests, lint, native capture or products. Do not edit shared noise,
pure sensor models, Cargo/dependencies, protocols, viewer, physics or another
owner's files. Coordinate a concrete additional lease before using another file.
Preserve all accepted and qualifying trees, old evidence and Fuel work.

Complete implementation, meaningful regressions, documentation and independent
source review; batch findings before one consolidated required verification.
Root alone serializes local heavy work. Reuse unchanged evidence only where the
reviewed checks permit it. Both phases require their focused native evidence,
full normal hook, strict workspace/per-crate/actual-parent coverage, exact hosted
CI and independent retained audits before promotion or acceptance. Batch actual
failures before affected reruns without weakening assertions, tolerances or gates.

This is a source-feature wave. M20 remains the installed preview; no archive is
silently relabeled. Update the web tracker truthfully and continue the next useful
bounded delivery after acceptance while retained scope remains.
