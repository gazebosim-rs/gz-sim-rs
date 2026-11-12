<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M47: Gaussian odometry velocity noise

Select one bounded increment, budgeted at most five working days including
review and mandatory checks: admit the existing OdometryPublisher
`gaussian_noise` option in dimensions 2 and 3. Noise changes the six reported
Twist components, not the physical robot, pose, orientation or TF. This phase
does not refresh the installed preview or add a new noise framework.

Development starts at exact clean local M46 commit
`cca6b07d9dd0f8f85c98e0de9b9488780bfa7d83`, tree
`ebcf1dc3cfef068b372e1bb5fa1f7c48c1fbd032`. Its complete normal hook passed
in 2050.424 seconds and independent retained local audit passed: 1735 headless
and 1034 renderer profile passes with overlap, 402 aggregate Python tests and
20 finite examples. Both earlier failed normal attempts remain retained.
This is locally qualified source, not accepted Main. M1–M42 remain accepted;
M43–M46 hosted/publication closeout remains separate. Preserve those frozen
trees and all original evidence.

## Current Source and reference environment

Implementation, regression Source and the feature guide are complete and passed
independent whole-batch review. The single offline formatter passed in 1.220
seconds; its eight owned Rust-file deltas passed integration review. No product
test, lint or full normal-hook result is claimed at this source checkpoint.

The first full normal hook returned actual exit one after 178.137 seconds at
required Clippy compilation, before tests or coverage. One retained M46 Server
replay initializer omitted the newly required `gaussian_noise` field. The complete
21-constructor/alias inventory found no further omission. Its correction adds
only `Default::default()` noise to that initializer, preserving every original
numeric and wire assertion. The failed source/tree/logs remain retained; no
native reference, audit or formatter is repeated. A separately reviewed complete
normal retry is still required; no Rust or coverage pass is claimed.

Two pre-child reference invocation failures are retained separately. The first
omitted required command arguments and stopped at argument parsing. The next
stopped at the saved SDK hash guard before any compiler or helper started.
Read-only comparison of all 349 saved nodes found 348 unchanged and exactly one
changed installed MySQL client library. The package log records an upgrade from
`8.0.46-0ubuntu0.24.04.4` to `.5`; its current bytes match the installed package
checksum. The initiator is unknown. SONAME and DT_NEEDED are unchanged; this is
not a claim of binary equivalence or transferred qualification.

A separately reviewed current reference manifest retains all 349 identities
and the original immutable manifest, with the single changed node explicitly
bound to the current installed package. Every current-node hash, loaded-provider,
pre/post, resource and actual-exit guard remains. The reference workflow rebuilt
no SDK, component, physics backend or provider and performed no package operation;
no old result is relabelled. The fresh original-component capture returned actual exit zero in 58.053 seconds
against that current baseline. Independent retained native DATA also returned
actual exit zero, checking all 16 modes, 18 configurations, 1418 inputs, 1316
triples, 3948 canonical packets and 102 suppressed inputs. Each stationary
Two/Three case supplied 512 actual observations; all six means and population
variances satisfy the unchanged upstream bounds, and both covariance matrices
match. All 349 current nodes and 337 mapped providers were checked. This is
scoped component evidence, not Rust/full-normal or phase acceptance. This environment maintenance does not
establish Ubuntu snapshot recovery or release any CI retry.

## Behavior

Follow pinned Sim `71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`
`src/systems/odometry_publisher/OdometryPublisher.cc` and its existing
`GaussianNoiseTest`. After noiseless pose-derived rolling means, acquire six
zero-mean Gaussian values in linear X/Y/Z, angular X/Y/Z order. Acquire once
per eligible history update, before publication throttling and independently
of subscriber demand. Initialization, pause and elapsed intervals at or below
one microsecond do not draw. Two-dimensional mode still noises linear Z and
angular X/Y; its pose remains planar. Do not feed noisy velocities back into
the pose or rolling means.

One noisy sample supplies both Odometry and OdometryWithCovariance. TF retains
the same unmodified pose/header. Native fills BOTH pose and Twist covariance
matrices: 36 floats, sigma squared on all six diagonal entries (indices
0, 7, 14, 21, 28 and 35), positive zero elsewhere, including in Two mode.
This seemingly unusual pose covariance is intentional native behavior.

Admit finite nonnegative standard deviations only, with sigma squared finite
and representable as finite wire `float`. Reject invalid or overflowing input
before loaded-scene mutation; do not serialize infinity or clamp covariance.
Omitted and explicit signed zero share the exact existing noiseless path.
Native Math treats sigma at or below f64 epsilon as no random contribution;
positive tiny sigma still retains its configuration identity and covariance.

Reuse the existing independently owned ChaCha8 noise stream and loaded seed.
Key each publisher stream by stable public model identity. Healthy loaded
reset restores its initial stream; removal retires it, reconstruction starts
the configured stream again. No owner shares mutable RNG state with another
sensor. No random state or allocation is required for the no-draw domain.
Document that deterministic Rust stream replay is a project contract, not
bitwise identity with C++'s process-global standard-library RNG or its reset.

Configuration must be immutable and checked. Scene ownership and Server
prepared/reset/stream identity must include it. Preserve the current terminal
fault, invalid metadata/sample, timing, history, throttle, removal and reset
contracts. A failure must not publish a partial noisy sample or successful
nonfinite packet. Exact zero-path arithmetic and all M34/M38/M39/M46 numeric
fixtures remain unchanged.

Use `GroundTruthOdometryNoise`, a Copy/Default/Eq value with private sigma bits,
checked `from_stddev(f64)`, `stddev()` and `variance() -> f32`, in the new
`GroundTruthOdometryPublication.gaussian_noise` field. Existing Publication
literals receive explicit default noise under compatibility review; the Sample
type need not change. Use a specific `GroundTruthOdometryError::Noise`
for invalid configuration, preserving `Arithmetic` for nonfinite runtime output.
Reuse `NoiseStream` through a focused checked white-Gaussian method, delegating
to its existing normal sampler without creating a bias model or consuming an
initial bias-sign draw. Do not change its RNG algorithm or existing consumers.

For the nonzero-noise branch, stage only mutable publisher state including filter
history and RNG, without copying Publication strings or heap boxes each time.
Commit after successful calculation and finite output
validation. Throttled eligible acquisitions commit history and draws even when
no new sample is published. A failed candidate commits neither. Keep the exact
zero branch unchanged. Two-mode out-of-plane Twist zero checks still apply
through the native no-draw threshold, not merely to canonical zero sigma.
Loaded snapshot reconstruction supplies stream/history reset; bare
`Simulation::reset()` continues its existing sample-clear contract and does not
gain a new replay promise.

## Source lanes and evidence

Root owns the contract, public API agreement, shared loader/export seams,
qualifier and peer registration, retained-control mappings, example and
planning/tracker integration. The Scene and Server/reference leaves may be
authored independently after contract review, with an independent whole-batch
review before checks. No Cargo, lockfile, workflow, Fuel, renderer, backend,
package or launcher changes are selected.

Replace each of the five original positive-noise refusal inputs (four external
tests and one private parser test) with a
stronger positive control in its original mode and offset context; retain a
new invalid-input refusal in its negative table. Cover all six channels,
finite-wire variance admission, tiny/zero domains, seeded repeatability and
isolation, history-before-throttle, pause, reset, configuration mutation,
removal, real force/torque body equivalence, unchanged pose/TF and complete
covariance and three-stream wire messages. Old assertions and tolerances are
not weakened. The existing twenty-example prefix remains intact; add one
finite M47 example without changing the old installed package.

Fresh scoped original-system observations must demonstrate Two and Three
noise placement and covariance. Preserve the original Gaussian test's six
mean and population-variance assertions (sigma 1, means 0 +/- 0.5, variance
1 +/- 0.5), with at least 500 stationary observations EACH in Two and Three
mode and explicit upper limits. Zero and tiny-noise observations establish
covariance; together with pinned Rand Source they establish draw suppression.
Zero observed output alone does not witness an unconsumed random stream. These
observations do not establish cross-generator random-byte parity. Review the actual
helper/procedure before execution, reuse the existing original component and
providers, and do not rebuild the SDK or run native Physics trajectories.
Real C++ Zenoh controls separately check shared samples, full covariance,
pause/reset and owned shutdown. Record actual outcomes; no placeholder
fixture or inferred success is allowed.

Complete implementation, regression Source, documentation and independent
review; batch findings before one consolidated normal hook with unchanged
strict workspace, per-crate and actual-parent coverage. Root alone releases
and watches local heavy operations with the existing floors and reservations.
Provisional local checks require complete procedure review first. Actual
accepted predecessor, exact hosted success, independent official evidence,
normal Main/readbacks and an acceptance record remain mandatory for source
publication and acceptance. A changed accepted predecessor needs integration
review and affected checks; no evidence transfers automatically. No CI retry,
snapshot probe, mirror/pin/workflow workaround, eviction or acceptance waiver
is authorized by this phase.
