<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Gaussian noise on pose-derived odometry Twist

M47 admits the existing OdometryPublisher `gaussian_noise` setting in dimensions
2 and 3, including existing translation and rotational offsets. Development uses
locally qualified M46 `cca6b07d9dd0f8f85c98e0de9b9488780bfa7d83`; that source
is provisional, not accepted Main. Accepted Main remains `73e03b9f`. Predecessor
hosted/publication closeout remains separate. This guide records Source and regression intent plus the scoped original-component
proof below; it supplies no Rust/full-normal, hosted or phase acceptance claim.
No installed preview or renderer refresh is included.

The checked `GroundTruthOdometryNoise` value owns private standard-deviation
bits. `from_stddev(f64)` rejects negative/nonfinite sigma and a square that is
nonfinite either as a double or after narrowing to the wire float.
`stddev()` returns the requested finite sigma; `variance()` returns the native
float-rounded sigma squared. Positive finite underflow to zero covariance is
not clamped upward. Explicit positive/negative zero normalize to default.
Positive tiny sigma remains distinct in configuration identity even when it
contributes no randomness. The new immutable
`GroundTruthOdometryPublication.gaussian_noise` field participates in existing
Scene/Server ownership and reset metadata equality. `GroundTruthOdometrySample`
has no new field. Invalid configuration uses the specific `Noise` error;
nonfinite runtime calculation uses the existing `Arithmetic` error.

## Placement, covariance and exact zero behavior

Pinned Sim `71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`,
`src/systems/odometry_publisher/OdometryPublisher.cc`, adds six zero-mean Gaussian
contributions after the pose-derived rolling means and before publication
throttling. Their order is linear X/Y/Z, angular X/Y/Z. The two-dimensional
linear Z and angular X/Y values are noise-only, not forced zero when randomness
is enabled. All eligible histories acquire noise even if no message is due or
no transport subscriber is present. Initialization, paused dispatch and elapsed
intervals at or below one microsecond do not draw. Noisy velocities never feed
back into pose, physics or the filter histories.

Odometry and OdometryWithCovariance share the same actual noisy Twist. Pose,
orientation and TF stay identical to the noiseless calculation. Native fills
BOTH pose and Twist covariance matrices with sigma squared on all six diagonal
entries, indices 0, 7, 14, 21, 28, 35, and positive zero elsewhere. Each entry
is rounded through `FloatV`'s float representation. Nonzero pose covariance
therefore does not mean pose samples were randomly perturbed. This behavior
also applies in Two mode.

The complete prior numeric body remains the exact zero branch, renamed
`advance_noiseless`; no new arithmetic or random operation is inserted there.
All old M34/M38/M39/M46 fixture assertions remain in place. Pinned Math
`1316ed334f4d5bda7be59aae07cd6e8ff31cfe13`, `src/Rand.cc`, returns the mean
without a draw when nonnegative sigma is at or below f64 epsilon. The existing
NoiseStream threshold is preserved. Neither zero nor this no-draw domain owns
a Box or initialized random stream. Two-mode out-of-plane zero validation must
remain in force throughout the no-draw threshold. A positive tiny sigma can
still have a positive wire covariance despite no random contribution.

## Stream ownership, transactions and reset

The implementation uses the existing loaded seed, default all-zero or explicit
through `load_world_seeded`, and stable public MODEL identity as the independent
ChaCha8 stream ID. It calls the focused checked `NoiseStream::white_gaussian`
method. No Gaussian bias model, initial bias-sign uniform draw, dynamic bias,
quantization, new RNG algorithm or global mutable stream is added.
Deterministic Rust replay and per-owner isolation are project guarantees;
matching C++'s process-global standard-library RNG bytes or reset policy is not.

The nonzero branch stages only mutable previous-pose/time, rolling means,
publication clock/revision and a stack clone of the owned RNG. It does not
clone Publication strings, metadata or heap boxes per acquisition. It commits
the candidate only after all kinematic, random-output, finite-Twist and revision
checks succeed. A successful throttled acquisition commits history and draws,
while retaining the previously published sample. On failure no candidate RNG,
mean, clock, pose or revision is committed; the dispatcher faults and removes
stale publication rather than reporting a partial/noisy success. The zero
branch retains its previous behavior without this new staging path.

Healthy `reset_loaded_world` reconstructs the initial Publisher clone, including
its initial stream and empty history, with the same public IDs. Internal Bevy
handles are reacquired. Removal retires the owner and its stream; healthy
reconstruction restores the configured initial one. A latched terminal fault
still refuses reset before its observer or mutation. Bare `Simulation::reset`
continues the existing odometry sample-clear contract; it does not gain a
promise to rewind histories or random state. Scene history/noise acquisition
is independent of Server demand, unlike demand-gated pressure sampling.

## Regression and proof obligations

The new public leaf reproduces all five formerly refused finite-noise inputs:
private parser sigma 1 with the original M34 model/default Two/raw plugin
contents, explicit Two sigma 0.01 in original M34, Three sigma 0.01 in original
M38, and sigma 0.01 in original M39 XYZ and M46 XYZ/RPY fixtures. Only the
existing world wrench consumer is added when needed to provide real force and
torque; each original model, mode, topic and offset context is preserved.
Each positive verifies load, actual body motion, unchanged physics/pose versus
a quiet peer, finite changed Twist, immutable metadata and healthy seeded reset.
Only those five old negative rows become negative-sigma refusals. All unrelated
old assertions, tolerances and native fixture bodies remain unchanged except
explicit default noise fields required by the additive metadata API.

New private controls check ordered six-channel contributions, no-draw startup
and threshold behavior, stream-owner independence and cloned replay, history
before throttle, pause/equal-time handling, and transactional failure rollback.
A revision failure deliberately occurs after six candidate draws to prove RNG
rollback rather than just a pre-draw gate. Stationary deterministic Rust tests
use 500 observations EACH in Two and Three and preserve all six original
GaussianNoiseTest bounds: sigma 1, absolute mean below 0.5, population variance
within 0.5 of 1. These are distribution controls, not native random-byte parity.
Public controls also cover seed divergence, same-seed body equivalence,
float-wire variance validation, zero/tiny compatibility, sigma-bit identity,
removal, reset and malformed/unsupported configuration.

Fresh original-system observations must separately verify native Two/Three
noise placement, both covariance matrices, the same six statistical bounds and
bounded sample counts. The original SDK/DSO must be reused with no native Physics
trajectory or rebuild. A zero observed value alone does not prove an unconsumed
RNG: retained Rand Source and zero/tiny scope are part of that proof. Canonical
C++ transport checks must compare actual current Scene samples and complete
messages, including shared Twist, unchanged TF, float covariance and shutdown.
No captured literal or native result is fabricated or inserted by this batch.

Independent whole Source review precedes one consolidated normal hook with
unchanged strict workspace/per-crate/actual-parent gates and independent local
DATA. Root alone reviews and releases heavy operations. Any local result is
provisional. Actual accepted predecessor, exact hosted success, independent
official evidence and normal Main/readbacks still precede publication and
acceptance. No package/browser, native RNG, physical-noise, performance, broad
compatibility or phase acceptance credit is supplied here.

## Original-component reference

Fresh original-system capture returned actual exit zero in 58.053 seconds;
independent retained native DATA also returned zero. The audited 16 modes and
18 configurations contain 1418 inputs, 1316 observed triples, 3948 complete
canonical packets and 102 suppressed inputs. Both stationary Two/Three cases
have 512 actual samples with the original six mean bounds (0 +/- 0.5) and
population variance bounds (1 +/- 0.5). Zero/tiny/boundary, mixed-offset motion,
throttle, pause/equal-time and fresh-instance cases retain their actual inputs,
headers, shared Twist, unchanged pose/TF and both 36-float covariance matrices.

[Generating provenance](../planning/reference/m47-odometry-noise.json) records
the exact generating program, compile/run commands, current provider hashes,
SDK/source identities and actual capture/audit bindings. The helper-only
capture tree over accepted Main is distinct from the provisional Rust source
parent. Reproduction requires the exact original DSO and current providers.
The package-upgrade baseline amendment in the delivery contract preserves all
349 identities and the historical manifest; 337 libraries were actually mapped.
No SDK or native Physics was rebuilt or executed by this reference workflow.
No random numeric fixture is invented: native stochastic observations establish
placement, distribution and covariance, while the separate current Rust live
peer checks packets against one actual seeded Scene sample. Rust RNG replay is
not bitwise equivalence with native global randomness. Rust/full normal checks
remain pending at this source checkpoint.

## First normal-hook failure

The first ordinary full normal hook failed required Clippy compilation before
any tests or coverage. A retained M46 replay initializer needed the new explicit
default-noise field. The complete constructor inventory closed with that single
compatibility correction; all old numeric/wire assertions remain unchanged.
Its original failed tree and output are preserved. The separate native proof
above remains valid and is not replayed. Corrected full normal qualification,
hosted/official evidence and acceptance are still required.
