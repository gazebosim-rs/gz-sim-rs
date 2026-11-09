<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M28/M29: AirSpeed noise and the next installed preview

These bounded phases continue D30–D32. Each includes review and mandatory
verification within a target of at most five working days. M1–M27 are accepted;
M25 remains the published installed preview. Parent Main `0f525592` passed the
complete normal hook, strict coverage, focused native comparisons, independent
native/local/official audits and exact hosted run `42570074529` attempt 1. Normal
Main promotion and readback completed on 2026-11-09 UTC; see
[M26/M27 acceptance](M26_M27_ACCEPTANCE.md). Independent child Source preparation
overlapped that frozen parent closeout without inheriting acceptance. Parent
corrections would require rebasing and reviewing affected work.

## M28: loaded AirSpeed pressure noise

Extend the existing single top-level direct-link Rapier AirSpeed sensor with
Gaussian and Gaussian-quantized pressure noise using the existing scalar noise
model and seed/identity policy. Preserve zero wind, default atmosphere, sensor
point/frame velocity, all other admission bounds and the ideal path. Do not add
a parser, dependency, noise framework, transport service or sensor family.

The pinned native AirSpeedSensor applies noise to its pre-publication `f32`
pressure, assigns the noisy result back to `f32`, then multiplies by `100.0f`.
Noise parameters therefore use the native **pre-100 pressure units**: a mean of
0.1 changes the published number by about 10 Pa, subject to float rounding.
Temperature is unchanged, and negative pressure remains valid. Do not add noise
to the published result or reconstruct the rounded precursor by division.
Extract a small shared numeric seam; preserve the ideal wrapper's exact outputs.

Reuse GaussianNoise and NoiseStream with the actual spawned public identity.
Preserve strict default None, finite metadata and nonnegative deviations and
precision. Both Gaussian kinds use the native precision threshold and rounding;
default elapsed time is zero, so dynamic bias does not evolve. Gaussian with all
zero parameters remains enabled state. A nonfinite noisy result, overflow on
conversion to float, or overflow during final scaling is a typed noise fault
with no stale sample or wire packet. Preserve existing ideal-error causes.

Exit evidence includes meaningful deterministic precursor/unit/quantization
boundaries, valid negative output, temperature preservation, fixed-bias and
disabled-dynamic behavior, seed/identity isolation, scheduler/demand draw
position, core/loaded reset, absence/removal and actual canonical wire/fault
controls. A focused native SensorFactory helper must prove deterministic cases,
precision boundaries, zero-elapsed bias and predefined 100-sample moment bounds.
Do not claim exact cross-engine RNG sequences, trajectories or performance.
Complete implementation, regression source, guide and independent Source review
before one consolidated affected pass. Full normal hook, strict per-crate and
workspace line/region coverage, actual accepted-parent checks, exact hosted CI,
independent audits and qualified Main remain mandatory before acceptance.

## M29: updated installed Ubuntu preview through M28

Prepare the packaging source independently while M28 is authored. Actual
release inputs and installation proof require the qualified integrated source.
Retain the default robot, launcher behavior, all four existing noise examples,
old guides and the six accepted c404 viewer files with both unchanged 14 MB
gzip budgets. Add the ideal AirSpeed, PosePublisher and M28 noise examples and
their dedicated installed guides. Keep unsupported capabilities explicit.

Build fresh source-bound native-free CLI/helper inputs, collect actual metadata,
ELF closure and notices, assemble ordinary files, verify relocation and audit
the complete payload. Prove a clean supported Ubuntu 24.04 x86_64 installation
without a compiler or development SDK, verification as UID 65534, and real finite
load/step execution of all seven example scenes. These finite example executions
do not establish canonical message acquisition or noise statistics.

Run the newly packaged default robot through actual live browser Play, forward,
turn, Pause and Reset, then prove owned launcher/process/listener shutdown.
Retain actual exits, partial failure evidence and simulator-grandchild exit
limitations honestly. Reuse maintained procedures, not old binaries, archives,
installed executions or browser acceptance. Require independent package,
installation and browser evidence, normal tag/assets/body/Latest readbacks and
all mandatory source/hosted gates before acceptance. No renderer rebuild,
scene-picker, telemetry facade, runtime framework, new dependencies, optional
native backend or RGB/depth-worker distribution is included.

## File leases and integration

Light owns M28's `gz-sensors` AirSpeed model/tests; Scene AirSpeed leaf, private
and new noise tests, fixture and native helper; three existing private Server
AirSpeed test files; the new noise example and `M28_AIR_SPEED_NOISE.md` guide.
The existing shared noise implementation, Cargo/proto/transport and production
Server files are outside that lease. Root owns any necessary shared glue.

Messages owns the M29 package assembler, meaningful package regression source,
new installed capability/install/preview guides and installed-guide mappings.
Messages consumes Light's example and guide without editing them. The launcher,
viewer, default robot and old fixtures remain unchanged. Exact packaging leases
are `scripts/package-preview.py`, `tests/test_preview_package.py`, new
`docs/M29_INSTALL.md`, `M29_CAPABILITIES.md` and `M29_PREVIEW.md`; the finite
copied-binary qualifier lease is `scripts/qualify-rapier-runtime.py` and existing
`tests/test_rapier_runtime_coverage.py` / `test_rapier_runtime_retention.py`.
Do not change the Rust target controls or native-free admission checker.

Preserve the four old exact stdout matchers. The new M27 finite control must
check its full header and two sorted link rows with three finite coordinates,
no missing/extra data, base `(1,2,3)` and tip
`(1+0.8*cos(0.6), 2+0.8*sin(0.6), 3)`. Its zero-gravity, zero-effort initial
configuration stays at those composed world positions. Only this new case uses
absolute tolerance `1e-12`, relative tolerance zero, consistent with the existing
M27 native component-equality bound and double pose composition. Test structural,
nonfinite and beyond-bound corruptions; do not relax old cases. The ideal/noisy
AirSpeed examples keep the strict `(0,0,3)` body geometry. Execute all seven
commands once in the existing installed guest stage, with the existing deadlines.

Root owns the integration worktree, canonical planning, shared exports/glue,
review publication and tracker. Authors use separate worktrees over the exact
parent and run no per-edit checks. SDF independently reviews contracts, source,
regressions and integration. Batch findings before one consolidated pass. Root
alone serializes local heavy work; neither child qualification nor publication
inherits parent acceptance. Preserve Fuel, frozen trees, accepted demos, all
original failures, resource floors and native SDK bytes. No new eviction.
