<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M39/M40: translation offsets and a fifteen-example preview

This wave uses accepted M38 Main
`00382fbc342ccc79a5e3c126f8bd5a41064c7a80`; see [M38 acceptance](M38_ACCEPTANCE.md).
Implementation, meaningful regression Source, guides and the new actual native
fixture are integrated after independent leaf review. The scoped native capture
passed in 106.975 seconds; independent native and literal DATA confirm 34 modes,
1,152 inputs, 986 observations and 2,958 packets. Those component proofs do not
qualify the new Rust or distribution. Final whole integration review precedes
one formatting/staging pass and the full normal qualification below.

Each phase is bounded to at most five working days including review and all
mandatory checks. Disjoint feature, canonical/native and packaging Source work
may proceed in parallel under the leases below. Root runs heavy work serially.
Every individual feature and fresh distribution acceptance criterion remains
required before accepting this atomic pair. Preserve the qualified parent and
all old fixtures, tests, package/browser evidence and real failures. No result
transfers from the parent or between phases.

## M39: noiseless odometry translation offsets

Admit finite three-component `xyz_offset` and `xyz_global_offset` on the existing
flat top-level Rapier ground-truth odometry model, for both dimensions 2 and 3.
The native result is `globalOffset.Inverse() * worldPose * localOffset`.
Local translation follows the model's orientation; global translation changes
the reference origin. Preserve the native scalar operation order, startup,
signed-zero behavior, cadence and all velocity histories. Do not substitute
instantaneous backend Velocity for the filtered pose-derived Twist.

Keep every rotational offset and Gaussian-noise restriction unchanged. No new
backend, topology, message field, transport, solver or generic transform
framework belongs to M39. Missing/zero translation configuration preserves the
accepted paths and every old numerical and canonical fixture. In particular,
legacy all-zero vectors, including previously admitted signed-zero spellings,
keep their old behavior. The owners must agree an exact finite, bit-aware
configuration identity and its zero-vector parsing rule before implementation;
ordinary float equality must not silently reuse changed nonzero configuration.
The agreed fixed-role type is `GroundTruthOdometryOffsets`: private local and
global `[u64; 3]` bit arrays, Copy/Eq/Default. Its public constructor is
`from_translations(local: [f64; 3], global: [f64; 3]) -> Result<Self, GroundTruthOdometryError>`;
nonfinite arguments return the singular `GroundTruthOdometryError::Offset`.
The value getters are `local_translation()` and `global_translation()`, each
returning `[f64; 3]`. Publication owns one public `offsets` field of this type.
Each independently wholly-zero vector becomes the legacy +0 default; every bit
in any nonzero vector is preserved. Existing XML parse errors keep their current
mapping. This is owned publication identity, not a generic numeric framework.

Offsets are immutable prepared configuration. Live mutation must fault and
remove stale samples under the existing ownership contract. Healthy loaded
reset reconstructs offsets, histories and public IDs. A latched fault remains
terminal and reset must refuse before observation or mutation. Nonfinite or
malformed configuration is rejected atomically; finite composition overflow
must produce an existing typed fault, never a valid packet or zero clamp.

Inventory all existing translation-offset refusals before editing. Each exact
newly admitted input becomes a stronger positive through real loading, motion
and healthy reset. Preserve all unrelated inputs, errors, assertions and
bounds; record the independent disposition before replacing any negative row.
The accepted M34 301-input/221-observation/663-packet and M38
510-input/429-observation/1,287-packet fixtures and controls remain unchanged.

Trace the pinned original OdometryPublisher Source and Math Pose3 operations.
Prepare finite native cases for local/global/combined offsets, every axis,
tilted and rotating models, both dimensional modes, zero/signed-zero cases,
startup/filter/throttle boundaries and reconstruction. Derive actual counts
from the complete helper case plan. Reuse the unchanged original system DSO
with exact generating Source/commands/providers; do not rebuild the SDK or
replay old native modes. Record requested and effective vectors separately in each new native case.
Apply the agreed legacy all-zero normalization before native configuration;
label that scope and do not claim parity with unnormalized native negative-zero
SDF. Preserve mixed signed zeros in nonzero vectors. The literal schema binds
dimensions, cadence and effective offset bits per instance, plus MODEL-local
input under the identity parent. Convert MODEL to WORLD exactly once before
applying global/local offsets. New actual observations need independent DATA
and literal-fixture review before Rust replay insertion. These are component
pose-history proofs, not native Physics trajectory or performance claims.

Meaningful Scene controls use actual Rapier force/torque, independently check
model-relative point geometry, and distinguish filtered Twist from physical
Velocity. The real C++ peer consumes all three canonical streams, compares
complete messages keyed by acquisition stamp, and verifies demand, pause,
healthy reset, collisions, removal and owned shutdown with the existing bounded
queue/fence contract. No pose or body-velocity teleportation is allowed.

## M40: refreshed fifteen-example Ubuntu preview

Refresh the maintained native-free Ubuntu 24.04 x86_64 distribution through the
qualified combined candidate. Preserve the thirteen existing example bytes and
ordinary relative OBJ closure; add unchanged M38 and the reviewed M39 example,
with their guides, for fifteen real finite installed commands. The default
robot, launcher, protocol and six accepted c404 viewer files stay unchanged.
Both 14,000,000-byte viewer gzip budgets remain required; no renderer rebuild
or optional camera-worker/native-backend distribution is selected.

Keep source-bound fresh release CLI/helper builds, exact Cargo metadata and
complete notices, actual ELF/system-library closure, manifest/archive/relocation
verification and independent package DATA. Install on one clean supported
Ubuntu environment as a nonroot user without a compiler or SDK; execute all
fifteen copied examples. Those finite commands prove load/step behavior, not
canonical acquisition or native equivalence. Feature proofs remain separate.

Require one fresh maintained live-browser default-robot demonstration with
Play, increasing clock, real forward and turn commands, stable Pause, Reset
and clean owned launcher/process/listener shutdown. Preserve prior previews
and every original failure. Reuse only supported free loopback ports; never
stop an unknown or protected service. Report simulator-grandchild status as
unknown if the unchanged launcher does not expose it. Publish new Main/tag,
exact assets/checksums/notes and Latest only after independent required DATA
and actual publication readbacks. Earlier releases remain intact.

## Parallel leases and one integration

Root integrates `/home/nkoenig/work/gazebo/rust/gz-sim-m39-m40-translation-preview`.
Light's separate lane is `gz-sim-m39-scene`; Messages' is `gz-sim-m39-server`.
All start at the same locally qualified M38 commit above.

- Light owns `crates/gz-sim-scene/src/odometry_publisher.rs`,
  `crates/gz-sim-scene/tests/odometry_translation_offsets.rs`,
  `examples/m39_odometry_translation_offsets.sdf` and
  `docs/M39_ODOMETRY_TRANSLATION_OFFSETS.md`. Its exact old negative-row
  dispositions are in that odometry leaf, `crates/gz-sim-scene/tests/odometry_publisher.rs`
  and `crates/gz-sim-scene/tests/odometry_3d.rs`; preserve every other case.
- Messages owns `crates/gz-sim-server/src/odometry_publisher.rs`,
  `crates/gz-sim-server/tests/odometry_translation_offsets.rs` and
  `crates/gz-sim-server/tests/native_odometry_translation_offsets.cc` for its
  distinct transport-only and explicit native-reference roles. It owns the
  new fixture procedure and eventual audited
  `crates/gz-sim-scene/tests/fixtures/m39_native_odometry_translation_offsets.rs`
  plus its `.provenance.json` sibling; no placeholder values are inserted.
- Root owns `crates/gz-sim-scene/src/lib.rs`, shared records and peer registration
  in `scripts/native-peer.py`, `scripts/zenoh_peer.py`, `scripts/verify.py`,
  `tests/test_selected_native_gate.py` and `tests/test_zenoh_peer.py`. Its M40 lane
  is `scripts/package-preview.py`, `scripts/qualify-rapier-runtime.py`,
  `tests/test_preview_package.py`, `tests/test_rapier_runtime.py`,
  `tests/test_rapier_runtime_coverage.py`, `tests/test_rapier_runtime_retention.py`,
  new `docs/M40_INSTALL.md`, `docs/M40_CAPABILITIES.md`, `docs/M40_PREVIEW.md`,
  README and current planning/handoff/status/AGENTS records. The old M37 guides
  and launcher remain unchanged.
- SDF independently reviews the full contract, exact test dispositions, complete
  Source and evidence. Shared API details require explicit agreement before edits.

Finish both bounded implementations, meaningful regression Source and guides;
batch independent review findings before one integrated full normal hook.
Preserve strict workspace, per-crate and actual-parent line AND region coverage,
exact hosted CI and independent local/official DATA. Root alone releases heavy
jobs. M39 and M40 become accepted only after every individual feature and fresh
distribution criterion passes and normal qualified Main promotion completes.
If a lane exceeds its budget, split optional scope before freeze; do not weaken
criteria, silently drop an example or label Source readiness as acceptance.

After acceptance, continue the next useful bounded retained increment. No Fuel
change, new dependency/framework, rotational-offset/noise expansion, broad
compatibility inventory or performance claim belongs to this wave.
