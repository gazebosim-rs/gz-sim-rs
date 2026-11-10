<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M43 IMU reference frames and M44 installed preview

This is the next source-preparation wave under D30–D32. Each phase is bounded
at five working days including review and required checks. The checkout starts
at accepted Main `73e03b9f741689b1538898bde96bf975a49f068e`. M41/M42 passed
exact hosted CI, independent official DATA and Main promotion/readback. The
source-only overlap did not transfer qualification to this candidate. Complete
reviewed procedures and Root's serial release still precede each native, test,
build, packaging or browser operation. M40 remains the published installed
preview until this wave passes fresh distribution acceptance.

Independent source preparation may overlap that closeout. Review this complete
contract and its private interfaces before implementation. Then finish source,
meaningful regression source and guides, batch independent review findings, and
run one complete normal hook with unchanged strict coverage and exact hosted CI.
M43/M44 form one integrated candidate; neither is accepted until the feature,
distribution, installation, browser and publication requirements below pass.

## M43: named IMU orientation references

Admit the existing SDF IMU localization names ENU, NED and NWU, alongside the
accepted default CUSTOM behavior. Use only the existing centered-IMU domain.
Named references require no spherical-coordinate or heading metadata; enforce
that condition explicitly for named IMUs rather than assuming a general world
guard exists. Preserve legacy CUSTOM admission and all existing backend,
model, topology, gravity, rate, noise and sensor-position restrictions. Nonzero
custom RPY, custom parent frames and unknown localization names remain excluded.
Named-frame spherical/geodesy configuration and runtime reference mutation are
excluded; do not retroactively tighten legacy CUSTOM world metadata.

The core IMU input already has a reference quaternion. Reuse it; add no public
sensor API or new residual-math framework. A private Scene helper validates the
selected name and chooses the immutable reference. Root keeps the current
initial-sensor quaternion multiplication and evaluation order exact for CUSTOM;
its helper path must return those bits unchanged. Named references follow the
pinned original Sensors implementation and original Sim world-frame handoff.
Do not guess the ENU/NED/NWU table or invent a getter that the provider lacks.
InitialScene's existing clone/reset contract carries the loaded reference.

This changes the orientation reference only. Gyroscope and specific-force
measurements remain in sensor axes; physics, noise streams, covariance fields,
frame IDs, canonical schemas and the existing orientation-disabled behavior
remain unchanged. Keep existing error and acquisition ordering.

Before capture, agree a finite case table, direct input domain and exact recipe
with the independent reviewer. Use the already-installed pinned IMU component;
verify its actual source/provider identity and record actual mapped provenance.
Record the direct WORLD sensor pose, WORLD acceleration and angular velocity,
and WORLD gravity. The pinned native Math reverse rotation converts the two
motion vectors into sensor axes exactly once before the SDK setters. Retain
both the WORLD inputs and actual setter values; the scalar `ImuInput` replay
uses those sensor-axis values directly. Reset both setters before each native
update because the component mutates them. This component capture does not
qualify native Physics or Scene's velocity-history acceleration calculation.
No SDK rebuild, original Sim system execution or native Physics campaign is
selected. Capture observed component readings and canonical messages, then audit
retained bytes and generated literals independently before Rust replay. Keep the
established native IMU comparison and strict <1e-9 bound without widening it; require
exact canonical encoding of the actual Scene values. No RNG-stream parity claim.

Acceptance covers all three named references with nonzero initial body and
sensor rotations, actual angular motion, default CUSTOM equivalence, quaternion
sign cases, orientation-disabled output, unchanged gyro/acceleration/noise and
covariances, demand/pause/rate behavior, reset with stable public IDs and fresh
internal handles, removal and owned peer shutdown. Convert the exact former NED
refusal into a strong load/motion/reset positive; preserve neighboring refusals.
Use `examples/m43_imu_reference_frames.sdf` and focused Scene/Server controls.

## M44: eighteen examples and the installed joint-force helper

Refresh the native-free Ubuntu 24.04 x86_64 preview from the qualified integrated
source. Preserve the existing fifteen-example package prefix and ordinary
relative OBJ asset, then add M41 wind, M42 joint-force feedback and the M43 IMU
reference example, for eighteen finite installed examples. Preserve the default
M8 robot and existing launcher behavior.

Build and ship `bin/gz-joint-command` from the maintained joint_command example,
alongside the simulator and existing drive helper. Include its actual source,
Cargo artifact, feature, notice and ELF dependency closure; do not infer it from
the drive helper. Keep the helper's position/velocity behavior and bounded force
mode, including explicit zero effort that coasts rather than brakes. Extend the
maintained assembler and validators directly, with meaningful missing/foreign
artifact, relocation, required-file and source-binding controls. Add no new
packaging or runtime framework.

Reuse the six actual accepted c404 viewer files, project/vendor/font/proto notices
and unchanged 14,000,000-byte gzip budget per flavor. Rebuild no renderer. Collect
actual metadata and notices without changing raw license declarations, selecting
OR branches or dropping identities. Keep all existing bounds and source, mode,
manifest, archive and relocation requirements; review any necessary bounded
extension before execution. Source readiness is not installation acceptance.

Require fresh serial source-bound release inputs, actual metadata/ELF/notices,
assembly and relocation, and independent retained package DATA. Then perform one
actual supported Ubuntu installation without a compiler or native SDK, as the
existing nonroot user, with all eighteen finite examples and all three ELF
closures. Exercise the installed joint helper against the packaged hinge through
real canonical force/feedback and completion fences. Keep finite load checks
separate from acquisition and trajectory evidence.

Use the maintained live browser for the packaged default robot: actual Play,
forward and turn commands, stable Pause, Reset and owned launcher shutdown.
Preserve existing previews and listeners. Surface only observed waits; the
simulator grandchild exit remains unknown if the launcher does not expose it.
Retain original failures and original image bytes. No cold-start, performance,
legal or universal-platform claim follows from this bounded installation.

After full normal/local DATA, exact hosted success, official DATA and the fresh
distribution proofs, publish through normal nonforced Main/tag/release approval.
Verify actual immutable assets, notes and Latest readbacks, and preserve the
prior M40 release. Update canonical acceptance and the web tracker. Continue to
the next useful bounded retained increment after acceptance.

## Ownership and execution

Light owns the private Scene IMU-reference helper, focused Scene regression
source, example and feature guide. Messages owns component-reference and
Server/canonical peer source, plus independently audited literal extraction.
Root owns shared Scene registration/admission seams, packaging/helper input
collection, qualification registration, guides, planning and integration. SDF
independently reviews interfaces and complete source batches. Agree exact paths
before editing; do not change another owner's leaves.

Root alone releases and watches serial local heavy operations. Remote CI and
disjoint source work may overlap. Preserve frozen trees, Fuel, prior accepted
behavior and all resource floors. M41/M42's exact hosted success, collection,
independent official DATA and Main acceptance are complete. Their evidence does
not qualify M43/M44. Each new operation still requires its complete reviewed
source/procedure and Root's serial release against that actual accepted parent.
