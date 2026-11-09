<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M34/M35: parallel bounded system source wave

M1–M33 are accepted at qualified Main `17dd62f2`. M33 passed its full normal
hook, exact hosted CI42667053571 attempt1 and independent local/official DATA,
then normal nonforced Main promotion and exact readback. M32 remains installed
Latest. M34 ground-truth odometry and M35 canonical wrench inputs are active in
parallel source lanes under [recorded scope and leases](M34_M35_DELIVERY.md).
Both implementations and their source reviews are complete. The original native
systems supplied independently audited reference records: 221 odometry packet
triples and 32 wrench command records. A separately audited literal fixture now
backs the new Rust numeric and serialization regressions. The full normal hook,
strict coverage, exact hosted CI and independent local/official audits remain
required before either phase is accepted. See [M33 acceptance](M33_ACCEPTANCE.md)
and the [reference checkpoint](M34_M35_DELIVERY.md#native-reference-checkpoint).

The source worktree is bound to the actually accepted M33 parent. Root released
implementation only after both detailed contracts and typed seams were reviewed.

Each phase is budgeted at most five working days including review and mandatory
checks. Source planning and disjoint authoring may overlap hosted qualification.
Root serializes all local heavy work. Record and independently review exact
native arithmetic, lifecycle and admission contracts before implementation;
return unresolved interface or dependency requirements to Root before edits.
No checks, native helpers, compilers, imports, metadata or browser runs are
released by this scope document.

## M34: noiseless 2D ground-truth OdometryPublisher

Add one explicitly configured top-level flat Rapier model publisher using its
actual model WorldPose, separate from existing DiffDrive encoder integration.
Support dimensions 2, zero local/global offsets and zero noise, with finite
validated frequency and bounded canonical frame/topic configuration. Reject
3D, nondefault offsets/noise and unsupported topology/backend inputs explicitly.
Reuse existing world poses, schedules, glam and canonical Odometry,
OdometryWithCovariance and Pose_V definitions with upstream Zenoh.

The native contract must fix the first unpaused call initialization, including
zero time/duration, and nonzero initial-pose behavior; later eligible finite
differences, ten-sample rolling means
before publication throttling, body-frame cosf/sinf arithmetic and yaw wrapping.
Use the original native implementation as a bounded computation/publication
oracle, with actual source/header/tool/provider pins and raw observations.
Document loaded reset as a deterministic Rust policy; native rewind is TODO and
must not be claimed equivalent. Require actual pose-motion versus wheel-encoder
separation, filter filling/rate/frame/covariance checks, zero/paused/non-due
behavior, invalid metadata and atomic endpoint/reset refusal, demand,
removal/reconstruction/weak lifetime, and all three real canonical C++ streams.

## M35: canonical ApplyLinkWrench inputs

Add one configured world input system for existing explicit dynamic Rapier link
targets in top-level, single-link, unjointed models, with exactly zero inertial
center translation, finite world-axis force/torque and zero force_offset. Reuse
queue_link_world_wrench and PhysicsBackend::add_world_wrench. Support bounded
one-shot commands, bounded additive persistent commands and clear. Reject TPE,
model/canonical-link indirection, nested/jointed/multilink bodies, noncentered
inertial centers, nonzero offsets and future targets in this
slice; do not add physics/contact behavior or silently accept unsupported data.
This keeps the native link-origin force application equivalent to the existing
center-of-mass queue mechanics without introducing new frame math. Preserve the
general queue API; Root owns a minimal nonmutating validation/atomic batch seam
if required. Do not validate by enqueuing zeros or duplicate private bindings.

Fix the native clear-before-paused-return ordering, queued one-shot lifetime
and repeated-persistent-add semantics in the reviewed contract before editing.
Define explicit removal and loaded-reset clearing policies rather than inheriting
native future-insertion behavior. Require actual one-step/repeated delta velocity
and torque, pause/clear ordering, additive sums/finite overflow/atomic refusal,
identity/removal/reset/shutdown and real canonical EntityWrench/Entity C++ input.

## Native reference boundary

Both original sources are at native gz-sim commit
71c60db31fddd5c98d2aa689e9a2cccb3548a0a3, under
src/systems/odometry_publisher/OdometryPublisher.{cc,hh} and
src/systems/apply_link_wrench/ApplyLinkWrench.{cc,hh}. Targeted source is clean.
The installed SDK did not provide either target DSO. The reviewed reference
procedure built each original system as a packet-only DSO against the retained
Sim11/common8/math10/messages13/transport16 providers. This did not run native
Physics or establish a trajectory or performance claim. Native sources, SDK,
old captures and paused owners remain unchanged.

## Native reference checkpoint

The first reference run completed both original DSO builds and all nine odometry
modes, then failed compiling the wrench helper because `sdf/SDF.hh` was absent.
Those successful builds and odometry observations were retained without replay.
A separate isolated source tree corrected only that include to the installed
`sdf/SDFImpl.hh`. Its affected continuation compiled only the helper and passed
all eight wrench modes, producing 32 six-component command records.

Odometry retained 299 unpaused and two paused inputs, 221 observed triples and
663 canonical packets. The retained odometry reader completed its body checks
but failed its final self-hash because its mutable resource snapshot changed.
A separate retained-only finalization audit passed; neither the original native
run nor the original reader is relabeled successful. Independent wrench DATA
then passed for the affected continuation. The literal fixture was independently
compared with all 301 inputs, 221 output bit-tuples and 663 captured byte arrays;
15 unobserved delayed rows remain distinct from 65 suppressed rows.

The first full normal hook stopped after 84.076 seconds during compilation:
all four E0425 diagnostics named the same missing `gz_sim_core::entity_by_id`
import in the wrench module. No Rust tests or coverage ran. The failed source,
log and partial packet remain retained. The complete correction adds that
existing function import; it does not change runtime behavior, native helpers,
fixtures, assertions or numerical tolerances. Independent source review precedes
the required full-hook retry.

That retry stopped after 138.079 seconds at Clippy's large-enum-variant gate:
the canonical wrench command occupied at least 400 bytes versus 88 for clear.
The correction boxes the wrench message after its existing bounded decode,
keeping channel admission, queue limits, ordering and consumers unchanged.
No lint exception was added. Tests and coverage were not reached in either run;
both failed source trees and complete logs remain retained. The complete
correction receives independent source review before another required full hook.

The next run reached Server test compilation, then stopped after 170.089
seconds at three older fixture helpers whose exhaustive family matches omitted
the two new variants. Every caller selects one of the fixture's seven retained
families. Explicit new-variant panic arms keep that bounded contract and fail
accidental out-of-fixture use; all old branches and assertions remain intact.
M34/M35 endpoint interactions keep their separate fixtures. No runtime tests or
coverage had run. The original failure is retained before the reviewed retry.

The fourth run completed both Clippy configurations. Root then interrupted it
after 304.107 seconds when independent source review proved that Rust skipped
the native zero-offset pose composition. The immutable sign-mode fixture has
negative-zero input quaternion components and positive-zero native output bits.
That is a source finding, not an observed Rust numerical assertion failure.
The correction evaluates the native scalar pose operations before history and
publication. Fixture model-component inputs receive one explicit identity-parent
conversion to world poses; production world poses receive only the two offset
compositions. A separate direct-world signed-zero regression guards that domain
boundary. The fixture, native observations and exact assertions stay unchanged.
No native recapture is needed; full corrected Rust qualification remains required.

The fifth normal run completed the exact direct-world signed-zero, 301-input /
221-output native-bit replay and all 663 canonical-byte controls. Its headless
no-fail-fast run then reported ten failures across four new test targets, ending
after 1,464.264 seconds. Renderer qualification and final strict coverage were
not reached. The complete failure packet remains retained.

The correction batch fixes scalar XML entity-reference handling and the existing
template lookup's cross-kind name ambiguity: a collision and visual may share a
scoped name, so the visual lookup must also select Visual kind. A public-entry
regression checks their distinct IDs and the chosen source visual. New IO test
fixtures now use qualified topics and a supported nonempty partition. The COM
control keeps Rapier's existing loader refusal, with a separate metadata-only
API refusal/no-mutation test that never steps a tampered world. The live C++
peer's readiness conditional receives braces and statement layout only; its
original capture-source projection is explicit, without recapturing native data.
The canonical examples, all lifecycle assertions, numeric tolerances, input
bounds and exact native fixture expectations remain unchanged. Full qualification
of the complete reviewed correction is still required.

The sixth normal run completed every headless Rust test with no failures,
including the corrected controls, and all 367 Python tests. It stopped after
1,522.283 seconds because server region coverage was 13,078/13,768 (94.9884%),
below the unchanged strict greater-than-95% requirement. Renderer and changed-line
qualification were not reached. Independent read-only inspection also found
1,381/1,456 changed executable server lines covered; that is a correction target,
not an executed changed-line gate result. Preserve the failed source and packet.
The correction adds meaningful new-system invalid-sample, identity and endpoint
lifetime controls before independent review and one consolidated full retry.

The seventh full hook stopped after 178.092 seconds at two lints in the new
wrench controls: an unused local import and default-value boxing syntax. Both
are corrected together without changing test behavior. No runtime tests or
coverage ran in that attempt; its original source and packet remain retained.

The fixture and private Rust regressions require exact numeric bits and exact
canonical bytes, with no new tolerance. Native unpaused zero-duration wrench consumption
remains explicitly different from the selected Rust advancing-only policy.
No package refresh, browser replay or new native capture is required here.

## Exclusive leases

Light owns only new files:
- crates/gz-sim-scene/src/odometry_publisher.rs
- crates/gz-sim-scene/tests/odometry_publisher.rs
- crates/gz-sim-server/src/odometry_publisher.rs
- crates/gz-sim-server/src/odometry_publisher_io.rs
- crates/gz-sim-server/tests/odometry_publisher.rs
- crates/gz-sim-server/tests/native_odometry_publisher.cc
- examples/m34_odometry_publisher.sdf
- docs/M34_ODOMETRY_PUBLISHER.md

Messages owns only new files:
- crates/gz-sim-scene/src/apply_link_wrench.rs
- crates/gz-sim-scene/tests/apply_link_wrench.rs
- crates/gz-sim-server/src/apply_link_wrench.rs
- crates/gz-sim-server/src/apply_link_wrench_io.rs
- crates/gz-sim-server/tests/apply_link_wrench.rs
- crates/gz-sim-server/tests/native_apply_link_wrench.cc
- examples/m35_apply_link_wrench.sdf
- docs/M35_APPLY_LINK_WRENCH.md

Root owns every shared file: module attachments/metadata/admission, schedules,
activation/reset/removal and producer guards, Scene/Server lib and world systems,
Server options/IO/counters, maintained peer registry AND CLI/profile guards,
verification/retention controls, shared docs/planning and all procedure artifacts.
Propose exact interfaces and needed shared integration before leaf code. Do not
edit another owner's files. SDF reviews both contracts and full source batches;
authors may cross-review disjoint leaves. First author each guide's complete
contract and proposed typed interfaces, then obtain Root/reviewer release for
implementation. The separate Root source-implementation release records the actual authoring release; this document never releases checks or native execution.

## Qualification and exclusions

Finish both bounded implementations, meaningful regression source, examples,
guides and complete independent source review; batch findings and corrections.
After accepted-parent binding and reviewed reference procedures, Root releases
one bounded native-reference batch and one consolidated complete normal hook,
strict per-crate/workspace/actual-parent coverage, exact hosted CI and independent
local/official DATA with durable originals. Each phase requires its own actual
new behavior proof before qualified Main acceptance. Reuse unchanged evidence
only where existing checks allow it; never replay old accepted manual work.

Keep M32 installed Latest unchanged. Exclude packages/browser refresh, new noise
models/backends/dependencies/schema/transport/runtime/packaging frameworks,
dynamic Rust plugins, body teleportation, contact/reaction sensing, sensor widgets,
full compatibility inventories and performance/legal/cold-start claims. If a
required dependency is larger than this scope, return it to Root before edits;
split optional scope without weakening mandatory acceptance. Preserve Fuel,
frozen qualification trees, paused work and every original failure.
