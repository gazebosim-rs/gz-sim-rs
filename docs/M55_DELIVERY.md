<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M55: independent loaded PosePublishers

## Current provisional implementation and reference evidence

The reviewed base implementation is integrated over locally qualified, unaccepted
M54 d140e51e. All M54 repairs and the older logical-camera fixture corrections are
preserved. The new original-object helper compiled and linked successfully, then
completed 30 actual child stages. Independent native DATA, immutable export and
independent literal DATA each passed with 10 configurations, 40 explicit LOCAL
inputs/calls, 65 native unordered vectors and 95 nested Pose/Header records.
See [the exact reference provenance](../planning/reference/m55-multiple-pose-publishers.json).
The prior DSO-input diagnostic remains failed 1; its provider and strict guard
were unchanged. No SDK or original provider was rebuilt.

The Scene and Server observed controls are complete and independently reviewed:
17 headless and 10 overlapping renderer Source declarations, not passes. One
consolidated formatter passed. The first full normal failed in Clippy test
compilation after 190.704 seconds, with E0308/E0277 at one new Server assertion
comparing actual u32 and fixture u64 IDs. The complete tree and raw failure are
retained. The lossless actual-ID widening preserves the same equality and every
other assertion; runtime, schema, immutable fixture and all four reference
proofs remain unchanged. Review the complete correction/failure-retaining retry
procedure before one affected full normal rerun. The captured 29-file input
checkout remains immutable. No M55 Rust test or coverage pass is claimed. All existing acceptance criteria below still apply,
including independent results DATA, accepted predecessors and hosted/official
qualification. M1–M42/175 slices and M40 Latest remain unchanged.

## Earlier Source-preparation checkpoint and continuing scope

Selected for Source preparation under the reviewed proposal and exact leases in `artifacts/m55-delivery/selection.json` and `source-authorship-release.json`. Only implementation, regression-test Source and documentation are released. Formatting, tests, lint, Cargo metadata, provider inspection and native execution remain held. This continues the retained M27 flat-model/M31 split-static PosePublisher work and SENSOR-05/SYS-EXTENSION scheduling and ownership requirements, under D30's runnable increments, D31's disjoint ownership and D32's continuation policy. Budget: at most five working days including review, corrections and mandatory qualification; reduce optional scope rather than weaken an exit criterion.

Admit zero through eight configured top-level flat Rapier models, one existing built-in PosePublisher per model. Preserve absent/single behavior, all defaults, strict XML, frame selection/classification, nonpositive/positive-rate arithmetic, checked duration conversion, parent-relative local transforms, lexical Rust ordering, zero canonical wire IDs, absent outer header and each Pose's exact ordered frame headers. Keep M31 split-static mode, independent dynamic/static clocks and classification unchanged. Preserve the existing per-owner maximum of 128 selected records (1,024 across eight owners) and at most two declared channels per owner (sixteen leases). Both endpoints remain reserved even if a group is empty; no empty vector is published. This resource cap is not an upstream limit.

There is one Scene singleton admission seam, Plan::validate's existing Publisher check. Installation, sampling, reset and last-owner retirement already iterate owners. Server streams and prepare_reset each have one singleton guard. Both paths must validate the complete candidate set, owner IDs/kinds/full names and immutable initial metadata, all resolved dynamic/static endpoint aliases and occupied keys, and the complete prepared set before any output lookup or declaration. Preserve the current Prepared tuple prefix `(owner_id, metadata, Arc<output>)`; append immutable owner name if required. Keep metadata/removal/reset fences and static-first enqueue behavior. Existing initial_pose_publications already yields borrowed owner ID plus publication; Root should add the missing immutable kind/full-name identity seam additively, preserving the old query. Do not split a hypothetical WORLD prefix incorrectly; actual loaded WORLD names retain the parser's admitted domain, including its reserved-name refusal.

No new sensor, renderer/backend, nested model, visual/collision/sensor record, individual-Pose, fixed-joint Physics, dynamic plugin, transform graph, noise, latching, subscriber-interest or overload framework is added. Acquisition remains independent of listeners. Preserve positive-time period anchoring and eligibility of equal-time unpaused/backward updates, paused suppression, revisions, per-channel last clocks, direct-core reset's retained cadence versus loaded-reset reconstruction, and global terminal fault semantics. A bad due owner clears its stale vectors and fails the step; do not promise cross-owner atomicity or deterministic query order beyond existing behavior.

Regressions must establish zero/single/two/eight/ninth and late-invalid admission, all 128 selected records, independent dynamic/static rates and empty groups, same-input owner separation, physical force/torque and supported joint-motor motion at actual acquisition times, exact held-frame equality, no-interest acquisition, pause/equal-time/rewind, reset with new handles/original IDs, owner and selected-link retirement, last-owner resource absence, metadata drift and terminal reset nonmutation. Strengthen the precise old singleton assertion in `pose_plan_domain_is_flat_rapier_and_one_publisher_static_models_are_not_static_streams` into a meaningful distinct-owner motion/reset positive under that same existing identity, separately required BOTH outside new-ID/pool counts. Preserve every neighboring backend/nested/default/invalid-transform assertion and all duplicate-name/plugin/endpoint negatives. Add a distinct dynamic paired-channel public BOTH control. No test result is inferred from Source declarations.

Native prerequisite: retained M31 result records actual zero and 21 stages, original PosePublisher object SHA e5faf2ca… and DSO SHA1a4e0d79…; M27 separately records the same identities. One reviewed metadata diagnostic observed the retained object/DSO identities and aliases unchanged but returned 1: the DSO has an empty RUNPATH entry and is not admitted. No static dependency closure or loader proof was established. Preserve that failed admission and its strict policy. The separately reviewed new direct-object design below is Source preparation for a different exact helper/input recipe; complete all input/recipe/helper reviews before any compilation or capture, with no provider build, scan or fallback. Proposed fresh pair/eight original-System controlled local-Pose inputs should prove independent owners/configurations/channels and actual callbacks, without replaying old six/seven-mode campaigns or claiming Physics/reset/demand equivalence. Freeze counts only from completed helper Source. Retain every actual Header/full wire. Native Pose_V record order is unordered: compare by unique child frame at the retained 1e-12 local-pose bound without sorting or rewriting captured bytes. Separately use an actual C++ peer to check Rust's full canonical lexical-order packets. No fabricated full-vector native byte equality.

Three proposed disjoint leases: Scene owns pose_publisher.rs, private/pose_publisher.rs and new multiple_pose_publishers.rs; Server/reference owns corresponding Server three Rust leaves plus new native multiplicity helper and live peer; Root owns additive initial.rs/lib.rs/private-initial query/export, shared registration, one example/guide, scripts/docs/planning and procedures. No lease is released here. Sampling/math/clock/packet/IO lifetime bodies stay exact except independently reviewed admission/ownership seams.

Source-only preparation can overlap M54's sole serial normal run on exact qualified unaccepted M53 9bf3ada4. Final integration parent remains unselected until M54 actual normal and independent local DATA pass, every repair is adopted, and full overlap review preserves M54's lidar/query/shared changes. Preserve all old failures. Qualification requires whole Source review, current input closure, fresh scoped native DATA and literal DATA, real observed consumers, one consolidated formatter, unchanged full normal hook and strict coverage, independent local DATA/raw obligations, exact hosted/official and accepted-predecessor/Main gates. M1–M42/175 slices and M40 Latest remain unchanged; this proposal grants no acceptance or publication credit.


Root owns `initial_pose_publications_with_identity(&Simulation) -> Result<impl Iterator<Item = (u64, EntityKind, &str, &PosePublication)>, SceneResetError>`. It borrows kind, complete WORLD-scoped original name and publication from the immutable initial scene, including removed owners. The existing `(u64, &PosePublication)` query remains unchanged. Server preserves the Prepared `(owner_id, metadata, Arc<output>)` prefix and validates the full current/original/prepared identity and endpoint sets before any output lookup or declaration.

The current Source preparation parent is qualified, unaccepted M53 `9bf3ada4675022b45e39d5f089e1bb3aaeaa2c04`. M54 first full normal verification failed in two cases: malformed XML in a new identity fixture and changed legacy collision-error precedence. The exact failed tree and evidence are retained. The corrected full normal and strict coverage passed at local commit d140e51e, and independent retained local DATA passed (report a912f219). M55 integration onto that locally qualified, unaccepted parent requires complete repair preservation and independent overlap review. M1–M42, 175 accepted slices and M40 Latest remain unchanged.


## Revised reference input decision after the fixed-DSO diagnostic

The original M31 Server-based helper really loaded its PosePublisher DSO; the
old success is never relabelled object-only. The single new metadata diagnostic
returned 1 because that unchanged DSO has an empty RUNPATH entry. Preserve the
failed contract, raw dynamic table, result 5f05051f and Root receipt ee558685.
No diagnostic pass, runtime closure, safe loading or fresh capture is claimed.

The reviewed direct-object proposal fbdad455 and Source decision 26c7dbe2 select
a NEW reference helper recipe: link the unchanged original ET_REL object e5fa
into a fresh helper and call the original public Configure/PostUpdate interfaces.
Own instances through the public virtual System base; do not instantiate a
concrete private-PImpl deleter, copy private state or provide a replacement shim.
No Server, SystemLoader, dlopen or PosePublisher DSO is used by the new helper;
actual maps must reject that DSO. No original object or provider is rebuilt,
relinked or modified, and the failed empty-path guard stays unchanged.

Complete helper/recipe/object-input Source review and actual compiler/link/symbol
resolution, full input/provider snapshots and callback/wire DATA remain mandatory.
All helper ABI/counts/outputs and final runtime parent are held until established
by their required reviews and actual operations. Native numeric comparisons stay
keyed at 1e-12 with native record order untouched; Rust's lexical exact wire uses
a separate real C++ peer. There is no native Server/plugin-loader, Physics,
lifecycle, demand, RNG or whole Source/binary equivalence. M54's corrected full
normal and independent local DATA passed. Its complete repair/overlap adoption
review still precedes M55 integration.


## Second normal failure and complete Source correction batch

The corrected ID-width expression compiled, and the aggregate403 Python checks passed. Full normal attempt2 returned actual1 in1696.864 seconds with four failed cases across three targets. Retain failure receipt d81ae4d9, the complete original packet/logs, present partial profile index, missing renderer/coverage artifacts and unqualified preservation ref ea4eb52a/tree465548. No M55 qualification or successful results DATA follows this run.

Both new articulated physical controls passed a two-link tip to the queued-wrench API, which deliberately accepts only free single-link models. Preserve the exact typed NotFreeLink negative and apply the same force/torque to the same loaded BodyHandle through the existing public backend body-wrench API. Keep both fixtures, joint motors and all physical/acquisition/held/cadence/reset assertions unchanged; this is no wider queued-input admission or pose/velocity teleport.

The retained joint-state remap control exposed a production compatibility regression: old output keys were incorrectly checked against newly remapped original keys. Bind retained leases to immutable owner/channel/name/config, reject duplicate old endpoints/channels/shared output Arcs and rebinding, then validate all new resolved keys before IO. Valid removed original owners can still be restored. Keep the old joint-state case byte-exact and the existing endpoint-based output reuse, packet/binding/stream/IO behavior unchanged.

The new selected-link control despawned a real joint child, so global joint preflight faults before PostStep sampling. Snapshot and compare every cached field and floating-point bit for both channels, require the exact public joint-binding diagnostic and StepError::Faulted, fence all existing revisions, require no new enqueue, and retain terminal reset nonmutation. Do not change production fault handling, force cache clearing or promise peer rollback/fault isolation.

All four leaf corrections receive complete independent Source review as one batch, including exact full-file reversals to465548. Current guides and immutable native/literal artifacts remain unchanged. One affected consolidated formatter precedes the full mandatory normal/strict retry; independently audited actual results, accepted predecessor, hosted/official and qualified Main/readback remain required before acceptance or publication.


## Pre-execution endpoint-binding Source finding

After the four-case batch was reviewed and its affected formatter returned0, independent M56 review and Root's M55 trace identified the same concrete prepared-output gap in M55: swapping two distinct CPU output Arcs preserves tuple owner/config/name/cardinality while their existing UUIDs still point to the opposite IO declarations. No normal3 invocation occurred. Preserve original finding6297b8bd, first formatter b8f6e125 and all earlier Source/evidence.

The complete supplemental Source batch adds only immutable setup endpoint metadata and a pure matching predicate to PosePublisherOutput. Retained streams must match their OLD endpoint before valid remap restoration; each prepared output must match the NEW resolved endpoint before lookup/clone/IO. Keep UUID/job/Weak/dirty/drop/reconcile/declaration/publish behavior exact, with no registry or transport change. Strengthen the existing endpoint control with unique same-schema prepared and retained SWAP plus duplicate refusals; retain all eight old cases, IDs and prior physical/global-fault corrections. Five corrective Rust paths now include the previously unchanged IO leaf, so its old Source proof comes from exact failed tree465548/parent bytes, not an invented entry in the old34-path changed-source map.

Complete independent whole supplemental Source and factual guide review, one affected complete formatting pass, exact35-path current Source/census binding, and the unchanged full normal/strict/local DATA workflow. Frozen capture-runtime29 files and95 inputs, actual native/literal evidence and all prior failures remain unchanged. No Source qualification, acceptance or new native proof is inferred. M56's own narrow odometry-output fix is developed separately; both remain gated by their own reviewed candidates.
