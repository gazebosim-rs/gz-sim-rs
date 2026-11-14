<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M57: independent loaded joint-state publishers

M57 admits zero through eight independent JointStatePublisher model owners in the existing top-level dynamic Rapier domain. Each owner keeps its admitted single-axis revolute graph, a nonempty deduplicated selection of at most 128 joints, its own rate and one canonical Model endpoint. The aggregate selection ceiling is 1,024 slots; this does not expand any existing physical graph limit. The delivery budget is five working days including review and mandatory verification.

The exact development parent is locally qualified, unaccepted M56 `d635b119ca8d9e3f338fb0cf8af7df88b71a7f34`, tree `09d855cb9479033c3394ee5003c2dbfddb59f12b`. Its full normal hook and independent retained audit passed. They establish the parent, not M57 acceptance. M1–M42 remain accepted and M40 remains the installed Latest. Hosted Ubuntu snapshot recovery remains unconfirmed; no CI retry or workaround is selected.

## Behavior and exclusions

The change broadens owner count, not joint physics or message contents. Sampling, model LOCAL pose, child-relative construction joint pose, positions, velocities, axis metadata, numerical rules and force omission stay unchanged. No measured force, axis2, new joint type, effort/velocity-limit support, controller-chain expansion, nested/static model domain, native backend or transport is added. Upstream Rust Zenoh and schema-only Messages remain in use.

One family schedule serves configured owners. Each owner retains its own publication clock and sample revision. Only positive elapsed time below its period suppresses publication; equal-time updates can publish even while paused. Core reset and loaded-world reconstruction retain their different existing history behavior. Zero owners and final-owner removal must own no joint-state callbacks or publication resources. Removing a valid owner must leave supported peers active. Existing global joint-binding faults remain terminal; no rollback, per-owner fault isolation or peer progress after a global fault is promised.

Before acquiring or reusing outputs, the Server validates the complete current, immutable-original, prepared and retained sets. IDs, model kind, full world-prefixed names, complete metadata, selected joint ownership and raw/remapped endpoint collisions must agree. Removed originals are restored from the blueprint rather than required to remain live. Retained output endpoints are checked against their old keys before remapping; prepared outputs must match the new effective keys. Distinct same-channel Arc swaps are refused. Valid legacy manual singleton behavior and error precedence remain intact.

## Parallel ownership and evidence

Root integrates shared identity queries, the finite fixture, registries and guides. Scene and Server owners use separate Source worktrees with disjoint leases. The reference owner authors the original helper and input/schema proposals separately. All use the reviewed scope at `artifacts/m57-delivery/phase-source-contract.json`; Source authors do not approve their own retirement or coverage exclusion.

The fresh reference calls the original JointStatePublisher through Configure and PostUpdate only. Its fixed pair/eight ECM graphs use explicit LOCAL poses and controlled joint q/v inputs with empty force arrays. Every input/getback, Info, actual call, fence, received Model/nested field and wire remains accounted for. The first lazy publisher's time-zero call and actual callback are retained; no warmup is erased. Provider/header/ABI/link admission and exact procedure review precede execution. This grants no original Physics, Server/SystemLoader, loaded-world lifecycle, solver or RNG proof; the accepted M18 physical evidence stays unchanged.

The agreed borrowed fixture preserves actual native IDs and order. Scene's observed control must use genuine loaded graph/handles and fixed initial pose getters, then supply only public JointSample q/v/time/epoch inputs before one unchanged family sampler call per group. It must not write backend or pose state. This is a controlled-component comparison; actual physical motor/force controls are separate. Server's captured-value encoder and actual Rust canonical peer proofs distinguish native IDs/order from live Rust identities. Native bytes are never rewritten to fit Rust.

## Exit criteria

Complete implementation, meaningful regression Source, guides and independent Source review; batch all findings before consolidated checks. Retain the separately strengthened old count control alongside new owner-count, malformed/late-invalid, endpoint/Arc, rate, reset, removal, global-fault and real motor/force regressions. Native capture, independent native DATA, immutable export and independent literal DATA must bind exact original inputs and actual exits before generated golden data is inserted.

Then run one reviewed complete normal hook with unchanged strict workspace/per-crate line and region gates, actual-parent changed-production coverage and all required finite/native-free/renderer controls. An independent retained results audit must bind actual tests, Source and artifacts and compare the four fresh RAW sets once at the end. No product, native or old RAW replay substitutes for missing proof. Exact accepted predecessors, hosted qualification, independent official DATA with durable originals, and ordinary Main/readbacks still precede acceptance or publication.

Current evidence: the fresh original-object Configure/PostUpdate campaign completed once with actual Root0 and all 41 actual child stages 0/reaped. Independent native DATA, immutable borrowed export and independent fixed-literal DATA also each completed with a separate actual Root0. The two modes contain 10 configurations, 20 global groups, 100 complete owner inputs/Post calls/fences, 66 Model messages, 108 selected Joint records and 278 raw rows. The aggregate owner count of 10 covers separate pair/eight instances, never ten simultaneous owners. Fixture `c0dc5db7` is 480,049 bytes; provenance `2b4a98c3` is 374,652 bytes. No native or product replay supplies these proofs.

The first fixed-input diagnostic remains actual1 for its pre-provider phase-field method guard. Its corrected diagnostic remains actual1 for the original DSO's empty RUNPATH entry, after two real readelf children 0/reaped; original DSO admission and loader use stay refused. A separately reviewed exact unchanged 170,920-byte ET_REL object was the one input to the new owned helper link. The old DSO was not rebuilt, relinked, patched or loaded. The successful new helper's link/ELF/maps and complete retained native data are separate from both failures. Original M18 Physics/Server/SystemLoader evidence keeps its earlier scope.

Observed-consumer Source is independently reviewed and integrated with the audited fixture/provenance, exact helper and append-only hooks. Scene uses genuine loaded graph/handles/definitions and verifies fixed construction getters for all selected and unselected joints; all public JointSample inputs precede one unchanged sampler call per group, with no pose/backend writes. Server's isolated captured-value encoder checks its canonical numeric/metadata boundary separately from untouched native root/nested wire and order custody. The reviewed live Rust peer Source separately checks actual Rust-assigned identities and canonical bytes when the mandatory verification runs. None of these scopes grants solver/gathering/native lifecycle or whole native-versus-Rust wire equivalence.

The original and Name-import formatters each completed with actual Root 0.
The 22 headless/11 overlapping renderer Source controls are obligations, not
individual runtime passes. The first normal failed compilation after
185.73234095191583 seconds at eight uses of one missing Name import. Its failed
ad809bd7 tree/result/logs and direct tree ref remain intact. The reviewed import
correction compiled under the second full normal; the first failure stays failed.

The second normal retired actual Root/child 1 after1650.1202665108722 seconds.
Strict Clippy, 403 aggregate Python checks and the maintained native-free/RGB
stage passed. Keep-going runtime execution found exactly two failed targets:
Scene276/1 and Server443/1. Full normal/strict coverage and local DATA remain
incomplete. Failed tree281b6af9 is retained directly at
refs/qualification/m57-normal-attempt2 with its result9a587807 and original logs.
No partial success grants qualification or acceptance.

Both corrections preserve the closed production domain. The observed Scene
control must load the supported unbounded/zero-damping graph, verify its genuine
construction getters and public sampling values, and separately retain exact
captured finite-limit/damping config/output custody. Explicit typed refusals
cover finite limits and nonzero damping. It must not overlay native metadata on
a loaded publication. Server's terminal control must corrupt a live selected or
unselected binding before preflight; complete entity disappearance is a distinct
retirement case, and a positive subperiod call may suppress selected-ID reading.
All cache bits, enqueue boundaries, Weak/output retirement and terminal reset
assertions remain under those explicit premises. No production callback, fixture,
native input or tolerance changes. Finish the complete correction/regression/docs
batch and independent Source review before one affected formatter and full normal
retry. The independent retained DATA and hosted/official/Main gates remain held.
