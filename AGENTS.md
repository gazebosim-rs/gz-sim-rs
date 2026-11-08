# Instructions for implementation agents

> M1–M20 are accepted and published at qualified Main `9a3e81c`.
> M20's checked installed preview is published as `m20-preview`; its frozen
> source, artifacts and original failures remain preserved.
> Current M21 altimeter noise and M22 air-pressure noise use independent Source
> lanes under D30–D32. Neither inherits acceptance. One integrator serializes
> local heavy checks. Preserve accepted evidence, paused trees and Fuel work.

Read `docs/HANDOFF.md` first for the current state and next tasks, then
`docs/PLAN.md`, `docs/DECISIONS.md`, and the document for your assigned area.
Confirmed owner decisions override proposals. Do not resolve an outstanding
product decision by silently deleting behavior. Read `docs/STATUS.md` to distinguish
implemented behavior and checks from planned release requirements.

## Current priority after explicit resume (D30)

Read `docs/DELIVERY_PLAN.md` and `docs/M21_M22_DELIVERY.md` before editing.
Root integrates M21/M22 in `gz-sim-m21-m22`; sensor authors own the disjoint
leaves/tests/native helpers/guides recorded there. Reuse the pure Gaussian library
unchanged. Both native sensors apply noise at zero elapsed time; pressure retains
native double `sqrt(stddev)` variance. No tests, lint or native captures run until
the complete Source/regression/docs batch and independent review are finished.
Parent M20 acceptance and all exact gates remain prerequisites for promotion.

The M20 preparation details below describe the frozen parent scope, not another
authoring lease or inherited acceptance.
M19 is accepted at `e807a97d`, published as `m19-preview` with actual package,
clean Ubuntu installation, installed robot/browser controls and owned shutdown.
Its full normal hook, exact hosted CI and independent audits passed. Preserve
all original failures, unknown causes and accepted viewer bytes.

M20 pairs explicit launcher `run --lifecycle` with existing native
`--web-lifecycle` and outer viewer `lifecycle=1`. It exposes the existing admitted
local lifecycle controls and publication credit without changing defaults,
protocols, queue bounds, the full M19 robot or viewer bytes. The complete scope,
separate worktrees and narrow file leases are in M20_DELIVERY.md. Root owns
integration, shared planning, publication and the tracker. Do not run per-edit
checks or create extra private proposal/review machinery. Under D32, continue
to the next useful bounded phase after acceptance while retained work remains.

Finish source, meaningful regressions, documentation and independent Source review
before one consolidated normal required pass. Batch real failures before affected
reruns; preserve assertions, tolerances and strict coverage/mandatory jobs.

Root owns release/publication decisions. Preserve all mandatory hooks, exact
hosted jobs, strict coverage, license/attribution records and historical failures.
Do not replay accepted M1–M11 manual proofs or the already-completed M12 payload,
archive and notice-data audit. Full P0–P7 and unsupported configurations remain
outside this bounded closeout. Any later named increment needs an explicit
scope of at most five working days including independent review and checks.

The owner explicitly resumed implementation. Work only on the current delivery
or a concrete blocking dependency; earlier pause records are historical. The
maintained viewer is `../gzweb-rs` (`gazebosim-rs/gzweb-rs`, Bevy 0.20), not the
older untracked Bevy 0.18 prototype under `/home/nkoenig/work/gazebo-web`. Reuse
its existing renderer, UI and backend interface. Stop unrelated backend/parity/coverage optimization work;
preserve pending candidates and the Fuel owner's work. If a phase will overrun,
report it and split optional scope; never weaken an exit criterion or gate.
Batch source review and fixes before a single final verification pass.

## Architecture

For M12 integration, package ordinary copied files and explicit source/build
identities. Installed execution must not require the development checkout, RAM
symlinks, a compiler, native physics SDK or remote asset service. Serve the
existing viewer only on its admitted loopback origins and fail occupied ports
without touching their owners. Keep physics authoritative, canonical commands
and all accepted live/replay protocols unchanged. Document the actual binary
profile, system-library closure, browser requirements and limitations. Include
project and applicable dependency/asset license notices in the distribution.

- D28/D29 supersede earlier transport/Messages scope: keep sdformat-rs,
  gz-fuel-tools-rs and schema-only gz-msgs-rs independent. Use upstream Zenoh
  directly; no gz-transport-rs target, ZeroMQ compatibility, successor transport
  facade, custom Messages runtime, reflection framework, message CLI or codec work.
  Consumers generate bindings from pinned proto sources with maintained tools.
  Preserve historical work; migrate consumers before removing old runtime trees.
  Do not edit the active Fuel rewrite without an explicit handoff from its owner.
- Put simulator, sensors, physics integration, and residual math in this Cargo
  workspace. Do not copy the old repository layout into four isolated ports.
- Use Bevy ECS for entities, queries, change tracking, resources, and scheduling.
  Do not recreate EntityComponentManager or build a second general-purpose ECS.
- Use Bevy rendering. Do not port gz-rendering, Ogre, or gz-gui. Keep the server
  usable without a GPU, window, or renderer dependency enabled.
- Keep DART, MuJoCo, and TPE. Add the chosen Bevy-integrated Rust backend. Do not
  implement Bullet or Bullet Featherstone. Backend limitations must be explicit.
- Do not introduce gz-cmake, gz-utils, gz-tools, gz-plugin, or gz-common successor
  repositories without an approved architecture decision. Preserve required
  behavior through standard libraries, dependencies, and focused workspace code.
- Reuse mature Rust libraries before writing general-purpose algorithms. Record
  the exact missing semantics and tests before adding residual math or utilities.
- Keep independent repositories free of Bevy and simulator dependencies. Shared
  value types must not create a dependency cycle through this monorepo.
- Build simulation systems into this workspace. Do not implement a dynamic system
  plugin loader, native system C ABI, extension host or replacement gz-plugin.
  Recognized legacy SDF plugin records are configuration for built-in systems.
- Activate systems only for loaded world/model requirements. An absent system has
  no scheduled callbacks, per-tick polling, owned heap state, workers or endpoints.
  Maintain demand through load/spawn/remove/reset transitions; release resources
  after the last consumer and reconstruct them outside stepping when needed.
  An empty query or a per-tick run condition alone does not meet absence criteria.
- Use f64 for authoritative physical state, SI units, explicit frame conventions,
  and integer simulation timestamps. Rendering transforms are derived data.
- Keep network, filesystem, and GPU waits outside the simulation's critical step.
  Bound queues and buffers; make overload behavior observable and tested.

## Correctness and evidence

- Preserve behavior, file formats, transport protocols, and existing test intent.
  New Rust APIs are allowed. Do not recreate the C++ API by default.
- Trace each upstream test to an unchanged harness, faithful Rust port, or an
  explicitly reviewed disposition. Keep fixtures and assertion strength. Missing
  hardware, skipped tests, and unimplemented features are not passing results.
- Production Rust line and region coverage must each be strictly greater than
  95%, for the workspace and every production crate. Follow `docs/TESTING.md` for
  feature coverage, exclusions, native code, shaders, and local/CI enforcement.
- Never weaken tests, inflate coverage with meaningless execution, delete hard
  code from the denominator, or alter numerical tolerances just to get a pass.
- Add meaningful negative, boundary, lifecycle, and regression cases. Fuzz parsers
  and ABI inputs. Test real backends as well as fakes.
- Measure performance against the pinned C++ implementation with equal settings.
  A backend change, reduced fidelity, or fewer sensors is not a rewrite speedup.
- Keep unsafe code in small audited boundaries. Document each unsafe invariant.
  Do not use unchecked Send/Sync to make a native engine fit Bevy scheduling.
- Do not allow Rust panics or C++ exceptions to cross an FFI boundary. Follow the
  chosen library's failure contract; unrecoverable native faults are not caught
  by catch_unwind. Test ownership, destruction, and concurrent shutdown.
- Run the required hooks and CI checks. Do not use `--no-verify`, skip failing
  jobs, or call an unavailable check successful. Report the exact limitation.

## Code and comments

- License original project code and documentation under Apache-2.0. New crates
  inherit the workspace license. Preserve imported license and attribution notices.
- New source files use `Copyright (C) YEAR Open Source Robotics Foundation` and
  `SPDX-License-Identifier: Apache-2.0`, after any shebang. Use the creation year;
  preserve original notices and years on imported material. Include LICENSE and
  NOTICE in distributions.
- Write ordinary, idiomatic Rust. Use concrete types and small cohesive modules.
  Add traits and indirection only for a real boundary or multiple implementations.
- Use names that express units, frames, and ownership where ambiguity matters.
  Avoid cryptic abbreviations, gratuitous one-liners, and repetitive wrappers.
- Keep code and comments concise, direct, and human. Comments explain a reason,
  invariant, unit convention, or non-obvious tradeoff; do not narrate the code.
- Document public contracts, failure modes, and safety requirements. Avoid filler,
  promotional language, generated essays, and speculative future abstractions.
- Use typed library errors. Reserve unwrap/expect for tests or a locally proven
  invariant with a clear explanation. Do not hide partial failure with defaults.
- A TODO needs a task ID, owner, and completion condition. No success-returning
  stubs or placeholder features in accepted milestones.

## Work protocol

1. Claim a bounded task from `docs/IMPLEMENTATION_TASKS.md`. Record owned files,
   dependencies, test IDs, and expected evidence before editing.
2. Read the relevant upstream code and tests at the pinned SHA. Documentation can
   be stale; confirm behavior in code and a reference run.
3. Inspect existing dependencies and modules before adding another implementation.
4. Implement one reviewable vertical slice. Update its test mappings, capability
   records, documentation, and benchmarks in the same change.
5. Run applicable checks and attach exact commands, versions, and results. State
   what remains unrun. Do not claim end-to-end success from a unit-only result.
6. Have a different reviewer examine semantics, test integrity, unsafe code, and
   performance evidence. The task author cannot approve their own test retirement
   or coverage exclusion. The reviewer need not be another AI agent.
7. Commit only owned changes. Do not reset another contributor's work. Changes to
   shared contracts need coordinated review before consumers are updated.

Completion means the task's acceptance evidence exists and required checks pass;
it does not mean only that the code compiles.

## Live progress

Update [the public tracker](https://gazebosim-rs.github.io/rewrite-progress/)
when a bounded task starts, a reviewed slice is accepted, a blocker changes, or
verification evidence changes. Follow the tracker repository's `status-contract.md`
and use its `scripts/update-status.py` to publish only `gz-sim-rs` status. Keep
local reports in ignored `artifacts/`. Repository agents own their own reports.
Reviewed increments do not complete broad plan packets; coverage is not progress.
The coordinator alone updates phase acceptance and individual system status.

## Continuation after explicit resume

The owner paused work on 2026-11-02. Do not resume implementation, agents,
checks or publication of pending feature candidates without explicit resume.
After resume, prioritize the current weekly delivery in DELIVERY_PLAN.md;
standing reviewed/checked publication authorization still applies. The
coordinator owns simulator/browser integration and phase acceptance. Choose
bounded work that advances the runnable milestone, not unrelated broad parity.
Keep required gates and accepted behavior intact; notify the owner about a
shipped milestone, a meaningful failure or a genuine product decision.

## Owner guidance (2026-10-27)

These answers from the owner apply to every agent, Claude or ChatGPT.

- Keep the real gates: tests, coverage above 95% per crate, the staged
  pre-commit hook, hosted CI, real C++ interoperability and preserved upstream
  test intent. Do not add process on top of them: no frozen proposal files,
  hash-bound review receipts, CPU/disk slot reservations, versioned reruns of
  the same check, or archiving every failed build. Keep `artifacts/` small.
- The coordinator may commit and push reviewed work to main, publish dashboard
  status, and edit sdformat-rs, gz-msgs-rs and gz-transport-rs. Never edit
  gz-fuel-tools-rs. List non-regenerable files for the owner before deleting.
- Ask the owner only for real product or dependency decisions, such as which
  native library build to depend on or what to reject; record each answer in
  DECISIONS.md. Make routine engineering choices yourself.
- Write status and reports in plain sentences. Earlier compressed shorthand
  could not be read by the next agent or the owner.

## Lessons from the P2 and P3 work

- Review catches real defects. Every adapter needed several fix rounds before an
  independent reviewer approved it. Give the reviewer the code and the author's
  claims to verify, not your conclusion, and have it re-check each fix commit.
- Native libraries differ by build. A test that passes against one library on
  `LD_LIBRARY_PATH` may abort against the pinned one (DART assertions). Pin the
  runtime library (DT_RPATH, run-time `dladdr` check) and test which file loaded.
- Treat a process abort from admitted input as a blocking bug, including inside
  third-party code. Search for them with child-process fuzz tests at the
  admission bounds, not only with hand-picked cases.
- Compare against the native engine before calling a behaviour a bug: the DART
  divergence reproduced exactly in the native plugin.
- Parity fixtures need the generating program, exact command and library
  hashes checked in, and must regenerate byte for byte.
- Lazy or batched work must not move a failure from the call that caused it to
  a later step; admitted edits must always compile and step.
- Changed-line coverage counts build scripts. Keep `main` in build.rs to one
  line over a tested function, and test it the way Cargo runs it.
- Measure performance claims against the native plugin with equal settings, and
  include the scene's real access pattern (queries after each edit), not only
  the easy case.
- Parallel agents each work in their own git worktree on separate crates. The
  coordinator owns shared files (workspace members, Cargo.lock, scene, CLI, CI)
  and lands each piece as one squashed commit through the hook.

