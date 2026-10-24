# Instructions for implementation agents

Read `docs/PLAN.md`, `docs/DECISIONS.md`, and the document for your assigned area.
Confirmed owner decisions override proposals. Do not resolve an outstanding
product decision by silently deleting behavior. Read `docs/STATUS.md` to distinguish
implemented behavior and checks from planned release requirements.

## Architecture

- Keep sdformat-rs, gz-fuel-tools-rs, gz-transport-rs, and gz-msgs-rs independent.
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
