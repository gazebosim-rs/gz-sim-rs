<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# D33: retire DART and MuJoCo

The owner directed on 2026-11-17: “Drop all dart and mujoco work. We will not use them.”
DART and MuJoCo are cancelled product backends, not deferred deliverables and not
completed parity work. This supersedes the corresponding earlier retention and
native-library decisions. Rapier f64 and TPE remain supported targets.

The bounded removal deletes the two adapter crates, their workspace and consumer
features/dependencies, native link setup and dedicated provisioning/verification
jobs. Mixed controls keep their Rapier/TPE cases and original assertions and
tolerances. Backend-only controls receive an explicit D33 retirement disposition;
they must not be counted as passing tests or covered behavior. An independent
reviewer must check the retirement inventory before one consolidated final check.
Remaining strict workspace/per-crate line and region coverage gates stay intact.

Historical source commits, accepted records, original fixtures, failure evidence,
paused worktrees, caches, SDKs and notices are preserved. This is source retirement,
not permission for disk eviction. Native Gazebo observations and reference tooling
needed to reproduce retained Rapier/system/Zenoh comparisons may remain reference
inputs; they do not expose our retired adapters or authorize new backend campaigns.
Legacy SDF engine/plugin names remain metadata where the format requires them;
selecting an unavailable product engine must fail explicitly without fallback.

PHY-03 and PHY-04 and their pending continuations are cancelled by owner. Remove
them from active dependencies and tracker denominators without marking them done.
Historical ledgers remain historical. Record the exact deleted controls and any
relocated shared fixtures alongside the implementation before final Source review.

The isolated retirement candidate starts from accepted Main `d19d5a69`.
Its adapter, consumer and gate source inputs are byte-identical to the initially
reviewed frozen M70 `fcd5a30d`; no M70 package/runtime Python change is included. M70's existing
code/package/install evidence remains unchanged and does not qualify this new
candidate. M70 is still unaccepted and its browser-origin approval remains pending.
No new M71 acceptance or phase number is invented for this cross-cutting task.
After complete implementation, regressions, docs and independent Source review,
run the maintained consolidated normal/coverage and exact hosted gates before
source publication or acceptance. No per-edit tests or linting.

The [retirement inventory](../planning/backend-retirement.json) names each retired
Rust/Python control, the retained mixed controls, deleted gate/source paths and
byte-preserved reference relocation. It records scope dispositions, not test passes.

D33 also retires shared interfaces with no remaining consumer or implementation:
`CompoundBodySpec`, `spawn_compound_link`, `add_serial_revolutes` and
`add_serial_revolutes_in_child_frames`. `CollisionSpec` and the retained plane
refusal remain. Four capabilities already implemented by Rapier (`add_revolute`,
`add_reduced_revolute`, `set_self_collision_group`, `add_joint_effort`) become
required `DynamicBackend` methods instead of carrying unused optional defaults.
Rapier's implementations and behavior are unchanged. External Rust trait
implementers must now supply those four methods; this source interface change is
intentional. The existing scheduling test recorder retains its original explicit
unsupported behavior. No coverage exclusion or threshold changes accompany this
cleanup.


The retirement was published to Main at
`8f13dc7446be4fded424b1eba83df4c8253d0cef` after its complete normal hook,
strict coverage, independent local audit, exact review CI
[44248938975](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/44248938975)
attempt 1 and independent hosted audits with durable originals. The preceding
candidate description records the historical separation from M70. M70 resumes in
a new isolated successor on that Main; it must keep the retirement and qualify
its own package/runtime changes before publication or phase acceptance.
