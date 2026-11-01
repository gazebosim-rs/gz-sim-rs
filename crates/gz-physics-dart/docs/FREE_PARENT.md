<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# PHY-03 free-root pair

This records the initial contact-free slice. The later bounded static-contact
extension is described in [ARTICULATED_CONTACTS.md](ARTICULATED_CONTACTS.md).

The backend candidate started at `506f94a`. Coordinator integration now starts
at accepted `78b702f`, retaining the extraction-cache repair from `c5f73a1`.
Owned paths cover `crates/gz-physics-dart/{src,tests,docs}` and the focused Scene
consumer controls in `crates/gz-sim-scene/tests/body_joint.rs`.
This extends the two-body domain to one
isolated dynamic free root and one dynamic revolute child (seven DOFs), with
external contacts and active multibody limit rows still refused. Scene admission,
shared tests and public acceptance remain the coordinator's work. The original
ApplyJointForce ground-contact world is not qualified by this slice.

## Source and reference

The existing expanded COM Jacobian maps dependent coordinates into all seven
skeleton coordinates before native inverse-mass impulse response. A root velocity
edit snapshots the whole skeleton's rates and refreshes all cached bodies; a leaf
edit projects only its incoming coordinate while holding ancestors fixed. Native
joint-local force applies the child effort and free-root reaction. No solver or
numeric admission setting changes are planned.

Pinned gz-physics `b44cd0d0ea6606b6c98320c0c87a5670d4fc6be6`
`dartsim/src/FreeGroupFeatures.cc:216–270` sets free-root origin linear/angular
velocity. The reference records COM velocity separately using the rotated COM
offset. `free_parent.cc` uses that plugin API, one local revolute, gravity -9.81,
1 ms steps, nonzero root twists and child rates, and positive/negative force
pulses. The tilted case also has offset COMs. There are 34 complete observations.

```sh
python3 crates/gz-physics-dart/tests/fixtures/probe-free-parent.py \
  --physics-source /home/nkoenig/work/gazebo/rust/gz-physics \
  --build /home/nkoenig/work/gazebo/rust/gz-physics/build \
  --prefix /home/nkoenig/local \
  --work-dir artifacts/free-parent-reference \
  --report artifacts/free-parent-reference/report.json
```

The helper reuses `probe-moving-parent.py`'s fail-closed SDK/header/runtime
identity checks. This is the same explicitly hybrid SDK documented in
[MOVING_PARENT.md](MOVING_PARENT.md), with ROS DART build headers/link inputs and
verified OSRF runtime libraries, not a homogeneous SDK or performance comparison.
Both config hashes, actual compiler/link roots, installed source header identity
and loaded-library hashes are recorded in the report.

## Qualification status

The actual native reference ran successfully before production admission edits.
Independent prior review reproduced all 34 native observations and checked both
body impulse responses, all-seven-rate rollback, Scene wiring and admission-bound
subprocesses. Strict production/changed coverage and mandatory staged/hosted gates
remain pending. World-hinge removal with a live child remains refused even though
free-root construction is added.

On the integration base, nine focused backend tests pass, including all recorded
native observations and the repaired attachment rollback controls. Separate
prepared positives pass for the exact old native ownership input and the exact
old Scene XML. The Scene control checks endpoint binding, stationary motion,
effort response, loaded-reset identity and sampling, and model removal. The parent
must translate and rotate under internal effort, ruling out world anchoring. These
controls do not replace the native physical oracle or qualify external contacts.

Exactly three capability predicates change after independent prior approval:
the native free-pair refusal becomes attach/state-preservation/detach before its
unchanged world-root lifecycle checks; the backend refusal becomes attachment
and retirement before its unchanged reduced-joint/self-collision/error checks;
and only DART leaves the Scene engine-rejection array. MuJoCo and TPE errors stay
unchanged. The full DART package and all 15 Scene body-joint tests pass this fold.
Retained offset/rotated impulse controls verify both total world momenta, and a
native-origin bound failure restores all seven rates and subsequent motion.

The private boundary probe also ran 192 isolated mass/rotation/rate/timestep cases:
84 advanced and 108 explicitly refused stepping with unchanged public state.
These bounded diagnostic cases are supplemental, not exhaustive safety coverage
or a performance comparison. Acceptance still requires the full ordinary gates.

## Three-body investigation

The isolated `dart-free-chain-worktree` contains a native-first control for one
free root and two serial revolute children. It exercises actual third-body
`moveTo`, eight-coordinate impulses, immediate rollback with warm ancestor
Jacobians, contact-free steps and a middle-subtree split with last-owner
destruction. Its stationary input repeats the exact retained free-pair
`(existing leaf, extra)` refusal input. The isolated candidate's source-bound
Rust controls now exercise this input. After independent prior review, only
this one tuple now invokes the shared exact-input positive on fresh worlds; all
six tuple inputs, five other refusals and the original world's lifecycle checks
remain intact.

The actual native control compiled and exited successfully inside the existing
qualified `cb921f` image, using its unchanged packages and GCC 13.3. All eighteen
mapped libraries, including DART 6.13.2 and ODE, match that image's qualified
reference bytes. Two scenarios each perform 1,000 contact-free steps. The native
impulse path and all attachment, rollback, anchor, finite-state, subtree and
last-owner assertions pass. Fourteen complete body/joint observations are stored
in `tests/fixtures/free_chain.json`; the fixture identifies the actual source,
compiler, command, libraries and raw observation hash. It is direct DART evidence,
not Gazebo plugin/application parity or a general numeric-domain proof.

The first capture stopped before compilation on a wrapper newline comparison;
the first compile found a fixture type-name collision. A later native execution
passed its assertions but produced duplicate default-joint-name messages. These
outcomes remain separate. Explicit body-specific joint names, matching the
adapter, produce the final exact success output and empty stderr without filtering.
Commands, compiler dependency data, maps, executable and original logs are kept
under ignored `artifacts/native-first`. No SDK or image was rebuilt.

The private candidate changes only the symmetric Rust/native serial-pair
extension predicate and an earlier contact refusal for trees larger than two
bodies. Existing pair/static-contact mathematics and all active-limit guards
remain unchanged. The exact old-row transition was independently approved after
its literal two-spawns, first-hinge, extra-spawn construction and complete
snapshot equality passed. The Rust tests
include source-bound native parity, attachment/contact rollback, reset, queued
commands, subtree ownership and isolated numeric/contact/limit controls. The
first focused run passed all eleven controls, including all fourteen native rows
and forty-eight owned numeric subprocesses: forty-two advanced and six explicitly
refused. These counts do not imply exhaustive numeric-domain safety. A separate
exact-input replay then preserved all three complete body states with exact
snapshot equality, and the private native-owner split/retirement control passed.
The subsequent full DART package passed all 151 tests across 23 test targets,
with no failures or ignored tests; this includes the six direct native fixtures.
Selected DART all-targets Clippy also passed on the folded source. Mandatory
strict coverage, staged and hosted acceptance remain pending.

The candidate domain does not include a world-rooted three-body chain, branches,
cycles, four bodies, component merging, three-body contacts or active multibody
joint-limit solves. Scene topology, its one-collision limit and the complete
upstream B03 fixture are outside this backend-only slice. Root reviewed the actual
native evidence before authorizing this isolated admission implementation. Real
adapter evidence and transition review are complete for this candidate; ordinary
strict local/hosted gates remain required.
