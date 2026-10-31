<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# PHY-03 free-root pair

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
