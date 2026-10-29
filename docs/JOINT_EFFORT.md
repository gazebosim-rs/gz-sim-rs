<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Joint effort commands

`DynamicBackend::add_joint_effort(id, torque)` accumulates generalized torque in
Nm, positive along increasing joint angle. DART and MuJoCo support their existing
world revolutes. Other adapters currently return `Unsupported`; no backend is selected
implicitly. This API does not enable finite position limits or new joint types.

DART stores commands by stable joint ID, separately from world wrenches. Scalar
inputs and their accumulated sum must be finite and at most 1e60 in magnitude.
The next successful nonzero step consumes them once. A zero step, rejected input,
failed preparation or native step refusal preserves admitted commands. Successful
joint/body retirement and either reset discard the affected pending efforts.
Initial-state snapshots contain construction state, not queued commands; preparing
a reset leaves current commands intact until that reset is applied.

Preparation projects each world-hinge effort onto its stored unit world axis and
combines it with the child's external COM torque. This is work-conjugate because
the hinge's angular Jacobian is that unit axis, including rotated links and offset
anchors. A temporary overlay checks all combined components before native state
changes. Existing world wrenches remain intact if a combination exceeds 1e60.
Commands remain bounded by registered joints. The no-effort path avoids the
extra map clone; active commands currently require a temporary wrench overlay.
Throughput and broader actuator support remain separate work.

## MuJoCo

MuJoCo accumulates finite scalar inputs with a finite sum. After settling pending
model edits, it resolves each joint's current DOF address and writes directly to
`mjData.qfrc_applied`. This is generalized force, with no actuator or world-frame
projection. Written indices are cleared after every native step attempt; the
pending queue is consumed only on success. Joint/body retirement and both resets
clear the affected commands, including after DOF addresses change.

Native fatal-warning behavior remains adapter-specific: MuJoCo resets its data
and reports a typed error. Queued joint effort survives for explicit cancellation
or retry, while world wrenches keep their existing attempted-step consumption.
This is not DART's previous-state rollback. No automatic retry or new solver
setting is introduced.

## Evidence

The public controls cover analytic inertia/sign, accumulation, normalized axes,
rotated offset-hinge pose/twist, combined world torque, both reset paths, joint
and body retirement, stale IDs, scalar and combined overflow, zero steps, native
refusal/retry and the explicit unsupported adapter path. Existing assertions and
solver settings are unchanged. No original-test credit is added by these controls.

`tests/fixtures/joint_effort.cc` uses the actual pinned Gazebo DART plugin's
`SetForce`, then `ForwardStep` at 1 ms. Six positive/negative/zero force pulses
match Rust joint position and velocity within 1e-12. The oracle binds its C++
source hash and upstream physics SHA; the normal tooling suite checks both.

Reproduce from the repository root with the pinned native source/build/install:

```sh
python3 scripts/probe-joint-effort.py \
  --physics-source /path/to/gz-physics --build /path/to/gz-physics/build \
  --prefix /path/to/installed/sdk --work-dir /tmp/gz-joint-effort-native \
  --report artifacts/joint-effort-reference.json
```

The helper recompiles the C++ client, checks selected upstream files against the
baseline, verifies the native build's source/install directories, rebuilds the
plugin target, and records mapped DART/ODE/plugin hashes. DART 6.13.2 and ODE must load from their system pkg-config
directories. The loaded plugin must come from that verified build directory. The report
qualifies a rebuilt target against installed dependencies; it does not claim a
fresh SDK configuration or a performance comparison.

The MuJoCo C++ oracle uses the pinned 3.11.0 SDK directly, with no actuators. A
free body occupies six DOFs before the driven hinge; six force pulses match Rust
at 1e-12. The helper rebuilds the pinned native target, verifies source/cache and
installed-header identities, and requires the mapped library to be that build.

```sh
python3 scripts/probe-mujoco-effort.py \
  --sdk-root /path/to/physics-reference --work-dir /tmp/gz-mujoco-effort-native \
  --report artifacts/mujoco-effort-reference.json
```

The SDK root is the source/build/install tree made by `bootstrap-physics.py`.
These are functional and lifetime controls, not performance acceptance.
