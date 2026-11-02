<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Producer constructor and automatic-inertia adoption

This consumer source batch is unqualified. The original constructor adoption pinned
producer `c94da539`, published after exact hosted run `41417270384` and independent
official audit. That consumer gate failed after 359 seconds at fresh B03 motion.
The current joint-pose adoption pins accepted successor `f0d05146`; see
[NATIVE_JOINT_POSE_ADOPTION.md](NATIVE_JOINT_POSE_ADOPTION.md). Accepted simulator Main
still pins `13adde94`. Producer acceptance does not qualify a changed simulator.
The adopted producer includes bounded rotation-only automatic inertia from
`a549b745` as well as the radian constructor correction.

The migration requires no DTO, dependency-edge or public API rewrite. Producer
changes since the consumer pin expand the bounded single box/sphere automatic
inertia domain and add owned light DOM helpers. Scene does not call those new
light mutation helpers. Existing backend collision, COM, topology and numeric
admission policies remain authoritative.

The supplemental `sdfb246_admission.rs` table retains all seven original XML
inputs, their order and URI `rejected.sdf`. Two prior producer refusals become
positive controls: omitted collision density and an explicit identity collision
pose. The omitted density remains `None` in metadata while the producer computes
with native density 1000. Explicit density 2170 remains present. Both cases
compare their computed inertial values with the equivalent existing unposed,
explicit-density input, then exercise actual mass/inertia response and metadata
mutation, reset, removal and identity restoration on all four backends. TPE
retains its external-wrench refusal.

The other five producer negatives retain their error codes and source URI.
Manual negative density keeps its separate existing Scene refusal. No original
upstream completion credit or numerical tolerance changes with these supplemental
test dispositions.

The producer's radian constructor correction preserves native ordered Euler
products and first quaternion normalization. Degree and quaternion-input paths
remain unchanged. Its effect on the failed B03 trajectory is not yet proven.
The combined simulator candidate retains strict fresh-native B03 comparisons,
historical cached references and the reviewed atomic child-frame construction
changes. The preceding 361-second qualification failed, with all 486 retained
Rust values unchanged from its earlier run. That failure is not acceptance.

The dependency revision/lock change was made only after producer qualification
and publication. No package versions or dependency edges changed. Complete
independent review of this combined consumer batch, then run one consolidated
mandatory simulator verification. Preserve actual failures and batch
corrections before affected reruns. No checks have run for the successor joint-pose consumer batch.
