<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Independent PosePublishers

M55 is under development. The scoped native reference and independent data audits passed;
integrated Rust verification and release acceptance remain pending. M1–M42 remain accepted and M40 is
the installed preview. See [the delivery contract](M55_DELIVERY.md) for gates.

Up to eight top-level flat Rapier models may each configure the existing
PosePublisher built-in. Each owner selects at most 128 model/link frames,
for at most 1,024 records across all owners and sixteen dynamic/static channels.
These are resource bounds. Existing frame selection, local transforms, frame
classification, publication defaults and lexical Rust record order remain intact.

Each owner keeps separate dynamic and static clocks. Acquisition does not depend
on listeners. Paused updates remain suppressed; equal-time unpaused and backward
updates retain the existing eligibility rules. Static and dynamic output groups
keep their existing order and never send empty vectors. Model static status alone
does not move every record into the static stream.

All dynamic/static topics must be distinct after normalization and remapping,
including other world, sensor, controller and command endpoints. An endpoint
remains reserved for an original owner removed before loaded reset. The complete
original/current/prepared set is validated before any publisher is reused or
created. Loaded reset restores original IDs, metadata and local-frame bindings;
a terminal fault continues to fail the whole run.

`examples/m55_multiple_pose_publishers.sdf` places two existing two-link articulated
rigs at different locations with independent topic pairs and publication rates.
The example applies no commands; physical regression controls separately exercise
force, torque and joint-motor motion. It needs no new launcher or renderer.

The fresh reference links the unchanged original System object into a new helper
and supplies complete controlled LOCAL ECM inputs. Capture, independent native
DATA, immutable fixture export and independent literal DATA passed. The old DSO
input was rejected for an empty search-path entry and stays rejected; no provider
was rebuilt or modified.

Native Pose_V record order remains untouched. Numeric comparisons use unique
child frames at the retained 1e-12 component bound; a separate C++peer checks
Rust's full lexical canonical packets during required verification. This reference
does not prove original Server, plugin-loader, Physics, lifecycle, demand or RNG
equivalence. Real-force, torque, joint-motor and reset/removal controls remain
separate required Rust checks.
