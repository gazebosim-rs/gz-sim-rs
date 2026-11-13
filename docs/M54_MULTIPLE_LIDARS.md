<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Multiple planar CPU lidars

M54 is under development. Native component and fixed-literal DATA passed.
The first full normal run found two regressions; fixture and error-precedence
corrections are being batched. Full qualification and release remain pending. M1–M42 remain accepted and M40 stays the installed
preview. See [the delivery contract](M54_DELIVERY.md) for gates.

The Rapier scene supports up to eight otherwise-supported loaded planar CPU
lidars, each with 2–128 beams. At most 1,024 rays are due in one tick. This is a
resource bound, not a throughput measurement. GPU, multilayer and interpolated
scans, intensity output and point clouds are outside this increment.

Each sensor owns its demand, acquisition cadence, cached sample and seeded noise
stream. Without demand, neither its acquisition deadline nor its noise stream advances.
Rays follow the full acquired sensor WORLD pose, including tilted scan planes,
and exclude only the sensor's own parent body. Other sensor bodies can obstruct
them. Positive infinity is a present miss range; it is not an absent scan.
Beam order, endpoint-inclusive angle arithmetic, range limits and existing noise
admission remain unchanged. A lidar fault remains terminal for the entire run.

`examples/m54_multiple_lidars.sdf` places two independent three-beam sensors in
front of one wall. Both scanners are static; this example does not demonstrate
force-driven motion. Separate regression controls exercise dynamic translation
and rotation, demand, cadence, loaded reset, owner removal and endpoint conflicts.
The existing finite-example qualifier registers this world as its 28th Source
entry; it has not yet run for M54. No new launcher or renderer is needed.

Every Server lidar needs a unique normalized/remapped endpoint, disjoint from
world services and other sensor, controller and command endpoints. Distinct frame
names alone do not make shared scan topics valid. All candidate identities,
original metadata and endpoint bindings are checked before publishers are reused
or created. A removed original retains its reserved endpoint for loaded reset.
The borrowed `initial_lidar_publications_with_identity` query returns the original
public ID, Sensor kind, complete WORLD-scoped name and publication metadata,
including sensors removed from the current scene. Raw WORLD names containing
`::` remain invalid SDF input.

Loaded reset restores original IDs, metadata, poses and noise streams. Removing
one model retires its sensors; remaining owners retain their independent demand
and outputs. Removing the last owner releases the shared sampler/reset callbacks.
Shutdown and canonical LaserScan/Header encoding retain their existing behavior.

The fresh C++ reference used the original CPU-lidar SensorFactory component with
controlled complete ray results and connected scan subscribers. All 25 child
stages returned zero; independent native DATA, fixture generation and independent
fixed-literal DATA also returned actual Root zero. The immutable fixture retains
ten configurations, 1,032 LOCAL ray pairs, thirty complete pose/RayResult inputs
and thirty actual scans containing 3,096 ranges: 1,719 finite and 1,377 positive-
infinite present misses. Each scan retains its full canonical bytes and Header;
thirty standalone Header copies are not extra transport messages. See the
[compact reference](../planning/reference/m54-multiple-lidars.json) for exact
commands, providers, input domains, receipts and provenance.

The observed Scene numeric and Server full-LaserScan/Header consumers passed
independent Source review. The Server copies actual captured output poses and
ranges through the unchanged packet path; input rays do not form an output
oracle. Native component, live physical geometry and exact-wire proofs keep
their existing separate bounds. This reference grants no original System/ECM,
Rapier raycast, Physics, demand/schedule, reset/removal/lifecycle or global-RNG
equivalence. Accepted `73e03b9f` helper-only capture and locally qualified, unaccepted
runtime `9bf3ada4` are distinct. No SDK/provider rebuild or old campaign replay occurred.

The complete implementation and shared Source batches are integrated after
independent review, preserving M53 repairs. No M54 formatter, Rust/Python tests,
full normal, coverage or local qualification has run. Final integration review
precedes consolidated formatting and unchanged strict checks. Hosted/official,
accepted-predecessor and Main/readback gates remain; Source readiness and native
DATA are not acceptance.
