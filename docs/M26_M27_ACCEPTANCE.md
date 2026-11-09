<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M26/M27 accepted source features

M26 ideal AirSpeed and M27 flat-model PosePublisher were accepted at verified
Main promotion on 2026-11-09 at 06:36:36 UTC. The exact commit is
`0f52559202b58b73adfdb79a59b02fd7d3c99272`, tree
`c2cd94713d4e228755b6dc62acef382b37464490`, parent accepted M25
`08ae55256f6f29c1aaa4bd9ef5b43c2da2f9c86a`.
[Exact review CI](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42570074529)
attempt 1 passed all 16 steps. Normal nonforced Main publication and exact remote
readback completed after independent Source, native, local and official audits.
No result is inferred for the separate automatic Main workflow.

## Accepted behavior

M26 admits one direct-link top-level Rapier AirSpeed sensor with zero wind,
default atmosphere and default no-noise metadata. It uses actual sensor-point
velocity, inverse composite sensor-frame rotation and the native float pressure
and temperature evaluation. Published pressure has the native signed Pa-scale
numeric value despite the protobuf's hPa comment; no extra conversion is added.
Unsupported metadata, multiple/nested sensors and invalid inputs fail explicitly.

M27 admits one built-in flat-model vector PosePublisher with bounded model/link
selection. Its parent-relative poses, per-record headers, zero wire IDs, absent
outer header and unpaused cadence follow the pinned native contract. Native
records are compared by unique child-frame key, not unordered-map iteration
order. Loaded reset reconstructs initial state; direct core reset's retained
timer remains a separately documented convenience. See the complete scope and
limits in [M26/M27 delivery](M26_M27_DELIVERY.md).

Both features passed canonical wire, atomic admission/collision, bounded IO,
demand/absence, pause/reset/removal, weak-generation and stale-data controls.
This does not establish general Gazebo compatibility, another backend's sensor
support, new renderer behavior, trajectories or performance.

## Qualification

The final complete normal hook passed in 2040.364 seconds. Retained producer and
commit-hook results are zero; the original outer watch was unavailable at a
later poll, so no independent outer wait receipt is invented. Actual profile
results are 1,506 headless and 913 renderer Rust passes; these profiles overlap.
The 353 quiet Python passes are an aggregate without individual quiet outcomes.

| Profile | Local lines | Local regions | Hosted lines | Hosted regions |
| --- | --- | --- | --- | --- |
| Headless | 26328/27001 | 39791/41286 | 26323/27001 | 39787/41286 |
| Renderer | 16248/16690 | 24638/25660 | 16248/16690 | 24638/25660 |

Every required production crate and separate workspace profile passed strict
line and region coverage above 95%, with actual-M25-parent changed-line gates.
Separate runs are neither combined nor forced to have identical hit sets.
Each audit made its four byte-exact raw comparisons once, from 1780/375/18/33
inputs. The proof binds 1,488 Git entries, 51 changed paths, 1,437 protected parent
entries, 717 source inputs, six retained auxiliary ELF copies and four actual
finite copied-CLI controls. Full-profile executable records are metadata, not
retained binary bytes. Finite controls do not prove sensor acquisition.

The focused native run passed all 32 stages and nine modes in 69.300 seconds,
retaining 342 canonical message rows. AirSpeed point/component assertions inside
the helper are not additional printed observations. PosePublisher selection,
cadence, pause, real time-only reset, component poses and headers passed. The
349-node SDK closure and explicit current inputs/providers remained bound.
Historical native noise/joint roles remain separately scoped evidence.

Three original hosted ZIP streams total 123,164,564 bytes. All remain in RAM and
in verified individual durable SSD copies; decoded reports do not replace them.
Remote DSO provenance is retained, but remote DSO bytes are not archived or
locally rehashed. Key independent receipts are:

- Native DATA: `6b149a1331a6cdf5ab7d3327a2fdb945c7f5adfea3ed2ea032cfd6df43b1a2f3`.
- Local DATA: `85c2dcd614d4757f04806a87ac5a67894f2b10065ab199002fd409df1dca06d2`.
- Official DATA: `cdf67c3f83783b02916bf1efde47887d6fa0802e25af18c7c3b06739a6395a09`.
- Main promotion receipt: `b5fa7921d238345401e60cf7da9a503e8ffb74e630fd5a4afae3be3022aefac5`.

## Preserved failures and distribution boundary

The first normal run's three test-contract failures, the correction's unsupported
`World.clone()` compile failure and the native rewind fixture's callback deadline
failure remain original failures. The native control acknowledgement only queued
reset: its first unpaused iteration performed reset without a measurement.
Two finite paused steps now consume and apply reset, with an actual callback at
time zero while paused, before the unchanged ten advancing steps and three-message
assertion. No deadline, numerical tolerance or retained assertion was weakened.
All prior frozen trees, original logs, profiles and accepted manual evidence are
preserved; accepted manual qualification was not replayed.

M25 remains the verified installed preview and Latest, with its original outer
143 and separate successful retained completion faithfully disclosed. M26/M27
acceptance adds source features, not a new package, installed browser proof or
release-profile measurement. [M28/M29](M28_M29_DELIVERY.md) continues with pressure
noise and a separately qualified fresh installed preview.
