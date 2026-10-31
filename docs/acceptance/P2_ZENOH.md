<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# P2 requalification: selected headless world over Zenoh

The selected P2 runtime contract is requalified at
[5870373](https://github.com/gazebosim-rs/gz-sim-rs/commit/5870373f0d8280601b051f2cbb1bb0aba264f70f).
Its normal staged hook and
[exact hosted verification](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41248418524)
passed. Independent review checked the downloaded source, inputs, coverage,
native clients and software-renderer evidence. This report publishes the fresh
Zenoh matrix, dependency receipt and matched performance observations. Its own
publication commit must pass the normal hook and exact hosted/source checks
before the coordinator advances P2 on the dashboard.

The [fifteen-row matrix](../../planning/acceptance/p2-zenoh.json) names the current
passing witnesses, fixture hashes, protocol translations and limits. The
[earlier P2 record](P2.md), its matrix, refusal and dependency receipt remain
unchanged historical evidence. Their ZeroMQ results do not qualify Zenoh.

## Selected behavior

The world has a static ground, a moving sphere, explicit inertia and gravity,
and a noise-free IMU at a 1 ms timestep. A separate world-revolute variant checks
the initial joint contract. TPE supplies its declared motion/contact semantics;
full-f64 Rapier supplies the selected dynamic backend. Existing native DART
physical observations remain qualified cross-engine oracles for that selection.
They do not establish general same-engine or articulated DART parity.

The current witnesses cover resolved SDF frames and hierarchy, local resources,
exact stepping, start-paused operation, clock/poses/IMU, velocity/force input,
pause/reset/removal, bounded admitted work and native-owner shutdown. Stable
opaque entity roles replace C++ storage identities. Reset reconstruction occurs
in `prepare_io`, outside raw ticking; `update` combines preparation and ticking.

Real C++ Zenoh peers exercise 39 server scenarios: 19 world cases, 16
schedule/orientation/noise cases and four root-clock cases. Additional live CLI
peers run on TPE, Rapier, DART and MuJoCo. Rate and force harnesses expand seven
and five native process scenarios respectively; the latter belong to three Rust
tests. These are actual scenario expansions, not counts of compiled executables.
The selected reference has Zenoh enabled, with native source, binary and mapped
library identities bound in the hosted evidence.

## Quality and dependency closure

The runtime qualification passes 783 headless, 435 renderer and 217 tooling
tests. Headless hosted coverage is 9763/9935 lines and 14937/15376 regions;
local coverage is 9761/9935 and 14935/15376. Renderer coverage agrees at
5058/5191 and 7523/7830. Every required production crate and both workspace
profiles exceed 95% for lines and regions. Missing/duplicate source, uncovered
entries, changed-line checks and generated-binding guards remain enforced.
The additional renderer evidence does not make rendering a P2 requirement.

Independent audit binds all 1767 hosted files, 68 production sources and 272
inputs to the runtime commit. The
[current dependency receipt](../../planning/acceptance/p2-zenoh-normal-dependencies.json)
records 426 normal/build packages for both the CLI and explicit headless
workspace on Linux x86-64, with all 22 manifests and Cargo.lock checked against
5870373. Neither closure contains a renderer, window or GPU package. This is
the Cargo closure; native runtime linkage has separate qualified records.

Runtime evidence uses the pinned SDF and Fuel revisions recorded in that
receipt. This publication does not silently upgrade SDF or change admitted
scene behavior. It identifies the equivalent-source schema-only Messages
release `d030d06`; the original minimal consumer inventory retains its actual
`a39a945` provenance. The new source-policy record requires fresh build evidence
in this publication's checks; earlier build and timing records are unchanged.

## Matched observations

The [complete public packet](../../planning/benchmarks/zenoh-matched-streams-v1/README.md)
contains one warmup and all seven measured alternating pairs of 1000 steps.
Each request follows complete clock/pose/IMU receipt by at least 20 ms. The
measured work includes the server call, receipt, decoding and assertions;
pacing and wall time are recorded separately. Physical values, frames,
sequence/covariance, cadence, raw bytes and role identities retain their original
checks. Only the previously declared opaque-ID varint adjustment applies to
cross-run byte comparisons.

Median work latency is 63289.562 ns native and 58563.369 ns Rust per step,
about 7.47% lower for this small TPE case. This is a paced observation workload,
not an unpaced throughput, causal optimization or balanced robotics result.
Native local delivery may bypass serialization while Rust uses encoded queues.
Unequal direct and unpaced Server measurements are not folded into this result.

The packet preserves every textual observation and original raw file hash.
Its explicit privacy projection replaces three local path prefixes only;
numeric values, wire hex, sample order, map columns and stdout/stderr remain.
The native ELF remains locally retained by its original identity. The immutable
native image is locally qualified, not claimed publicly pullable. The packet
binds its release executable, source and image separately from debug hosted
artifacts.

## Remaining scope

This selected milestone does not complete P0, broad loader/sensor/network
packets, ACT-01 zero-cost absence, general stopped-peer pressure, physics/backend
conformance, articulated robotics, Bevy sensor families, all simulator systems
or release qualification. The separately reviewed CPU IMU shutdown-storage fix
is outside measured commit 5870373 and earns no borrowed performance credit.
P3 and P4 continue with these boundaries intact.
