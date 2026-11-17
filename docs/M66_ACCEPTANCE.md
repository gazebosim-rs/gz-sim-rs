<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M66 acceptance

M66 is accepted at Main `1b0c3be1af7c751987df53b4b96269236f7ae9c5`, tree
`fd7edcdd920fb780551a3a23022b97b167138668`, accepted parent
`069f315c1466c40120d85f6944a5f478ef6e9c71`. M1–M66 contain 199 reviewed slices.

Full normal/strict coverage and independent retained local DATA passed. Exact
review CI [44010465041 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/44010465041)
and independent official DATA passed with durable original ZIPs. Ordinary Main
promotion and remote readbacks confirm the exact qualified commit.
The immutable local receipt is `artifacts/m66-delivery/acceptance-main-promotion.json`
SHA256 `ae9a03e8108eef9879021fa83ca8c39d1cb72856ff54aa4b76b026cc9d7c8fb7`.

Actual counts are 2047 headless, 1247 overlapping renderer, 476 aggregate Python,
38 finite examples and 11 RGB CLI controls. The 19 authored Rust functions map
to one headless and 19 renderer executions, with two separate borrowed renderer
support controls. Four local and four hosted RAW comparisons each ran once LAST.

Opaque canonical sphere visuals use protocol5 and one bounded lazy UV sphere.
Real software pixels, lifecycle and canonical Image/CameraInfo consumer proofs
passed. This is finite-mesh evidence, not analytic/native Ogre/GPU/performance
parity; parser normalization and discarded worker-exit limitations remain.
Generic/depth remain box-only. No package refresh was selected: public Latest
[m65-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m65-preview)
keeps its 35 installed examples, accepted viewer bytes and both14MB budgets.
Automatic Main CI is separate and was not queried. Do not repeat accepted proofs.
