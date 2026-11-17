<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M69 accepted: two independently demanded depth cameras

Accepted Main `d19d5a69b94ed959bf1348665783996a531a3c36`, tree
`af81c8d49b53f5bd19a4f0a518fff67e86b146ad`, directly over accepted M68
`24ea62519d0a671f789e6b3fc98bd2b72de18ae2`. M1–M69 and 202 reviewed
slices are accepted; M65 remains the public Latest preview with 35 installed examples.

The complete normal and strict gates passed, followed by independent local DATA,
[exact review CI44109413456 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/44109413456),
168,593,943 bytes of durable original ZIPs, independent official DATA and ordinary
nonforced Main promotion/readback. Local and hosted observations each record
2,052 headless and 1,305 overlapping renderer passes, 498 aggregate Python checks,
41 finite examples and 14 auxiliary RGB cases. Quiet Python output does not expose
individual authored case outcomes. Four local and four hosted RAW comparisons
were each performed once, last, with counts 1862/530/55/64 and byte-exact merges.

The phase adds exactly two independently demanded calibrated 64 by 64 metric-depth
cameras with a separate strict depth-pair API and independent image/info demand,
10/20 Hz scheduling, canonical full numeric packets, peer clock isolation and
worker/lifetime retirement controls. Existing scalar and RGB-pair APIs remain strict.
No package refresh, browser depth panel, RGBD, point cloud, new physics or dependency
is accepted by this phase.

Both failed normal attempts remain unqualified and retained. The first stopped at
a stale strict finite count (40 versus 41). The second failed a new Busy assertion
after pumping an invalid worker; its actual alternative error was not printed and
remains unknown. The corrected test checks typed Busy at admission before pumping,
then retains exact peer publication/clock/no-retick and retirement assertions.
No production code or tolerances were weakened for that correction.

The immutable acceptance receipt is
`artifacts/m69-delivery/acceptance-main-promotion.json`, SHA256
`53c415581b398d629111bac10a28f9790bd71368bf69e42a601a6f87d4c378c7`.
Its proof cards bind original local, hosted, collection, official and Main facts.
The automatic Main CI is separate and was not queried for this acceptance.
