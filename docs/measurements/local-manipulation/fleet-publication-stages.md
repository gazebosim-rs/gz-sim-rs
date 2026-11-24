<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Cost of publishing a 100,000-deck fleet

Three consecutive Rust release runs recorded elapsed wall time for the native solve and each Bevy publication system. Each run used 100,000 moving decks, 1,000 free payloads, 404,001 public poses and one active IMU. The experiment used the same automatic scene and 10 ms clock as the [full-observation benchmark](fleet-translation.md). Timing surrounded each system and ended before writing its sample. All three processes used CPUs 24–31 on the shared host, with 20 initial steps discarded and the next 100 retained.

| Public stage, median / p95 in ms | Run 1 | Run 2 | Run 3 |
|---|---:|---:|---:|
| Native step | 6.600 / 9.594 | 4.434 / 5.830 | 4.231 / 6.090 |
| Body poses and velocities | 2.171 / 3.776 | 1.286 / 2.072 | 1.235 / 2.046 |
| Canonical link to model | 4.373 / 7.841 | 2.273 / 4.231 | 2.416 / 4.687 |
| Parent-relative poses | 3.017 / 5.004 | 1.670 / 2.918 | 1.688 / 3.910 |
| Attached world poses | 4.125 / 6.354 | 2.142 / 3.272 | 2.165 / 3.676 |
| IMU sample | 0.007 / 0.014 | 0.005 / 0.007 | 0.005 / 0.007 |
| Complete public tick | 21.221 / 29.366 | 11.865 / 18.324 | 12.102 / 16.803 |

The native solve is the largest single stage in all three runs. Together the four pose publication systems occupy more time than that solve. The second and third runs show similar median costs, while the first run was slower across all stages. Individual stage percentiles come from different ticks and should not be added. These measurements locate the cost of the observed world, including its hierarchy, without establishing an isolated speedup. The 100× target for the fully observed CPU scene remains open.

The same ordinary scene has a [gz-web-rs replay and reproduction command](fleet-publication-view.md).
