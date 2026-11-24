<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Recovering a slipping sheet

The gripper watches the original material patch on each finger while an external force pulls a suspended 8.4 g folded sheet sideways. Both arms in each comparison start from matching native joint, material and tactile histories. A guarded arm may tighten by up to 0.15 mm over 30 ms after sustained slip. If slip persists it lowers toward the receiving plane, and may open only after that plane carries at least 80% of the sheet’s weight for 20 consecutive milliseconds. The unguarded arm uses the same original pressure controller and transverse load.

| Sheet width | Pull | Guarded result | Unguarded result |
|---|---:|---|---|
| 5 and 7 columns | 0.30 N | Held without tightening | Identical through all 1,200 ticks |
| 5 columns | 1.10 N | Tightened at 62 ms, held with 1.106 mm residual slip | Held with 12.136 mm residual slip |
| 7 columns | 1.16 N | Tightened at 64 ms, held with 1.086 mm residual slip | Held with 17.407 mm residual slip |
| 5 columns | 1.25 N | Lowered at 99 ms, sheet escaped at 140 ms | Escaped at 132 ms |
| 7 columns | 1.30 N | Lowered at 100 ms, sheet escaped at 142 ms | Escaped at 133 ms |

The additional 1.14 N five-column trial changed the outcome. Tightening at 55 ms retained the sheet for the full 1.2 seconds. The unguarded sheet cleared both fingers at 173 ms. The guarded sheet still shifted 5.112 mm. [Play both versions and compare the measured force and motion](fold-recovery-rescue.md)

Escape ends a grasp trial after four consecutive airborne milliseconds with both finger forces below 1 nN and every particle, including its skin radius, beyond each finger’s lateral edge. The terminal frame is captured even when it falls between the usual 25 ms captures. The timed runs record every physical step through that endpoint. The receiving surface in this fixture is an ideal horizontal plane. None of these overloads achieved a controlled placement. [Per-case reactions, residual slip, clocks and p95 costs](fold-recovery.json)

The archive test checked all twelve cases. gz-web-rs played and rewound 1,956 frames, checking 72,468 poses and shapes and 47,040 material positions. The optimized and instrumented builds reproduced all 85 measurement and replay files byte for byte. The example suite passed 70 tests. The changed command code covered 252/254 lines and 507/523 regions. The scene supervisor covered all 21 changed lines and 21 regions. [Guard and control timelines](fold-recovery-response.svg)

From the simulator workspace, generate all twelve recordings in an unused directory. The optional `--columns 5 --load-mn 1140` arguments select a single matched pair.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- fold-recovery target/my-fold-recovery
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9850 \
  target/my-fold-recovery/mesh-7-1160-guard/replay
```

The optional `--follow-dropped` switch continues through 1.2 s after the matched grip. The five-column 1.25 N overload can also run until the escaped sheet comes to rest. [Sliding and rest measurements](fold-rest.md)
