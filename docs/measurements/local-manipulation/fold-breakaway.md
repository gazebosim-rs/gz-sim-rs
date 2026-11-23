<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Persistent slip in a folded grasp

The arm lifts the same 8.4 g folded sheet on five- and seven-column grids. Its two fingers close until the mean force between the layers lies within 2.00 ± 0.25 mN in two consecutive sampling windows. Each trial starts from the bench, repeats the pickup and force-controlled closure, and verifies the incoming native robot state, material state, previous contact reactions and full acquisition trace against an unloaded twin. For the clock comparison, the simulation increases material substeps and solver iterations only after acquiring this common grip. The public robot and actuator step stays at 1 ms.

Each nonzero case ramps a distributed +Y force up in 50 ms, holds it for 100 ms, ramps down in 50 ms and observes recovery for another 200 ms. Each finger tracks the same eight identified material locations on its panel in its moving jaw frame. The analysis subtracts the contemporaneous unloaded twin from this displacement. A load qualifies as persistent slip if either patch remains more than 0.25 mm forward on every one of the final 20 recovery ticks. The displacement criterion follows the arm's independent rigid-object slip metric. The scan reports the measured bracket for this criterion and pulse duration. The retained-support check counts the same window's ticks on which both pads support the sheet, total vertical force is within 2.5% of weight, bench force is below 1 µN and the sheet skin clears the bench by more than a centimetre.

Reproduce the scan and its native archives with a new output directory.

```sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli \
  --example local_robot_foundations -- fold-breakaway target/my-fold-breakaway
```

Each `mesh-{columns}-{clock}-{millinewtons}` directory includes the starting scene, full force trace, independent particle frames and a native gz-web-rs replay. A load of zero identifies the matching unloaded branch. The fine-clock scan repeats the primary clock's measured endpoints and searches further when they move.

| Points across sheet | Material substeps × iterations | Highest hold / lowest lasting slip (N) | Lower-load movement, left / right (mm) | Upper-load movement, left / right (mm) | Upper-load bilateral support in final window |
|---:|---:|---:|---:|---:|---:|
| 5 | 32 × 24 | 1.075 / 1.100 | 0.0015 / 0.0042 | 12.1328 / 12.1361 | 20/20 |
| 5 | 64 × 48 | 1.075 / 1.100 | 0.0015 / 0.0043 | 11.0050 / 11.0063 | 20/20 |
| 7 | 32 × 24 | 1.125 / 1.150 | 0.0026 / 0.0070 | 5.9650 / 5.9672 | 20/20 |
| 7 | 64 × 48 | 1.125 / 1.150 | 0.0007 / 0.0046 | 4.9336 / 4.9357 | 20/20 |

Movement is the smallest signed jaw-relative displacement on each pad during the final 20 ms, after subtracting the unloaded twin. The load bracket width is the force resolution of the scan. The row does not establish where microscopic sliding first occurs.

The five-point sheet also received a 1.150 N pulse in the primary sweep. It left the fingers, accumulated about 1.19 m of jaw-relative travel and had no jaw support in the final window. At the same force, the seven-point sheet retained support. Both contact patches must be tracked to distinguish a loaded grasp that slides from complete ejection.

[Measured cases](fold-breakaway.json) · [Slip and retained support](fold-breakaway-response.svg) · [Seven-point fine-clock replay in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9817%2F)

The release and LLVM-instrumented runs reproduced all 39,000 measured ticks and 20 native archives byte for byte, excluding host timings. The four 0 and 0.3 N branches on the primary clock also reproduced the preceding study. Nine related tests and strict Clippy passed. For the breakaway changes the exact example and test executables covered 142/143 production lines and 307/313 regions.
