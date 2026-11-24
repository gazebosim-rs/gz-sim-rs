<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# The sheet first tips during a loaded set-down

The arm finishes its lowering command at 2.550 s and starts opening its fingers at 2.650 s. Both independent runs pressed the folded sheet onto the table while the fingers were still closed. Both picked it slightly back up before opening. By 2.550 s the two sheets already had opposite angular momenta about their own centers of mass. [Watch the earlier set-down at 1,000 exchanges](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10021%2F) · [Watch 2,000 exchanges](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10022%2F)

| Measured while lowering with the jaws closed | 1,000 exchanges/ms | 2,000 exchanges/ms |
|---|---:|---:|
| First recorded floor load | 2.521380 s | 2.521375 s |
| First loaded particle | 0 | 0 |
| Last floor load before the sheet rises from the table | 2.549227 s | 2.549257 s |
| Total upward impulse during contact | 34.1502 mN·s | 34.1284 mN·s |
| Angular momentum about the sheet center, X axis, at 2.521 s | −0.331 nN·m·s | −0.633 nN·m·s |
| Angular momentum about the sheet center, X axis, at 2.550 s | −3.480 nN·m·s | +70.686 nN·m·s |
| Height difference between the positive- and negative-Y corners of panel one at 2.675 s | −1.64 µm | +34.61 µm |

The average upward table force over this first contact was about 1.23 N. The sheet weighs only 0.0824 N. The jaws were still pressing it down. The measured reversal precedes release and the later landing on opposite edges. Separating the jaws’ contribution to roll from the table’s contribution during unloading remains open. [Individual impacts at the final landing](cloth-table-particles.md)

## Two controls

A trial solved the viscous bending equations for all neighboring hinges together. An independent Rust test verified the final angular-rate equations, negative energy change, conserved linear and angular momentum, and agreement when hinge order was reversed. All five full 3.4 s grasp runs succeeded. At the finest two rates the change increased the largest trajectory difference from **328.30 to 330.54 µm**. The shipping solver retains its existing hinge law. The native gripper and actuator sequence were identical in both experiments. [Watch the experimental material at 2,000 exchanges](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10020%2F)

A second trial translated every model, particle and plane by 25 cm and repeated the same grasp. After subtracting the translation, the largest particle differences across the full trajectory were 0.50 µm and 0.54 µm for the negative and positive shifts at 500 exchanges, and 0.93 µm for the positive shift at 1,000. The zero-shift control reproduced the original particle trace byte for byte. The origin sensitivity at these settings is small compared with the 328 µm resolution difference.

Both early-contact runs used a fixed 1 ms actuator clock. Their matched material copies reproduced the real robot at all 40,000 and 64,000 traced native intervals. Each early-contact browser playback checked 1,932 particle positions and 5,796 native visual poses. The machine-readable results include the angular history and full-run p95 costs. Timing trials from different sessions are descriptive. [Numerical receipts](cloth-preload-study.json) · [Three native replays and the two experimental Rust changes](cloth-preload-study.tar.xz)

## Reproduce

Use the sheet-jaws command in the [corner-impact study](cloth-table-particles.md). For the first set-down, substitute the arguments below. To run either experimental control, apply its patch separately to source commit `63ce160`. The translation study reads `GZ_STUDY_Y_METRES`. Run complete trials by omitting both inspection arguments.

```text
--exchanges-per-ms 1000 --inspect-native-tick 2515000 --inspect-native-count 40000
--exchanges-per-ms 2000 --inspect-native-tick 5038000 --inspect-native-count 64000
```
