<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# How the folded sheet arrives at the table

The independent 1,000- and 2,000-exchange simulations released the jaws only 0.5 µs apart. Immediately after release their most distant corners were 0.024 mm apart. The difference reached 0.026 mm just before touchdown. Both centers of mass followed free fall to within 1.2 × 10⁻¹⁴ m/s over the measured unloaded interval. Their incoming poses and velocities carried differences from pickup, and the table amplified these during impact. [Watch 1,000 exchanges](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10010%2F) · [Watch 2,000 exchanges](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10011%2F)

| Measured event | 1,000 exchanges per ms | 2,000 exchanges per ms |
|---|---:|---:|
| Last positive fingertip reading | 2.6763680 s | 2.6763685 s |
| First positive table reading | 2.6892760 s | 2.6891485 s |
| First native upward table impulse | 0.091405 mN·s | 0.090547 mN·s |
| Upward impulse over each run’s first 200 µs of impact | 1.511975 mN·s | 0.738643 mN·s |
| Sideways Y impulse from 2.6891 through 2.6901 s | −0.014027 mN·s | +0.281290 mN·s |

Comparisons used 160 exactly aligned samples spaced 0.2 ms apart. At 2.6914 s, node 8 was the first sampled corner more than 0.1 mm apart. At 2.7020 s, node 6 reached 0.350 mm, with a sideways center-of-mass difference of 0.330 mm. The 25 ms whole-run publication had shown a maximum of 0.328 mm. Refining from the same state at 2.680 s gave a smaller 0.028 mm peak over the local landing window. The signed sideways impulse motivates measuring which individual parts of the folded sheet touch first. [Full traces and window impulses](cloth-incoming-history.json) · [Matched-state experiment](material-support-edge.md)

Both full simulations used the same 1 ms actuator period from time zero. Their native material baselines independently reproduced all 32,000 and 64,000 solver intervals in the recorded window. To reproduce, use the viscous-pad command with `--static-plane-friction 0.7`, one material substep and the following arguments. [Base fixture and commands](viscous-gripper-pads.md)

```text
--exchanges-per-ms 1000 --inspect-native-tick 2672000 --inspect-native-count 32000
--exchanges-per-ms 2000 --inspect-native-tick 5344000 --inspect-native-count 64000
```
