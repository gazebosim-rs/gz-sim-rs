<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Which corner of the folded sheet hits the table?

At 1,000 exchanges per millisecond the sheet landed on its positive-Y edge. At 2,000 it landed on the opposite edge 127.5 µs earlier. A violet sphere in each native replay marks the particle that received the greatest upward impulse during each displayed interval. It returns below the tabletop when the sheet is airborne. [Watch 1,000 exchanges](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10013%2F) · [Watch 2,000 exchanges](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10014%2F)

| Measured quantity | 1,000 exchanges | 2,000 exchanges |
|---|---:|---:|
| First impact time | 2.6892760 s | 2.6891485 s |
| First loaded corner | 11, positive Y | 0, negative Y |
| First upward impulse | 0.091405 mN·s | 0.090547 mN·s |
| Time from first contact to all six bottom nodes having touched | 13 µs | 282.5 µs |
| Upward impulse in the first 200 µs after each impact | 1.511975 mN·s | 0.738643 mN·s |
| Sideways impulse over the common 1 ms window | −0.014027 mN·s | +0.281290 mN·s |

At 2.689 s the two positive-Y bottom corners in the coarser run cleared the table by 35.62–35.63 µm. The negative-Y corners cleared it by 37.21–37.30 µm. In the finer run the positive-Y corners were 55.86–55.94 µm above the table while the negative-Y corners were only 19.07–19.08 µm above it. The sheet was already tilted the other way before either run touched down.

In the common interval from 2.6891 to 2.6901 s the coarser run's negative-Y edge received +0.02539 mN·s sideways and its positive-Y edge received −0.03350 mN·s. In the finer run both edges and the center received positive sideways impulses. The resulting motion still exceeds the 0.1 mm whole-trajectory goal. [Each particle, both native browser checks and coverage](cloth-table-particles.json) · [Prior trajectory and center-of-mass comparison](cloth-incoming-history.md)

The Rust solver probe records signed normal and tangential impulses, including unloading, on each contacted plane. It uses the same native collision solve as an unobserved material. A diagnostic accumulation overflow invalidates the reading and still allows the physical step to commit. The two material copies exactly reproduced the native scene at all 3,200 and 6,400 endpoints. Each particle sum agreed with the separate table reaction within 2.6 × 10⁻²¹ N·s. Every displayed contact marker was independently reconstructed from the impulse history and the native particle coordinates. Each WebGPU replay checked 161 frames and 1,932 particle positions and rewound to 2.689 s.

With the turntable and plane changes combined, 230 XPBD tests and nine focused CLI example tests passed. Exact scoped LLVM coverage reached 84/84 changed executable lines and 93/94 regions in XPBD, and 58/58 lines and 102/107 regions in the CLI example. Both combined targets passed strict Clippy. [Solver probe](../../../crates/gz-physics-xpbd/src/pad_loads.rs) · [Release coverage](replay-release-gate.md)

## Reproduce

From the simulator checkout, choose a fresh output directory for each rate. The 1 ms actuator clock stays fixed from time zero. Each clip spans 3.2 ms and emits a frame every 20 µs plus the final native endpoint.

```sh
export RUSTC_WRAPPER=
for rate in 1000 2000; do
  cargo run --release --locked --no-default-features -p gz-sim-cli --example local_robot_foundations -- \
    sheet-jaws "artifacts/table-corners-$rate" --jaw-mass-scale 0.015625 \
    --pad-compliance 0.000015625 --tune-damping --bend-damping 0.001 --pad-damping 4 \
    --panel-balance --static-plane-friction 0.7 --substeps 1 --exchanges-per-ms "$rate" \
    --inspect-native-tick "$((2689 * rate))" --inspect-native-count "$((32 * rate / 10))"
done
cargo run --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  --server --replay --web-viewer 10013 artifacts/table-corners-1000/replay
```
