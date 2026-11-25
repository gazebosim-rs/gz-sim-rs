<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Automatic standby on the local parcel arm

The arm places a 1 kg parcel on a shelf, retreats to the empty pickup station and parks after its links settle. Its next pickup command wakes it before the next physics step. In a separate run the loaded shelf disappears during standby. The arm wakes on that boundary and the parcel starts falling on the next solve. Both modes execute the same workcell edits and motor commands at the same simulation timestamps. The standby policy in this preview applies to the installed parcel-arm example. [Run the Linux preview](../../LOCAL_CPU_MVP.md) · [Watch the ordinary and shelf-removal recordings](adaptive-arm-view.md)

## Accuracy and repeatability

The largest distance between corresponding link centers was 2.83 µm on the ordinary route and 2.08 µm with the shelf removed. The parcel followed the same path throughout. Sleeping each body separately initially left the arm in the native solver's awake set. Parking the complete connected group reduced the difference in the first resumed mount impulse from 0.02768 to 0.000508 N·s. The first shelf and finger impulses agreed in both runs. The ordinary and removal routes reproduced across one and eight Bevy threads. A separate stress test repeated 10,000 complete park/wake cycles across reset. [Contact, trajectory and momentum measurements](adaptive-arm-handoff.md) · [Repeated switching under load](serial-switch-soak.md)

## Step cost

The installed package and two earlier runs of the development build each ran five alternating-order pairs on an AMD Ryzen 9 9950X3D, pinned to logical CPU 5, with one Bevy compute thread and rustc 1.97.1. Each process started fresh. Both lanes recorded the same scene and 32,032 public 1 ms steps. The standby window covers the 500 solves starting at 31.032 s, or 1.56% of the run. Each cell gives the median of five per-process measurements. Per-step times surround the public physics advance. Elapsed time includes scene loading, control logic, standby checks and writing the replay. Timing runs disabled the optional detailed physics trace. The development host was shared during measurement. [All 960,960 step samples and three summaries](adaptive-arm-cost.tar.xz)

| Series and mode | Full run elapsed | All steps p95 | All steps p99 | Standby p95 | Standby p99 |
|---|---:|---:|---:|---:|---:|
| Installed, continuous | 3.903 s | 129.6 µs | 170.6 µs | 92.8 µs | 98.7 µs |
| Installed, adaptive | 3.871 s | 130.5 µs | 149.3 µs | 16.9 µs | 23.1 µs |
| First development series, continuous | 4.127 s | 169.4 µs | 193.9 µs | 121.9 µs | 128.4 µs |
| First development series, adaptive | 4.950 s | 204.5 µs | 214.3 µs | 24.3 µs | 38.5 µs |
| Second development series, continuous | 4.860 s | 193.8 µs | 211.9 µs | 134.8 µs | 145.3 µs |
| Second development series, adaptive | 6.148 s | 229.6 µs | 255.3 µs | 24.5 µs | 35.4 µs |

On the installed package, parking cut standby p95 by 82% and standby p99 by 77%. Whole-route elapsed time differed by 0.8%, with adaptive finishing 33 ms earlier. The earlier series also showed large standby savings, but their adaptive whole-route medians were 20% and 26% slower. With a short standby window and substantial variation on this host, these runs establish a standby benefit and leave the whole-route speedup unresolved.

## Reproduce the timing comparison

Run from the simulator checkout using the installed Linux preview. Set `arm` to its executable. Choose a logical CPU available to that process and a fresh output directory. The Rust audit tool checks the reported quantiles, step counts, simulation clocks and standby bounds before summarizing the pairs.

```bash
arm=/absolute/path/to/preview/bin/serial_transfer
core=5
mkdir -p target/arm-timing
for pair in 1 2 3 4 5; do
  out="target/arm-timing/pair-$pair"
  mkdir "$out"
  modes=(dynamic adaptive)
  if (( pair % 2 == 0 )); then modes=(adaptive dynamic); fi
  for mode in "${modes[@]}"; do
    taskset -c "$core" "$arm" clear-obstacle "$out/$mode-view" --fidelity "$mode" \
      --parcel-kg 1 --grip-friction 0.85 --threads 1 --timings "$out/$mode.json" > "$out/$mode-result.json"
  done
done
RUSTC_WRAPPER= cargo run --locked --release --manifest-path tools/physics-audit/numerics/Cargo.toml \
  --bin arm_tier_cost -- 31032 31532 target/arm-timing/pair-{1,2,3,4,5}
```

## Installed preview

A fresh fetch of source commit `57f9b283d6268b6e4e22ed84abd6a8e78e93ddd8` produced the x86-64 Linux preview. An unprivileged, network-disabled Ubuntu 24.04 container verified every packaged checksum, reproduced all four new recordings and the previous 1 kg placement byte for byte, and reran the 10,000-cycle comparison across reset. The archive SHA-256 was `9e7ca8518af0aa01db6b1ab7cfafd0ce4f3cf915a4c05eb66959265fe6c35399`. [Build the package](../../LOCAL_CPU_MVP.md#build-the-installable-preview) · [Hosted checks for the tested source](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/45620815849)
