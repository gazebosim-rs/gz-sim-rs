<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Automatic standby on the local parcel arm

The arm places a 1 kg parcel on a shelf, retreats to the empty pickup station and parks after its links settle. Its next pickup command wakes it before the next physics step. In a separate run the loaded shelf disappears during standby. The arm wakes on that boundary and the parcel starts falling on the next solve. Both modes execute the same workcell edits and motor commands at the same simulation timestamps. The standby policy in this preview applies to the installed parcel-arm example. [Run the Linux preview](../../LOCAL_CPU_MVP.md) · [Watch the ordinary and shelf-removal recordings](adaptive-arm-view.md)

## Accuracy and repeatability

The largest distance between corresponding link centers was 2.83 µm on the ordinary route and 2.08 µm with the shelf removed. The parcel followed the same path throughout. Sleeping each body separately initially left the arm in the native solver's awake set. Parking the complete connected group reduced the difference in the first resumed mount impulse from 0.02768 to 0.000508 N·s. The first shelf and finger impulses agreed in both runs. The ordinary and removal routes reproduced across one and eight Bevy threads. A separate stress test repeated 10,000 complete park/wake cycles across reset. [Contact, trajectory and momentum measurements](adaptive-arm-handoff.md) · [Repeated switching under load](serial-switch-soak.md)

An earlier spindle fixture uncovered a separate case. Rapier still tracked the jaws and a free parcel across gaps of 6.65–19.76 mm. The engine now checks the live shapes and contact margins before retiring an unloaded prediction. Moving a neighboring body, resizing a collider or changing a contact margin between steps invalidates a stale clearance. A 1 kg neighbor moved back into contact at the same tick and delivered the same first impulse as continuous dynamics. An upright universal wrist with only damping or a weak spring also stays dynamic. [Native clearance regressions](../../../crates/gz-physics-rapier/tests/native_prediction_cut.rs) · [Wrist and stop cases](../../../crates/gz-sim-cli/examples/serial_switch_soak/tests.rs)

## Step cost

Each of five series ran five alternating-order pairs on an AMD Ryzen 9 9950X3D, pinned to logical CPU 5, with one Bevy compute thread and rustc 1.97.1. Each process started fresh. Both lanes recorded the same scene and 32,032 public 1 ms steps. The standby window covers the 500 solves starting at 31.032 s, or 1.56% of the run. Each cell gives the median of five per-process measurements. Per-step times surround the public physics advance. Elapsed time includes scene loading, control logic, standby checks and writing the replay. Timing runs disabled the optional detailed physics trace. The development host was shared during measurement. [All 1,601,600 step samples and five summaries](adaptive-arm-cost.tar.xz)

| Series and mode | Full run elapsed | All steps p95 | All steps p99 | Standby p95 | Standby p99 |
|---|---:|---:|---:|---:|---:|
| Installed c825db3, continuous | 4.519 s | 190.9 µs | 207.7 µs | 100.7 µs | 115.5 µs |
| Installed c825db3, adaptive | 4.350 s | 180.9 µs | 206.1 µs | 17.7 µs | 23.8 µs |
| Installed 537252e, continuous | 5.588 s | 203.6 µs | 251.2 µs | 126.0 µs | 133.1 µs |
| Installed 537252e, adaptive | 4.557 s | 193.5 µs | 206.1 µs | 21.3 µs | 25.8 µs |
| Installed 57f9b28, continuous | 3.903 s | 129.6 µs | 170.6 µs | 92.8 µs | 98.7 µs |
| Installed 57f9b28, adaptive | 3.871 s | 130.5 µs | 149.3 µs | 16.9 µs | 23.1 µs |
| Development 1, continuous | 4.127 s | 169.4 µs | 193.9 µs | 121.9 µs | 128.4 µs |
| Development 1, adaptive | 4.950 s | 204.5 µs | 214.3 µs | 24.3 µs | 38.5 µs |
| Development 2, continuous | 4.860 s | 193.8 µs | 211.9 µs | 134.8 µs | 145.3 µs |
| Development 2, adaptive | 6.148 s | 229.6 µs | 255.3 µs | 24.5 µs | 35.4 µs |

On the final installed package, parking cut standby p95 from 100.7 to 17.7 µs and standby p99 from 115.5 to 23.8 µs. Median elapsed time for the entire placement was 4.35 s adaptive and 4.52 s continuous. Across the five series, standby p95 fell by 80–83%, while whole-route elapsed time ranged from 18% faster to 26% slower. The full placement has too little idle time to establish a consistent end-to-end speedup on this host.

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

A fresh fetch of source commit `c825db3b9e41b0a05b748f8b9facb1399f4c9ff3` produced the x86-64 Linux preview. An unprivileged, network-disabled Ubuntu 24.04 container verified every packaged checksum, reproduced all four new recordings and the previous 1 kg placement byte for byte, and reran the 10,000-cycle comparison across reset. The archive SHA-256 was `6973dadea4c21915c77c05ac9cdcb12f51dc8adb620c3e1a86f2ff2b67d5b544`. [Build the package](../../LOCAL_CPU_MVP.md#build-the-installable-preview) · [Hosted checks for the tested source](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/45622329753)

## Focused test coverage

The table covers executable Rust changed since `2925c38`, including both installed example programs under the CLI and the vendored solver patch. LLVM 22.1.6 mapped the exact candidate's release executables. Each executable used its own matching profiles. Counts combine overlapping source locations once and exclude test-only source. The per-executable exports completed without profile mismatch warnings.

| Component | Changed lines exercised | Changed regions exercised |
|---|---:|---:|
| gz-physics-rapier | 155/162 (95.68%) | 285/292 (97.60%) |
| gz-sim-cli | 848/866 (97.92%) | 1535/1605 (95.64%) |
| gz-sim-scene | 46/47 (97.87%) | 76/78 (97.44%) |
| rapier3d-f64-0.36.1 | 193/194 (99.48%) | 340/353 (96.32%) |

All 45 ordinary serial-transfer example tests passed, including the earlier parcel-control regressions. The checks exercise correct and refused switches, contact prediction after geometry edits, paused shelf retirement, stale references, invalid commands, reset and the installed 10,000-cycle run. Historical spindle tests also protect loaded contacts, speed stops and wake behavior. The full-workspace release gate remains open. The next local iteration qualifies humanoid balance on uneven support and recovery from a push. [Release roadmap](../../PHYSICS_RELEASE_ROADMAP.md) · [Repository testing policy](../../TESTING.md)
