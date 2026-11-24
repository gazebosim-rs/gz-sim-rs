<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# CPU MVP robot performance

A speed-limited screw and universal wrist lifted a 200 g parcel with opposing prismatic jaws, then released it onto a moving plate. The always-dynamic and switched robots received the same control inputs over 10,000 public 1 ms steps. Both first touched the parcel at tick 1274, maintained bilateral contact for 1,050 loaded steps and lifted it 126.595 mm. The switched robot parked for 750 unloaded steps, woke on the next control input twice, and retained its three held efforts when it refused to park under load.

## Accuracy and repeatability

Over the complete trajectory, the largest difference in position across five tracked links was 4.790 µm. The largest difference among the four measured contact impulses was 0.802 µN·s. Both runs settled at 1.962 N on the plate. These are differences from the always-dynamic run of the same solver. The rotating joint's maximum speed, individual contact impulses and other state differences appear in the [machine-readable results](mvp-local-performance.json).

Three single-thread runs and one eight-thread run produced byte-identical native recordings for each mode and identical numerical results. Resetting and rerunning each switched fixture (standard and speed-limited) reproduced the complete sampled state sequence. The separate public journal regression replayed screw and universal drive inputs on the moving wrist. All 24 public journal regressions passed, including wrong-joint inputs, which leave the trace at the last committed step.

## Timing

These are medians of three paired release runs pinned to logical CPU 5 on an AMD Ryzen 9 9950X3D, Linux x86_64. Each pair ran the always-dynamic mode first. The one- and eight-thread counts configure Bevy's compute pool. This robot uses one serial native solver. The eight-thread run used logical CPUs 2–9. The viewer used the same byte-identical frames from the earlier run. The final benchmark used candidate `836bbcd` with the floating-base motor, static plane and asset changes.

| Metric, single compute thread | Always dynamic | Switched |
| --- | ---: | ---: |
| Wall time to load, simulate 10 s and write a native recording | 6.324 s | 5.746 s |
| Simulated seconds per wall second including the recording | 1.581 | 1.740 |
| Simulated seconds per wall second in public physics steps | 1.595 | 1.763 |
| p95 public step across the full cycle | 0.8231 ms | 0.7600 ms |
| p95 public step during the two parking windows | 0.9281 ms | 0.0296 ms |

The switched run used 9.2% less total time, and its median full-cycle p95 was 7.7% lower. Individual p95 measurements varied from 0.797 to 1.099 ms for the reference and 0.752 to 0.783 ms for the switched robot. End-to-end time runs from immediately before each world load through native archive write. It excludes compilation, opening the common SDF for metadata, the comparison between completed runs and the browser. Timings vary with hardware and system load.

## Reproduce

Run from the workspace root, using a fresh recording directory for each invocation. The example writes `comparison.json` and a native archive in each of `dynamic/` and `switched/`. Select an available logical CPU for `taskset`.

```bash
mkdir -p target
RUSTC_WRAPPER= taskset -c 5 cargo run --locked --release --no-default-features \
  -p gz-sim-cli --example parked_spindle -- target/mvp-robot --capped --threads 1
RUSTC_WRAPPER= cargo test --locked --release --no-default-features \
  -p gz-sim-cli --example parked_spindle
RUSTC_WRAPPER= cargo test --locked --release --no-default-features \
  -p gz-sim-cli --test local_cpu_replay
```

## View the run

The gz-web-rs WebGPU viewer played all 251 frames of each recording and verified 3,765 body poses and shape records per mode. Load the [switched replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11547%2F) or the [always-dynamic replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11548%2F) while the local servers are running.

To serve a fresh switched recording with the local frontend on port 8082, run the following in a separate terminal. The same command with port 11548 and directory `dynamic` serves its reference.

```bash
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  --server --replay --web-viewer 11547 target/mvp-robot/switched
```
