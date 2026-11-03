<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M4: demand-activated IMU delivery

Candidate implementation, not accepted. The scope budget is three working days
including independent source review and mandatory verification. M1–M3 remain
the accepted baseline. Reuse the existing sensor model, SDF loader, Rapier state,
schedules, lifecycle and direct upstream Rust Zenoh publication; no separate
sensor runtime, transport facade or Messages functionality is introduced.

## Runnable example

Use the accepted viewer setup in [MVP_BROWSER.md](MVP_BROWSER.md). The candidate
world contains a falling two-kilogram box, static ground and one centered IMU
rotated 90 degrees about local X. The SDF physics step is 1 ms and the IMU rate
is 100 Hz, with explicit orientation output and no noise. Its canonical topic
is `/mvp/imu` and frame is `imu_box/body/imu`.

```sh
GZ_PARTITION=mvp-imu cargo run --locked --no-default-features \
  -p gz-sim-cli --bin gz-sim-rs -- examples/mvp_imu.sdf \
  --physics-engine rapier --server --web-viewer 9006
```

Open `http://127.0.0.1:8081/?backend=primitive&ui=full&url=ws://127.0.0.1:9006/`.
The existing browser visualizes the body and controls the real simulation;
canonical IMU samples use the existing Zenoh boundary in [ZENOH_IO.md](ZENOH_IO.md).
This command is a candidate launch recipe, not a claimed observed run.

## Sensor and lifecycle contract

Specific force uses m/s² and angular velocity rad/s in the configured sensor
axes. Gravity is removed from free-fall acceleration. Acquisition follows the
existing end-step integer timestamp schedule; the first admitted sample and
subsequent phase-aligned samples have explicit regression expectations. Paused
simulation does not advance acquisition. Core and loaded-world reset clear old
samples and establish a new epoch with the construction state restored.

Loaded sensor/model requirements own IMU callbacks and buffers. Removing the
last loaded consumer retires those resources; a world without an IMU has none.
Subscriber-interest suppression and explicit deactivation are distinct from
structural removal: a still-loaded sensor can retain its registration/history
while acquisition is inactive. Reuse the existing interest policy and lifecycle
rules; do not weaken them to make a demonstration publish.

Focused tests use the same SDF input for analytic sensor-axis/unit controls,
exact sample counts/timestamps, pause/reset, loaded-consumer teardown and actual
upstream Zenoh delivery decoded by maintained generated protobuf bindings.
Existing lifecycle, rate and canonical-wire tests remain retained. Source review
and the complete regression/documentation batch must finish before the single
consolidated mandatory verification pass. No qualification result is claimed.

Other sensor families, general noise-model parity, new dynamic plugins and broad
Gazebo compatibility remain deferred. Keep strict owned and changed-line gates,
all retained assertions and Apache-2.0/OSRF attribution.
