<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Installed sensor and pose examples

These commands use ordinary immutable copies under `share/examples/` in the M29
Ubuntu 24.04 x86_64 preview. They need only the installed executables and ordinary
system runtime libraries from INSTALL.md. No development checkout, Cargo,
compiler, native physics SDK, remote assets, browser or new helper is required.
Keep the whole installation together and verify it before running:

```sh
preview_prefix="$HOME/gz-preview-m29-install/gz-preview-m29-linux-x86_64"
"$preview_prefix/bin/gz-preview" verify
```

Run each self-contained `default` world through the existing copied binary:

```sh
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m21_altimeter_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m22_air_pressure_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m23_magnetometer_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m24_cpu_lidar_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m26_air_speed.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m27_pose_publisher.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m28_air_speed_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
```

All seven commands load and step twice with a 1 ms step, then exit. This establishes
finite installed execution, not measured noise statistics, canonical packet
acquisition, trajectory accuracy or browser sensor visualization. Check its
actual exit and diagnostics. `gz-preview run` still starts the complete default
robot; it has no scene selector. Do not edit the manifest or fixtures to conceal
a verification error, and do not treat the two steps as a publication test.
At 100 Hz the PosePublisher first positive-time frame is at 10 ms, beyond this finite run.

| Copied example | Configured canonical topic / schema | Bounded behavior |
| --- | --- | --- |
| `m21_altimeter_noise.sdf` | `/m21/altimeter`, `gz.msgs.Altimeter` | 100 Hz world-Z position and offset-point velocity, Gaussian/quantized channels, noise-free load reference. |
| `m22_air_pressure_noise.sdf` | `/m22/air_pressure`, `gz.msgs.FluidPressure` | 100 Hz pressure after the default absolute-height atmosphere model; zero reference altitude, finite positive pressure, native variance `sqrt(stddev)`. |
| `m23_magnetometer_noise.sdf` | `/m23/magnetometer`, `gz.msgs.Magnetometer` | 100 Hz constant world field inverse-rotated into sensor axes, then distinct X/Y/Z noise. No geographic field model. |
| `m24_cpu_lidar_noise.sdf` | `/m24/scan`, `gz.msgs.LaserScan` | 100 Hz planar three-beam CPU scan, Gaussian mean/stddev only, minimum zero; finite hits noised before clipping, infinite misses unchanged. |
| `m26_air_speed.sdf` | `/m26/air_speed`, `gz.msgs.AirSpeed` | 100 Hz ideal sensor-point/frame-correct differential pressure and temperature; zero wind and default atmosphere. |
| `m27_pose_publisher.sdf` | `/m27/pose`, `gz.msgs.Pose_V` | 100 Hz model and two link local poses in an unchanged articulated fixture, with per-pose frame headers and absent outer header. |
| `m28_air_speed_noise.sdf` | `/m28/air_speed`, `gz.msgs.AirSpeed` | 100 Hz Gaussian/quantized pressure noise in native pre-100 units; noise-free temperature and valid negative pressure. |

The schema names and topics describe canonical Zenoh interfaces (sensor
acquisition is demand-driven; PosePublisher acquisition is not). No message CLI
or new observer is bundled. Use a compatible canonical consumer in the matching partition for actual streamed measurements; acquisition
headers, frame IDs and sequence follow the established sensor contracts. The
finite commands above are not a substitute for those separate wire proofs.
The c404 browser has no numeric widgets for these noise channels.

Altimeter, pressure and magnetometer admit only finite Gaussian parameters,
nonnegative deviations/precision and signed finite means; `none` is default-only.
Native application has `dt=0`, so dynamic bias does not evolve with sample time.
Private seeded streams, no-demand/pause/non-due no-draw behavior and deterministic
Rust reset reconstruction retain their separate source contracts. Failed noisy
acquisitions cannot publish fabricated or stale valid measurements.

CPU lidar deliberately differs: only Gaussian mean and nonnegative standard
deviation are admitted. Bias, dynamic bias and precision stay default;
Gaussian-quantized is refused. Every Gaussian declaration requires minimum zero,
even at default/near-zero inactive settings; noise-free existing nonzero minima
are unchanged. The native near-zero activation boundary is preserved. Finite
noise overflow faults and clears the scan rather than becoming a clipped return.
Existing 2–128 beam, planar, one-sensor/Rapier, metadata and visibility restrictions
remain. This package adds no GPU scan, point cloud, intensity model or new sensor.

AirSpeed's signed sensor-X quadratic pressure retains native float atmosphere,
float conversion and final multiplication by 100. Its noise mean and deviation
are in the pre-100 pressure units: mean 0.1 changes the published pressure by
about 10 Pa, subject to float rounding. Noise is not applied to the published
pressure or temperature. Gaussian/quantized metadata must be finite, with
nonnegative deviations and precision; the existing precision threshold and
half-away-from-zero rounding apply. Zero-elapsed dynamic bias does not evolve.
An invalid acquisition cannot emit a stale valid packet.

The PosePublisher finite command checks the full sorted CLI world-link output:
base `(1,2,3)` and tip `(1+0.8*cos(0.6),2+0.8*sin(0.6),3)` in its zero-gravity,
zero-effort world. Its separate canonical packets contain parent-relative local
poses, not those CLI world positions. The native publisher's unordered records
are compared by name in feature evidence; no native wire order is invented.
Paused ticks never publish; subscriber absence does not stop configured pose
acquisition. The browser still shows only the default robot when launched with
`gz-preview run`, and no example acquires new lifecycle editing support.

The matching release binds fresh binaries, manifest, selected notices and actual
installed-example results. Parent model/native/wire evidence remains explicitly
scoped reuse; no C++/Rust RNG-bit, native reset, cross-engine, performance or legal
verdict is implied by these commands. INSTALL.md covers the unchanged default
robot, finite drive commands, lifecycle opt-in, Pause/Reset and owned shutdown.
