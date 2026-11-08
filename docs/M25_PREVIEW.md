<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Installed sensor-noise examples

These commands use ordinary immutable copies under `share/examples/` in the M25
Ubuntu 24.04 x86_64 preview. They need only the installed executables and ordinary
system runtime libraries from INSTALL.md. No development checkout, Cargo,
compiler, native physics SDK, remote assets, browser or new helper is required.
Keep the whole installation together and verify it before running:

```sh
preview_prefix="$HOME/gz-preview-m25-install/gz-preview-m25-linux-x86_64"
"$preview_prefix/bin/gz-preview" verify
```

Run each self-contained `default` world through the existing copied binary:

```sh
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m21_altimeter_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m22_air_pressure_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m23_magnetometer_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
"$preview_prefix/bin/gz-sim-rs" "$preview_prefix/share/examples/m24_cpu_lidar_noise.sdf" --physics-engine rapier --iterations 2 --step-ns 1000000
```

Each command loads and steps twice with a 1 ms step, then exits. This establishes
finite installed execution, not measured noise statistics, canonical packet
acquisition, trajectory accuracy or browser sensor visualization. Check its
actual exit and diagnostics. `gz-preview run` still starts the complete default
robot; it has no scene selector. Do not edit the manifest or fixtures to conceal
a verification error, and do not treat the two steps as a publication test.

| Copied example | Configured canonical topic / schema | Bounded behavior |
| --- | --- | --- |
| `m21_altimeter_noise.sdf` | `/m21/altimeter`, `gz.msgs.Altimeter` | 100 Hz world-Z position and offset-point velocity, Gaussian/quantized channels, noise-free load reference. |
| `m22_air_pressure_noise.sdf` | `/m22/air_pressure`, `gz.msgs.FluidPressure` | 100 Hz pressure after the default absolute-height atmosphere model; zero reference altitude, finite positive pressure, native variance `sqrt(stddev)`. |
| `m23_magnetometer_noise.sdf` | `/m23/magnetometer`, `gz.msgs.Magnetometer` | 100 Hz constant world field inverse-rotated into sensor axes, then distinct X/Y/Z noise. No geographic field model. |
| `m24_cpu_lidar_noise.sdf` | `/m24/scan`, `gz.msgs.LaserScan` | 100 Hz planar three-beam CPU scan, Gaussian mean/stddev only, minimum zero; finite hits noised before clipping, infinite misses unchanged. |

The schema names and topics describe existing demand-driven canonical Zenoh
interfaces, not a bundled message CLI or new observer. Use a compatible canonical
consumer in the matching partition for actual streamed measurements; acquisition
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

The matching release binds fresh binaries, manifest, selected notices and actual
installed-example results. Parent model/native/wire evidence remains explicitly
scoped reuse; no C++/Rust RNG-bit, native reset, cross-engine, performance or legal
verdict is implied by these commands. INSTALL.md covers the unchanged default
robot, finite drive commands, lifecycle opt-in, Pause/Reset and owned shutdown.
