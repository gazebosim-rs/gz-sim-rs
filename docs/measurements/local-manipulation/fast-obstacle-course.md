<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Fast links at a thin obstacle

Three 2 kg, 60 mm-radius robot tips move at 100 m/s toward separate 12 mm-thick fixed rails. The orange tip hits the flat face, the yellow tip overlaps the end by 5 mm, and the green tip has 5 mm of clearance. Each lane uses the same initial pose and velocity across all step sizes, in a gravity-free world with frictionless, inelastic surfaces. The course runs for 20 ms. [SDF](../../../examples/worlds/local_fast_obstacle.sdf) · [Journal](../../../examples/replay/local_fast_obstacle.json) · [Interactive gz-web-rs recordings](contact-course-view.md)

| 100 µs public step | Head-on | Grazing | Clear |
|---|---:|---:|---:|
| Analytic touch time | 7.840 ms | 8.2002084 ms | — |
| End of first tick with a positive impulse | 7.900 ms | 8.300 ms | — |
| End of first positive tick at 5 µs | 7.840 ms | 8.205 ms | — |
| Cumulative normal impulse | 200 N·s | 79.9305312 N·s | 0 N·s |
| Continuous 5 µs run | 200 N·s | 79.9305254 N·s | 0 N·s |
| Maximum center error vs analytic flight | <1 nm | 37.3 nm | <1 nm |
| Maximum sampled penetration | 0 mm | 0 mm | 0 mm |

No lane diverged from the 5 µs run by more than 1 µm at a matched timestamp. The clear tip remained at least 5 mm from its rail and finished at full speed. The grazing tip left the rail at approximately (84.028, 36.635, 0) m/s. Its peak angular speed was 1.93 × 10⁻¹³ rad/s, consistent with a frictionless impact through the center of a sphere. A 10 ms control stepped past first impact. It produced 10.66 mm of sampled head-on penetration and a 164.76 mm grazing trajectory error. The authored runnable profile uses 100 µs.

## What changed

Before this fix, the 100 µs solver delivered 112.113 N·s to the tip that cleared the rail, and 124.939 N·s to the grazing tip. Even the original 5 µs control falsely struck the clear tip. A speculative contact plane reused near the rounded edge could cross the free path. Rebuilding the initial geometry removed the false clear strike but still assigned the early contact normal to the grazing tip. The solver now checks a cold, separated CCD sphere against the fixed box along its actual swept path, uses the normal at first touch and applies the impulse along the sphere's radial line. Native handling remains active for gravity, external loads, damping, joints, multiple possible contact neighbors and warm contact data. [Regression tests](../../../crates/gz-sim-scene/tests/local_fast_obstacle.rs)

The independent analytical check computes where the sphere center first reaches one radius from the nearest point of the box. Removing the incoming normal component there gives both outgoing velocity and normal impulse. The benchmark compares trajectories at every common end-of-tick time to both the analytic solution and the 5 µs simulation. The specified limits are 2 mm center error, 0.5 mm sampled penetration, 5% impulse error and one candidate tick of onset difference. The clear lane must show zero impulse and finish within 1 µm of free flight. Unexpected positive impulses between different lanes fail the run.

## Cost and reproduction

Each p95 measures a complete release-mode Bevy physics step on an AMD Ryzen 9 9950X3D. Each grid ran once for its numerical trace, then 21 times for timing in rotating order. Timings exclude setup, diagnostic sampling, recording and rendering. The final column scales the public-step p95 by the number of steps in 1 ms to show the cost of running the finer clocks. It is a scaled single-step statistic.

| Public step | Timed steps | p95 per step | Scaled cost per simulated ms |
|---|---:|---:|---:|
| 100 µs | 4,200 | 5.69 µs | 56.9 µs |
| 10 µs | 42,000 | 4.37 µs | 437 µs |
| 5 µs | 84,000 | 4.31 µs | 862 µs |
| 10 ms control | 42 | 117.12 µs | 11.71 µs |

Run the installed local replay executable from this checkout, then reproduce the multi-grid measurements in Rust.

```bash
gz-sim-local-replay examples/worlds/local_fast_obstacle.sdf \
  examples/replay/local_fast_obstacle.json --trace fast-obstacle.jsonl
RUSTC_WRAPPER= cargo run --release --locked --no-default-features \
  -p gz-sim-scene --example fast_obstacle_course -- target/fast-obstacle-results
```

The archive contains the initial failure and final pass, with per-tick poses, velocities, impulses and signed clearances on all four grids. Final traces also include angular speed. The baseline used the native solver from `be3bf37`. [Raw comparisons and traces](fast-obstacle-course.tar.xz)
