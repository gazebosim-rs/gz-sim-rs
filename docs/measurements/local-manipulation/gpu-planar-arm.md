# Two-joint arms on a GPU

A named Vulkan GPU advances 2,048 independently driven, fixed-base arms in one Bevy world. Each arm has two revolute joints, massive rigid links, a rigid point payload, gravity and torque-limited servos. Both joints change targets after 300 ms and continue for another 300 ms. The payload remains attached for this experiment. Finger contact, grasp acquisition and arbitrary SDF robot import remain future work for this GPU profile.

| Tip load (kg) | Previous first-order step vs. fine RK4 (mm) | Midpoint vs. fine RK4 (mm) | Midpoint vs. fine native Rapier (mm) |
|---:|---:|---:|---:|
| 0 | 2.023 | 0.0489 | 0.2013 |
| 0.10 | 1.636 | 0.0240 | 0.1533 |
| 0.25 | 1.450 | 0.0196 | 0.1452 |

Each value is the greatest tip-position error across all 600 ticks and eight sampled arms of that load. The independent Cartesian reference sums the mass, gravity and centripetal acceleration of both link centers and the payload through their Jacobians. The fine run uses eight fourth-order Runge–Kutta steps per public tick. Native Rapier independently models two reduced-coordinate hinges with both link inertias, combined payload mass properties and equal and opposite joint torques. The native run uses eight 125 µs steps and zero passive hinge damping, matching the explicit damped servo. Its largest difference from fine RK4 was 0.2124 mm. After the first comparison, the example tightened its maximum allowed errors to 0.1 mm against fine RK4 and 0.5 mm against fine Rapier.

The Rust-authored f64 shader solves the coupled two-by-two inertia matrix twice per 1 ms tick. A midpoint velocity update and unit-phasor rotations retain second-order accuracy without device trigonometric instructions. Across all 1,228,800 lane ticks the independent same-clock Cartesian calculation differed by at most 1.25 × 10⁻¹⁵ m at the tool. Both actuators saturated, reaching 8 and 2.8 N m. The most mobile sampled tip traveled 0.323 m from its starting point. A reset, the same command reversal and a second 600-tick run reproduced every final lane byte.

On the RTX 5090, the midpoint shader took 10.72 µs p95, versus 8.19 µs for the first-order shader. Dense Bevy tick p95 was 225.56 µs, including the device fence, decoding and a 458,784-byte readback. First-order dense p95 was 157.37 µs. The independently coded serial Cartesian midpoint reference took 336.30 µs p95 on this host. These are warm adapter measurements from a shared development machine. The nine visible lanes cover all three payloads.

[Open the verified native replay in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9834%2F) · [Viewer capture](gpu-arm-view.jpg) · [Timing and trajectory measurements](gpu-planar-arm.json)

To regenerate with an f64-capable Vulkan GPU, give its adapter name and a new output directory.

```sh
RUSTC_WRAPPER= cargo run --release --locked -p gz-physics-gpu --example planar_arm -- \
  'NVIDIA GeForce RTX 5090' artifacts/gpu-arm-rerun
```

The output contains a native replay, the source world points for all 61 frames, and the comparison measurements.

The native viewer audit checked all 549 sampled arm states, all 2,196 visible body poses and shapes, and rewind to time zero. The release and instrumented runs reproduced the native replay and frame evidence byte for byte. Four focused tests, including a hardware command/reset check, and strict Clippy passed. Exact changed production and example coverage was 561/566 executable lines (99.12%) and 1,230/1,252 regions (98.24%).
