<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Pick up a parcel, clear the barrier and place it on the shelf

The local CPU preview includes `serial_transfer`, the Rust controller for the serial arm. On the 1 kg run the screw lifted the parcel 125.97 mm, the carriage carried it past the barrier with 18.84 mm clearance, and the jaws opened after 30 ms of continuous shelf support. The parcel settled 0.129 mm from the shelf center in the horizontal plane. Its full weight of 9.81 N rested on the shelf. [Authored workcell](../../../examples/worlds/local_robot_serial_obstacle.sdf) · [Per-body clearances, forces and final poses for all five runs](arm-placement-course.json)

Run these commands from an extracted package. Choose a fresh output directory for each recording. The program prints a JSON report and saves the measured scene frames for playback.

```sh
./bin/serial_transfer clear-obstacle placed --parcel-kg 1 --grip-friction 0.85 > placed.json
./bin/serial_transfer clear-obstacle rejected --parcel-kg 0.2 --grip-friction 0 > rejected.json
./bin/gz-sim-rs placed --server --replay --web-viewer 9002
```

The same command can run from source with `RUSTC_WRAPPER= cargo run --release --locked --no-default-features -p gz-sim-cli --example serial_transfer -- clear-obstacle placed --parcel-kg 1`. The workcell has a 460 mm rail, screw-driven lift, universal wrist and two sliding fingers. Gravity and friction act on a free parcel. A load check on both pads and a sweep of the parcel and all moving links must clear before travel. Opening requires measured load and low relative speed at the receiver throughout the support dwell. Each trial ran for 18 seconds at a 1 ms step.

| Parcel | Grip friction | Lift at dispatch | Result |
|---:|---:|---:|---|
| 0.200 kg | 0.85 | 126.07 mm | Released at 14.074 s, final shelf load 1.962 N |
| 1.000 kg | 0.85 | 125.97 mm | Released at 9.195 s, final shelf load 9.810 N |
| 1.245 kg | 0.85 | 114.04 mm | Cleared the barrier by 6.65 mm and released at 8.892 s, final shelf load 12.213 N |
| 1.2475 kg | 0.85 | 105.86 mm | Sweep predicted obstruction at dispatch, parcel remained in the jaws at pickup |
| 0.200 kg | 0 | −0.055 mm | Failed grasp at dispatch, the parcel stayed on the tray, which carried 1.962 N |

No robot or parcel delivered a normal impulse to the barrier in these five runs. Each successful carry had both pads loaded for all 4,001 traverse observations. The controller withheld travel and opening in both unsuccessful runs. In the frictionless run the rail was 0.009 mm from pickup at the scheduled arrival time. The largest sampled success was 1.245 kg at friction 0.85. Other geometries, materials, trajectories or positional loads can change the usable mass range. [Earlier 26-case load study](serial-payload.md)

The mass option scales all six entries of the authored center-of-mass inertia tensor by the same ratio, keeping geometry, inertial pose and contact material fixed. The friction option changes the two ODE sliding coefficients on the parcel box and each finger’s contact face. Both options work on the built-in route or on a compatible `--world` with the named collision faces and explicit inertial/ODE properties. The CLI rejects invalid inputs before creating a recording.

The installed-executable regression runs the 1 kg placement, the frictionless rejection and an invalid mass through the public command. Loading the supplied workcell via `--world` reproduced the built-in 1 kg measurements and recording byte for byte. Focused LLVM coverage on changed executable example code reached 81/81 lines and 221/222 regions (99.5%). The native recordings contain 452 frames each.

```sh
GZ_TEST_LOCAL_CPU_INSTALL=/absolute/path/to/extracted/package RUSTC_WRAPPER= \
  cargo test --release --locked --no-default-features -p gz-sim-cli \
  --example serial_transfer installed_workcell -- --ignored
```
