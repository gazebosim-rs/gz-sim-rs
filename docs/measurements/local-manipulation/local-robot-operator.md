<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Live serial robot with a universal wrist

The same serial transfer SDF that worked in an ordinary Bevy application failed when started through the live Gazebo server. Server controller admission read a single scalar position, while the wrist exposes two angular coordinates. Admission now checks the coordinate selected by the installed actuator. It still verifies the original physics owner, public joint identity, native joint and child link. The screw and wrist can share the real transport server.

A local Zenoh publisher sent opposite position targets to the wrist's second-axis topic before and after resetting the server. After 100 one-millisecond updates, the axis reached −0.0098531102 and +0.0098587143 radians. Both coordinates and their rates matched an independently advanced ordinary local simulation on all 200 updates. The server applied each transport command exactly once while paused.

The shipped operator executable then opened a fresh WebSocket connection for each command. Across three checked steps, every rendered robot and parcel pose matched the ordinary solver, including rotations. Stale ticks, stale epochs and a recovery request without a parked worker were refused. Play advanced simulated time and pause held the acknowledged state when a fresh operator reconnected.

The native browser recording contains 1,200 milliseconds of the ordinary authored startup motion. The gz-web-rs WebGPU viewer verified 8,200 rendered poses and shapes over 410 captured frames and rewound to time zero. [Watch the serial robot in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11344%2F)

![Serial robot and separate receiving shelf at 350 milliseconds](local-robot-operator.jpg)

The seven Ubuntu transport and controller tests passed, including the existing C++ peers, admission and reset controls. The two native CLI tests passed. Instrumented execution covered all five changed executable lines and all 14 changed regions. The normal local command client reached 109 of its 146 regions, up from zero in the preceding combined headless report.

To drive the ordinary local SDF from two terminals, use a free local viewer port:

```sh
RUSTC_WRAPPER= cargo run --locked -p gz-sim-cli --bin gz-sim-rs -- \
  examples/worlds/local_robot_serial_transfer.sdf --physics-engine multifidelity \
  --server --web-viewer 9002
```

```sh
RUSTC_WRAPPER= cargo run --locked -p gz-sim-cli --bin gz-sim-convoy-control -- --port 9002 status
RUSTC_WRAPPER= cargo run --locked -p gz-sim-cli --bin gz-sim-convoy-control -- --port 9002 step 0 0
```
