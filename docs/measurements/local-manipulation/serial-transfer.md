<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Carrying a free parcel to a fixed shelf

The serial gripper lifted a 200 g parcel 126.07 mm, carried it to a shelf centered 460 mm from pickup, and set it down. The shelf supported the parcel continuously after release. Its center settled another 0.013 mm as the gripper opened and retracted. [Watch the transfer in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9884%2F)

| Measurement | Transfer | Open jaws | Missed shelf |
|---|---:|---:|---:|
| Rail command | 460 mm | 460 mm | 180 mm |
| Parcel lift before travel | 126.07 mm | −0.055 mm | 126.07 mm |
| Parcel travel at end of rail command | 462.44 mm | −0.021 mm | 180.98 mm |
| Travel solves carried by both pads, clear of both supports | 4,001 / 4,001 | 0 / 4,001 | 4,001 / 4,001 |
| Maximum movement of parcel center relative to palm during travel | 0.450 mm | 479.2 mm | 0.383 mm |
| Shelf-qualified opening | 14.074 s | none | none |
| Final mean vertical load on destination | 1.962 N | 0 N | 0 N |
| Final mean vertical load on pickup tray | 0 N | 1.962 N | 1.887 N |
| Final center sideways from pickup | 460.16 mm | −0.021 mm | 179.95 mm |

The same fixture runs all three trials at a 1 ms step. A horizontal prismatic rail carries the original screw lift, universal wrist and two frictional jaws. A bounded 80 N force controller follows a four-second quintic position trajectory using measured rail position and velocity. A loaded body has up to 2.44 mm of travel overshoot at the end of this command. Both supports are fixed. Horizontal motion comes from the rail drive, and the jaws carry the parcel through solved contacts.

The gripper uses the public receiving-support sensor and both supports' signed forces during final descent. The controller requires 30 ms of sustained load on the destination, a mean of at least 94% of parcel weight and a relative vertical speed at most 0.01 m/s. At opening, the shelf averaged 1.8443 N and the relative speed was 0.000000762 m/s. After retraction the shelf carried the full weight with zero pad or tray load. In the missed-target run the pickup tray qualified for 5,992 ticks while the gripper kept its closing force. In the open-jaw run the empty carriage completed the full route while the parcel stayed at pickup. [Open-jaw replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9885%2F) · [Missed-shelf replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9886%2F)

Loaded-world reset reproduced every reported number and all 451 sampled world-pose frames, bit for bit, in all three trials. Six release tests covered the new route, controls and failure handling. The existing serial grasp regression also passed on its original SDF. Strict Clippy passed. Instrumented runs covered 192 of 193 executable lines (99.48%) and 404 of 423 regions (95.51%) in the new Rust example. gz-web-rs played and rewound each native recording, checking all 451 frames and 9,020 visual poses and shapes per trial.

## Reproduce

Run from the repository root. Each recording requires a fresh directory. Select `transfer`, `open-jaws` or `missed-shelf` as the trial.

```sh
RUSTC_WRAPPER= cargo test --locked --release --no-default-features \
  -p gz-sim-cli --example serial_transfer -p gz-sim-scene --test local_robot_serial_grasp
RUSTC_WRAPPER= cargo run --locked --release --no-default-features \
  -p gz-sim-cli --example serial_transfer -- transfer target/my-transfer
RUSTC_WRAPPER= cargo run --locked --release --no-default-features \
  -p gz-sim-cli --bin gz-sim-rs -- --server --replay --web-viewer 9884 target/my-transfer
```

[Authored robot and fixed workcell](../../../examples/worlds/local_robot_serial_transfer.sdf) · [Rust control, recording and reset checks](../../../crates/gz-sim-cli/examples/serial_transfer.rs) · [Receiving-support sensor](receiving-support.md)
