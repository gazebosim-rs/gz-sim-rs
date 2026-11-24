<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Read sustained support from a local Bevy scene

A robot can now ask whether a particular receiver is carrying its payload. The detector binds the public link IDs in one world, measures signed normal force through the complete physics step, and requires sustained load on the selected face. The serial gripper uses this public API for its original tray and the sliding shelf. Its three withdrawal trajectories reproduced the previously qualified geometry byte for byte. [Play the public sensor controlling the robot](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9880%2F)

```rust
use std::time::Duration;
use gz_sim_scene::receiving_support::{ReceivingSupport, SupportCriteria};

let criteria = SupportCriteria::for_weight(0.2 * 9.81, Duration::from_millis(30));
let mut sensor = ReceivingSupport::new(sim.world(), parcel_link_id, shelf_link_id, criteria)?;
sim.step_exact(1)?;
let reading = sensor.sample(sim.world())?;
if reading.ready {
    // The controller may now command the jaws to open.
}
```

Sample after each native step and pose publication. In a Bevy application the detector can also be stored as a component. It reports the projected force, relative normal speed, receiver motion and sustained duration. A completed dwell supplies the time-weighted mean force. Duplicate reads do not accumulate time. Link retirement, an invalid binding, a skipped step, reset, receiver movement or excessive relative speed clear the old decision. Local applications can configure the up direction, face alignment, load thresholds, duration and speed limits.

| Independent trial | Result |
|---|---|
| 200 g resting on a static block and on a parked prescribed shelf | Both reported 1.962 N, including the four-slice prescribed solve. A 30.5 ms dwell first qualified on sample 31. Reversing native body creation order reversed the force vector on the opposite body and preserved the decision. |
| Move the loaded shelf sideways, then stop | The shelf continued supporting the weight. The detector withdrew permission while moving and accumulated a fresh dwell after parking. Selecting a different nearby surface reported zero load. |
| Oblique impact against a moving sphere in a single 120 ms step | Across 240 native slices the measured impulse on the payload was (0.523939, 0.199602, 0) N·s. The vector matched the change in momentum within 3.34 × 10⁻¹⁶ N·s. Contact had ended by publication, so the detector refused release. |
| Runtime-spawned parcel, retirement and reset | A spawned parcel qualified independently. Removing it invalidated its detector. Loaded-world reset cleared the old dwell, and a restored trajectory checkpoint required a fresh solve to supply its directional impulse. |
| Supported placement, interrupted release and deliberate dropping | The shared detector preserved the original control outcomes and the 27/28 ms continuous-support boundary. gz-web-rs verified 501 frames, 7,515 poses and shapes, and rewind for the new public-sensor recording. |

Counted time and force use the simulation clock. When the requested dwell contains a fractional first sample, the average includes only its overlapping duration. The rolling impulse integral uses compensated addition. Reading an ordinary pair goes straight to its colliders. A prescribed trajectory accumulates signed impulses during the same native pass as the scalar physics receipt. Existing trajectory checkpoint bytes still carry the scalar receipt, and the public detector waits for a new signed observation after restoration.

Release tests passed for seven public sensor cases, four trajectory cases and four robot control cases. Strict Clippy passed on all three targets. The optional timing sweep is documented in the earlier withdrawal measurements. Instrumented release runs covered 349 of 353 changed production lines (98.87%) and 564 of 579 regions (97.41%). The public sensor itself covered 229 of 230 lines and 363 of 367 regions.

## Reproduce

```sh
RUSTC_WRAPPER= cargo test --locked --release --no-default-features -p gz-sim-scene \
  --test receiving_support -- --nocapture
RUSTC_WRAPPER= cargo test --locked --release -p gz-physics-multifidelity \
  --features partition-checkpoint --test trajectory
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --example serial_place -- withdrawal target/my-public-sensor
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --bin gz-sim-rs -- --server --replay --web-viewer 9880 target/my-public-sensor
```

[Public detector](../../../crates/gz-sim-scene/src/receiving_support.rs) · [Independent scene tests](../../../crates/gz-sim-scene/tests/receiving_support.rs) · [Withdrawal timing measurements](serial-withdrawal.md)
