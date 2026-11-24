# Measuring a parcel that hits and leaves a sorting pad within one tick

A 10 g parcel approached the narrow pad at 4 m/s downward and left at 3.1625 m/s upward. The controller samples every 20 ms. The pad previously reported 0 N because the parcel was already airborne at the end of that interval. It now reports a **3.58125 N average reaction**, the 71.625 mN·s momentum change divided by 20 ms. The sensor still checks for load at the end of the tick before starting its release timer. [Measurements and coverage](receiving-impact.json)

The test checks the equal and opposite reaction, a neighboring parcel that misses, two fresh epochs, repeated reads, and the first ordinary step after impact. The parcel stayed within 7 × 10⁻¹⁵ m of a continuously stepped 200 µs reference at all five common timestamps through 80 ms. This comparison checks the public sensor against the actual solver impulse and the motion against the same fine native clock. The restitution coefficient was 0.8, gravity was zero, and the inspector saw no settled support. The sensor changes covered 22/22 executable lines and 46/47 regions. [Watch the 20 ms control loop](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10073%2F) · [Watch the fine reference at impact](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10074%2F)

The bundle contains both authored worlds, camera markers and complete native recordings. From the repository root, extract it into a new directory, then run the commands below. [Portable native replays and SDF](receiving-impact.tar.xz)

```sh
mkdir receiving-impact
tar -xJf docs/measurements/local-manipulation/receiving-impact.tar.xz -C receiving-impact
RUSTC_WRAPPER= cargo test --locked --no-default-features -p gz-sim-scene --test receiving_support
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli --example local_robot_foundations -- record receiving-impact/bounce.sdf 4 1 receiving-impact/new-run
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli --bin gz-sim-rs -- --server --replay --web-viewer 10073 receiving-impact/new-run/replay
```
