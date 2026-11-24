<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Keep the arm on its rail while a parcel approaches

A 200 g parcel starts 1.2 mm from a moving gripper. With an experimental contact budget, the simulator accelerated a robot link to 406 m/s on the second millisecond, before the parcel touched either finger. Giving the carriage rail a finite constraint frequency of 100 kHz brought that speed to **0.327 m/s**. The parcel hit on the following tick, and either finger could take the load. [Watch the corrected left approach](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10084%2F) · [Right approach](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10085%2F) · [Original failure](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10081%2F)

The world plugin selects the prismatic joint by its scoped name. Its frequency controls how quickly the solver corrects motion away from the guide and its active position stops. The 100 kHz value is a numerical setting tested on this arm. A physical spring rate would require an independent measurement. Construction and reset retain the authored damping, motor and speed limit. The standard source solver uses a nominal 1 MHz hard constraint. In a separate fixture the guide reached both ±20 mm travel stops under 20 N of sideways force and its own 9.81 N weight. Its lateral drift was below 0.000001 mm.

```xml
<plugin name="gz::sim::systems::MountedPrismaticGuide" filename="gz-sim-mounted-prismatic-guide-system">
  <natural_frequency_hz joint="default::gripper::travel">100000</natural_frequency_hz>
</plugin>
```

The independently initialized continuous 25 ns runs used the same authored guide and actuator commands. Through 24 ms, the corrected left and right approaches stayed within **0.351 and 0.296 mm** of every link and parcel position. Their maximum rotation errors were 0.187° and 0.747°, and total jaw impulse errors were 2.185% and 0.173%. The two missed approaches produced zero jaw impulse. Explicitly selecting the native 1 MHz frequency reproduced both original trajectories exactly. A separate three-robot scene allowed distinct rails to respond differently to the same load at chosen frequencies, and swapping the frequencies swapped their deflections.

## Cost and remaining accuracy limits

Each 24 ms approach uses the original actuator clock. With the existing 500-call impact setting, the left and right impact ticks cost 59.56 and 56.31 ms at p95. With the finite guide, 64 native calls, eight temporal iterations and eight internal solver passes, they cost 15.10 and 13.84 ms at p95. Each setting and jaw ran 15 times in three reordered blocks on logical CPU 18. The landing remains slower than a 1 ms wall-clock deadline. [Timings, physical controls and per-gap errors](mounted-rail.json)

The first pulse and the post-impact trajectory depend on the contact grid. From a 50 µm gap the faster setup still differed from the continuous 25 ns guide by 10.01% in the left impulse and 8.70% in the right. At 350 µm on the right the original continuous 125 ns and 25 ns controls themselves differ by 1.91° over 24 ms. The smaller gaps remain under investigation. [Individual pulses and fine controls](jaw-gap-pulses.md)

## Run the arm

```sh
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli --example serial_impact_inertia -- \
  --divisor 1.75 --automatic --gap-um 1200 --substeps 64 --impact-iterations 8 --impact-pgs 8 \
  --guide-hz 100000 --write-world target/mounted-rail.sdf --replay target/mounted-rail-view
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  target/mounted-rail.sdf --physics-engine multifidelity --iterations 24
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  --server --replay --web-viewer 10184 target/mounted-rail-view
```

With gz-web-rs running on port 8082, the new recording is available on port 10184. Add `--right` to the example for the reflected arrival. [Open your local recording](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10184%2F)

The ordinary simulator reproduced all six final body positions in each case exactly. Both native recordings passed 25 WebGPU frames and 500 pose-and-shape checks each. The changed production code covered 128/128 executable lines and 199/203 regions, and each affected crate exceeded 95% on both measures. [Both portable SDF worlds and native replays](mounted-rail-replays.tar.xz)
