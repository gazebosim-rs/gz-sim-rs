<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# When a moving parcel first loads the gripper

A small gap lets the jaw move before impact. A total impulse at the end of a controller tick can combine several separate touches. The Rust gripper study can now measure each uninterrupted load on either finger, its native time range, the signed impulse and its angular impulse about the parcel center. The ordinary 1 ms controller and all six body trajectories reproduced the uninstrumented run exactly in the regression. [Native step observations and regression coverage](jaw-gap-pulses.json)

With a 350 µm gap at the right finger, the 2 µs native clock resolved two pulses, 25.30 and 16.53 mN·s. At 125 ns it resolved three, 23.13, 7.24 and 13.18 mN·s. At 25 ns it resolved two, 24.76 and 17.55 mN·s, starting at 492.950 and 501.875 µs. The nominal straight-line crossing time is 466.667 µs. The moving finger changes the actual impact time. Comparing only total impulse would hide the short release and the sensitivity to the native clock.

Five gaps from 50 µm to 1.2 mm were tested on both fingers. The delayed 1.2 mm approach exposed a second problem. On the second controller tick the parcel still had zero contact impulse in the ordinary and experimental runs. With 64 native calls, eight temporal iterations and eight fixed-time solver passes, one arm link was already moving at 406 m/s. The existing 500-call setting kept link speeds below 0.34 m/s at the same instant. Both native replays show the same incoming state and the following impact. [Ordinary approach](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10082%2F) · [Failed experimental setting](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10081%2F)

The presets are measured against independent continuous 125 ns steps over 24 ms. All ten routes also ran continuously at 25 ns. With the existing 500-call setting, the worst differences across the ten cases were 5.85% in total impulse, 1.17 mm in position and 1.83° in rotation. The experimental settings are comparison inputs. The unstable case identified the world-mounted carriage slider as a candidate for finite joint stabilization. A 100 kHz setting on that rail subsequently stabilized both delayed approaches. [Guide qualification and two corrected replays](mounted-rail.md)

Run the pressure history or extract the two complete SDF worlds and native recordings. Each recording passed 25 WebGPU frames and 500 pose-and-shape checks. [Portable worlds and recordings](jaw-gap-pulses.tar.xz)

```sh
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli --example serial_impact_inertia -- \
  --divisor 1.75 --right --gap-um 350 --substeps 8000 --ticks 3 --impact-pulses
```
