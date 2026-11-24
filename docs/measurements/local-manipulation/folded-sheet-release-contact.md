<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# The folded sheet lands after the gripper opens

In both fine runs the two fingers released the sheet near 2.67637 s. It fell freely for about 13 ms before the table carried load. At 500 exchanges per millisecond the first table impulse was 0.18260 mN·s upward at 2.689298 s. At 1,000 exchanges it was 0.18202 mN·s at 2.689245 s. By 2.710 s their largest particle separation was 0.1996 mm, including 0.1942 mm sideways. The recordings show the same release and an observable difference in the subsequent table reaction. [500 exchanges in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9993%2F) · [1,000 exchanges in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9994%2F) · [Numerical results](folded-sheet-release-contact.json)

Each run cloned its complete incoming cloth state at 2.675 s. Four branches then followed the same measured rigid jaw trajectory through 2.710 s. At each native endpoint, the one-step branch reproduced the independent Bevy simulation's particle coordinates, velocities and boundary reactions exactly. This checked 560, 17,500 and 35,000 native intervals at the three rates. All branches shared their own run's initial conditions. The two complete Bevy runs accumulated their own histories before the cut.

| Native exchanges per ms | Largest material-only difference through first table contact, one vs eight cloth steps | Largest difference over the 35 ms window | Net native upward table impulse |
|---:|---:|---:|---:|
| 16 | 7.10 µm | 156.29 µm | 2.807 mN·s |
| 500 | 0.23 µm | 36.81 µm | 2.741 mN·s |
| 1,000 | 0.11 µm | 95.48 µm | 2.847 mN·s |

The late differences also appeared when holding the jaw trajectory fixed. At the finest rate, changing from one to eight material steps changed the signed sideways table impulse from +49.89 to −52.06 µN·s. This identifies the release and landing as a problem for the cloth and table contact model. The eight-step branch still lacks an independent convergence witness. The 0.1 mm full-trajectory acceptance criterion remains open. Both native replays checked all 161 frames and 1,932 measured particle positions, including rewind to the first endpoint.

A separate regression found that a point landing exactly on the mathematical skin of a plane or sphere could retain its incoming velocity until the next tick. The solver now publishes the inelastic impulse at the landing endpoint and leaves tangential motion, separating motion and genuine near misses unchanged. The regression checks the 2 N·s momentum transfer and zero duplicate impulse on the next tick. All 212 ordinary XPBD release tests passed. The full damped gripper at 16 exchanges retained its original particle, force and archive bytes. LLVM covered 42/43 changed production lines and 106/109 regions across the endpoint fix, capture and comparison tools. [Endpoint solver](../../../crates/gz-physics-xpbd/src/collision.rs) · [Contact regression](../../../crates/gz-physics-xpbd/tests/solver.rs)

To reproduce, run the viscous-pad example with 500 exchanges, `--inspect-native-tick 1337501 --inspect-native-count 17500`. For 1,000 exchanges use `--inspect-native-tick 2675001 --inspect-native-count 35000`. Each writes signed table impulses at every native endpoint and all material positions and velocities at each replay sample. The normal `sheet-jaw-convergence` command also reports the first out-of-tolerance time and particle and the location of the maximum error. [Example command and viewer setup](viscous-gripper-pads.md)
