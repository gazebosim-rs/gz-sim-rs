<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Refining a folded sheet as it slides

A seven-column sheet pulled from the gripper under 1.30 N still landed at 267 ms and came to rest at 1.800 s. Its most expensive millisecond of contact took 11.32 s on the shared Ryzen 9 9950X3D, down from 18.53 s. On that interval both contact force and particle motion moved closer to the finer reference. [Control in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9897%2F) · [Guarded run](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9896%2F)

| Seven-column unguarded control | Previous solver | Revised solver |
|---|---:|---:|
| Controller and solver at postgrip tick 737 | 18.532 s | 11.322 s |
| Accepted slices within the final refinement | 361 | 196 |
| Maximum particle position error against the 512-slice reference | 21.55 µm | 15.46 µm |
| Mass-weighted RMS particle velocity error | 0.05995 m/s | 0.04630 m/s |
| Ground impulse vector error | 83.93 µN·s | 59.65 µN·s |
| Median over 1,800 postgrip steps | 28.30 ms | 31.87 ms |
| p95 over the same steps | 51.60 ms | 53.04 ms |

Both runs used one public millisecond and the same starting coordinates, velocities, material laws, gravity, jaw poses and floor. The ground impulse error is the Euclidean distance between the three components of the impulse on the support. The reference started with 512 intervals and locally subdivided to 768 accepted slices. A second 512-interval calculation used the older contact ordering. Its maximum particle position differed from the first reference by 0.041 µm and its reaction differed by 0.084 µN·s. Increasing the first reference grid from 256 to 512 changed those quantities by 2.75 µm and 12.72 µN·s. These comparisons measure convergence of this solver on this cut.

The ordinary solve still processes impacts in time order. After its uniform refinements fail, an eligible sheet first retries on its original clock. Actual geometric contacts take precedence. Among contacts whose entire path already clears the material thickness tolerance, the solver first treats the pair closing fastest. If this ordering alone cannot resolve the interval, it doubles the base resolution and bisects individual unresolved slices. The original subdivision and terminal ordering remain available if the finer attempt exhausts its budget. Every accepted solution uses at most 1,024 leaves for this fixture.

Both five-column runs and the guarded seven-column run reproduced their former particle and force traces. The seven-column control first changed at postgrip tick 737. Its greatest subsequent difference from the previous recording was 0.2593 mm at a sampled particle. The first landing impulse remained 1.75655 mN·s. Both seven-column runs passed 50 consecutive milliseconds of independent velocity, ground-load and unloaded-jaw checks at rest. gz-web-rs verified 401 frames, 11,228 independent particle positions and rewind in the two finer runs. Geometry checks found zero triangle intersections between the two panels or with the jaw cores across all recorded frames.

## Reproduce

Run the matched-state regression in release mode. It reconstructs the original SDF material limits, uses the recorded native poses of both jaws, advances the public solver, runs the older subdivision and recomputes the fine reference. It prints timings and errors separately for each solve. Numerical agreement is checked against the retained 28-particle reference. The timed fold example also writes one controller-and-solve duration in nanoseconds for every postgrip tick in each case's `step-cost-ns.json`.

```sh
export RUSTC_WRAPPER=
cargo test --release --locked -p gz-physics-xpbd --lib matched_supported_shell_refines -- --ignored --nocapture
cargo run --release --locked --no-default-features -p gz-sim-cli --example local_robot_foundations -- \
  fold-recovery --until-rest --columns 7 --load-mn 1300 target/my-sheet-slide
GZ_FOLD_RECOVERY="$PWD/target/my-sheet-slide" cargo test --release --locked -p gz-sim-scene \
  --test fold_breakaway_replay guarded_and_unguarded -- --ignored
cargo run --release --locked --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  --server --replay --web-viewer 9897 target/my-sheet-slide/mesh-7-1300-control/replay
```

[Machine-readable measurements](fold-slide-refinement.json) · [Recorded cut](../../../crates/gz-physics-xpbd/tests/fixtures/fold_slide_737.rs) · [Rust comparison](../../../crates/gz-physics-xpbd/tests/private/fold_slide_cut.rs) · [Both mesh resolutions and rest criteria](fold-rest.md)
