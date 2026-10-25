# Shared-SDF native IMU acquisition

SLICE-01's next reference control runs the same `headless_imu.sdf` bytes through
native Server + Physics + Imu and the Rust scene loader. Native backend/system
selection uses ServerConfig rather than changing the SDF's entity or sensor
parameters. The fixture contains static box ground, one spherical body, gravity,
a noise-free IMU, a 1 ms step and real-time factor zero.

Both native DART and TPE produce exactly 100 messages at 1..100 ms. The Rust test
compares every timestamp, orientation, angular velocity and specific-force
component to those observations. Rust TPE has native TPE's supported constant
motion semantics. Rust Rapier free-fall measurements are cross-engine physical
checks with a declared 1e-9 sensor-value bound over this 100 ms horizon; they are
not same-engine trajectory parity. Neither run reaches ground contact here.
The full selected-fixture assertion matrix and equal-work performance report
remain required for P2 acceptance.

## Startup and sample integrity

Native Imu skips acquisition unless a sensor is due and has subscribers.
The C++ control registers a process-local subscriber first, starts the server
paused, observes the matching publisher, then queues unpause through the native
world-control service. There is no fixed discovery sleep. All samples, including
extra or unexpected zero-time samples, count toward validation.

This startup protocol matters. Native finite Run counts advancing iterations,
so a blocking initial `Run(true, 1, true)` does not finish while paused. A direct
SetPaused call during initialization can also change the pause flag of an
in-flight zero-time PostUpdate. Exploratory failed runs are retained locally;
the control uses queued playback at iteration boundaries and requires exactly
the intended sample times. The Rust initial paused tick also produces no sample,
and a paused tick after acquisition preserves its final sample/time.

`planning/reference/headless-imu.json` records both complete native sample sets,
the shared fixture hash, source/compiler/binary/build hashes and loaded simulator,
Physics, Imu and backend library hashes. Installed dependencies and the existing
native build remain qualified P0 provenance. This is a functional reference with
a process-local subscriber; it is not a network-throughput or speedup result.
The nine existing actual C++ client scenarios remain required separately.

Reproduce with:

```sh
python3 scripts/probe-headless-imu.py --sim-source /path/to/gz-sim \
  --physics-source /path/to/gz-physics --sim-build /path/to/native-build \
  --dependency-prefix /path/to/native-prefix --work-dir /tmp/headless-imu \
  --output artifacts/headless-imu-proof.json
```

Build the native `gz-sim-imu-system` and Physics targets first. Rust comparison is
`cargo test --locked -p gz-sim-scene --test headless_imu`. No production Rust,
existing assertions, fixtures, tolerances or coverage policy changed. Independent review reproduced all 200 native samples and the selected Rust
comparison; changing sample 38 by 0.001 failed its exact assertion. The original
staged gate failed in the existing external C++ client: the pinned blocking RPC
API raced its own handler cleanup. Published 94814ee repairs only that client
through the public async API; all seven reference files remain byte-identical.
The adopted exact gate passes 195 Rust tests, twelve tooling checks and all nine
C++ scenarios, with unchanged production coverage. The qualified receipt is
planning/reviews/headless-imu.json. Normal-hook publication and hosted
qualification remain required.
