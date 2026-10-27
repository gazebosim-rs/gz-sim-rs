<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Loaded IMU activation

The activation increment adds `imu_active` and `set_imu_active` to
`gz-sim-scene`, plus `Server::set_imu_active`. These local APIs address an existing
loaded IMU by its public ID. Missing or wrong-kind IDs return a typed error.
Terminal simulation/reset faults reject mutation. No transport endpoint is added.

Inactive ordinary acquisition preserves the pending deadline and cached sample.
It consumes no sample sequence or noise draw. Physics velocity history continues
to update, so reenabling a sensor does not manufacture an acceleration spike.
The global raw-deadline/connection admission check retains the native ordering:
an inactive, connected, overdue sensor can still admit acquisition for its peers.
Changing activation does not rephase a finite-rate sensor.

Loaded full and time-only reconstruction restore the initial active state,
which defaults to true. Pause and entity removal use the existing lifecycle.
These are tested Rust contracts; the base Sensor probe does not establish native
whole-server activation/reset parity.

The qualified native probe records two rates and eighteen activation/force/time
observations, and runs the unchanged `Sensor_TEST.Sensor` original. Only its
initial-active and set-inactive getter assertions are mapped here. All other
assertions remain outstanding. Native forced-update rows are reference-only;
there is no forced Rust acquisition API.

Reproduce the installed-build observations with:

```sh
python3 scripts/probe-imu-activation.py \
  --prefix /path/to/pinned/native/install \
  --sensor-source /path/to/pinned/gz-sensors \
  --output /path/to/new/capture
```

The collector verifies source, compiler, headers, libraries and original-test
identity against the recorded lineage. It does not rebuild or qualify a clean
SDK. See [the reference](../planning/reference/imu-activation.json).

Independent native/provenance, source and runtime reviews have passed. The fresh
reviewer replay passes 91 checks, six production sensitivity controls and 91
restored checks. The tooling suite passes 88 tests. A control exposed a gap in
the first velocity-history test: velocity commands explicitly update that history.
Its assertions remain; an added natural Rapier gravity witness detects the faulty
ordering. The restricted server run's socket failures and qualified retry are
recorded separately in [the review](../planning/reviews/imu-activation.json).

The canonical staged gate passes 279 Rust and 88 tooling tests, retaining seventeen
functional C++ scenarios and three lifecycle controls. Independent full review
recomputes all nine crates above 95% lines and regions: workspace 3830/3870 lines
and 5873/6004 regions; changed scene and server lines are 22/22 and 7/7.

Two separately reviewed current-source SDK batches retain eleven process pairs
each, including warmup. The paired Rust/native work-latency ratios are 0.63209 and
0.64465 for the default-active TPE clock/pose/IMU workload. This measures selected
work, excludes declared host pacing, and does not isolate activation cost or show
a general speedup. See [all observations and intervals](../planning/benchmarks/headless-streams-imu-activation-v1.json).

Acceptance requires final metadata binding, the normal protected hook and exact
hosted CI for the final commit. These selected checks do not accept SENSOR-01
or P2.
