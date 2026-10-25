# IMU sampling cadence

The scene owns a small simulation-time clock for each loaded IMU. Bevy still owns
system scheduling. This clock uses standard Duration arithmetic; it is not a
second scheduler or a wall-clock timer. No network or filesystem wait occurs in
the simulation step.

The clock preserves the pinned native Linux Sensor update rules: nonpositive rates
sample each update, positive periods truncate to nanoseconds and clamp to one
nanosecond, early updates skip, and forced clock updates leave the phase unchanged.
After a due sample, constant-time catch-up selects the first phase-aligned time
past now. There is at most one acquisition per physics update, including when the
requested sensor rate exceeds the physics rate. Missed periods are not replayed.
The internal force rule is tested; runtime trigger/force commands are not added.

The loader rejects nonfinite or unrepresentable periods explicitly. Native
out-of-range floating-to-clock casts are not reproduced. A next-time overflow
reports a typed StepFault, produces no new sample and makes the run terminal;
the core retains its last committed time. Existing effects are not rolled back.

Physics velocity differences are computed every physics tick before output gating,
so a slow output rate still uses the current physics acceleration. Pause leaves
the clock and derivative state unchanged. Reset clears phase, prior velocity and
samples; the loaded blueprint owns the initial clock without IO. The server's
existing (epoch, timestamp) deduplication suppresses repeated publication of a
retained sample between acquisitions.

Both engines are checked at 0, 10, 60, 1000 and 2000 Hz with a 1 ms physics step,
including exact phase, acceleration, pause and reset. Two additional real C++
transport cases receive exactly the 1 ms/100 ms frames at 10 Hz, check sequences,
metadata, covariances and readings, then observe sequence zero after a second
reset. Warmup uses real received packets; it does not infer subscription readiness
from registration. The previous seven C++ cases and their assertions remain.

Ten fresh native clock controls and the unchanged SensorUpdate.NextDataUpdateTime
original run against pinned gz-sensors source. Source, header, library and binary
hashes are recorded in planning/reference/sensor-schedule.json. Reproduce with:

```sh
python3 scripts/probe-sensor-schedule.py --dependency-prefix "$DEPENDENCY_PREFIX" --sensor-source "$SENSOR_SOURCE" --work-dir /tmp/sensor-clock --output /tmp/sensor-clock.json
```

The source and built native test are required; linked SDK dependencies remain
qualified. This is clock evidence, not full sensor or simulator parity.
[SUBSCRIBER_DEMAND.md](SUBSCRIBER_DEMAND.md) records the subsequent native
world-wide subscriber gate and explicit local acquisition override.
[IMU_NOISE.md](IMU_NOISE.md) records Gaussian sampling and acquisition gaps.
Runtime activation/registration, trigger modes, metrics, other sensor families
and the complete SENSOR-01/P2 contracts remain open. No speedup is claimed.
