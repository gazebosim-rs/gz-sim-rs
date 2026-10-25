# Complete clock, pose and IMU observations

The selected TPE fixture now requests the same three streams from actual Rust
and native servers: world clock, parent-relative poses and IMU. Each public
one-step call must finish with all three decoded observations before another
step is admitted. Native uses Physics, Imu and SceneBroadcaster; Rust calls its
production Server::tick path. Both use unchanged headless_imu.sdf bytes, 1 ms
physics steps, real-time factor zero, one 1000 Hz noise-free IMU and initial
1 m/s horizontal body velocity. Queue capacity/high-watermarks stay 1000.

## Cadence and measurement boundary

Pinned SceneBroadcaster hardcodes pose/info to 60 wall-clock messages/second.
The initial unpaced attempt failed at its second step with counts IMU=2,
clock=2, pose=1. No reference code, subscriber assertion or queue limit was
changed to hide that failure.

Both callers now wait at least 20 ms after each completed observation before
issuing the next step, an upper bound of 50 requests/second. This prospective
workload pacing is separate from readiness: an actual warmup clock, pose and IMU
must first arrive. There are no extra advancing steps, retries or resampling to
fill missing observations. A missing stream still fails.

The report records the sum of each public server call plus complete receipt,
decode and validation time. It excludes the declared pacing and records actual
pacing and total wall time separately. Do not invert this latency into unpaced
throughput. The earlier unpaced IMU-only reports remain unchanged. Rust currently
publishes pose/info on every tick; this experiment matches requested observations
at the common cadence and does not establish default pose-rate parity.

## Assertions and byte accounting

All 128 measured steps per process have exactly one clock, one four-role pose
packet and one IMU. The received warmup at 1 ms is outside timing. Every simulated
timestamp and checksum must match 2..129 ms. Body motion, frame IDs, independent
sequence, covariance entries, field presence and nested headers are checked.
Clock real time is nonnegative and monotonic. System time has normalized fields
and its seconds value must be within one receiver wall-clock second bin.
Host real/system values are independent across processes; their raw payload
sizes are recorded, not required to match.

Pose names, model/link grouping, unique nonzero IDs and each role's identity
stability are checked, along with every translation/quaternion component. Runtime
ID values can differ between implementations. Raw pose sizes remain in the
report; the cross-run size comparison subtracts only the protobuf varint values
of these opaque IDs. Every small pose record must remain below 128 encoded bytes,
so its length delimiter stays unchanged. The verifier checks the exact subtraction
from the four reported IDs. No other field is masked. IMU logical byte totals
must match directly. Extra frames and local error/drop counters fail acceptance.

## Scoped result

The report is planning/benchmarks/headless-streams.json. On shared AMD 5950X CPU31,
one warmup process pair precedes seven alternating measured pairs. Each value is
a process's mean of 128 work intervals; the table shows the median and range of
those seven means, not individual request percentiles.

| Implementation | Median work latency | Range of process means |
| --- | ---: | ---: |
| Native | 77.438 us | 73.459..127.151 us |
| Rust | 43.748 us | 36.051..268.729 us |

The median native/Rust ratio is 1.770 for this requested-observation contract.
The large Rust outlier is retained. This shared-host result is not a statistical
service guarantee, maximum throughput, balanced robotics or whole-Gazebo speedup.
Native scene graph/root-clock work and local delivery optimizations differ from
Rust encoded worker queues. Compiler/build settings, critical source and loaded
library hashes, fixture echo/hash, application binaries and all pairs are recorded.
Installed dependency provenance remains qualified rather than a complete immutable
reference image. No original upstream test or existing assertion is retired.

Independent review reproduced the complete 128-step by seven-pair protocol from
a private raw export and separate release target. Every process matched the
clock/pose/IMU counts and time sums, IMU bytes, pose bytes after exact opaque-ID
subtraction, force sum, final body position and zero warmup acceleration. The
missing-pose production mutant failed at warmup receipt; seven independent report
mutants rejected stream, time, pacing and ID-accounting errors. A shorter 16-step
run is retained as calibration, alongside its private Git-setup failure; it is
not substituted for the complete replay. Private timings are separate evidence.
The authored range and large outlier remain unchanged.

The exact source staged gate passes 204 Rust tests, sixteen tooling checks and
all nine real C++ scenarios: 3193/3228 production lines and 4964/5079 regions,
every production crate above 95%, no new executable production lines. The receipt
is planning/reviews/headless-streams.json. Normal-hook publication and hosted
qualification remain required. Full P2 acceptance still needs the combined
selected-fixture matrix.

Reproduce after building the native Release Physics, Imu and SceneBroadcaster
targets and the matching TPE/sensors dependencies:

```sh
cargo test --locked --release -p gz-sim-server --lib --no-run \
  --message-format=json > artifacts/streams-build.jsonl
python3 scripts/benchmark-headless-pipeline.py --path streams --steps 128 \
  --request-period-us 20000 --cpu 31 --sim-build /path/to/sim-release \
  --physics-build /path/to/physics-release --sensors-build /path/to/sensors-release \
  --dependency-prefix /path/to/native-prefix \
  --reference-prefix /path/to/transport-reference/install \
  --rust-build-json artifacts/streams-build.jsonl --work-dir artifacts/streams-run \
  --output artifacts/streams-report.json
```
