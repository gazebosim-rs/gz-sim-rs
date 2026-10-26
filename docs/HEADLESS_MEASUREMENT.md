# Selected headless stream measurement

Public simulator `1c32b9d` with Transport `c21ffcc` was compared with the pinned
native Server, Physics, Imu and SceneBroadcaster. Both run the same two-model TPE
fixture with one noise-free 1000 Hz IMU, 1 ms physics and initial 1 m/s velocity.
Each request completes decoded clock, pose and IMU receipt before another step.
This measures summed request work latency; it excludes the declared 20 ms host
pacing and does not measure unpaced throughput.

| Measurement | Native C++ | Rust |
| --- | ---: | ---: |
| Median work latency per request | 65.229 us | 37.211 us |

The paired Rust/native geometric ratio is 0.56948, with paired bootstrap 95%
interval [0.55765, 0.58286]. This is lower observed work latency for this selected
fixture. It does not establish balanced robotics performance, general simulator
speedup, dedicated-hardware behavior or formal P2 acceptance.

The run retains eleven alternating process pairs on shared-host CPU31. Each
process receives an initial warmup sample outside timing, then 128 timed requests.
The first complete process pair is also declared warmup and excluded from summary
statistics; all eleven pairs and every extreme value remain in the report.
Bootstrap uses 10,000 paired resamples, seed42 and sorted percentile indices
floor((N-1)*p). The previous public05/native dataset remains unchanged; differences
between these batches do not isolate an individual optimization's effect.

[The public report](../planning/benchmarks/headless-streams-1c32.json) retains every
numeric and identity observation, with repeated fixture bytes replaced by their
hash. Independent source/build/binary/library/raw-packet/statistical review is
recorded in planning/reviews/headless-streams-1c32.json. Native installed dependency
and build provenance remain qualified; internal transport serialization paths
also differ. [STREAM_OBSERVATIONS.md](STREAM_OBSERVATIONS.md) defines field, byte,
cadence and failure assertions. The benchmark script and harness are unchanged.

Reproduce with `scripts/benchmark-headless-pipeline.py --help`, the pinned native
builds and uninstrumented Rust Release build JSONL. Use `--steps 128 --samples 10
--cpu 31 --path streams --request-period-us 20000`; retain all stdout, stderr,
loaded-library maps and build/source hashes. No correctness failure may be removed
by retrying, increasing queues or deadlines, dropping observations or changing
precision. The wider balanced suite and proposed budgets remain unaccepted.
