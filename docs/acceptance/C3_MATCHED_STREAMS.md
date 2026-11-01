<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# C3 matched headless stream observations

This is a new measurement of program commit
[`c3f5a2f`](https://github.com/gazebosim-rs/gz-sim-rs/commit/c3f5a2f3c39f27d98314e032e78c39cb44d660a0),
not a replacement for the historical 5870373 observations. Both sides run the same
small two-model TPE world, one constant 1 m/s body and one noise-free 1000 Hz IMU.
Every requested Clock, Pose and IMU sample must be received and decoded before
another step. Existing header, frame, sequence, covariance, motion, timestamp,
count, raw/adjusted byte and native library assertions remain unchanged.

One full warmup pair precedes seven alternating measured pairs. Each invocation
also preserves the existing in-program warmup and 1000 measured 1 ms steps, with at
least 20 ms request pacing after complete receipt. The work boundary includes the
server call, receipt, decode and checks; pacing is excluded and wall time is
retained. Both programs use CPU 0. Host/compiler preflight and a coordinated
no-build window were observed; this is not continuous proof of exclusive hardware.
No failed attempt, sample or outlier was discarded, and the batch ran once.

| Work latency per step | Native C++ | Rust C3 |
| --- | ---: | ---: |
| Median | 63,501.247 ns | 58,582.404 ns |
| Minimum pair | 57,597.449 ns | 55,188.337 ns |
| Maximum pair | 67,783.928 ns | 60,363.089 ns |

The observed Rust median is 7.7461% lower for this workload. Seven pairs on one
host do not establish a causal optimization, broad throughput improvement,
all-engine parity or balanced robotics performance. Native local delivery may
bypass serialization while Rust uses encoded queues. Clock wall-time bytes vary;
opaque Pose ID varints receive only the original documented size adjustment.
All original sizes and IDs remain in the raw data.

The actual release ELF is `4c9d40081e0ce8ef9f384d8f120bf74cb1722747f589942395cc96a88833d49e`,
built from the 982-file C3 source record with Rust 1.97.1/LLVM 22.1.6. The measurement
tool bytes used in the run were later committed as
[`1c70b81`](https://github.com/gazebosim-rs/gz-sim-rs/commit/1c70b81b5af3427b26afe1575844b4f5aea57eeb).
Their explicit native/Rust source contexts and all four tool input hashes remain
separate from the compiled program. No Cargo JSON was rewritten or source relabeled.
The immutable native image is `sha256:bae6ddc3a806532805a8103967d7ab353fdaf14ab0fcb957343f8f32bece11da`.
The qualified release-runtime overlay preserves common image libraries and
binds only the eight missing canonical DART-family files read-only, yielding an
18-file Rust loader closure. The native
image is locally retained, not claimed publicly pullable. Debug hosted objects
are not the measured release executable.

The [complete packet](../../planning/benchmarks/zenoh-matched-streams-c3/README.md)
contains 47 complete textual projections and hashes for all 48 original inputs;
the native ELF is retained unmodified by identity. Six declared private path
prefixes are replaced by stable tokens. Numeric JSON fields, all 32 stdout/stderr
files, eight map tables, every observation and every warmup/measured pair remain.
Original files and historical 5870373 evidence are unchanged.

C3 passed its [whole hosted verification](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41270504128).
The measurement tool passed its normal full staged hook, independent artifact audit and
[exact hosted verification](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41278421417),
with independently recomputed source/input/coverage/native/software-Vulkan evidence.
Independent raw/source/runtime/statistic audit SHA is
`da60023b4f3a1f9fd0ec0f1f94fe44e3d0e6fdacc12fa910fdbe3b26354eba0c`;
Its downloaded audit SHA is
`0c6c70df23e311508c53c87679f802142b7df19724f950001ee1fea506be231b`.

This report and privacy projection still require independent review and normal
publication checks. They do not qualify newer Main versions, rendering, joints,
network latency, whole ACT-01 absence or any new phase acceptance.
