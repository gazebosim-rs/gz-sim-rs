<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Matched Zenoh headless stream observations

This packet retains the complete textual observations for one warmup pair and
seven alternating measured pairs, each with 1000 steps. Both implementations use
the same 1 ms TPE world and request at least 20 ms after complete Clock, Pose and
IMU receipt. The measured boundary includes server call, receipt, decoding and
assertions; pacing is excluded and wall time is separately retained. Header,
frame/sequence, covariance, motion, role identities, raw and adjusted byte counts
and every measured sample remain in the data. Only declared opaque ID varints
receive the existing cross-run byte adjustment; raw sizes remain recorded.

Median work latency is 63289.562 ns native and 58563.369 ns Rust per step (7.47%
lower for this Rust case). This is not unpaced throughput, a causal optimization,
all-engine parity or balanced robotics performance. Native local delivery may
bypass serialization while Rust uses encoded queues. Unequal direct/Server-only
measurements are not folded into this packet.

The source is consumer commit `5870373f0d8280601b051f2cbb1bb0aba264f70f`, tree
`5cd4cf6c9b3d6c98300f5820c9e508e0bdc135f9`. See `projection.json` for the native
image and release identities. `original-raw-sha256.json` binds every original
file. Original files remain untouched; the projection replaces only three
private source/build/home path prefixes with documented stable identity tokens.
All numeric JSON values and map address/offset/inode columns remain identical;
stdout/stderr is byte-identical. The native ELF is retained locally by its
original SHA; no edited executable is distributed. All textual observations,
original native source and linkage/maps are included.

The [phase report](../../../docs/acceptance/P2_ZENOH.md) and current dependency
receipt accompany these observations. Their publication commit requires the
normal staged hook and exact hosted/source verification before the coordinator
advances P2. Runtime and measurement evidence remains bound to the implementation
commit above; this publication does not extend its scope.
