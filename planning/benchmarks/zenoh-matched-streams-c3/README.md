<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Current C3 matched stream observations

This packet preserves one warmup and seven alternating measured pairs, each with
1000 steps, a 1 ms TPE world and at least 20 ms request pacing after complete
Clock/Pose/IMU receipt. The unchanged measured boundary includes the server call,
receipt, decode and assertions; pacing is excluded and wall time is retained.
All observations, opaque role IDs, raw and normalized bytes, timestamps, motion,
checksums, headers and sequence/cadence assertions remain. Only the existing ID
varint adjustment applies to the cross-run Pose byte comparison.

The measured program is C3 commit `c3f5a2f3c39f27d98314e032e78c39cb44d660a0`.
The measured measurement tool driver bytes were later committed as
`1c70b81b5af3427b26afe1575844b4f5aea57eeb`; its explicit native/Rust source
contexts preserve all original measurement code. The exact measurement-time
file hashes, rather than a fabricated measurement-time Git HEAD, are retained.
The actual release ELF and immutable native image are identified in
`projection.json`. This does not relabel the historical 5870373 driver or results.

The observed medians are 63501.247 ns native and 58582.404 ns Rust per step,
7.7461% lower for this Rust case. Seven pairs on one host do not establish a
causal benefit, broad throughput improvement or balanced robotics performance.
Native local delivery may bypass serialization; Rust uses encoded queues.

All 48 original inputs remain unchanged. Their hashes include the unmodified
native ELF (identity only); 47 complete textual projections replace only six
declared private path prefixes with stable identity tokens. No numeric, wire,
physical, cadence, sequence or sample observation is removed. Map columns and
stdout/stderr observations remain complete. The program, native app and
measurement-tool source contexts are deliberately distinct.

Independent raw/source/runtime/statistic review and measurement tool exact hosted/source
verification passed. Separately reviewed report publication remains pending.
No new accepted performance claim follows merely from preparing this packet.
