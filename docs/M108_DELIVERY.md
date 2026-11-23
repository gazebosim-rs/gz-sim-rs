<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M108 — bounded three-dimensional CPU lidar

This selected Source lane extends the existing loaded Rapier CPU lidar through
its current acquisition and canonical LaserScan paths. It introduces no GPU
lidar, point cloud, interpolation, transport, SDK or runtime framework.

## Closed geometry and compatibility

`PlanarLidar` and its public configuration, direction construction, range rules
and existing tests remain unchanged. `SpatialLidar` composes that model with
`VerticalLidarConfig`. A single row accepts only zero vertical angles and keeps
the exact legacy direction bits and positive-zero vertical packet fields,
including an explicitly parsed negative zero. The caller configuration is not
mutated. Two or more rows require finite strictly ordered angles within
[-pi/2,+pi/2], a finite positive representable step and at most 128 total rays.
Checked multiplication precedes allocation. Horizontal and vertical resolution
remain exactly one. The existing planar horizontal sample/span/range policies,
Gaussian activation and zero-minimum-range restriction remain.

Rows increase in elevation; each row increases in horizontal azimuth. This is
the vertical-major/horizontal-minor ordering in retained
`gz-sensors/src/CpuLidarSensor.cc` (154–177 and 204–214). Directions are
[cos(elevation) cos(azimuth), cos(elevation) sin(azimuth), sin(elevation)].
Every scan still uses one actual acquisition pose; no per-ray motion estimate
is invented. Misses remain positive infinity and finite ideal hits retain
continuous range clipping. Noise consumes the same stream in beam order.

`LidarPublication` retains its planar horizontal metadata and adds explicit
vertical configuration. The complete immutable metadata participates in the
existing ownership/reset comparisons. Server packet validation checks both
axes and the complete ray product before any packet can be emitted. Both
counts, angle ranges and steps are carried through existing LaserScan fields;
intensities remain zero, with one value per ray. Eight-owner demand, cadence,
latest sample, sequence, reset and retirement mechanisms remain in use.

## Regression Source and evidence boundary

New sensor regressions cover independent spherical directions, total-product
bounds, ordered-angle precision, underflow/overflow, exact planar bits and range
rules. New Scene regressions cast real Rapier rays against a wall and distinct
elevated planes, compare independently derived distances using the actual
acquisition frame during motion, and cover misses, minimum clipping, demand,
pause, noise ownership, reset, removal, ninth-owner/backend and bad-grid errors.
New private Server regressions cover canonical row-major packet encoding and
malformed metadata/sample refusal, including the full 128-ray boundary.

The new transport-only `spatial_lidar` C++ observer receives one real canonical
3×3 packet through the existing selected native-client framework. It validates
both axis descriptions, acquisition pose/stamp/frame, complete row-major ranges,
zero intensities and the protobuf wire roundtrip. Its bounded deadline,
channel captures and child retirement are in the new Rust integration leaf.
It is not a native sensor or physics oracle. Root owns registration in existing
native-peer manifests. The old planar observer remains unchanged.

Root also owns the two retained Server metadata constructor additions:
`vertical: Default::default()`. No old test case or assertion is retired.
These regression declarations are Source obligations, not executed results.

## Leases and acceptance

Owned production files are sensor, Scene and Server `src/lidar.rs`; new tests
are `gz-sensors/tests/spatial_lidar.rs`, Scene `tests/spatial_lidar.rs`, Server
`tests/private/spatial_lidar_packets.rs`, `tests/spatial_lidar_native.rs` and
`tests/native_spatial_lidar_peer.cc`. No shared Cargo or Scene export edit is
needed. The lane budget is at most five working days including review and
mandatory verification.

The shared candidate includes reviewed but unverified M102/M103 foundation
Source over qualified08a0. Neither that foundation nor ancestor evidence grants
this phase a pass. Complete all ten lanes, documents and independent Source
review before Root's one consolidated formatter/full staged hook, strict
coverage, local evidence audit, exact hosted attempt and original-artifact
audit. No tests, lint, formatter, builds, native observer or network operations
have run for this authored lane. Execution, new commit and acceptance are unknown;
the resource admission hold and persistent demo are unchanged.
