<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M114: organized CPU lidar point clouds

This phase adds optional `gz.msgs.PointCloudPacked` output from the existing
bounded CPU lidar acquisition. `ServerOptions.lidar_point_clouds` defaults to
false; the CLI exposes `--lidar-point-clouds` only with `--server`, excluding
replay. The implementation budget is at most five working days, including
independent review and every mandatory verification gate.

The source parent is `02592f7aaf21d616bd6299aa87123455834794e7`, tree
`4a0e9343a5e3dba4d7e32c9ef6e56feb7cab3536`. Its normal hook, independent
local audit, exact hosted CI and original-artifact audit passed. It is not
accepted. The first full hook on frozen tree `367a2c34` failed after 121.851
seconds: 498 Python tests ran with one failure because `native-peer.py` omitted
`lidar_points` from its command-line choices. Rust runtime profiles and coverage
were not reached; no commit or qualification resulted. This isolated successor
adds the choice, explicit Zenoh-only refusal and diagnostic, and extends the
existing dispatcher regression to check the source path and both wrong-profile
refusals before reference reads or compilation. Test identities, assertions and
counts are preserved. The successor has no verification result and inherits no
qualification. Both failure packets remain retained.

The dispatcher-corrected full retry on `471e7448` ran for 4,428.109 seconds.
Python recorded 498 passes. Headless recorded 2,170 passed, one failed and two
ignored; renderer recorded 1,683 passed, zero failed and two ignored with its
retained coverage outputs. Resource floors held. The sole failure occurred
while compiling the new C++ consumer: the decimal-digit loop and independent
canonical-sequence check occupied one ambiguous line, rejected by the retained
`-Werror=misleading-indentation` flag. The native consumer therefore did not
execute. This successor only braces that loop and separates the existing check.
Every assertion, input, tolerance, deadline, cap, test identity and Rust byte is
unchanged. Headless coverage and full qualification remain required; renderer
results do not qualify the failed candidate or this successor.

## Output contract

The selected cloud shares the actual scan acquisition, timestamp, sensor pose,
ray results and noise draws. It performs no extra raycast or random draw.
Coordinates are in the acquisition sensor frame, not the world frame. The
organized grid has horizontal width and vertical height, vertical rows outside
horizontal columns, with at most 128 points. Existing scan geometry and range
semantics remain unchanged.

Each point occupies exactly 18 bytes, without alignment padding:

| Field | Type | Offset | Count |
| --- | --- | ---: | ---: |
| x | FLOAT32 | 0 | 1 |
| y | FLOAT32 | 4 | 1 |
| z | FLOAT32 | 8 | 1 |
| intensity | FLOAT32 | 12 | 1 |
| ring | UINT16 | 16 | 1 |

Encoding is explicitly little-endian. `point_step` is 18 and `row_step` is
18 times width. Intensity is zero; ring is the zero-based vertical row. XYZ is
the corresponding unit ray direction multiplied by the acquired range and
narrowed to f32. A no-return range remains positive infinity: multiplying its
components produces signed infinities and NaNs for zero direction components.
There is no fabricated all-NaN miss sentinel. `is_dense` is false if any range
is nonfinite. NaN or negative-infinite ranges are invalid samples.

The header contains the acquisition stamp, `frame_id` and decimal `seq`.
The new cloud has a separate counter, initially zero and reset with its stream.
The existing LaserScan counter and packet bytes retain their old behavior.
This intentionally differs from the retained native Sensor default sequence
counter, which is shared and interleaves increments when scan and cloud are
both published. Native dual-stream sequence parity is not claimed.

The cloud's native topic is the raw scan topic plus `/points`, independently
normalized, remapped once and qualified. For example, remapping `/scan` to
`/other` leaves the raw sibling `/scan/points` unchanged. Remap that sibling
explicitly to move it too. No new transport family, session or runtime is added.

## Admission, ownership and limits

Cloud output is selected for the complete lidar family before declarations.
There remain at most eight owners, with at most eight Scan and eight Points
declarations. Live and removed-original endpoints reserve both selected keys;
other sensor/controller families and loaded reset cannot steal either key.
Prepared publishers must match the original owner, metadata, endpoint and
message kind. A failed later declaration drops all newly acquired CPU leases.
The existing IO owner performs declaration retirement and callback quiescence.

Opt-in requires finite `range_max <= f32::MAX`; finite XYZ conversion is also
checked before sample/counter mutation. Scan-only range admission is unchanged.
Cloud `frame_id` and fully qualified key are each bounded to 1024 UTF-8 bytes.
Cloud encoding is at most 4096 bytes. Selecting clouds requires configured
`max_message_bytes >= 4096`, but that minimum does not guarantee admission of
every allowed frame: LaserScan repeats the frame in both its header and frame
field. Pure worst-case timestamp, sequence, pose and range encoding checks
every owner before any declaration. Each message must fit the configured
global cap and the two encoded messages together must fit 8192 bytes.
The 128-point payload itself is at most 2304 bytes; five field descriptors,
bounded header and dimensions keep the cloud below its dedicated cap.

Scan, cloud and viewer interest are unioned for acquisition. With no interest,
there is no acquisition or absent-family work. A scan-only subscriber does not
cause cloud conversion or advance the cloud counter. Cloud-only interest can
drive the same acquisition. The Scan packet is still generated and its counter
advances once per demanded acquisition, including cloud-only demand; the cloud
counter remains independent.
The two messages occupy one bounded WorldIo queue item. Both typed Weak lease
identities are checked before either put. Remote Zenoh delivery is not a
transaction: an admitted put may complete independently while an owner or
subscriber retires. Epoch checks, old-job cancellation, removal and loaded reset
retain the existing rules. While cloud demand is active, malformed selected-cloud samples leave the
previous sample marker and both counters unchanged, permitting repair before
publication. Without cloud demand the unchanged Scan validation and marker
semantics apply.

## Retained native reference

The retained `gz-sensors` source commit is
`8ddf4c17be8a434c51b51d5be7c866fb69aaa54f`. The inspected files are:

| Local source | SHA-256 |
| --- | --- |
| `/home/nkoenig/work/gazebo/rust/gz-sensors/src/CpuLidarSensor.cc` | `1c839efc4e5c3650eb41dd32f3f62bbe9b9b0c640fd70712e018c9c5251587a9` |
| `/home/nkoenig/work/gazebo/rust/gz-sensors/src/Sensor.cc` | `5fb2ea0aaa27b960c463c82de3074ad0bb221341454fb999946eca9f5afff2a2` |
| `/home/nkoenig/work/gazebo/rust/gz-sensors/include/gz/sensors/CpuLidarSensor.hh` | `3e53bdfac804e9367902f6d9e388aab088d29b0d0eb00edc70da360732004eed` |
| `/home/nkoenig/work/gazebo/rust/gz-sensors/include/gz/sensors/Sensor.hh` | `5938859b63479e6bd872f5df6bc48070c9a733fa3f7a240da7bfa3adbde3156b` |

CpuLidarSensor's field setup, direction calculation and point publication supply
the five-field unaligned layout, organized order, ring, frame, `/points` suffix
and componentwise miss behavior. Sensor's default sequence-key implementation
establishes the compatibility disposition above. Its source is retained evidence,
not proof that a separately installed binary was built from these exact bytes.
The historical pinned sensor component
`/home/nkoenig/local/lib/libgz-sensors-cpu_lidar.so.11.0.0~pre2` has SHA-256
`2947b2d4007f36eb0d2dfabc36233710587ddac416460fa1a90d37935549933a`.
This phase adds no native sensor execution or physics parity campaign.

The retained `gz-msgs` commit is `89c0c3d1474f545cb889335daf930bfdab6ab852`.
`/home/nkoenig/work/gazebo/rust/gz-msgs/core/include/gz/msgs/PointCloudPackedUtils.hh`
has SHA-256 `383a3b5abf12d7fd6ba6cc4813ad3ab66dda1a08092b76c610650e1ac3b67747`;
its `detail/PointCloudPackedUtils.hh` has
`8afb86a459787432904ef8b03a5a3ce9585d0b9eee2005ca4351856c445b9a06`.
These match the retained selected SDK headers. The helper's false argument is
the memory-alignment flag, not an endian flag. The unchanged canonical schema
`crates/gz-sim-proto/schemas/gz/msgs/pointcloud_packed.proto` has SHA-256
`21360fcd43d239b607a446291e1f63a89f85974f428a4507274da206ef7524cb`.
No schema or generator change is needed.

## Regression source and verification

The new Server source declares 14 headless-only cases: eleven private cases in
`lidar::point_tests`, two public cases in `tests/lidar_points.rs`, and the
`tests/lidar_points_native.rs` C++ interoperability case. They cover analytic
sensor coordinates and unaligned decoding, componentwise misses, malformed
samples/pose/time, float and UTF-8 boundaries, grid/message limits, independent
remaps, eight-owner/sixteen-declaration bounds, typed and retired jobs, paired
real wire, independent counters and late demand, default scan behavior, reset,
metadata faults, removal, and live/removed-original cross-family reservations.
These are source obligations, not executed passes. The two CLI parser cases
owned by the integrator are separate shared-profile obligations.

The `lidar_points` native peer subscribes only to `/m114/scan/points`, validates
the actual canonical five-field message and independently computes a 3x3 wall's
sensor coordinates. It decodes unaligned floats safely and retains the actual
packet. Its C++ deadline is ten seconds; the Rust child wrapper retains the
bounded 32-KiB stdout and 4-KiB stderr capture, a 4096-byte packet and
8192-digit hex limit, twenty-second deadline and kill/reap cleanup. It uses
the existing selected NativePeer SDK and process-local environment. It neither
registers a new transport nor mutates parent process environment.

Finish the whole source review and batch findings before one affected formatter
and one consolidated final staged hook. Both mandatory profiles, unchanged
workspace/crate/changed-base coverage, the maintained Python/native qualifiers,
independent local retained audit, exact automatic hosted CI and independent
original-artifact audit must pass for this candidate. No individual regression
run, cached test result or parent qualification replaces those gates.

Existing planar scans, spatial scans, old error controls, Rapier f64 and TPE
remain. Reflectance, color, normals, arbitrary cloud input, GPU lidar, native
sensor parity, DART/MuJoCo, transport redesign and future M115 are excluded.
Accepted M1–M76 and 202 slices are unchanged. The no-expiry M77 demo remains
protected; formal browser and owner-confirmed shutdown still gate acceptance.
No demo, Fuel, cleanup or permission change is authorized by this source work.
