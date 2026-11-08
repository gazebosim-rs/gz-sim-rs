<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Explicit lifecycle preview: capabilities and limits

M20 adds an explicit lifecycle/publication-credit choice to the Ubuntu 24.04
x86_64 native-free installed preview. It retains the complete M19 robot and
M13–M18 behavior. [INSTALL.md](INSTALL.md) gives the commands. The matching published release
records exact source, release-profile Cargo artifacts, selected dependencies,
ELF/system-library closure, notices and actual clean-install/browser evidence.
Source feature acceptance alone is not evidence that this new archive works.

The CLI and finite sender use locked release builds, `--no-default-features`
and no `rgb-camera`. Rapier f64 is authoritative. The six viewer files remain
accepted `c40418cd85e21db2bd0271919c90eda0c1184ece`; no renderer rebuild, new UI
or protocol is included. Both existing 14,000,000-byte gzip limits remain.
Python 3.11 or newer and ordinary dynamically linked Ubuntu runtime libraries
are required. No compiler, native physics SDK, camera worker, source checkout
or external asset service is needed to execute the installation. The server is
headless; the browser still needs working graphics support.

The launcher admits loopback HTTP 8080, 8081 or 8082, defaulting to 8081.
The example commands choose 8082 so older local HTTP services can remain running.
WebSocket and asset origin checks admit only the explicit local HTTP origins;
unlisted ports, remote hosts and other schemes remain refused. Zenoh TCP 7449
is fixed, and an occupied required port fails without touching its owner.

## Default and explicit lifecycle mode

The installed commands remain `run`, `drive` and `verify`. Only `run` accepts
`--lifecycle`; the default omits both the native flag and the viewer query.
Opt-in pairs `--web-lifecycle` with the outer HTTP `lifecycle=1` query and reuses
the maintained c404 viewer's existing one-flight/Ready publication contract.
It does not add a wire schema, enlarge a queue or change a deadline. Use the
printed URL so the native and viewer choices agree.

Opt-in exposes the existing full-UI lifecycle panel. Spawn is limited to
catalog-declared local templates. Remove and state operations retain their
existing admission checks and catalog flags. Every robot visual remains part
of the scene/catalog, but the jointed robot's `can_remove` and `can_set_state`
are false. It can still be driven normally; this release does not add
articulated-model editing or remove its sensors, joints or built-in systems.
Fixed-body state still requires zero velocity, and unsupported edits are refused.
No new sensor widgets or remote resource access are supplied.

This mode is an explicit functional choice, not a diagnosis or promised repair
of M19's earlier unknown consumer stalls. Connection warnings and failures remain
part of the matching release evidence; throughput and browser scheduling are not
performance claims.

## Selected interfaces and installed example

The package retains earlier bounded interfaces; not every interface is exercised
by the installed robot. The acceptance receipt distinguishes fresh artifact
integration from reused individual feature qualification.

| Feature | Native-free profile and installed example | Limits |
| --- | --- | --- |
| Primitives and local meshes (M1–M3, M14, M16) | Rapier box/sphere dynamics, local glTF visuals, bounded STL/triangle OBJ input, static triangle and already-convex dynamic collision. The example has an asymmetric copied OBJ cargo and static OBJ landmark. | No arbitrary polygon conversion, materials/textures, mesh repair, decomposition or remote loading. |
| IMU and planar lidar (M4, M7) | Existing demand-activated CPU models and canonical publication remain available. Neither is configured in this robot. | Retained bounded frame/cadence/lifecycle behavior; no broad noise, GPU or multilayer claim. |
| RGB/depth (M5, M6) | Excluded from this native-free distribution. | Accepted optional source remains; no renderer worker, camera acquisition or GPU sensor dependency is packaged. |
| Differential drive (M8) | Two revolute wheel joints, canonical Twist commands and encoder Odometry. The installed finite helper drives and turns the robot. | Motor bounds are not hard physical speed bounds under impulses; no general drivetrain or trajectory parity. |
| Joint controllers (M9) | Retained admitted position/velocity controller APIs. The example uses DiffDrive instead. | Existing reduced-coordinate servo policy, not general articulation or native PID parity. |
| Lifecycle and replay (M10, M11, M20) | Installed `run --lifecycle` pairs the existing native mode and viewer opt-in, including publication credit and admitted local template spawn/remove/state controls. Default `run` is unchanged; direct CLI recording/replay remains separate. | No arbitrary XML/resources, new edit domain, installed recording/replay option or full sensor recording. The articulated robot has both edit flags false. |
| Ideal altimeter (M13) | The robot configures 50 Hz world-Z position/reference/point velocity publication. | Noise-free admitted Rapier model; no native rewind-parity claim. |
| Ideal pressure (M15) | The robot configures 50 Hz canonical FluidPressure using the accepted geopotential formula at sensor world height. | No noise, nonzero reference altitude or nondefault atmosphere. Zero variance means unknown, not measured certainty. |
| Ideal magnetometer (M17) | The robot configures 100 Hz canonical Tesla vectors from a finite constant world field and its resolved rotated/offset sensor pose. | No noise, geographic/spherical-coordinate field model, UI or other backend. |
| Joint-state publisher (M18) | The robot configures 100 Hz canonical Model packets with actual admitted single-axis joint position/velocity and supported metadata. | No invented force, multiaxis/nested/dynamic joints or new solver. Equal-time pause cadence follows the documented native publisher contract. |

The robot keeps the M8 base/wheels, wheel radius 0.2 m, separation 0.5 m,
+Y wheel axes and configured 1 ms step. Its command topic is
`/model/robot/cmd_vel`; odometry is `/model/robot/odometry`, with
`robot/odom` and `robot/base_link` frames. Command bounds are ±1 m/s and
±2 rad/s; wheel velocity/torque bounds are 10 rad/s and 5 N m. The finite
helper uses wall-clock duration and ends successful sending with zero targets.

The added explicit magnetic field is `(0.00002, -0.00003, 0.00004)` Tesla.
The packaged fixture retains `/m17_m18/magnetometer` and
`/m17_m18/joint_state` topics from the qualified coexistence world. Canonical
sensor/joint publications use upstream Zenoh and maintained protobuf schemas;
the browser does not display numeric widgets for them. Sensor demand and joint
publisher equal-time behavior retain their separate documented contracts.

## Artifact evidence and exclusions

The release contains a simulator, finite sender, standard-library launcher,
robot fixture and ordinary copied local mesh, loopback config, both accepted
viewer bundles, these two guides, project LICENSE/NOTICE, complete notices and
manifest. Download the checksum and provenance from the matching release.
Exact package counts are derived from the actual selected native/helper graphs
and the retained conservative viewer graphs; old M12 counts are not assumed.
Raw declarations and every required license branch remain represented. Identified
canonical definitions and actual author records do not invent author grants;
any empty original is separately disclosed and never treated as license text.

The matching integration evidence covers a clean supported installation and an
explicitly opted-in installed robot session with forward/turn, Pause, Reset and
owned shutdown. It does not repeat
all earlier individual feature tests or qualify unsupported platforms, every
browser/driver, performance, odometry accuracy or cross-engine trajectories.
Retain connection warnings, actual failures and unknown shutdown observations
with the release evidence. M12 and earlier releases and their limitations remain
unchanged. No remote service, Fuel change, new transport/runtime framework,
dynamic plugins, backend expansion or whole-rewrite compatibility is added.
