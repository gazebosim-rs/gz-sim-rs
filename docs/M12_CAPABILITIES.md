<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Integrated preview: profile and capability limits

This is the M12 Linux preview's native-free profile. The matching published
release provides its exact source, feature, dependency and notice provenance.
See [installation and commands](M12_INSTALL.md) and that release's acceptance
evidence for qualified behavior and limitations.

The supported target is **Ubuntu 24.04 x86_64**, with Python 3.11 or newer and a
graphics-capable browser for the viewer. The clean installation used Ubuntu
24.04.5 and Python 3.12.3. Both selected native executables need the standard
`libgcc_s.so.1`, `libm.so.6`, `libc.so.6` and x86-64 loader; actual resolved file
ownership was `libgcc-s1:amd64` `14.2.0-4ubuntu2~24.04.1` and `libc6:amd64`
`2.39-0ubuntu8.9`. They are dynamically linked, not fully static. Other Linux
releases, architectures and graphics-driver/browser combinations are not
qualified by this result.

Both native inputs were built from simulator
`a542cb1b2c84280180e6acc73821a225557f6f44` with Rust 1.97.1,
`--no-default-features` and no `rgb-camera`. Rapier f64 remains authoritative.
The six copied viewer files are accepted
`c40418cd85e21db2bd0271919c90eda0c1184ece`, without viewer code or protocol
changes. Runtime libraries and the viewer require no checkout, compiler, native
physics SDK or camera worker. The server needs no display or GPU; the installed
browser demo used WebGL2. The retained WebGPU bundle is also included, without a
new universal graphics-support claim.

The release prefix is `gz-preview-m12-linux-x86_64/`, containing the simulator,
finite sender, standard-library launcher, robot fixture, loopback configuration,
both viewer bundles, these two guides, project LICENSE/NOTICE, `notices.json`,
`notices/` and a manifest. Obtain its external checksum from the same published
release.

## Package interfaces versus the robot demo

“In profile” below means the retained interfaces selected by the actual
native-free build. It does not mean every interface was exercised by this
installation demo. The demo reuses the unchanged M8 robot: box chassis,
spherical wheels, static box ground and two configured revolute wheel joints.
The installed robot demo exercised real finite forward/turn commands, browser
Play/Pause/Reset and owned shutdown.

| Accepted increment | In the native-free profile | Observed robot demo use | Retained boundary |
| --- | --- | --- | --- |
| [M1 primitives](https://github.com/gazebosim-rs/gz-sim-rs/blob/101f1ae8b5bcc58543daadd3660872cef5c2b5c8/docs/M1_ACCEPTANCE.md), simulator `35100cf8` | Rapier f64 scene loading, box/sphere dynamics, clock and viewer controls. | Primitive robot/ground, real motion, Play/Pause/Reset. | This is not a replay of the earlier 20-body acceptance demo or a new performance result. |
| [M2 local visuals](https://github.com/gazebosim-rs/gz-sim-rs/blob/101f1ae8b5bcc58543daadd3660872cef5c2b5c8/docs/M2_ACCEPTANCE.md), `7b8a4e10` | Existing bounded local glTF 2.0 visual/resource path. | No mesh asset in this robot world. | Scoped local resources only; not arbitrary formats, remote resources or general asset conversion. |
| [M3 mesh collisions](https://github.com/gazebosim-rs/gz-sim-rs/blob/101f1ae8b5bcc58543daadd3660872cef5c2b5c8/docs/M3_ACCEPTANCE.md), `ea8048f1` | Existing static triangle surfaces and already-convex dynamic mesh admission. | No mesh collision in the robot world. | Unsupported dynamic nonconvex inputs remain typed refusals; no automatic decomposition. |
| [M4 IMU](https://github.com/gazebosim-rs/gz-sim-rs/blob/101f1ae8b5bcc58543daadd3660872cef5c2b5c8/docs/M4_ACCEPTANCE.md), `eae6f3b8` | Demand-activated CPU IMU and canonical message path. | No IMU is configured. | Earlier bounded frame/rate/reset and teardown evidence is retained; this demo is not a new sensor qualification. |
| [M5 RGB](https://github.com/gazebosim-rs/gz-sim-rs/blob/101f1ae8b5bcc58543daadd3660872cef5c2b5c8/docs/M5_ACCEPTANCE.md), `60a3a710` | **Excluded** from this build/distribution profile. | No RGB image acquisition. | Accepted optional renderer-worker source remains intact; no worker or camera GPU dependency is packaged. |
| [M6 depth](https://github.com/gazebosim-rs/gz-sim-rs/blob/101f1ae8b5bcc58543daadd3660872cef5c2b5c8/docs/M6_ACCEPTANCE.md), `7fccd049` | **Excluded** with the optional camera worker. | No depth images. | Accepted metric-depth source remains intact; no depth accuracy or GPU claim transfers to this robot package. |
| [M7 lidar](https://github.com/gazebosim-rs/gz-sim-rs/blob/101f1ae8b5bcc58543daadd3660872cef5c2b5c8/docs/M7_ACCEPTANCE.md), `9a652063` | Existing bounded planar CPU ray lidar and canonical scan path. | No lidar is configured. | One admitted planar scan, primitive/mesh queries and demand lifecycle; not GPU or multilayer lidar. |
| [M8 drive](https://github.com/gazebosim-rs/gz-sim-rs/blob/101f1ae8b5bcc58543daadd3660872cef5c2b5c8/docs/M8_ACCEPTANCE.md), `fc967388` | Canonical Twist command and encoder Odometry with maintained native velocity motors. | The same robot drives and turns from the packaged finite sender. | Command bounds are not hard physical speed limits under external impulses; no general drivetrain parity. |
| [M9 controllers](https://github.com/gazebosim-rs/gz-sim-rs/blob/101f1ae8b5bcc58543daadd3660872cef5c2b5c8/docs/M9_ACCEPTANCE.md), `0f0f031e` | Existing admitted two-joint chain's bounded position/velocity controller APIs. | The robot uses DiffDrive, not the M9 controller plugins. | Continuous reduced-coordinate position servo is the documented Rust policy, not native ABS/PID parity or general articulations. |
| [M10 lifecycle](https://github.com/gazebosim-rs/gz-sim-rs/blob/101f1ae8b5bcc58543daadd3660872cef5c2b5c8/docs/M10_ACCEPTANCE.md), `d3967b61` | Existing explicit CLI/browser lifecycle opt-in and admitted standalone model edits. | Launcher does not enable lifecycle editing. The robot is joint-owned, not an editable standalone template. | No arbitrary XML/URLs from the browser, multiworld or distributed stepping. |
| [M11 recording/replay](https://github.com/gazebosim-rs/gz-sim-rs/blob/101f1ae8b5bcc58543daadd3660872cef5c2b5c8/docs/M11_ACCEPTANCE.md), `182773f7` | Existing direct CLI `--record-scene` and `--replay` interfaces. | Launcher does not record or replay, and exposes no such subcommand. | Bounded local scene/pose/control archive; not full sensor recording, seeking, historical log compatibility or physics re-execution. |

The simulator/helper artifact source is `a542cb1b`; the accepted viewer is
`c40418cd`. Notice generation is separately bound to the final qualified tooling
revision; it is unshipped build-time tooling, not native executable provenance.
The notice inventory covers 739 selected package records and 233 explicit extra notices,
retaining raw declarations and all required license branches. Disclosed missing
author-body cases use identified full definitions plus separate actual author
records, not invented grants. An actual zero-byte source placeholder is preserved
as raw evidence and never used as license text. The manifest and release
provenance record the distributed file and source bindings.

The copied viewer bundles have independent 14,000,000-byte gzip limits. Recorded
bundle measurements are 12,901,614 bytes WebGPU and 12,879,428 bytes WebGL2.
Those sizes describe the supplied bytes, not browser performance or universal
driver support.

## Robot commands and frames

The demo retains the M8 `robot` model, `base_link` canonical link, 0.2 m spherical
wheels at y = ±0.25 m, +Y wheel axes and the 1 ms configured physics step. Its
command topic is `/model/robot/cmd_vel`; odometry is
`/model/robot/odometry`, with frames `robot/odom` and `robot/base_link`.
Messages use upstream Zenoh and maintained canonical protobuf bindings. The
partition must match between the launcher and command helper.

Configured linear/yaw command bounds are ±1 m/s and ±2 rad/s; wheel target speed
and torque bounds are 10 rad/s and 5 N m. A zero command requests bounded braking,
not instantaneous physical rest. The finite helper's duration is wall-clock
seconds and it ends a successful send with zero targets. Click Play in the
live world before sending commands; sending while paused need not
produce visible motion. Odometry remains measured encoder output, not a claim
that contact slip or all physical trajectories match a reference engine.

## Deliberate package limits

- `run`, `drive` and `verify` are local preview operations. The launcher serves
  the packaged web tree on loopback and uses a fixed loopback Zenoh endpoint;
  this is not a remote deployment or multi-instance service manager.
- The native-free server needs no display or rendering GPU. The browser still
  needs a working supported WebGL2 or WebGPU path. Renderer workers, DART/MuJoCo
  SDK distributions and `rgb-camera` are not included. Selecting an unavailable
  backend or camera feature remains an explicit refusal, not silent fallback.
- The example uses no remote Fuel or external asset server. This neither changes
  nor requalifies the independently owned Fuel implementation. No ZeroMQ, custom
  transport runtime, dynamic system loader or full Gazebo compatibility is added.
- Existing acceptance restrictions, assertions, numerical tolerances, feature
  coverage and license/attribution requirements remain. The installed demo
  exercised forward/turn motion, Pause and Reset to the initial pose and zero
  clock. This does not newly qualify the other retained interfaces in the table.
- Visible browser behavior and control receipts do not establish numeric
  odometry, cold-start timing or performance. Consult the matching release's
  evidence for the actual observations and their limits.
