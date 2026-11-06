<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Integrated preview: profile and capability limits

**M12 is pending.** These are the agreed package profile and retained source
boundaries, not a claim that a release archive, clean installation or integrated
demo has passed. Read the repository package contract at `docs/M12_PREVIEW.md` and the
[installation guide](M12_INSTALL.md), which is also copied into the prefix. M1–M11 runtime acceptance remains separate
from the new package's required source review, normal gates, exact hosted CI,
source-bound builds and clean-install/browser acceptance.

The initial target is Ubuntu 24.04 x86_64. The simulator and finite command
helper use `--no-default-features`; the simulator does not enable `rgb-camera`.
Rapier f64 supplies the robot's authoritative physics. The intended prefix has
`bin/gz-preview`, `bin/gz-sim-rs`, `bin/gz-drive-command`, one robot world and
loopback configuration, both maintained viewer bundles, these two guides, project LICENSE/NOTICE,
`notices.json`, the source-backed `notices/` tree and a manifest.
There is no claim of a fully static executable or compatibility with other Linux
releases, CPU architectures, browsers or graphics drivers.

## Package interfaces versus the robot demo

“In profile” below means retained source selected by the intended native-free
build. Actual package artifacts and their dependency/feature bindings are still
pending. The planned demonstration reuses the unchanged M8 robot: box chassis,
spherical wheels, static box ground and two configured revolute wheel joints.
It does not exercise every available interface.

| Accepted increment | In the proposed profile | Robot demo use (M12 installation run pending) | Retained boundary |
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

The accepted viewer base is `c40418cd`; simulator metadata base is `101f1ae8`
with runtime `182773f7`. These identify inherited source/evidence, not an M12
release commit. The assembler must bind the actual selected native artifacts and
both viewer bundles to source and Cargo records. Earlier WebGL2 observations,
WebGPU/WebGL2 bundle builds and their separate size gates are not universal
browser/driver support or a newly tested installation.

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
seconds and it ends a successful send with zero targets. Start playback of the
live world with Play before sending commands; sending while paused need not
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
  coverage and license/attribution requirements remain. M12 still must prove a
  clean supported installation runs this integrated demo, including actual
  forward/turn commands, viewer Pause/Reset and clean shutdown, after required
  checks. No package size, timing, dependency list or clean-machine pass is
  inferred from older acceptance records.
