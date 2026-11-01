<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Programmatic loaded-camera application

`CameraServer` in `gz-camera-output` composes the existing headless `Server` and
single-camera output pump. A recognized, bare **world**
`gz::sim::systems::Sensors` / `gz-sim-sensors-system` record and exactly one
supported loaded camera are required. The qualified camera domain remains RGB,
64×64 with the existing calibration, visual and capture limits. Camera metadata
without that world intent retains the manual API; it does not activate output.
Duplicate Sensors records, model placement and Sensors payload are refused in
this first application slice. These are scope limits, not native-invalid input.

The ordinary Server refuses configured camera execution with
`CameraOutputRequired` before creating world IO. A configured empty world stays
headless. `with_external_camera_output` is an explicit ownership contract for the
one validated camera ID, not a renderer inside Server. Only the camera crate
depends on Server; neither default Server nor CLI gains a rendering dependency.
CLI ownership and full native Sensors/Manager behavior remain pending.

## Preparation and ownership

Construct from a loaded `Simulation`, `ServerOptions`, explicit world Zenoh
configuration, command budget and borrowed `CameraServerOptions`. Pure preflight
validates world intent, camera count, public identity, scoped hierarchy, topics,
loaded frame, calibration, bounds and operator worker path. Camera IO is prepared
before Server construction; construction failures release that prepared IO.
This is not an atomic network-advertisement guarantee.

With no configured camera, the application retains no camera runtime, Session,
writer scope, schedule, endpoint, worker path or callback. Its advancing path
skips camera dispatch. With a configured camera, existing declarations advertise
the image and camera-info topics. Image interest alone starts the existing
out-of-process renderer. Info-only publication uses the existing CPU packet path;
this ownership choice is not proof of native info publication without a renderer.

Every preparation, update, pump, retirement and destruction belongs **outside the
critical step**. They can resolve Zenoh state, spawn/join a worker or close a
Session. The Server raw tick remains the explicit critical-step API. Camera work
uses no registered Bevy callback or second scheduling algorithm. The application
reads committed `UpdateInfo` through the existing `ScheduledCameraOutput`.


`camera_demand(&mut self)` observes `(image, camera_info)` matching on the actual
current declarations without advancing time, cadence or sequence. It returns
`None` for an absent owner, and applies the same terminal, pending-reset and
identity/epoch retirement checks as pumping. It belongs outside the critical
step and can retire invalid native state. Matching remains advisory; actual
native receipts establish delivery.

One acquisition remains the bound. A full update may advance physics before a
camera error such as Busy. The error is returned; no capture or frame is silently
retried. Phase consumption follows the existing scheduler, including no-demand,
partial-info, failure and pause semantics. Explicit steps while configured paused
are governed by their committed nonzero dt, not by the configured pause flag.

Normal removal retires the camera family. Identity, topology, metadata or name
replacement invalidates the old output rather than rebinding it. An accepted
pending reset makes `pump` retire old output and return `IoPreparationRequired`
before the Scene epoch changes. An update that accepts reset during its raw
Server tick also retires before its camera phase and returns the same typed
error; its already-committed physics/counters are preserved. Explicit
`prepare_io` commits reset and rebuilds
from restored metadata using borrowed operator options. Committed preparation
errors release old output before returning; there is no stale-frame publication.
Shutdown through either application or exposed Server is terminal for camera
work even though the preserved Scene and epoch remain inspectable. Dynamic
camera insertion/registration is outside this loaded-world slice.

## Native mapping and current evidence

The selected native default server configuration does **not** include Sensors.
SimulationRunner may merge operator/default plugin settings. The bounded CPU
constructor probe observes the selected default three systems and a fourth
Sensors system only with explicit Sensors configuration; it executes no frame.
Native model placement, duplicate records and unknown payload are accepted by
that reference and are explicitly outside the Rust admission domain above.

Native Sim `RenderUtil::AddNewSensor` supplies a world-stripped scoped sensor
name and, when topic is empty, the typed world/model/link/sensor path plus
`/image`. `CameraSensor::Load` normalizes that topic. Explicit camera-info topic
is normalized separately; an empty or invalid override derives a sibling
`camera_info` from the normalized image **before** transport remap/namespace.
Each resulting topic receives exactly one `ServerOptions::topic_key` mapping.
Loaded frame precedence uses the existing checked helper; an explicitly empty
frame remains subject to the existing constructor refusal.

The CPU metadata probe covers the two unchanged native camera fixtures plus six
supplemental topic cases, with captured expanded compiler argv, dependency
selection and mapped libraries. The separate immutable-image activation probe
covers eight zero-step constructor cases. Source pins are Sim
`71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`, Sensors
`8ddf4c17be8a434c51b51d5be7c866fb69aaa54f` and SDFormat
`1249805d9fceeff94ca1fe63e26ab9c051cea24e`. The checked-in [probe record](../reference/automatic-camera-native.json) binds
the [topic program](../crates/gz-camera-output/tests/fixtures/loaded_camera_topics.cc)
and [activation program](../crates/gz-camera-output/tests/fixtures/loaded_camera_activation.cc),
all sixteen exact inputs/results, expanded compiler flags and mapped libraries.
Its paths are explicitly projected aliases; full actual argv, dependency files
and raw byte/hash records remain privately under `artifacts/render03-single-camera`.
The recorded ELF hashes identify those captures, not portable cross-host golden binaries. They establish
metadata/activation facts only, not native Scene rendering, Update/RenderUtil
execution, info-only image-free rendering, full original-case credit or physical
GPU parity. Native camera `set_rate`, full always-on/trigger behavior, Manager,
CLI integration and broader RENDER-03/SENSOR-05/ACT-01 remain open.

Focused CPU qualification passes 52 CameraOutput, 166 Server and 64 Scene
library controls plus selected Clippy. A first new-test host-port failure
(46/49) remains separately recorded; independently reviewed namespace isolation
preserves every predicate and default suite threading. Three software-worker
lifecycle cases and two shared native-helper controls pass. The unchanged C++
subscriber also passes separately through this wrapper's default-derived,
once-remapped topics and explicit loaded frame: exact first 1ms Image/CameraInfo,
RGB pixel/type/header/sequence checks, exit zero and empty stderr.

Two earlier 5/6 worker outcomes remain separately retained. A downstream Peer
matching witness was invalid under the selected routing contract; the application
now exposes `camera_demand` on its own declarations. That advisory observation
adds no sample, listener or worker, and does not acknowledge delivery. The next
attempt proved delivery but failed a strict stderr assertion on native interface
fallback warnings in the isolated loopback namespace. Child-only `GZ_IP=127.0.0.1`
provides the actual interface explicitly, preserving all assertions and backend
selection. Only the corrected C++ case was replayed: these are composed passing
proofs, not a fresh single-batch 6/6 claim. Actual native stdout/stderr/status and
compiler/library provenance are retained together under the private worker
capture. Independent final source and composed-evidence review passed. The candidate based on
Main `3d290ee` preserved collision density and Sensors intent together across
reset. The first integrated normal hook failed CameraOutput's strict region
coverage gate at 1267/1365; its failed stage is retained and no commit was
published. The repair adds meaningful lifecycle and identity/hierarchy controls
under independent review on accepted Main `ce071305`, without changing
production behavior, retained assertions, deadlines or coverage policy.
Complete normal and exact hosted qualification for the repair remain pending. The
existing renderer timeout history and unchanged lifecycle deadlines remain part
of qualification; no new retry, timeout relaxation or coverage exclusion applies.


| Original native case | Bounded mapping; remaining assertions |
| --- | --- |
| `CameraSensorTest.ImagesWithBuiltinSDF` | Unchanged fixture CPU Load metadata only. Its 256×257 images and forced inactive updates are not completed by this 64×64 path. |
| `CameraSensorTest.CustomCameraInfoTopic` | Original topic preprocessing plus separate Rust CPU packet receipts. Native image rendering and complete case parity remain open. |
| `SensorsFixture.HandleRemovedEntities` | Actual loaded default-topic source and zero-step activation; Rust one-camera removal/cancellation controls. The native multi-family Manager test remains open. |
| `SensorsFixture.UpdateRate` | Existing native Sensor-clock/scheduled-output phase contract reused. Multi-family scheduling and camera `set_rate` remain open. |


To reproduce the native observations, materialize each record's exact `input_xml`,
compile its checked-in program using `command_path_projected` with the declared
filesystem aliases resolved to the selected source/build/install roots, and run
its recorded case arguments with the allowlisted backend/configuration settings.
Change only capture/source/output filesystem locations; keep compiler flags,
header order and library selection. The topic program intentionally supplies
`SetName`/`SetTopic` only for its explicitly labeled supplemental case. The
activation program runs inside the recorded immutable image and never invokes
Run/RunOnce. Compare structured results, loaded library identities and source
pins; a new build's ELF hash is its own provenance, not presumed equal to this
capture. Full dependency/raw command data remains in the private capture named
by the record hashes.
