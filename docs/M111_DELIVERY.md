<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
## Actual-worker publication correction

Both real-worker cases timed out while reading an image after stopping at
`FrameReady`, which retains the received frame for the next pump transition.
Their shared helper now pumps every completed owner exactly once more and
asserts `ImagePublished` before the existing image/info reads, including after
full and time-only resets. Simulation ticks, timestamps, packet assertions and
all deadlines remain unchanged. This is a test-driver correction; independent
Source review and the consolidated verification cycle remain required.

## Server ownership regression correction

The frozen combined normal passed all 2,144 headless tests (two ignored) but
failed Server's strict region-coverage gate before renderer verification. The
application and real-worker cases below remain unchanged. Three new private
headless Server cases independently exercise the external three/four-RGB trust
boundary without constructing a renderer: complete unordered IDs with actual
clock/step/reset and world-edit refusal, malformed or incomplete membership
refused before network setup, and unchanged scalar/pair/depth/mixed admission.
They reuse the existing RGB fixture and occupied-listener witness, retain all
production semantics, and distinguish Server identity ownership from real image
output qualification. The new declarations supply no pass or coverage claim.
Formatting and one consolidated mandatory verification cycle follow completed
independent Source review; all existing gates remain required.

# M111: explicit three/four RGB camera applications

M111 adds an opt-in application family of exactly three or four loaded RGB
cameras. It reuses the existing renderer clients, per-camera schedules, writer
scopes, bounded output state machines and world Zenoh session. Existing scalar,
RGB-pair, depth-pair and mixed-pair constructors retain their admission, error,
absence and lifecycle behavior. The phase has a maximum budget of five working
days including review and mandatory verification.

The selected combined wave is based on qualified `08a0eeae`, with the exact
reviewed but unverified M102/M103 foundation `93a2d820`. The authoritative scope is
`artifacts/m104-m113-delivery/ten-phase-reviewed-source-selection.json` in the
primary checkout. This phase inherits no test result, qualification or acceptance.

## Explicit application contract

`CameraServer::new_rgb_family_with_server_session` requires one configured sensor
world and exactly three or four eligible RGB sensors. Every identity, ancestry,
calibration, rate, worker configuration and canonical image/info route is checked
before world IO is opened. Topics must be distinct across the complete family.
Depth and mixed families remain outside this new constructor. Unsupported image
sizes, dimensions, materials or sensor domains remain unsupported.

The installed CLI uses the explicit `--camera-rgb-family` flag with
`--camera-worker`; it conflicts with `--camera-mixed-pair`. Root owns that parser
attachment. Without the flag the existing constructors are chosen unchanged.
There is no implicit promotion, dynamic sensor registration or additional session.

Use `tick` then `capture_all`, or the existing per-camera pump/demand methods.
Scalar capture/pump/update APIs refuse the multi-camera family before advancing
physics. Whole-family binding validation precedes output visitation; a Busy or
failed earlier output does not prevent later healthy owners from being visited.
Each camera retains its own cadence, demand, buffered acquisition and worker.
No-demand/info-only operation creates no renderer worker. Output backpressure
never repeats an already committed simulation tick.

Normal removal retires only removed outputs and leaves the shared world session
alive. Original family intent survives a shrinking live subset. Epoch reset
retires the old outputs before rebuilding the same original ID set; missing or
replaced identities are errors rather than fallback registration. Full loaded
reset can restore removed sensors. A time-only reset does not promise to recreate
missing sensors. Application shutdown retires all workers and closes its owned
world Server; merely retiring camera outputs does not close the shared session.
Partial construction drops earlier outputs and their writer reservations.

## Leases, fixtures and exclusions

Systems owns `gz-camera-output/src/application.rs` and `src/lib.rs`,
`gz-sim-server/src/lib.rs`, and `gz-sim-cli/src/live.rs`. Root owns the bounded
`main.rs` flag/argument hook. Server lib has one writer shared with M105.
New tests are `gz-camera-output/tests/private/m111_rgb_family.rs` and
`gz-render-worker/tests/m111_rgb_family.rs`. They reuse the maintained camera
fixtures and isolated network helper; no dependency or fixture is added. Real
worker cases use Cargo's exact `CARGO_BIN_EXE_gz-render-worker` binding.

This phase does not enlarge images, change rendering geometry or shaders, add a
transport/runtime framework, support arbitrary family size, or broaden existing
external-camera constructors. M109/M110 separately own new rendering geometry;
M111's tests retain the existing calibrated RGB box fixture.

## Regression obligations and exit criteria

Six private application cases cover cardinality/type/topic refusal and legacy
boundaries, independent three/four-owner info cadence without workers, subset
removal and full reset, partial-install cleanup, late binding drift, and a busy
first owner with three healthy later outputs. Two real-worker integration cases
cover three/four independent RGB and CameraInfo packets, exact fixture pixels,
unique workers, complete retirement, subset removal, full reset and time reset.
These eight declarations are renderer-only under the maintained profile plan;
no new headless, Python, finite-CLI or native reference case is claimed.

Source implementation, regression code and this guide are written. Independent
whole source review is pending at this author checkpoint. No formatter, lint,
test, build, worker invocation or qualification operation has run for the wave.
Root will release one consolidated final cycle after all feature/docs/source
reviews and fresh resource admission. Required strict coverage, staged hook,
local retained evidence, exact hosted CI and official audits remain unchanged.
The SSD headroom hold, protected Fuel work, frozen evidence and persistent M77
demo/formal acceptance gate remain intact.
