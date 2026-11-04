<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M8 accepted: live differential drive and encoder odometry

Accepted and published on 2026-11-04 at simulator Main
`fc9673881bae5b0c28e8e22fe2ab013d3f87f117`, tree
`7f5a6e24b8077a1c57d8598262ee76b41d29ec93`, parent `4a985bfb`.
The required normal staged gate passed in 1163.291 seconds. Exact hosted
[CI41776261368](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41776261368)
passed all 16 steps on attempt 1. Independent source, retained local/official
profile and actual browser image reviews cleared before nonforced Main promotion.
M9 joint controllers are now the current five-working-day delivery. Acceptance
metadata follows its normal hook/CI separately; accepted manual work is reused.

## Runnable increment and limits

[Launch instructions](M8_DIFF_DRIVE.md) start one native-free Rapier robot with
one centered box chassis, two sphere wheels and a static box ground. A loaded
DiffDrive configuration owns one direct-model revolute wheel pair. Both wheel
axes are +Y, forward is +X and yaw is +Z. Finite canonical Twist commands enter
the existing bounded upstream Zenoh owner; the next advancing step updates
maintained Rapier velocity motors. Body motion comes from the native motor,
joint and contact solver. There is no body-velocity teleport or custom solver.

Body command bounds and finite wheel target speeds are enforced before mutation.
The torque cap is finite Nm; zero targets brake within that cap, and explicit
disable clears motor state. Both physics reset forms and removal retire runtime
motors. These target bounds are not hard physical speed caps against external
impulses. Existing moving-parent effort refusals and unsupported-backend errors
remain intact.

Canonical Odometry integrates actual continuous encoder coordinates from the
loaded/restored time-zero reference, including the first advancing tick. It uses
local x/y/yaw, a ten-sample measured velocity window, integer timestamps and
configured frame names. The demo publishes at 50 Hz. Encoder tracking and
actuation do not depend on odometry listeners; matching controls output only.
Pause does not acquire another sample. Reset clears commands/history, replaces
input/output generations and restarts sequence. The documented viewer reset
requests `pause: true` and leaves the loaded world paused.
Absent systems own no drive callbacks, command channel or sample buffer; last
model removal retires the family and loaded reset reconstructs it.

The admitted subset explicitly refuses unsupported layouts, topics, command
axes, acceleration/jerk controls and conflicting wheel ownership. General
articulations, Ackermann, tracked vehicles, TF output, mimic/transmissions and
full drivetrain parity remain deferred. No new C++ DiffDrive parity, performance
or slip-free claim is made. Upstream Rust Zenoh remains unmodified; the retained
C++ programs are external compatibility/reference peers, not the runtime.

## Required checks and independent evidence

All owned crates and each separate workspace profile exceed 95% lines and
regions; changed-line gates also pass separately. No hit-set union is used.
Both runs passed 1,094 headless, 688 renderer and 289 tooling tests; the separate
five/four delegated ignores remain explicit. New controls cover actual moving
parent contacts and turns, limits/braking/disable, measured encoders and frames,
canonical input/output, malformed commands, cadence, pause/reset, removal,
no-listener actuation, absent families and terminal binding/clock faults.

| Profile | Local lines | Local regions | Hosted lines | Hosted regions |
| --- | --- | --- | --- | --- |
| Headless workspace | 14814/15142 | 22178/22925 | 14814/15142 | 22179/22925 |
| Renderer workspace | 8900/9120 | 13131/13702 | 8902/9120 | 13133/13702 |
| Headless Scene | 4063/4160 | 6261/6487 | 4063/4160 | 6261/6487 |
| Renderer Scene | 4030/4160 | 6167/6487 | 4030/4160 | 6167/6487 |

The hosted headless difference is one Server region; renderer differs by two
worker lines/regions. Parent-changed Scene is 882/905 in both profiles; Rapier is
99/99 and physics 20/20. Server is 477/498 locally and 475/498 hosted, both above
95%. Hosted zero-base changed-source measurements are distinct from these
parent comparisons. The original gate failures remain preserved: compile/lint
failures, one malformed no-system test fixture and the real renderer Scene
6123/6487 region failure. Corrections retained all assertions and gates; meaningful
fault/admission regressions addressed the coverage failure. Closed failure
packets were losslessly compressed with every path, byte and mode verified.

Independent audits verified 2,368 local artifact hashes, 1,285 hosted source
blobs/modes and 592 distinct source/input bindings (90 production files and 272
profile inputs). Four data-only raw merges reproduce retained merged bytes.
Hosted raw counts are 1749/358/14/33, versus local 1749/354/14/33. Original
native-free proofs, separate camera feature proofs, inherited five depth/four RGB
and 34 support controls pass. Six auxiliary ELF copies are retained; 288
full-profile Rust executable records are metadata-only and their original files
are gone. CLI/worker normal build/test source and hash bindings remain explicit.
Nine existing C++ peer source/ELF records and 24 remote SDK library identities
are manifest-bound; remote DSO bytes were not archived or locally rehashed.

The official artifact ZIP SHA256 is
`789e657c74cfba80fabb72ee0f2051d4472c09c64d67c3605204900d0c62b8a6`;
source ZIP is `dc5a90b61dc5a822293f975c7fe7f0645983026b0d3a239affdfb2fb469870e8`.
Independent official audit summary is
`164baee0765a64f3957a59d490a47285a22e4999f8dd674dfcb5eab63ba20223`.

## Actual browser observations

The qualified retained native-free CLI SHA256
`cd87bff4a33a68db7b89296e6340a241f40c58c3a2fbaf870bcc8a158d774c77`
ran the committed demo (SHA256
`d3eba91bc124b8ee50cf3be80c5018e40ef44b759cb5b987dbee93cba20a4c73`).
Accepted viewer `19be842d` release bundles were reused unchanged; viewer Main
`516a9a0e` adds only closeout documentation and a deterministic test clock. Both
existing bundle budgets and prior manual qualification remain unchanged.

Five independently inspected images show four visual rows, forward translation,
changed chassis/wheel orientation after real turning commands, pause acknowledged
at epoch 0/time 71.123 seconds, and reset acknowledged at epoch 1/time zero with
the original pose restored. The before-command image was captured at 13.200
seconds, forward at 33.094 and turned at 63.547; it is not a time-zero image.
The preview remains paused on local socket 9008.

The finite canonical sender was built once for this runnable observation and
ended each successful command with zero-target braking. Its actual retained ELF
is 34,385,560 bytes, SHA256
`af9acd67dff24dad07cef8aa67a1b0a64227824f3a9d37e3efc13daac744f80b`.
The complete normal-build Cargo JSON was not retained: the 40.01-second build,
command exit codes and initial reconnect are coordinator-recorded observations,
not reconstructed independent provenance. Sender bytes and source, the qualified
CLI, fixture, viewer bundle and all five images were independently checked.
Screenshots prove visual motion/control behavior; quantitative units, cadence,
odometry and limits come from separate passing automated controls. No new
adapter-specific or frame-rate claim is made. No accepted M1–M7 manual browser
or runtime qualification was replayed.
