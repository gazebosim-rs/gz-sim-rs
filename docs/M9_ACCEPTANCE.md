<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# M9 accepted: live joint position and velocity control

Accepted and published on 2026-11-04 at simulator Main
`0f0f031e258da6d8c8f7de17fc694295431addb6`, tree
`51a56955428dc423edf1cece58533cbde83c7611`, parent `bddbb169`.
The normal required staged gate passed in 1411.351 seconds. Exact
[CI41817794153](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41817794153)
passed all 16 steps on attempt 1. Independent source, retained local/official
profile and actual browser image reviews cleared before nonforced Main promotion.
M10 interactive world lifecycle is now the current five-working-day delivery.
Acceptance metadata follows its normal hook/CI separately; accepted manual work
is reused.

## Runnable increment and limits

[Launch instructions](M9_JOINT_CONTROLLERS.md) start one native-free Rapier world
with a two-link, world-rooted revolute chain. Real canonical `gz.msgs.Double`
commands use the existing bounded upstream Zenoh owner. One joint accepts a
continuous position in radians; the other accepts a velocity in radians/second.
Commands replace pending targets without stepping, including while paused.
Actual native joint coordinates and body poses supply state and visualization.
There is no body-velocity teleport, user-space solver or dynamic system plugin.

Both M9 joints use maintained reduced-coordinate construction, ordered from the
world root while public IDs retain XML declaration order. Position control uses
Rapier's acceleration-based native position motor with fixed stiffness 100 s^-2
and damping 20 s^-1. A bounded waypoint uses the maintained zero-step ERP limit;
its prepared start-of-step positional velocity bias respects the configured
speed bound. The actual next-step duration, finite products and represented
waypoint rounding are checked before mutation. Native coordinates can evolve
during substeps. Velocity control uses an explicit native zero-compliance
velocity constraint. Both modes retain finite Nm torque caps. These are target
bounds, not hard physical speed limits or joint stops. Finite torque and coupled
loads can still cause tracking error. The accepted M8 compliant motor policy and
all retained moving-parent effort refusals remain unchanged.

The demo proves three continuous position targets, including multiple positive
and negative turns, under the unchanged 0.01-radian / 30,000-step assertion while
the second joint tracks 0.5 rad/s. Independent physical body-orientation checks
retain their original bounds. Reversed XML declarations preserve the intended
physical chain and stable public identities through reset. This is a bounded
Rust position-servo policy, not a claim of native ABS lowering/order or general
controller parity. Impulse-joint position servos explicitly refuse turn-aliased
quaternion semantics.

Each loaded controller owns one bounded command stream, at most two streams.
Malformed, oversized, mistyped, nonfinite or unrepresentable commands are refused
without replacing the last target. Identity drift is terminal. Every native
request is prepared and validated before any target is applied. Reset drops old
input leases and callback captures before creating a new generation, disables
native runtime motors and restores configured initial targets. Ordinary Scene
reset preserves pause state; the viewer reset explicitly requests pause. Removal
retires callbacks and input leases after the last consumer. An absent family owns
no controller callbacks, buffers, workers or endpoints.

General articulations, physical joint stops, mimic/transmission semantics, force
PID and actuator-message modes, extra controller families and dynamic plugins
remain deferred. No new C++ controller parity or performance acceptance is
claimed. Zenoh is upstream Rust and unmodified; Messages remains schema-only.

## Required checks and retained evidence

The local pass contains 1,146 headless, 716 renderer and 289 tooling passes.
Five/four delegated child entries remain explicit ignores. The 52 new headless
and 28 new renderer named controls each execute once in their respective profiles.
They cover real actuation, continuous targets, limits and native torque caps,
parent reaction, canonical input, pause/reset/removal, absence, malformed identity
and numeric states, constructor atomicity, pending-work conservation, callback
retirement, shared endpoint guards and rejected SDF capabilities.

| Profile | Local lines | Local regions | Hosted lines | Hosted regions |
| --- | --- | --- | --- | --- |
| Headless workspace | 16135/16490 | 24046/24845 | 16131/16490 | 24042/24845 |
| Renderer workspace | 9594/9834 | 14123/14750 | 9594/9834 | 14123/14750 |
| Headless Scene | 4767/4874 | 7289/7535 | 4767/4874 | 7289/7535 |
| Renderer Scene | 4724/4874 | 7159/7535 | 4724/4874 | 7159/7535 |

Every owned crate and separate workspace profile strictly exceeds 95% lines and
regions. Changed-line gates pass separately: Scene 683/691 in both profiles;
headless Server 394/408, Rapier 164/165 and physics 42/42. No hit-set union or
threshold reduction is used. Earlier real failures remain preserved: a test API
spelling error; two position-tracking failures; region failures at 7130/7535 and
7154/7535; two trait-object allocation errors; and three new regression assumptions
about ECS traversal order, the third query match and the earlier COM refusal.
Corrections retain exact identity mappings, fault identity and numerical intent;
COM cases now require both the unchanged Scene and exact native refusal boundaries.
Closed evidence was losslessly compressed with every path, byte and mode checked.

Independent local audit checked 2,371 artifact hashes, 610 source/input bindings,
97 production files and 272 profile inputs against Git, plus 195 Cargo target
source bindings. The official audit also checked 1,307 Git blobs/modes and 2,376
official files with the same 610 bindings and 37 feature paths. Four retained-data raw merges reproduce bytes exactly, with
1752/354/14/33 local raw inputs and 1752/358/14/33 hosted inputs. The four
additional renderer raw files are recorded, not merged into another profile. Separate native-free and camera feature proofs retain
six actual auxiliary ELF copies. The 291 full-profile Rust executable records are
metadata-only; their original paths are gone. Renderer CLI/worker normal build
and execution hash bindings remain explicit. The nine existing C++ peer source,
ELF and provenance chains and 24 local SDK library files were checked; official
remote audit verified 24 library identity records against 36 SDK manifest matches
and eight SDK source inputs. Those remote DSO bytes were not archived or locally
rehashed.

The local audit summary SHA256 is
`ab7a1f71a7e10db21a34184488976322393295a132eefdae5dbf9e568497270b`.
The official artifact ZIP SHA256 is
`9a6fb4aa17e97a0f52769b82474b0f024e077405e0cbaa69446d701554b50f65`;
source ZIP is `c3f264fb6264593c7cbde8f7850e5d82a0fdd5cb381bc4b3862ad1c9d77a66b1`.
The independent official audit summary SHA256 is
`45a29503f740f93c06762df71f861abddd22a7b843b3f3125dc2e5cdc2b8beb7`.
Hosted headless differs from local by four Server lines and four Server regions:
3922/4077 lines and 5652/5936 regions hosted, versus 3926/4077 and 5656/5936
locally. Renderer workspace and both Scene aggregates match. All still pass the
strict separate gates; hosted zero-base source measures are distinct from the
parent comparisons above.

## Actual browser observations

The qualified retained native-free CLI SHA256 is
`6537e1a50aa9041b8b67e41d04e6bc4437b7ef134307a32b0c2771c003c1c99c`;
the committed fixture SHA256 is
`8769c8edb49583659af1dd062f6ca9113cded3b703fcc791affd9ecbf286c8d1`.
Accepted viewer `19be842d` release bundles are reused unchanged; viewer Main
`516a9a0e` adds only documentation and a deterministic test clock. No viewer
rebuild, adapter change or replay of accepted M1–M8 manual qualification occurred.

Five independently viewed images show three visual rows, the initial aligned
bars at time zero, rotated base at 29.680 seconds after the real position command,
changed child orientation at 56.417 seconds after the real velocity command,
pause acknowledged at epoch 0/time 59.315 seconds, and reset acknowledged at
epoch 1/time zero with the original aligned bars restored. Resume acknowledgement
shows epoch 0/time 0.001 seconds. The new preview remains paused on socket 9009.

The finite canonical sender was built once after local qualification, in 26.992
seconds. Its complete normal Cargo JSON, command/environment, stderr and result
are retained. The actual retained ELF is 34,406,440 bytes, SHA256
`e845c220dacc18c728c461547355d05ac2b4a60709e16f1709c6a711953f017b`.
The position command was 0.7 radians, repeated for one second; the velocity command
was 0.5 rad/s for three seconds and ended with the documented zero-target brake.
Both completed with exit code zero. Command invocations/exits and control timing
remain coordinator tool observations, not independent execution logs. Actual
images and source/build/executable bindings were independently reviewed.
Screenshots establish visible motion/control/reset, not numerical angle precision,
encoded public IDs, C++ parity, performance or a new clean-install qualification.
