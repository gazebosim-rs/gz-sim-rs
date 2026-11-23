<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
## Server regression correction after retained coverage failure

The full normal on frozen tree `4084c78a` passed all 2,144 headless tests
(two ignored), then failed the Server region gate at 19,281/20,312, below the
required strict greater-than-95-percent threshold. Renderer verification and
later qualification did not run. This successor changes regression Source only.

Five additional private Server cases cover late-owner identity/body-binding
faults with queued commands, missing or forged controller membership/leases,
removal before Server construction followed by survivor actuation and full
loaded reset, unowned metadata/disconnected ingress, and original world identity
drift. They require unchanged state and no command consumption on refusal,
whole-family retirement where the identity contract requires it, retained keys
for removed originals, and restoration of original IDs with new leases. Existing
cases, inputs, tolerances and production code are preserved. These add five
headless-only declarations; they are not executed passes or a predicted coverage
result. Formatting and consolidated mandatory verification follow completed
independent Source review; no behavioral qualification transfers from this handoff.

# M105: independent multi-robot VelocityControl

M105 supports two through eight original eligible VelocityControl models in one
world, using Rapier or TPE. It preserves the existing single-owner path and the
existing eligible body domains: top-level dynamic free single links, centered
mass frames, and the primitive/posed/compound restrictions of each backend.
The owner selected this phase under D31, with at most five working days including
review and mandatory verification.

The combined candidate starts from qualified commit `08a0eeae` and copies the
reviewed, formatted M102/M103 foundation `93a2d820`. That foundation is unverified;
neither it nor M105 inherits qualification. The ten-phase selection is
`artifacts/m104-m113-delivery/ten-phase-reviewed-source-selection.json` in the
primary checkout. No acceptance, Main promotion or qualification is implied by
this guide.

## Behavior and ownership

Each owner retains its own immutable original model/link identity, initial
command, canonical command topic, committed target and latest valid pending
command. Existing model-frame rotation, ordinary engine stepping and PostStep
latching remain: a paused command does not move a body, the first advancing step
uses the old committed target, and the following step uses the latest target.
Invalid commands preserve the preceding pending value. Zero is a valid target.

Scene validates every live owner and every derived backend velocity before
changing any owner's velocity. Server validates the complete live roster and all
canonical topic reservations before network setup or message consumption. Late
identity drift retires the whole ingress family and faults the run; it is not an
implicit repair or reset recovery. Normal removal retires just that subscription.
The first remaining owner can hold the existing bounded family without changing
its identity. Full loaded reset restores original IDs and fresh callback leases;
time reset retains the existing epoch and lease reconstruction policy.

One global command budget rotates across all live queues, including across calls.
A busy first queue cannot starve later owners. Queues, message size limits, epoch
checks, overload accounting and Zenoh declarations reuse the existing machinery.
An absent system creates no controller, endpoint or polling work.

## Leases and dependencies

Systems owns Scene `velocity_control.rs` and `velocity_control/config.rs`, Server
`velocity_control.rs` and the shared Server `lib.rs`. The latter is also the sole
systems lease for M111. No message, transport, backend or solver implementation
changes are required. New regression leaves are
`crates/gz-sim-scene/tests/m105_velocity_control_fleet.rs` and
`crates/gz-sim-server/tests/private/m105_velocity_control_fleet.rs`; the existing
M79 fixtures are reference context, while the new tests construct bounded SDF
models locally. There are no new fixture files or dependencies.

Root owns the retained same-input multiple-owner refusal disposition in the final
case of Scene `tests/velocity_control.rs`, along with all other retained tests and
shared attachments. Its other refusals and case identity must remain intact.
No unsupported multi-link controller, new body teleport path, dynamic system
registration, new protocol or general transport framework is included.

## Regression obligations and exit criteria

The authored twelve declarations cover two/eight owners on both engines,
independent measured motion and rotated model commands, latest/invalid/zero
commands while paused, preserved Rapier compound geometry and global body queries,
full and loaded reset/removal, late-owner atomicity, capacity and multi-link
refusals, fair bounded queues, actual typed topic ingress, canonical route
collisions, callback retirement and stale epochs. Independent physical arithmetic
uses measured poses and a separate rotation matrix. It does not assert ideal
traction or replace backend movement with requested commands.

There are twelve new headless obligations and six renderer obligations, shared
across the six public Scene cases; the six Server private cases are headless only
under the maintained profile selection. These are declarations, not passes.
Independent whole source review is pending at this author checkpoint. Formatting,
tests, lint, the staged hook, strict coverage, independent retained evidence,
exact hosted CI and official artifact audits are unrun for this combined wave.
Root will execute one consolidated cycle after all ten phases, docs and source
review are complete and fresh resource admission permits it. The existing SSD
headroom hold remains; no target allocation, cleanup, Fuel change or live-demo
restart is authorized by this guide. M77's formal acceptance gate is unchanged.
