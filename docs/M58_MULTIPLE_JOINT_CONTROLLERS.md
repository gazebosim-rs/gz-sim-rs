<!-- Copyright (C) 2026 Open Source Robotics Foundation; SPDX-License-Identifier: Apache-2.0 -->
# Independent loaded joint-controller owners

M58 extends the accepted M9 owner count to at most two loaded model chains and
four input streams. Implementation and regression Source are reviewed. Fresh
family-isolated original-object captures, independent raw DATA, fixture export
and independent literal DATA have passed. The fourth full normal passed strict lint, 404 Python checks, both doc-test stages
and native-free/RGB qualification, then stopped before full profiles on a cached
Zenoh probe dependency-map mismatch after the recorded libc update. The repair
is verification-only: separately attest the same original probe and explicitly
bind that new proof while preserving the old SDK, manifest and probe. The
reviewed bounded operation returned genuine external 0; independent retained
review confirmed five successful stages and the complete current library map.
The separate selection is ready for the required full normal retry. Full-profile
coverage, independent product results DATA and qualification remain unpassed.

The original retrieval/static/compile failures, unsafe original-DSO refusals,
earlier capture and failed audit remain preserved. The current reference uses
one unchanged object and two owner instances per helper, with a separately
reconciled library snapshot after an installed libc6 upgrade. It establishes
component observations, not original cross-family execution or Rust-servo parity.
M57 qualification and reference evidence do not qualify M58 product behavior.
See [the delivery contract](M58_DELIVERY.md) for evidence and release holds.

## Fixture and command identities

[The fixture](../examples/m58_multiple_joint_controllers.sdf) contains two copies
of the M9 chain, `arm_left` at `(0, -1.5, 1)` and `arm_right` at `(0, 1.5, 1)`.
Each has canonical root link `base`, child link `tip`, a world-to-base
`base_hinge` and base-to-tip `tip_hinge`. Joint axes are +Z, raw link rotations
are identity, anchors are zero in child frames, and gravity is zero. Geometry,
inertia, step size and motor parameters retain the M9 values.

Each configured joint consumes one canonical `gz.msgs.Double` with its exact
canonical type attachment. The fixture defaults are:

| Owner | Position target, radians | Velocity target, radians/second |
| --- | --- | --- |
| `arm_left` | `/model/arm_left/joint/base_hinge/0/cmd_pos` | `/model/arm_left/joint/tip_hinge/cmd_vel` |
| `arm_right` | `/model/arm_right/joint/base_hinge/0/cmd_pos` | `/model/arm_right/joint/tip_hinge/cmd_vel` |

Existing partition qualification and explicit topic remapping apply once. Full
saved names such as `default::arm_left::base_hinge` distinguish repeated local
names. The accepted M9 sender's fixed default owner is unchanged; its old
`/model/arm/...` topics do not address these two owners. No new public CLI flag,
command acknowledgement or controller output protocol is added.

The authored maintained C++ peer uses the selected `multiple_joint_controllers`
client and an owner-count argument of `1` or `2`. Its bounded test protocol maps
slot `2*owner` to position and `2*owner+1` to velocity. `send SLOT WORD` carries
an exact finite binary64 word and returns the actual encoded Double bytes;
`matched OWNER` and `retired OWNER` observe both of that owner's typed endpoints.
`finish` terminates the owned peer, whose actual exit must be checked. These
Source protocol records and discovery messages are not an execution result or an
actuation acknowledgement. Registrations are source-complete; every real peer invocation remains part
of consolidated product verification.

## Supported physical domain

Each owner is a top-level dynamic Rapier model with exactly the existing two
links and two revolute joints. One or two distinct joints may be configured;
there are no more than two owners or four configured slots. Configured effort
and velocity limits are finite and positive; the fixture uses effort `3` and
velocity `4`. Position uses explicit `use_velocity_commands=true`, optional
`joint_index=0` and positive `cmd_max` (`2` in the fixture). Velocity uses default
or explicit `use_force_commands=false`.

Position targets are continuous angles, including multiple turns; there is no
shortest-angle wrapping. Velocity commands clamp to the configured target-speed
bound. Position retains M9's damped reduced-coordinate servo and conservative
per-step waypoint admission, with stiffness `100 s⁻²`, damping `20 s⁻¹` and the
same finite torque cap. These are target policies, not hard physical speed caps
or joint stops. Coupled dynamics can cause tracking error or overspeed. No motor,
backend, solver, arithmetic or timing change is part of multiplicity.

Unconfigured joints retain infinite effort/velocity bounds and zero dynamics.
Finite position stops, nonzero damping/friction/springs, unsupported axes/DOFs,
force-PID/actuator modes, shared actuator ownership, nested/static/other-backend
controller models and expanded topology remain refused. A captured native
metadata value does not authorize an unsupported loaded physical graph.

## Ownership, ingress and lifecycle

The original borrowed identity includes stable joint ID, kind, world-scoped
name, original parent model ID and configuration. The immutable blueprint keeps
removed originals; missing initial state differs from a genuinely empty loaded
controller set. Runtime metadata, a plausible handle or free numeric capacity
cannot enroll another owner.

Scene validates the complete live set against that blueprint, actual typed model
parents and native joint/child binding before target mutation. Before advancing,
every motor request and native handle is validated before any request is set.
The old finite/error/clamping rules remain. In particular, invalid commands
preserve the previous pending target, and admission does not teleport either
chain's poses or velocities.

Server preserves the existing exact type/size/decode/finite checks, per-stream
bounded capacity, command budget and acquisition epoch. At most four queues can
consume at most four times the existing per-stream budget. Malformed or stale
handoffs consume rejection budget; overload remains observable in existing
counters. No new thread, unbounded shared queue, reset epoch on the Double wire
or output-interest requirement is introduced.

Complete original/current/retained/prepared identity and endpoint checks precede
IO or generation mutation. Retained leases bind old declaration keys; prepared
leases bind new remapped keys. Legitimate remap reset therefore accepts a valid
old generation while requiring the new batch's exact endpoints. Distinct Arc
swaps and duplicate Arcs fail independently. Root's shared boundary keeps
historical constructor/reset collision winners across existing endpoint families.

Every surviving stream is reconciled before any queue is read. Live typed
identity/name/parent/configuration/handle drift is terminal for the family;
subsequent admission keeps the existing terminal error behavior. Full entity
absence or pending removal instead retires that stream. Use coherent public
model removal to demonstrate continued peer physics; a raw partially damaged
graph is not a supported next-step fixture. The last owner releases controller
callbacks, declarations, queues and leases.

Ordinary reset preserves its existing pause semantics. Loaded reset reconstructs
removed original owners and IDs, restores configured initial targets and retires
old queued work/callback captures before replacement admission. Reset's original
per-query-row overflow check and native validation order remain: an earlier
missing joint wins over a later counterfeit extra controller. All clears and
target restores follow the complete preflight. The old counterfeit-control ID,
its actual query-match diagnostic and all neighboring M9 negatives are retained.

## What the authored controls establish

Scene Source exercises both real chains with opposite signed velocity targets
and distinct continuous multi-turn position targets, checks actual backend
coordinates and corresponding body orientation at the unchanged M9 bounds, and
keeps pause/reset/removal/stable-ID assertions. Private cases cover absent,
single, pair and mixed configurations, third-owner/late-invalid refusal, complete
live corruption, global validate-before-mutate, and simultaneous reset faults.
No output estimator, pose write, body-velocity edit or assumed query spawn order
supplies these witnesses.

Server Source combines actual canonical Double bytes and counters with separate
backend movement assertions. Its negative controls cover full-set native/identity
refusal, retained/prepared swaps and duplicates, valid remap restoration, epoch
fences, per-stream budgets, no polling on live corruption and independent
retirement. Existing M9 tests and assertions remain unchanged. Actual passes,
coverage and final declaration counts await the consolidated verification and
independent retained results audit.

The original input boundary and reference pipeline remain separate from Rust
product evidence. Both helpers use genuine public Configure/PreUpdate and the
position helper also uses the public ParametersRegistry/ConfigureParameters
path. They preserve original raw velocity commands and ABS lowering separately
from Rust clamping and its damped servo. Capture, raw audit, fixed export and
literal audit each completed with an actual zero Root result after review.

The adopted Scene control maps captured owner/local names to genuine loaded
identities and passes only observed command inputs to the public setter. Peer
targets, all physical state and clocks must remain unchanged. The Server control
sends original Double bytes through real canonical Zenoh ingress, checks exact
selected-stream values and epochs, then application/rejection counters. Wrong
type, truncated and oversized packets are explicit synthetic Rust negatives.
Native positions, velocities and clocks are never installed into the Rust world.
These authored controls still require their first consolidated product run;
source-to-binary equivalence and original Server/solver/trajectory parity are not
claimed.

## Private binding witness

`joint_controller_binding_matches(&Simulation, id)` is a pure per-active-ID
witness for the loaded private controller owner, native joint binding and public
configuration. Server admission and retained-stream reconciliation call it before
IO, polling, counters or targets change. Callers retain the complete public
identity, parent/child, native-body and input-set checks; the witness does not
replace them or actuate a joint.
