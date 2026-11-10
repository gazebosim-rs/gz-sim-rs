<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M42: joint force with measured joint-state feedback

This bounded workflow combines existing ApplyJointForce and JointStatePublisher
on an already supported WORLD-hinge Rapier body. It adds an example, regression
controls and the `force` mode in the existing `joint_command` helper; it expands
no mechanical, publication, transport or backend domain. M41/M42 are accepted
at Main `73e03b9f` over accepted M40 after independent source review, the full
normal hook and strict coverage, exact CI 42808865907 attempt 1 and independent
local/official DATA. See [the acceptance record](M41_M42_ACCEPTANCE.md). The
commands below describe use, not an additional execution receipt. No new preview
package or repeat browser acceptance is part of this wave.

## Example and commands

`examples/m42_joint_force_feedback.sdf` contains world `default`, model `rotor`,
link `body` and direct joint `hinge`. The body has a radius 0.2 m sphere, mass 1 kg,
diagonal inertia 0.016 kg m², centered COM, zero gravity and a 1 ms step. A WORLD
revolute hinge turns about `(0,1,0)`. The example uses the existing model plugin
pairs for ApplyJointForce and JointStatePublisher; the latter runs at 100 Hz.

| Purpose | Topic | Canonical type |
| --- | --- | --- |
| Latest persistent effort in Nm | `/model/rotor/joint/hinge/cmd_force` | `gz.msgs.Double` |
| Full model/joint feedback | `/m42/joint_state` | `gz.msgs.Model` |

Existing partition, namespace and remapping rules apply. Public IDs must be
resolved from the loaded world; their allocator numbers are not fixed here.
The helper's four-argument shape remains unchanged. For a developer build:

```sh
cargo build --locked -p gz-sim-cli --no-default-features
cargo build --locked -p gz-sim-server --no-default-features --example joint_command
```

Use a fresh owned Zenoh configuration with a free listener, for example
`/tmp/m42-zenoh.json5` containing:

```json
{"listen":{"endpoints":["tcp/127.0.0.1:7449"]},"scouting":{"multicast":{"enabled":false}}}
```

Start the server and send a finite positive, coasting and opposite-effort sequence
from another terminal using the same partition:

```sh
GZ_PARTITION=m42 ZENOH_CONFIG=/tmp/m42-zenoh.json5 target/debug/gz-sim-rs examples/m42_joint_force_feedback.sdf --server --run
GZ_PARTITION=m42 target/debug/examples/joint_command tcp/127.0.0.1:7449 force 1.5 1
GZ_PARTITION=m42 target/debug/examples/joint_command tcp/127.0.0.1:7449 force 0 1
GZ_PARTITION=m42 target/debug/examples/joint_command tcp/127.0.0.1:7449 force -1.5 1
```

The maintained `joint_command` force mode is integrated; position and velocity
modes keep their previous topics and behavior. Force uses
finite Nm and the existing finite repeat interval 0..=3600 seconds, and a successful
invocation explicitly publishes zero effort at its end. Zero coasts; it is not
braking or a stopped-state guarantee. Matching or successful Publish establishes
neither Server admission nor physical application. Failure does not guarantee a
final zero. Use existing canonical Model consumers to inspect measured joint
position/velocity; no new message CLI or telemetry endpoint is introduced.
Existing frontend pose visualization can show the body, but this symmetric sphere
is not an angular gauge and screenshots are not the feedback proof.

## Exact feedback and timing

The canonical Model carries an empty-data Header with actual simulation stamp,
model public ID/name and model pose, and direct joints sorted by public ID. Each
joint has its public ID/name, parent/child names, construction pose relative to
the child, axis vector, legitimate negative/positive infinite limits, damping,
and measured position/velocity. All unsupported optional fields remain absent or
default, including parent/child IDs, second axis, links, visuals and nested models.
Axis.force is zero and unmeasured; it is not a command echo or a torque sensor.
There is no invented frame header, sequence, wire epoch or force measurement.

Server polls force ingress after physics and publication. The test waits for
both `force_messages_received` and `joint_forces_applied` to increment once and
for the public JointForceCommand to equal the requested value while paused.
The latter counter records a successful command update, not measured force
application. Position, velocity, time and epoch stay unchanged during admission;
the next advancing 1 ms step applies that command. Subsequent advancing steps
repeat the latest effort until replaced, reset or removed.

JointStatePublisher acquisition remains unconditional. Its native positive-rate
rule suppresses a positive elapsed interval shorter than 10 ms; equal-time
updates still acquire and increment the local revision. Paused updates can
therefore publish duplicate wire timestamps. Tests compare actual
`(epoch,revision)` before and after each controlled Server update, and drain one
actual FIFO Model only when that key changes, checking the full decoded message
and canonical bytes against the actual Scene sample and immutable metadata.
They do not key packets by stamp or require a subscriber to acquire a sample.

The C++ peer is transport-only, built through maintained NativePeer::build with
the selected Zenoh SDK/source/provider provenance. Its finite pipe phases only
coordinate the test. Force and Model values cross the real canonical transport.
Every callback validates the typed message and acquires one bounded shared
mutex before accepting a FIFO copy. Callback queues are capped at 256; a retained
failure stops later mutations. Stdout/stderr are independently bounded to 1 MiB,
phase waits to 8 seconds, callback/exit waits to 5 seconds, and owned failure
retirement retains partial bytes and actual wait or an explicit unknown.

Discovery is not callback completion. The test separately validates an actual
paused 0 Model, then performs an explicit ten-step, zero-effort warmup and waits
for the sole due 10 ms Model on that same subscription. No Server update occurs
while acknowledging strict FIFO arming. After arming, no late packet is silently
cleared or ignored: unexpected data fails exact counts/content. ACK and drain
waits do not update Server. This makes the Server pulse interval 10→20 ms;
the separate Scene native-pulse comparison starts at exact 0. It is not a general
network or cross-topic delivery barrier. No Clock receipt is used as a Model
callback fence. Reconnect/reset use fresh callback leases and a new same-topic
positive due-stamp cutoff before strict recording resumes. Priming does not
claim an exact history for early matching-time messages.

## Regression and lifecycle scope

The new Scene controls retain the six actual existing native effort observations
`[1.5,0,-3,0,2,0]`, one 1 ms step each. All six native velocity comparisons keep
the unchanged absolute 1e-12 bound. The accepted cross-engine fixture qualifies
joint rates; its DART position values are not a Rapier trajectory contract.
Position and velocity are instead compared exactly with a paired Rapier scene
using the identical SDF without ApplyJointForce, driven once per step through the
existing `queue_joint_effort` API. They coast through 10 ms and compare measured
JointSample to the due publication. A second cadence configuration still produces
the same physical state, so feedback rate does not change mechanics. No old native
helper, capture or Physics campaign is rerun or relabeled as new M42 evidence.

The first executed headless batch exposed the new controls' incorrect DART
position comparison in both Scene and Server. The unchanged native-rate controls
passed. The correction retains the native fixture and all six rate assertions,
adds the direct-input Rapier position reference, and changes no inertia, solver,
production API or tolerance. That batch remains failed; the later complete reviewed normal retry
and exact hosted CI passed before qualification.

The public Server/C++ workflow observes actual canonical input and full Model
feedback for those pulses, positive effort, explicit zero coasting and opposite
effort. Its paired direct-input Rapier reference first takes the same explicit
10 ms zero-effort warmup, then receives the same six individual commands and 1 ms
steps. Every pulse keeps native velocity equality and exact measured Rapier
position/velocity equality; full canonical packets continue to match the actual
Scene sample rather than a cross-engine position formula. It checks equal-time FIFO duplicates, due updates, a no-subscriber
interval with continuing Scene acquisitions, fresh reconnected delivery,
healthy loaded reset and removal. Reset discards an actually admitted command,
clears measured publication history, increments epoch and reconstructs original
public IDs with freshly resolved internal handles. The next ordinary paused
update supplies fresh time 0 feedback; the reset transaction itself has no
PostStep sample. The later warmup remains unforced. Removal uses the existing
canonical `/world/default/remove/blocking` service with an actual `gz.msgs.Entity`
MODEL request and the current public model ID. After actual Queryable matching,
the test sends one query, requires a true canonical Boolean reply and exactly one
`removals_completed` increment, and checks that the paused clock and epoch did not
change. It then retains all command/publication retirement and loaded-reset
reconstruction assertions. The narrower `remove_editable_model` API continues to
refuse joint-bearing owners with typed `WorldEditError::UnsupportedModel`; the
test verifies no entity, sample, counter, clock or pending-step mutation from
that refusal. This avoids the same API mismatch exposed by the wind Server test,
without expanding the world-edit domain. Actual native peer exit 0, empty stderr,
owned capture completion and Server shutdown/WorldClosed are required.

Existing private `unread_native_force_queue_is_discarded_when_subscriptions_restart`,
`shared_force_ingress_survives_one_consumer_then_discards_last_unread_command`,
`real_running_callback_keeps_only_bounded_release_witness_after_io_retirement`
and the existing full/malformed/late-callback controls remain unchanged evidence
for unread receiver queues, capacity and in-flight lifetime. This slice does not
add a private hook or claim that reset cancels packets still in a remote
publisher/network: force messages have no wire epoch. Old callbacks cannot
supply a new subscriber lease's accepted records.

The moving-parent Rapier effort refusal remains explicit. The new Scene control
loads the same publication workflow without ApplyJointForce on a moving-parent
joint and checks the existing typed effort refusal without command installation,
state change, clock change or revision change. It does not replace that refusal
with a world-space couple, torque pair or new solver behavior. World/fixed-parent
admission, existing nonfinite/overflow boundaries and all old native controls are
preserved. No performance, general joint-force migration or new force-sensor
compatibility claim is made.
