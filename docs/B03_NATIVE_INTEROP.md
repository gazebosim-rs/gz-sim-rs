<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Loaded B03 native Zenoh interoperability source

This completed source batch tests Gazebo C++ interoperability with the Rust
simulator. The simulator uses upstream Rust Zenoh directly, without modification;
C++ is only the external test peer, not a transport implementation to port.

Its parent is accepted simulator Main `8ebb02db`, qualified by the complete
local gate, exact hosted run `41447340434` and independent official audit.
That parent adopts producer `f0d05146` and passes strict fresh-native B03 motion,
serial lifecycle and world-light reset controls. The new interoperability case
has source review but has not executed. Run one consolidated mandatory gate
after final integration review; prior failed trees and captures remain unchanged.

Owned paths are the two server test sources, this document, and the handoff/task
documentation updates. All old Rust tests and C++ modes remain intact. There is no
production, Cargo, protobuf, helper, dependency, fixture or native-profile change.

## Concrete operation and observation

The new C++ --b03-world mode uses the qualified native Node and generated
Clock, Pose_V, WorldControl and Boolean messages. The Rust test loads the exact
unchanged b03_double_pendulum.sdf bytes with URI b03.sdf, world default, DART and
the SDF one-millisecond step. The original XML is copied to retained evidence.
The existing Network supplies an explicit router/client configuration, selected
partition, loopback GZ_IP and disabled scouting. NativePeer selects the explicit
gazebo-zenoh-client-v1 profile; no historical default profile or custom transport
is used.

The existing scripts/native-peer.py and scripts/zenoh_peer.py compile_client path
always read this worktree's current clock_client.cc, copy it into a new clock-*
evidence directory and compile it against the selected SDK/compiler/pkg-config
closure. Its existing provenance binds current source, actual compiler argv,
binary, linkage and selected library hashes and rechecks source/SDK after compile.
The new mode requires that newly compiled, attested peer. An old qualified ELF
without this source/mode is not evidence. No recipe adaptation is needed.

The peer subscribes to /world/default/clock and /world/default/pose/info and calls
only /world/default/control. It must receive actual initial clock and complete
poses before requesting one step. It then waits for actual clock AND pose receipt
at each requested boundary before requesting +9 and +90 steps, reaching1/10/100ms.
Only after observed100ms does it arm reset, obtain the reset RPC Boolean receipt,
and wait for a new zero snapshot, including reset real-time zero. It then requests
one post-reset step and waits for clock and poses1ms. Startup zero is not reused as
reset evidence. Requests use the existing asynchronous request-with-reply helper
and seven-second bound; each receipt phase uses the existing eight-second wait.
The Rust owner drives normal Server::update (including prepare_io outside tick),
with the existing45-second outer lifecycle bound and one-millisecond host yielding.
No retry, fallback or extra warmup motion is introduced.

The loaded world advertises exactly27 pose records: three models, seven links,
sixteen visuals including sunVisual, and one sun light. No collision, joint,
sensor or velocity record is invented. A fixed scoped-name inventory independently
checks those groups. An actual owned-ID/parent/name manifest disambiguates repeated
local link/visual names. C++ requires every ID once, the exact complete groups,
valid protobuf presence/headers/finiteness/quaternion norm, stable record order
and unchanged IDs across reset. Rust also verifies actual public identity ownership
and parents throughout all updates and compares each received protobuf row to the
saved Scene local-pose snapshot at the same epoch/time.

Pose_V carries LOCAL poses and names. C++ reconstructs each body's world pose from
its received link and model parent records using maintained gz-math Pose3d
composition. Expected world position/quaternion values are only the first seven
fields projected from the checked actual fresh-native B03 fixture at1/10/100;
all21 body rows, exact fixture hash, XML hash, ticks and22-field input schema are
validated before supplying them to C++. The unchanged1e-12 bound applies to every
received body world-pose component. The reset1ms pose also compares to the actual
native1ms observation. No expected trajectory is computed from Rust physics,
and no native/Rust velocity parity is inferred from pose messages.

Epoch is not a Clock or Pose_V wire field. The peer's phases are receipt fences,
not invented message epochs. The Rust owner separately requires epoch0 followed
by epoch1, exactly100 initial one-millisecond advances and one post-reset advance,
no extra elapsed motion, retained public IDs/parents and empty pending work. All27
reset local poses must restore the initial values. It verifies five accepted
control receipts, no rejection/response/clock/pose failures, no IMU output, final
paused state at1ms/epoch1 and terminal update refusal after shutdown.

## Evidence and limits

The new case retains the exact XML, native fixture and projected body expectations,
owned manifest, all six received Clock/Pose_V protobufs and text decodes, native
maps and stdout/stderr beneath the existing peer binary's evidence directory.
That nesting follows the existing verifier's native-peer retention. The native
peer is the sole delivery proof; registration, an expected file or a Rust-side
snapshot alone does not establish receipt. The new case creates file-backed
stdout/stderr before child spawn, overriding only its own command's piped streams.
Available actual child writes therefore remain on a Server panic, assertion or
timeout, and pipe backpressure cannot hide native diagnostics. After normal child
exit Rust reads those same files and checks status and output; stderr must remain
empty. Existing helpers, assertion strength and reference inputs are unchanged.

The new C++ peer compilation, wire delivery, loaded reset and required local/hosted
qualification remain UNRUN for this batch. Parent B03 correctness is accepted
separately and does not prove wire delivery. Complete independent integration
review before the mandatory gate. This slice makes no
performance, equal-work ratio, general Manager, rendering or phase-completion claim.
