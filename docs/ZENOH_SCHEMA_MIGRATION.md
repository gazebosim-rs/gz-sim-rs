<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Direct Zenoh and schema-only Messages

D28/D29 are owner decisions. They replace the transport and Messages runtime
rewrite, including work already accepted under the old scope. Stop adding
ZeroMQ compatibility, generic message runtimes, reflection/conversion frameworks,
standalone message/transport CLIs and custom codec optimizations. Do not move
those projects into the simulator under new names.

The target has four repositories: gz-sim-rs, sdformat-rs, gz-msgs-rs and the
independently owned gz-fuel-tools-rs. Use the maintained Zenoh Rust crate directly
at simulator networking boundaries. Focused application modules can handle topic
names, domain commands and bounded queues; they must not recreate gz-transport.

## Schema distribution

Keep canonical `.proto` definitions, complete imports including custom options,
pinned schema identity, notices, Apache-2.0 licensing and concise usage guidance
in gz-msgs-rs. Minimal standard schema validation is appropriate. Remove the
repository's Rust runtime, generated bindings, vendored runtime patches,
reflection/conversion framework, CLI and codec benchmarks from the release tree
after consumers have migrated. Preserve Git history and existing evidence.

Each consumer pins schemas and maintained code-generation tooling and generates
its bindings locally. Share the generated types within that consumer's build;
do not generate several incompatible copies for its workspace crates. Validate
declared syntax, field presence, enums, options, imports and field tags against
the native reference. The audited canonical set has 216 proto3 files and an
external import of google/protobuf/any.proto; synthetic proto2/default/extension
runtime tests do not expand this schema-only release scope. A compiling generator does not establish
behavioral equivalence. Domain helpers such as simulator time conversion belong
with their domain; they do not justify a general Messages successor.

## Network migration

The owner confirmed interoperability with Gazebo's C++ Zenoh backend, including
its topic keys and service contracts. Match that pinned implementation and verify
the boundary with actual C++ clients before fixing a public wire contract.
Plain protobuf payloads on arbitrary Zenoh keys do not prove native compatibility.

The inspected pinned C++ backend at gz-transport 41913b9 uses qualified keys,
message-type attachments and discovery/liveliness metadata. Its publisher may
withhold remote traffic unless it discovers the subscriber. Its service failure
behavior also differs from the old ZeroMQ envelope. Confirm all details in
source and real bidirectional probes before implementing consumers. A native
build with Zenoh disabled is not a Zenoh interoperability reference.

1. Pin the Rust Zenoh version/features and compatible native Zenoh reference.
   Record configuration, discovery, partition/namespace/topic encoding, type
   metadata, service requests/replies/failures and endpoint lifetimes.
2. Generate the simulator's required messages from the pinned proto tree. Check
   the native wire fixtures and exact existing domain assertions without relying
   on the discarded custom runtime. Report unsupported generator semantics.
3. Migrate clock, poses, IMU, camera output, world control, removal, rate and force
   endpoints to direct Zenoh. Keep network waits outside the simulation step,
   finite ingress budgets, explicit overload accounting and command phase order.
4. Rehost retained C++ peer scenarios on the selected Zenoh contract. Preserve
   pause/reset/removal/fault assertions, queued-message epochs, type mismatches,
   bounded buffers, multiworld isolation and clean shutdown. Demand-only systems
   must release Zenoh endpoints and owned state after the last consumer.
5. Remove the simulator's gz-transport and packaged gz-msgs runtime dependencies
   only when the replacement paths pass independent review, normal staged gates,
   strict owned line/region coverage, exact hosted CI and source-bound evidence.
6. Publish the schema-only repository tree and switch consumers to its pinned
   revision without relying on a deleted moving-branch dependency. End the
   transport repository's release role; preserve historical source/results.

Schema-only definitions have no executable coverage denominator. Validate them
with standard tools and consumer conformance tests; do not claim an empty runtime
passes 95%. Owned simulator/codegen integration code retains its real coverage
gates. Generated dependency exclusions stay explicit and reviewed.

## Progress and test dispositions

Record ZeroMQ implementation-only tests and discarded message tooling tests as
owner-approved scope removals, with D28/D29 and original identities. Retained
simulator behavior and protobuf semantics still require tests; do not delete or
weaken their assertions to ease the migration. Keep historical measurements and
accepted commits labeled with their original implementation and configuration.

The old P2 and C++ client results remain historical achievements. Add an explicit
Zenoh integration gate rather than silently relabeling them. Rebaseline networking
performance with matched Zenoh configuration and equal requested work. No speedup
or revised completion percentage follows merely from reducing scope. Physics,
sensors, Bevy rendering, SDF/Fuel integration and built-in system activation
remain in scope.
