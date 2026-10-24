# Independent repository implementation contracts

These repositories remain in `gazebosim-rs` and end in `-rs`. Their Rust package
names need not contain that suffix. They must build and test without the
simulator, Bevy, the new GUI, or workspace path dependencies. Apply the same test
traceability, >95% handwritten production Rust coverage, concise-code policy,
and pre-commit/CI requirements in each repository.

Each repo owns its upstream test ledger and baseline. gz-sim-rs owns a pinned
integration manifest and cross-repo tests. Generated schema code can have a
documented coverage exclusion; handwritten conversions and policies cannot.

## sdformat-rs

Use an XML library for syntax, then implement the SDF specification's actual
semantics: schema versions and conversions, defaults, typed values and errors,
includes, merge behavior, relative paths, scoped names, nested models, canonical
links, pose `relative_to`, frame attachment graphs, joint frames/axes, plugins,
sensors, inertial data and automatic inertia, extensions, and serialization.
Inventory URDF conversion and other import paths before declaring parity.

Do not replace this work with serde-deserialization of a convenient subset.
Unknown extension/plugin content must survive as the contract requires. Validate
duplicate names, unresolved references, graph cycles, malformed numbers, deep or
cyclic includes, and invalid mass/inertia. Keep syntax, validation, frame
resolution, resource resolution, and simulator instantiation separate.

Public output is a validated semantic model and source diagnostics. Provide
dependency-light numeric values and an injected include/resource resolver.
Do not depend on gz-sim-rs/gz-math in order to return a pose; use reusable math
types or plain values with conversion at the simulator boundary. Do not depend
on Fuel or transport merely to resolve a URI.

Deliverables: pinned schema/conversion corpus, parser/DOM/resolver APIs,
round-trip and differential tests, fuzz targets, CLI validation/conversion, and
performance reports for tiny models and large nested worlds. P2 needs validated
world/model/link/collision/inertial/joint/sensor subsets with explicit unsupported
errors; P7 requires every retained baseline semantic feature.

## gz-msgs-rs

Preserve baseline `.proto` packages, field numbers, types, enum values, imports,
descriptor names, defaults/presence behavior, and wire formats. Generate Rust
types from canonical schemas rather than hand-transcribing them. The inspected
main snapshot contains 216 proto3 schemas; retain a schema hash manifest and
decide historical schema support explicitly.

Evaluate prost plus descriptor/reflection support against alternatives using
the actual Gazebo tooling requirements. Prost generates proto2/proto3 types and
does not itself supply runtime reflection. Reflection, dynamic message creation,
JSON/text formatting, unknown fields/enums, Any/type names, and round trips need
separate conformance tests before choosing a runtime.
[Prost documentation](https://docs.rs/prost/latest/prost/)

Test Rust-encoded/C++-decoded and C++-encoded/Rust-decoded messages, including
unknown values, empty and optional fields, large bytes, timestamp ranges, malformed
inputs, descriptors, and conversions to/from domain values. Require semantic
equivalence where protobuf permits differing byte order; demand exact bytes only
where the protocol or fixture makes that meaningful. No arbitrary JSON bridge in
high-rate message paths.

Deliverables: generated types, descriptors, conversion helpers, tooling, pinned
generator/runtime versions, schema diffs, interoperability corpus, and evidence
for unknown-field policy. Message crates do not import the simulator's math crate.

## gz-transport-rs

Preserve the current wire protocol and operational behavior: discovery,
advertisement, pub/sub, services, request/reply correlation, partitions/namespaces,
timeouts, connection lifecycle, throttling/statistics, authentication where
present, logs, and command tooling. Inventory env/config behavior and multicast,
IPv4/IPv6, interface selection, and process shutdown from upstream tests.

Replacing transport with an unrelated DDS, Zenoh, or custom protocol is outside
this plan. Evaluate a maintained ZeroMQ implementation/binding against the actual
protocol, discovery behavior, and features; do not assume “speaks ZeroMQ” means
interoperable. Native libzmq behind Rust is an acceptable transition or dependency
if it is the most reliable way to preserve the required protocol.

Define a bounded, cancellable Rust API. Decide executor ownership explicitly:
the library must not silently install another global runtime. A synchronous
front end may wrap an owned IO worker; async integration should not force the
simulation step to await a socket. Preserve callback lifetimes and cancellation.

Required interop matrix: Rust↔C++ pub/sub, services in both directions, discovery
under delayed startup/reconnect, partitions, malformed data, slow consumers,
concurrent shutdown, and log read/write/replay. Include existing external clients
such as ROS bridges where they exercise baseline wire contracts; rewriting those
external projects is outside scope.

Deliverables: protocol contract, actual mixed-process tests, concurrency/overload
tests, transport CLI executables, log compatibility tests, latency/throughput and
allocation benchmarks. Coordinate schema evolution with gz-msgs-rs.

## gz-fuel-tools-rs

Another agent owns this implementation. Do not reimplement its HTTP, cache,
archive, authentication, or C++ facade work. Its inspected local plan has stronger
C++-API requirements than the simulator needs; preserve that separate commitment.

Agree a small Rust integration contract: resource identity/version, URI handling,
cache lookup, fetch/cancel/progress, offline mode, credentials injection, error
classification, resolved resource path/provenance, and concurrency. Start with
its existing public Rust API; request a focused change from its owner only if a
required contract is absent. Avoid crossing its JSON/C++ facade from Rust if a
direct Rust call exists.

Run simulator integration with deterministic local fixtures and a pre-populated
cache, then a separate authorized live-service smoke test when appropriate. Do
not make live network availability the only way to test world loading. Pin the
Fuel commit/release without assuming the uncommitted active checkout is stable.

The CLI packaging contract must prevent Fuel and gz-sim-rs from both installing
an incompatible `gz` dispatcher. Keep Fuel's standalone command usable.

## Integration order and release set

1. Inventory schemas/protocols and settle package names, value types, errors,
   resource resolver, and engine-independent timestamps.
2. Bring up message generation and minimal SDF semantic support.
3. Implement transport interop and agree Fuel's direct Rust contract in parallel
   with independent owners; this is coordination, not permission to edit their work.
4. Run the P2 world through the actual repo APIs. Use temporary adapters only with
   recorded owners/removal gates; never claim them as completed Rust rewrites.
5. Expand all ledgers to baseline parity and qualify a pinned multi-repo release.

A cross-repo change includes producer/consumer contract tests, a compatibility
manifest update, and a mixed-version policy. Never merge a broken producer and
hope all downstream repositories update simultaneously.
