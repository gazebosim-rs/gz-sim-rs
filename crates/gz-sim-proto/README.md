<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Simulator-owned generated messages (private migration candidate)

This package generates one shared binding set from the pinned canonical schema
snapshot. It contains no application runtime, conversion framework, reflection,
CLI, forwarding envelope or custom message traits. `prost` owns serialization;
its standard `Name` implementation provides native Zenoh type attachments.
Domain helpers remain in simulator owners.

The schema snapshot is byte-identical to the public revision in schema-source.json
and to the private schema-only candidate. Standard protoc 3.21.12 and prost-build
0.14.4 generate the types during this consumer's build. No build downloads code.
Original schema notices remain unchanged. The only third-party repair is the
explicit two-predicate signed-zero fix in third_party/prost-derive-0.14.4.

The native fixtures come from the separately qualified standard-protoc producer
in the Messages consumer feasibility experiment. Fixture hashes are recorded in
tests/fixtures/provenance.json. Native signed zero and complete Clock/IMU/325-byte
CameraInfo wires match. An independent stock-derive counter run fails both wire
tests on dropped negative zero; its output is retained separately.

The build validates every canonical source hash, complete inventory, standard Any
source and actual compiler version. Standard prost-build loads the compiler's
full descriptor set; its import names and pinned descriptor bytes must agree
before generation. Cargo watches schemas, source policy, the actual compiler and
standard import, including PROTOC/PATH/PROTOC_INCLUDE changes. Each successful
build retains source/compiler/descriptor/generated binding hashes in OUT_DIR.

The owned build script remains in the production line and region denominator.
A dev-only Syn2 auditor recognizes only this exact builtin generated include and
prost reexport; no runtime function, custom/shadow macro or arbitrary include can
receive its absent-source classification. Generated output accounting binds the
actual Cargo output to pinned inputs and its exact source hash. See
[the build and coverage contract](../../docs/PROTO_GENERATION.md).

This is not production adoption. Independent fresh-source review and replay pass
14 regular Rust tests, 24 retained profile controls, eight proto policy controls
and Clippy. Two delegated child entry points are explicitly executed by their
parent tests and the verifier. Isolated real LLVM reports owned build.rs coverage
of 189/190 lines and 329/340 regions. The reviewed schema-root guard also rejects
an external symlinked schema directory before generation or evidence is produced.
These are bounded leaf results, not both mandatory workspace profiles. The
renderer proof is unchanged. Simulator integration, full staged/renderer/hosted
checks and publication remain pending. Historical consumer dependencies remain
until actual server/camera/CLI replacement qualifies.
