<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Simulator-owned generated messages

This package generates one shared binding set from the pinned canonical schema
snapshot. It contains no application runtime, conversion framework, reflection,
CLI, forwarding envelope or custom message traits. `prost` owns serialization;
its standard `Name` implementation provides native Zenoh type attachments.
Domain helpers remain in simulator owners.

All 216 canonical proto files match both their original public revision in
schema-source.json and the verified schema-only Messages release
`d030d06e7d2b83bdd251a24a6e851605ee35a633`. The minimal consumer inventory retains
its actual `a39a945` provenance; the release adds its own compiler-validation
catalog. Those manifests and their descriptors are distinct, while canonical
sources are unchanged. The source-policy release anchor requires fresh generated
build evidence when updated. Standard protoc 3.21.12 and prost-build 0.14.4
generate the types during this consumer's build. No build downloads code.
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

Consumer adoption is published in simulator commit
[`5870373`](https://github.com/gazebosim-rs/gz-sim-rs/commit/5870373f0d8280601b051f2cbb1bb0aba264f70f).
Server, camera and CLI consumers use these generated bindings. Its ordinary full
staged gate passed 783 headless tests, 435 renderer tests and 217 tooling checks,
including actual native interoperability and software-Vulkan GPU-worker execution.
Both required profiles passed strict owned line and region coverage. Exact hosted
[run 41248418524](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/41248418524)
passed, and its downloaded source, inputs, coverage and native/GPU evidence were
independently audited. These results qualify this consumer integration; they do
not expand the bounded schema or serialization contract.

The earlier independent leaf review passed 14 regular Rust tests, 24 retained
profile controls, eight proto policy controls and Clippy. Two delegated child
entry points were explicitly executed by their parent tests and the verifier.
That isolated real LLVM report measured owned build.rs coverage of 189/190 lines
and 329/340 regions. The schema-root guard rejects an external symlinked schema
directory before generation or evidence is produced. Those initial leaf numbers
remain source-bound evidence, separate from the complete integration profiles.
The renderer proof and generated-output exclusions remain unchanged.
