<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Consumer-owned proto build and coverage

`gz-sim-proto` generates the consumer's single standard binding identity from
216 immutable canonical proto3 files. It is not the former Messages SDK, a
transport facade or a reflection/conversion/forwarding runtime. The schema-only
producer transition stays private until actual consumers qualify. D28/D29 and
[the migration contract](ZENOH_SCHEMA_MIGRATION.md) retain their release gates.

The checked-in source policy pins the source inventory hash, upstream identity,
compiler version, external standard Any source, generator and full descriptor
hash. Build.rs checks all schema bytes, sorted unique inventory, extra/missing
files and symlinks. It resolves the actual compiler, checks its output/version,
then uses maintained prost-build 0.14.4 load_fds/compile_fds with standard protoc
3.21.12. Complete compiled import names and descriptor bytes must match. Canonical
presence/tags/options/syntax are thereby validated by the standard compiler and
exact descriptor, without a custom proto parser or loader. Generated comments
and standard Name implementations remain present.

The build records the actual compiler and standard Any source hashes, descriptor,
policy/manifest hashes and generated gz.msgs.rs hash. Cargo rerun directives cover
the schema directory, policy, compiler and external Any file and the three
compiler/include/path environment inputs. The profile also binds the Cargo.lock,
crate manifests and complete schema inventory; no build fetches a moving schema.
Native +/-0, encoded length/exact slice capacity and complete Clock/IMU/CameraInfo
wire assertions remain in tests/native_wire.rs, including existing NaN behavior.

Production inventory retains BOTH build.rs and src/lib.rs. The former has real
executable ownership; each mandatory headless and renderer profile includes the
proto crate and must exceed 95% line AND region coverage. Empty crate or build
instrumentation is a failure. Tests include the actual build script and run its
entry point in child processes, as existing Cargo build-script controls do.
Missing/malformed inputs, wrong hashes/counts/paths, imports, compiler failures,
standard Any drift and generation/output failures use actual maintained tools.

The facade is exactly pub mod msgs with the builtin
include!(concat!(env!("OUT_DIR"), "/gz.msgs.rs")) plus pub use prost. A dev-only
Syn2 test target checks its complete AST, literal arguments and closed item set.
It refuses custom/shadow macros, extra runtime items, attributes, arbitrary paths
and altered reexports. The coverage verifier binds this auditor's source and
actual test executable, runs it before merging profiles and requires its exact
source-bound result. No source-hash admission list or added executable filler is
used. Renderer-specific source audit and GPU instrumentation guards are unchanged.

Only an ABSENT facade may use this classification. Actual LLVM/LCOV instrumentation
always wins: DA0 and executable regions stay measured, missing build/region counts
fail and changed-line thresholds remain unchanged. A real instrumented negative
program proves an uncovered runtime body cannot be converted into a facade proof.
The generated dependency is separately recognized only as the exact gz.msgs.rs
file produced by the source-verified Cargo build; its presence/hash are checked
and its bytes/descriptor retained in normal profile evidence. A second generated
output or missing generated instrumentation is rejected. This exclusion does not
apply to owned build code or justify an empty coverage denominator.

Independent fresh-source review and replay pass 14 regular Rust tests (plus two
delegated child entry points explicitly executed by parents), 24 retained profile
tests and eight proto policy controls including real LLVM. All-target leaf Clippy
passes. The reviewer reproduced an external symlinked schema-root acceptance;
the narrow real-directory guard and regression now reject that input before
generation or evidence, and the original reproducer fails as required.

The first real LLVM diagnostic failed at 315/333 regions. Meaningful input/error
controls initially increased it to 323/333, with 185/186 lines. After the reviewed
schema-root guard, final source-bound coverage is 189/190 lines and 329/340 regions;
the added guard remains measured. No thresholds, assertions or source exclusions
were weakened. Independent replay also confirms the genuine uncovered owned-build
line stays counted and fails the changed-line gate when marked changed. The integrated profile
helper was exercised on the leaf alone. This does not qualify the full workspace,
renderer runtime, hosted workflow or production migration. Those ordinary gates
remain required before acceptance.
