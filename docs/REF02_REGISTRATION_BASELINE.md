<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Audited non-gtest reference registrations

This REF-02 slice recognizes ten previously unclassified CTest registrations in
the selected headless SDK and separately executes their unchanged commands. One
additional registration is explicitly blocked by that SDK's Zenoh configuration.
It does not complete REF-02, qualify other profiles, retire behavior or add Rust
test credit. Independent source review, 112 tooling checks and a second execution in the
exact immutable image passed. Normal staged and exact hosted checks remain
pending for this candidate.

The immutable image is
`sha256:11b7378c7cca2844a4b7b2c9f52525e983b19d6651d33c83375d220672462d1d`;
its `/reference/environment.json` SHA-256 is
`0775304639aa56e106de8f49802246bb54a16816185cafdc433b0006ddd1f59f`.
The image's embedded scripts and source checkouts remain unchanged. An external
copy of the collector and runtime runner supplies the new observations.

| Registrations | Native assertion and observed result |
| --- | --- |
| Seven `INTEGRATION_*versioned_symbols` registrations in math, rendering, sensors, transport (two), utils and sdformat | Original shell guards inspect the original library symbol tables. All exit zero, report a positive versioned-symbol count and no unversioned symbols. These are ABI assertions, not gtest functional cases. |
| `gz-cmake/cmake_minimum_required_match` | Original Bash command checks CMake version policy; fresh XML contains its single passing `cmake_minimum_required.make` case. |
| `gz-msgs/basic_TEST.py` | Original pytest command and CTest environment modifications produce one passing `BasicTest.test_serialization` case. This is already the Messages ledger's 83rd original mapping, not an additional port. |
| `gz-tools/UNIT_gz_TEST` | Original CMake wrapper runs the configured Ruby dispatcher and emits its single passing `UNIT_gz_TEST.GZ_SCRIPT` case. CLI behavior is retained; there is no retirement decision here. |
| `gz-transport/UNIT_NodeShared_TEST` | The exact source guards all thirteen tests with `HAVE_ZENOH`; the exact target flags omit it, the generated header undefines it, and the cache says `GZ_TRANSPORT_ENABLE_ZENOH:BOOL=OFF`. Its exact binary/list output is profile-blocked, never passed. A Zenoh-enabled profile remains open. |

## Recognition and execution boundaries

[The contracts](../planning/reference/non-gtest-registration-contracts.json)
bind unique repository/CTest identities, exact commands and ordered properties,
interpreter hashes, selected source/helper/library bytes and generated files.
The collector also checks each checkout's pinned revision and tracked cleanliness.
Matching a contract produces `registered_native_wrapper` with
`registration_only_not_executed`; it adds no gtest cases. Missing/changed inputs,
altered arguments or properties and duplicate identities do not match.

Only the exact NodeShared source/flags/header/cache/binary combination permits
the expected empty stdout to become `profile_blocked`. A generic empty list,
changed output, nonzero exit or unexpected stderr remains an enumeration failure.
The collector still exits 1 for this blocked profile. There are zero unqualified
registrations in this particular observation, not a successful complete inventory.

`scripts/run-native-registration-baseline.py` requires the qualified environment
manifest, image identity and contract-bound inventory. It independently collects
the registrations, rechecks the profile gate, and runs exactly the ten original
commands with their original working directories and CTest environment semantics.
It checks selected inputs again after each command. It refuses pre-existing XML
endpoints without deleting them, requires fresh regular result files, checks exact
case identity and totals, and rejects errors, failures, skips and noncompletion.
The shell guards additionally require their exact success message with a positive
symbol count and empty stderr. Runtime results are separate from listing states.

The runner's `--image-id` is a caller-supplied identity, not cryptographic proof
that an arbitrary process runs in that image. Qualification therefore also uses
`docker image inspect` and execution by that exact image ID. An environment file
alone does not qualify another filesystem or dependency profile.

## Evidence and reproduction

The new [listing](../planning/reference/headless-registration-inventory.json)
preserves all 1,175 registration identities, 578 configuration/case payloads,
4,589 case expansions, 579 result guards and seven compiler-diagnostic
registrations. [Preservation evidence](../planning/reference/non-gtest-preservation.json)
identifies the eleven state refinements. The earlier
`headless-test-inventory.json` and its receipt remain byte-for-byte unchanged.
Some gtest GetParam comments contain addresses, so raw stdout equality is not
claimed; parsed native identities and disabled flags are preserved.

[The runtime report](../planning/reference/non-gtest-runtime-baseline.json)
records ten actual passes, original exit codes, selected input and output hashes,
the three fresh XML summaries, the blocked profile, image/environment identity,
and runner/collector/contract digests. Raw command/output/XML evidence remains in
private artifact directories. No listing is converted into a pass.

Run the external collector inside the qualified SDK with each of its fourteen
builds supplied as `--build REPOSITORY=/reference/builds/REPOSITORY`,
`--library-path /reference/install/lib`, a new `--raw-dir` and `--output`.
Then pass those same fourteen build arguments to the runtime runner together
with `--inventory`, `--environment /reference/environment.json`, the exact
`--image-id`, a new `--raw-dir` and `--output`. The runtime process uses
`LD_LIBRARY_PATH=/reference/install/lib`. The repositories are cmake, utils,
math, tools, plugin, common, msgs, transport, fuel-tools, rendering, sensors,
physics and sim with the `gz-` prefix, plus `sdformat`.

The actual replay uses no network and a read-only image root. Private executable
`/tmp`, `/root/.gz`, each build's `Testing` scratch directory, and the three
original CMake/Msgs/Tools `test_results` directories are writable tmpfs mounts.
Only the external script/data bundle is mounted read-only and private output is
mounted writable. The first attempt omitted writable CTest scratch directories
and failed closed; its results are retained and excluded from qualification.

Eight new boundary controls plus nineteen retained inventory controls pass.
They exercise actual synthetic shell execution, changed wrappers/properties/
interpreters/source/flags/header/cache/binary, generic empty lists, false success
output, wrong/missing/skipped/failed XML, stale XML without execution, and inputs
changed during execution. All 112 Python tooling checks pass. Independent review repeated all 112 tooling checks, verified unchanged parsed
case payloads and replayed the exact scripts and contracts in the immutable
image. The ten original runtime semantic results matched. Native runtime
passes do not complete broader REF-02 acceptance.

## Embedded collector packaging

The source recipe copies the required registration contracts into the image,
allowlists that input in its Docker context, and hashes it before and after the
native build. Missing or changed contracts fail the build provenance checks.
Independent tooling and minimal offline Docker controls passed: the actual COPY
lines and context rules include the original contract, while a missing COPY,
explicit exclusion or mutated contract is rejected. This is packaging evidence;
the complete SDK was not rebuilt and the historical image identity is unchanged.

The server test fixture also retains reusable discovery-port reservations for the
calling test thread. Exclusive ephemeral sockets previously closed before Node
startup, allowing another parallel test to take the selected port. A real-socket
control checks that exclusive binds remain denied before, during and after Node
startup. Existing simulation assertions and deadlines are unchanged.

## Remaining REF-02 work

The 579 result guards and seven compiler-diagnostic registrations are preserved
as registrations and were not executed by this slice. Full gtest runtime results,
complete source-ledger reconciliation, other backend/language/GPU/GUI profiles,
the Zenoh-enabled cases, and reviewed Rust/retirement dispositions remain open.
Native shell ABI passes do not require a replacement C++ ABI or system plugin
host. Conversely, that architecture decision does not discard useful Python,
CLI or simulator behavior. No completion percentage is inferred from these counts.
