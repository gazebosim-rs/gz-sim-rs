# Configured native test inventory

`python3 scripts/inventory-native-tests.py --help` collects CTest JSON from
configured native builds and lists recognized gtest executables. Each source
checkout must match its baseline commit without tracked changes. Repository-local
Git variables are cleared before both source probes, so a caller cannot redirect
those checks to another checkout. Cache, binary,
collector and raw-output hashes bind the observation. These hashes identify the
observed inputs; they do not attest a clean immutable build or its dependencies.

Pass each build as `--build REPOSITORY=PATH`, a private evidence directory with
`--raw-dir`, and a public JSON path with `--output`. Supply required native SDK
search directories using repeated `--library-path` arguments. Repository names
come from planning/baseline.json. The collector uses CTest working directories
and ordered ENVIRONMENT properties; CMake applies ENVIRONMENT_MODIFICATION and
reset semantics. The full configuration participates in duplicate identity.

Only listing flags are added and gtest XML-output arguments removed. Executable
bytes are checked again after listing, so a concurrent rebuild cannot silently
bind output to an earlier binary hash. Raw stdout/stderr and the report remain
in a unique evidence directory, including nonzero exits and partial timeout
output. Malformed/empty/duplicate lists and invalid source builds fail closed.

The public report contains hashes, case identities, disabled flags and explicit
missing-command, unrecognized-wrapper/binary, duplicate and failure states. Raw
commands, build locations and environment values stay in local evidence. A
recognized ELF must contain the gtest listing marker; shared-gtest binaries
without it remain unqualified. Wrappers and other test frameworks need separate
expansion. Runtime skips and test success cannot be inferred from a listing.

The earlier five-build observation lists 237 configurations and 2388
case expansions, while 724 registered entries remain unqualified. No cases were
executed by this tool. Additional builds/profiles, wrapper expansion, runtime
results and complete source-ledger reconciliation remain REF02/P0 work. The
selected clean headless SDK is qualified separately in REFERENCE_IMAGE.md; other
reference profiles remain open. The new Rust application retains its own Cargo checks and
strict coverage requirements.

Independent source/evidence review approved the corrected collector after
reproducing an inherited-Git-context bug in the first draft. The new two-checkout
control rejects a decoy baseline before CTest runs, accepts the correct checkout
and detects its dirty source under the same hostile parent environment. All
original tooling assertions remain. Exact staged checks pass 239 Rust tests,
35 tooling controls and the unchanged fifteen C++ scenarios, with every production
crate above 95% lines and regions. See planning/reviews/native-test-inventory.json.
Normal protected commit and exact hosted qualification remain separate gates.

## Headless result-file guards

The external collector now recognizes only the pinned gz-cmake result-file
helper, invoked by exact `/usr/bin/python3`, with a unique recognized gtest
parent and matching working directory and result XML endpoint. Changed helper
bytes, decoy interpreters, duplicate registrations and altered arguments or
environment remain gaps. Recognition lists a registration; it executes neither
the helper nor the guarded test and contributes no gtest cases or Rust ports.
The BSD helper remains upstream; its source is not copied or relicensed.

The immutable SDK from REFERENCE_IMAGE.md retains its original embedded
collector. A separately hashed external collector lists 579 result guards and
preserves all 578 native configurations and 4,589 case expansions. That observation
left eighteen entries unqualified: seventeen other wrappers/executables and the empty
Zenoh-gated NodeShared binary. The collector still exits1 for that empty list;
this is retained evidence, not a successful complete inventory.

See [the complete listing](../planning/reference/headless-test-inventory.json) and
[its evidence](../planning/reference/headless-test-inventory-evidence.json) for
image, external collector, checker, archive and preservation identities. The earlier 597-gap
report remains unchanged. Independent replay preserves case/configuration
payloads; three parameterized binaries have different raw GetParam address/object
comments. No raw-output byte or parameter-semantic equality, native functional pass, performance
improvement, full REF02 or P0 completion is claimed. Other runtime profiles and
full source-ledger reconciliation remain open.

## Compiler-diagnostic registrations

The collector separately recognizes seven pinned CTest registrations that require
specific compiler diagnostics: three physics FeatureList cases and four plugin
const/base-class cases. Recognition checks the exact helper and CMake source
hashes, target, diagnostic predicate, Release command and working directory,
and records each C++ source hash. Changed commands, helpers, properties, missing
or escaping sources and duplicate names remain gaps. Recognition executes neither
the wrapper nor its build target and adds no gtest case or functional pass.

Author and independent immutable-image listings preserve all 1,175 registered
identities, 579 result guards, 578 gtest configurations and 4,589 case expansions.
Exactly seven old wrapper rows become compiler-diagnostic registrations. Eleven
gaps remain: seven symbol-version guards, the CMake policy test, the Python
protobuf test, the CLI dispatcher test and the empty Zenoh-gated binary. Its
listing failure still produces exit1. No test has been retired or ported by this
classification; retained plugin, Python and CLI behavior remains in scope.

The public listing and evidence above now record this incremental observation.
The previous report and its hashes remain identified at their published commit;
the SDK image and original embedded collector are unchanged. Three raw stdout
hashes differ only within GetParam address/object comments. Parsed case payloads
are preserved; raw-byte and parameter-semantic equality are not claimed.
Independent source/native/projection review, focused19 checks and 22 synthetic
receipt rejections pass. Exact staged checks passed 246 Rust tests, 68 tooling
tests and fifteen unchanged legacy-reference C++ scenarios, with every production
crate above 95% line and region coverage. Final binding, normal protected commit
and exact hosted qualification remain separate gates. Full REF02/P0/P2
acceptance remains open.
