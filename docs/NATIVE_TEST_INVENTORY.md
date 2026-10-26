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

The selected five existing builds currently list 237 configurations and 2388
case expansions, while 724 registered entries remain unqualified. No cases were
executed by this tool. Additional builds/profiles, wrapper expansion, runtime
results, complete source-ledger reconciliation and the immutable reference image
remain REF02/P0 work. The new Rust application retains its own Cargo checks and
strict coverage requirements.

Independent source/evidence review approved the corrected collector after
reproducing an inherited-Git-context bug in the first draft. The new two-checkout
control rejects a decoy baseline before CTest runs, accepts the correct checkout
and detects its dirty source under the same hostile parent environment. All
original tooling assertions remain. Exact staged checks pass 239 Rust tests,
35 tooling controls and the unchanged fifteen C++ scenarios, with every production
crate above 95% lines and regions. See planning/reviews/native-test-inventory.json.
Normal protected commit and exact hosted qualification remain separate gates.
