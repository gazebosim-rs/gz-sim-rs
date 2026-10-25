# Planning manifests

The original inspection manifests below record source inspection on 2026-10-24.
Those manifests contain no passing-test or implementation-completion claims.

- `baseline.json`: repository, selected branch, inspected commit, and source
  cleanliness. The owner selected main. Verify any newer upstream commits before
  updating this lock, and rebuild the reference environment for a changed lock.
- `source-inventory.json`: public header candidates, test-source candidates,
  unexpanded C++ test macro counts, and simulator system directories.
- `system-migration.json`: all 78 simulator system directories assigned to work
  families in the task plan. All are initially not migrated.

The inventory used tracked files from `git ls-files`, excluding vendored gtest.
Header candidates match public include/gz paths (and include/sdf for sdformat),
excluding detail/testing paths where appropriate. Test candidates use TEST/test
filename patterns and integration/regression/performance/benchmark directories.
The C++ count recognizes TEST, TEST_F, TEST_P, and typed variants. It does not
expand parameters or prove registration, compilation, or execution. Script,
generated, conditionally compiled, and installed tests need P0 reconciliation.

The systems register was checked against the actual `gz-sim/src/systems` directory
set, with each directory assigned once. Tests for non-system server/library
features are tracked separately in the implementation task plan.

P0 must generate the executable case-level ledger described in TESTING.md. Do
not use this source inventory as that ledger, a coverage denominator, or a count
of tests that have passed.

Reference-results and reference/ record scoped actual native runs, with their
source/build/provenance limits. Benchmarks/ retain raw scoped measurements and
rejected-comparison qualifications; reviews/ bind independently checked source.
Test-ledger.json distinguishes original ports/partial cases from supplementary
controls. Acceptance/ records selected integration matrices and explicit status;
a candidate file does not accept a phase until independent exit review and
required exact-tree checks complete.
