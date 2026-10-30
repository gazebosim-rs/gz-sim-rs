<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Native math runtime baseline candidate

The selected immutable headless SDK executed all 135 configured `gz-math`
CTest registrations successfully: 67 original gtest executables, their 67 result
guards and the original ABI guard. The 67 fresh gtest XML files contain exactly
658 completed cases, with no failures, errors, disabled cases or skips.

These are C++ reference results for this configured profile. They add no Rust
test credit, cover no omitted language/backend profiles and do not complete
REF-02 or the math reuse audit. Existing inventory and prior evidence remain
unchanged. Independent source/data review and a second execution in the exact image passed.
The ordinary staged and exact hosted gates remain required before this candidate
is accepted. The math ABI guard also appears in the separate ten-registration
baseline; it is not a second distinct native test.

## Capture

The image is
`sha256:11b7378c7cca2844a4b7b2c9f52525e983b19d6651d33c83375d220672462d1d`.
Its environment manifest digest is
`0775304639aa56e106de8f49802246bb54a16816185cafdc433b0006ddd1f59f`.
The image contains the existing pinned source builds and unchanged CTest commands.
Execution uses no network and a read-only image root. Only private output,
temporary files, user configuration, CTest scratch and result directories are
writable. Fresh tmpfs result directories prevent old XML from satisfying guards.

From the repository root, use a new output directory:

```sh
mkdir artifacts/native-math-sample
cp reference/capture_math_runtime.py artifacts/native-math-sample/run.py
docker run --rm --network none --read-only \
  --tmpfs /tmp:rw,exec --tmpfs /root/.gz:rw \
  --tmpfs /reference/builds/gz-math/Testing:rw \
  --tmpfs /reference/builds/gz-math/test_results:rw \
  -v "$PWD/artifacts/native-math-sample:/out:rw" \
  sha256:11b7378c7cca2844a4b7b2c9f52525e983b19d6651d33c83375d220672462d1d \
  python3 /out/run.py
python3 scripts/summarize-native-math-runtime.py \
  --inventory planning/reference/headless-test-inventory.json \
  --capture artifacts/native-math-sample \
  --output artifacts/native-math-summary.json
```

The capture retains CTest stdout, stderr, exit status, aggregate JUnit XML and all
individual gtest XMLs. The summarizer matches every registration name and expanded
gtest identity to the pinned inventory, rejects duplicate/missing/extra cases and
files, and rejects failures, skips, noncompletion or a different run identity.
It records case source locations, configuration and binary hashes from the
inventory, and hashes of the raw capture files and generating programs.

The summarizer validates supplied data; it does not itself execute or attest an
image. Qualification also requires executing by the inspected immutable image
digest and independently reviewing the unchanged capture program and results.
Recorded test durations are diagnostic, not a performance comparison.

The checked-in `planning/reference/math-runtime-capture.zip` retains all 72 raw
capture files (including the exact generating program), compressed to about
49 KiB. `math-runtime-baseline.json` records the validated results. A tooling
control extracts these bounded relative files and checks every captured case and
recorded digest against the unchanged inventory. The archive does not include
native binaries or replace the immutable SDK.
