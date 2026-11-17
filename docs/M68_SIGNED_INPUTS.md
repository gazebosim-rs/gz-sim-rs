<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M68: preserve exact signed metadata as a CI input

The original M68 runtime at `103fd927` passed the full local normal hook, strict
coverage and independent retained DATA. Both hosted attempts of run
[44049236320](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/44049236320)
failed before product checks. Signed locked-package APT installation succeeded;
the following `--verify-packages-only` command received HTTP 503 while retrieving
a `Packages.xz` index. The exact failed suite/component and provider cause are
unrecorded. Vendor retrieval, Zenoh build and product checks never ran. Both
original jobs ZIPs and independent failure DATA remain retained.

One separately reviewed fresh package-only diagnostic then verified all three
signed releases, six exact indexes, sixteen direct packages and twenty-two
matching package records. Its seventeen original outputs survive in RAM and
byte-checked durable SSD copies. Independent retained DATA checked their source
pins, stable bytes, original signature outputs, summary and actual process exits.
That diagnostic did not qualify hosted CI. One reviewed unchanged CI retry also
failed; no further unchanged retry or snapshot probe is released.

## Input delivery and unchanged trust

Initial adoption copied all 1,852 original source blobs and executable bits.
The final repair changes eight existing workflow, guidance and plan blobs; the
remaining 1,844 original blobs, including all Rust, Cargo, fixtures, protocols
and the eight native SDK input files, remain byte-exact. `planning/zenoh-native-packages.json` still identifies
the original Ubuntu snapshot date, canonical URLs, signing fingerprint, index
hashes/sizes and complete package records. The bootstrap implementation is exact.

A reproducible raw USTAR archive contains only the nine original signed release
and package-index bodies. Its proposed name is
`ubuntu-20261024T000000Z-zenoh-package-metadata.tar`, on the existing public
`native-build-inputs-v1` build-input release. Original signed bytes are unchanged;
only their delivery is archived. One reviewed assembly produced 22,428,672 bytes with SHA256
`2eed96d6cb8c6b4d6cc5f61e35c81713254fd1906182354d0123d79ba17046e6`.
Independent archive DATA checked all nine bodies and deterministic headers against
the retained originals and canonical pins. The exact identity is bound in
`planning/zenoh-native-package-archive.json`. No upload has happened.

The narrow staging helper checks the fixed archive identity and bounded stream,
then admits exactly nine regular flat members with the canonical body hashes
and index sizes. It rejects links, extensions, duplicate/extra/missing members,
path traversal, malformed headers, invalid padding, oversized data and existing
output. Files are published atomically to a fresh metadata directory. Staging
means bytes are available; it does not authenticate package signatures or grant
qualification.

The next workflow command remains the unmodified bootstrap verifier. It checks
all body hashes, runs real `gpgv`, checks the exact signer and signed index
identity/size, and verifies the complete package-record closure. Original locked
APT installation, vendor SHA validation, reference build/validation, full
product verification, strict coverage and required artifact upload remain in
the same workflow. No unchecked fallback or alternate package trust is added.

## Review, publication and acceptance

Finish implementation, meaningful regression Source, guides and independent
whole Source review; batch findings before verification. The data producer is
separately reviewed before its one operation. Independent archive DATA and exact
manifest bindings precede the full normal and strict actual-parent gate over
accepted M67. Reuse unchanged Rust formatting evidence; the full normal still
checks formatting. Run no per-edit tests or lint.

Public input publication requires the exact completed archive, independent DATA,
reviewed identity and normal approval. Upload only one new asset with no clobber
to the existing non-Latest build-input release. Preserve every existing asset,
release setting and the public `m65-preview` Latest release.

The repaired source still needs full normal/strict coverage, independent local
DATA, ordinary review publication, exact hosted success, durable originals,
independent official DATA and qualified Main promotion before M68 acceptance.
M69's reviewed source remains frozen; adopt and review the finally accepted M68
repair parent before any M69 formatting or checks. M70 is an unselected proposal.
