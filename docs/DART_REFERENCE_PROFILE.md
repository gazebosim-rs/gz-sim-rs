<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Proposed clean native DART reference layer

This REF-01 recipe builds pinned `gz-physics` and `gz-sim` afresh with system
OSRF DART 6.13.2, over the already qualified headless TPE SDK dependency/source
base. It does not replace the historical SDK, change Rust engine pins or qualify
a DART image merely because the recipe or its controls pass. The private layer
build now succeeds; its bounded functional smoke also passes as recorded below.
Independent image/runtime review, expanded inventory and broader native execution
remain required; P0 and other profiles remain open.

## Why a separate profile

The installed host reference has useful native trajectories, but its existing
physics cache is `RelWithDebInfo` and selects ROS-vendored DART headers through
`/opt/ros/jazzy/opt/gz_dartsim_vendor`. The native articulation capture deliberately
loads the system OSRF libraries instead. Recording those exact local binaries
does not attest that they were built from those headers or the checked source.
The clean TPE SDK is a different profile and explicitly disables dartsim.

The new layer preserves the qualified TPE base and all fourteen source pins.
It adds only the ten selected DART runtime/development packages, then uses fresh
`/reference/dart-builds/{gz-physics,gz-sim}` and `/reference/dart-install` paths.
It keeps Release, tests enabled, native Bullet/Bullet Featherstone/MuJoCo engines
disabled, GUI disabled and Python bindings disabled. DART's Bullet collision
component is an upstream build dependency, not an additional Rust engine.

## Dependency provenance

[The lock](../reference/headless-dart.lock.json) binds the qualified parent image
ID and environment manifest, baseline, exact OSRF package names/version/arch,
payload sizes/hashes, and signed binary/source index bytes. The sole OSRF archive
key is retained explicitly; this does not copy a developer trust store.
`gpgv` verifies fingerprint `D2486D2DD83DB69272AFE98867170598AF249743` against the
retained signed InRelease. Its SHA-256 entries bind the decompressed Packages
and Sources indexes. Selected binary fields and DART's source-package identity
and upstream/Debian tarball checksums must match those signed indexes.

All ten actual `.deb` payloads have been downloaded from the official OSRF
repository and checked against the selected index digests and
`dpkg-deb` identity fields. They are private regeneration artifacts, not checked-in
packages. Six selected installed config/header/library files also match bytes
extracted from those exact payloads. This does not mean DART was rebuilt from its
source package; OSRF DART remains an approved binary dependency.

The recipe uses checksum-bound package URLs rather than a floating OSRF APT
source. Missing historical URLs fail rather than selecting newer packages.
Remaining Ubuntu dependencies resolve only through the already fixed Ubuntu
snapshot and are recorded in the resulting full package manifest. No host ROS
libraries, package registries, compiler flags, Python paths or loader hooks are
used. DART's CMake directory is explicitly `/usr/share/dart/cmake`; configured
include paths and the complete cache are checked for ROS contamination.

## Proposed build

First independently review this recipe and its dependency closure. The local base
tag must resolve to the lock's exact image ID before and after the build:

```sh
docker image inspect --format '{{.Id}}' gazebosim-rs-reference:headless-tpe-v6
docker build --pull=false --build-arg BUILD_JOBS=2 -f reference/Dockerfile.dart -t gazebosim-rs-reference:headless-dart-proposed .
docker image inspect --format '{{.Id}}' gazebosim-rs-reference:headless-tpe-v6
docker image inspect --format '{{.Id}}' gazebosim-rs-reference:headless-dart-proposed
```

The recipe's local parent tag is not independently immutable: these external
checks and the exact parent environment/installed-file validation are required.
The builder cannot place its own resulting image digest in an input manifest.
It emits `dart-environment.json` with input/package/compiler/source/cache/log and
installed-file identities, and listing counts with no execution-pass credit.
The final image ID and subsequent per-image observations are bound externally.
Historical non-gtest contracts and runtime reports belong only to their old image;
they must not qualify this new profile by reuse of that historical identity.

The builder verifies inputs before and after the build, refuses existing output
directories, checks both source trees, validates Release/test/engine/DART settings,
and requires the new physics core, DART engine, simulator and physics-system
modules. Future qualification must check actual mapped new-layer/base/system
library identities, not only basename or pkg-config version.

## Controls and next qualification

Eight focused controls pass: actual signed binary/source index binding, changed
metadata/signature rejection, wrong package version/source/hash/selection, recipe
base/checksum changes, missing/wrong/linked payloads, hostile ROS/compiler/search
environment, altered CMake settings and duplicate raw cache-key rejection.
Direct validation of all ten actual payloads also passes. All 111 Python tooling
checks pass on the isolated proposal parent: eight new focused controls and all
103 retained controls. Normal staged and exact hosted qualification remain
required before adoption; build and runtime evidence are separate from these controls.

## Actual private build and bounded smoke

The independently reviewed recipe built successfully with two jobs in 1,373.4
seconds. The resulting image is
`sha256:4ac74e616369164eaba07149e9b98f7594a33cb3ddbbd6a45e0dd43cb688b1d5`;
the base tag matched its locked identity before and after. Read-only post-build
inspection rechecks protected input, signed metadata, payload, header/library,
source/cache/log and installed-file hashes. The new physics and simulator builds
register 155 and 276 CTest entries respectively; those counts are not passes.

The first smoke failed closed because native system lookup chose the new installed
Physics system while the runner declared its build-tree copy. That failed output,
maps and original source bundle remain retained. The reviewed runner now permits
an explicit single system artifact, validates its role and distinct canonical
identity, and requires that exact mapped path/hash without fallback.

Using the explicit new installed Physics system, a preflight and two fresh sample
processes each complete 100 warmup and 100 measured steps with the unchanged native
double-pendulum fixture. All clock/count/finite/motion/semantic-name checks pass,
stderr is empty, and the four declared modules and full library set stay stable.
The run uses the exact new image, no network, read-only sources/root and private
scratch/output. Timing fields are diagnostic while other compilation is active;
this smoke is not a benchmark, full-profile or P0 acceptance. Independent review
of this new runtime evidence remains pending.

All 155 configured native physics CTest registrations have also executed. The
first run retained three compiler-guard failures because nested make could not
write a generated dependency cache on the read-only image. After adding an exact
private copy of that existing generated cache, the unchanged suite exits zero.
The source, CMakeCache, native libraries and test executables remain read-only;
all 17 existing object inputs in the private cache retain their original hashes.
Both full runs and their raw aggregate/per-test XML are preserved.

The 76 gtest XMLs contain 489 entries: 318 completed cases and 171 skips, with
no case failures or errors. These are not 489 passes. Exact pinned source sites
classify 160 skips as unavailable requested feature lists and eleven as explicit
upstream backend exclusions. A native-loader metadata diagnostic successfully
loads both exact common-test plugins with empty stderr and exposes their expected
names and 102/48 interfaces; this is not a wholesale plugin-load failure.
Skips remain skipped, including the original known-bug exclusions. Independent
review and remaining profile qualification are still pending.

Next collect the complete new-image case inventory and remaining simulator
runtime baseline, then a balanced capture when resources are quiet. Run against the new image with private
scratch/log directories and record actual loaded module maps. Preserve and compare
the existing source/case identities; failed or disabled cases stay explicit.
Full gtest/profile execution, source-ledger reconciliation, immutable DART runtime
qualification, image distribution review and balanced REF-03 performance remain
separate work.
