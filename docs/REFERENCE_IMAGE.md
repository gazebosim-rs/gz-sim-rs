# Native headless reference SDK

The headless TPE recipe builds the fourteen pinned C++ repositories from public
sources in an Ubuntu 24.04 amd64 image. It provides a clean reference for the Rust
rewrite; it does not complete REF-01, REF-02, P0 or P2. See
[the build instructions](../reference/README.md),
[the environment manifest](../planning/reference/headless-sdk-environment.json)
and [the selected evidence](../planning/reference/headless-sdk-evidence.json).

The qualified local image ID is
`sha256:11b7378c7cca2844a4b7b2c9f52525e983b19d6651d33c83375d220672462d1d`.
It is not a published registry image. A rebuild may have a different image ID;
compare its recorded inputs, source SHAs, effective flags and installed files.
The original lock's draft annotation is retained because its bytes are part of
the built image's provenance. This report qualifies only the results below.

## Build and selected execution

All fourteen Release builds and installs completed with BUILD_TESTING enabled,
two CMake jobs, GCC 13 and the fixed Ubuntu snapshot. The environment records
690 installed packages and 4,743 installed paths, including symlink targets and
file hashes. Compiler, source, recipe, license and configuration identities are
recorded before source commands execute and checked again before finalization.

Ten unchanged native originals passed: Model.BasicAPI, Link, BoundingBox,
CollideBitmask and NestedModel; World.BasicAPI and Model; and the IMU
CreateImuSensor, Orientation and OrientationReference cases. XML identity,
completion and zero failures/errors/skips were checked. The unchanged native
headless fixture also produced exactly 100 observations at 1 ms intervals through
actual Server, Physics, TPE and IMU libraries. Its source, SDF fixture, compiled
binary and loaded library hashes are retained. These are native reference runs,
not additional Rust test ports or measurements of simulator speed.

Runtime checks use the exact image ID with no pulls, no external network, a
read-only root filesystem and private temporary files. A separate writable
`/root/.gz` tmpfs supplies Gazebo's default log destination. Earlier numeric
passes exposed its missing writable path; those warnings remain in the evidence
history. The correction changes neither image bytes nor test assertions.

## Inventory and remaining scope

CTest/gtest expansion found 578 configurations and 4,589 case expansions across
all fourteen configured repositories. The collector correctly exits unsuccessfully:
597 entries remain unqualified. Of these, 596 need wrapper or binary handling;
`gz-transport/UNIT_NodeShared_TEST` lists no cases because this profile disables
Zenoh and its cases are guarded by HAVE_ZENOH. None is counted as a passing test.
Counts are listings, not evidence that the full suites ran.

GUI, Ogre, DART, MuJoCo, Zenoh, Python/SWIG, GPU and other backend/language profiles
remain unqualified. The reference includes legacy C++ dependencies, including
Bullet headers required by unchanged common-test CMake; both Bullet engines are
skipped. The Rust architecture still uses Bevy and has no Bullet backend.

Earlier missing-package SDK builds and the initial read-only CTest-log and
noexec-tmpfs probe failures are retained. No original test or tolerance was
changed. Image distribution/license review and registry publication remain open.
The SDK manifest is a different contract from the existing six-library native
client reference; adopting it into CI requires a separate integration change.
Equal-work results in HEADLESS_MEASUREMENT.md belong to their recorded revisions
and environment, not to this new SDK or its compiler.
