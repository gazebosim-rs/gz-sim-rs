# Headless TPE native reference

This recipe is under validation. A successful dependency bootstrap or SDK build
does not qualify P0, execute original tests, or cover other reference profiles.

Build from the repository root:

```sh
docker build --target reference-deps -f reference/Dockerfile -t gazebosim-rs-reference:headless-deps .
docker build --build-arg BUILD_JOBS=2 -f reference/Dockerfile -t gazebosim-rs-reference:headless-tpe .
```

The Dockerfile-specific context allowlist sends only the recipe, source lock,
builder, baseline, inventory collector and license notices. No developer checkout,
SDK, credential or host trust store is copied. Public sources are fetched at the
baseline SHAs into separate source/build/install directories. Release builds keep
BUILD_TESTING enabled; compilation uses at most four jobs, default two. Coordinate
builds with repository owners' latency-sensitive test windows.

The base image uses an official Ubuntu digest. `ubuntu-bootstrap.json` records the
signed noble InRelease/Packages/package hash chain verified using that base's
archive keyring. A checksum-pinned public CA package supplies initial PEM anchors;
normal snapshot packages then install the full CA/openssl setup. HTTPS and APT
signature/hash checks stay enabled. Only historical metadata expiry is disabled
for the explicit snapshot URI. Any APT index failure fails the build.

The APT setup rejects additional inherited sources before replacing the sole
Ubuntu entry with the fixed snapshot. The builder pins GCC/G++, clears compiler
flag/toolchain/include overrides, and records effective compiler hashes/versions.

The builder rejects changed source pins, dirty sources, submodules without a
source lock, stale build/install directories, disabled tests and non-Release
profiles. It checks effective CMake flags and required selected native libraries.
`/reference/environment.json` records package versions, compiler/tool and installed
file hashes, exact recipe/bootstrap/context/collector/license identities, effective
job count, source/configuration/log hashes and registered CTest counts. Logs and
raw CTest JSON remain under `/reference/logs`. These counts are listings, not
execution passes. Record the resulting image ID externally; it cannot contain its
own image digest.

The native dependency chain includes legacy CMake/tools/rendering core because
the pinned C++ simulator requires them. This does not change the Rust architecture.
Bullet development headers are a reference-only dependency: the pinned physics
common-test CMake inserts their include path even when both Bullet engines are
skipped. Their engines stay disabled; no Rust Bullet backend is introduced.

GUI, Ogre, DART, MuJoCo, Zenoh and Python/SWIG profiles are unqualified here; their
absence is not test retirement. Remaining acceptance requires an independently
reviewed clean image build, expanded native inventory, selected actual original
and headless checks, immutable image identification and provenance review.
