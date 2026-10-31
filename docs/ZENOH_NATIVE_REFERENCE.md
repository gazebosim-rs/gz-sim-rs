<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Selected C++ Zenoh client reference

This new client-only profile provisions the C++ dependencies used by simulator
interoperability fixtures. It does not change the historical Zenoh-OFF profiles,
qualify an entire C++ SDK test suite, or introduce a Rust ZeroMQ target. The
ordinary functional fixtures, strict coverage, staged checks and hosted checks
remain required. No new full SDK build or hosted acceptance is claimed yet.

`planning/zenoh-native-lock.json` pins the existing six canonical source commits,
Ubuntu 24.04 amd64 direct build package versions and the ROS Jazzy
`zenoh_cpp_vendor` Debian archive. The archive version is
`0.2.11-1noble.20260914.172423`, SHA256
`94325816203f7017b57713975d00ff63a5e5e304d31a5bfb1e992ddb06664c37`.
It supplies Zenoh C/C++ 1.8.0. This is the selected native vendor's existing
kernel, including its unstable-API and shared-memory build features; the
functional probe disables shared memory. Those C++ package features do not
enable unstable features or unsafe application code in Rust.

The script extracts that verified archive privately and uses its relocatable
CMake targets. It does not install packages, modify vendor code, require ROS to
be installed, or use machine-specific expected ELF hashes. It requires the
locked host packages before building. Provision them from the fixed Ubuntu
snapshot rather than a live unpinned package repository.

All 16 direct package versions are present in the signed
`20261024T000000Z` noble, noble-updates and noble-security main/universe indexes.
`planning/zenoh-native-packages.json` records their package archive hashes and
filenames, the signed release hashes and the six compressed index hashes/sizes.
The Ubuntu archive signing key is
`F6ECB3762474EDA9D21B7022871920D1991BC93C`. Repeat the metadata-only check with:

```sh
python3 scripts/bootstrap-zenoh-native.py --verify-packages-only
```

That command verifies HTTPS downloads, `gpgv` signatures, index hashes/sizes and
the exact package records. It downloads only metadata, installs nothing and
does not build the SDK. Raw signed inputs and verification streams remain in
`artifacts/zenoh-native-package-metadata`. Historical release expiry does not
replace signature or content-hash verification.

After the locked dependencies are provisioned:

```sh
python3 scripts/bootstrap-zenoh-native.py --jobs 2
python3 scripts/bootstrap-zenoh-native.py --validate-only
```

`--local-sources` may supply canonical Git repositories, and `--vendor-archive`
may supply the same checksum-verified archive offline. Builds require a fresh
output root. Existing completed references are checked with `--validate-only`.
Sources remain clean and builds live outside the source directories. Release
client libraries use `BUILD_TESTING=OFF`; this bootstrap does not count missing
native unit suites as passing. Simulator interoperability fixtures are separate.

The transport must configure both `GZ_TRANSPORT_ENABLE_ZENOH=ON` and
`HAVE_ZENOH=ON`. The native default implementation remains `zeromq` solely to
record the selected existing C++ build configuration. Every functional child
explicitly selects `GZ_TRANSPORT_IMPLEMENTATION=zenoh`, with isolated TCP
configuration and multicast/gossip disabled. A missing Zenoh build is an error,
never an OFF fallback. The small probe checks actual matching, publication,
receipt and negative-zero preservation on that selected implementation.

`environment.json` is published only after source, package, tool, cache,
installed-inventory and dynamic-link checks succeed. It records actual compiler,
header, library and probe hashes for this build. Validation rejects drift,
foreign Gazebo/Zenoh libraries, escaping links and stale files. Different hosts
may produce different SDK ELF hashes; each must qualify its own source-bound
build and run the unchanged functional fixtures. The manifest does not qualify
an unrelated pre-existing prefix such as the author's local installation.
