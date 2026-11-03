<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Runtime without native physics engines

M1 needs a Linux runtime that can load Rapier f64 primitive worlds without the
DART/MuJoCo libraries or the C++ Gazebo reference SDK. This source batch adds a
selectable build boundary; the live gzweb connection and M1 acceptance are owned
separately. The initial build and build-script source-alias correction were
subsequently qualified in simulator review commit `1a2648de`, including all ten
native-off cases and both unchanged full coverage profiles. That dependency graph
used executor 1.14.0. The current lock selects the compatible upstream 1.14.1
shutdown safety fix. Its native-off product build passed in 52.34 seconds and
ran the fresh browser observations in `docs/evidence/m1-browser`. Final staged
and hosted qualification of this patched graph remains pending; neither a
product build nor browser observations alone imply M1 acceptance.

The default `native-backends` feature stays enabled in Scene, Server and CLI.
Existing full builds, native versions, linker/RPATH checks, tests and verification
profiles retain their behavior. Scene's native dependencies are optional; Server
and CLI explicitly propagate the feature instead of implicitly enabling child
defaults. Without the feature, Rapier and TPE remain available. Selecting DART
or MuJoCo returns typed `LoadError::BackendUnavailable` and a CLI diagnostic; no
backend silently substitutes another. The SDF physics/admission policies and all
engine algorithms are unchanged.

The user-facing runtime build selects the CLI package explicitly:

```sh
cargo build --locked --release -p gz-sim-cli --no-default-features
```

After the complete M1 source batch is reviewed, the ordinary verifier runs
`scripts/qualify-rapier-runtime.py` once before its unchanged full coverage
profiles. This additional check runs the native-off Scene, Server and CLI
targets (four, two and four cases respectively), inspects their actual binary
dependency closure, and requires the
disabled-backend return's line and bounded code region to execute. Do not add
preliminary repetitions of those same tests or the closure check.

The actual Cargo normal/build dependency closure and executable shared-library
closure must exclude the native physics packages and engine/SDK libraries. The
new regression sources cover typed refusal, CLI aliases, real Rapier/TPE stepping
and hostile closure records. Other CLI features can be included using
the check script's `--features` argument. Build the binary with the identical
feature selection before inspecting it.

Select packages explicitly. A workspace-wide build still includes the unchanged
native backend member crates and needs their SDKs. Likewise, other packages may
enable the full default feature through Cargo feature unification. The existing
native-only integration tests remain unchanged and belong to the full profile;
the targeted no-native tests above are additional acceptance evidence. No old
native test is retired or reported as passing in the reduced configuration.
Ordinary Rust dependencies, standard protobuf code-generation tools and platform
build prerequisites remain; this does not claim a toolchain-free installation.

The build-script feature choice lives in `configured_link_directives(enabled,
libdir)`. Each `main` only prints that helper's output. New tests exercise all
three real helpers with disabled/absent and disabled/ambient metadata, enabled
verified RPATH, and enabled missing-metadata failure. Each owning crate includes
its own `../build.rs`, matching its existing native runtime harness. No test
includes another crate's build script through a second relative path. The
original native-link helpers and all old native-link tests remain unchanged.
The relocated cases passed on the prior graph; current-graph qualification must
still execute them without failure or skip.

The auxiliary report, raw counters and four byte-bound executable copies (the
CLI and the three test targets) are retained
under `artifacts/coverage/rapier-runtime`. Only the qualifier's fresh private
`target/rapier-runtime-*` cache is deleted after successful export and verified
copies. The old warm targets and all historical evidence are untouched. The
existing hosted coverage upload includes this auxiliary directory; staged
retention rejects missing, changed or unsafe complete proof and retains available
failed-run diagnostics as partial evidence. Separate native-off counters never
become a whole-profile >95% claim or alter either full-profile denominator.
