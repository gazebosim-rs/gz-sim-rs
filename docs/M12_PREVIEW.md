<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Integrated Linux preview

M12 is in implementation. No package, clean-machine installation or release
qualification is accepted yet. M1–M11 acceptance remains recorded separately.

## Package contract

The first supported target is Ubuntu 24.04 on x86_64. A relocatable archive
contains the native-free simulator, existing finite canonical drive-command
sender, the unchanged M8 robot world, a loopback Zenoh configuration, the six
unchanged accepted viewer files, license notices and a file manifest. Installed
execution requires Python 3, the documented system-library closure and a browser
with working WebGL2 or WebGPU. It requires no Rust compiler, source checkout,
native physics SDK, renderer worker or remote asset service.

The simulator is built with `--no-default-features` and without `rgb-camera`.
Rapier f64 remains authoritative. CPU IMU/lidar, drive, admitted joint controllers,
world lifecycle and bounded recording/replay keep their maintained APIs. RGB/depth
worker and native backend distributions are excluded from this initial package;
their accepted source and behavior remain retained. The capabilities matrix must
separate available interfaces from what the robot demo exercises.

The viewer files retain their relative imports under `web/target/`. They are
copied as ordinary files, never distributed as development RAM symlinks. Their
accepted source/build identities and separate 14,000,000-byte gzip budgets remain
bound to the shipped bytes. No viewer Rust, wire protocol or controls are changed.

A Python-standard-library launcher serves only `web/` on `127.0.0.1:8081`, starts
the packaged world and simulator with an explicit local Zenoh listener, and prints
the existing `backend=primitive&ui=full` URL with its loopback WebSocket endpoint.
It starts paused. An occupied port fails without killing its owner. Shutdown
reaps only processes owned by this launch. The package's finite command helper
sends forward and turning commands using the same partition and ends successful
commands with zero-target braking. Its maintained upstream Zenoh configuration
connects only to the requested endpoint, with no listeners, multicast or gossip. Pause/Reset remain maintained viewer controls.
No new teleoperation, transport or serving framework is introduced.

The assembler accepts explicitly selected native binaries and a viewer bundle,
checks input types, identities, resources and notices, and writes a new prefix
without replacing existing output. The installed manifest records repository
revisions, selected feature profile and every distributed file's hash. Paths and
contents must survive installation outside the development checkout. A failed
assembly leaves existing output and source untouched.

## Completion evidence

Finish package/launcher implementation, meaningful malformed-input, relocation,
occupied-port, manifest/resource and shutdown regressions, the supported capability
matrix and installation guide before independent Source review. Then run one
normal required staged gate and exact hosted CI for each changed repository,
retaining strict coverage, licensing and all existing mandatory jobs.

After those gates, build the source-bound native binaries and assemble the
package serially. A clean supported installation must run the integrated robot
using real canonical forward/turn commands and the live maintained browser,
including pause/reset and clean shutdown. This is installation evidence; accepted
M1–M11 manual qualifications are not repeated. Report actual dependencies and
limitations. M12 is at most five working days including review and checks; split
optional scope rather than relaxing this exit.

## Maintainer input record

The assembler never builds binaries, resolves dependencies, downloads assets or
chooses license alternatives. Its explicit input record is assembled from the
actual final build and accepted viewer evidence. This is a maintainer interface;
installed users run `bin/gz-preview` and do not need these tools or source paths.

After the completed source batch and required checks, build the two native inputs
from the exact release commit with the pinned toolchain. Preserve Cargo's actual
JSON compiler-artifact output and use separate explicit output records:

```sh
cargo build --locked --release -p gz-sim-cli --no-default-features --message-format=json-render-diagnostics > native-cli.jsonl
cargo build --locked --release -p gz-sim-server --no-default-features --example drive_command --message-format=json-render-diagnostics > drive-command.jsonl
```

These commands are authored instructions, not recorded build successes. A
maintainer may select an isolated `CARGO_TARGET_DIR`; the input paths must then
match the actual `executable` fields in those transcripts. The assembler requires
native-free executable artifacts, no test profile, and release opt-level 3. It
checks the ELF class/machine and exact binary/transcript hashes. The final package
qualification still checks actual system libraries and clean installation.

The top-level JSON keys are:

| Key | Required value |
| --- | --- |
| `schema_version` | `1` |
| `target` | `ubuntu-24.04-x86_64` |
| `repositories` | `gz-sim-rs` and `gzweb-rs` mapped to their actual 40-hex commits |
| `simulator` | Binary record for the built `gz-sim-rs` |
| `drive_command` | Binary record for the built `drive_command` example |
| `viewer_root` | Absolute real directory containing the accepted six-file layout |
| `viewer_files` | Each of the six relative paths mapped to its exact SHA-256 |
| `cargo_metadata` | The native and both accepted viewer configuration records below |
| `extra_notices` | Distribution-relative notice names mapped to actual absolute text files |

Each binary record contains absolute `path`, exact `sha256`, absolute
`cargo_messages`, and exact `cargo_messages_sha256`. A matching transcript entry
must identify that executable and its maintained target/package. No historical
ELF or invented compiler event substitutes for a release input.

Each Cargo metadata record contains an absolute JSON `path`, exact `sha256`,
`repository`, matching repository `commit`, `configuration`, and a nonempty unique
`selected_package_ids` list. Configurations are exactly `native`, `viewer-webgpu`
and `viewer-webgl2`; the last two belong to `gzweb-rs`. The metadata documents
retain their real `workspace_root` and package source/manifest/license fields.
Select every native/helper compiler-artifact package, and the actual resolved
accepted viewer feature graphs. Build-only packages may be included
conservatively; the notice inventory does not call them linked code. Metadata
configuration labels are provenance supplied by the reviewed release procedure;
labels alone are not proof that a feature graph was compiled.

Copy the six accepted viewer bytes into a real input tree while preserving their
relative imports and expected hashes; do not pass the development preview's
RAM forwarding links as an installable tree. The assembler checks both actual
Wasm headers and separate gzip sizes against the retained 14,000,000-byte limits.
Unchanged accepted browser bytes need no new renderer build. Their exact source,
feature and production-build identity must remain recorded separately.

Explicit notices must include `licenses/Apache-2.0.txt`, `OSRF-NOTICE.txt`,
`Geist-OFL.txt`, `Rust-COPYRIGHT.html`, `Rust-LICENSE-APACHE.txt` and
`Rust-LICENSE-MIT.txt`, taken from the actual project, embedded font and selected
Rust toolchain sources. Preserve the actual standard-library
`COPYRIGHT-library.html` bytes as `Rust-COPYRIGHT.html`; the complete compiler
distribution is not shipped and its separate full-toolchain copyright inventory
is not a substitute for this selected standard-library notice. Also required are `Viewer-mesh-loader-LICENSE.txt` from viewer
`crates/gz-collada/src/vendor/LICENSE-mesh-loader` and
`Viewer-proto-LICENSE.txt` from `crates/gz-proto/proto/LICENSE`. Supply any other
applicable embedded non-Cargo notices from their maintained source files. The selected
Cargo packages supply their actual license and attribution texts. An unresolved
text or SPDX branch fails assembly; no text is invented or fetched. The resulting
`notices.json` and copied texts are part of the installed file manifest. Absolute
development input paths remain in release evidence, not installed metadata.

Invoke the assembler with the completed actual record:

```sh
python3 scripts/package-preview.py --spec package-inputs.json --output gz-preview-VERSION --archive gz-preview-VERSION.tar.gz
```

Both outputs must be new. Archive members start with `gz-preview-VERSION/` in
this example. Extract into a new parent and preserve the complete directory.
A failure never replaces existing output or alters input source. A filesystem
failure can retain a new owned incomplete archive or a completed prefix without
a completed archive; that failed invocation is not release acceptance.
