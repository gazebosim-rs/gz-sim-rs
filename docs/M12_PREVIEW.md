<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# Integrated Linux preview

M12 runtime/distribution is accepted and published as
[m12-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m12-preview).
The exact final archive uses native runtime `a542cb1b`, viewer `c40418cd` and
separate generator/consumer-guide source `9147120a`. See
[M12_ACCEPTANCE.md](M12_ACCEPTANCE.md) for qualification, installed/demo evidence
and limits. M1–M11 stay preserved; this canonical record does not rewrite shipped
guide bytes or supply its own metadata qualification. Next work is unselected.

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

Package/launcher source, meaningful regressions and independent Source reviews
are complete. Runtime `a542cb1b` passed normal checks and exact hosted
`42228908156` attempt 3 with independent official DATA. Notice/guide child
`9147120a` passed its own full normal hook and exact hosted 42402368577 attempt 1;
independent official DATA is `14b87792…`. Strict separate profile/per-crate gates
and actual parent diff semantics remain unchanged.

Final archive/relocation DATA is `20bd1a84…`; final Ubuntu setup and installed
DATA `1044dc77…` passed. Actual installed robot/browser proof `2c92603e…` is
reused through exact 1560 non-guide/11 behavior byte/mode identity. Root formally
accepted and published the preview with exact asset/tag/Main readbacks. No new
manual replay or broad compatibility/performance claim follows. This canonical
metadata requires its own normal gate and exact CI/official review before its
publication; the qualification pipeline records that actual result separately.

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

These are reproducible maintainer commands. The actual selected release Cargo transcripts bind the two a542 executables; the command examples alone are not evidence for a new build. A
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


## Separately bound notice-generation compatibility tooling

The first local provisional assembly from runtime commit `a542cb1b` refused the
legacy Cargo declaration `MIT/Apache-2.0`. No prefix/archive was published. The
notice-tool correction is isolated from that frozen runtime and the accepted
`c40418cd` viewer. Native binaries, fixtures, installation/capabilities text and
browser bundles remain bound to their original sources. The changed collector
is an unshipped generation tool: release evidence must record its separate full
revision/tree/diff/hash and actual checks. The old runtime gate does not qualify
this new tooling. Hosted provisioning remains a separate unresolved blocker;
local provisional assembly/installation is not M12 acceptance.

The collector now admits only flat deprecated Cargo `ID/ID` alternatives, with
optional surrounding whitespace. It keeps the exact raw declaration in the
inventory and requires source text for every branch. Empty branches and mixed
slash/SPDX forms are refused. Existing SPDX `AND`, `OR` and `WITH` processing is
unchanged; no alternative is chosen and an exception is never replaced by its
bare license.

When named source texts cannot resolve a unit, the existing `extra_notices` map
may explicitly supply this distribution-relative key:

```text
licenses/packages/<full SHA256 of UTF-8 Cargo package ID>/<full SHA256 of UTF-8 unit>.txt
```

The unit is the collector's case-sensitive identifier, including the complete
`Apache-2.0 WITH LLVM-exception` spelling when applicable. Hashes are full lowercase
hex strings, not package names/version shortcuts. A binding for another package,
a different unit or a bare license cannot resolve the missing text. The source
normally comes from a reviewed actual author text: this key is an explicit
provenance binding, not license detection or a legal compatibility decision. A
combined author file may be supplied for multiple units only after its complete
contents and package provenance are reviewed. No registry-wide ancestor fallback
or new filename alias is added. Existing explicit complete-expression fallback
restrictions remain.

For an explicit, separately Source-reviewed package-ID/unit case where the full
author license body is absent, the release procedure may instead bind an immutable,
identified license-publisher or SPDX **full definition**. The current bounded cases
are MIT, Apache-2.0 and EPL-2.0 only. This is not an automatic registry fallback.
The package must also ship the actual unmodified crate `Cargo.toml` and supplied
`Cargo.toml.orig`, README and every provided attribution/notice applicable to that
case. Preserve the raw license declaration, authors and all supplied copyright
data. A short grant/reference record, such as the retained RustAudio record, is
still shipped even though it is not a full definition.

Record the exact package ID/unit, absent author-body finding, publisher/SPDX
identity and immutable full-definition bytes/hash, and each actual author record
in the reviewed input manifest and separately shipped sanitized provenance JSON.
That JSON contains no absolute development paths and distinguishes publisher
material from author material. The definition is **not** an author grant,
attribution, proof that the crate contains a clean canonical license copy, or a
legal verdict. Never invent a holder/year, fill a template placeholder, rewrite
an author record or drop an alternative. Unknown/custom license units or an
incomplete `WITH` exception remain unresolved. Existing actual exception sources,
including BLAKE3's Apache-with-LLVM file, retain their own author provenance;
a bare definition cannot replace an exception.

This documented input distinction does not change the collector API, matching
rules, per-file/read/output/count limits or all-branches requirement. The collector
copies and records the explicit source bytes; the release procedure and Source
review must establish each permitted case and separately shipped author records.

Named package texts keep priority and discovered copyright/attribution notices
remain included. Used bindings appear in each package's `used_unit_bindings` with
`unit`, copied `path` and `sha256`; its `license` remains the raw Cargo string.
These records contain no absolute development source path. Source paths/hashes
remain in generation evidence. Every pre-existing per-file/read/output/count,
namespace, collision, symlink and destination bound still applies. Missing source
texts remain errors; no metadata ID or branch may be dropped to finish a package.


### Raw empty registry placeholders

A separate notice-tooling correction preserves an unusable, exact zero-byte
registry-package-root file only when its basename is in the existing generic
`LICENSE`/`COPYING` set, `license_file` is absent, and the declaration has at least
two distinct units with no `WITH` exception. Ordinary nonsymlink/in-bound path
checks and an actual zero-byte read are required, including a final recheck.
Whitespace, binary, oversized, named-license, nested, notice/copyright, local or
git-source files and declared license files retain the strict nonempty rules.

The empty file never enters the license-text cache, package `texts` or a unit
binding. Every unit must independently resolve through unchanged nonempty named,
explicit scoped or permitted complete-expression inputs, or collection still
fails before destination creation. The original empty bytes are copied under a
separate `source-placeholders/` package notice path. Per-package
`source_placeholders` records the relative path, original basename, zero size,
empty SHA-256 and `used_as_license_text: false`. No author body is synthesized.

A separate cache accounts for these source files; candidate/file/output count
limits still include them, zero-length output consumes an output slot, and all
existing byte limits and cross-graph deduplication remain. This new generator
revision and its actual checks must be bound separately. The earlier tooling
tree and its 38-test evidence, runtime `a542cb1b` and browser `c40418cd` remain
frozen; none qualifies this new change or establishes package/CI/M12 acceptance.
