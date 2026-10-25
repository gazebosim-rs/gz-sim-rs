# Local SDF resources

The gz-assets workspace crate loads local UTF-8 SDF resources before simulator
construction. It has no Bevy dependency and performs no downloads. LocalResources
implements the independent SDF ResourceResolver contract, using the producer's
pure model.config selector rather than another XML/version implementation.

The CLI accepts local world files and configured world directories. Includes may
resolve model directories or explicit SDF files. A standalone top-level model is
not wrapped into a world by this CLI. `--resource-path` adds roots after existing
SDF_PATH and GZ_SIM_RESOURCE_PATH entries; duplicate entries retain their first
position. Configuration is captured per resolver without mutating process globals.

For include lookup, explicitly configured URI maps precede cwd. Cwd precedes a
direct absolute candidate and the remaining ordered roots. Native SDF filesystem
append concatenates even absolute-looking suffixes: a matching cwd shadow of an
absolute include wins, and `model:///name` still searches inside the mapped root.
The narrow append helper uses standard OsString operations to preserve this rule;
Path::join would discard the earlier root. Explicit read_path uses normal absolute
path semantics, independently of include lookup.

URI suffixes retain literal spelling. `file://a%20b.sdf` finds the literal-percent
filename; `file://a b.sdf` finds the space filename. The including file's directory
is not an implicit include root. Failed candidate stat calls are treated as misses,
including symlink loops, matching native search fallback. Errors after a resource
is selected, or during an explicit read_path, remain typed errors.

A directory uses model.config, falling back to deprecated manifest.xml only when
the model.config stat fails. The selected filename is appended literally and is
not reinterpreted as a URI. No alternate file/version is tried if reading it fails.
A selected directory is not recursively treated as another configuration. Version
selection does not establish that the selected content is supported by the loader.
The pinned selector's qualified XML grammar and safe-error boundaries still apply.

Canonical file URLs are lossless source identities for resources and configuration
warnings, including non-UTF-8 filenames and symlink aliases. They do not change the
native literal-percent lookup policy. The canonical target is resolved before it
is read, keeping identity/content associated when an input alias changes. Loading
is synchronous and outside simulation steps; this is not an atomic snapshot of a
concurrently changing filesystem.

The CLI caps each XML read at 64 MiB by default (`--max-xml-bytes`). Reads use
Read::take(limit+1), so a changing file cannot bypass the byte cap. The warning log
holds at most 1,024 warnings by default (`--max-resource-warnings`). Exceeding either
budget fails loading explicitly. A short Mutex protects only diagnostic storage,
not IO or simulator state. Known manifest deprecation warnings are recorded before
reading/parsing it, so malformed manifests retain them. Selector warnings are
recorded in order; its separately documented partial-error warning limitation is
not reconstructed in the consumer. CLI warnings also survive failed world loading.

Tests construct both real physics engines through configured local includes,
then drop the resolver and delete the source file before stepping/resetting.
Lookup/configuration/error/budget/identity and CLI controls retain the prior
physics assertions and all seven actual C++ transport scenarios. No source file
or filesystem operation is retained in the loaded-scene reset blueprint.

Eighteen installed-native SDK controls establish lookup order, absolute-suffix
concatenation, stat failure fallback, configuration path assembly, legacy-warning
ordering and actual Root.Load include behavior. Source/binary/library hashes are
in planning/reference/local-resources.json. Reproduce with:

```sh
python3 scripts/probe-local-resources.py --dependency-prefix "$DEPENDENCY_PREFIX" --work-dir /tmp/local-resource-probe --output /tmp/local-resource-proof.json
```

These controls use an installed SDFormat SDK, with linked dependencies qualified;
they do not close the complete reproduced native simulator/Fuel baseline.
Offline Fuel lookup, downloads/cancellation, SDK install-directory defaults,
resource-path services, package discovery, decoded mesh/material/image caching,
standalone-model wrapping and the broader headless P2 milestone remain open.
