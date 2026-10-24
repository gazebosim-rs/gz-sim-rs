# Repository disposition and source audit

This audit is based on tracked local main-branch source, not just repository
descriptions. Exact commits are in `planning/baseline.json`. The generated
`planning/source-inventory.json` lists public-header and test-source candidates.
It deliberately does not claim runtime tests have been enumerated or passed.

## Disposition

| Existing repository | Destination | Rationale and retained responsibilities |
| --- | --- | --- |
| gz-sim | gz-sim-rs workspace | Bevy ECS runtime, world lifecycle, robotics systems, services, logging, distributed/levels behavior, external GUI protocol |
| gz-sensors | Workspace sensor modules/crates | Scheduling, sensor models, calibration, noise, messages, rendering sensor pipelines; usable without the full server |
| gz-physics | Workspace physics API and adapters | Capability contracts and DART/MuJoCo/TPE/Rust integration; discard C++ template feature machinery |
| gz-math | Small workspace gz-math crate plus dependencies | Retain only missing robotics semantics; no wholesale vector/matrix/geometry port |
| gz-common | Focused workspace modules plus dependencies | Resource resolution, assets, terrain, media, batteries, and domain semantics remain; generic utilities disappear |
| sdformat | sdformat-rs | Independent XML/schema/semantic model support; no ECS or rendering dependency |
| gz-fuel-tools | gz-fuel-tools-rs | Independent, actively owned; consume Rust API and agreed fixtures |
| gz-transport | gz-transport-rs | Independent interoperable networking and logging |
| gz-msgs | gz-msgs-rs | Independent schemas/code generation/descriptors/tooling |
| gz-cmake | Drop | Cargo and focused native build support replace Gazebo's build abstraction |
| gz-rendering | Drop implementation | Bevy rendering; preserve retained scene/sensor capabilities and their tests |
| gz-gui | Out of scope | Separate user-owned replacement; retain server-facing contracts it needs |
| gz-utils | Retire | std, clap, tracing, and ordinary Rust ownership cover the examined functionality |
| gz-tools | Retire repo; CLI crate in workspace | Preserve dispatcher, command discovery/help/version selection where required; use executable subcommands |
| gz-plugin | Retire repo; extension crates in workspace if runtime loading required | Bevy plugins for source integration; narrow versioned ABI for dynamic integration |

The last three are architecture recommendations subject to the decision register.
There is no demonstrated cross-project reason for another general utility or
plugin repository at this stage. Extract one later only after at least two real,
independent consumers establish a stable shared contract.

## gz-utils review

The public headers include Environment, ImplPtr, NeverDestroyed, Subprocess,
SuppressWarning, ExtraTestMacros, and Logger/SplitSink. The CLI component vendors
CLI11. This is more than PIMPL macros, but it does not justify a Rust utility port.

| Existing responsibility | Replacement |
| --- | --- |
| PIMPL / reference-counted implementation pointers | Private Rust fields/modules, Box/Arc where ownership requires them |
| NeverDestroyed / globals | Explicit application resources; OnceLock only for true immutable process state |
| Environment | std::env, captured configuration, platform tests; avoid process-global mutations after threads start |
| Subprocess | std::process with explicit pipe draining, cancellation, exit status, and timeout policy |
| Warning suppression | Scoped lint settings with reasons; no global warning-disabling shim |
| Extra test macros | Native test configuration plus a visible capability/skip ledger |
| CLI11 | clap; preserve accepted argument grammar and exit codes through fixtures |
| Logger / SplitSink | tracing and configured subscribers; preserve severity and destination semantics where used |

Source anchors: `gz-utils/include/gz/utils`, `gz-utils/log/include/gz/utils/log`,
and `gz-utils/cli`. Migration tests cover env handling, child shutdown, pipe
deadlocks, and log destinations rather than recreating C++ helper types.

## gz-tools review

`gz-tools/src/gz.in` is a Ruby dispatcher. It reads YAML registrations, groups
commands by library versions, selects versions, provides help/command listings,
and loads Ruby/library entry points. `src/cmdUNIT_gz_TEST.rb` contains existing
tests. A C++-only test search would miss these.

Retain user-facing discovery and dispatch in workspace `gz-cli`, but launch
independent executable subcommands with argument arrays, not a shell. Define a
small versioned command manifest and collision rules. Keep help, exit status,
stdout/stderr, signal forwarding, and version selection traceable to old tests.
Legacy YAML support is a compatibility adapter, not the new extension ABI.

Independent repos can ship `gz-fuel`, `gz-topic`, `gz-service`, `gz-msg`, or
`gz-sdf`-style executables without depending on the monorepo. Final names require
an inventory of current commands and Fuel's existing install layout. Package
ownership must prevent two repositories installing competing `gz` binaries.

## gz-plugin review

The old library registers classes, queries interfaces, loads shared libraries,
manages library/object lifetimes, supports factories, and exposes `gz plugin`.
These behaviors are not all replaced by implementing Bevy's `Plugin` trait.

Built-in systems should be source-built Bevy plugins. If runtime loading is
required, place `gz-extension-api` and `gz-extension-host` in the workspace.
Export opaque handles, fixed-width values, slices with explicit ownership, and
versioned function tables through a C ABI. Never export Bevy World/Entity memory
layouts, Rust trait objects, Vec, String, or references across that ABI.

Keep loaded libraries alive until their callbacks, data, tasks, and objects are
destroyed. Prefer no unload during a running world in the first release. Do not
claim hot unload or binary compatibility with old gz-plugin libraries. Preserve
configuration discovery and useful inspection through the new contract.

## gz-common review

The inspected tree contains graphics/mesh loaders and animation, AV encoding,
geospatial DEM/heightmaps, event helpers, profiling, filesystem/URI/resource paths,
battery and material models, CSV/data frames, worker pools, and testing helpers.
It must not disappear under a blanket “use std” decision.

Move resource policy and loader integration into `gz-assets`, media adapters into
rendering/recording modules, and domain-specific battery/environment behavior
into relevant systems. Prefer existing mesh/image/geospatial libraries after
testing Gazebo fixtures. Collada material/animation fidelity, terrain coordinates,
FFmpeg integration, and URI search precedence are explicit migration work.

There is no `gz-common` catch-all crate in the proposed layout. Shared modules
need a named responsibility. sdformat-rs accepts a resource resolver interface;
it does not depend on gz-assets or Fuel to resolve an include.

## Inventory observations

- The simulator has 78 directories under `src/systems` at the pinned commit.
  All appear in the inventory, including media/export and less common robotics
  systems. The final release cannot be declared complete after only the common
  diff-drive and camera examples work.
- Message sources currently include 216 `.proto` files declaring proto3. Test
  real baseline schemas; do not assume proto2-only presence rules from historical
  Gazebo knowledge. Historical protocol support is a separately enumerated input.
- `gz-sim/include/gz/sim/System.hh` specifies priority/load ordering and that
  UpdateInfo.simTime refers to the end of the current step. Preserve these
  observable semantics in an explicit Bevy schedule.
- MuJoCo's README is stale relative to its source. `JointFeatures.hh` and
  `SimulationFeatures.cc` implement joint state/commands and contact reporting;
  other APIs are commented out or constrained. Inspect source plus runtime probes
  before publishing a backend support table. Do not copy the README's gap list.
- Existing performance runners in `gz-sim/test/performance` and benchmarks in
  `gz-sim/test/benchmark` are useful starting points, not enough evidence by
  themselves for the rewrite's performance claim.
- Fuel's current plan includes a Rust core and C++ facade. Respect that owner's
  compatibility commitment and consume its Rust library directly when possible.

## Inventory completion in P0

Enumerate CTest registration, expanded gtest parameterizations, Ruby tests, Python
tests, examples used as tests, backend-specific tests, and GPU tests. Include
schemas, worlds, resources, environment flags, public services/topics, Python
entry points, and every system configuration parameter. Compare those runtime
lists with the static source inventory and explain every difference.

Public header lists are candidates, not an API-completeness claim: templates,
generated headers, SDF headers, private test hooks, and script APIs need separate
inspection. Preserve upstream licenses and fixture provenance when importing.
