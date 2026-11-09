<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M35 ApplyLinkWrench: selected bounded contract

This is the selected contract and source-implementation guide. M33
`17dd62f27234eab1482abb233389d167dafa31b4` is now accepted at qualified Main.
The original-system component capture and its independent retained DATA audit
have passed. Rust verification, full normal/strict coverage, exact hosted CI and
independent local/official DATA remain pending; M35 is not accepted. The
five-working-day scope includes review and all mandatory gates. M32 remains the
installed Latest.

The fifth full normal hook exited 1 after 1,464.264 seconds, with ten headless
failures across four targets. The new odometry numeric, signed-zero and 663-packet
replay controls passed; that does not qualify the complete hook or coverage.
The retained failures exposed M35 scalar entity-reference handling, a test that
tried to load an already-refused Rapier COM configuration, and template controls
whose collision and visual share a local name. This correction keeps the fixture
and template assertions unchanged, accepts contiguous plain text/entity references
without admitting scalar smuggling, and tests COM refusal at its actual boundary.
The remaining shared and M34 corrections require the complete reviewed batch and
another full normal hook; no partial result is phase acceptance.

## Original behavior and admitted subset

The original system is `gz::sim::systems::ApplyLinkWrench`, configured on a
world with filename `gz-sim-apply-link-wrench-system`. At native Sim commit
`71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`, its default inputs are:

| Input | Canonical message | Effect |
| --- | --- | --- |
| `/world/<world>/wrench` | `gz.msgs.EntityWrench` | Add one wrench for one advancing step. |
| `/world/<world>/wrench/persistent` | `gz.msgs.EntityWrench` | Append another persistent wrench; do not replace earlier entries. |
| `/world/<world>/wrench/clear` | `gz.msgs.Entity` | Remove every persistent entry for the resolved target; leave one-shot entries intact. |

Native `PreUpdate` drains clears first, including while paused. It then returns
if paused, retaining all one-shot commands. On an advancing update it consumes
one-shots in FIFO order, followed by all persistent entries in insertion order.
The native guard checks only `paused`: an unpaused `PreUpdate` with zero `dt`
also consumes one-shots and applies persistent entries. This is distinct from the
selected Rust advancing-step policy below. Multiple additions sum, including
repeated additions to the same link. A clear
already queued for the next update also removes persistent additions whose
callbacks completed after the clear callback but before that update. There is
no cross-topic transport FIFO or delivery acknowledgment in this system.
A successful Publish or HasConnections is not proof of callback completion.

Native entity lookup gives nonzero ID precedence over name/type; otherwise it
resolves name plus type. Native also accepts models through their canonical
link, link-frame force offsets, persistent commands for future insertion, and
initial SDF `<persistent>` rows. Those broader cases are not silently inherited.
The selected M35 subset is:

- One explicit world plugin, with the three default topics or nonempty validated
  `topic`, `topic_persistent`, `topic_clear` overrides. Resolve each through the
  existing Server namespace/remapping rules. All three final keys must differ
  and must not alias any existing service, clock, command or typed publication.
- Explicit `Entity.type=LINK`. A nonzero ID selects that exact current public
  link identity; name is ignored as in native ID precedence. With ID zero, require
  an exact unique scoped link name, without fuzzy/suffix lookup. Reject empty,
  ambiguous, missing, model, static, jointed and unsupported targets atomically.
- Preserve the existing wrench backend domain: a dynamic, unjointed Rapier link
  in a top-level single-link model, with a valid physics owner/body binding.
  Reject TPE and other backends in this slice. Do not widen existing input APIs.
- Require both enclosing `EntityWrench.entity` and `EntityWrench.wrench`
  messages, matching the native callbacks. An explicitly present empty Wrench
  is a valid zero wrench; an absent Wrench is rejected. Finite world-axis force
  (newtons) and torque (newton-metres) are admitted. Missing force/torque vector
  fields inside the present Wrench mean zero, matching protobuf/native defaults. `force_offset` must be
  absent or exactly zero, with all supplied coordinates finite. Headers do not
  schedule commands or create another reference frame.
- Root-selected bound: require the target's inertial centre translation to
  be exactly zero. Native zero offset applies force at
  the **link origin**, whereas `queue_link_world_wrench` applies at the **centre
  of mass**. These coincide for a centred target. Rejecting translated centres
  avoids inventing origin-to-COM torque arithmetic or changing old queue APIs.
  Test zero with finite coordinate equality to `0.0`: both `+0.0` and `-0.0`
  are admitted for centre/force offsets. Any NaN, infinity or nonzero offset is
  rejected. Finite signed-zero force/torque is accepted; there is no wire-level
  signed-zero preservation or new bitwise arithmetic equivalence claim.
- Rapier loading already rejects translated inertial centres. The public
  regression retains that exact loader refusal; a separate private metadata-only
  regression verifies M35's target error and unchanged queues without stepping a
  tampered world. It does not claim an admitted offset-COM Rapier body. The old
  general wrench queue remains separately exercised on an admitted centred body.
- Refuse initial SDF `<persistent>` rows, unknown attributes/children, repeated
  scalar parameters and malformed scalar/comment-tail XML in this first slice.
  Accept the empty/default plugin and the three topic overrides only; a verbose
  override is outside this selected subset. This previously unsupported plugin
  does not lose an accepted behavior by admitting this explicit subset.
  XML character references are decoded as contiguous plain scalar text, including
  a leading reference; comments, CDATA, nested elements and trailing fragments
  remain outside that grammar.

The centred-target, topic-only parameter and buffer restrictions are explicit
Root decisions. The complete implementation and shared integration require
independent Source review before any execution. A larger target/configuration domain would
need its own recorded tests and native evidence, not an implicit extension.

## Bounds, failure and lifecycle

Use fixed bounded one-shot, persistent-entry and clear buffers (maximum
256 entries **each** per configured world), plus the existing bounded Server input
queues, message-byte limit and command budget. Reject capacity excess without
evicting or replacing an earlier command. Persistent storage must retain separate
entries so add/clear behavior and native summation order stay visible. Queue-full
transport drops use the existing world-lifetime ingress counter, not a stale
per-endpoint metric. Invalid packets also consume bounded CPU budget.

For all commands admitted to a tick, clears have priority over application,
including a paused tick. Network messages not yet admitted remain bounded pending
input; native's unbounded callback queues are not promised. One-shots survive
pause and are consumed only after a successful advancing application. Persistent
entries are added again on every advancing tick. For Rust, zero-dt updates retain
one-shots and do not apply persistent force; this preserves the existing input
API rather than claiming native unpaused-zero-dt equivalence. The clear hook must
still execute on every paused or zero-dt update, before the existing
`inputs::apply` zero-dt return. Paused/non-due network handling
must never invoke physics or silently consume a one-shot.

Resolve names to public IDs at admission and retain the matching current link,
world/epoch and physics-owner identity. Never retarget an old command merely
because a replacement link has the same name. Removal discards that link's
one-shot/persistent/clear entries. An unknown/future target is rejected, not
retained for eventual insertion. Both direct core reset and loaded reset clear
all dynamic commands. Loaded reset reconstructs only admitted initial metadata
and fresh input leases; it does not resurrect old commands. This is an explicit
Rust policy: the native system has no `ISystemReset` implementation and its
reload/future-insertion behavior is not a reset-equivalence oracle.

Admission must check each finite component and every intermediate sum. For each
target, start a shadow force/torque sum with the actual existing `PendingInput`
wrench, then add **each** one-shot in FIFO order, then **each** persistent entry
in insertion order, checking every component after every addition. Do not first
aggregate the system total and add it once: that changes floating-point grouping
and can hide an intermediate overflow. A rejected command leaves the previous
buffers and pending physical inputs unchanged. Before applying a tick, validate
all target identities and these sequential shadow sums before committing any
target, then commit the already validated final shadow values atomically. Aggregate overflow cannot apply
force to an earlier link and fail on a later one. Existing backend runtime faults
remain terminal; no backend rollback or successful packet is fabricated.

The configured world metadata may survive as an immutable component. Without the
plugin, there must be no queues, endpoints, callbacks, per-tick polling or owned
heap state. After the last eligible link is removed, release the active buffers,
callbacks and three input leases; retain only metadata needed for a later loaded
reset. Root owns reactivation at admitted load/spawn/reset boundaries. Direct Scene
template spawn can reactivate the configured system. The existing Server spawn
API promises not to add IO streams: while M35 is configured but inactive, Server
template spawn returns `ApplyLinkWrenchReactivation` before mutation. IDs, epoch,
state and leases stay unchanged. Active-M35 and unconfigured Server spawning
retain their existing behavior; loaded reset may reconstruct fresh IO normally.
No dormant declaration or dynamic plugin framework is added. Shutdown disables callbacks before closing
IO, discards commands and joins existing IO ownership outside stepping. A late
callback from an old epoch/lease cannot re-enter the restored world.

## Selected interfaces and Root integration

These signatures describe the selected source integration contract; execution
is still unqualified. Root owns all existing-file changes and final shared integration. Private
Server/Scene fields are not assumed available to an integration test.

Scene leaf `apply_link_wrench.rs`:

- Immutable public `ApplyLinkWrenchConfiguration { topic, persistent_topic,
  clear_topic }` on the configured world; an initial getter yields the world
  public ID and configuration for reset preflight.
- Public `ApplyLinkWrenchMode::{Once, Persistent}` and
  `queue_apply_link_wrench(&mut Simulation, link_id, force, torque, mode)` plus
  `clear_apply_link_wrench(&mut Simulation, link_id)`. Return a typed
  `ApplyLinkWrenchError` distinguishing unconfigured/unsupported target, invalid
  value/overflow, capacity, stale identity and the exact underlying `InputError`.
- Private `is_plugin`, `parse(world) -> Option<Plan>`, `Plan::validate` and
  `Plan::build`, with `install` and `retire_if_unused`. Parse raw XML strictly;
  do not add Cargo or parser dependencies. Root registers the installed apply
  hook in `Phase::PhysicsPrepare` with `.before(inputs::apply)` and makes the
  existing apply function `pub(super)`. The hook processes clears before its
  own zero-dt application return, so paused/zero-dt maintenance is retained.
  A reset hook clears dynamic state.
- Necessary Root-owned seams in existing `inputs.rs`: a read-only
  `validate_world_wrench_batch(&World, &[(u64, [f64;3], [f64;3])]) -> Result<(), InputError>`
  that checks existing targets/bindings and all sequential shadow sums without
  inserting a component, plus
  `queue_world_wrench_batch(&mut World, &[(u64, [f64;3], [f64;3])]) -> Result<(), InputError>` accepting the
  same ordered individual entries. Start each target shadow at its existing
  pending wrench, retain once-FIFO then persistent-insertion order, validate
  every intermediate sum, and atomically commit the validated final shadows.
  Neither helper accepts a pre-aggregated system total as a substitute for the
  ordered entries.
  Share the existing free-link validation and wrench accumulation with the
  singleton API. Do not duplicate private binding rules or validate by submitting
  a zero wrench (which would mutate state). Root owns this existing-file change
  and exact schedule ordering, including ordinary direct queue coexistence.

Server leaf `apply_link_wrench.rs` and IO leaf:

- A prepared input owner derives the one world identity, configuration and three
  final keys from current/initial Scene metadata. Validate all reservations before
  declaring any endpoint at initial setup or reset. Poll one-shot, persistent and
  clear inputs fairly within the existing command budget, then call the typed
  Scene API. Decode `EntityWrench`/`Entity` only within message bounds and require
  the exact canonical type attachment. Treat semantic rejection separately from
  transport queue drops; do not add an acknowledgment protocol.
- `prepare`, `poll`, `prepare_reset`, `bind`, `retire` and `shutdown` methods are
  analogous to existing `Forces`, but own this system's three input keys. The IO
  setup contains bounded senders, active/generation guards and world-lifetime
  ingress metrics. Declarations live in the existing WorldIO worker. Callbacks
  never own a Simulation, resolve entities or call physics.
- Root supplies Server `ApplyLinkWrenchIdentity`/reset error variants, explicit
  received/rejected/applied/cleared counters, optional active owner, WorldIO
  setup/retirement channels, all-family key reservations and reset/shutdown glue.
  Root also owns Scene exports, world-plugin admission, initial snapshot,
  activation/schedule/removal hooks and shared six/typed-family collision tests.

Remaining callable hooks to wire together (type aliases below name only these
concrete records; they do not add a general system interface):

| Owner | Signature / integration point |
| --- | --- |
| Scene leaf | `is_plugin(&sdformat::Plugin) -> bool`; `parse(&sdformat::World) -> Result<Option<Plan>, LoadError>` |
| Scene leaf | `Plan::validate(&self, &sdformat::World, PhysicsEngine) -> Result<(), LoadError>`; `build(Plan, &mut Simulation, Entity) -> Result<(), LoadError>` after admitted bodies exist |
| Scene leaf / Root registration | `install(&mut Simulation)`; `retire_if_unused(&mut Simulation) -> Result<(), bevy_ecs::schedule::ScheduleError>`; `apply(&mut World)` for ordered PhysicsPrepare; `reset(&mut World)` for dynamic clearing |
| Scene public leaf | `queue_apply_link_wrench(&mut Simulation, u64, [f64;3], [f64;3], ApplyLinkWrenchMode) -> Result<(), ApplyLinkWrenchError>`; `clear_apply_link_wrench(&mut Simulation, u64) -> Result<(), ApplyLinkWrenchError>` |
| Root initial snapshot/export | `initial_apply_link_wrench_configuration(&Simulation) -> Result<Option<(u64, &ApplyLinkWrenchConfiguration)>, SceneResetError>`; borrowed immutable world identity/configuration only, never queued commands; Server clones configuration into its prepared value |
| Server leaf | `ApplyLinkWrenches::prepare(&Simulation, &ServerOptions, &BTreeSet<String>, &Arc<AtomicU64>) -> Result<Option<(Self, WrenchSetup)>, Error>` for current metadata, keys, bounded channels and ingress metrics |
| Server leaf | `poll(&mut self, &mut Simulation, NonZeroUsize, &mut Counters) -> Result<(), Error>`; `retire(&mut self, &Simulation)`; `shutdown(&mut self)`; `is_empty(&self) -> bool` |
| Server reset leaf / Root glue | `prepare_reset(&Simulation, &ServerOptions, &BTreeSet<String>) -> Result<Option<Prepared>, Error>` validates initial world identity/configuration and all three keys before any reset/IO change; `Prepared` contains that immutable binding. After loaded reset, `bind(&mut self, &Simulation, &Prepared) -> Result<(), Error>` validates restored ownership; old dynamic queues/epoch leases are not reused. |
| IO leaf / existing WorldIO owner | `WrenchSetup` contains the three final keys, bounded senders, active/epoch guards and ingress metric; `WrenchDeclarations::new(&zenoh::Session, WrenchSetup) -> zenoh::Result<Self>`, `begin_shutdown(&self)`, `reconcile(&mut self) -> bool` and `is_empty(&self) -> bool` follow existing `ForceDeclarations` callback-release witness ownership. Derive the process identity from the actual session and a fresh per-declaration node UUID, matching the maintained presence scheme. |

Root stores `Option<ApplyLinkWrenches>` and replaces or drops the three input
leases at setup/reset/removal/shutdown boundaries, alongside the existing input
systems. Concrete prepared IO replacement follows that existing path; a new
worker or a private mutable simulation getter is not part of these signatures.

Root additionally re-exports the read-only `apply_link_wrench_active(&Simulation)
-> bool` installed-marker getter, supplies lifecycle-only `needs_activation` and
`has_eligible_outside` calls into the leaf, and calls
`discard_apply_link_wrenches(&mut Simulation)` on Server shutdown. Discard clears
only dynamic commands, without changing time, physics or configuration. Scene
install is called after initial ownership capture and at admitted reconstruction
boundaries; it never guesses private initial-state ownership.

The public APIs are testable through normal Scene loading and Server options;
no new mutable-Simulation escape hatch or private integration-test attachment is
needed. The separate M34 publisher only shares Root's lifecycle/key-reservation
integration. Neither author edits the other's leaves or common files.

## Native packet plan and acceptance

Use the original system as a command/component oracle, not a different physics
engine or trajectory oracle. The reviewed procedure compiled one original
`ApplyLinkWrench.cc` object and packet-local DSO using the retained CMake flags/link
recipe. A separately reviewed affected-only continuation compiled the corrected
helper and captured the eight wrench modes. Neither procedure installed into or
mutated the SDK. The leased C++ file has two explicit build roles: its default canonical
peer mode uses only the maintained transport/message SDK; a procedure-only
`GZ_APPLY_LINK_WRENCH_ORACLE` compile definition enables the original-system
Sim/ECM oracle mode. Bind the definition and libraries in each actual build; do
not make the ordinary peer's transport-only recipe implicitly require Sim.
The oracle loads that actual DSO, configures the original world system and
records actual `ExternalWorldWrenchCmd` force/torque components before any physics
consumer clears them. Record original source/recipe/helper/compiler and mapped
provider bytes, actual child exits, partial stdout/stderr, final guards and all
current SDK inputs through the maintained procedure. No claim of native measured
acceleration, CPU timing, native RNG or trajectory follows from component rows.

Source pins already read (SHA-256):

| Original input | SHA-256 |
| --- | --- |
| `ApplyLinkWrench.cc` | `9c52516165aacf47246a956b7fdcb828832b7e4c7ebed69e2788ada22c494561` |
| `ApplyLinkWrench.hh` | `34ae360243b54548a66fa072b4bfcf76c91ba2b11cf90da6274324c796df56fa` |
| its `CMakeLists.txt` | `0361f20b3847bb9295cc2aa3a8220f0b2ec1df1121f01465256d0f6ce9a823dc` |
| native `Link.cc` | `fb3636f9ec5c64d1c43c0b42cdab20c95702f4935bef90c6546068a749b23eb7` |
| native `Util.cc` | `6e35af3fa7c0a13d420a6cabae954177c16938d27d543d3e9f432fd980ade270` |
| retained ApplyLinkWrench `flags.make` | `89404c8325115e0de59f3bc06d0944db713754e5dcb2085fb4d1d28bc2876757` |
| retained ApplyLinkWrench `link.txt` | `1dc691f69e20bbf41d14d4e1f706c7c6d166c4845cd396abf58e2164aa947b7b` |

The retained native evidence binds the actual compiler18 path/version/hash,
Sim11/common8/math10/msgs13/transport16 and plugin providers, helper bytes and
capture source trees. Actual provider maps were recorded while the original DSO
was loaded; the audit also checked the retained 349-node SDK closure. Capture
identities remain separate from the future Rust-qualified source tree. Do not
treat an old image inventory as a current provider or use an implicit
license/source fallback.

The first native batch at `b460be63` exited 1 after both original plugin builds
and all nine odometry modes passed: the wrench helper included unavailable
`sdf/SDF.hh`, so no wrench mode ran. Its failure is preserved. The original
partial reader also exited 1 after saving its report at a final serialization
self-check; a separate retained finalization passed without repeating its body.
The isolated `c36db5ca` correction changed only that helper include to
`sdf/SDFImpl.hh`. The affected-only continuation exited 0, reused the original
DSOs, and captured 32 full six-component records across eight wrench modes.
It repeated neither plugin builds nor odometry modes. The independent wrench
retained audit exited 0; its receipt records report SHA-256
`e773a96852c9137bec51ec5ebf2eb2c0d36739958491a7d5589bb7127ab9b315`.

These records establish original-system command/component behavior with no
Physics system, body integration, native trajectory or Rust equivalence claim.
The native unpaused zero-dt consumption remains distinct from Rust's selected
advancing-only application policy. Callback arrival timestamps are not inferred
from paused zero rows; the source ordering and retained continuation effects
remain the stated evidence. All Rust and hosted acceptance gates below remain
pending.

Keep raw cases small and individually identified: one-shot applied once then
zero on the next step; queued one-shot retained through pause; two persistent
adds sum on repeated advancing steps; clear removes all persistent entries but
not a queued one-shot; clear during pause followed by empty continuation;
independent link targets; ID precedence and name lookup. Bound asynchronous
readiness with actual callback-effect/component witnesses, not a sleep or
HasConnections alone. Retain every settling observation; do not reinterpret
transport arrival order as an exact simulation timestamp. Record invalid native
messages/configuration as raw diagnostics where safe; native's permissive or
future-target behavior does not override explicit M35 refusals.

Rust regressions must separately establish actual Rapier one-step and repeated
linear/angular velocity changes for a centred free body with declared mass and
inertia, with no force replay after clear. Include sums/cancellation/order,
finite-input and intermediate-overflow refusal without partial mutation
(including existing direct-pending cancellation/overflow grouping witnesses),
capacity bounds, identity/type/name/backend/offset/COM refusals, direct pending
queue coexistence, pause/clear, removal/reset/shutdown/absence, and atomic three-key
reservation/reset rejection. A real C++ canonical peer must deliver
`EntityWrench` and `Entity` to the Rust Server and observe actual command effects;
its original-system rows remain a separate oracle. Keep the maintained native
peer registration, public CLI/profile guard and source-retention controls aligned.

Only after accepted-parent binding, complete leaf/shared Source review and the
fresh native proof may Root run one consolidated normal hook, every strict
workspace/per-crate/actual-parent line and region gate, exact hosted CI and
independent local/official DATA. No current outcome is assumed. No package,
browser, renderer, transport schema, backend or performance extension is included.

## Focused native helper protocol

The shared C++ source's default build is the real `apply_link_wrench` canonical
peer. It publishes one EntityWrench, two additive persistent EntityWrenches, and
one Entity clear message. Its bounded harness phase lines and stdin advancement
are test coordination, not a product wire acknowledgment. The Rust test advances
physics only after the actual Server semantic queue counters confirm receipt;
publication matching alone never supplies that evidence. The peer must build and
exit zero through the maintained selected SDK/profile/provenance path.

The explicit oracle build takes exactly `DSO CASE MAPS_PATH`, with an exact
packet-local DSO path, an exclusive bounded ordinary maps output, and one mode:
`once`, `paused-once`, `persistent-add`, `clear-paused`, `independent`, `zero-dt`,
`id-precedence`, or `name`. It invokes public Configure/PreUpdate interfaces of the
original loaded system on a real ECM with two link identities, with no Physics
system. Each raw row retains both full six-component world wrenches, actual
paused/dt arguments and identities; completion preserves every settling row.
The observed provider is checked against the requested DSO via dynamic type
information. A bounded actual `/proc/self/maps` snapshot is exclusively written
and fsynced while the original system remains alive, including a best-effort
failure capture without replacing an earlier/partial file. After each observation the helper performs the same command-component
clear boundary as the original Physics consumer, without integrating a body.

No fixed callback-arrival time is assumed: actual component effects end bounded
settling loops. Three finite paused calls are retained in pause cases, but do not
alone establish that the asynchronous callback completed before the first call.
The native paused-clear ordering is established by the pinned original source;
raw paused/continuation rows and strict Rust paused-clear counter-fenced cases
supply the separate behavior evidence. The `zero-dt` native mode intentionally
records the native/Rust policy difference; it must not be relabeled equivalent.
