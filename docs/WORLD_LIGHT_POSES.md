<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# World Light pose publication

This candidate extends the accepted [world Light metadata](WORLD_LIGHTS.md)
with the nongeometric child created by native Sim and complete Server pose
groups. Full staged and hosted qualification remain pending.

Each loaded world Light owns one `EntityKind::LightVisual` child named
`<light-name>Visual`. It has an identity local pose, no geometry, false shadow
casting and zero transparency. Its existing attached-pose marker follows the
parent through the existing Scene schedule. Loaded reset restores its reserved
public identifier, parent, names, properties and attached-pose behavior. Removing
the Light removes its child. No new system, callback, resource, worker or endpoint
is installed for this metadata; a world without lights constructs no such entities.

Server publication includes Visual-family records before Light-family records.
Child wire poses are local identity poses; Light wire poses are resolved relative
to their world parent. Actual owned public IDs identify both. Native numeric IDs
and reverse traversal observed in one reference run are not portable ordering
contracts. Rust retains its existing deterministic ID order within each family.

Construction and subsequent packet generation validate the complete hierarchy.
Partial Light metadata, orphan or duplicate children, wrong parent or name,
nonidentity local pose, altered required shadow/transparency properties and
missing ID ownership fail
before a packet is admitted. Scene renderer extraction still returns
`UnsupportedLighting`, including for an orphan LightVisual identity. This does
not connect SDF intensity to the separate explicit-lux renderer API, implement
light commands or admit model/link lights, GUI geometry or B03 articulation.

## Evidence and retained intent

The [checked-in native program and recipe](NATIVE_LIGHT_POSES.md) reproduce
the twelve-Light grid and lightless control against the qualified C++ Zenoh
reference. Native creates a child even for an off or hidden Light. A separate
reference run used the two exact old Server XML literals: off sun and default
sun both produce two real pose records with identity local poses. These native
observations do not establish Rust constructor error ordering or native reset
parity.

Prior independent review approved the two packet/setup assertion transitions below. The
complete off-sun packet now checks the child and Light instead of expecting a
lighting refusal; its later metadata/kind failures remain intact. The valid
cold sun now reaches the existing invalid-world-name error before capacity
validation. A separate malformed-metadata control still fails before WorldIO
setup. No timeout, numeric tolerance or remaining negative assertion changed.

The first full hook failed the retained exact Sdf8 identity list: native child
construction adds `default::lamp::lampVisual` to its five original names. Prior
independent review approved adding only that name and stronger child ownership,
hierarchy, identity local pose, shadow/transparency and no-geometry controls
inside the same four-backend reset cycles. The exact XML, DTO/source/defaults,
fixed-body refusal, no-motion and all original identity/reset checks remain.
The failed hook is retained. Unfiltered Scene tests and selected Clippy pass
after this correction. The next hook stopped at Scene changed-line coverage,
16/17: LLVM attributed the child allocation error propagation to a standalone
closing-call line. Binding the child kind and identity local pose before the same
fallible call keeps its ordinary execution on that statement. The typed error
is unchanged. The following partial profile confirmed all 39 prior cold Scene
regions remain, including the moved allocation-error region at count zero.
Allocator exhaustion is not claimed as tested.
The third hook stopped at Server changed-line coverage, 49/52. A meaningful
additional regression inserts actual Light metadata on an otherwise valid
lightless World entity: packet admission rejects that mismatch, and removing
the metadata restores the valid empty packet. All eight packet tests pass.
Defensive missing-parent paths remain cold; no dangling ECS relationship is
fabricated. Complete staged and hosted qualification remains pending.

Focused Scene checks pass across TPE, Rapier, DART and MuJoCo, including changing
the restored parent after reset. Eight packet tests cover complete publication,
sixteen malformed-hierarchy cases, misplaced metadata and ID ownership corruption; four renderer
checks preserve explicit refusal. The C++ client test makes seven real peer
runs: twelve Lights on all four backends, then lightless, off-sun and default-sun
controls on TPE. Each receives initial, one-millisecond and loaded-reset poses
with clocks, checks stable opaque IDs and field presence, and saves actual
received protobuf messages. Rust independently checks those IDs against Scene
ownership. The final combined focused replay passes the seven peer runs, ten Scene tests,
eight packet tests and four renderer refusal tests. Selected Clippy passes with
warnings denied. Complete staged and hosted qualification remains required.

This is supplemental lifecycle and interoperability evidence. It does not
complete an upstream original system family, zero-cost performance measurements,
balanced robotics benchmarks, renderer radiometry or the broader rewrite.
