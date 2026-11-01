<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Owned world-light construction metadata

The following record describes the metadata slice accepted at `db9d002`.
The [current pose-publication candidate](WORLD_LIGHT_POSES.md) separately adds
the native derived visual hierarchy and Server records; its complete qualification
remains pending. The original refusal evidence below is historical.

This private slice admits direct world lights into the headless Scene loader.
It does not implement lighting, light commands, the native GUI visual scaffold,
full Server interoperability or B03 articulation. The unchanged B03 fixture still
fails the existing DART three-link/two-joint admission check after its sun is
accepted. No physics topology rule, fixture, dependency or producer pin changes.

At native Sim `71c60db`, `SdfEntityCreator.cc:678` copies the complete `sdf::Light`,
resolves its semantic pose, and assigns its name and type. The world loader parents
it to the world. It also creates a GUI-only LightVisual child, which this slice
does not fabricate. Native `SceneBroadcaster.cc:599–628` publishes both visual
and light poses. Those additional wire records are not qualified by retaining
light metadata.

Scene's `WorldLight(Box<sdformat::Light>)` owns every field of the accepted
producer's typed light, including its raw pose, URI/line/column source record,
colors, on/visualize/shadow flags, attenuation, spot parameters, direction and
intensity. This retains the existing producer Source contract, not a mutable C++
Element or a new original-XML API. `EntityKind::Light`, the existing public ID,
scoped/local names, world parent and resolved local/world poses identify the
entity. The existing frame resolver supplies the initial pose; the raw reference
remains metadata. No new light callback, schedule, worker, endpoint or renderer
dependency is installed. Editing metadata does not issue native light commands
or change physics. The immutable initial-scene blueprint owns a separate clone
and restores metadata, poses, names and the same reserved ID after mutation,
component removal or entity removal. An originally absent component stays absent.

Rendering explicitly returns `SnapshotError::UnsupportedLighting` for either
Light identity or WorldLight data, including disabled lights or missing metadata.
The check is inside the existing extraction entity loop. Both ordinary and
camera extraction refuse the request before producing a
snapshot. Server Pose construction similarly returns `Error::PoseLighting`
inside its existing packet loop, before successful encoding/publication admission.
It neither adds an absent-family tick scan nor silently drops native light/GUI
visual records. This advances Core loading, not full Server/B03 parity. Link
lights and every unrelated producer guard remain unsupported as before.

## Test and transition scope

The native B03 file is copied byte-for-byte from the pin, SHA-256
`24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa`.
Its directional sun subtree is also copied unchanged into the supplemental
three-type world fixture. The complete B03 sun DTO projection is exact; frame
composition cases use the existing loader's f64 pose tolerance. All other
metadata and reset comparisons remain exact.

The initial four-backend run failed at `world lights` on TPE, Rapier, DART and
MuJoCo. After implementation, seven supplemental Scene tests and the original exact
DTO validation positive pass on the warm CPU target. The three new renderer
refusal controls and thirteen retained renderer controls pass. Both new Server
Pose/cold-construction refusal controls and three retained Pose controls pass.
The Scene tests cover complete typed metadata, world ownership, resolved frames,
include/source identity, backend-independent retention, reset/removal/absence,
unchanged schedules and physics, link-light refusal, bad references and the
unchanged B03 DART refusal. These are supplemental tests, with no upstream
original completion or full absence/performance credit. Scene/Render and Server library/all-test Clippy pass with warnings denied.
The first selected retained Scene run passed its unit and earlier selected targets,
then stopped at a second preexisting world-light rejection described below. After
its separately approved fold, the final selected Scene run passes 111 tests and
the renderer extraction run passes all 16 tests; Scene/Render Clippy passes again.
Normal complete staged/hosted gates remain pending.

The existing private guard test retains its entire six-case input table. Its first case parses
its original light-free XML with URI `guard`, then pushes `Light::default()` into
the DTO. The proposed stronger positive repeats exactly that input/URI/mutation,
checks the entire World Debug projection and complete default Light, and verifies
that validation does not mutate anything. It does not pretend the XML contains
that light or expose a new DTO-to-runtime loader. The separate XML tests prove
actual construction and lifecycle behavior. Before the authorized first fold, the unchanged old rejection test failed
solely because `unwrap_err()` received successful validation.
Independent PRIOR review approved only that row, which now calls the same
complete helper; the duplicate temporary prepared test was removed after the
fold. The five other world predicates, link/model guards and all other old
assertions stay intact. The notice-only fixture change has now been replayed; the first folded
private guard passes. Its earlier failure remains separate.

Evidence and source-bound ELF copies are under `artifacts/world-light-study`.
Independent source review approved both exact assertion transitions and the
extraction-loop change. This remains a private candidate pending the complete
staged and hosted gates.

A second retained predicate in `sdf8_admission.rs` rejects the exact point-light
XML generated by its original `world` helper, with URI `sdf8`. Before its authorized fold, that actual test
failed because loading succeeded. Independent PRIOR review approved replacing only that row with the exact
positive; its table, XML generator, URI and every other call remain unchanged. `tests/support/world_light.rs` prepares an exhaustive
positive for those exact bytes/URI: complete World/Light/default/source data,
static mass/inertia and sphere, world-only component ownership, stable IDs,
fixed-body input refusal and unchanged pose/velocity, metadata edit/removal and
loaded-reset restoration on all four backends. The temporary prepared test passed on all four backends and was removed only
after the approved row fold. No other old table row or fixture changed. The
current fold and extraction-loop refactor pass their final focused replay,
including all six retained Sdf8 tests. No complete-profile coverage claim follows.

Server's existing constructor already validates its first Pose packet before
name/configuration and WorldIO creation, so the new typed guard also rejects
unsupported lights before opening endpoints. The packet-loop check remains a
defense against runtime metadata/identity insertion; no new periodic scan was
added.

## Simulator integration

The integration is based on camera candidate `e0b1f6b`, whose complete normal
hook and exact hosted run `41307067791` passed with independent downloaded
source/profile/native auditing before Main publication. Initial-scene capture
and restoration retain collision density, world Sensors intent and world-light
metadata together. A supplemental Core regression removes world Sensors intent,
deletes one light and mutates the remaining lights, then checks reset restores
the complete typed light values, reserved IDs, world parent and Sensors intent.
It does not activate a camera or claim lighting support. This regression passed on the integrated source. The complete integrated normal
and exact hosted gates remain pending.

The first combined normal hook stopped at changed-line coverage, 16/17. Light
construction had executed, but LLVM attributed the cold identity-error
propagation to its standalone closing call line. A small naming refactor binds
the world parent and scoped name before the same fallible call. It retains the
typed error and its cold region in owned coverage; it does not claim execution
of fresh-allocator exhaustion. Focused normal and instrumented checks pass 114
controls. All 38 previously cold regions in the compared `lib.rs` profile
remain present and cold. No test, error path, exclusion or coverage gate was removed.
Complete integrated qualification remains required.
