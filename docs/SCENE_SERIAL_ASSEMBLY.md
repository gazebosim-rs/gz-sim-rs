<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Atomic construction of a loaded DART serial chain

This candidate failed its mandatory Rust qualification. Its complete source and
native references still require runtime acceptance. It is built
on accepted offset-COM Main `e55356c8`, whose mandatory local gate, exact
hosted run `41383974766` and independent official audit passed. The earlier
qualifying COM tree is unchanged.

Scene routes only the already-validated DART model with three local dynamic
links and two serial revolutes through atomic construction. It retains the
same world poses, axes, body handles and parent/child names for public metadata.
When both joint orientations relative to their children are exactly identity
(either quaternion sign), it submits child-local anchors and axes through
`add_serial_revolutes_in_child_frames`. The backend forms each joint frame from
the live child transform, following native construction order. Other joint
orientations retain `add_serial_revolutes` and its world-frame contract.
Scene orders inputs from root to leaf and submits both edges together. Public
joint entities are still created in SDF declaration order after both constraints
are admitted. World-rooted chains, single joints and other backends keep their
existing construction paths. No runtime polling or system plugin is introduced.

The backend must prepare and validate a complete candidate before replacing
live state. Final external-contact, velocity, grid and conditioning guards stay
in force. A transient contact with an intended child during sequential assembly
is a plausible explanation for the existing B03 refusal; no observed branch
diagnostic establishes that cause. Atomic construction is a separately tested
capability, not permission to omit final contact validation.

New Scene controls use touching intended links, a nonidentity model pose,
explicit offset COM, both declaration orders, joint efforts, loaded reset and
model retirement. The same controls also cover nonidentity joint orientations
through the retained world-frame path. A separately constructed external dynamic body must still
cause a typed joint-admission refusal. Existing contact, topology, backend and numerical negative controls remain
unchanged. Independent whole-source review approved the exact same-input B03
construction/reset positive, which now replaces only its earlier joint-refusal
expectation. The full qualification stopped at the B03 trajectory comparison;
it did not establish acceptance of the complete candidate.

## Actual native B03 reference

The exact unchanged `b03_double_pendulum.sdf` has SHA256
`24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa`.
One reviewed C++ observer ran the actual pinned Gazebo Server for 100 unpaused
one-millisecond steps after one paused preparation. It queried all seven bodies
and four joints on every step and retained steps 1, 10 and 100. Independent
read-only audit verified all 486 snapshot numbers, 31 scoped identities and
parents, actual clock stamps, 842 compiler inputs and 157 mapped libraries.
No compiler or semantic replay was used by the reviewer.

`reference/b03_world.cc` is the exact observer;
`reference/b03_world.json` is its exact actual provenance. The strict
`scripts/materialize-b03-world.py` only repackages actual captured state into
`tests/fixtures/b03_world_native.json` in the Scene crate. Provenance and fixture
hashes are respectively
`a54d9cdc2beb5289a25e096859c51601598e3deb4ff04c9996007164e34db01a` and
`d27be211cf7e267864507cda799ac51f697e0960caa96ddd4dcdb7f6fe0afbff`.

The native body vector distinguishes link-origin velocity from COM velocity
and includes origin pose, angular velocity, COM world position and local COM.
The observed default gravity is `(0, 0, -9.8)`. The declared Rust URI `b03.sdf`
is distinct from native `/probe-source/input.sdf` and its actual world line 3;
opaque IDs are mapped by names, never equated across implementations. Only one
runtime map was observed, after stepping while Server remained alive. The body
queries read Gazebo's ECS components. Subsequent source inspection and actual
per-tick observations proved that these components can contain older link
states; they are not an independently fresh physics snapshot on every tick.

New Rust Scene comparisons require all native fields at the unchanged 1e-12
bound. The Server integration separately compares 100 complete Server updates
with an independent raw Scene, including all body origin states, joint samples,
clocks, stable identities and terminal shutdown. The mandatory run failed at
step 100: base x was `9.47354856103904085e-12` in Rust and
`2.98575162182136334e-11` in the native ECS record. A retained diagnostic
reproduced that failure and exposed larger differences later in the same
snapshot. The native capture used `-O0`, assertions and concurrent unrelated
qualification work. Its elapsed window is not performance evidence. Rust B03
construction, trajectory parity and matched performance remain unqualified.

## Investigation of the failed B03 comparison

A separate diagnostic retained all 101 native and Rust snapshots and contact
observations. Each reproduced its own original steps 1, 10 and 100 exactly.
Independent source and retained-data review confirmed that native base rows at
steps 2 through 9 repeat step 1. Gazebo's changed-pose selection also gates its
link velocity component updates. Rust's direct backend observations are fresh,
so the early comparison mixes observation ages.

Joint samples update every tick. Their later differences, first exceeding the
unchanged bound at step 39 for velocity and step 51 for position, remain
unexplained. Stale link components therefore do not explain the entire failed
trajectory. Contact counts first differ at step 90; the observations do not
establish a solver ordering cause. The historical captures and all numerical
bounds remain intact.

A subsequent source-reviewed diagnostic obtained fresh native origin and
declared-COM point states through Gazebo's existing generic child-frame queries.
It added fourteen observation entities with no physical kind or solver changes.
Independent audit verified 856 compiler inputs, 157 mapped libraries, 1,414 point
rows and 707 complete body rows. All 101 cached body and joint snapshots
reproduced the preceding capture exactly when keyed by identity. Named-pair
contact point arrays also matched; some readout and contact-message ordinals
changed, which does not establish physical or solver ordering equality.

The existing Rust observations were reused without another Rust run. Against
fresh native states, the first difference above `1e-12` occurs at step 37 in an
upper link's origin x velocity, with difference
`1.890825007417468e-12`. The later maximum remains
`0.022837514146162315`. Thus fresh observation removes the early stale-component
comparison error but does not resolve the trajectory divergence. The historical
cached reference, fresh observations and failed qualification remain separate;
no reference replacement or production correction is accepted from this result.

The proposed regression disposition retains `b03_world_native.json` and its
original generator/provenance unchanged. A separately named
`b03_world_fresh_native.json` supplies the same 22 body and two joint fields at
steps 1, 10 and 100. The XML, URI, identities, 100 one-millisecond steps and
`1e-12` bounds remain unchanged. Joint rows still come from the native per-tick
joint observations. This comparison must continue to fail while the later
physical divergence remains unresolved.

`scripts/materialize-b03-fresh-world.py` reuses the unchanged, independently
reviewed diagnostic inspector. Before extracting fresh snapshots it checks the
actual generator, validator, provenance, original fixture, all raw capture
hashes and previous 101-frame capture, then repeats the retained-data fidelity
checks without running either simulator. It writes a new file exclusively and
does not overwrite the historical fixture. The copied observer, validator and
actual provenance are retained as `reference/b03_world_fresh.cc`,
`scripts/inspect-b03-fresh-world.py` and `reference/b03_world_fresh.json`.
Independent source review approved the transition before one retained-data
materialization. A separate audit verified all 486 selected values and their
source, provenance, historical-reference and raw-capture bindings. No simulator
or compiler ran for materialization. Runtime verification remains pending;
this reference correction does not grant trajectory acceptance.

## Selected-plugin wrench and contact reference

A separate reviewed observer ran once against the same qualified plugin and
captured two final three-body trees: free and static-plane contact, with a
one-shot COM wrench and local center `(0, 0, 0.1)`. All eight snapshots at counted
steps 0, 1, 10 and 100 passed the capture's finite/unit/domain predicates and
actual floor-response witness. The record binds 672 compiler inputs and actual
before/after runtime maps. COM position and velocity are conversions from the
observed link FrameData and the explicitly declared local center, rather than
independent queries of the loaded inertial center. Its timestamps are requested
duration times counted ForwardStep calls, not a queried simulator clock.

The exact actual provenance and fixture SHA256 values are
`04a96f4cf5977e8c0f8743d5d81027951cb04336d266fe214db988e29bda8c49` and
`f8030cc5729b1f2771b8f38b17a490f8d6c2316ce1c530eb350bbe26c5313b25`.
Independent raw/source/library audit passed for all 560 numbers, exact source,
672 compiler inputs and all 37 before/after mapped libraries. No native process
was replayed for materialization or review. These 560 numeric values supply the direct-backend
comparison and prepared-reset replay; they do not qualify Rust assembly,
exception atomicity, B03 motion, the earlier refusal's cause or performance.
