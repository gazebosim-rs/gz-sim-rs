<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Bounded DART serial-chain Scene construction

This candidate consumes the accepted contact-free three-body backend contract.
It permits a dynamic top-level model with three local links and two serial
revolute joints, with one free root. It preserves existing axis, dynamics,
limits, geometry and backend restrictions. Forks, repeated children, cycles,
world-rooted three-link chains, nested/static models and larger models remain
unsupported. This does not add backend contact admission or multiple collisions.

Validation requires three graph nodes, unique children, at most one outgoing
edge per node and an acyclic graph. Construction orders the two constraints
parent first, including when declarations are reversed. Public joint entities
and IDs still follow declaration order. Existing world-rooted two-link handling
uses the same root selection and remains supported. No runtime schedule or
optional-system owner is added.

The composed controls compare all thirteen body-state fields, WorldPose and
joint samples against the accepted direct backend for both declarations,
loaded reset and removal. The new native-backed control binds the exact XML to
[the pinned reference](NATIVE_FREE_THREE_BODY_SCENE.md), then compares both
orders over ten one-millisecond steps and the original .7/-.2 joint efforts.
It compares all body/joint values at the existing 1e-12 bound and compares joint
frames against actual native SemanticPose resolution. Generic native ECM joint
traversal is not used as a semantic-frame oracle. Native observed IDs are opaque.

Prior independent review approved only the retained B03 test's name and expected
Unsupported entity/feature transition. Its exact fixture, URI, engine call and
complete sun DTO assertion stay intact. The newly admitted serial topology
reaches the base link's existing two-collision rejection. The whole B03 world
remains unsupported; this is not an acceptance or benchmark result for B03.

Source review found and corrected incomplete negative fixtures and native-data
dimension/name checks. Those changes are batched with implementation, test code
and documentation before consolidated verification. The earlier two-test
prototype result is a precursor; the final native-backed controls, retained
suite, Clippy, staged coverage and hosted qualification remain unrun. No final
Rust/native parity, complete-system migration or performance claim follows.
