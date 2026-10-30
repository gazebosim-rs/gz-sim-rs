<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ApplyJointForce configuration

The scene loader recognizes the model plugin pair
`gz-sim-apply-joint-force-system` / `gz::sim::systems::ApplyJointForce` with one
`joint_name` naming a direct local joint. It installs immutable
`JointForceConfiguration` metadata and a zero `JointForceCommand`. The existing
Bevy force system applies programmatic latest-value commands. No native plugin
framework or second ECS is introduced.

Configured joints must already support effort on the selected backend; loading
checks the existing pure capability contract at zero torque. Each joint accepts
one controller configuration. Multiple distinct joints remain independent.
Missing/scoped targets, duplicate controllers, other plugin pairs/placements,
unknown attributes/children/namespaces and mixed lexical text are explicit Rust
boundaries. Native Configure tolerates extra payloads and lazily searches for a
missing joint; those behaviors are not claimed by this slice.

Ordinary text follows native ASCII whitespace collapse. A single CDATA value
trims only outer ASCII whitespace and preserves internal spaces. Unicode blanks
are not silently removed. Maintained roxmltree supplies XML structure and decoded
text; quick-xml preserves lexical boundaries which roxmltree coalesces. Both
packages were already locked dependencies; only direct Scene edges are added.
Native TinyXML GetText selects the first text node of split CDATA/comment forms.
Rust rejects those forms instead of concatenating them into a different target.
No general XML parser or new topic converter is implemented.

Native SystemManager reloads configured plugins lacking Reset. The Rust initial
blueprint therefore stores immutable configuration and reconstructs a zero
command; it does not snapshot the latest runtime force. Core reset clears force
while retaining metadata. Model retirement removes both; loaded-world reset can
restore the original configuration and zero command after retirement.

Seven public controls exercise all three dynamic backends, initial zero samples,
programmatic actuation, core/loaded reset, retirement, independent joints and
strict payload/target boundaries. Private controls compare fourteen actual native
clone/Get text observations, including whitespace/entities/CDATA/Unicode and the
explicit split rejection, and check inconsistent raw metadata. Existing scene
and joint-force assertions remain unchanged. No original-system credit is added.

Rebuild the text fixture from the pinned native source trees and installed SDK:

```sh
python3 scripts/probe-joint-force-text.py \
  --sdformat-source /path/to/sdformat --sim-source /path/to/gz-sim \
  --prefix /path/to/sdk --work-dir /tmp/gz-joint-force-text \
  --report artifacts/joint-force-text-reference.json
```

The helper verifies selected source files against the baseline, compiles the C++
client and records actual loaded sdformat/TinyXML hashes. This qualifies installed
SDK observations, not a fresh SDK build or the complete simulator system. The
normal tooling gate checks source/script/baseline bindings; independent review
and staged/hosted qualification remain required before acceptance.

Bounded transport subscriptions and topic normalization are described in
[JOINT_FORCE_TRANSPORT.md](JOINT_FORCE_TRANSPORT.md). The genuine upstream fixture
also needs plane geometry, world-system selection and moving-parent actuation
which remain unqualified.
No performance improvement or complete ApplyJointForce migration is claimed.
