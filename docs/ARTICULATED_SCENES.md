<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Local-link revolute graphs

This candidate expands Rapier scene loading to acyclic revolute graphs among a
top-level dynamic model's direct links and the world. Each child has at most one
parent joint. Disconnected free links and separate rooted trees are allowed.
The loader uses the already locked petgraph library for cycle detection.

Every articulated model receives its own backend-local contact-exclusion group.
Admitted links have self collision disabled; model-level self-collision settings
are still rejected by the producer. Contacts with other models remain enabled.
Top-level multi-link contact defaults also cover unarticulated models; see
[MODEL_CONTACT_DEFAULTS.md](MODEL_CONTACT_DEFAULTS.md). Nested-model contact
policy remains separately unqualified. This does not add general collision layers or query
filtering.

World and body joints resolve initial anchors/axes through the SDF graph.
Multi-link joint poses follow their physical child's construction frame,
independently of the model's canonical link. Existing captured endpoint/model
identity, velocity and sampling checks remain in force. Loaded reset and model
removal preserve their existing public identity and atomicity contracts.

DART additionally admits exactly two dynamic links joined by a world hinge and
one parent-to-child hinge. It constructs the world hinge first regardless of SDF
declaration order, then assigns public joint identities in declaration order.
Model removal orders physical leaves before parents using joint edges, rather
than numerical body IDs, so it does not temporarily enable contacts between
retiring endpoints. Loaded reset preserves public and backend joint identities.
The new analytical scene controls cover separated and overlapping internal
bodies, both link/joint declaration orders, coupled child effort and restoration
after model removal. They do not replay the different native moving-parent
fixture or qualify the original ApplyJointForce world, whose free root and
external ground contact remain unsupported.

The DART backend refuses actual external articulation contacts and active
multibody angular-limit rows. A later contact failure faults the simulation;
it is not a scene retry contract. Distant independent models are not excluded.
MuJoCo retains its previous scene topology restrictions. TPE has no dynamic-joint
support. Nested/static articulations, cross-model or frame aliases
as physical endpoints, loops, multiple incoming joints, other joint kinds,
limits and nondefault dynamics remain explicit errors. No runtime model-spawn
API is introduced: loading constructs a new Simulation and does not mutate an
existing one.

The candidate includes physical three-link motion and lifecycle controls,
noncanonical world-anchor behavior, mixed world/body joints, cycle and duplicate
parent rejection. Two exact formerly unsupported Rust XML inputs become positive
physical tests; unrelated rejection assertions remain intact. Their capability
disposition requires independent approval. No upstream conformance family or
performance improvement is claimed.

Native model-contact controls and source/build/runtime provenance are recorded
in planning/reference/model-contacts.json and require independent review. Full scene tests, independent review and normal strict
staged/hosted checks remain before acceptance.

## Native contact controls

The reference runs the baseline `gz-physics` dartsim plugin rebuilt from its
verified source/build directory, not an assumed identical installed copy.
Six controls distinguish default same-model exclusion, distinct-model contact,
link-only flags, a model self-collision flag, and nonadjacent links in a native
three-link revolute chain with self contact disabled/enabled. Contact counts
must distinguish these stationary constrained cases; identical positions alone
do not prove contact exclusion. Sorted actual contact-owner names identify
`l0`/`l2` directly as the nonadjacent positive control. Loaded plugin/DART/ODE/SDF/math hashes and compiler,
package, source and build identities are recorded. These observations establish
contact presence, not cross-engine numerical accuracy or complete model policy.
The two positive native flags are not new supported Rust scene inputs.

```sh
python3 scripts/probe-model-contacts.py \
  --physics-source /path/to/gz-physics \
  --physics-build /path/to/gz-physics/build \
  --dependency-prefix /path/to/native-prefix \
  --work-dir /tmp/gz-model-contacts-qualified \
  --output /tmp/model-contacts.json
```

The public 20,000-edge loader regression reaches the later geometry guard on the
default test stack. Independent review confirmed that restoring the previous
recursive cycle-check API makes this exact test abort; the iterative topological
check passes without changing stack size or admission limits.
