<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# PHY-03 two-body static contacts candidate

The candidate starts at `004baa8`, now accepted after its full local and exact
hosted qualification. Owned paths are this crate's source, includes, tests and
docs. The coordinator owns Scene, server, shared gates and integration. This is
not acceptance of the original ApplyJointForce system test.

The proposed extension admits existing free-root/revolute pairs against static
geometry. World-root contacts, dynamic external articulation contacts,
self collision, larger graphs and active multibody joint-limit rows remain
refused. Existing isolated-body contact paths and numerical settings stay intact.

At world contact point p, expand the native linear point Jacobian J into all
skeleton coordinates, including coordinates absent from a body's dependent
Jacobian. Read native M and N=M^-1. With D=sqrt(diag M), C=D^-1 M D^-1 and
K=D N D, require finite positive definite matrices. DART explicitly mirrors the
triangular halves of M and N; asymmetric returned matrices are refused. The
normalized mass condition ceiling of 1e8 is a new articulated-contact restriction,
not a consequence of the individual-body principal-inertia anisotropy bound.
It is checked before requesting the native inverse.

Let e=||I-CK||F. Require e<1, then bound the point response by
tr(J N J^T)/(1-e), evaluated as ||J D^-1 chol(K)||F^2/(1-e). Since CK is similar
to SPD C^(1/2)KC^(1/2), its real eigenvalues lie within e of one. Therefore
C^-1 <= K/(1-e), which bounds both the native inverse and ideal inverse-mass
quadratic responses. Every unit-direction diagonal is at most the trace;
off-diagonals follow by Cauchy-Schwarz. The summed contact ceiling remains 4e5.
This real-matrix argument is not a certified floating-point error bound and does
not prove native impulse-path symmetry at all inputs. Native unit-response and
actual ODE subprocess controls are required; admission must be narrowed if they
find failures. Solver, timestep, physical fixture and tolerances are unchanged.

Native-first diagnostics compare the full response against DART's actual
updateBiasImpulse/updateVelocityChange path at both bodies' corner points.
The initial 126 child processes cover both root types, geometry scales 1e-3 to
1e3 and root mass ratios 1e-20 to 1e20: 98 pass the matrix checks and 28 are
refused before inverse evaluation. No process failed, including native block
symmetry checks for points under the contact ceiling. These diagnostic samples
are not exhaustive safety coverage. These raw diagnostics include inputs outside the existing per-body admission
domain and precede the narrower candidate controls below.

The exact original world uses SDF default gravity -9.8 m/s². Its unchanged SDF
is retained beside `ground_contact.cc`; the shared provenance driver regenerates
34 observations for ±0.001 N m after ten zero-effort steps through step 800.
The adapter matches every body pose/twist and joint position/rate within the
existing 1e-12 backend tolerance. No damping or fixture changes are involved.

A native replay of the earlier world-root attachment rollback input aborts in
RevoluteJoint::updateRelativeTransform by step 20. A world-constrained pair can
have a rank-deficient point response even with SPD generalized mass matrices.
The initial trace-only candidate was therefore narrowed to free-root pairs;
all world-root contact refusals remain unchanged. The failing native program and
per-scenario output are retained in ignored artifacts/contact-design/native-static.

`static_contacts.cc` replays the exact free-root attachment and approaching-contact
inputs previously refused by free_parent.rs, including initial twists, the queued
root wrench and child effort. Its 34 observations through 100 steps are separate
from the original-world reference. Independent prior review approved exactly two
free_parent.rs transitions: attachment with the static sphere, and the approaching
static-contact branch. Both now check the same 34 native observations; the
world-root branch and original active-limit branch retain their rejection,
rollback and future-motion assertions. Separate prepared controls retain reset,
detachment and retirement coverage.
Additional dynamic-contact and normalized-matrix refusals check public state,
repeated failure, queued effort and future motion against clean controls.

Both fixture probes reuse `probe-moving-parent.py` unchanged. Their provenance
is the documented hybrid SDK with verified OSRF native runtime, not a homogeneous
SDK build or performance result. Reproduction commands:

```sh
python3 crates/gz-physics-dart/tests/fixtures/probe-ground-contact.py \
  --physics-source /home/nkoenig/work/gazebo/rust/gz-physics \
  --build /home/nkoenig/work/gazebo/rust/gz-physics/build \
  --prefix /home/nkoenig/local --work-dir artifacts/contact-design/native-ground \
  --report artifacts/contact-design/native-ground/report.json
python3 crates/gz-physics-dart/tests/fixtures/probe-static-contacts.py \
  --physics-source /home/nkoenig/work/gazebo/rust/gz-physics \
  --build /home/nkoenig/work/gazebo/rust/gz-physics/build \
  --prefix /home/nkoenig/local --work-dir artifacts/contact-design/native-static \
  --report artifacts/contact-design/native-static/report.json
```

The final admissible native point probe checks 672 rotated/offset corner responses
against actual native impulse propagation, including both bodies. It retains
1e-10 relative comparison and native 1e-6 absolute symmetry checks; 336 raw points
below the unchanged mass floor are reported separately, not as helper refusals.
A below-floor root mass of 1.2e-8 kg exceeded the response comparison in a raw
native diagnostic; it remains outside the adapter's input domain.

The expanded 768 actual adapter subprocess cases cover four collision shapes,
rotated/offset and anisotropic inertia, extreme mass ratios, both root types and
an oversized final timestep. Independent review found that the initial anisotropic
tensor violated the principal-moment triangle inequality. The repaired corpus
uses valid [small, large, large] tensors. It reports 96 spawn refusals, 408
attachment refusals and 264 attached cases advancing at least once; 138 reach
39 steps. No process aborts. These outcomes are not 768 successful contact solves.

The native fixture retains the independently verified condition-boundary pair:
coincident unit-mass bodies, root isotropic inertia 1e-4, child inertia multiplied
by 2.49e7 or 2.51e7, and a Z hinge. Normalized condition numbers near 9.96e7 and
1.004e8 respectively produce finite and refused point responses.

Independent review approved the bounded production source, repaired tests and
two exact transitions. Fresh final replay and the real C++ server command case
pass. Ordinary staged, coverage and exact hosted gates remain required. No
production source changes were requested or made during the test repair.

After the repairs, the unfiltered DART package passes all 139 checks, including
the native harness and both former rejection controls. Clippy with all targets
and warnings denied also passes. The production source hashes remain unchanged
from independent review; these focused results do not replace ordinary gates.
