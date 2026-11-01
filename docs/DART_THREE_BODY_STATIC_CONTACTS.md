<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Prepared serial three-body static contacts

This private candidate extends the existing direct DART6.13.2 free-root serial
three-body topology to bounded external static contacts. The production change
only raises contactsWithin's defensive maximum from two to three bodies and
updates its comments. Dynamic external contacts, self-contact, active multibody
limits, other topology,4-body attachment, inverse-response/condition/velocity
limits and all numeric constants remain unchanged. There is no Scene, B03,
multiple-collision, Gazebo plugin or general-contact acceptance claim.

The existing all-coordinate response helper already expands dependent Jacobians
and checks the normalized SPD mass/inverse residual,1e8 condition ceiling and
4e5 contact ceiling. No new residual math or alternate solver is introduced.
The native references use the existing selected DART/ODE algorithms and packaged
library identities, not an approximate physics model.

## Exact retained inputs

The two old free_chain.rs refusal tests first remained byte-identical while their
prepared counterparts ran. Their reviewed folds now use those same positive
helpers, keeping the existing isolated-child wrappers and deadlines. Both start
with mass1/radius0.1 spheres and diagonal
inertia(0.004,0.004,0.004), zero COM/gravity, positions(0,0,0),(0,0,-2),(10,0,0),
firstY hinge through(0,0,-1), secondY hinge through world0. Their native controls
use exactly these values, not the earlier120-mass box study.

The attachment positive keeps the old static box half(.25,.25,.1), center
(10,0,-.15), warm zero middle impulse and deferred rootF(.2,0,0)/T(0,.1,0),
first effort.7. Its new successful attachment must preserve all immediate states,
first joint and contiguous identity. The original obstacle removal and three
future1ms clean comparisons remain; a separate100ms native-backed contact run
checks real stepping with the obstacle present.

The approaching positive keeps the old static box half(20,20,.1), center
(0,0,-3.1), root velocity(0,0,-3),1000 one-millisecond rounds and per-step root
force(.01,0,0)/second effort.001. Selected full13-field body and joint rows are
compared at the retained component-wise1e-12 tolerance. No shorter horizon or
quaternion sign equivalence is substituted.

Both old error/atomicity/repeated-error/pending-recovery contracts are separately
prepared as dynamic-obstacle negatives, with the explicit input change mass
None→Some(1). They are not mislabeled as the old static input. Duplicate-joint
errors are not used to stand in for provisional native topology rollback.
All unrelated retained tests remain untouched.

The new above-condition static negative has a directly characterized counterpart
using the bridge's actual immobile FreeJoint/default box ground. It starts just
above contact at root z.0051, down velocity.1, zero gravity, root mass120,
child masses.6, isotropic moment1e-4 with leaf multiplier74282549781.30127,
Y/X joints, deferred rootF(1e-5,0,0) and second effort1e-5. Native normalized
condition remains100000172.094–100000177.982; actual contact response is infinity
at the unchanged ceiling. This is explicitly a new approach variant, not the
initial overlapping WeldJoint-ground input. Rust must refuse atomically and
retain pending motion through obstacle removal.

A separate actual spawn rejection uses the recorded invalid root from class0
of the126-class diagnostic with an already-live root and two children. It checks
nonempty body/joint snapshots and pending-command future motion. The earlier
126-run is frozen: its42 root-first refusals only compared empty state prefixes
and are not credited for this guarantee.

## Native rows and numeric controls

The checked-in reference source and33 selected rows cover both literal paths,
actual sphere and four-corner box contact, and the above-condition variant.
The fixture records source/compiler/system-header/18-DSO identities. Run
`python3 scripts/reference-three-body-static-contacts.py --output <directory>`
with the selected qualified packages to recompile and replay all seven controls.
The normal Rust test suite invokes that replay too. Raw observations use exact
recorded JSON numeric equality; this does not prove signed-zero/raw-f64 identity.
Physics parity keeps the existing1e-12 component tolerance,1e-10 response and
1e-6 native symmetry checks.

The candidate numeric test runs96 bounded30s children: four existing shapes,
three radii, four retained mass profiles, and two COM/tilt classes. Accepted
setups begin in real static contact; typed setup refusals preserve existing
state. Attachment refusals preserve body/joint snapshots. Actual step refusals
must be recognized contact/active-limit outcomes with repeated atomic refusal.
Contact refusal must recover after obstacle removal. An active-limit refusal
must persist atomically after removal, then recover after explicit public
zero-velocity edits from root to middle to leaf. Successful recovery alone does
not prove quantitative queued-input retention; separate controls cover that.
Any abort, timeout, nonfinite state or other error fails the control. The new
matrix runs 40 one-millisecond steps; existing numeric matrices, horizons and
limits remain unchanged in their own tests. These controls do not declare
every input admitted or prove general robustness.

## Current state

Native source/probe replay passed in the immutable cb921 image: the new
FreeJoint-ground condition variant plus the six existing literal/footprint modes
all exited 0. The source-bound host replay also passed with the exact pinned
GCC 13 compiler and native dependency closure. Eight focused Rust controls passed,
including both prepared static positives and the genuine dynamic/conditioning
negatives. The corrected numeric matrix passed all 96 children: 39 advanced,
32 refused attachment, 16 refused spawning and nine preserved active-limit
refusal until explicit velocity correction. No child aborted or timed out.

Earlier outcomes remain separate: a stale linked two-body guard, an incorrect
new final-middle-contact assertion, compiler-selection and child-output parsing
mistakes, the false assumption that floor removal repairs an independent active
limit, and a missing trait import after relocating the positive helpers. Their
evidence is retained rather than relabeled as passing.

Independent source review approved exactly the two old folds and helper
relocation. After the required trait import was restored, the unfiltered DART
package passed all 159 tests without failures or ignored tests, and all-target
Clippy passed with warnings denied. All 24 physics-linked test executables were
checked for the three-body comparison before execution. Independent final review
and full strict workspace/local/hosted qualification remain pending; these
package results do not qualify the complete workspace.
