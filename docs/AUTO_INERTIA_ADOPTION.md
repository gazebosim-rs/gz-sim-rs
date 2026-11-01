<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Automatic primitive inertia and collision density

The workspace pins SDFormat to `b246a806006550ace0930cd3cd3ecc503b3b1af0`.
Its bounded automatic-inertia loader uses public Parry mass properties for one
origin box or sphere with explicit positive density and no overrides. The
producer documents its numerical domain, preserved exact original assertions,
and new differential roundoff bounds. Scene does not calculate inertia again.

Scene retains explicit density as `CollisionDensity` on the collision entity,
in kg/m³. Omission leaves that component absent. Admission requires a finite,
positive value; the remaining collision, model, sensor and backend guards apply
unchanged. Explicit manual and static density are retained too, without changing
the producer's authoritative mass and inertia tensor.

This component is construction metadata. Editing or removing it does not change
the physics body's mass, recompute inertia or rebuild collision geometry. Loaded
reset restores the initial value or true absence and collision identity.
Producer include URI/line/column provenance is checked separately; this change
adds no per-entity Source or Element facade. No new callback or simulation system is
installed for density.

Focused controls compare automatic box/sphere bodies with exact manual-inertia
counterparts across TPE, Rapier, DART and MuJoCo load, step and reset. TPE retains
its kinematic and unsupported-wrench behavior. Existing backend restrictions,
including tiny geometry and plane admission, remain explicit. Include provenance,
static/manual density, mutation/removal and nonfinite values have separate cases.

Two earlier supplemental density refusals receive reviewed positive replacements:
the same mutable World DTO at URI `guard`, and the same static-sphere XML at URI
`sdf8`. The former validates metadata only; its hinge is not relabeled as TPE
lowering support. Every unrelated negative predicate remains unchanged.

This is a bounded consumer adoption, not complete automatic-inertia, mutable
SDFormat API or native framework parity. Acceptance requires independent review,
the complete normal staged hook, strict owned coverage, real native/rendering
checks and exact hosted/source qualification.
