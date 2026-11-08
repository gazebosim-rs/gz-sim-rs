<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M19: refreshed installed preview

Deliver one Ubuntu 24.04 x86_64 native-free preview containing the accepted
M13–M18 features, with a five-working-day budget including review and mandatory
verification. Source preparation may overlap M17/M18 qualification under D31;
the new distribution must use their finally qualified Main source. After the
exact parent passes its local normal hook, native proof and independent local
audits, M19's reviewed local checks may overlap that parent's hosted CI. Local
heavy jobs remain serial. Neither phase may be accepted or released before its
exact hosted checks and all required evidence pass. If the parent needs a source
correction, rebase M19 and repeat the affected required checks. Preparation does
not accept M17/M18 or qualify an installer.

Reuse the existing assembler, standard-library loopback launcher, finite drive
helper and exact accepted c404 viewer files with both unchanged 14 MB gzip
budgets. Package the coexistence robot as ordinary `share/robot.sdf`, including
its referenced local OBJ file under `share/meshes/`. Preserve all old releases,
frozen trees, accepted demos, Fuel work and evidence. Do not rebuild the renderer.

The installed guides become `INSTALL.md` and `CAPABILITIES.md`, sourced from
dedicated M19 guides. The verifier also continues to accept the complete legacy
M12 guide pair, so existing schema-1 installations remain verifiable. A partial
or mixed guide pair cannot replace either complete pair. Keep all runtime,
notice, ordinary-file, hash, mode and complete-inventory checks.

Root owns the fixture, guides, shared planning, integration and publication.
The asset owner leases `scripts/package-preview.py` and
`tests/test_preview_package.py`: copy the fixed new fixture, its fixed local
mesh and the dedicated guides; retain admission bounds, atomic publication and
archive protections. The Messages owner leases `scripts/preview_runtime.py`
and `tests/test_preview_runtime.py`: admit the new complete guide pair while
retaining legacy verification and meaningful negative controls. This lease also
includes the existing CLI WebSocket/asset origin checks, their tests and startup
message for the one additional HTTP port described below. The SDF owner
independently reviews the complete source, fixtures and documentation. No tests,
lint, imports or product executions run before the complete bounded batch and
independent source review. Batch findings before one consolidated pass.

Actual host inspection found both supported HTTP ports occupied: wildcard 8080
and the protected old static server on loopback 8081. Add exactly loopback HTTP
8082 to the launcher's admitted choices and the CLI's explicit WebSocket/asset
origin lists for `127.0.0.1` and `localhost`. Keep the default and prior choices,
loopback binds, path restrictions and every other origin refusal. Regression
controls must admit both new exact origins while refusing adjacent port 8083,
remote hosts, alternate schemes and malformed origins. Do not allow arbitrary
origins or terminate an old service. The existing viewer bytes remain unchanged;
the new installed artifact uses HTTP 8082 for its fresh integration proof.

Acceptance requires fresh locked release-profile native-free CLI/helper builds
from the new qualified source, actual Cargo artifact identities and dependency
notices, and actual ELF/system-library closure. Do not reuse instrumented test
binaries or relabel the old M12 executables. Reuse unchanged notice inputs only
where their exact identities remain applicable; new selected dependencies need
their own complete texts. Canonical template and empty-source disclosures remain
explicit and are not legal opinions or author grants.

Assemble, archive, relocate and verify the actual copied payload and manifest.
Independently inspect retained package data. On clean supported Ubuntu, install
ordinary signed prerequisites, demonstrate actual non-root installed verification
and runtime library ownership without a compiler or native SDK, then run the
installed robot through real forward/turn commands, Pause, Reset and owned clean
shutdown in the maintained browser. Prove the refreshed artifact integration;
do not replay accepted individual feature qualification. Retain any actual
shutdown observation limits rather than inventing a grandchild exit code.

Full normal hooks, strict workspace/per-crate coverage, exact hosted CI and
independent Source/DATA review remain required before Main promotion and a new
public preview release. The M12 release remains unchanged. No new transport,
plugin framework, solver, sensor family, UI, asset format or backend is included.
