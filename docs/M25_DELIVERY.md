<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M25: updated installed preview through M24

Selected under D30–D32, with a budget of at most five working days including
independent review and mandatory checks. Ship a fresh Ubuntu 24.04 x86_64
native-free archive containing the qualified sensor features through M24.
M20 remains the installed preview and Latest until this release actually passes
its own checks and publication. This does not rename or reuse an old archive.

## Atomic integration update

[M21_M25_DELIVERY.md](M21_M25_DELIVERY.md) supersedes the intermediate-parent
sequencing below. All already reviewed M21–M25 source now qualifies in one new
candidate over accepted M20. Every separate functional/native criterion and new
full normal/exact hosted/independent DATA gate, followed by all fresh M25 product
proofs, remains mandatory before accepting these phases together. The original
source checkpoint and ownership history below are retained; this is no runtime
scope expansion or old-evidence relabeling.

## Parent and parallel work

Source starts from locally qualified M23/M24 `a42b1c94`, tree `951da29d`.
Its full normal hook and fresh native DATA passed; its retained/hosted acceptance
still depends on M21/M22. That parent's unchanged second hosted attempt is
running after a preserved LLVM linker SIGBUS of unknown cause. No parent or
child acceptance is inferred from source work or a partial CI result.

Authors may prepare disjoint source lanes now. Root integrates the finally
reviewed parent before checks. After actual parent normal/native/local DATA
clearance, reviewed provisional local checks/artifact preparation may overlap
parent hosted qualification with local heavy work serialized. Acceptance,
Main promotion and release publication require the finally accepted parent and
every M25 gate. Parent source corrections require integration review/rebasing
and affected checks. Preserve every qualifying tree and original failure.

## Product scope

Reuse the maintained assembler, installed launcher, full M19 coexistence robot,
finite drive helper and the exact six accepted c404 viewer files. Default robot
launch, lifecycle/credit opt-in, loopback ports, canonical commands, braking and
pause/reset behavior remain unchanged. Do not rebuild the viewer.

Add ordinary immutable package copies of these already reviewed parent fixtures:

- `examples/m21_altimeter_noise.sdf`
- `examples/m22_air_pressure_noise.sdf`
- `examples/m23_magnetometer_noise.sdf`
- `examples/m24_cpu_lidar_noise.sdf`

Install them under `share/examples/` with their original basenames. Include their
exact bytes/modes in the manifest and archive, verify them after relocation, and
document direct finite execution with the installed `bin/gz-sim-rs` and
`--iterations 2`. Their world name is `default`. These are self-contained SDF
examples, not a new browser sensor dashboard. Keep the default robot workflow
simple; add no scene-selector API, telemetry facade, message CLI or new helper.

Update dedicated M25 installation, capability and preview guides. State the
actual source/binary profile, sensor noise subsets (including CPU-lidar Gaussian
zero minimum), canonical-message consumers, limits, browser requirements and
distribution exclusions. Copy those guides to the installed product rather
than silently relabeling the old release documentation. Preserve all existing
root/viewer/font/vendor/proto/Rust/dependency notices and provenance, including
the published-source limitations and canonical-template disclosures. No license
branch selection, invented author grant or legal-compliance verdict is allowed.

No simulation/physics/noise-model/transport/viewer production change, new Cargo
dependency, native backend or RGB/depth-worker distribution is included. Reuse
qualified parent native model/wire evidence only through explicit unchanged
source checks. This packaging increment needs no new C++ model campaign.

## Ownership

| Owner | Worktree | Exclusive files |
| --- | --- | --- |
| Light | `gz-sim-m25-package` | `scripts/package-preview.py`, `tests/test_preview_package.py`; new `docs/M25_INSTALL.md`, `docs/M25_CAPABILITIES.md`, `docs/M25_PREVIEW.md`. |
| Messages | `gz-sim-m25-install-checks` | `scripts/qualify-rapier-runtime.py`, `tests/test_rapier_runtime_coverage.py`, `tests/test_rapier_runtime_retention.py`; finite installed-example procedure design as source only. Keep existing Rust cases and targets unchanged; require all four real copied-CLI fixture loads/steps with actual argv/output/exit evidence. The closure-only `check-rapier-runtime.py` remains unchanged. |
| Root | `gz-sim-m25` | Shared planning/routes/README/AGENTS, integration and qualification/publication procedures; coordinate any concrete additional lease. |
| SDF | Read-only | Independent complete source/test-intent review and batched findings. |

Do not edit `preview_runtime.py`, the notice collector, Cargo, runtime leaves,
another owner's files, accepted/qualifying trees or Fuel. Authors run no tests,
lint, formatting, build, metadata, SDK/container/browser or product commands.
Finish implementation, regression sources, documentation and independent review;
batch findings before one consolidated required verification. Root alone releases
and watches local heavy operations. All operating floors and the native-gate
initial reservation remain unchanged. No eviction is authorized.

## Required exit evidence

1. Retain old package/launcher tests and assertions. Cover all four fixture
   copies, exact bytes/modes/manifests, missing/tampered examples, ordinary files,
   immutable serving and clear finite command documentation. Add genuine
   native-free CLI load/step controls to the maintained checker and meaningful
   malformed/failed-control regressions without fabricating pass counts.
2. Pass the full normal hook, strict workspace/per-crate/actual-parent coverage,
   exact hosted CI and independent retained source/profile/native audits. Do not
   infer release acceptance from source review or a successful unit test.
3. Build fresh source-bound CLI and finite drive helper with ordinary release
   Cargo evidence. Collect actual artifact IDs, ELF dependencies, package inputs
   and exact notice closure; do not substitute old binaries even if bytes match.
4. Assemble once, verify the assembled prefix, extract the actual archive into a
   new location and verify the relocated manifest. Independently audit the
   retained payload/archive/modes/notices/source closure without replaying checks.
5. On a clean supported Ubuntu installation, prove non-root installed verification,
   actual library ownership/no compiler or SDK requirement, and all four installed
   SDF examples loading and stepping through the real copied binary. Retain actual
   commands, output and exit codes. This is not noisy wire or trajectory proof;
   those contracts retain their separate qualified parent evidence.
6. Use the maintained browser with the newly installed default robot: real Play,
   finite forward and turn commands, Pause, Reset and owned clean shutdown with
   actual process/listener retirement. Reuse existing reviewed procedures after
   binding the new package and actual outcomes. Keep any unrecovered/unknown
   diagnostics honest; simulator-grandchild exit remains unknown where the
   launcher does not retain it. Do not replay an old accepted manual artifact.
7. Publish the checked archive, manifest, guides, checksums and actual acceptance
   evidence with exact tag/asset/body/Latest readbacks; preserve every old release.
   Update the web tracker and continue the next useful bounded phase.

Use admitted free HTTP 8082/WS9014/Zenoh7449 for installed proof, preserving old
8080/8081 services. An old M12 port-loan reply is obsolete and unused; do not
signal old demos or reuse that handoff. Unknown port ownership means hold the
affected operation, not eviction or extra signals. No package/container/browser
success is claimed at this Source-selection checkpoint.
