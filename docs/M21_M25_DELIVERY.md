<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M21–M25: one qualified sensor-preview integration

This sequencing correction retains D30–D32 and each phase's original scope,
start and at-most-five-working-day budget. It combines already reviewed source
into one candidate over accepted M20 `9a3e81cf`; it adds no new runtime feature.
Root integrates in `gz-sim-m21-m25`, with qualification artifacts under
`artifacts/m21-m25-delivery`. The future review ref is
`review/m21-m25-integrated-preview`. M20 remains the accepted installed preview
and Latest until the new distribution actually passes and is published.

## Reason and retained failures

Frozen M21/M22 `36eb202c` passed its full normal hook and independent local DATA.
Both exact hosted attempts of run 42493504723 failed linking the Scene renderer
test library: cc/rust-lld LLVM 22 reported SIGBUS7. Their original logs and partial
artifacts are retained in RAM and verified durable copies. Neither qualifies a
whole CI run. No recorded test assertion, disk-full or OOM diagnostic establishes
the cause. Do not call the underlying cause known or launch another unchanged
retry. M20 previously passed hosted debug information level 2 too.

The passing local normal uses dev/test debug information level 0, while both
failed hosted attempts used level 2. The new CI configuration aligns these two
debug-info settings with local qualification and adds bounded pre/post disk,
inode, memory/cgroup and target-allocation diagnostics. This reduces a concrete
configuration mismatch and debug-symbol burden; it is not a proven SIGBUS fix.
The exact verification exit is preserved even if diagnostics fail. Pre/post
samples do not establish transient peaks. Toolchain, linker selection, SDKs,
locks, instrumentation, optimization/debug assertions/overflow checks, targets,
features, numerical assertions, strict coverage and all mandatory jobs stay intact.
No cache, old artifact or service is removed.

## Integration and acceptance

M21/M22 altimeter and pressure noise, M23 magnetic noise, M24 planar CPU-lidar
noise, and M25 installed-preview packaging retain their separate functional
contracts and regression sources. The complete reviewed M25 source snapshot
already contains the earlier source increments. Combining them avoids three
serial normal/hosted qualification cycles. It replaces the old requirement to
publish each intermediate parent before checking its child with one atomic
qualified candidate. It does not reuse an old whole-gate success as a new result.

Required sequence:

1. Finish the complete source, regressions, guides and CI correction, and obtain
   independent integrated Source review. Preserve the old `36eb202c`, `a42b1c94`
   and `f040aae5` checkpoints unchanged. All previous passes/failures remain tied
   to their actual trees and methods.
2. Run one new full normal hook against accepted M20, with unchanged strict
   workspace/per-crate line AND region coverage and actual-M20-parent checks.
   Independently audit retained source, profiles, six actual auxiliary ELF copies,
   all new Rust/Python contracts and four actual copied-CLI finite controls.
3. Publish the exact reviewed candidate normally to its new review ref and pass
   its complete exact hosted CI plus independent official DATA with durable
   original ZIPs. Never run success collectors against failed runs.
4. Build fresh source-bound release CLI/helper inputs; collect actual dependency,
   ELF and notice closure; assemble, verify, extract and verify the relocated
   archive, followed by independent retained DATA. Retain exact c404 viewer bytes.
5. Prove a clean supported Ubuntu 24.04 non-root installation, actual runtime
   library ownership, absence of compiler/SDK requirements, and all four installed
   examples executing two real finite steps with complete stdout and actual exits.
6. Use the maintained browser with the newly installed default robot for real
   Play, forward, turn, Pause, Reset and owned clean shutdown. Retain actual images,
   process identities/waits/listener retirement and unknown diagnostics faithfully.
7. Publish the M25 archive, manifest, guides, checksums and actual evidence. Verify
   exact tag, assets, notes and intentional Latest. Accept M21–M25 together only
   after all their criteria pass, update the tracker and continue the next phase.

Root alone serializes local heavy work. Ordinary source/review lanes can overlap
hosted CI. All operating floors and full normal initial reserve remain unchanged.
No new eviction, old-preview port loan, renderer rebuild, old manual-proof replay,
transport/plugin framework or weaker checks are authorized.

## Native evidence and ownership

Retain three distinct native evidence roles: original AEC altimeter/pressure
capture and DATA `6f780b9c`; original tree951 magnetic/CPU-lidar capture and DATA
`5853f74c`; and inherited joint-state-only DATA `d47a6a15`. Do not relabel their
original capture trees, procedures, results or counts as the aggregate candidate.
The old magnetic proof does not qualify changed magnetic noise. The new candidate
must bind every relevant unchanged helper, model/runtime/Scene/Server/shared
Gaussian/scheduling/physics/core/proto/Cargo/fixture and SDK input for each role,
with exact reviewed exports/metadata exceptions only. Broad changed-file exclusions
or equality to an unrelated old whole-tree map are invalid. Actual Rust integration
and wire behavior must pass the new complete normal and hosted checks.

Root owns `.github/workflows/ci.yml`, shared planning/docs and integration. Light
owns retained normal/local/official collection/audit procedures; Messages owns
release/package/install procedure adaptation. SDF independently reviews the full
source and batched findings. All operation pins stay held until actual preceding
results and final Source bindings exist. Product archive/guides/container contracts
remain named M25; source qualification identifies the aggregate separately.
