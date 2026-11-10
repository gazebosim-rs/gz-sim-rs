<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M38 acceptance: noiseless 3D ground-truth odometry

M38 was accepted at Main `00382fbc342ccc79a5e3c126f8bd5a41064c7a80`, tree
`ed71b0c8e009a8e3f345ae440f358ad48190c1ae`, over accepted parent `6d2d5320`,
at 2026-11-10T05:52:46.796390+00:00. Root's ordinary nonforced Main push
returned 0 and exact readback matched. Acceptance receipt SHA-256 is
`b2a63842eb5358c1b3b7b74efdd945638e8b291095b2b82fe822eab344715438`.

The corrected full normal hook passed in 1,936.349 seconds. Exact hosted CI
[42760064239 attempt 1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42760064239)
passed all sixteen steps. Independent local DATA `49c85a07` and official DATA
`db27227e` passed, with actual Root exits 0 for normal, local audit, hosted watch,
original-evidence collection and official audit. Durable original ZIPs total
133,032,274 bytes; the retained evidence was not replayed to record acceptance.

Headless 1,639 and renderer 976 passes overlap (2,615 profile passes, not unique
test identities); Python 380 is aggregate. New controls are 14/9, retained
controls 97/45. All strict workspace/per-crate/actual-parent line and region
gates passed unchanged. Hosted headless coverage is 31,635/32,472 lines and
47,825/49,669 regions; renderer is 18,764/19,268 lines and 28,592/29,784 regions.
Local headless is 31,637/32,472 and 47,827/49,669; local renderer is
18,762/19,268 and 28,590/29,784. Keep those measured results separate.
Four raw comparisons per independent audit ran once from 1,793/382/27/33 inputs.
The audits bind 766 Source inputs, 1,575 tracked/25 changed/1,550 protected
entries, thirteen finite CLI commands, the relative OBJ and six auxiliary ELFs.

Fresh original-system pose-history reference and native DATA `42b68ad5`, then
literal DATA `74770276`, bind 510 inputs, 429 observations and 1,287 packets.
The actual Rust numeric and canonical byte replays passed exactly. This is a
component pose-history proof, not native Physics trajectory or performance
parity. The default planar path and its 301/221/663 fixture remain unchanged.

The first normal hook failed because a new test expected healthy loaded reset
after deliberately latching an Identity fault. Existing core preflight correctly
refused with `Clock(Faulted)`. Only that test and guide changed; the correction
preserves healthy-reset positives and strengthens terminal refusal/nonmutation.
The original failure and all prior evidence remain intact.

This Source increment added no preview tag/package/browser replay. Actual Latest
remains [m37-preview](https://github.com/gazebosim-rs/gz-sim-rs/releases/tag/m37-preview),
release 439888006. Automatic Main CI is separate and is not inferred. Continue
M39/M40 over exact accepted M38 with every new feature and distribution gate.
