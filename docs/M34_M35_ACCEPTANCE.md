<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# M34 and M35 accepted source delivery

Qualified Main is `1982ce86918df9caf941f8be79b586757bfa77fb`, tree
`a2302c57b1d6ca9939bcbc77ee262c4334f46612`, parent M33 `17dd62f2`.
Normal nonforced promotion and exact remote readback completed after the full
1,976.362-second normal hook, independent local DATA, exact required hosted
[CI42715967423 attempt1](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/42715967423)
with all16 steps successful, and independent official DATA. The generated
`artifacts/m34-m35-delivery/acceptance-main-promotion.json` record has SHA-256
`2a5295dc8b7470c62e4a1ebb697b4a1be7f269baa70e14b20e6039d0dbc8b030`.
Local DATA is `1215c700…`; official DATA is `82bcbbc0…`; actual producer,
collector, audit and promotion exits are zero. Automatic Main CI is separate
and has not been queried or inferred. M32 release439342072 remains verified
Latest; no new package, installation or manual browser proof is claimed.

## Accepted bounds and measured evidence

M34 is one flat top-level Rapier model's noiseless 2D ground-truth odometry:
zero offsets, actual pose-derived ten-entry history, bounded topics/frames/rate
and three canonical streams. The original system supplied221 observed triples
across301 inputs, with15 unobserved and65 suppressed ticks; all663 canonical
packet comparisons and the direct-world signed-zero control pass. Native first
initialization/history/pose arithmetic is preserved; no encoder integration,
3D/noise/offset extension or native Physics trajectory claim is made.

M35 accepts bounded one-shot, additive persistent and clear commands for existing
centered free Rapier links, with actual force/torque velocity effects, atomic
refusal, lifecycle retirement and real canonical EntityWrench/Entity delivery.
The original system supplied32 ECM component records, not native Physics output.
Native unpaused zero-duration consumption differs explicitly from the selected
Rust advancing-only policy. No new solver, transport, dynamic plugin or general
compatibility claim follows. See the dedicated M34/M35 guides for exact limits.

| Evidence | Headless | Renderer |
| --- | --- | --- |
| Rust profile passes (overlap) | 1,616 | 963 |
| New observed controls | 47 | 22 |
| Retained controls | 45 | 19 |
| Local lines | 31,070/31,896 | 18,385/18,889 |
| Hosted lines | 31,064/31,896 | 18,385/18,889 |
| Local regions | 46,899/48,725 | 27,944/29,129 |
| Hosted regions | 46,892/48,725 | 27,944/29,129 |

All strict workspace, per-crate and actual-parent gates passed independently;
local and hosted totals are not unioned or forced equal. Python367 is aggregate,
not individually logged quiet case outcomes. Each independent audit compared
raw1789/380/23/33 byte-exact once. They bind1554 Git entries,49 changed paths,
1505 protected entries and754 source/input records. Nine finite copied-CLI runs
and six retained auxiliary ELF copies remain distinct from Rust profile counts;
full-profile executables are metadata identities, not retained executable bytes.

Three original hosted ZIPs total131,614,207 bytes, retained in RAM and verified
durable individual copies. Remote SDK DSOs are provenance identities, not
archived remote bytes. Original native/audit failures, the distinct retained-only
finalization, all seven earlier normal-hook outcomes and the later one-off
summary-printer field error remain preserved. No successful workload or raw
comparison was repeated to repair a report. M36 earns its own qualification.
