<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# World light include adoption

This candidate pins sdformat-rs `13adde94a8e2ca9f3c6a93d46dee02fa129326b6`.
That producer revision passed its complete normal gate, exact hosted run
`41326073574` and independent official source, coverage and native-evidence audit
before Main publication. Its 688 Rust and 50 tooling checks qualify the producer;
they do not qualify this consumer candidate.

The producer resolves a world include containing exactly one external Light.
The include name and pose replace the resource values; the Light retains the
resource's canonical source. Existing frame validation remains authoritative.
Root/model Light includes and model-only include options remain unsupported.
No production resolver, runtime system or dependency is introduced here.

The supplemental Scene test loads the producer's unchanged imported
`included_light_original.sdf`, checks the explicit include override in a world
frame, then verifies full metadata, canonical source and original entity identity
across removal and loaded reset on all four existing backends. It compares
scheduled work with a lightless world. The fixture bytes match the accepted
producer fixture; its imported XML carries no added copyright claim.

Only the producer revision changes in Cargo.toml and Cargo.lock. Full locked
Cargo metadata resolves the exact new source without dependency version churn.
The focused Scene world-light target passes all nine controls, including the new
four-backend include/reset case. Full workspace coverage and staged/hosted
qualification have not run. Existing Light renderer extraction and Server
pose-wire refusals remain; this does not qualify light rendering, native application wire parity,
B03 or broader system activation.
