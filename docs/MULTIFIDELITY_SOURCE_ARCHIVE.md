<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Historical physics sources

The [integrated simulator](https://github.com/gazebosim-rs/gz-sim-rs/tree/review/local-cpu-mvp-20261124) is the runnable candidate. The source at `c19f05b3c9384a5ac43b0e31413ef5b9923b366e` preserves the original development checkout. The following commits preserve older comparison programs, profiling changes and topic branches. Each can be opened independently with `git worktree add --detach <directory> <commit>`. The pool-tree entry overlays the surviving files of a partial historical checkout on the original simulator snapshot. These measurements used the toolchains and model revisions of their time.

| Workload or topic | Source commit |
|---|---|
| cloth-bench-velocity-source | `11bbb7e11d6b1178497ca01dfa1677ae1d2570e8` |
| cloth-law-candidate | `1f2956d1e5e64eaa44a52e6e1180bb7979cbb128` |
| cloth-timing-source | `a55cbaffce3d96d330a6112170f334516c860555` |
| fold-impact-reference-observer | `8ba0da34ddfc84f515433bd554cca3cd0786bd6a` |
| landing-contact-source | `ea58d6699f7ae4199b3c2f62aa1d6a1287076bfd` |
| local-fleet-native-source | `af2673117ee8729b57a6e58b29555017dda28691` |
| local-fleet-query-abba | `d890e77df070075808b34f7f408ed36ac89e8066` |
| local-fleet-ray-bounds-source | `b11e5c3bcfbd178ba6542583129479d344e31eac` |
| local-kinematic-control | `d80b5744c8ce514f0188161f294fedcc53832cea` |
| local-kinematic-milestone | `7a35994e3009ebfd4696375201cec4b2b4603d2a` |
| local-kinematic-original | `a37e1ca27ef1f6a11afaa8d51150661456c047e5` |
| local-kinematic-publication | `a35034a6ac1cf7cca7845ea5b02713cc7bc843e1` |
| local-kinematic-publication-base | `e347176b2dee12d223139a1334bcc1c14ea04d15` |
| local-kinematic-publication-final | `74b9d4aa0b188c5431299d2d4e0b66eaa2d96b50` |
| local-kinematic-selective | `eb0c89384df996ddf9fc2eafbfef4d429402502d` |
| material-before-restick-source | `6bf6342719e84152577c13affbc5123d9944bbab` |
| shell-cost-source | `fc0eeafc9d611b23c1f09cca86e91b3220a165bd` |
| shell-profile-source | `3866bd4e54f33fc993aaded9f6704b6ff37f451f` |
| stiff-contact-cut-source | `ea4f82214cda8d11d04db46378f3166d9a6bd9f7` |
| pool-tree | `2a5d97dd96caf8302b2676e4610c09b37372c2bb` |
| physics/gpu-glancing-corner-impulse | `a45de42e07990c4256824e8af0ae87050bd49dde` |
| physics/gpu-shelf-reload | `aeedd468111ca3654710a61d85b678762617ebeb` |
| physics/gpu-shelf-stick | `7af27c3d5064a168f8e5685d3e23f2c358636c52` |
| physics/jaw-window-speed | `ea924b279d11c4e146e882132bc5e2a90279ad0d` |
| physics/powered-slider-view | `c3c61faf829c4b41a509c10ab94f5adcaec80ac6` |
| review/local-cloth-conveyor | `4dc024d51b53a57c083c61f960e85dc4428bb8cb` |
| review/local-cloth-turntable | `455198e48a7ad0808b366fc133bfa041124dcbb4` |
| review/local-cpu-cli-coverage-supplement-20261124 | `9a7e2c988e6dc5782000f553d10a67111ae89de5` |
