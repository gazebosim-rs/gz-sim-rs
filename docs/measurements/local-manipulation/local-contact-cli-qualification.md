<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Local contact physics through the command line

The offline simulator loaded a finite rocker onto a support, advanced its ordinary 0.5 ms Bevy step, and printed the resulting link position. The same authored scene advanced through the public scene loader registered one strike at 328,543 ns with more than 0.8 N·s of upward impulse. The printed and observed positions matched in all three coordinates. In a separate zero-gravity run, the reported low point of a two-particle cable and the three coordinates of its moving guide agreed with their free-flight solutions to within 10⁻¹² m at 0, 8, and 40 ms.

Malformed local contact options also receive a diagnostic before any state is published. The executable refused an invalid boolean in each of 46 options, 33 configurations lacking their required timing or geometry, and one selection of two mutually exclusive geometries. Each refusal produced the relevant option diagnostic and exited with status 1. All four tests passed in the ordinary CLI build and in a separate build with default features disabled. Strict Clippy passed.

To reproduce the measurements, run both commands from the workspace.

```sh
RUSTC_WRAPPER= cargo test --locked -p gz-sim-cli --test local_contact_inputs
RUSTC_WRAPPER= cargo test --locked --no-default-features -p gz-sim-cli --test local_contact_inputs
```

The frozen integration candidate was `84569c948e66`, compared with `438b0afb6128`. Its ordinary test run had one dense pose publication failure. A separate coverage measurement combined its 1,323 ordinary process profiles, the profiles from all 39 stock-worker checks, and the 84 new simulator profiles produced by these four local cases. All 783 instruments came from the frozen Cargo build. This exercised an additional 169 of the authored contact parser’s regions, bringing it to 1,222 of 1,246, and 15 more regions in the CLI entry point, bringing it to 242 of 402. These local cases raised headless scene region coverage from 88.817% to 89.195% and CLI from 88.828% to 88.940%. The release target remains 95% per crate.
