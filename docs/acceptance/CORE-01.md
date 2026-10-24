# CORE-01 acceptance

Accepted at [d3524f58](https://github.com/gazebosim-rs/gz-sim-rs/commit/d3524f58c18d7f3de9a1cd59d9a2c9a66befc5f2).
The [matching hosted gate](https://github.com/gazebosim-rs/gz-sim-rs/actions/runs/40133104649)
passed. This record accepts the foundation packet's concrete exit criteria in
[IMPLEMENTATION_TASKS.md](../IMPLEMENTATION_TASKS.md), not a complete plan phase.

| Requirement | Evidence |
| --- | --- |
| Minimal Cargo workspace and f64 values | Six focused production crates; f64 physics, sensor and scene values; Rust 1.95.0, Bevy ECS 0.20.0, Rapier3d-f64 0.36.1 and glam 0.33.12 pinned |
| Stable IDs and lookup | Opaque, world-local, nonzero EntityId; monotonic allocation; Bevy lifecycle cleanup; live-owner lookup; transfer, stale-entity and exhaustion tests |
| Simulation clock and epochs | Integer Duration and u64 iteration/epoch; exact stepping, pause, reset, overflow preflight and independent-world tests |
| Tests and coverage | 54 Rust checks and 9 gate regressions; core 167/167 production lines and 219/222 regions; workspace 1176/1186 lines and 1702/1738 regions; every crate above 95% |
| Headless dependency closure | Independently reproduced Linux x86_64 normal dependency tree contains no bevy_render, bevy_window, bevy_winit, wgpu or winit |
| Independent review | Messages repository reviewer audited the packet exit and replayed all 54 Rust checks; code review covered diff SHA256 `0e304c78589def0196161e931a189ae7235a03c1bd2426f4bb63bda97f8724db`; exact staged and hosted gates passed |

The accepted Cargo.lock SHA256 is
`994aa74c6870357206b3ff167960e0fe5553d267edd526603fbbb6c97a36a14d`.
Reproduce the gate with `python3 scripts/verify.py` after building the native
client reference described in [HEADLESS.md](../HEADLESS.md). Inspect the Linux
closure with:

```sh
cargo tree --locked --offline --workspace -e normal --target x86_64-unknown-linux-gnu --prefix none
```

API-01 and QLT-01 remain open across their broader contracts and profiles.
CORE-02, initial-scene reset, remote removal, renderer profiles, full upstream
test mapping, performance and release qualification also remain open. The native
clock, pose and IMU clients add integration evidence; they do not establish full
simulator parity. No upstream test is retired, coverage source excluded, or plan
phase certified by this acceptance.
