# Gazebo in Rust

This repository contains the implementation plan and the first Rust building
blocks for `gazebosim-rs/gz-sim-rs`: a Bevy ECS simulation clock and schedule,
a full-f64 Rapier rigid-body probe, and a noise-free IMU measurement model.
It is not yet a complete simulator. See [implementation status](docs/STATUS.md)
for tested behavior, coverage, reference evidence, and remaining work.

Run `python3 scripts/verify.py` for formatting, lint, tests, and strict per-crate
coverage checks. Install the commit hook with `git config core.hooksPath .githooks`;
it checks an isolated copy of the staged tree. Rust 1.95.0 is pinned.

The design combines the responsibilities of `gz-sim`, `gz-sensors`, `gz-physics`,
and the necessary parts of `gz-math` in one Cargo workspace. Bevy supplies the ECS
and renderer. Existing Rust libraries replace general-purpose Gazebo utilities.
The simulator retains DART, MuJoCo, and TPE and adds a Bevy-integrated Rust physics
backend. Bullet is removed. The replacement GUI is a separate application.

`sdformat-rs`, `gz-fuel-tools-rs`, `gz-transport-rs`, and `gz-msgs-rs` remain
independent repositories in `gazebosim-rs`. Fuel already has another owner; this
plan does not modify that work.

The owner has confirmed behavioral, file-format, protocol, and test compatibility
with new Rust APIs, and Linux first with headless and Bevy rendering support.
Native runtime plugins use a versioned C ABI. The Rust physics backend must use
a full-f64 solver; prioritize Rapier and validate it with probes. Balanced robotics
is the primary performance workload. Remaining proposals and validation choices are in
[the decision register](docs/DECISIONS.md). Review those before implementation.

Public repositories: [gz-sim-rs](https://github.com/gazebosim-rs/gz-sim-rs),
[sdformat-rs](https://github.com/gazebosim-rs/sdformat-rs),
[gz-msgs-rs](https://github.com/gazebosim-rs/gz-msgs-rs),
[gz-transport-rs](https://github.com/gazebosim-rs/gz-transport-rs), and the
independently owned [gz-fuel-tools-rs](https://github.com/gazebosim-rs/gz-fuel-tools-rs).

Start with [the implementation plan](docs/PLAN.md), then use these documents:

| Document | Purpose |
| --- | --- |
| [Agent instructions](AGENTS.md) | Mandatory implementation and writing rules |
| [Repository audit](docs/REPOSITORY_AUDIT.md) | What moves, what disappears, and why |
| [Architecture](docs/ARCHITECTURE.md) | ECS, time, physics, rendering, sensors, and extensions |
| [Dependency decisions](docs/DEPENDENCIES.md) | Reuse candidates and semantic gaps |
| [Independent repositories](docs/EXTERNAL_REPOS.md) | Deliverables and integration contracts |
| [Test and coverage policy](docs/TESTING.md) | Existing tests, strict >95% coverage, pre-commit, and CI |
| [Performance plan](docs/PERFORMANCE.md) | Workloads, measurements, and proposed release budgets |
| [Implementation tasks](docs/IMPLEMENTATION_TASKS.md) | Dependency-ordered work packets and acceptance evidence |
| [Decisions](docs/DECISIONS.md) | Confirmed requirements, proposals, and open questions |
| [Baseline](planning/baseline.json) | Exact local upstream source commits |
| [Source inventory](planning/source-inventory.json) | Public headers, systems, and test-source candidates |
| [System assignments](planning/system-migration.json) | All 78 systems allocated to implementation families |

The owner selected upstream `main` branches. The baseline records local main
commits inspected on 2026-10-24 (America/Los_Angeles); P0 verifies their freshness
and compatibility before running the reference suite.
The source inventory is a starting list, not an executed test manifest.

The first implementation milestone establishes reproducible C++ reference runs,
dependency versions, test traceability, and enforced quality gates. A small
end-to-end simulation follows before broad feature migration.
