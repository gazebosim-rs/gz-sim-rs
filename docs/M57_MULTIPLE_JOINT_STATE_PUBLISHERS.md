<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Multiple joint-state publishers

M57 extends the admitted JointStatePublisher to zero through eight loaded top-level dynamic Rapier models. Each model has one plugin, an independent topic and rate, and a nonempty deduplicated selection of up to 128 single-axis revolute joints. Existing graph, namespace and metadata restrictions still apply. Publication uses the canonical Model message; unavailable force stays omitted.

[The example](../examples/m57_multiple_joint_state_publishers.sdf) has two passive articulated arms. The first publishes both joints at 100 Hz; the second repeats a tip-joint filter, which is deduplicated, at 50 Hz. Their topics are `/m57/joint0` and `/m57/joint1`. Both use the existing world-anchored two-joint geometry with simple shapes and default unactuated limits. There are no controller plugins: the existing single controller-chain admission is unchanged. Gravity can move the admitted passive geometry; the finite check requires complete named finite positions, not an invented exact solver trajectory.

After this development Source completes qualification, the finite command is:

```sh
cargo run --locked --no-default-features -p gz-sim-cli --   examples/m57_multiple_joint_state_publishers.sdf --physics-engine rapier --iterations 20
```

This is not a command for the older installed M40 package. It proves finite loading/stepping only. Separate public regressions must use real backend joint motors and the existing articulated-wrench path, retaining the typed NotFreeLink refusal. No body teleport supplies a motion witness.

Each owner keeps its existing time rules and sample revision. A pause is not a new publication gate: equal-time calls remain eligible, while positive elapsed time below the configured period holds the prior sample. A core reset retains its existing history; a supported loaded reset reconstructs original owners and selected joints with stable IDs and new handles. Removing the last owner retires the family's callbacks and endpoints. A global joint-binding fault remains global and terminal; caches need not clear and peers are not promised progress after that fault.

The Server checks original, live, prepared and retained identity for the full set before publisher IO. Original full names retain their world prefix. Reset restores removed original models/joints from immutable metadata while preserving valid topic remaps. Immutable output endpoints prevent distinct handles from being swapped between owners. Existing manual zero/singleton compatibility, collision reservations and typed error order remain supported.

The fresh original Configure/PostUpdate component campaign and independent native audit passed with separate actual Root0 results. The validated borrowed fixture export and independent fixed-literal audit also passed. The retained pair/eight cases account for 100 complete inputs/Post calls/fences and 66 Model messages with 108 selected Joint records. These fixed pose inputs and controlled q/v values are not Physics trajectories. Actual native IDs, ordering and full root/nested wires remain intact.

Observed Scene and Server consumer Source is reviewed and integrated with the actual audited fixture/provenance, exact helper and append-only hooks. The original consolidated formatter passed and a reachable 22 headless/11 overlapping renderer Source census was recorded; no Rust pass follows from those declarations. Scene checks a genuine loaded graph and fixed construction getters, then supplies only public JointSample inputs before one unchanged global sampler call; it never changes a pose or backend. Server's isolated captured-value encoder, native wire custody and live Rust canonical peer remain separate boundaries. Live-wire checks retain actual Rust identities; native IDs/bytes are not rewritten to fit them. Real motor/force regressions remain distinct.

The first full verification failed compilation at one missing Name import; its
reviewed correction and formatter are complete. The second normal failed after
1650.1202665108722 seconds in two runtime-test premises. Scene tried unsupported
finite limits/damping; Server expected an immediate global fault after entity
disappearance despite subperiod suppression. Strict Clippy,403 aggregate Python
and the maintained native-free/RGB stage passed, but full normal/coverage and local
DATA did not. Original failed trees/results/logs remain failed and retained.

The bounded test correction keeps production unchanged: genuinely load supported
unbounded/zero-damping graphs, separately validate exact captured metadata and
explicit unsupported-domain refusals; distinguish live binding corruption from
entity retirement with unchanged global terminal/cache/reset/endpoint checks.
Complete independent Source review before affected formatting and full retry.
No native campaign, fixture or old RAW replay is authorized.

M57 is development Source. See [the delivery contract](M57_DELIVERY.md) for ownership, exclusions and remaining gates. Source review or unit readiness does not establish release acceptance; hosted CI and official qualification remain required.
