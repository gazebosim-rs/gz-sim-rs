# Shortening the gripper solve after impact

The left and right jaws of a six-body serial robot struck a free 200 g parcel across either a 35 µm or 50 µm gap. Keeping fine steps through the approach and impact, then taking longer steps after verified separation, reproduced the timing, linear impulse and angular impulse of every pulse in eight independent reference runs. The largest change in any body’s position over the following 24 ms was 0.953 µm relative to the original fine-prefix policy. [Measurements and raw timing samples](released-jaw-tail.json)

## What controls the step size

The existing predictor nominates one approaching, parallel box face. The original fine prefix starts from the real incoming state and must fit its authored budget. The additional SDF fields `early_face_guard_ns` and `early_face_tail_step_ns` enable one check after the predicted crossing, rounded upward to a fine-grid boundary. At that boundary, the native solver requires a positive impulse from the nominated pair, no positive impulse from a second pair in this interval, a closest-point gap greater than the combined contact skins, and positive relative normal speed at the two closest points. The two colliders must be individual cuboids. Everything through that boundary runs on the original fine grid.

Once the check passes, the rest of the original fine prefix uses the authored longer step, capped by the surrounding native grid. A failed check spends the full original fine budget. Each native solve still handles collisions, joint constraints and the held actuator input. Command controllers and normal observations stay on the 1 ms public clock. The two fields are optional and must appear together. This qualification used one normal impact against a mounted gripper with a 100 kHz carriage guide, zero incoming spin, a parcel speed of 0.75 m/s and the 1.75 inertia divisor.

## Impact and motion

| Incoming gap | Jaw | 25 ns pulses | 25 ns total (mN·s) | 1 ns pulses | 1 ns total (mN·s) |
|---|---|---:|---:|---:|---:|
| 35 µm | Left | 2 | 47.6686 | 4 | 48.2550 |
| 35 µm | Right | 3 | 47.5331 | 2 | 47.3385 |
| 50 µm | Left | 2 | 48.8457 | 3 | 47.4359 |
| 50 µm | Right | 1 | 48.1948 | 2 | 46.5912 |

Each comparison started from identical six-body states with no cached contact points. Continuous controls took 40,000 or 1,000,000 native steps over the entire first millisecond at their respective resolutions. Every per-pulse start/end time, number of loaded steps and rows, normal impulse, vector impulse and moment about the parcel’s center of mass matched exactly at its own resolution. The resolution dependence remains measurable, particularly on the 50 µm right jaw. [First-strike wrist and finger loads](wrist-impact-loads.md)

The 25 ns profile used a 2 µs guard and a 125 ns tail. The 1 ns profile used a 3 µs guard and a 25 ns tail. A shorter 2 µs guard retained the original 93,808 calls on the 50 µm left jaw because separation was unconfirmed. With the longer guard both mirrors passed. At 24 ms, comparison with a continuous 25 ns control across all 24 public ticks and six bodies gave maxima of 0.187 mm and 0.112° for the 25 ns profile, and 0.538 mm and 1.277° for the 1 ns profile. Both production profiles used one native call per public tick after the first millisecond. The 1 ns control covers the first millisecond.

Twenty holdouts repeated the ordinary SDF path across both mirrors and both resolutions, with touching, 175/350/1,200 µm separations and a transverse miss. Each produced the same 24 public samples as the original policy for that resolution. The reset regression also checked every observed native timestamp and the full public contact receipt.

## Why the approach stays fine

A separate sweep ran 72 explicit schedules on the same six-body robot, including controls. At a 25 ns target, a 125 ns approach followed by 25 ns steps in a ±2 µs impact window split the 50 µm right-jaw pulse. At a 1 ns target, even a 2 ns approach added a pulse on the 50 µm left jaw. A 5 ns approach kept the pulse counts but changed individual impulses by as much as 0.864 mN·s. Finishing the last part of the approach on the finer grid did not erase the guide motion already accumulated. The `coarse_preflight_splits_the_fifty_micron_right_jaw_reaction` Rust test reproduces the 125 ns control using the ordinary native schedule API. The measurement file records each schedule, its impulses and its trajectory error.

## Cost of the first public tick

| Gap | Jaw | Fine step | Native calls before → after | First-step p95 before → after |
|---|---|---:|---:|---:|
| 35 µm | Left | 25 ns | 2,560 → 2,118 | 374 → 356 ms |
| 35 µm | Left | 1 ns | 62,560 → 50,241 | 10.56 → 6.70 s |
| 35 µm | Right | 25 ns | 2,560 → 2,118 | 412 → 345 ms |
| 35 µm | Right | 1 ns | 62,560 → 50,241 | 8.20 → 8.46 s |
| 50 µm | Left | 25 ns | 3,808 → 3,006 | 500 → 416 ms |
| 50 µm | Left | 1 ns | 93,808 → 70,689 | 15.18 → 10.41 s |
| 50 µm | Right | 25 ns | 3,808 → 3,006 | 507 → 495 ms |
| 50 µm | Right | 1 ns | 93,808 → 70,689 | 13.34 → 11.57 s |

For the 25 ns profile, five alternating warm blocks of four fresh worlds per case ran on the same CPU. Both mirrors and gaps reduced p95 in this comparison and in a separate run of 20 fresh worlds per case. For the 1 ns profile, each gap/mirror had a dedicated CPU, with 20 timed samples per setting and reversed order between mirrors. The 35 µm right jaw reduced median time from 7.82 to 6.99 seconds while its measured p95 rose from 8.20 to 8.46 seconds. The measurement file retains every sample. Each p95 is the 19th fastest of 20.

Loading the SDF, initializing the Bevy schedule, building recordings and checking results were outside each timed `step_exact(1)`. Every timed world was fresh. The harness compared all six committed body poses and velocities with its unobserved reference, and checked the public time and native call count. Durations are single-host release-build measurements. All 23 subsequent 1 ms ticks cost one ordinary native call on these four strikes.

## Watch the contact

The cyan ball beside the gripper rises 2 cm for each 10 mN·s of accumulated impulse. Step through the first and second rise on the 35 µm left jaw, then compare the 50 µm right jaw. Each frame contains the six measured poses at a solver timestamp. Both recordings passed live gz-web-rs WebGPU playback, reset and pose/shape checks for all 3,234 displayed body visuals. [35 µm left jaw](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11424%2F) · [50 µm right jaw](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11425%2F) · [Portable recordings, SDF worlds and browser captures](released-jaw-tail-replays.tar.xz)

Recreate the left-jaw recording from a clean review checkout with a fresh output directory.

```sh
export RUSTC_WRAPPER=
mkdir -p target/jaw-tail-study
cargo run --locked --release --no-default-features -p gz-sim-cli --example serial_impact_inertia -- \
  --divisor 1.75 --gap-um 35 --guide-hz 100000 --automatic --substeps 64 \
  --impact-iterations 8 --impact-pgs 8 --early-face-step-ns 25 --early-face-max-steps 4096 \
  --early-face-tail-step-ns 125 --early-face-guard-ns 2000 --audit-first-strike \
  --write-world target/jaw-tail-study/left.sdf --replay target/jaw-tail-study/left
cargo run --locked --release --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  --server --replay --web-viewer 11426 target/jaw-tail-study/left
```

Open the replay with the local gz-web-rs frontend on port 8082. Add `--right` and use `--gap-um 50` for the reflected 50 µm trial. For the high-resolution comparison, use a 1 ns early step, 100,000-step budget, 25 ns tail and 3,000 ns guard. Replace audit/replay/world flags with `--ticks 1 --timing-runs 20` to measure cost. The independent continuous first-tick reference uses `--divisor 1.75 --gap-um 35 --guide-hz 100000 --substeps 1000000 --ticks 1 --impact-pulses`. [Open the reproduced recording](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11426%2F)

Changed executable Rust against `108ef65` exceeded 95% line and region coverage in each of the four affected crates. Rapier covered 75/77 lines and 131/133 regions, the multifidelity facade 13/13 and 11/11, the scene crate 104/104 and 184/187, and the CLI example 12/12 and 23/23. The measurement file records the exact instrumented binaries and profile hashes.
