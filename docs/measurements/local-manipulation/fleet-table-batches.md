<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Dense Bevy model and body publication

In the staged Ubuntu build of `84569c948e66`, the first physics commit for 9,000 models stopped in Bevy’s query fetch. The native thread pool split component data into 2,048 rows while the query still supplied 9,000 entity rows. Fetching a whole table and then scheduling disjoint slices of its component data keeps the identities and physics values aligned. Batches scale with pool width and have a 2,048-row minimum. Changing a world pose or velocity continues to mark that component as changed.

The staged regression now publishes all 9,000 independently rotated model frames, replaces and reorders 3,000 links under stable public identities, and compares every transformed frame after reset. A second test publishes 9,001 native Rapier bodies across uneven Bevy tables. Every published pose and twist matches the backend, including the rotating offset from the center of mass to the link origin. All 322 scene library tests passed with debug assertions enabled. The changed functions covered 62/62 executable lines and 187/188 regions before a formatting-only line wrap. Strict Clippy passed.

## Complete local simulation step

Each executable simulated 100,000 moving decks. Four independent process pairs advanced matched blocks on CPUs 24–31. Each trial warmed up both native worlds, then alternated eight blocks of five warm ticks followed by 40 measured ticks per version. The table reports nearest-rank percentiles from 1,280 measured ticks per executable and layout. A second comparison ran four pairs of complete, independently loaded processes per layout. Both sets ran on a shared workstation. [Per-trial measurements](fleet-table-batches.json)

| Payloads | Previous p50 / p95 | Corrected p50 / p95 |
|---|---:|---:|
| 1,000 | 6.161 / 8.122 ms | 6.051 / 8.305 ms |
| 0 | 7.654 / 21.234 ms | 6.511 / 21.980 ms |

Median step time improved in both matched-block layouts. Their p95 rose by 0.183 ms and 0.746 ms respectively. The complete-process repetitions gave a lower median of the four p95 values for each layout. This measurement establishes correct parallel publication and leaves the high-latency tail open.

## Replay

A fresh run of the corrected executable recorded two simulated seconds of the 100,000-deck, 1,000-payload fleet. The manifest and all 51 recorded frames are byte-identical to the original CPU fleet recording. The independent trajectory audit compared 459 deck positions, 51 payload poses, 51 stationary reference poses and 50 IMU observations. The gz-web-rs WebGPU renderer independently checked all 561 displayed poses and shapes and rewound to the first frame. The payload advanced 0.493587 m, with a final deck-relative lag of 6.413 mm. [Open this recording in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9892%2F)

The [reproduction commands](fleet-publication-view.md) regenerate the recording. The debug-build regression runs with the following command on a machine with multiple CPU threads.

```sh
RUSTC_WRAPPER= cargo test --locked -p gz-sim-scene --lib canonical_publication_tests
```
