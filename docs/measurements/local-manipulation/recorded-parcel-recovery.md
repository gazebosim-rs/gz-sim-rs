# Replaying a parcel edit and repairing a copied recording

The operator tried to reposition a jointed gripper and then launched its free 0.2 kg parcel onto the receiving shelf. The live CLI refused the gripper edit, kept its links in place and accepted the parcel edit at the same paused simulation time. The recording preserves all three zero-time frames, including both command outcomes. A fresh player showed the same parcel landing 180 ms later. [Watch the native replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10083%2F) · [Exact frames, profile identities and coverage](recorded-parcel-recovery.json)

| Measured quantity | Result |
| --- | ---: |
| Accepted parcel position and vertical velocity | (0, 460, 675) mm and −20 mm/s |
| Height after the first 1 ms solve | 674.974482 mm |
| First-solve error against gravitational free fall | 0.613 µm |
| Shelf top plus parcel half-height | 615.000 mm |
| Parcel center at 180 ms | 614.980275 mm |
| gz-web-rs checks | 12 frames, 240 poses and shapes, 12 catalogs, rewind to 0 ms |

A separate native capture dropped a parcel carrying a mesh marker. With the original SDF and OBJ removed, the test successively restored a missing manifest, a partial manifest, a missing mesh, a changed mesh of the original length and a shortened frame stream. Each attempt identified the offending archive file and exited before publishing. Restoring the original bytes reopened the recording, served the exact captured GLB, reproduced the landing and allowed a rewind after an end-of-recording resume request was refused. Explicit solver selection, including `--physics-engine rapier`, now fails during playback argument parsing. Solver selection still applies when recording new physics.

## Release coverage

The frozen release was `522a03f`. Each profile supplied its own simulator executable and generated 13 additional raw profiles. Every raw profile contained its simulator's ELF build ID. The renderer's original 807 objects reproduced the counts for all 41 owned CLI source files before the new observations were merged. The before/after exports retained identical denominators and the same pre-existing LLVM mismatch diagnostic.

| Frozen CLI scope | Lines before → after | Regions before → after |
| --- | ---: | ---: |
| Complete renderer | 7,965 → 7,976 / 8,470 | 13,674 → 13,682 / 14,724 |
| Retained headless objects | 7,905 → 7,916 / 8,470 | 13,605 → 13,613 / 14,723 |

The original headless run covered 7,965 lines and 13,673 regions. Its retained 814 objects reproduce 40 of 41 per-file baselines. Six supplemental executables from the original 820-object run were unavailable at their recorded digests. Their omission removes 60 covered lines and 68 covered regions in `live/convoy.rs`. The headless comparison above uses the same 814 objects on both sides. The whole-crate 95% gate remains open in both profiles.

On candidate `c579c1e`, changed production code covered **36/36 mapped lines and 64/65 regions**. Changed acceptance code covered 314/320 lines and 640/648 regions. All 157 CLI library tests, 18 command tests, 19 local physics journal tests and five focused native recorder/player tests passed. Strict CLI-only all-target Clippy passed.

## Run the parcel edit

From the repository root, choose a new output directory and a free viewer port. The first command launches the real server, performs both edits, waits for the last physics step, and checks a fresh playback process.

```sh
GZ_RECORDED_PARCEL_EDIT="$PWD/target/recorded-parcel-demo" RUSTC_WRAPPER= \
  cargo test --locked --release --no-default-features -p gz-sim-cli --test native \
  a_refused_robot_edit_and_an_accepted_parcel_launch_replay_at_their_exact_cuts
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli --bin gz-sim-rs -- \
  target/recorded-parcel-demo --server --replay --web-viewer 9002
```

With gz-web-rs served on port 8082, open the local viewer and press play. The measurement receipt lists the first three frames separately because they share simulation time zero. [View the reproduced recording on port 9002](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9002%2F)
