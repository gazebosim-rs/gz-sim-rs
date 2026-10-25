# TPE motion kernel

The gz-physics-tpe crate provides a validated, centered Body over shared f64
BodyState values. Construction checks finite values and squared quaternion norm
within 1e-12 of one. The immutable state accessor preserves that invariant;
advance updates position/orientation only after fallible arithmetic completes.
Overflow leaves state unchanged, allowing a smaller-step retry.

The operation retains pinned Model/Link UpdatePose semantics for world-frame
motion: constant linear velocity and a left-multiplied angular increment. Native
Vector3::operator== uses a component tolerance of 1e-3, so both velocity vectors
inside that boundary leave the pose unchanged. Zero-duration and stationary calls
preserve pose bits. This threshold is covered on both sides by native fixtures.

Rotation uses glam. Its squared norm loses tiny nonzero angular increments, while
native Quaternion::Integrate retains their half-angle. A narrow underflow fallback
preserves that missing behavior, including a native 5e-201 quaternion component.
Advanced rotations are renormalized when squared norm drifts beyond 1e-14. Unit
normalization and finite/overflow Result errors are explicit Rust API contracts;
bit-identical arbitrary-magnitude C++ arithmetic is not promised.

Twelve rows from the freshly built pinned TPE library exercise original motion
inputs, nonidentity orientation, integer-duration boundaries, stationary thresholds
and tiny angles. tests/native_motion.cc reproduces them; fixture provenance records
source/generator/library/header/binary hashes. Original Model.BasicAPI permits
1e-3 component error; the motion sample uses a stronger 1e-6 check, supplementary
rows use 1e-12, and the tiny component is checked exactly. Names, identities,
static flags and the rest of that original case remain unmigrated.

The simulator still selects Rapier. Bevy body ownership, TPE installation, contact
queries, removal/reset integration, commands and the full original ledger remain
open. D23 separately awaits the owner's decision on native static-model movement
and rotated-link velocity conversion; this kernel does not select those policies.
The kernel acceptance gate passed 106 Rust checks and 9 tooling checks, retaining all six
real C++ server scenarios. Motion coverage is 48/48 lines, 73/73 regions and 46/46
changed executable lines. Independent source/assertion/fixture review and the exact staged/hosted gates
passed at [2962f1d](https://github.com/gazebosim-rs/gz-sim-rs/commit/2962f1d07b09d4b6abf0ae8be143b5eb1cff7d28).
PHY-01, PHY-02 and P2 are not complete.

## Measured operation

The reproducible example and native_motion_benchmark.cc each perform 1000 body
poses × 1000 updates at 1 ms, excluding construction. The runner alternates native
and Rust on one allowed CPU, discards one warmup pair and records seven pairs.
Analytic final pose, operation count and aggregate checksum are checked. See
planning/benchmarks/tpe-motion-kernel.json for raw samples and exact hashes.

The initial validating free function measured about 65 ns/update. Inlining, bounded
renormalization and moving validation into the private Body construction boundary
reduced the Rust operation to about 23 ns/update, while retaining every existing
assertion and adding near-unit/transactional-retry cases. The final report records
26.64 ns native and 22.71 ns Rust medians (1.17× ratio); these are exploratory
shared-host measurements.
Native shared-library and Rust static packaging/compilers differ. This is not an
ECS/contact/sensor/transport benchmark or a whole-simulator speedup/P2 report.

Reproduce against an empty build directory of pinned gz-physics
b44cd0d0ea6606b6c98320c0c87a5670d4fc6be6, with its required dependencies installed:

```sh
cmake -S "$GZ_PHYSICS_SOURCE" -B "$TPE_BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$DEPENDENCY_PREFIX" -DBUILD_TESTING=ON -DSKIP_dartsim=ON -DSKIP_mujoco=ON -DSKIP_bullet=ON -DSKIP_bullet-featherstone=ON
cmake --build "$TPE_BUILD" --target gz-physics-tpelib
c++ -std=c++17 -O3 -Wall -Wextra -Werror -I"$GZ_PHYSICS_SOURCE/tpe/lib/src" -I"$TPE_BUILD/include" crates/gz-physics-tpe/tests/native_motion_benchmark.cc -L"$TPE_BUILD/lib" -lgz-physics-tpelib $(pkg-config --cflags --libs gz-math gz-utils) -o /tmp/tpe-motion-native
cargo build --locked --release -p gz-physics-tpe --example motion_benchmark
python3 scripts/benchmark-tpe-motion.py --native /tmp/tpe-motion-native --rust target/release/examples/motion_benchmark --reference-build "$TPE_BUILD" --dependency-lib-dir "$DEPENDENCY_PREFIX/lib" --output /tmp/tpe-motion-results.json
```

Installed dependency qualification remains explicit; these results do not close
the complete reproduced-reference or representative robotics performance work.
