# TPE model contacts

The selected TPE path reports one overlap point per model pair through the
Contacts resource. Each pair contains current Bevy model Entities and a world-frame
point. Resolve EntityId in the same world when a stable simulator identity is
needed. Query order is unspecified. Contact reporting supplies no collision
response, normal, depth, force, joint or contact-sensor transport message.

The loader retains a local bounding box for each supported centered box/sphere.
It transforms link boxes into model coordinates, merges them, then transforms the
combined box into world coordinates. This matches pinned TPE's model-level bounds,
including enlargement from rotating a sphere's local cube or a merged model box.
Computing tight world bounds for each primitive would produce different contacts.

Parry 0.31.1 supplies Aabb transforms, merging, intersection and a dynamic Bvh.
There is no port of AABBTree or a second scene/entity registry. The spatial index
retains the current generational Entity alongside each leaf index, removes absent
models and filters old, larger tree envelopes against current precise boxes.
Static models do not initiate queries; dynamic/static pairs remain eligible.
Dynamic/dynamic pairs appear once. Pair masks intersect with bitwise AND.
Loaded models currently use the all-bits mask; explicit Surface XML is still
rejected. Validated ModelBounds replacement exposes the low-level mask policy.

Contacts recompute in PhysicsCommit and after MotionReset. The immutable scene
blueprint retains link/model bounds, so loaded reset reconstructs contacts for the
restored entities and original public IDs. Runtime physics bindings remain outside
that blueprint's supported reset scope. Core reset only restores surviving Motion
components. The previous published frame remains intact if collection fails.

The overlap point uses standard scalar f64::midpoint per coordinate. Parry's glam
vector midpoint can overflow a sum at finite world coordinates such as 1e308;
independent review caught that case. The Rust regression and fresh native probe
now require the finite native point exactly. Nonfinite transformed bounds report
a typed ContactError through StepFault and invalidate the run.

ContactLimit defaults to 1,000,000 pairs per collection. Exceeding the configured
limit fails explicitly before replacing the last complete frame; output is never
silently truncated. Two output buffers reuse allocated capacity. Geometry/index
storage scales with registered models; there is no independent entity allocator.

Eleven freshly generated native rows cover overlap, touching, separation, static
filtering, masks, rotated boxes/spheres, model-local merging and large coordinates.
Source and linked-library hashes are in planning/reference/tpe-contacts.json;
tests/native_tpe_contacts.cc regenerates the rows against the pinned TPE build.
The full original CheckStaticCollisionFiltering assertion is faithfully mapped.
The broader CheckCollisions geometry case and eight-corner helper coverage remain
open. All nine native TPE targets / 28 original cases were rerun without changes.
Installed dependency qualification still applies.

## Measured step/contact work

The reproducible Rust/C++ programs each advance 1,000 unit-box models for 500 one-ms
steps: half static, dynamic velocity 0.01 m/s, zero angular velocity and 250 dynamic
overlap pairs per step. Construction and two zero-duration priming steps are
excluded for both programs. Priming initializes geometry/scheduler/index/output
state without changing poses or simulated time. Both programs validate every final
model pose and every contact pair/point outside timing; the runner checks analytic
aggregate state as well. Neither program includes sensors, transport or scene-pose
publication. See planning/benchmarks/tpe-contacts.json for the alternating CPU-pinned
samples, one discarded warmup pair, seven measured pairs, build/source/library
hashes and actual native compiler flags.

The recorded medians are 1283.58 ms native and 87.63 ms Rust (14.65× ratio).
These are author measurements on a shared host. Native dynamic-library versus
Rust static packaging and compiler/ISA settings differ; native profiling is disabled. Independent review
must assess work equivalence and evidence; no whole-simulator speedup, representative
robotics budget or full P2 acceptance follows from this narrow comparison.

With the pinned native source and fresh Release build from TPE_MOTION.md:

```sh
cargo build --locked --release -p gz-physics-tpe --example contact_benchmark
PKG_CONFIG_PATH="$DEPENDENCY_PREFIX/lib/pkgconfig" python3 scripts/benchmark-tpe-contacts.py --native /tmp/tpe-contact-native --rust target/release/examples/contact_benchmark --reference-source "$GZ_PHYSICS_SOURCE" --reference-build "$TPE_BUILD" --dependency-lib-dir "$DEPENDENCY_PREFIX/lib" --output /tmp/tpe-contact-results.json
```

The runner compiles the C++ benchmark outside timing, records the actual command,
and verifies that it loads the requested TPE library. It requires the qualified
math/utils/common-graphics development packages, not an installed TPE SDK.
