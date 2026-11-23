# Local physics audit tools

These standalone Rust packages check local Gazebo physics experiments against their recorded source measurements. The numerical tools independently recompute physical quantities and validate generated figures. The browser tool checks how gz-web-rs presents real replay archives. Run the commands from the simulator repository root with the corresponding experiment data. The [replay gallery](../../docs/MULTIFIDELITY_EIGHT_HOUR_REPLAYS_2026-11-21.md) links each run and its evidence.

## Numerical readback

Build all numerical checkers once. Each binary in [numerics/src/bin](numerics/src/bin) has the name of its Rust source file.

```sh
export RUSTC_WRAPPER=
cargo build --locked --offline --release --manifest-path tools/physics-audit/numerics/Cargo.toml \
  --bins --target-dir target/physics-audit-tools
check=target/physics-audit-tools/release
"$check/native_impact" artifacts/multifidelity-native-impact
"$check/impact_certification" artifacts/multifidelity-impact-certification
```

For the high-resolution surveys, export the numerical summaries and figures before checking them. The checker reads the original completed fixture files, recomputes the metrics and validates the plotted cells at their physical coordinates. Each survey command also works on a partially populated cases directory.

```sh
cargo build --locked --offline --release --no-default-features -p gz-sim-cli --example landing_audit_view
view=target/release/examples/landing_audit_view
primary=artifacts/multifidelity-primary-survey
relaxed=artifacts/multifidelity-relaxation-survey
"$view" primary-survey "$primary" "$primary/report"
"$check/primary_survey" "$primary" "$primary/report"
"$view" relaxation-survey "$relaxed" "$relaxed/report" --default-survey "$primary"
"$check/relaxation_survey" "$relaxed" "$relaxed/report" "$primary"
```

To verify the kinematic reconstruction independently, the calibration program recreates the first indexed native donor at every physical offset and each available stroke. It saves all checked native poses and the largest reconstruction errors. The program creates the supplied scratch directory, recreates the trajectories there and removes the directory after successful verification.

```sh
cargo build --locked --offline --release --no-default-features -p gz-sim-cli \
  --example support_reacquisition --features landing-audit
"$check/cut_pose_calibration" target/release/examples/support_reacquisition "$primary" \
  target/calibration-native-cuts "$relaxed/checks/cut-poses.json"
```

The relaxation report also checks the two candidate trajectories that share the complete first eight-pass interval and diverge only in subsequent relaxation. It writes `relaxation-continuation.json`, a native receipt table and an SVG. The independent checker verifies full prefix equality against the raw serialized observations before reclassifying the held-out cut. A distinct cost benchmark times the ordinary native API and checks the resulting states against the observed histories. [Decision design and replay](../../artifacts/multifidelity-relaxation-survey/continuation/README.md) · [Pinned CPU benchmark and commands](../../artifacts/multifidelity-relaxation-survey/continuation/timing/README.md)

The retained-load study regenerates all four native trajectories on the timing cohort, verifies their archived measurements, checks the ordinary-API masks and exports the selected extra-load replay.

```sh
timing="$relaxed/continuation/timing"
"$check/continuation_loads" target/release/examples/support_reacquisition "$relaxed" "$primary" \
    "$timing/cpu-7/design.json" target/continuation-native-rerun target/continuation-load-study
"$check/continuation_loads" --check "$timing/cpu-7" target/continuation-load-study
```

The continuation-duration campaign uses six ordinary public-receipt treatments and independently timed permutations. The numerical analysis joins incoming states, validates common public histories, records both empirical reference rankings and withholds all speeds at five support offsets.

```sh
spans=artifacts/multifidelity-continuation-span
target/release/examples/support_reacquisition --continuation-survey --survey-count 1250 "$spans"
target/release/examples/landing_audit_view continuation-spans "$spans" "$relaxed" "$primary" "$spans/report"
target/release/examples/support_reacquisition --span-replay INDEX --survey-stroke STROKE "$spans"
taskset -c 7 target/release/examples/support_reacquisition --span-cost "$spans" "$spans/timing/cpu-7"
"$check/continuation_span_cost" "$spans" "$spans/timing" "$spans/timing/cpu-7" "$spans/timing/cpu-8"
```

The finite support-edge pilot compares its measured impulse vector with the independent geometric jump and reconstructs postimpact COM, normal and tangent differences from its three verified traces. `finite_policy_case FINITE_DIR PRIMARY_DIR INDEX STROKE` writes the per-case PNG source and readback alongside the replay.

The moving-support feature study joins an independent finite-geometry prediction, its actual projected native impulse, the eight-call relaxation field and two 4,096-call trajectories. The checker reconstructs the geometry classes, clips the projected support polygons directly and evaluates the held-out decisions from the raw histories. The replay producer reexecutes the archived donor through the two native drivers.

```sh
finite=artifacts/multifidelity-finite-policy-refined
field=artifacts/multifidelity-contact-feature-relaxation/report
target/release/examples/landing_audit_view feature-relaxation "$finite" "$relaxed" "$primary" "$field"
"$check/feature_relaxation" "$finite" "$relaxed" "$primary" "$field"
driver=target/release/examples/support_reacquisition
"$check/feature_replay" "$field" "$finite" "$relaxed" "$driver" "$driver" target/feature-native-replay
```

The three-stroke reader reconstructs assembly COM and spin from the two real links, return times, subsequent normal reactions, terminal footprint and the first observed trajectory separation at several physical distances. It writes a field plot and the predeclared maximum, with unavailable cases marked.

```sh
"$check/three_stroke_fields" propagation artifacts/multifidelity-primary-propagation target/propagation-field
"$check/three_stroke_fields" convergence artifacts/multifidelity-three-stroke-convergence target/convergence-field
```

The landing-persistence study compares a better or worse first 13 ms with the final outcome after three returns. The Rust driver reexecutes the two continuous fine references and all three onset interventions on the same 512-call suffix used by the source campaigns. It checks first and later full states, terminal values and return events before writing the selected replay. The independent reader merges the published donor files and explicitly distinguishes pending 8,192- and 32,768-call references from off-grid cases.

```sh
persist=artifacts/multifidelity-landing-persistence
target/release/examples/support_reacquisition --persistence-prefix --survey-count 250 "$persist"
"$check/landing_persistence" artifacts "$persist/report"
target/release/examples/support_reacquisition --persistence-replay 10 "$persist"
"$check/landing_persistence_case" "$persist/case-010"
```

The secondary restart crosses the actual incoming histories from the persistence replay with the 512- and 4,096-call grids. It reads the native solver state, checks a same-grid twin and measures the separate effect of discarding collision geometry. The input directory and the restart output share the other source archives as siblings.

```sh
restart=artifacts/multifidelity-secondary-restart-conditioned
target/release/examples/support_reacquisition --secondary-restart 10 "$restart"
"$check/secondary_restart" "$restart/case-010"
```

The point study reads the real native solver rows and checks each signed normal and tangential receipt against whole-assembly linear and angular momentum. A separate field replays 50 predeclared second-landings. Each case includes a fully uninterrupted twin and an exact incoming state. The row-order run adds no-op hooks and two controlled permutations on both retained and rebuilt graphs.

```sh
driver=target/release/examples/support_reacquisition
points=artifacts/multifidelity-secondary-contact-points
cache=artifacts/multifidelity-secondary-cache-field
order=artifacts/multifidelity-secondary-order-field
"$driver" --secondary-contact-points 10 "$points"
"$check/secondary_points" "$points/case-010"
"$driver" --secondary-cache-cohort --survey-start 0 --survey-count 50 "$cache"
"$check/secondary_cache_field" "$cache" "$cache/report"
"$driver" --secondary-order-cohort --survey-start 0 --survey-count 50 "$order"
"$check/secondary_cache_field" "$order" "$order/report"
```

The original fine-grid archive and field outputs live in sibling directories. Both commands atomically publish each fixture, can process disjoint ranges, and preserve censored cases. The row-order checker revalidates the unmodified branch of both graphs against the collision-graph field before comparing the permutations. [Fixture coordinates and selection rule](../../artifacts/multifidelity-secondary-cache-field/README.md) · [Row-order controls](../../artifacts/multifidelity-secondary-order-field/README.md)

The next factorials use the same 50 second-landings with original or experimentally permuted native rows. Each returning-box support-plane proof is separately checked by enumerating both boxes’ vertices. All campaign paths share the same parent.

```sh
factor=artifacts/multifidelity-secondary-relaxation
natural=artifacts/multifidelity-secondary-natural
fixed=artifacts/multifidelity-secondary-return-features
travel=artifacts/multifidelity-secondary-travel-features
"$driver" --secondary-relaxation-field --survey-count 50 "$factor"
"$driver" --secondary-natural-field --survey-count 50 "$natural"
"$check/secondary_relaxation_field" "$factor" "$factor/report"
"$check/secondary_relaxation_field" --natural "$natural" "$natural/report"
"$check/secondary_box_normals" --field "$natural" artifacts/multifidelity-secondary-feature-normals/report
"$driver" --secondary-return-feature-field --survey-count 50 "$fixed"
"$check/secondary_relaxation_field" --features "$fixed" "$fixed/report"
"$driver" --secondary-travel-feature-field --survey-count 50 "$travel"
"$check/secondary_relaxation_field" --travel "$travel" "$travel/report"
"$driver" --secondary-return-feature-replay 34 "$fixed"
```

For the two timing repeats, run `--secondary-return-feature-cost 1` and `--secondary-return-feature-cost 2` in the same output directory, then read it with `secondary_relaxation_field --features-cost COST_DIR COST_DIR/report`. The travel-scaled variants use `--secondary-travel-feature-cost` and `--travel-cost`. Every timed run checks its physical result against the observed untimed twin before publishing.

## Chromium WebGPU readback

Start the phase’s gz-sim-rs replay server and the gz-web-rs frontend, then pass the native source directory, exported frame index, capture directory and replay port. The tool opens the real frontend, plays and resets the recording, compares every rendered frame with the independent source, and saves the requested event screenshots and a JSON readback.

```sh
cargo build --locked --offline --release --manifest-path tools/physics-audit/browser/Cargo.toml \
  --target-dir target/support-loss-view-check
audit=artifacts/multifidelity-native-impact
target/support-loss-view-check/release/rocking-foot-viewer-check \
  "$audit" "$audit/view/frames.json" "$audit/view" 9481
```

The browser uses the local X11 display. Set `GZWEB_CHROME_BIN` to select a Chromium executable and `GZWEB_FRONTEND_PORT` to select the loopback HTTP port. Their defaults are `/opt/google/chrome/chrome` and `8082`. For a standard CLI archive the source argument is the recording directory. The phase guide describes the source and replay setup for that run.

## Material reaction on the second support landing

The fifty original second landings have independent finite-point onset and continuous material responses. Recheck public signed reactions against the qualified physical interval, along with the work, momentum, finite separating planes and all incoming conditions.

```sh
target/physics-audit-tools/release/secondary_material artifacts/multifidelity-secondary-material-response artifacts/multifidelity-secondary-material-response/report
target/physics-audit-tools/release/secondary_relaxation_field --registered artifacts/multifidelity-registered-return-features artifacts/multifidelity-registered-return-features/report
target/physics-audit-tools/release/secondary_material_policy artifacts/multifidelity-secondary-material-policy artifacts/multifidelity-secondary-material-policy/report
```

The certified survey additionally joins the identical pinned executable with and without the face certificate. Its full field retains the measured collision graph before constructing both retained and rebuilt branches. The ordinary Bevy SDF field reconstructs the same source motion in a fresh world. [Fixed budget measurements](../../artifacts/multifidelity-secondary-material-policy/report/README.md) · [Named SDF geometry](../../artifacts/multifidelity-registered-return-features/README.md)
