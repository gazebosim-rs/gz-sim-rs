# Full Rapier crate coverage

The Rapier gate passed with **27,821/29,051 lines (95.77%) and 45,798/48,149 regions (95.12%)**. All 114 owned source files were mapped. The 660 test executables ran 2,440 passing tests and emitted 672 matching native profiles. Contact geometry, prescribed motion and native joint behavior are exercised through both Rapier’s own tests and Gazebo clients of that library. Four CLI integration targets have separate native, camera or GPU runtime profiles. The local robot foundations example is also tracked separately. At capture, it had completed 73 of 74 tests and was still running its pathological 30 mm cloth test. The Rapier receipt includes every Rapier crate test executable.

This phase tests a tilted foot striking the ceiling and a rising floor within one native interval. Solving both strikes together gives the same final state and impulses as solving them separately. A welded foot’s four-corner response matches its independently calculated combined mass and inertia. Additional tests exercise retained forces after failed contact registration, paused solves, moving-belt restart, and rejection of internally inconsistent snapshots with valid checksums.

The measured snapshot is in [rapier-crate-coverage.json](rapier-crate-coverage.json). Its line and region totals come from one LLVM export, followed by the repository’s `check_coverage` function. That gate requires strictly more than 95% for both metrics and a mapping for every owned source file. The isolated Rapier-only suite and the workspace-wide hosted verification are separate measurements.

## Reproduce the scoped profile

Run from the repository root with Rust 1.97.1, the matching `llvm-tools-preview`, GNU readelf and jq. Use the same native build prerequisites as the headless CI job. A fresh evidence directory keeps unrelated profiles out of the result. All five packages must resolve together.

```bash
set -euo pipefail
mkdir -p artifacts
d=$(mktemp -d "$PWD/artifacts/rapier-coverage.XXXXXXXX")
export RUSTC_WRAPPER= RUSTFLAGS= CARGO_BUILD_JOBS=12
export CARGO_TARGET_DIR="$PWD/target/rapier-local-gate"
export CARGO_PROFILE_TEST_OPT_LEVEL=1 CARGO_PROFILE_TEST_DEBUG=1
export CARGO_PROFILE_TEST_DEBUG_ASSERTIONS=true CARGO_PROFILE_TEST_OVERFLOW_CHECKS=true
mkdir -p "$PWD/artifacts/local-implementation/rapier-gate"
export RUSTC_WORKSPACE_WRAPPER="$PWD/artifacts/local-implementation/rapier-gate/rustc-coverage.py"
install -m755 scripts/rustc-coverage.py "$RUSTC_WORKSPACE_WRAPPER"
export LLVM_PROFILE_FILE="$d/compile-%p-%m.profraw"
llvm="$(rustc --print sysroot)/lib/rustlib/$(rustc -vV | sed -n 's/^host: //p')/bin"
cargo test --locked --offline --all-targets --no-run --message-format=json \
  -p gz-physics-rapier -p gz-physics-multifidelity -p gz-physics-distributed \
  -p gz-sim-scene -p gz-sim-cli \
  --features gz-physics-rapier/landing-audit,gz-physics-rapier/partition-checkpoint,gz-physics-multifidelity/landing-audit,gz-sim-cli/landing-audit \
  > "$d/build.jsonl"
jq -r 'select(.reason=="compiler-artifact" and .executable and .profile.test)
  | select((.target.src_path | contains("/gz-sim-cli/tests/")) and
    (.target.name as $n | ["native","no_native_runtime","rgb_camera","gpu_sensor_transport"] | index($n)) | not)
  | select(.target.name != "local_robot_foundations")
  | [.executable,.manifest_path] | @tsv' "$d/build.jsonl" > "$d/tests.tsv"
objects=()
while IFS=$'\t' read -r exe manifest; do
  name=$(basename "$exe")
  raw="$d/$name"
  mkdir "$raw"
  (cd "$(dirname "$manifest")" && LLVM_PROFILE_FILE="$raw/%p-%m.profraw" "$exe") > "$raw/run.log" 2>&1
  id=$(readelf -n "$exe" | sed -n 's/.*Build ID: //p')
  matched=0
  for sample in "$raw"/*.profraw; do
    if "$llvm/llvm-profdata" show --binary-ids "$sample" 2>/dev/null | grep -Fx "$id" >/dev/null; then
      printf '%s\n' "$sample" >> "$d/matching-profiles"
      matched=1
    fi
  done
  test "$matched" -eq 1
  objects+=(-object "$exe")
done < "$d/tests.tsv"
"$llvm/llvm-profdata" merge -sparse -f "$d/matching-profiles" -o "$d/merged.profdata"
"$llvm/llvm-cov" export --fatal-warnings --summary-only -num-threads=3 \
  -instr-profile="$d/merged.profdata" "${objects[@]}" \
  -sources $(find crates/gz-physics-rapier/src -name '*.rs' | sort) > "$d/coverage.json"
PYTHONPATH=scripts python3 - "$d/coverage.json" <<'PY'
import json, sys, verify
owned = {"gz-physics-rapier": verify.production_sources()["gz-physics-rapier"]}
with open(sys.argv[1]) as src:
    report, _ = verify.owned_coverage(json.load(src), "", owned)
print(verify.check_coverage(report, owned))
PY
```
