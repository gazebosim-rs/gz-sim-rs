<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Reproducing the native Light pose observations

[The reference program](../reference/light_pose_reference.cc) is an exact copy of
the successful native twelve-Light/lightless probe. [Its recorded inputs](../reference/light_pose_reference.json)
bind the Sim/SDFormat commits, immutable image, qualified manifest, GCC/compiler
hash, source, libraries, exact compiler/run argv and whitelisted environment.
`actual_recorded` contains observed commands and results. The separate
`declared_mount_projection` maps checked-in or temporary inputs to their original
container paths; it does not rewrite commands or assert a new execution result.

The native program receives real first-step Pose_V messages, including a real
empty message for the lightless world. It checks parent links, identity local
Visual poses, resolved local/world Light poses and Visual-before-Light grouping.
IDs and within-family traversal in the JSON are observations of that pinned run,
not portable ordering promises. Received protobuf serialization is not a network
packet capture. Publisher/subscriber are in one native process using Zenoh.

The synthetic checked-in XML fixture adds only the OSRF/SPDX notice after its XML
declaration. Its source lines move by two; Pose_V has no source URI/line field.
For exact historical input reproduction, the recipe removes those two exact
notice lines from a temporary copy, verifies the original input hash, and derives
the lightless control by removing its twelve complete single-line Light records.
The licensed checked-in fixture stays unchanged. Both materialized XML hashes
were verified during packaging; this packaging did not recompile or rerun the
native probe. Raw original logs, ELF, header inventory, maps and received messages
remain in ignored artifacts.

Use the already-qualified local `cb921...` image described in
[the existing reference recipe](ZENOH_DART_REFERENCE.md). No SDK or image rebuild
is required. Run from an integrated checkout containing the notice-bearing XML
fixture with the exact hash recorded in the JSON. The temporary mount aliases
below retain the historical source basename and input/output paths.

```bash
export LIGHT_POSE_INPUTS="$(mktemp -d)"
export LIGHT_POSE_OUTPUTS="$(mktemp -d)"
python3 - <<'PY'
from pathlib import Path
import hashlib, json, os, shutil
meta = Path('reference/light_pose_reference.json')
data = json.loads(meta.read_text())
inputs = Path(os.environ['LIGHT_POSE_INPUTS'])
sha = lambda value: hashlib.sha256(value).hexdigest()
source = Path(data['source']['path']).read_bytes()
assert sha(source) == data['source']['sha256']
inputs.joinpath('light_wire_probe.cc').write_bytes(source)
shutil.copyfile(meta, inputs / 'reference.json')
m = data['input_materialization']
licensed = Path(m['licensed_fixture']).read_bytes()
assert sha(licensed) == m['licensed_fixture_sha256']
lines = licensed.splitlines(keepends=True)
assert lines[1:3] == [value.encode() for value in m['notice_exact']]
world = b''.join(lines[:1] + lines[3:])
assert sha(world) == m['historical_world_sha256']
assert sum(line.startswith(b'<light ') for line in world.splitlines()) == 12
empty = b''.join(line for line in world.splitlines(keepends=True)
                 if not line.startswith(b'<light '))
assert sha(empty) == m['historical_empty_sha256']
inputs.joinpath('world.sdf').write_bytes(world)
inputs.joinpath('empty.sdf').write_bytes(empty)
PY

docker run -i --pull=never --rm --network=none --read-only \
  --tmpfs /tmp:rw,size=128m \
  -v "$LIGHT_POSE_INPUTS:/probe-source:ro" \
  -v "$LIGHT_POSE_OUTPUTS:/evidence:rw" \
  --entrypoint /usr/bin/python3 \
  sha256:cb921fdafe5763b3c9148326dfd93b1cb291099f648702c4e27cc009137037ae - <<'PY'
from pathlib import Path
import hashlib, json, shutil, subprocess
out = Path('/evidence')
data = json.loads(Path('/probe-source/reference.json').read_text())
sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
assert sha('/reference/zenoh-dart-environment.json') == data['pins']['qualified_manifest_sha256']
assert sha('/probe-source/light_wire_probe.cc') == data['source']['sha256']
assert sha(data['compiler']['path']) == data['compiler']['sha256']
for path, checksum in data['source']['native_source_sha256'].items():
    assert sha(path) == checksum
record = data['actual_recorded']
assert shutil.disk_usage(out).free >= 1342177280
for path, checksum in record['mapped_libraries_sha256'].items():
    assert sha(path) == checksum
with (out / 'compile.stdout').open('wb') as stdout, (out / 'compile.stderr').open('wb') as stderr:
    subprocess.run(record['compile_argv'], env=record['compile_environment'],
                   cwd=out, stdout=stdout, stderr=stderr, check=True, timeout=180)
assert not (out / 'compile.stderr').read_bytes()
assert sha(out / 'probe') == record['elf_sha256']
for name, case in record['cases'].items():
    assert shutil.disk_usage(out).free >= 1342177280
    (out / name).mkdir()
    env = record['runtime_environment_base'] | case['environment_overrides']
    with (out / (name + '.stdout')).open('wb') as stdout, (out / (name + '.stderr')).open('wb') as stderr:
        subprocess.run(case['argv'], env=env, cwd=out, stdout=stdout,
                       stderr=stderr, check=True, timeout=15)
    assert (out / (name + '.stdout')).read_text() == case['stdout']
    assert not (out / (name + '.stderr')).read_bytes()
    assert sha(out / name / 'pose_v.pb') == case['wire_sha256']
    assert sha(out / name / 'observations.jsonl') == case['observations_sha256']
print('Selected native Light reference reproduced; outputs remain under /evidence')
PY
```

The C++ program retains its five-second receipt wait. A timeout, nonzero process
status, unexpected message or hash mismatch is a failed reproduction; preserve
that attempt separately. This recipe does not qualify Rust admission, native
reset parity, remote interoperability, GUI, light rendering, SDF radiometry,
physical GPUs, B03, original upstream cases or performance.
