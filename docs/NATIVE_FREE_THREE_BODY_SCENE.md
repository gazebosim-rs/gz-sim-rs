<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Native contact-free three-body Scene reference

`reference/free_three_body_scene.cc` is the exact successful attempt-4 native
observer, not a Rust plugin. The fixture contains the exact two XML strings,
actual 13-field body states and two-field joint states at ticks 1–10, commanded
efforts, opaque observed IDs, and independently resolved native joint frames.
The two declarations produce identical numeric states by name and tick.
No Rust/native quantitative comparison, reset/removal qualification, B03 or
original-case completion is claimed.

The historical Rust materialization hash records extraction provenance only.
A future consumer test must compare the fixture XML directly with its current
`xml(reverse)` output; it must not pin a hash of its own source file. Native IDs
and traversal order are observations, not portable ordering requirements.

Joint `components::Pose` is child-relative, while `ParentEntity` is the model.
Consequently the captured generic ECM `worldPose(joint)` is **not** the semantic
joint frame. The fixture separately records actual native
`sdf::Joint::SemanticPose().Resolve(..., "__model__")`, default child-frame
resolution, and child model pose multiplied by child-local joint pose. For this
identity model, model-frame joint poses are world poses: first `(0,0,-1)`,
second origin. See pinned `SdfEntityCreator.cc:866` and
`Physics.cc:2033–2052`; `Physics.cc:2641–2660` applies `JointForceCmd` to
`SetForce`. Native DOM line 1 and empty file path come from `LoadSdfString`;
they do not establish equivalence with Rust `Source` fields.

The metadata records the immutable qualified cb921 image, source pins, manifest,
compiler, exact container compile/run argv, whitelisted environments, and all
157 actual mapped-file hashes. Only private **host mount paths** in the recorded
outer Docker argv are projected to named variables. Raw 852 compiler dependency
records, process maps, and author attempts remain ignored evidence. The observed
ELF hash binds that successful compilation; independently reproduced ELF bytes
may differ with compilation paths or tool output metadata.

Reproduce using the already-qualified local image and existing SDK. No SDK or
image build is required. From the repository root, run the following commands.
This recipe materializes the exact fixture XML and mounts the exact C++ bytes
at their recorded container paths. Its stdin Python and read-only metadata mount
replace the historical ignored runner; compile/run argv and environments stay
as recorded. Outputs go to a fresh temporary directory.

```bash
reference_task_dir="$(mktemp -d /tmp/free-three-body-reference.XXXXXX)"
mkdir "$reference_task_dir/source" "$reference_task_dir/output"
python3 - "$PWD" "$reference_task_dir/source" <<'PY'
from pathlib import Path
import hashlib, json, sys
repo, source = map(Path, sys.argv[1:])
meta = json.loads((repo / 'reference/free_three_body_scene.json').read_text())
fixture_path = repo / 'tests/fixtures/free_three_body_scene.json'
digest = lambda b: hashlib.sha256(b).hexdigest()
assert digest(fixture_path.read_bytes()) == meta['fixture_sha256']
cpp = (repo / 'reference/free_three_body_scene.cc').read_bytes()
assert digest(cpp) == meta['cpp_sha256']
(source / 'probe.cc').write_bytes(cpp)
for case in json.loads(fixture_path.read_text())['cases']:
    raw = case['xml'].encode()
    assert digest(raw) == case['xml_sha256']
    (source / (case['name'] + '.sdf')).write_bytes(raw)
PY
docker run --pull=never --rm -i --network=none --read-only \
  --tmpfs /tmp:rw,size=128m \
  -v "$reference_task_dir/source:/probe-source:ro" \
  -v "$reference_task_dir/output:/evidence:rw" \
  -v "$PWD/reference/free_three_body_scene.json:/input/reference.json:ro" \
  --entrypoint /usr/bin/python3 \
  sha256:cb921fdafe5763b3c9148326dfd93b1cb291099f648702c4e27cc009137037ae - <<'PY'
from pathlib import Path
import hashlib, json, os, subprocess
meta = json.loads(Path('/input/reference.json').read_text())
digest = lambda p: hashlib.file_digest(open(p, 'rb'), 'sha256').hexdigest()
assert digest(meta['manifest']['path']) == meta['manifest']['sha256']
assert digest(meta['compiler']['path']) == meta['compiler']['sha256']
assert digest('/probe-source/probe.cc') == meta['cpp_sha256']
for name in ['compile', 'forward', 'reverse']:
    assert os.statvfs('/evidence').f_bavail * os.statvfs('/evidence').f_frsize >= 1342177280
    command = meta['actual_recorded_commands'][name]
    Path('/evidence/' + name + '-command.json').write_text(json.dumps(command, indent=2))
    with open('/evidence/' + name + '.stdout', 'wb') as out, open('/evidence/' + name + '.stderr', 'wb') as err:
        subprocess.run(command['argv'], env=command['environment'], cwd='/evidence',
                       stdin=subprocess.DEVNULL, stdout=out, stderr=err,
                       timeout=180 if name == 'compile' else 30, check=True)
    if name != 'compile':
        mapped = {}
        for line in Path('/evidence/' + name + '.state.maps').read_text().splitlines():
            fields = line.split(maxsplit=5)
            if len(fields) == 6 and fields[5].startswith('/'):
                path = Path(fields[5]).resolve()
                if path.is_file() and str(path) != '/evidence/probe':
                    mapped[str(path)] = digest(path)
        assert mapped == meta['qualified_actual_mapped_files_sha256']
print('Native observations collected; no Rust comparison performed')
PY
```

Inspect `forward.state`, `reverse.state`, and their `.semantic` companions.
`BODY` rows contain tick, name, observed ID, and the fixture's `body_fields`.
`JOINT` rows contain tick, name, observed ID, position and rate. `DOM_JOINT`
contains name, raw pose (7), raw relative frame, resolved model pose (7), resolved
child-local pose (7), composed model pose (7), native line and path. Quaternion
order is XYZW throughout. Efforts `.7` and `-.2` are reissued each PreUpdate.
Failed author attempts remain failures: one used nonexistent component header
names; another attempted to stream an optional source-line value directly.
Neither failure changed the inputs, numeric limits, commands or runtime bounds.
