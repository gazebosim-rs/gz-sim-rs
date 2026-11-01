<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Separate Zenoh and DART native reference

This REF-01 prerequisite prepares the native side of B03. The recipe, native
build, bounded smoke and fresh read-only image validation have run as described
below. Independent image/evidence review passed. Recipe publication was accepted
on Main `ce071305` after the complete normal hook, exact hosted run `41298713640`
and independent downloaded source/profile/native audit passed. Those ordinary
hosted profiles verify the selected client and Rust workspace; they do not
repeat the separate local native application build or B03 smoke.
It does not qualify Rust articulation, balanced-suite accuracy or performance.
The existing TPE reference, its validator, immutable image and C3 observations
remain separate. The eight DART-family library mounts used by a Rust release
are not qualification of the native Gazebo DART plugin.

The owned files are `scripts/bootstrap-zenoh-dart.py`,
`scripts/zenoh_dart_probe.cc`, `planning/zenoh-dart-app-lock.json`,
`tests/test_zenoh_dart_build.py` and this document. No application, fixture,
consumer pin, shared bootstrap or benchmark driver changes are required.

The build base is the existing immutable clean DART image
`sha256:4ac74e616369164eaba07149e9b98f7594a33cb3ddbbd6a45e0dd43cb688b1d5`.
It supplies system OSRF DART 6.13.2 and its actual Ubuntu snapshot dependencies.
The lock binds its `dart-environment.json` bytes, all package/compiler identities,
the original SDK manifest and the existing signed DART recipe. The ten OSRF
payloads remain binary dependencies. The archive signer, signed binary/source
indexes and six selected installed DART header/config/library files are checked
with the existing verifier. The recipe neither installs new packages nor claims
that DART was built from its source package.

Inside a fresh disposable container, the recipe builds a new six-repository
Zenoh client at `/zenoh-dart-client`. This uses the existing source pins,
signed-package contract and checksum-bound ROS Zenoh 1.8 vendor archive. The
read-only host client supplies only source caches and that archive; its compiled
libraries are not mounted. The selected client verifies its own packages, tools,
source revisions, ON configuration, vendor and actual negative-zero probe.

Physics, Sensors and Sim are then all rebuilt at their pinned revisions into
`/reference/zenoh-dart-builds`, with `/reference/zenoh-dart-install` as the new
installation. Release and native test compilation stay enabled; GUI, Python,
Bullet/Bullet Featherstone and MuJoCo settings retain the headless contract.
Physics enables dartsim against `/usr/share/dart/cmake`. Compiled TPE artifacts
are not an allowed runtime substitute for the selected DART plugin.
Old `/reference/install`, `/reference/dart-install` and their build directories
are validated before and after, never overwritten. Explicit CMake package
directories, disabled registries and empty compiler/linker launchers prevent
an inherited prefix or launcher from selecting the old application.

Qualification records the actual new caches, installed and build libraries,
compiler dependency files, objects and include flags. Sensor.cc,
SimulationRunner.cc and SceneBroadcaster.cc must include the selected ON client
headers. The new Physics system must include the new Physics API. DART feature
objects must include the system DART config and contact-surface headers. This
provides compiler evidence beyond a claimed environment prefix.

The untimed smoke reads the complete unchanged pinned
`gz-sim/test/worlds/double_pendulum.sdf`, SHA-256
`24e85ff944b5efe2d758fdda543e392f34d59f26297a0a87fafbb41d5a3b16aa`.
Its directional sun, ground and both pendulums stay present. Like the existing
native articulation smoke it runs 100 acquisition steps and 100 observation
steps. It requires seven links, four joints with the exact scoped names, one
sun, finite poses/q/dq, actual motion, integer 1ms steps and final 200ms clock
delivery over the selected native Zenoh transport. No elapsed-time observation
or comparison is produced. A 15-second process timeout kills and reaps only
the probe's own session, preserving stdout and stderr on failure.

Mapped libraries must include the new Sim core, installed Physics system and
DART plugin, new Physics core, same-image client and exact system DART/ODE solver
bytes. Old common-client copies, historical app prefixes, other DART versions
and other physics plugins fail closed. The complete actual mapped DSO hashes
are retained and rechecked. The resulting `zenoh-dart-environment.json` is a
separate profile and can be revalidated without executing its probe again.
An immutable resulting image ID and independent build/runtime review must be
recorded after an actual build; the base image ID is not the resulting identity.

Source and unit checks, without Docker or native execution:

```sh
python3 -m unittest discover -s tests -p test_zenoh_dart_build.py
python3 -m py_compile scripts/bootstrap-zenoh-dart.py
```

To print the exact offline build argv, supply a qualified selected client root:

```sh
python3 scripts/bootstrap-zenoh-dart.py --client-reference /absolute/client-reference --jobs 2
```

Execute that argv only with adequate disk space and after other reference work
has released its build hold. It uses the immutable base, `--pull=never`,
`--network=none`, three read-only inputs and fresh output paths. It leaves the
container in place for evidence. Inside the completed container, with this
source mounted unchanged at the same absolute path:

```sh
python3 /absolute/source/scripts/bootstrap-zenoh-dart.py --validate-only
```

Fourteen focused source/recipe controls currently pass, including actual signed
metadata and negative source/header/module/environment boundaries. The first
unit run exposed inherited compiler launchers in a historical environment helper;
the new profile now explicitly clears them without changing that helper.
The unchanged TPE builder's twelve controls and signed DART recipe's eight
controls also pass. These are source/recipe tests, not native execution counts.

## Actual build and post-failure continuation

The first whole-builder invocation completed the same-image client and all three
application builds and installations, then failed before the probe: DART and
the retained TPE plugin both produce `SimulationFeatures.cc.o.d`. The reviewed
correction limits that witness search to `dartsim`, retaining all source,
header, object and containment checks. The first continuation then found a
second evidence-layout mistake: DART's object resides under a target's `src/`
subdirectory, while `flags.make` resides at the enclosing CMake `.dir` root.
The reviewed correction requires exactly one such target and still rejects
missing or escaping flags. Both failures, their source and their stopped
containers remain separate from the passing continuation.

The successful continuation revalidated the existing client, sources, caches,
objects, flags and libraries without rebuilding the SDK/application outputs.
It checked their actual logged configure/build/install commands against the
reviewed recipe, then executed the remaining compiler-audit/probe/manifest
steps unchanged. Its origin/result records explicitly retain the first failed
whole build and first failed continuation. This is not a fresh whole-bootstrap
success claim.

The unchanged probe compiled with the selected compiler and passed its 100
acquisition plus 100 observation steps. Stdout was exactly:

```text
zenoh dart ready warmup=100 steps=100 links=7 joints=4 light=1 moving=1
```

Stderr was empty, the process exited zero, and the six compiler witnesses and
157 mapped DSOs passed validation. DART's Bullet collision DSO is an upstream
binary dependency in this closure; no Gazebo Bullet physics plugin was loaded.
The new installed inventory contains 1,865 entries. The manifest SHA-256 is
`f770aa807d5aa5f36711e5e8a84cdf2e12afebf9adfeb08fc0601b9e5d49a3c1`.

The resulting local candidate image is
`sha256:cb921fdafe5763b3c9148326dfd93b1cb291099f648702c4e27cc009137037ae`.
It derives from the preserved unqualified build checkpoint and clean DART base;
it does not overwrite either earlier image. A fresh container with read-only
root/source, no network and only temporary validation scratch passed
`--validate-only`, with empty stderr and no probe re-execution. Raw compiler,
source, ELF, maps, command and validation records are retained with the image.

These results do not complete P0 or REF-02/REF-03, execute an upstream native
suite, establish balanced-workload accuracy/performance or qualify Rust light
and articulation admission. The existing Rust limits remain unchanged.

An independent reviewer checked the immutable image in a fresh read-only,
network-isolated container: all three build inventories, 1,865 app entries,
1,255 client entries, six compiler witnesses, 157 mapped DSOs, source/tool/probe
and fixture identities, and the complete base/checkpoint/candidate layer chain
matched. All 138 system DSOs and the package inventory matched the untouched
clean DART base. This was artifact/validator inspection, with no native app
re-execution or build by the reviewer. The retained independent report is under
`artifacts/qualified/independent-review.json`.
