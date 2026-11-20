<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->
# M79: one model VelocityControl system

The owner requested continued development on November 20, 2026. Root selected
this retained SYS-CONTROL increment with a budget of at most five working days,
including independent review and mandatory verification. The selection and file
leases are recorded in primary `artifacts/m79-delivery/m79-scope-and-source-leases.json`.

The implementation and regression Source lanes are complete: 23 new Rust
declarations (23 scheduled headless, 14 also scheduled in the renderer profile),
with zero new Python declarations. These are Source counts, not passes. Two
existing Python positive-roster fixtures have additive peer entries. Whole
independent merged Source review and the native-reference adoption remain pending;
no M79 tests, lint, formatter or native capture has run.

Root selected a shared M78/M79 candidate under D31 after standalone M78's
full hook failed: Root/Git 1 after 4169.970975892 seconds, at the unchanged
3600-second strict-headless profile timeout. The retained reset test was
unfinished; the cause remains UNKNOWN. Failed M78 tree `9774cd21` and all partial
evidence remain frozen. No qualified M78 commit or standalone pass exists.

This checkout integrates M78's reviewed posed-box implementation and contract
with M79 over qualified but unaccepted M77
`57b44de10713dc6132c5549d769dac00dc041f56`; accepted Main ancestor is M76
`dddf1f4284557c420e384d263a4365b2153dfde2`. M78 and M79 remain separate scopes
in one complete candidate: 39 distinct Rust declarations, H39/R21 with overlap21,
and zero new Python declarations. M78 owns 16 (H16/R7); M79 owns 23 (H23/R14).
These are unexecuted Source obligations. Final merged Source/parent review and
one fresh combined qualification must prove both; neither phase inherits the
failed run or obtains a fictional intermediate Main promotion. M77 predecessor
acceptance remains required before downstream acceptance/Main.

M1–M76 and 202 slices remain accepted. M79 grows the catalog to 79 phases;
public Latest M70 remains the original 41-example package. Source finite/CLI
inventories stay 47/21. M77's persistent manual demo remains running without
session expiry until the owner confirms stop; it is not formal GUI acceptance.
This work does not stop or restart it, alter frozen M78, or modify Fuel work.

## Selected behavior

Admit at most one explicitly configured `gz::sim::systems::VelocityControl`
plugin, filename `gz-sim-velocity-control-system`, per loaded world. Its target
is a top-level, nonstatic, unjointed model with one free link, identity resolved
link pose and centered inertia, using existing centered primitive admission on
Rapier f64 or TPE. Model pose may be rotated: commands are in model axes, not
world axes. No M78 posed-box API or new physics interface is needed.

The closed plugin configuration admits only `topic`, `initial_linear` and
`initial_angular`. Missing initial vectors are zero. Vectors contain three finite
f64 values each, in m/s and rad/s, with checked representable world conversion.
The default topic uses the plain SDF model name, `/model/<name>/cmd_vel`; one
valid explicit custom topic is supported. Decode the existing canonical `gz.msgs.Twist` through the
existing upstream Zenoh owner. Missing message vectors are zero. Preserve the
existing message-size, bounded-inbox, lease and epoch guards; no new protocol or
network framework is selected.

Configure initializes both committed and pending targets. On each advancing
PreStep, apply the previous committed target, rotated by the current model world
orientation, through maintained world-velocity APIs before ordinary engine
stepping. PostStep latches the latest accepted pending target for the next
advancing step. Last targets persist, including zero. While paused, ingress may
replace the pending target, but application and latching do not advance. The
first resumed step uses the old committed target before its PostStep latch.

This is the upstream opt-in velocity-command system. Never call a pose,
position or body-state setter, fabricate motion publications, or route existing
demos or DiffDrive through it. Engine stepping and ordinary pose synchronization
produce motion. Existing wheel controllers remain motor/force driven. A zero
target is not advertised as force-balanced station keeping: gravity or applied
wrenches may act between velocity-command applications.

The Scene/Server boundary uses immutable `VelocityControlConfiguration` with
fully scoped `model_name` (such as `default::rig`), public link ID, `topic`,
`initial_linear` and `initial_angular`.
`set_velocity_control_command(&mut Simulation, model_id, linear, angular)` stores
the pending target. Initial-configuration/identity getters and
`velocity_control_binding_matches` / `_matches_configuration` keep transport
leases tied to the exact loaded owner; callbacks never mutate a simulation.

For a qualifying model, the opt-in plugin configuration is:

```xml
<plugin filename="gz-sim-velocity-control-system"
        name="gz::sim::systems::VelocityControl">
  <topic>/rig/cmd_vel</topic>
  <initial_linear>0.25 0 0</initial_linear>
  <initial_angular>0 0 0</initial_angular>
</plugin>
```

This is a Source capability; the published package is not refreshed by M79.

## Ownership and lifetime

Plugin presence is structural demand. Without the plugin there is no new
controller, callback or endpoint allocation. Removal and shutdown retire only
owned controller and ingress resources; survivors remain usable.

The Scene reset handler for a surviving controller (`Simulation::reset`)
restores configured initial targets and clears pending commands.
`reset_loaded_world` also restores removed/original configuration and public IDs.
Server `all` and `time_only` controls use its existing loaded-reconstruction
path and renew ingress channels and identity/epoch fences. Stale callbacks and
queued commands cannot control restored IDs. Server `model_only` behavior stays
unchanged, without a new VelocityControl reset hook. These named Rust paths are
not blanket time-rewind, model-only or native full-manager reset parity.

The configured link has exclusive velocity ownership. Separate direct velocity
injection and `set_model_state` on that owned model are explicitly refused rather
than racing or bypassing the controller. Read-only model-state queries and
uncontrolled-model inputs/edits retain their behavior. Existing Rapier wrenches
and TPE capability refusals remain intact.

Reject unknown, duplicate or malformed configuration; `link_name`; nested,
static, multi-link or jointed targets; multiple controllers; competing ownership
or endpoint collisions; nonfinite or unrepresentable inputs. Perform configuration
and endpoint preflight before opening external IO. Invalid commands preserve the
previous admitted target. Runtime spawning, controller multiplicity, wheel or
actuator modes, limiters, pose edits and generic plugin loading are excluded.

## Native reference and proof limits

The native Source pin is gz-sim
`71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`. `VelocityControl.cc` configures initial
targets, writes the old latch in PreUpdate, copies the received target in
PostUpdate, and returns from both while paused. `Physics.cc:3119–3217` rotates
model commands into world axes. These Source references do not claim execution.

Prepare a bounded CPU-only direct-component capture using the existing original
VelocityControl object and retained SDK. Record numbered Configure, PreUpdate
and PostUpdate calls, actual UpdateInfo and complete component vectors, canonical
Twist publications/taps, discovery, all component-effect probe rounds and owned
maps/exit/cleanup evidence. A bounded component-effect fence can observe an
effect in the one planned run. Publish success, a tap and HasConnections are
**not** receipts for the system's private callback. Callback receipt timing and
dispatch quiescence remain UNKNOWN when not observed. Keep native Source-proven
timing/pause semantics distinct from actual asynchronous component traces.

No physics or full SystemManager reset runs in that helper. `SystemManager.cc`
reloads plugin instances without ISystemReset, but cannot reconstruct arbitrary
in-memory instances. Bind that Source and limitation; do not describe a helper
reconstruction as native reset parity. Actual Rust loaded reset is verified under
the simulator's declared reconstruction contract. No SDK, DART or MuJoCo build,
native backend job, renderer, Docker or browser campaign is selected.

## Leases and acceptance

| Owner | Lease |
| --- | --- |
| Scene | `crates/gz-sim-scene/**`: focused configuration/runtime/API leaves, shared integration there and additive tests/fixtures. Agree DTOs with Server before interface edits. |
| Server | `crates/gz-sim-server/**`; Root-extended native-peer roster leaves `scripts/{native-peer,zenoh_peer,verify}.py` and the two existing positive-roster fixtures `tests/test_{selected_native_gate,zenoh_peer}.py`. Canonical ingress, bounded lifecycle and additive real C++ peer/tests; no negative-fixture weakening. |
| Documentation/native Source | This contract, `planning/delivery-phases.json`, `docs/DELIVERY_PLAN.md`, `docs/HANDOFF.md`, `docs/DECISIONS.md`, `README.md`, `AGENTS.md`; held CPU capture Source/procedure only in primary artifacts. |
| Root | Shared integration decisions, independent merged review, actual parent/source reconciliation and serial native/formatter/full gates/publication. |

Require configured initial values and all six axes; rotated-model frame controls
with independent analytic Rapier/TPE motion; previous-latch timing, paused pending
targets and persistent/zero commands; gravity or wrench countercontrols; canonical
real C++ ingress; invalid-message preservation and bounded inbox; pre-IO refusals;
absent/last-owner demand, exclusive velocity ownership, loaded reset/removal,
stale leases, survivors, shutdown and unchanged uncontrolled/DiffDrive behavior.
Use test-owned fixtures so finite/CLI/package registries remain unchanged.

Finish implementation, meaningful regression Source, docs and independent whole
Source review; batch every confirmed finding before one consolidated final
formatter/normal/strict cycle. Fresh bounded native component evidence and
independent DATA, own strict coverage, exact hosted/official evidence, parent
reconciliation and qualified Main gates remain mandatory. The shared candidate
needs one fresh complete normal/strict/hosted/official gate chain proving both
scopes; the failed standalone M78 run contributes no success. Root alone releases
heavy jobs after merged Source clearance. All future
capture, test, run, commit and acceptance results remain unknown until actual.

Days 1–2 cover disjoint Scene/Server/native Source; day 3 integration/docs and
independent review; days 4–5 the required consolidated qualification and closeout.
This is a bounded work budget, not permission for per-edit checks or a guarantee
of success. Retain historical failures, unknown waits, all old assertions and
thresholds. D33 keeps DART/MuJoCo product work cancelled; keep Rapier f64 and TPE.

## Checked-in native reference reproduction

The preparation-time statements above are historical checkpoints. The original
component capture and its independent retained DATA completed before this Source
closure. The fixture values and original transcripts are unchanged. This section
adds reproducibility inputs; it does not run a new capture or qualify M78/M79.

The exact captured generating program is checked in as
`crates/gz-sim-scene/tests/reference/m79_velocity_control_components.cc`
(SHA-256 `9be8ea04acd56d53d05a45842e5901d6720e9fdbad230821b85367e524df6d60`).
It is distinct from the C++ ingress peer. The fixture's adjacent provenance JSON
now records the actual compiler argv, both actual run argv/environment records,
original object and gz-sim revision `71c60db31fddd5c98d2aa689e9a2cccb3548a0a3`,
compiler/readelf target hashes, named header cards, and all 336 actually mapped
library hashes from the successful capture. Match those inputs before rebuilding;
the paths below describe that recorded SDK installation, not a generic substitute.
The object is linked directly; no VelocityControl plugin DSO, new SDK build,
physics engine step or SystemManager reset is part of this reference.

For a separately authorized regeneration, run from this repository root. Use a
fresh directory and transport partition; preserve all original capture files.
The following compile command is the recorded argv with only the generating
Source and new output paths substituted. The two commands retain the recorded
mode and owned model names, changing only the binary/maps/temp/partition paths.
They retain the helper's 20-second active bound and five-second effect fences.
They were not executed for this checked-in closure.

```sh
set -eu
repro="$(mktemp -d)"
partition="m79-reproduce-$(basename "$repro")"
env -i PATH=/usr/bin:/bin GIT_CONFIG_NOSYSTEM=1 GIT_CONFIG_GLOBAL=/dev/null GIT_NO_REPLACE_OBJECTS=1 LC_ALL=C TMPDIR="$repro" GZ_SIM_SERVER_CONFIG_PATH= GZ_IP=127.0.0.1 GZ_PARTITION="$partition" GZ_SIM_SYSTEM_PLUGIN_PATH= LD_LIBRARY_PATH=/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang/lib:/home/nkoenig/local/lib:/opt/ros/jazzy/opt/zenoh_cpp_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib DEBUGINFOD_URLS= M79_NATIVE_COMPONENT_CAPTURE_ROOT_RELEASED=1 /usr/bin/clang++ -std=c++17 -O2 -Wall -Wextra -Werror -I/home/nkoenig/work/gazebo/rust/gz-sim/include -I/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang/include -I/home/nkoenig/work/gazebo/rust/gz-sim/test -I/home/nkoenig/work/gazebo/rust/gz-sim -I/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang -I/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang/core/include -I/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang/src -I/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang/gz_msgs_gen -I/home/nkoenig/work/gazebo/rust/gz-sim/vendor/backward-cpp -isystem /home/nkoenig/local/include/gz/math10 -isystem /home/nkoenig/local/include/gz/utils5 -isystem /home/nkoenig/local/include/gz/plugin5 -isystem /home/nkoenig/local/include/gz/common8 -isystem /usr/include/uuid -isystem /home/nkoenig/local/include/gz/fuel_tools12 -isystem /home/nkoenig/local/include/gz/msgs13 -isystem /home/nkoenig/local/include/gz/physics10 -isystem /usr/include/eigen3 -isystem /home/nkoenig/local/include/gz/rendering11 -isystem /home/nkoenig/local/include/gz/transport16 -isystem /home/nkoenig/local/include/gz/sdformat17 -isystem /home/nkoenig/local/include/gz/gui11 -isystem /usr/include/x86_64-linux-gnu/qt6/QtCore -isystem /usr/include/x86_64-linux-gnu/qt6 -isystem /usr/lib/x86_64-linux-gnu/qt6/mkspecs/linux-g++ -isystem /usr/include/x86_64-linux-gnu/qt6/QtQml -isystem /usr/include/x86_64-linux-gnu/qt6/QtQmlIntegration -isystem /usr/include/x86_64-linux-gnu/qt6/QtNetwork -isystem /usr/include/x86_64-linux-gnu/qt6/QtQuick -isystem /usr/include/x86_64-linux-gnu/qt6/QtGui -isystem /usr/include/x86_64-linux-gnu/qt6/QtQmlModels -isystem /usr/include/x86_64-linux-gnu/qt6/QtOpenGL -isystem /usr/include/x86_64-linux-gnu/qt6/QtQuickControls2 -isystem /usr/include/x86_64-linux-gnu/qt6/QtWidgets -isystem /home/nkoenig/work/gazebo/rust/gz-sim/test/gtest_vendor/include -I/home/nkoenig/work/gazebo/rust/gz-sim/src/systems/velocity_control "$PWD/crates/gz-sim-scene/tests/reference/m79_velocity_control_components.cc" /home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang/src/systems/velocity_control/CMakeFiles/gz-sim-velocity-control-system.dir/VelocityControl.cc.o -o "$repro/native-velocity-control-components" -lstdc++fs '/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang/lib/libgz-sim.so.11.0.0~pre3' -lpthread '/home/nkoenig/local/lib/libgz-fuel_tools.so.12.0.0~pre2' '/home/nkoenig/local/lib/libgz-physics.so.10.0.0~pre2' '/home/nkoenig/local/lib/libgz-rendering.so.11.0.0~pre2' '/home/nkoenig/local/lib/libgz-common-profiler.so.8.0.0~pre2' '/home/nkoenig/local/lib/libgz-common-av.so.8.0.0~pre2' /usr/lib/x86_64-linux-gnu/libswscale.so /usr/lib/x86_64-linux-gnu/libswscale.so /usr/lib/x86_64-linux-gnu/libavdevice.so /usr/lib/x86_64-linux-gnu/libavdevice.so /usr/lib/x86_64-linux-gnu/libavformat.so /usr/lib/x86_64-linux-gnu/libavformat.so /usr/lib/x86_64-linux-gnu/libavcodec.so /usr/lib/x86_64-linux-gnu/libavcodec.so /usr/lib/x86_64-linux-gnu/libavutil.so /usr/lib/x86_64-linux-gnu/libavutil.so '/home/nkoenig/local/lib/libgz-common-io.so.8.0.0~pre2' '/home/nkoenig/local/lib/libgz-common-geospatial.so.8.0.0~pre2' '/home/nkoenig/local/lib/libgz-common-graphics.so.8.0.0~pre2' '/home/nkoenig/local/lib/libgz-transport-parameters.so.16.0.0~pre2' '/home/nkoenig/local/lib/libsdformat.so.17.0.0~pre2' '/home/nkoenig/local/lib/libgz-gui.so.11.0.0~pre2' '/home/nkoenig/local/lib/libgz-transport.so.16.0.0~pre2' '/home/nkoenig/local/lib/libgz-msgs.so.13.0.0~pre2' /usr/lib/x86_64-linux-gnu/libprotobuf.so '/home/nkoenig/local/lib/libgz-common-events.so.8.0.0~pre2' '/home/nkoenig/local/lib/libgz-math.so.10.0.0~pre2' '/home/nkoenig/local/lib/libgz-plugin-loader.so.5.0.0~pre2' '/home/nkoenig/local/lib/libgz-plugin.so.5.0.0~pre2' '/home/nkoenig/local/lib/libgz-utils.so.5.0.0~pre2' /usr/lib/x86_64-linux-gnu/libQt6QuickControls2.so.6.4.2 /usr/lib/x86_64-linux-gnu/libQt6Quick.so.6.4.2 /usr/lib/x86_64-linux-gnu/libQt6QmlModels.so.6.4.2 /usr/lib/x86_64-linux-gnu/libQt6Qml.so.6.4.2 /usr/lib/x86_64-linux-gnu/libQt6Network.so.6.4.2 /usr/lib/x86_64-linux-gnu/libQt6OpenGL.so.6.4.2 /usr/lib/x86_64-linux-gnu/libQt6Widgets.so.6.4.2 /usr/lib/x86_64-linux-gnu/libQt6Gui.so.6.4.2 /usr/lib/x86_64-linux-gnu/libQt6Core.so.6.4.2 /usr/lib/x86_64-linux-gnu/libGLX.so /usr/lib/x86_64-linux-gnu/libOpenGL.so /usr/lib/x86_64-linux-gnu/libtinyxml2.so /usr/lib/x86_64-linux-gnu/libtinyxml2.so /usr/lib/x86_64-linux-gnu/libdw.so /usr/lib/x86_64-linux-gnu/libelf.so /usr/lib/x86_64-linux-gnu/libpthread.a /usr/lib/x86_64-linux-gnu/libz.so /usr/lib/x86_64-linux-gnu/libbz2.so /usr/lib/x86_64-linux-gnu/liblzma.so /usr/lib/x86_64-linux-gnu/libzstd.so '/home/nkoenig/local/lib/libgz-common-testing.so.8.0.0~pre2' '/home/nkoenig/local/lib/libgz-common.so.8.0.0~pre2' -lpthread '/home/nkoenig/local/lib/libgz-utils-log.so.5.0.0~pre2' /usr/lib/x86_64-linux-gnu/libspdlog.so.1.12.0 /usr/lib/x86_64-linux-gnu/libfmt.so.9.1.0 /usr/lib/x86_64-linux-gnu/libuuid.so /usr/lib/x86_64-linux-gnu/libuuid.so -Wl,-rpath-link,/opt/ros/jazzy/opt/zenoh_cpp_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib -Wl,-rpath,/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang/lib:/home/nkoenig/local/lib:/opt/ros/jazzy/opt/zenoh_cpp_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib
env -i PATH=/usr/bin:/bin GIT_CONFIG_NOSYSTEM=1 GIT_CONFIG_GLOBAL=/dev/null GIT_NO_REPLACE_OBJECTS=1 LC_ALL=C TMPDIR="$repro" GZ_SIM_SERVER_CONFIG_PATH= GZ_IP=127.0.0.1 GZ_PARTITION="$partition" GZ_SIM_SYSTEM_PLUGIN_PATH= LD_LIBRARY_PATH=/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang/lib:/home/nkoenig/local/lib:/opt/ros/jazzy/opt/zenoh_cpp_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib DEBUGINFOD_URLS= M79_NATIVE_COMPONENT_CAPTURE_ROOT_RELEASED=1 "$repro/native-velocity-control-components" defaults m79_87028e5cb1d56eadcc793149 "$repro/defaults.maps" >"$repro/defaults.stdout" 2>"$repro/defaults.stderr"
env -i PATH=/usr/bin:/bin GIT_CONFIG_NOSYSTEM=1 GIT_CONFIG_GLOBAL=/dev/null GIT_NO_REPLACE_OBJECTS=1 LC_ALL=C TMPDIR="$repro" GZ_SIM_SERVER_CONFIG_PATH= GZ_IP=127.0.0.1 GZ_PARTITION="$partition" GZ_SIM_SYSTEM_PLUGIN_PATH= LD_LIBRARY_PATH=/home/nkoenig/work/gazebo/rust/gz-sim-rs/artifacts/reference/gz-sim-clang/lib:/home/nkoenig/local/lib:/opt/ros/jazzy/opt/zenoh_cpp_vendor/lib:/opt/ros/jazzy/opt/gz_dartsim_vendor/lib DEBUGINFOD_URLS= M79_NATIVE_COMPONENT_CAPTURE_ROOT_RELEASED=1 "$repro/native-velocity-control-components" explicit m79_e00069b2de93cc5affb114d6 "$repro/explicit.maps" >"$repro/explicit.stdout" 2>"$repro/explicit.stderr"
python3 - "$repro" crates/gz-sim-scene/tests/fixtures/m79_velocity_control_native.json <<'PY'
import hashlib, json, math, struct, sys
from pathlib import Path
work = Path(sys.argv[1])
expected = Path(sys.argv[2])
names = ("initial_pre", "a_first_pre", "a_effect", "paused_held",
         "first_resume_pre", "zero_effect", "partial_before_post",
         "partial_effect", "persistence")
def vec(words):
    assert len(words) == 3
    values = [struct.unpack(">d", bytes.fromhex(word))[0] for word in words]
    assert all(len(word) == 16 for word in words) and all(map(math.isfinite, values))
    return values
fixture = {"schema_version": 1,
           "state": "M79_ORIGINAL_VELOCITY_CONTROL_COMPONENT_REFERENCE", "modes": []}
for mode in ("defaults", "explicit"):
    raw = (work / (mode + ".stdout")).read_bytes()
    assert 0 < len(raw) <= 16 * 1024**2 and raw.endswith(b"\n")
    lines = raw.splitlines()
    assert len(lines) <= 4096 and all(len(line) + 1 <= 16384 for line in lines)
    rows = [json.loads(line) for line in lines]
    assert all(row["schema"] == "M79_NATIVE_COMPONENT_LEDGER_V1" and
               type(row["ordinal"]) is int and row["ordinal"] == i
               for i, row in enumerate(rows))
    assert rows[0]["kind"] == "configure_call" and rows[0]["mode"] == mode
    assert rows[-1]["kind"] == "summary"
    assert rows[-1]["state"] == "NATIVE_PUBLIC_COMPONENT_OBSERVATIONS_PENDING_ACTUAL_EXIT_AND_DATA"
    calls = {row["call"]: row for row in rows if row["kind"] == "update_call"}
    pre = [(calls[row["call"]], row) for row in rows
           if row["kind"] == "update_return" and calls[row["call"]]["interface"] == "PreUpdate"]
    def first(reason):
        return next(ret for call, ret in pre if call["reason"] == reason)
    def effect(phase):
        start = next(row["ordinal"] for row in rows
                     if row["kind"] == "publication" and row["phase"] == phase)
        fence = next(row for row in rows
                     if row["kind"] == "effect_fence" and row["phase"] == phase)
        assert fence["completion"] == "OBSERVED_PUBLIC_COMPONENT_EFFECT"
        probes = [ret for call, ret in pre if start < ret["ordinal"] < fence["ordinal"]
                  and call["reason"] == "component_effect_probe"]
        assert 1 <= len(probes) == fence["actual_probe_rounds"] <= 32
        return probes[-1]
    selected = (first("initial"), first("first_pre_before_post_latch"), effect(0),
                first("paused_after_publication"), first("first_resume_pre"), effect(1),
                first("partial_pre_before_post_latch"), effect(2), first("persistent_target"))
    observations = [{"milestone": name, "linear": vec(ret["commands_after"]["linear"]),
                     "angular": vec(ret["commands_after"]["angular"])}
                    for name, ret in zip(names, selected)]
    fixture["modes"].append({"mode": mode, "initial_linear": observations[0]["linear"],
                           "initial_angular": observations[0]["angular"],
                           "observations": observations})
encoded = (json.dumps(fixture, indent=2, ensure_ascii=True) + "\n").encode("utf-8")
assert hashlib.sha256(encoded).hexdigest() == "6bc234d23191fb59f753ec6bc962c3e42f45d8b3e6271804d956c9c579dcea98"
assert encoded == expected.read_bytes()
with (work / "regenerated-native.json").open("xb") as output:
    output.write(encoded)
PY
```

The closed projection selects actual PreUpdate returns by reason and the last
actual probe return before each numbered effect fence. It never fixes an
asynchronous probe count or invents a callback receipt. It decodes the recorded
binary64 component words, emits the nine approved milestones in order, and
requires byte equality with the unchanged checked-in fixture. The full retained
DATA reader, rather than this small projection recipe, validates complete wire,
call ordering, SDK custody and lifecycle evidence. Native private callback timing
and quiescence remain UNKNOWN; this fixture proves public component observations,
not native physics, full-manager reset or Source-to-original-object equivalence.
