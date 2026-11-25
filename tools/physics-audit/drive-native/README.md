# Native Gazebo differential-drive reference

The reference runs the stock Gazebo `test/worlds/diff_drive.sdf` from gz-sim revision `9e3b78f5de297def8fe47fd189500fb5cd6538ef`. The workspace copy at `examples/worlds/upstream_diff_drive.sdf` has SHA-256 `4b71ce66989d416db8f93c4eeeb2c3934e0a5ec4492e1a35b688b61e9366fbf3`. It has one vehicle, two spherical drive wheels and a passive ball-jointed caster.

The Rust capture uses PyO3 and the installed Gazebo Sim and Transport bindings. The stock DiffDrive plugin receives `gz.msgs.Twist` messages on `/model/vehicle/cmd_vel` between blocking runs. Its velocity and acceleration limits apply. At 1 ms per step, the input schedule is

| Steps (inclusive) | Linear X (m/s) | Angular Z (rad/s) |
| --- | ---: | ---: |
| 1–500 | 0 | 0 |
| 501–2500 | 0.5 | 0 |
| 2501–4500 | 0.35 | 0.8 |
| 4501–5500 | 0 | 0 |

The program records initial link poses in PreUpdate. At release it assigns the SDF's zero initial wheel angles and speeds. Every 10 ms after release it reads committed link poses and wheel encoders in PostUpdate. The first six CSV signal columns describe the world pose of the chassis link. The next four are left/right wheel angle (rad) and speed (rad/s). The last three are the mean of the two wheel-link world positions, in metres. World +X points forward at release, +Y left and +Z up. Positive yaw is counterclockwise viewed from above. The log records the physics engine chosen by the server and the process mappings of the loaded plugins.

The capture also subscribes to the world-scoped `joint_state` topic at tick 4499 and saves its actual `gz.msgs.Model` packet at tick 4500 as protobuf text. In this packet the revolute wheel joints have one axis each. The caster has two published axes, each with position and velocity, with no axis-vector field. Gazebo's publisher warns that it can publish only two axes.

## Reproduce

Run these commands from the gz-sim-rs workspace root on a host with a native Gazebo installation. The CPython and Gazebo installation paths below are from the retained runs. The system Python package path supplies the protobuf runtime compatible with the installed generated message classes.

```bash
audit=tools/physics-audit/drive-native
py=/home/nkoenig/.local/share/uv/python/cpython-3.10.21-linux-x86_64-gnu
export PYTHONHOME="$py" PYO3_PYTHON="$py/bin/python3"
export PYTHONPATH=/home/nkoenig/local/lib/python:/usr/lib/python3/dist-packages
export LD_LIBRARY_PATH="$py/lib:/home/nkoenig/local/lib:${LD_LIBRARY_PATH:-}"
export RUSTC_WRAPPER=
build=target/drive-native-tools
cargo build --release --locked --manifest-path "$audit/Cargo.toml" --target-dir "$build"
out=$(mktemp -d)
GZ_PARTITION="drive_reference_$$" "$build/release/drive-native-reference" \
  examples/worlds/upstream_diff_drive.sdf "$out" > "$out/server.log" 2>&1
"$build/release/metrics" "$out/native.csv" "$out/native-metrics.json"
sha256sum "$out/native.csv" "$out/joint-state-4500.textproto"
```

On this host two independent processes produced the same SHA-256 values for the CSV (`b0a8b646392e11b8bb002c445b7c41a069bc02f41b8f1bf3b8980126de94243e`) and the joint-state packet text (`83797550af213d914344f5afa375544032ed7fb35416455cc192dafa44567b4a`). The retained `artifacts/drive-native/source-and-libraries.sha256` identifies the input SDF, libraries, bindings and server configuration.

## Measurements and comparison

For each 10 ms interval, the metrics tool projects axle-midpoint displacement on the direction perpendicular to the interval's midpoint chassis heading. Dividing by 10 ms gives signed lateral axle speed. This measures lateral motion at the drive axle during a turn. The chassis origin is about 0.706 m behind the axle and sweeps sideways around it. The braking distance sums the horizontal axle path from the stop command through the end of the run. Stopping time is the first start of an observation interval after which axle speed stays below 0.01 m/s and absolute yaw rate stays below 0.02 rad/s. A null stopping time means the trace ended before that condition was observed.

On DART, the axle travels 0.874750 m straight. The arc adds 1.439533 rad of yaw with peak lateral axle speed 0.0001434 m/s. After the zero command the axle travels 0.061395 m, the chassis turns another 0.160393 rad and the vehicle meets the stopping criterion after 0.39 s.

Pass a candidate CSV with the same schema and simulation clock to generate a comparison. The tool requires all 551 finite, ordered samples, writes both sets of measurements and returns a nonzero exit code when a tolerance is exceeded.

```bash
"$build/release/metrics" "$out/native.csv" "$out/comparison.json" artifacts/drive-view/final/ground.csv
```

| Matched quantity | Maximum absolute difference |
| --- | ---: |
| Chassis horizontal position (Euclidean distance) | 0.10 m |
| Chassis heading (wrapped angular distance) | 0.10 rad |
| Each cumulative wheel angle | 0.15 rad |
| Axle path during braking | 0.03 m |
| Signed lateral axle speed, interval by interval | 0.05 m/s |

These engineering thresholds were set before measuring the native or candidate trajectories.

## Retained comparisons

All three Rust runs used the same SDF, command journal and observation grid. The initial motor passed the lateral-speed bound alone. Switching to the force-limited joint velocity servo brought the wheel angles, chassis position and heading within their bounds, with braking distance still 43.95 mm longer than DART. Selecting the decoded SDF default friction coefficient (1.0 on these surfaces) in the force-limited servo run brought all six quantities inside their original thresholds. The replay switch for this profile is `--sdf-default-friction`.

| Maximum difference from DART | Soft motor | Servo, Rapier friction | Servo, SDF friction |
| --- | ---: | ---: | ---: |
| Chassis horizontal position (m) | 1.58102 | 0.080494 | 0.000316 |
| Chassis heading (rad) | 1.46711 | 0.088013 | 0.000403 |
| Left wheel angle (rad) | 2.57917 | 0.000635 | 0.000647 |
| Right wheel angle (rad) | 6.74250 | 0.003442 | 0.003456 |
| Braking path (m) | 0.130411 | 0.043949 | 0.000020 |
| Lateral speed (m/s) | 0.000143 | 0.043468 | 0.000100 |

The retained evidence lives under `artifacts/drive-native`. `comparison-initial.json` covers the soft motor, `comparison-servo.json` covers the servo with Rapier default friction, and `comparison.json` covers the servo with SDF default friction. Each report includes per-phase results for both engines.

The compressed bundle `artifacts/drive-native/drive-reference.tar.gz` contains the input world and journal, the Rust capture and comparison sources, raw measured CSVs, the three comparison reports, plugin startup log, native joint-state packet, and the original replay profiles with binary and input hashes. After extracting, run the retained comparison independently using Cargo. Disabling the native capture feature builds the Rust comparator without Gazebo or Python.

```bash
cargo run --locked --release --manifest-path tools/physics-audit/drive-native/Cargo.toml \
  --no-default-features --bin metrics -- artifacts/drive-native/native.csv \
  /tmp/drive-comparison.json artifacts/drive-native/rust-sdf-friction.csv
cmp /tmp/drive-comparison.json artifacts/drive-native/comparison.json
```
