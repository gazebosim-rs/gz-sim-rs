# Loading the GPU tool robot from SDF

The regular Gazebo executable now loads the two-joint GPU arm from an SDF world. The SDF declares the poses and axes of both revolute joints, the link inertias and centers of mass, a spherical collision at the tip, a static plane and both friction coefficients. The model file includes a servo command that pushes the tool onto the floor, then lifts it at the start of tick 301. [Robot and world](../../../examples/gpu_planar_tool_world.sdf)

```sh
RUSTC_WRAPPER= cargo run --release --locked -p gz-sim-cli --features gpu-batch --bin gz-sim-rs -- \
  examples/gpu_planar_tool_world.sdf --physics-engine gpu-batch \
  --gpu-adapter 'NVIDIA GeForce RTX 5090' --iterations 600
```

The GPU world plugin associates each independently moving SDF model with an environment ID and a static floor. It reads the maximum torque from each SDF joint and combines the two surface friction coefficients using the minimum. Drive targets use radians of displacement from the authored pose. A Rust caller can update both joints between steps through `gpu_scene::planar_tool::set_targets` and read committed tool reactions through `observe`. Ordinary Bevy link transforms, twists and joint samples update every millisecond. The GPU tracks full revolutions at each internal rotation so a fast hinge can pass through more than half a turn per public tick in either direction. World reset restores the original targets, poses and command schedule.

This profile supports a fixed world shoulder, two unlimited revolute joints along world −Y, link centers on their local X axes, and one tool sphere above a horizontal floor. Link geometry used only for display can be arbitrary supported primitives. Additional dynamic collision shapes, intersecting independent tool volumes, joint stops, speed limits and unsupported material laws are rejected. A shared plane can support several admitted arms.

| Measurement | SDF-loaded GPU | Fine Cartesian CPU | Native Rapier CPU |
|---|---:|---:|---:|
| First contact (ms) | 51.718 | 51.720 | 51.813 |
| First normal impulse (N s) | 0.38723 | 0.38556 | 0.40908 |
| Total normal impulse (N s) | 4.49932 | 4.50163 | 4.49889 |
| End of last loaded public tick (ms) | 314 | 314 | 314 |

The fine Cartesian reference uses RK4 and a 62.5 µs contact step. Native Rapier uses two physical hinges, the spherical tool collider and a static floor on the same fine clock. Both references were independently configured from the published fixture dimensions. Over 600 steps, the largest GPU tool-position error was 0.076 mm against the Cartesian reference and 0.250 mm against Rapier. The published joint displacement differed from the device angles by at most 2.23 × 10⁻¹⁶ rad. The GPU’s first impact impulse was 5.34% below Rapier’s. [Measurements](gpu-sdf-tool.json)

The standard CLI recorded the initial frame and 18 subsequent frames up to 591 ms at its normal display cadence. The viewer audit checked all 95 visual transforms and shapes against the link poses and the authored SDF offsets. Exact force and pose comparisons use all 601 public states. [Verified playback capture](gpu-sdf-playback.md) [Open the native SDF replay in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9843%2F)

Seventeen tests passed, including seven GPU solver tests, six loading and execution tests for this arm, and four existing GPU scene regressions. Across the implementation changes, 731 of 745 executable lines (98.12%) and 1,346 of 1,409 regions (95.53%) were covered.

To record another run, choose a new directory and an available local port.

```sh
RUSTC_WRAPPER= cargo run --release --locked -p gz-sim-cli --features gpu-batch --bin gz-sim-rs -- \
  examples/gpu_planar_tool_world.sdf --physics-engine gpu-batch \
  --gpu-adapter 'NVIDIA GeForce RTX 5090' --server --run --web-viewer 9844 \
  --web-lifecycle --iterations 600 --record-scene artifacts/gpu-sdf-tool-recording
```
