# GPU arm touchdown, slip and lift-off

A rigid round tool at the end of each two-joint arm now collides with a horizontal floor. The GPU applies the normal and friction impulses to both joints through the arm's inertia matrix. The tool rolls with the wrist, so its tangential contact velocity includes the tool radius. The base and links do not collide with these tool test surfaces. Each public Bevy tick covers 1 ms and contains two 0.5 ms midpoint solves.

The nine observed arms combine payloads of 0, 0.10 and 0.25 kg with friction coefficients of 0, 0.20 and 0.45. Each drops 35–41 mm, pushes against the floor, and receives a lift command at 300 ms. The three frictionless tools reported zero tangential impulse throughout. All nine had positive support in the 20 ms before the lift command and zero reaction in the final 20 ms. The most deeply penetrating sampled GPU tool crossed its floor by 34.4 nm.

| Worst discrepancy over the nine arms | Fine Cartesian reference | Native Rapier |
|---|---:|---:|
| First contact time | 2.17 µs | 115.02 µs |
| First internal-step normal impulse | 0.93% | 7.73% |
| Normal impulse over first ten loaded-window ticks | 1.52% | 1.42% |
| Tangential impulse over the same window | 0.00217 N s | 0.00203 N s |
| Normal impulse over the entire run | 0.088% | 0.401% |
| Net tangential impulse over the entire run | 0.0265 N s | 0.0548 N s |
| Last loaded public tick | 1 ms | 0 ms |
| Largest tool-position error | 0.45 mm | 0.83 mm |

The clock and impulse measurements answer different questions. The GPU and Cartesian calculation estimate each geometric crossing from the signed gap before and after their internal step. They apply the impact impulse at that step's end. Rapier reports the end of its first step with a positive measured normal impulse, using a 62.5 µs clock and zero prediction distance. The ten-tick window begins with each solver's own first positively loaded public tick. The total includes subsequent support, sliding and release.

The sharpest disagreement occurred for the 0.1 kg tool with friction 0.45. Its first GPU normal impulse was 0.3594 N s, versus 0.3586 N s in the independent Cartesian solve and 0.3895 N s in Rapier. The ten-tick normal impulses were 0.5330, 0.5348 and 0.5344 N s respectively. Rapier and the GPU both last reported support in the tick ending at 312 ms. This first-impact discrepancy remains open for matched incoming-state analysis.

The GPU uses an inelastic normal constraint and solves the coupled two-dimensional contact mass for sticking or sliding bounded by Coulomb friction. A separate mass-weighted position correction resolves penetration without changing joint velocity. The independent CPU reference assembles Cartesian Jacobians for both link centers and the tip mass, advances the servos with RK4 at 62.5 µs, and eliminates the normal equality to compute friction. Native Rapier uses finite colliders on an actual reduced-coordinate articulated body. Its normal and tangential values come from solved contact rows.

[Open the replay in gz-web-rs](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A9837%2F) · [Touchdown and lift impulse chart](gpu-tool-impulses.svg) · [Viewer capture](gpu-tool-contact-view.jpg) · [Measurements by load and friction](gpu-tool-contact.json)

The orange sphere is the tool. The red marker extends above it only during a tick with a measured upward impulse. The replay contains all 601 frames. Independent checks covered 5,409 arm states, 32,454 displayed poses and shapes, and rewind to zero.

```sh
RUSTC_WRAPPER= cargo run --release --locked -p gz-physics-gpu --example planar_tool -- \
  'NVIDIA GeForce RTX 5090' artifacts/gpu-tool-rerun
```

The 2,048-lane run includes a negative control with its floor lowered by 2 m. It remained at least 1.510 m clear and received zero impulse throughout. Reset and replay reproduced the complete dense GPU trajectory byte for byte across all 600 ticks. Device dispatch p95 was 29.15 µs and dense Bevy tick p95 was 134.34 µs, including the fence and full readback. Timing used a separate pass without CPU reference solves or visualization.

Nine focused tests including two physical GPU tests and strict Clippy passed. Release and instrumented runs reproduced the contact trace, independent frame data and native replay byte for byte. Exact changed production and example coverage was 581/595 executable lines (97.65%) and 1,289/1,326 regions (97.21%).
