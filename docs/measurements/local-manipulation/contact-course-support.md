<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Rocking off a ledge and removing a loaded support

The course uses two locked, two-link feet. In the first world, a toe moves past the edge of a finite box. In the second, the foot spans two separate pads and the right pad disappears while the simulation is paused. Both cases compare the switching robot with an independent copy that runs full dynamics throughout. All times below refer to the original world clock. [Watch both cases](contact-course-view.md) · [Edge input](../../../examples/replay/course_foot_edge.json) · [Paused-removal input](../../../examples/replay/course_paused_support.json) · [Measured results](contact-course-support.json)

| Measurement | Toe crossing the ledge | Right pad removed during pause |
|---|---:|---:|
| Physics step | 25 µs | 500 µs |
| Last instant before the resumed solve | 57.150 ms | 1.200 s |
| First resumed impulse on remaining surface, switching run | 0.000493382 N·s | 0.00936085 N·s |
| Same impulse, uninterrupted dynamics | 0.000490499 N·s | 0.00933832 N·s |
| Difference from uninterrupted dynamics | +0.588% | +0.241% |
| First unequal sampled pose | 0.250 ms | 1.2005 s |
| First pose error above 1 µm or 10 µrad on either link | 1.450 ms | 1.2255 s |
| Maximum foot-center error over the full route | 0.04947 mm | 0.006424 mm |
| Maximum geometric foot–support penetration, switching / uninterrupted | 0.01250 / 0.01446 mm | 0.04170 / 0.04170 mm |

**Toe crossing.** The toe starts 50 µm inside the ledge. Both runs have identical poses, velocities and cached impulses at 0.225 ms, when one foot starts following its prescribed motion. Its toe crosses the lip at 57.150 ms, which expires the four-point contact history. At promotion the old 0.000139 N·s normal seed and its friction seeds are cleared. The uninterrupted reference has been following a slightly different path since the first prescribed step.

To measure the handoff independently of this difference, a third full-dynamics run starts at the wake boundary with a fresh contact history, the same geometry and the measured pose and twist of the locked assembly. The reconstructed link centers match within 3.9 × 10⁻¹⁵ m and linear velocities within 4.6 × 10⁻¹⁹ m/s. Against this incoming-state reference, the resumed impulse differs by 6.6 × 10⁻¹⁵ N·s. The foot centers remain within 1.2 × 10⁻¹⁵ m over the next 3,200 steps. Both reference comparisons use the public outer step.

**Paused removal.** Both pads carry positive load at 1.200 s. The switching lane parks at that instant and the pad is removed from both lanes before either one advances. The survivor keeps its four contact points, their features, anchors, and normal, tangential and torsional seeds. The retiring pair becomes inaccessible to the solver’s public warm-start query. Its most recent narrow-phase snapshot is retired on the next advancing step. Three paused updates leave the clock, both link poses and twists, and the surviving cache unchanged. At 1.2005 s the load-loss policy has promoted the foot for the first solve. The removed pad contributes zero impulse in both lanes for the full remaining 0.30 s.

**Measurement.** Maximum penetration uses a fresh oriented box–box proximity query on committed foot and pad poses each tick. The 5% first-impulse and 0.1 mm edge / 1 mm removal trajectory bounds were specified before the qualification run. Wall-clock timings include a complete `Simulation::step_exact(1)` call and exclude measurement, file output and rendering. Timing uses four fresh runs of each case on an AMD Ryzen 9 9950X3D, alternating lane order by step and by run. Over 21,944 timed steps per lane at the ledge, p95 was 12.37 µs for switching and 13.00 µs for full dynamics. For removal the corresponding costs were 14.67 and 14.56 µs over 12,000 steps per lane.

Run the measurements from the repository root. The installed `gz-sim-local-replay` binary can also execute either SDF/journal pair directly.

```sh
RUSTC_WRAPPER= cargo run --locked --release --no-default-features -p gz-sim-cli \
  --example support_edge_course -- target/support-edge-course.json
```
