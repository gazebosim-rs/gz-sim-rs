<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# An off-center impact against a powered finger’s travel stop

A 2 N drive presses an 80 g sliding finger against its upper stop. A 200 g sphere approaches at 0.75 m/s from 50 µm away. It hits 20 mm off center. Restitution is 0.6 and friction and gravity are zero. The wrist is free to rotate about a vertical hinge. Reversing the sphere alone tests whether the finger can leave the loaded stop. Reflecting the entire scene tests the lower stop under a −2 N drive. [SDF world](../../../examples/worlds/local_powered_slider_impact.sdf)

The upper stop and same-drive release are available in the native viewer. Both replays use the ordinary local journal. [Held upper stop](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11426%2F) · [Release away from the stop](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A11427%2F) · [Portable recordings](powered-slider-replays.tar.xz) · [Replay instructions](powered-slider-view.md)

## Independent mechanics

Use slider position s and hinge angle θ, measured toward the loaded upper stop. The finger center in wrist coordinates is (s, L), with L = 0.1 m. Its mass m is 0.08 kg, the sphere mass M is 0.20 kg, and the sum I of the wrist and finger spin inertias is 0.0004587533333 kg·m². At contact d is the signed distance from the finger center to the sphere’s line of force along the guide’s transverse axis. Let a = +1 or −1 be the incoming direction in the guide frame, and let v be the closing speed along the instantaneous contact normal.

With a free slider, a unit impulse changes relative normal speed by K_free. With a seated slider constrained against the impact, the wrist and finger rotate together and the corresponding change is K_stop.

~~~text
A = I + m (L² + s²)
K_free = 1/M + 1/m + d²/(I + m s²)
K_stop = 1/M + (L + d)²/A
J = (1 + restitution) v / K
~~~

The calculation first finds the free response. When the finger is on the upper stop and that response would move it farther into the stop, it uses K_stop, zero slider speed, and a compressive stop impulse. For a seated arrival from rest this gives Δω = −a(L + d)J/A. The stop receives a generalized impulse of −mLΔω − aJ on the finger. An impact that moves the finger away uses K_free and zero collision impulse from the stop.

With room to move, the constant drive changes both velocities before impact. The independent Rust calculation integrates these two equations up to the earlier of contact or a stop, then locates the event time by bisection. Travel-stop engagement preserves angular momentum about the hinge. The frictionless collision uses the actual closing speed and lever arm at that time.

~~~text
θ̇ = ω,  ṡ = u
ω̇ = [L F + m s ω (Lω − 2u)] / (I + m s²)
u̇ = Lω̇ + sω² + F/m
~~~

The signed stop impulse in the native runs is inferred from the finger’s change in axial momentum. The calculation subtracts the measured contact impulse along the slider, the held motor impulse FΔt and the integral of msω² over the native step. This gives the stop reaction over the whole reporting interval, including any drive preload. The ideal instantaneous collision reaction is listed separately.

## Predicted physical cases

The table uses the upper-stop coordinate convention. Each case also runs as a reflected lower-stop fixture. The final two columns give signed speeds immediately after the sphere strike, before a subsequent stop can act.

| Initial stop and drive | Sphere contact (µs) | Sphere impulse (mN·s) | Wrist speed (rad/s) | Slider speed (m/s) |
| --- | ---: | ---: | ---: | ---: |
| Seated, +2 N, strike into stop | 66.6667 | 72.99319 | −6.95862 | 0 |
| Seated, +2 N, strike away | 66.6667 | 65.31704 | +2.84759 | −0.53170 |
| 15 mm clearance, +2 N | 66.71498 | 65.22244 | −2.81438 | +0.53551 |
| 50 nm clearance, +2 N | 66.68249 | 72.99318 | −6.95862 | 0 |
| 2 µm clearance, +2 N | 66.71498 | 65.22244 | −2.81438 | +0.53551 |
| Initially seated, −2 N, strike toward upper stop | 66.61850 | 65.41151 | −2.88075 | +0.52790 |

The drive closes the initial 50 nm clearance at 38.18118 µs, before the sphere arrives. With 2 µm clearance, the sphere hits first and the finger reaches the stop at 70.16390 µs. With −2 N pulling the finger away from its initial seat, the impact sends it back to the stop at 66.90684 µs. Both events in this last case fall within the same 1 µs public sample. The 100 ns and 25 ns runs separate them.

## Measured first strike

The control uses the same authored force, finite stop, reduced wrist and slider, with the upstream Rapier 0.36.1 travel row. At 1 µs, 100 ns and 25 ns the control gave 66.53423, 65.67910 and 65.42266 mN·s for the initially seated 0.6-restitution strike. Corresponding outward slider speeds were 0.44739, 0.50662 and 0.52438 m/s. The measured sphere impulse approached the free-slider value as the grid narrowed. At 25 ns the inferred whole-step stop reaction was −0.23867 mN·s. The independent value including drive is −17.32430 mN·s.

The local solver at `6b202dc` couples the loaded travel limit to the point contact in the native constraint solve. With the upper stop initially loaded, the measured 0.6-restitution strike delivered 72.99319 mN·s on each grid. The wrist left at −6.95862 rad/s and the slider stayed seated. The inferred whole-step stop impulse at 1 µs was −17.32625 mN·s, including −0.00200 mN·s from the drive held for that microsecond. Reversing the sphere with the drive still held let the finger leave the stop. Fully plastic and fully elastic initially seated strikes gave 45.62074 and 91.24148 mN·s, respectively. Both mirrors reproduced these results. [Paired measurements and independent solutions](powered-slider-impact.json)

| 1 µs grid, restitution 0.6 | Native sphere impulse (mN·s) | Inferred whole-step stop impulse (mN·s) | Wrist at tick end (rad/s) | Slider at tick end (m/s) |
| --- | ---: | ---: | ---: | ---: |
| Seated, strike into stop | 72.993188 | −17.326251 | −6.958617 | 0 |
| Seated, strike away | 65.317485 | −0.001000 | +2.847826 | −0.531673 |
| 15 mm clearance | 65.222568 | <0.000001 in magnitude | −2.814264 | +0.535531 |
| 50 nm clearance, seated before strike | 72.993182 | −17.326246 | −6.958617 | <0.000001 in magnitude |
| 2 µm clearance, seated after strike | 65.222568 | <0.000001 in magnitude | −2.814264 | +0.535531 |
| −2 N pulling away from the seat | 65.411515 | <0.000001 in magnitude | −2.880914 | +0.527878 |

The reverse-drive case exposes the timing limit. At 1 µs the sphere impulse was accurate to 0.000014%, but the first tick reporting sphere contact ended at 67 µs with the finger still moving at 0.52788 m/s. The independent endpoint was −0.000006 m/s because it already included the second contact. The stop reaction appeared at 68 µs. At 100 ns and 25 ns the two events separated and the stop was reported at 67.000 and 66.925 µs. For the 2 µm clearance, the corresponding three stop readings were 71.000, 70.300 and 70.175 µs.

All 60 native trials check the first impulse, published time, both joint speeds and the sphere’s net momentum to 120 µs. Each trial checks the held drive at 50 µs and the initial and subsequent stop times when present. The table below gives the largest errors on each grid. The positive-restitution trials also check the independent trajectory through 120 µs. At zero restitution the drive can transfer momentum through a continuing contact, so the one-impact ballistic trajectory applies only to the separating cases.

| Grid | Largest first-impulse error | Latest first-contact reporting delay | Wrist angle at 120 µs (µrad error) | Slider at 120 µs (nm error) | Sphere at 120 µs (nm error) |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 µs | 0.001086% | 381.51 ns | 2.163 | 194.66 | 133.02 |
| 100 ns | 0.000109% | 85.03 ns | 0.296 | 25.16 | 15.52 |
| 25 ns | 0.0000272% | 17.52 ns | 0.073 | 4.87 | 3.83 |

The final multi-slider fallback at `6b202dc` reproduced all 60 measured rows and ten independent solutions from `b393c7a` byte for byte. The independent Rust oracle and ordinary SDF test passed strict Clippy. The instrumented run covered 371/371 lines and 771/772 regions (99.87%) in these two new test files and reproduced all 60 numerical records from the uninstrumented executable. The solver implementation carries its own production coverage receipt.

## Reproduction

~~~sh
RUSTC_WRAPPER= cargo test --release --locked --no-default-features \
  -p gz-sim-scene --test powered_slider_impact -- --nocapture
~~~

The ordinary local test loads the SDF, sets the persistent force on the slider before the first advancing tick and reads contact patches and both joint states after each tick. Both mirror directions and the 1 µs, 100 ns and 25 ns grids are exercised independently.

To record the driven upper-stop fixture with both joint encoders and signed contact impulses, choose a fresh trace path and recording directory.

~~~sh
RUSTC_WRAPPER= cargo run --release --locked --no-default-features \
  -p gz-sim-cli --bin gz-sim-local-replay -- \
  examples/worlds/local_powered_slider_impact.sdf examples/replay/powered_slider_impact.json \
  --trace target/powered-slider-trace.jsonl --record target/powered-slider-recording --record-every 1
~~~
