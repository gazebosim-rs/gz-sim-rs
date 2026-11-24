<!-- Copyright (C) 2026 Open Source Robotics Foundation -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# How a hinged wrist shares a finger impact

A 200 g sphere approaches a free 80 g sliding finger at 0.75 m/s across a 50 µm gap. It strikes 20 mm above the finger’s center. A passive wrist hinge lets the finger and its guide rotate about a fixed vertical axis. The sphere gives a single frictionless contact with a known lever arm. Both shapes have restitution 0.6 and gravity is zero. The mirrored release reverses the impulse and wrist rotation. [Left native replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10099%2F) · [Right native replay](http://127.0.0.1:8082/?backend=primitive&ui=full&quality=low&replay=1&lifecycle=1&url=ws%3A%2F%2F127.0.0.1%3A10100%2F) · [Portable recordings](hinged-jaw-replays.tar.xz)


The passive native tree now reproduces this predicted impulse for both directions of impact. [Corrected solver and measured cost](reduced-wrist-slider.md)

## Independent calculation

Use wrist angle θ and slider displacement s. In the wrist frame the finger’s center is at (s, L), with L = 0.1 m and initially s = 0. Its contact point is at height L + d, where d = 0.02 m. The hinge passes through the wrist center of mass. Let I be the sum of the finger and wrist inertias about their own vertical center axes, m the finger mass, M the sphere mass, and J the signed horizontal impulse on the finger. For positive approach the incoming sphere moves along +X.

At impact, the generalized mass matrix has first row [I + mL², −mL] and second row [−mL, m]. The generalized impulse is [−(L + d)J, J]. Solving the two equations and applying normal restitution e gives

~~~text
hinge speed        ω = −dJ / I
slider speed       u = J/m + Lω
impact impulse     J = (1 + e) v_in / (1/M + 1/m + d²/I)
sphere exit speed    = v_in − J/M
~~~

Here I = 0.0004587533333 kg·m². Contact occurs at 66.666667 µs. The ideal rigid joints give **65.317041 mN·s** on the finger, a common spin of **−2.847589 rad/s**, slider speed **0.531704 m/s** and sphere exit speed **0.423415 m/s**.

After separation, the independent calculation integrates the unloaded mechanism using ω̇ = msω(Lω − 2u)/(I + ms²) and u̇ = Lω̇ + sω². The sphere keeps its outgoing velocity. Fourth-order Runge–Kutta steps of at most 20 µs preserve kinetic energy and angular momentum about the hinge within 10⁻¹³ J and 10⁻¹³ N·m·s over 24 ms in all six independently checked offsets and directions. Both bodies clear each other at the sampled post-impact times. No native contact report enters this calculation.

## Ordinary impulse-joint baseline

Both mirrors gave the same impulse magnitude. First load is the end of the first public native solve reporting a positive normal impulse. The trajectory column is the largest sampled finger or sphere center error relative to ideal rigid joints over the stated duration.

| Native step | First load | Total impulse | Shortfall | Duration | Trajectory error |
| --- | ---: | ---: | ---: | ---: | ---: |
| 10 µs | 70 µs | 57.3716 mN·s | 12.16% | 24 ms | 2.383 mm |
| 1 µs | 67 µs | 54.6363 mN·s | 16.35% | 24 ms | 3.202 mm |
| 100 ns | 66.700 µs | 43.7197 mN·s | 33.07% | 4 ms | 1.062 mm |
| 25 ns | 66.675 µs | 41.0389 mN·s | 37.17% | 1 ms | 0.283 mm |
| 1 ns | 66.667 µs | 40.8237 mN·s | 37.50% | 0.2 ms | 0.041 mm |

At 1 ns, the first native impulse was 40.0856 mN·s. Further contacts raised the total through 0.2 ms to the value shown. The finger first spun at −24.9251 rad/s while the wrist spun at −0.004568 rad/s. At 10 µs, using the measured impulse as the starting condition of a second independent trajectory calculation reduced its 24 ms position discrepancy from 2.383 mm to **0.529 µm**. This locates most of the trajectory error in the amount transferred at impact. The full off-center gripper has additional contacts, joints and actuator forces. [All measured impulses, states and errors](hinged-jaw-oracle.json)

## Controls

- **Known point impulse.** A 0.04 N·s input applied to the finger over 100 ns provided a known moment of −0.0008 N·m·s about its center. Immediately after the input the finger spun at −13.528 rad/s and the wrist at −0.858 rad/s. At 100 µs both spun at −1.743856 rad/s, within 4.0 × 10⁻¹⁰ rad/s of the independent trajectory for the positive offset. Mirroring the direction reversed both rates. Reversing only the lever arm reversed the rotation. The centered input moved the slider at 0.5 m/s with zero spin.
- **Detached finger.** Removing the slider lets the finger rotate independently. Replacing I by the finger inertia in the impact equation predicts 40.045068 mN·s. Both mirrored 25 ns collisions match to within 3 × 10⁻¹⁷ N·s. The wrist stays still.
- **Centered hit and miss.** An attached centered hit at 1 µs transferred 68.571429 mN·s with zero spin. A sphere passing 70 mm from the finger center produced zero impulse and left both links stationary.

The ordinary native recordings cover 200 ms on each side. The quantitative coupled-trajectory comparison above covers the shorter intervals listed in the table. Four Rust regressions and strict Clippy passed. This qualification adds the SDF world and physical controls.

~~~sh
RUSTC_WRAPPER= cargo test --release --locked --no-default-features \
  -p gz-sim-scene --test hinged_jaw_impact -- --nocapture
RUSTC_WRAPPER= cargo run --release --locked --no-default-features \
  -p gz-sim-cli --bin gz-sim-rs -- examples/worlds/local_hinged_jaw_impact.sdf \
  --physics-engine multifidelity --iterations 20000
~~~
