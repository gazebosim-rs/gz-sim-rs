// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
fn model() -> PlanarSoleRocking {
    PlanarSoleRocking {
        mass: 2.,
        roll_inertia: 0.05,
        half_width: 0.1,
        center_height: 0.13,
        gravity: 9.81,
    }
}
#[test]
fn gravity_quadrature_conserves_energy_and_integrates_bearing_force() {
    let m = model();
    let v = 1.25;
    let c = m.cycle(v).unwrap().unwrap();
    let initial = 0.5 * m.m0() * v * v;
    let t = 2. * c.half_period;
    let n = 100;
    let mut j = 0.;
    for k in 0..=n {
        let s = c.sample(t * f64::from(k) / f64::from(n)).unwrap();
        let pot = m.mass
            * m.gravity
            * (m.half_width * s.angle.sin() - 2. * m.center_height * (0.5 * s.angle).sin().powi(2));
        let kinetic =
            0.5 * (m.roll_inertia * s.angular_speed.powi(2) + m.mass * s.normal_velocity.powi(2));
        assert!((initial - pot - kinetic).abs() < 3e-15);
        assert!(s.supporting_force > 0.);
        let weight = if k == 0 || k == n {
            1.
        } else if k % 2 == 0 {
            2.
        } else {
            4.
        };
        j += s.supporting_force * weight;
    }
    j *= t / (3. * f64::from(n));
    assert!((j - m.mass * (m.gravity * t - 2. * m.half_width * v)).abs() < 1e-9);
}
#[test]
fn time_limit_encloses_independent_cycle_sum_and_closed_form_tail() {
    for moment in [0.01, 0.021, 0.055, 0.16] {
        let m = PlanarSoleRocking {
            roll_inertia: moment,
            ..model()
        };
        let mut v = 0.9;
        let r = m.speed_ratio().unwrap();
        let bounds = m.settling_limit(v, 5e-9).unwrap().unwrap();
        let mut clock = 0.;
        let mut all_j = 0.;
        let mut suffix_j = 0.;
        for k in 0..1000 {
            if v < 1e-15 {
                break;
            }
            let c = m.cycle(v).unwrap().unwrap();
            clock += 2. * c.half_period;
            let j = m.mass * m.half_width * v * (1. + r);
            all_j += j;
            if k >= bounds.resolved_cycles {
                suffix_j += j;
            }
            v *= r;
        }
        let [a, b] = bounds.time_bounds();
        assert!(a <= clock && clock <= b, "{clock} vs [{a},{b}]");
        assert!(b - a < 6e-9);
        assert!((all_j - m.mass * m.half_width * 0.9 * (1. + r) / (1. - r)).abs() < 1e-13);
        assert!((bounds.remaining_impact_impulse - suffix_j).abs() < 1e-14);
    }
}
#[test]
fn tiny_oscillations_retain_their_energy_and_unsafe_rates_decline() {
    let m = model();
    let speed = 1e-9;
    let c = m.cycle(speed).unwrap().unwrap();
    let tt = m.m0() * speed / (m.mass * m.half_width * m.gravity);
    assert!((c.half_period / tt - 1.).abs() < 1e-13);
    let v = c.sample(0.4 * c.half_period).unwrap();
    assert!((v.angular_speed / speed - 0.6).abs() < 2e-14);
    assert!(m.cycle(100.).unwrap().is_none());
    assert!(m.cycle(-1.).is_err());
}
#[test]
fn crossing_bearing_switches_and_its_accumulation_commutes_with_restarting() {
    for r in [0.018, 0.045, 0.09] {
        let m = PlanarSoleRocking {
            roll_inertia: r,
            ..model()
        };
        let initial = 0.7;
        let c = m.cycle(initial).unwrap().unwrap();
        let x = c.sample(c.half_period * 1.62).unwrap();
        let t = 0.38 * c.half_period;
        let mid = m
            .advance(x.angle, x.angular_speed, 1, t * 0.42)
            .unwrap()
            .unwrap();
        assert!(mid.first_impact.is_none());
        let total = 0.21;
        let a = m
            .advance(x.angle, x.angular_speed, 1, total)
            .unwrap()
            .unwrap();
        let b = m
            .advance(mid.signed_angle, mid.angular_rate, 1, total - t * 0.42)
            .unwrap()
            .unwrap();
        assert!(a.settled && b.settled);
        let e = a.first_impact.unwrap();
        assert!((e.seconds - t).abs() < 5e-14);
        assert!(e.numerical_time_bounds[0] <= t && t <= e.numerical_time_bounds[1]);
        let ratio = m.speed_ratio().unwrap();
        let sum = m.mass * m.half_width * initial * (1. + ratio) / (1. - ratio);
        assert!((sum - a.integrated_impact_impulse).abs() < 1e-12);
        assert_eq!(a.includes_infinite_tail, ratio > 0.);
        assert!(
            (mid.integrated_normal_impulse + b.integrated_normal_impulse
                - a.integrated_normal_impulse)
                .abs()
                < 1e-12
        );
        assert_eq!(b.angular_rate, 0.);
    }
}
