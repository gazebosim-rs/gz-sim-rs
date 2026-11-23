// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::trajectory::{PathEnd, TranslationSpline, Waypoint};
use std::time::Duration;
fn poly(t: f64) -> ([f64; 3], [f64; 3], [f64; 3]) {
    // p = 2t-3t²+t³ about one fixed spatial axis; p'(T=1)=-1.
    let g = |k: f64| [k, -1.7 * k, 0.3 * k];
    (
        g(t * (2. + t * (-3. + t))),
        g(2. + t * (-6. + 3. * t)),
        g(-6. + 6. * t),
    )
}
#[test]
fn analytical_derivatives_and_interior_knot_times() {
    let to = |time: Duration| {
        let (p, v, _) = poly(time.as_secs_f64());
        Waypoint {
            time,
            displacement: p,
            velocity: v,
        }
    };
    let mut points: Vec<_> = [0, 37, 211, 691, 1000]
        .map(|ms| to(Duration::from_millis(ms)))
        .to_vec();
    points.push(Waypoint {
        time: Duration::from_secs(2),
        displacement: [1.; 3],
        velocity: [0.; 3],
    });
    let spline = TranslationSpline::new(&points, PathEnd::Hold).unwrap();
    for j in 0..100000 {
        let t = Duration::from_nanos(j * 9999);
        let (p, v, a) = poly(t.as_secs_f64());
        let s = spline.sample(t);
        for i in 0..3 {
            assert!((s.displacement[i] - p[i]).abs() < 3e-15);
            assert!((s.velocity[i] - v[i]).abs() < 1e-14);
            assert!((s.acceleration[i] - a[i]).abs() < 6e-13);
        }
    }
    for p in points {
        assert_eq!(spline.sample(p.time).displacement, p.displacement);
    }
    assert_eq!(spline.sample(Duration::MAX).velocity, [0.; 3]);
    assert_eq!(
        spline.until_knot(Duration::from_millis(40)),
        Some(Duration::from_millis(171))
    );
}
#[test]
fn repeat_uses_integer_nanoseconds_and_admits_only_closed_paths() {
    let w = |t, x, v| Waypoint {
        time: Duration::from_nanos(t),
        displacement: [x, 0., 0.],
        velocity: [v, 0., 0.],
    };
    let mut points = [w(0, 0., 0.), w(1000003, 0.04, 0.), w(3000021, 0., 0.)];
    let path = TranslationSpline::new(&points, PathEnd::Repeat).unwrap();
    let phase = Duration::from_nanos((Duration::MAX.as_nanos() % 3000021) as u64);
    assert_eq!(path.sample(Duration::MAX), path.sample(phase));
    assert_eq!(
        path.until_knot(Duration::from_nanos(3000020)),
        Some(Duration::from_nanos(1))
    );
    assert_eq!(path.sample(path.duration()), path.sample(Duration::ZERO));
    points[2].velocity[0] = 1.;
    assert!(TranslationSpline::new(&points, PathEnd::Hold).is_err());
    assert!(TranslationSpline::new(&points, PathEnd::Repeat).is_err());
    points[2] = points[0];
    assert!(TranslationSpline::new(&points, PathEnd::Repeat).is_err());
}
#[test]
fn local_acceleration_bounds_enclose_vectors_through_knots_repeat_and_held_tails() {
    let mut pts = Vec::new();
    for i in 0..=8 {
        let t = Duration::from_micros(i * 10_003);
        let (x, v) = if i == 0 || i == 8 {
            ([0.; 3], [0.; 3])
        } else {
            let a = i as f64;
            ([a.sin(), (0.3 * a).cos(), 1. / a], [a, -a.cos(), a.sin()])
        };
        pts.push(Waypoint {
            time: t,
            displacement: x,
            velocity: v,
        });
    }
    for policy in [PathEnd::Repeat, PathEnd::Hold] {
        let spline = TranslationSpline::new(&pts, policy).unwrap();
        for i in 0..4000 {
            let at = Duration::from_nanos(i * 49_979);
            let step = spline
                .until_knot(at)
                .unwrap_or(Duration::from_millis(1))
                .min(Duration::from_micros(739));
            let bound = spline.acceleration_bound_over(at, step).unwrap();
            assert!(bound <= spline.acceleration_bound() * (1. + 3e-15));
            for k in 0..31 {
                let a = spline.sample(at + step * k / 31).acceleration;
                assert!(a[0].hypot(a[1]).hypot(a[2]) <= bound * (1. + 3e-15));
            }
            if let Some(knot) = spline.until_knot(at) {
                assert!(
                    spline
                        .acceleration_bound_over(at, knot + Duration::from_nanos(1))
                        .is_none()
                );
            } else {
                assert_eq!(bound, 0.);
            }
        }
    }
}

#[test]
fn a_one_nanosecond_contact_step_recovers_deck_velocity_below_position_resolution() {
    let time = Duration::from_secs;
    let far = 2.0_f64.powi(52);
    let points = [(0, 0., 0.), (1, far, 1.), (2, far, 0.)].map(|(t, x, v)| Waypoint {
        time: time(t),
        displacement: [x, 0., 0.],
        velocity: [v, 0., 0.],
    });
    let path = TranslationSpline::new(&points, PathEnd::Hold).unwrap();
    let at = time(1);
    let dt = Duration::from_nanos(1);
    // The second cubic is p(u) = far + u(1-u)^2, in metres.
    assert_eq!(
        path.sample(at).displacement,
        path.sample(at + dt).displacement
    );
    let velocity = path.mean_velocity(at, dt).unwrap();
    let expected = (1. - dt.as_secs_f64()).powi(2);
    assert!((velocity[0] - expected).abs() < 3e-16);
    assert_eq!(&velocity[1..], &[0., 0.]);
    assert_eq!(path.mean_velocity(at, Duration::ZERO), None);
    assert_eq!(path.mean_velocity(at - dt, 2 * dt), None);
    assert_eq!(path.mean_velocity(time(2), dt), Some([0.; 3]));
    assert_eq!(path.waypoints().collect::<Vec<_>>(), points);
    assert_eq!(path.end_policy(), PathEnd::Hold);
    assert!(path.extent()[0] >= far);
    assert!(path.speed_bound() >= far);
}
