// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec3;
use gz_physics_xpbd::{Boundary, BoundaryPath, Particle, SoftBody, SolverConfig};
use std::time::Duration;

struct Parabola {
    origin: DVec3,
    dir: DVec3,
    amplitude: f64,
    dt: f64,
    knot: Duration,
    valid: bool,
}
impl BoundaryPath for Parabola {
    fn center(&self, t: Duration) -> [f64; 3] {
        let q = t.as_secs_f64() / self.dt;
        (self.origin + self.dir * (4. * self.amplitude * q * (1. - q))).to_array()
    }
    fn mean_velocity(&self, t: Duration, h: Duration) -> [f64; 3] {
        let mid = (t.as_secs_f64() + h.as_secs_f64() * 0.5) / self.dt;
        (self.dir * (4. * self.amplitude / self.dt * (1. - 2. * mid))).to_array()
    }
    fn acceleration_bound(&self, _: Duration) -> f64 {
        if self.valid {
            8. * self.amplitude / (self.dt * self.dt)
        } else {
            f64::INFINITY
        }
    }
    fn until_knot(&self, t: Duration) -> Option<Duration> {
        (t < self.knot).then(|| self.knot - t)
    }
}
fn body(p: DVec3, m: f64) -> SoftBody {
    SoftBody::new(
        vec![Particle {
            position: p.to_array(),
            velocity: [0.; 3],
            inverse_mass: 1. / m,
        }],
        vec![],
        SolverConfig {
            gravity: [0.; 3],
            substeps: 1,
            iterations: 1,
            particle_radius: 0.006,
            friction: 0.,
            self_contact: false,
        },
    )
    .unwrap()
}
#[test]
fn bounded_curved_sweeps_converge_to_closed_form_release_and_match_the_reaction_moment() {
    let mut state = 1249801234u64;
    let mut random = || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
        (state >> 11) as f64 / 9007199254740992.
    };
    for i in 0..180 {
        let duration = Duration::from_nanos(5_000_000 + (random() * 70_000_000.) as u64);
        let dt = duration.as_secs_f64();
        let dir = DVec3::new(random() - 0.5, random() - 0.5, random() - 0.5).normalize();
        let origin = DVec3::new(random(), random(), random());
        let amp = 0.004 + random() * 0.016;
        let path = Parabola {
            origin,
            dir,
            amplitude: amp,
            dt,
            knot: Duration::from_nanos((duration.as_nanos() * 313 / 1000) as u64),
            valid: true,
        };
        let gap = amp * (0.10 + 0.73 * random());
        let m = 0.04 + random() * 0.25;
        let radius = 0.035;
        let axis = dir.any_orthonormal_vector();
        let arm = if i % 2 == 0 {
            DVec3::ZERO
        } else {
            axis * (0.025 + 0.075 * random())
        };
        let start = origin + dir * (gap + radius + 0.006) + arm;
        let b = if i % 2 == 0 {
            Boundary::Sphere {
                center: origin.to_array(),
                radius,
                velocity: [0.; 3],
                angular_velocity: [0.; 3],
            }
        } else {
            Boundary::Capsule {
                center: origin.to_array(),
                axis: axis.to_array(),
                radius,
                half_length: 0.2,
                velocity: [0.; 3],
                angular_speed: 0.,
            }
        };
        let touch = 0.5 * dt * (1. - (1. - gap / amp).sqrt());
        let speed = 4. * amp / dt * (1. - 2. * touch / dt);
        let mut clear = body(start, m);
        clear.step(duration, &[b]).unwrap();
        assert_eq!(clear.particles()[0].velocity, [0.; 3]);
        for tolerance in [1e-5, 1e-7] {
            let mut solved = body(start, m);
            solved
                .step_with_boundary_paths(duration, &[b], &[(0, &path)], tolerance)
                .unwrap();
            let p = solved.particles()[0];
            let r = solved.last_boundary_impulses()[0];
            let v = DVec3::from(p.velocity);
            let a = path.acceleration_bound(Duration::ZERO);
            let v_error = (8. * a * tolerance).sqrt();
            assert!(
                v.distance(dir * speed) < v_error + 1e-8,
                "{i} observed={v:?}, reference={speed}, bound={v_error}"
            );
            assert!(
                DVec3::from(p.position).distance(start + dir * speed * (dt - touch))
                    < dt * v_error + 10. * tolerance
            );
            assert!((DVec3::from(r.linear) + m * v).length() < 1e-10);
            assert!(
                (DVec3::from(r.angular) + (DVec3::from(p.position) - origin).cross(m * v)).length()
                    < 1e-9
            );
        }
        let mut miss = body(origin + arm + dir * (radius + 0.006 + amp + 1e-5), m);
        miss.step_with_boundary_paths(duration, &[b], &[(0, &path)], 1e-6)
            .unwrap();
        assert_eq!(miss.last_boundary_impulses()[0].linear, [0.; 3]);
    }
}
#[test]
fn invalid_curve_schedule_keeps_previously_published_particles_and_reactions() {
    let dt = Duration::from_millis(20);
    let mut path = Parabola {
        origin: DVec3::ZERO,
        dir: DVec3::Z,
        amplitude: 0.012,
        dt: dt.as_secs_f64(),
        knot: dt / 3,
        valid: true,
    };
    let b = Boundary::Sphere {
        center: [0.; 3],
        radius: 0.08,
        velocity: [0.; 3],
        angular_velocity: [0.; 3],
    };
    let mut s = body(DVec3::Z * 0.09, 0.1);
    s.step_with_boundary_paths(dt, &[b], &[(0, &path)], 1e-6)
        .unwrap();
    let particles = s.particles().to_vec();
    let reactions = s.last_boundary_impulses().to_vec();
    let steps = s.last_path_substeps();
    assert!(steps > 0);
    for (index, tol) in [(1, 1e-6), (0, 0.), (0, 1e-25), (0, f64::NAN)] {
        assert!(
            s.step_with_boundary_paths(dt, &[b], &[(index, &path)], tol)
                .is_err()
        );
    }
    assert!(
        s.step_with_boundary_paths(dt, &[b], &[(0, &path), (0, &path)], 1e-6)
            .is_err()
    );
    path.valid = false;
    assert!(
        s.step_with_boundary_paths(dt, &[b], &[(0, &path)], 1e-6)
            .is_err()
    );
    assert_eq!(s.particles(), particles);
    assert_eq!(s.last_boundary_impulses(), reactions);
    assert_eq!(s.last_path_substeps(), steps);
}
