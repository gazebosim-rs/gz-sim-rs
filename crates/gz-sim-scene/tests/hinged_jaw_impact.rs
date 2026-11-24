// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::DVec2;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine, load_world_with_sdf_step, queue_link_force_at_point,
};
use serde_json::json;

const WORLD: &str = include_str!("../../../examples/worlds/local_hinged_jaw_impact.sdf");
const FINGER: f64 = 0.08;
const PARCEL: f64 = 0.20;
const I_WRIST: f64 = 0.0004266666666666667;
const I_FINGER: f64 = 0.00003208666666666667;
const I: f64 = I_WRIST + I_FINGER;
const RAIL_Y: f64 = 0.10;
const SPEED: f64 = 0.75;
const GAP: f64 = 50e-6;
const FINGER_HALF_X: f64 = 0.009;
const FINGER_HALF_Y: f64 = 0.0335;
const SPHERE_RADIUS: f64 = 0.015;
const INITIAL_DISTANCE: f64 = FINGER_HALF_X + SPHERE_RADIUS + GAP;
const BALL_INERTIA: f64 = 0.4 * PARCEL * SPHERE_RADIUS * SPHERE_RADIUS;
const E: f64 = 0.6;

fn body(sim: &Simulation, name: &str) -> (u64, BodyId) {
    #[allow(deprecated)]
    let e = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap();
    (
        e.get::<EntityId>().unwrap().get(),
        e.get::<BodyHandle>().unwrap().id(),
    )
}

struct Rig {
    sim: Simulation,
    finger: u64,
    bodies: [BodyId; 3],
}
impl Rig {
    fn new(ns: u32, sign: f64, offset: f64, launch: bool, attached: bool) -> Self {
        let x = if launch {
            -sign * INITIAL_DISTANCE
        } else {
            0.3
        };
        let xml = WORLD
            .replace(">0.00001<", &format!(">{:.9}<", f64::from(ns) * 1e-9))
            .replace("-0.02405 0.12", &format!("{x:.8} {:.8}", RAIL_Y + offset))
            .replace(
                ">0.75 0 0<",
                &format!(">{} 0 0<", if launch { sign * SPEED } else { 0. }),
            );
        let xml = if attached {
            xml
        } else {
            let start = xml.find("<joint name=\"slider\"").unwrap();
            let end = start + xml[start..].find("</joint>").unwrap() + "</joint>".len();
            let mut detached = xml;
            detached.replace_range(start..end, "");
            detached
        };
        let sim = load_world_with_sdf_step(
            &xml,
            "hinged-jaw.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        let ids = [
            "default::wrist_rig::wrist",
            "default::wrist_rig::finger",
            "default::parcel::body",
        ]
        .map(|n| body(&sim, n));
        Self {
            sim,
            finger: ids[1].0,
            bodies: ids.map(|(_, b)| b),
        }
    }
    fn states(&self) -> [BodyState; 3] {
        let physics = self.sim.world().resource::<PhysicsBackend>();
        self.bodies.map(|id| physics.state(id).unwrap())
    }
    fn reaction(&self) -> (f64, f64) {
        let native = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let mut impulse = 0.;
        let mut moment = 0.;
        for patch in native.rigid_contact_patches() {
            assert!(
                [patch.a, patch.b].contains(&self.bodies[1])
                    && [patch.a, patch.b].contains(&self.bodies[2])
            );
            let on_a = patch.a == self.bodies[1];
            for p in &patch.points {
                let dir = if on_a { 1. } else { -1. };
                let dx = dir * patch.normal_on_a[0] * p.normal_impulse;
                let dy = dir * patch.normal_on_a[1] * p.normal_impulse;
                let r = if on_a { p.lever_a } else { p.lever_b };
                impulse += dx;
                moment += r[0] * dy - r[1] * dx;
            }
        }
        (impulse, moment)
    }
}

// Lagrange coordinates [hinge angle, jaw displacement, angular and slider speed].
type Q = [f64; 4];
fn post_impact(j: f64, d: f64) -> Q {
    let w = -d * j / I;
    [0., 0., w, j / FINGER + RAIL_Y * w]
}
fn derivative([_, s, w, u]: Q) -> Q {
    let alpha = FINGER * s * w * (RAIL_Y * w - 2. * u) / (I + FINGER * s * s);
    [w, u, alpha, RAIL_Y * alpha + s * w * w]
}
fn add(q: Q, k: Q, h: f64) -> Q {
    std::array::from_fn(|i| q[i] + h * k[i])
}
fn free_flight(mut q: Q, t: f64) -> Q {
    let n = (t / 0.00002).ceil() as usize;
    if n == 0 {
        return q;
    }
    let h = t / n as f64;
    for _ in 0..n {
        let a = derivative(q);
        let b = derivative(add(q, a, 0.5 * h));
        let c = derivative(add(q, b, 0.5 * h));
        let d = derivative(add(q, c, h));
        q = std::array::from_fn(|i| q[i] + h * (a[i] + 2. * b[i] + 2. * c[i] + d[i]) / 6.);
    }
    q
}
fn world_finger(q: Q) -> DVec2 {
    let (sin, cos) = q[0].sin_cos();
    DVec2::new(q[1] * cos - RAIL_Y * sin, q[1] * sin + RAIL_Y * cos)
}
fn xy(s: BodyState) -> DVec2 {
    DVec2::new(s.position[0], s.position[1])
}
fn slider_speed(s: [BodyState; 3]) -> f64 {
    let r = xy(s[1]) - xy(s[0]);
    let w = s[0].angular_velocity[2];
    let relative = DVec2::new(
        s[1].velocity[0] - s[0].velocity[0] + w * r.y,
        s[1].velocity[1] - s[0].velocity[1] - w * r.x,
    );
    let q = s[0].orientation;
    let theta = 2. * q[2].atan2(q[3]);
    relative.dot(DVec2::from_angle(theta))
}

fn angular_momentum(s: [BodyState; 3]) -> f64 {
    let f = s[1];
    I_WRIST * s[0].angular_velocity[2]
        + I_FINGER * f.angular_velocity[2]
        + FINGER * (f.position[0] * f.velocity[1] - f.position[1] * f.velocity[0])
}
#[test]
fn known_off_center_impulse_shares_slider_and_hinge_motion() {
    for (sign, offset) in [(1., 0.02), (-1., 0.02), (1., -0.02), (1., 0.)] {
        let ns = 100;
        let j = 0.04 * sign;
        let mut r = Rig::new(ns, sign, offset, false, true);
        queue_link_force_at_point(
            &mut r.sim,
            r.finger,
            [-sign * FINGER_HALF_X, offset, 0.],
            [j / (f64::from(ns) * 1e-9), 0., 0.],
        )
        .unwrap();
        r.sim.step_exact(1).unwrap();
        let s = r.states();
        let q = post_impact(j, offset);
        let early_moment = angular_momentum(s);
        r.sim.step_exact(999).unwrap();
        let relaxed = r.states();
        let reference = free_flight(q, 0.0001);
        assert_eq!(r.reaction(), (0., 0.));
        assert!((FINGER * s[1].velocity[0] - j).abs() < 1e-12);
        assert!((early_moment + (RAIL_Y + offset) * j).abs() < 1e-12);
        assert!((angular_momentum(relaxed) + (RAIL_Y + offset) * j).abs() < 1e-11);
        for link in &relaxed[..2] {
            assert!((link.angular_velocity[2] - reference[2]).abs() < 1e-8);
        }
        assert!((slider_speed(relaxed) - reference[3]).abs() < 1e-9);
        if offset == 0. {
            assert_eq!(s[0].angular_velocity, [0.; 3]);
            assert_eq!(s[1].angular_velocity, [0.; 3]);
        }
        println!(
            "{}",
            json!({
                "case":"known","sign":sign,"offset_m":offset,"J":j,
                "first_hinge_rad_s":s[0].angular_velocity[2],"first_finger_rad_s":s[1].angular_velocity[2],
                "first_slider_mps":slider_speed(s), "expected_hinge_rad_s":q[2],"expected_slider_mps":q[3],
                "relaxed_hinge_rad_s":relaxed[0].angular_velocity[2],"relaxed_finger_rad_s":relaxed[1].angular_velocity[2],
                "relaxed_slider_mps":slider_speed(relaxed),"relaxed_w_error":relaxed[0].angular_velocity[2]-reference[2],
                "relaxed_u_error":slider_speed(relaxed)-reference[3],
                "moment_error_initial":early_moment+(RAIL_Y+offset)*j,
                "moment_error_relaxed":angular_momentum(relaxed)+(RAIL_Y+offset)*j
            })
        );
    }
}

#[test]
fn sphere_impact_turns_the_hinge_and_moves_the_slider_in_both_directions() {
    for (sign, d, ns, end_ns) in [
        (1., 0.02, 10000, 24_000_000),
        (-1., 0.02, 10000, 24_000_000),
        (1., 0.02, 1000, 24_000_000),
        (-1., 0.02, 1000, 24_000_000),
        (1., 0.02, 100, 4_000_000),
        (-1., 0.02, 100, 4_000_000),
        (1., 0.02, 25, 1_000_000),
        (-1., 0.02, 25, 1_000_000),
        (1., 0.02, 1, 200_000),
        (-1., 0.02, 1, 200_000),
        (1., -0.02, 1000, 1_000_000),
        (1., 0., 1000, 1_000_000),
        (1., 0.07, 1000, 1_000_000),
    ] {
        let mut r = Rig::new(ns, sign, d, true, true);
        let hit = d.abs() <= FINGER_HALF_Y;
        let j = if hit {
            sign * (1. + E) * SPEED / (1. / PARCEL + 1. / FINGER + d * d / I)
        } else {
            0.
        };
        let mut sum = 0.;
        let mut torque = 0.;
        let mut first = None;
        let mut last = None;
        let mut peak_pos = 0_f64;
        let mut peak_w = 0_f64;
        let mut peak_u = 0_f64;
        let mut first_loaded = None;
        let mut first_j = None;
        let mut positive_steps = 0;
        let mut conditioned_position = 0_f64;
        let mut peak_moment_error = 0_f64;
        for tick in 1..=end_ns / ns {
            r.sim.step_exact(1).unwrap();
            let t_ns = tick * ns;
            let t = f64::from(t_ns) * 1e-9;
            let (impulse, moment) = r.reaction();
            sum += impulse;
            torque += moment;
            if impulse != 0. {
                first.get_or_insert(t_ns);
                first_j.get_or_insert(impulse);
                positive_steps += 1;
                last = Some(t_ns);
                first_loaded.get_or_insert_with(|| r.states());
            }
            if tick % 100 == 0 || t_ns == end_ns {
                let a = r.states();
                let flight = if hit { (t - GAP / SPEED).max(0.) } else { 0. };
                let reference = free_flight(post_impact(j, d), flight);
                let expected = world_finger(reference);
                if t >= 0.0001 {
                    let conditioned = free_flight(post_impact(sum, d), flight);
                    let ball = -sign * INITIAL_DISTANCE + sign * SPEED * t - sum / PARCEL * flight;
                    conditioned_position = conditioned_position
                        .max(xy(a[1]).distance(world_finger(conditioned)))
                        .max(xy(a[2]).distance(DVec2::new(ball, RAIL_Y + d)));
                }
                let initial_angular = -(RAIL_Y + d) * PARCEL * sign * SPEED;
                let ball_angular = PARCEL
                    * (a[2].position[0] * a[2].velocity[1] - a[2].position[1] * a[2].velocity[0])
                    + BALL_INERTIA * a[2].angular_velocity[2];
                peak_moment_error = peak_moment_error
                    .max((angular_momentum(a) + ball_angular - initial_angular).abs());
                let ball = -sign * INITIAL_DISTANCE + sign * SPEED * t - j / PARCEL * flight;
                let pe = xy(a[1])
                    .distance(expected)
                    .max(xy(a[2]).distance(DVec2::new(ball, RAIL_Y + d)));
                if t >= 0.0001 {
                    peak_w = peak_w.max((a[0].angular_velocity[2] - reference[2]).abs());
                    peak_u = peak_u.max((slider_speed(a) - reference[3]).abs());
                }
                peak_pos = peak_pos.max(pe);
            }
        }
        let s = r.states();
        assert!((sum + PARCEL * (s[2].velocity[0] - sign * SPEED)).abs() < 1e-10);
        assert!(peak_moment_error < 1e-8);
        assert!(conditioned_position < 1e-6);
        if hit {
            let expected_end = (GAP / SPEED * 1e9 / f64::from(ns)).ceil() as u32 * ns;
            assert_eq!(first, Some(expected_end));
            assert!((torque + d * sum).abs() < 1e-8);
            if d == 0. {
                assert!((sum - j).abs() < 1e-10 && peak_w < 1e-10);
            } else {
                // A solver improvement may reach the ideal attached response. The native
                // partial transfer shall remain between a free finger and the entire wrist.
                let free = (1. + E) * SPEED / (1. / PARCEL + 1. / FINGER + d * d / I_FINGER);
                assert!(sign * sum >= free - 1e-10 && sign * sum <= j.abs() + 1e-10);
                assert!(first_loaded.unwrap()[0].angular_velocity[2] * d * sign < 0.);
            }
        } else {
            assert_eq!((first, sum, torque), (None, 0., 0.));
            assert!(peak_pos < 1e-10);
        }
        println!(
            "{}",
            json!({
                "case": "collision",
                "sign": sign,
                "offset_m": d,
                "dt_ns": ns,
                "end_ns": end_ns,
                "first_ns": first,
                "last_ns": last,
                "loaded_steps": positive_steps,
                "first_impulse": first_j,
                "expected_j": j,
                "impulse": sum,
                "finger_moment": torque,
                "expected_finger_moment": -d*j,
                "peak_pos_m": peak_pos,
                "peak_angular_rate_error": peak_w,
                "peak_slide_rate_error": peak_u,
                "observed_final_vx": s.map(|x|x.velocity[0]),
                "conditioned_position_error_m": conditioned_position,
                "peak_moment_error": peak_moment_error,
                "first_finger_spin": first_loaded.map(|s|s[1].angular_velocity[2]),
                "first_hinge_speed": first_loaded.map(|s|s[0].angular_velocity[2]),
                "first_slider_speed": first_loaded.map(slider_speed)
            })
        );
    }
}

#[test]
fn detached_finger_has_its_own_rotational_impulse() {
    for sign in [1., -1.] {
        let ns = 25;
        let d = 0.02;
        let mut r = Rig::new(ns, sign, d, true, false);
        let exact = sign * (1. + E) * SPEED / (1. / PARCEL + 1. / FINGER + d * d / I_FINGER);
        let mut sum = 0.;
        for _ in 0..1_000_000 / ns {
            r.sim.step_exact(1).unwrap();
            sum += r.reaction().0;
        }
        let s = r.states();
        assert!((sum - exact).abs() < 1e-11);
        assert!((s[1].angular_velocity[2] + d * exact / I_FINGER).abs() < 1e-8);
        assert_eq!(s[0].angular_velocity, [0.; 3]);
        println!(
            "{}",
            json!({
                "case": "detached",
                "sign": sign,
                "impulse": sum,
                "expected": exact,
                "finger_rad_s": s[1].angular_velocity[2],
                "expected_finger_rad_s": -d*exact/I_FINGER,
                "hinge_rad_s": s[0].angular_velocity[2],
            })
        );
    }
}

#[test]
fn independent_free_flight_conserves_energy_and_moment_and_clears_the_sphere() {
    for side in [1., -1.] {
        for d in [-0.02, 0., 0.02] {
            let impulse = side * (1. + E) * SPEED / (1. / PARCEL + 1. / FINGER + d * d / I);
            let q = post_impact(impulse, d);
            let invariants = |[_, s, w, u]: Q| {
                let moment = (I + FINGER * (s * s + RAIL_Y * RAIL_Y)) * w - FINGER * RAIL_Y * u;
                let energy =
                    0.5 * (I * w * w + FINGER * ((u - RAIL_Y * w).powi(2) + (s * w).powi(2)));
                (moment, energy)
            };
            let initial = invariants(q);
            for k in 0..=24 {
                let t = f64::from(k) * 0.001;
                let reference = free_flight(q, t);
                let (moment, energy) = invariants(reference);
                assert!((moment - initial.0).abs() < 1e-13 && (energy - initial.1).abs() < 1e-13);
                let ball = DVec2::new(
                    -side * (FINGER_HALF_X + SPHERE_RADIUS) + (side * SPEED - impulse / PARCEL) * t,
                    RAIL_Y + d,
                );
                let center = world_finger(reference);
                let local_x = (ball - center).dot(DVec2::from_angle(reference[0]));
                assert!(-side * local_x >= FINGER_HALF_X + SPHERE_RADIUS - 1e-14);
            }
        }
    }
}
