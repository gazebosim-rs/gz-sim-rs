// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
#[path = "support/powered_slider_oracle.rs"]
mod oracle;

use bevy_ecs::prelude::{Entity, EntityRef};
use glam::DVec2;
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, JointSample, Name, PhysicsEngine, load_world_with_sdf_step, set_joint_force,
};
use oracle::{CASES, Case, Hit, I, L, M, MP, SPEED};
use serde_json::json;

const WORLD: &str = include_str!("../../../examples/worlds/local_powered_slider_impact.sdf");

fn entity<'w>(sim: &'w Simulation, name: &str) -> EntityRef<'w> {
    #[allow(deprecated)]
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
        .unwrap()
}

struct Rig {
    sim: Simulation,
    finger: BodyId,
    sphere: BodyId,
    hinge: Entity,
    slide: Entity,
}
impl Rig {
    fn new(c: Case, mirror: f64, ns: u32, restitution: f64) -> Self {
        let (lo, hi) = if mirror > 0. {
            (-0.04, c.upper)
        } else {
            (-c.upper, 0.04)
        };
        let sign = c.approach * mirror;
        let xml = WORLD
            .replace(">0.6<", &format!(">{restitution}<"))
            .replace(
                "<lower>-0.04</lower><upper>0</upper>",
                &format!("<lower>{lo}</lower><upper>{hi}</upper>"),
            )
            .replace("-0.02405 0.12", &format!("{} 0.12", -sign * 0.02405))
            .replace(">0.75 0 0<", &format!(">{} 0 0<", sign * SPEED))
            .replace(">0.000001<", &format!(">{:.9}<", f64::from(ns) * 1e-9));
        let mut sim = load_world_with_sdf_step(
            &xml,
            "powered-slider.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::MultiFidelity,
        )
        .unwrap();
        let finger = entity(&sim, "default::wrist_rig::finger")
            .get::<BodyHandle>()
            .unwrap()
            .id();
        let sphere = entity(&sim, "default::parcel::body")
            .get::<BodyHandle>()
            .unwrap()
            .id();
        let hinge = entity(&sim, "default::wrist_rig::hinge").id();
        let slide = entity(&sim, "default::wrist_rig::slider").id();
        let id = sim.world().get::<EntityId>(slide).unwrap().get();
        set_joint_force(&mut sim, id, mirror * c.force).unwrap();
        Self {
            sim,
            finger,
            sphere,
            hinge,
            slide,
        }
    }
    fn state(&self, id: BodyId) -> BodyState {
        self.sim
            .world()
            .resource::<PhysicsBackend>()
            .state(id)
            .unwrap()
    }
    fn joint(&self, id: Entity) -> (f64, f64) {
        let q = self.sim.world().get::<JointSample>(id).unwrap().state;
        (q.position, q.velocity)
    }
    fn moving(&self, mirror: f64) -> Reading {
        let (theta, w) = self.joint(self.hinge);
        let (s, u) = self.joint(self.slide);
        let finger = self.state(self.finger);
        let x = DVec2::from_angle(theta * mirror);
        let p = M * DVec2::new(finger.velocity[0] * mirror, finger.velocity[1]).dot(x);
        Reading {
            theta: theta * mirror,
            s: s * mirror,
            w: w * mirror,
            u: u * mirror,
            p,
        }
    }
    fn reaction(&self, mirror: f64) -> DVec2 {
        let backend = self
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let mut impulse = DVec2::ZERO;
        for patch in backend.rigid_contact_patches() {
            assert!(
                [patch.a, patch.b].contains(&self.sphere)
                    && [patch.a, patch.b].contains(&self.finger)
            );
            let sign = if patch.a == self.finger { 1. } else { -1. };
            let n = DVec2::new(patch.normal_on_a[0] * mirror, patch.normal_on_a[1]);
            for point in &patch.points {
                impulse += sign * n * point.normal_impulse;
            }
        }
        impulse
    }
}
#[derive(Clone, Copy, Default)]
struct Reading {
    theta: f64,
    s: f64,
    w: f64,
    u: f64,
    p: f64,
}

fn cases() -> impl Iterator<Item = (Case, f64)> {
    CASES.into_iter().map(|case| (case, 0.6)).chain(
        [CASES[0], CASES[1]]
            .into_iter()
            .flat_map(|case| [(case, 0.), (case, 1.)]),
    )
}
#[test]
fn closed_form_seated_mass_and_release_satisfy_unilateral_impulse_balance() {
    for (c, restitution) in cases() {
        let h = Hit::solve(c, restitution);
        assert!((60e-6..75e-6).contains(&h.time));
        assert!(h.approach_velocity > 0.);
        let sphere_exit = DVec2::new(c.approach * SPEED, 0.) - h.impulse * h.normal / MP;
        let y_at_contact = c.approach * (SPEED * h.time - 0.02405) * (-h.pre[0].sin())
            + (L + 0.02) * h.pre[0].cos();
        let point_exit = c.approach * (h.post[3] - y_at_contact * h.post[2]);
        assert!(
            (point_exit - sphere_exit.dot(h.normal) - restitution * h.approach_velocity).abs()
                < 1e-12
        );
        assert!(
            (M * ((h.post[3] - L * h.post[2]) - (h.pre[3] - L * h.pre[2]))
                - c.approach * h.impulse
                - h.stop_impulse)
                .abs()
                < 1e-12
        );
        assert_eq!(h.blocked, matches!(c.name, "seated" | "near_before"));
        if h.blocked {
            assert_eq!(h.post[3], 0.);
            assert!(h.stop_impulse < 0.);
        } else {
            assert_eq!(h.stop_impulse, 0.);
        }
        if c.name == "interior" {
            // Before contact, the motor is internal. It adds F·s to kinetic
            // energy and no angular momentum about the world hinge.
            let [_, s, w, u] = h.pre;
            let momentum = (I + M * (L * L + s * s)) * w - M * L * u;
            let energy = 0.5 * (I * w * w + M * ((u - L * w).powi(2) + (s * w).powi(2)));
            assert!(momentum.abs() < 1e-13 && (energy - c.force * s).abs() < 1e-13);
        }
        if c.name == "seated" {
            let exact =
                (1. + restitution) * SPEED / (1. / MP + (L + 0.02).powi(2) / (I + M * L * L));
            assert!((exact - h.impulse).abs() < 1e-14);
        }
        println!(
            "{}",
            json!({"oracle":c.name,"restitution":restitution,"contact_time_ns":h.time*1e9,"J":h.impulse,"blocked":h.blocked,
            "stop_impulse":h.stop_impulse,"stop_time_ns":h.stop_time.map(|t|t*1e9),"second_stop_time_ns":h.next_stop(c).map(|t|t*1e9),"pre":h.pre,"post":h.post})
        );
    }
}

#[test]
fn driven_slider_impacts_both_sides_of_a_physical_travel_limit() {
    for (c, restitution) in cases() {
        let h = Hit::solve(c, restitution);
        for mirror in [1., -1.] {
            for ns in [1000, 100, 25] {
                let dt = f64::from(ns) * 1e-9;
                let mut r = Rig::new(c, mirror, ns, restitution);
                let mut prev = Reading::default();
                let mut peaks = 0_f64;
                let mut first = None;
                let mut later_stop = None;
                let mut prior_stop = None;
                let mut total = DVec2::ZERO;
                let mut before_50us = Reading::default();
                let ideal_50us = h.before(c, 50e-6);
                let mut expected_stop = 0.;
                let mut first_position = 0.;
                for tick in 1..=120000 / ns {
                    r.sim.step_exact(1).unwrap();
                    let end_ns = tick * ns;
                    let t = f64::from(end_ns) * 1e-9;
                    let q = r.moving(mirror);
                    let j = r.reaction(mirror);
                    total += j;
                    // Generalized slider momentum is the finger's axial COM momentum.
                    // Bound quadrature to a single native step; this is an inferred stop
                    // impulse including the continuous drive reaction during that step.
                    let drift = 0.5 * M * (prev.s * prev.w.powi(2) + q.s * q.w.powi(2)) * dt;
                    let stop = q.p
                        - prev.p
                        - c.force * dt
                        - j.dot(DVec2::from_angle((prev.theta + q.theta) / 2.))
                        - drift;
                    if first.is_some() && later_stop.is_none() && stop < -10. * c.force.abs() * dt {
                        later_stop = Some(end_ns);
                    }
                    if first.is_none() && j == DVec2::ZERO && stop < -0.1 * c.force.abs() * dt {
                        prior_stop.get_or_insert(end_ns);
                    }
                    if end_ns == 50_000 {
                        before_50us = q;
                    }
                    if first.is_none() && j.length_squared() > 0. {
                        let post = h.at_end(c, t);
                        let pre = h.before(c, t - dt);
                        let expected_drift =
                            0.5 * M * (pre[1] * pre[2].powi(2) + post[1] * post[2].powi(2)) * dt;
                        expected_stop = M * (post[3] - L * post[2] - pre[3] + L * pre[2])
                            - c.force * dt
                            - h.impulse * c.approach
                            - expected_drift;
                        first_position = q.s;
                        first = Some((end_ns, j.dot(h.normal), stop, q.w, q.u, post[2], post[3]));
                    }
                    peaks = peaks.max(q.s - c.upper);
                    prev = q;
                }
                let (end, j, stop, w, u, ew, eu) = first.unwrap();
                let ball = r.state(r.sphere);
                let dp = MP
                    * DVec2::new(
                        mirror * ball.velocity[0] - c.approach * SPEED,
                        ball.velocity[1],
                    );
                // With positive restitution the sphere separates until the end of
                // these short trials. A plastic contact may continue transferring
                // drive force, outside a single instantaneous-impact solution.
                let final_error = (restitution > 0.).then(|| {
                    let end = 120e-6;
                    let ideal = h.at_end(c, end);
                    let ball_start = DVec2::new(-c.approach * 0.02405, L + 0.02);
                    let ball_expected = ball_start + DVec2::X * (c.approach * SPEED * end)
                        - h.impulse * h.normal / MP * (end - h.time);
                    let ball_position = DVec2::new(ball.position[0] * mirror, ball.position[1]);
                    [
                        (prev.theta - ideal[0]).abs(),
                        (prev.s - ideal[1]).abs(),
                        ball_position.distance(ball_expected),
                    ]
                });
                if let Some([hinge, slide, sphere]) = final_error {
                    // Allow two public steps of contact-time uncertainty at the
                    // approach speed, expressed as arc length for the hinge.
                    let allowed = 2. * SPEED * dt;
                    assert!(L * hinge < allowed && slide < allowed && sphere < allowed);
                }
                assert!((dp + total).length() < 1e-10);
                assert!((0. ..=f64::from(ns) + 1.).contains(&(f64::from(end) - h.time * 1e9)));
                // Drive work can change the relative speed across the reporting
                // interval. Resolve the stop contribution above that bounded impulse.
                assert!(
                    (j - h.impulse).abs() < 2. * c.force.abs() * dt + 1e-10,
                    "{} e={restitution} dt={ns}: {j} vs {}",
                    c.name,
                    h.impulse
                );
                assert!((before_50us.s - ideal_50us[1]).abs() < 1e-9);
                assert!((before_50us.u - ideal_50us[3]).abs() < 1e-5);
                assert!((before_50us.w - ideal_50us[2]).abs() < 1e-4);
                assert!(peaks < 1e-9, "travel past authored stop: {peaks} m");
                assert_eq!(prior_stop.is_some(), h.stop_time.is_some());
                if let (Some(t), Some(end_ns)) = (h.stop_time, prior_stop) {
                    assert!((f64::from(end_ns) * 1e-9 - t).abs() <= dt + 1e-15);
                }
                if h.blocked {
                    assert!(u.abs() < 1e-7 && (w - ew).abs() < 2e-4);
                    assert!((stop - expected_stop).abs() < 4. * c.force.abs() * dt + 1e-10);
                } else if h.next_stop(c).is_none_or(|t| t > f64::from(end) * 1e-9) {
                    // When two events lie in one sample the reported endpoint can
                    // precede the second physical response; retain that measured error.
                    assert!((u - eu).abs() < 2e-5 && (w - ew).abs() < 2e-4);
                    assert!((stop - expected_stop).abs() < 4. * c.force.abs() * dt + 1e-10);
                }
                if let Some(next) = h.next_stop(c) {
                    // Crossing, then reporting, can occupy consecutive native steps.
                    assert!((f64::from(later_stop.unwrap()) * 1e-9 - next).abs() < 2. * dt);
                }

                println!(
                    "{}",
                    json!({
                        "case":c.name,"restitution":restitution,"mirror":mirror,"dt_ns":ns,"first_loaded_end_ns":end,"expected_time_ns":h.time*1e9,
                        "phase_error_ns":f64::from(end)-h.time*1e9,
                        "measured_impulse":j,"expected_impulse":h.impulse,"error_percent":100.*(j/h.impulse-1.),
                        "inferred_stop_impulse":stop,"analytic_collision_stop_impulse":h.stop_impulse,
                        "hinge_rad_s":w,"ideal_hinge_at_end":ew,"slider_mps":u,"ideal_slider_at_end":eu,
                        "first_position_m_normalized":first_position,"ideal_first_tick_stop_impulse":expected_stop,
                        "final_abs_error_120us":final_error,
                        "actual_50us":[before_50us.theta,before_50us.s,before_50us.w,before_50us.u],"ideal_50us":ideal_50us,
                        "max_stop_overshoot_m":peaks,"stop_loaded_before_ns":prior_stop,"stop_loaded_after_ns":later_stop,
                        "next_stop_expected_ns":h.next_stop(c).map(|t|t*1e9)
                    })
                );
            }
        }
    }
}
