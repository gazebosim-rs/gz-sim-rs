// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics_gpu::{STEP, SolidSphere as Body, SpherePairLane as Lane};
use rapier3d_f64::prelude::*;
pub type V = Vector;
pub type Q = Rotation;
pub const DT: f64 = 0.001;
fn rot(q: Q, spin: V, dt: f64) -> Q {
    let d = spin * dt / 2.;
    (Q::from_xyzw(d.x, d.y, d.z, 1.).normalize() * q).normalize()
}
pub fn inertia(b: &Body) -> f64 {
    0.4 * b.radius.powi(2) / b.inverse_mass
}
pub fn fixture(count: usize) -> Vec<Lane> {
    (0..count)
        .map(|i| {
            let select = |s: usize, o: usize| ((i * s + o) % 2048) as f64 / 2047.;
            let theta = 2. * std::f64::consts::PI * select(883, 315);
            // The first three archived lanes use a horizontal impact axis and weak
            // gravity so that the approaching shapes and their contact are visible.
            let direction = if i < 3 {
                V::X
            } else {
                V::new(theta.cos(), theta.sin(), -0.3 + 0.6 * select(683, 437)).normalize()
            };
            let side = V::new(-direction.y, direction.x, 0.).normalize();
            let vertical = direction.cross(side);
            let ra = 0.095 + 0.065 * select(157, 117);
            let rb = 0.085 + 0.085 * select(547, 471);
            let r = ra + rb;
            let mass_a = 0.45 + 1.5 * select(367, 453);
            let mass_b = if i % 8 == 0 {
                mass_a
            } else {
                0.55 + 2.2 * select(863, 223)
            };
            let speed = 1.5 + 1.6 * select(337, 985);
            let bfrac = match i % 8 {
                0 => 0.,
                7 => 1.15,
                _ => 0.12 + 0.75 * select(673, 187),
            };
            let impact_time = 0.095 + 0.19 * select(397, 423);
            let lateral = r * bfrac;
            let contact_x = (r * r - lateral * lateral).max(0.).sqrt();
            let separation = direction * (speed * impact_time + contact_x) + side * lateral;
            let center = V::new(0.04 * select(499, 119), 0.04 * select(107, 73), 0.);
            let com_v = if i < 3 {
                V::new(0.06, 0., 0.)
            } else {
                V::new(
                    -0.3 + 0.6 * select(89, 541),
                    -0.3 + 0.6 * select(887, 109),
                    0.15,
                )
            };
            let make = |k, rad, m, pos: V, vel: V, w: V| Body {
                position: pos.to_array(),
                velocity: vel.to_array(),
                angular_velocity: w.to_array(),
                orientation: Q::from_axis_angle(
                    (direction + side + vertical).normalize(),
                    select(773, k) * 1.6,
                )
                .to_array(),
                radius: rad,
                inverse_mass: 1. / m,
            };
            let a = make(
                37,
                ra,
                mass_a,
                center - separation * (mass_b / (mass_a + mass_b)),
                com_v + direction * speed * (mass_b / (mass_a + mass_b)),
                side * (-3.5 + 7. * select(743, 399))
                    + vertical * (-2. + 5. * select(281, 133))
                    + direction * 1.3,
            );
            let b = make(
                929,
                rb,
                mass_b,
                center + separation * (mass_a / (mass_a + mass_b)),
                com_v - direction * speed * (mass_a / (mass_a + mass_b)),
                side * (-4. + 8. * select(541, 771)) + vertical * (-3. + 7. * select(787, 21))
                    - direction * 2.,
            );
            let (friction, restitution) = match i % 8 {
                0 => (0., 1.),
                1 => (0.1, 0.),
                2 => (0.8, 0.),
                3 => (0., 0.4),
                4 => (0.045, 0.65),
                5 => (0.85, 0.55),
                6 => (0.22, 0.25),
                _ => (0.4, 0.85),
            };
            let gravity = if i < 3 {
                [0., 0., -0.16]
            } else {
                [
                    -1. + 2. * select(347, 787),
                    -0.5 + select(631, 89),
                    -9.81 + select(347, 333),
                ]
            };
            Lane {
                a,
                b,
                gravity,
                friction,
                restitution,
                normal_impulse: 0.,
                tangential_impulse: [0.; 3],
                contact_normal: [0.; 3],
                impact_time: -1.,
            }
        })
        .collect()
}
fn mobility(d: V, a: &Body, b: &Body, ra: V, rb: V) -> f64 {
    a.inverse_mass
        + b.inverse_mass
        + ra.cross(d).length_squared() / inertia(a)
        + rb.cross(d).length_squared() / inertia(b)
}
/// Independent scalar reference. Solve the quadratic TOI (rather than
/// finding the perpendicular chord) and evaluate full directional point
/// mobility from the two moment arms and two inertia tensors.
pub fn analytical(s: &mut Lane) {
    advance(s, STEP.as_secs_f64(), rot);
}
/// One exact-in-time event solution and exact quaternion exponentials over
/// the specified free-motion interval. Returned impulse spans that interval.
pub fn exact_interval(s: &mut Lane, h: f64) {
    advance(s, h, |q, w, t| {
        let theta = w.length() * t;
        if theta == 0. {
            q
        } else {
            (Q::from_axis_angle(w.normalize(), theta) * q).normalize()
        }
    });
}
fn advance(s: &mut Lane, h: f64, rotation: fn(Q, V, f64) -> Q) {
    let g = V::from(s.gravity);
    let d = V::from(s.b.position) - V::from(s.a.position);
    let u = V::from(s.b.velocity) - V::from(s.a.velocity);
    let r = s.a.radius + s.b.radius;
    let c = (d.length() - r) * (d.length() + r);
    let a = u.length_squared();
    let b = d.dot(u);
    let disc = b * b - a * c;
    let mut t = h;
    let mut arm = [V::ZERO; 2];
    let mut impulse = V::ZERO;
    s.impact_time = -1.;
    s.normal_impulse = 0.;
    s.contact_normal = [0.; 3];
    s.tangential_impulse = [0.; 3];
    if a > 0. && b < 0. && disc >= 0. {
        let candidate = (c / (-b + disc.sqrt())).max(0.);
        let scale = r + d.length() + u.length() * h;
        if candidate <= h || (d + h * u).length() <= r + 64. * f64::EPSILON * scale {
            let candidate = candidate.min(h);
            let n = (d + u * candidate).normalize();
            let un = u.dot(n);
            if un < 0. {
                t = candidate;
                arm = [n * s.a.radius, -n * s.b.radius];
                let ja = V::from(s.a.angular_velocity).cross(arm[0]);
                let jb = V::from(s.b.angular_velocity).cross(arm[1]);
                let rel = u + jb - ja;
                let tangent = rel - rel.dot(n) * n;
                let normal = -(1. + s.restitution) * un / mobility(n, &s.a, &s.b, arm[0], arm[1]);
                let jt = if tangent.length_squared() > 0. {
                    let direction = tangent.normalize();
                    -direction
                        * (s.friction * normal)
                            .min(tangent.length() / mobility(direction, &s.a, &s.b, arm[0], arm[1]))
                } else {
                    V::ZERO
                };
                impulse = n * normal + jt;
                s.normal_impulse = normal;
                s.contact_normal = n.to_array();
                s.tangential_impulse = jt.to_array();
                s.impact_time = candidate;
            }
        }
    }
    for (body, j, r) in [(&mut s.a, -impulse, arm[0]), (&mut s.b, impulse, arm[1])] {
        let v = V::from(body.velocity);
        let w = V::from(body.angular_velocity);
        let dv = j * body.inverse_mass;
        let spin = w + r.cross(j) / inertia(body);
        body.position =
            (V::from(body.position) + h * v + 0.5 * h * h * g + (h - t) * dv).to_array();
        body.velocity = (v + h * g + dv).to_array();
        body.angular_velocity = spin.to_array();
        body.orientation =
            rotation(rotation(Q::from_array(body.orientation), w, t), spin, h - t).to_array();
    }
}

pub struct Native {
    world: PhysicsWorld,
    handles: [RigidBodyHandle; 2],
    colliders: [ColliderHandle; 2],
    ground: Option<(ColliderHandle, V)>,
    substeps: usize,
    pub support_impulses: [f64; 2],
    pub state: Lane,
    pub impulse_b: V,
    pub native_impact: Option<usize>,
    pub device_impact: Option<usize>,
    /// Maximum difference in slip impulse between Coulomb friction capped by
    /// the compression impulse and friction capped by the total Newton impulse.
    pub cap_difference: f64,
}
impl Native {
    pub fn new(s: Lane) -> Self {
        Self::world(s, 16, None)
    }
    pub fn with_support(s: Lane, normal: V, offset: f64, substeps: usize) -> Self {
        Self::world(s, substeps, Some((normal, offset)))
    }
    fn world(s: Lane, substeps: usize, plane: Option<(V, f64)>) -> Self {
        let mut world = PhysicsWorld::new();
        world.gravity = V::from(s.gravity);
        world.integration_parameters.dt = DT / substeps as f64;
        world.integration_parameters.normalized_allowed_linear_error = 1e-7;
        world.integration_parameters.normalized_prediction_distance = 1e-6;
        world.integration_parameters.contact_recycling = false;
        let ground = plane.map(|(n, offset)| {
            let (_, collider) = world.insert(
                RigidBodyBuilder::fixed().translation(n * offset),
                ColliderBuilder::new(SharedShape::halfspace(n))
                    .friction(0.)
                    .friction_combine_rule(CoefficientCombineRule::Min)
                    .restitution(0.)
                    .restitution_combine_rule(CoefficientCombineRule::Min),
            );
            (collider, n)
        });
        let spheres = [s.a, s.b].map(|b| {
            world.insert(
                RigidBodyBuilder::dynamic()
                    .can_sleep(false)
                    .pose(Pose::from_parts(
                        V::from(b.position),
                        Q::from_array(b.orientation),
                    ))
                    .linvel(V::from(b.velocity))
                    .angvel(V::from(b.angular_velocity)),
                ColliderBuilder::ball(b.radius)
                    .mass(1. / b.inverse_mass)
                    .friction(s.friction)
                    .restitution(s.restitution),
            )
        });
        Self {
            world,
            handles: spheres.map(|v| v.0),
            colliders: spheres.map(|v| v.1),
            ground,
            substeps,
            support_impulses: [0.; 2],
            state: s,
            impulse_b: V::ZERO,
            native_impact: None,
            device_impact: None,
            cap_difference: 0.,
        }
    }
    pub fn impulse_a(&mut self, j: [f64; 3]) {
        self.world.bodies[self.handles[0]].apply_impulse(V::from(j), true);
    }
    pub fn step(&mut self) {
        let old = V::from(self.state.b.velocity);
        self.support_impulses = [0.; 2];
        for _ in 0..self.substeps {
            self.world.step();
            if let Some((ground, _)) = self.ground {
                for (slot, collider) in self.support_impulses.iter_mut().zip(self.colliders) {
                    if let Some(pair) = self.world.narrow_phase.contact_pair(ground, collider) {
                        *slot += pair.total_impulse_magnitude();
                    }
                }
            }
        }
        for (b, h) in [&mut self.state.a, &mut self.state.b]
            .into_iter()
            .zip(self.handles)
        {
            let rb = &self.world.bodies[h];
            b.position = rb.translation().to_array();
            b.velocity = rb.linvel().to_array();
            b.angular_velocity = rb.angvel().to_array();
            b.orientation = rb.rotation().to_array();
        }
        self.impulse_b = (V::from(self.state.b.velocity) - old - DT * self.world.gravity)
            / self.state.b.inverse_mass;
        if let Some((_, normal)) = self.ground {
            self.impulse_b -= normal * self.support_impulses[1];
        }
    }
}
