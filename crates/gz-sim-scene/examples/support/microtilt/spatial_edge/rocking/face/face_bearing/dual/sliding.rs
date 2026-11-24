// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Verify the coupled frictional yaw/load capacity with dual spatial wrench LPs.
#[path="sliding/capture.rs"]
pub(crate) mod capture;
use super::spatial::Trial;
use super::*;
use gz_physics::CoplanarSlidingContact;
#[derive(Clone, Copy)]
struct FixtureContact {
    scene: Trial,
    mu: [f64; 2],
}
struct Dual {
    a: Vec<DVec3>,
    plot: Vec<DVec2>,
}
impl Dual {
    fn build(c: CoplanarSlidingContact, patch: [&[DVec2]; 2], inertia: DMat3) -> Self {
        // The two independent forbidden angular accelerations use the full
        // inverse inertia in world space. A feasible dual affine majorant
        // bounds every possible nonnegative spatial pressure distribution.
        let mut a = vec![];
        let mut plot = vec![];
        for k in 0..2 {
            for &p in patch[k] {
                let traction =
                    -c.kinetic_friction[k] * (c.velocity - c.pad_velocities[k]).normalize();
                let force = (-c.mass * c.gravity_normal) * traction.extend(1.);
                let alpha = inertia.inverse() * ((p - c.center).extend(-c.height).cross(force));
                a.push(DVec3::new(1., alpha.x, alpha.y));
                plot.push(DVec2::new(k as f64, alpha.z));
            }
        }
        let sx = a.iter().map(|v| v.y.abs()).fold(1f64, f64::max);
        let sy = a.iter().map(|v| v.z.abs()).fold(1f64, f64::max);
        for q in &mut a {
            q.y /= sx;
            q.z /= sy;
        }
        Self { a, plot }
    }
    fn feasible(&self) -> bool {
        let p = self
            .a
            .iter()
            .map(|a| DVec2::new(a.y, a.z))
            .collect::<Vec<_>>();
        if p.is_empty() {
            return false;
        }
        for i in 0..p.len() {
            for j in 0..i {
                let delta = p[i] - p[j];
                if delta.length() < 1e-13 {
                    continue;
                }
                let axis = delta.perp().normalize();
                if p.iter().all(|v| axis.dot(*v) > 1e-12) || p.iter().all(|v| axis.dot(*v) < -1e-12)
                {
                    return false;
                }
            }
        }
        true
    }
    fn maximum(&self, n: DVec2) -> Option<f64> {
        let rhs = self.plot.iter().map(|s| s.dot(n)).collect::<Vec<_>>();
        let mut best = f64::INFINITY;
        for i in 0..self.a.len() {
            for j in 0..i {
                for h in 0..j {
                    let mat = DMat3::from_cols(self.a[i], self.a[j], self.a[h]).transpose();
                    if mat.determinant().abs() < 1e-12 {
                        continue;
                    }
                    let price = mat.inverse() * DVec3::new(rhs[i], rhs[j], rhs[h]);
                    if price.x >= best {
                        continue;
                    }
                    if self
                        .a
                        .iter()
                        .zip(&rhs)
                        .all(|(a, r)| a.dot(price) >= *r - 2e-10 * (1. + r.abs()))
                    {
                        best = price.x;
                    }
                }
            }
        }
        best.is_finite().then_some(best)
    }
}
impl FixtureContact {
    fn model(self) -> (CoplanarSlidingContact, [Vec<DVec2>; 2], DMat3) {
        let f = self.scene.body;
        let s = f.start;
        let q = DMat3::from_quat(s.motion.q);
        let yaw = s.motion.q.to_euler(glam::EulerRot::ZYX).0;
        let sole = Rect {
            center: s.c.truncate(),
            half: DVec2::new(0.18, 0.10),
            yaw,
        };
        let c = CoplanarSlidingContact {
            mass: f.shape.mass,
            height: s.c.z,
            gravity_tangent: f.shape.gravity.truncate(),
            gravity_normal: f.shape.gravity.z,
            center: s.c.truncate(),
            velocity: s.v.truncate(),
            pad_velocities: self.scene.v,
            kinetic_friction: self.mu,
        };
        (
            c,
            self.scene.sites.map(|p| p.patch(sole, false).unwrap()),
            q * f.shape.inertia * q.transpose(),
        )
    }
    fn measure(self) -> Result<Value> {
        let (c, patch, iw) = self.model();
        let pads = [patch[0].as_slice(), patch[1].as_slice()];
        let physical = c.free_yaw(pads, iw.z_axis)?;
        let dual = Dual::build(c, pads, iw);
        if physical.is_some() != dual.feasible() {
            return Err(format!("independent tilt feasibility mismatch {:?}", c.center).into());
        }
        let Some(family) = physical else {
            return Ok(json!({"mode":"no full-face acceleration"}));
        };
        let zero = c.solve(pads)?;
        let slice = family.fraction_at_yaw(0.)?;
        if slice.is_some() != zero.is_some() {
            return Err(format!(
                "zero yaw slice membership {:?} vs {:?}, mu {:?}",
                slice,
                zero.map(|s| s.load),
                self.mu
            )
            .into());
        }
        let mut error = [0f64; 5];
        let mut yaw = [f64::INFINITY, f64::NEG_INFINITY];
        let mut fractions = [1f64, 0.];
        let mut choices = vec![];
        if let Some(set) = zero {
            let r = set.load.fraction_b;
            let direct = slice.unwrap();
            error[0] = (r[0] - direct[0]).abs().max((r[1] - direct[1]).abs());
            for fraction in [r[0], (r[0] + r[1]) * 0.5, r[1]] {
                let s = set.choose(pads, fraction)?.ok_or_else(|| {
                    format!(
                        "missing physical split {r:?} at {fraction} on {:?}",
                        c.center
                    )
                })?;
                let mut torque = DVec3::ZERO;
                for (site, p) in s.site_centers.iter().enumerate() {
                    if let Some(v) = p {
                        torque += (*v - c.center)
                            .extend(-c.height)
                            .cross(s.tangential_force[site].extend(s.normal_force[site]));
                    }
                }
                error[1] = error[1].max(torque.length());
                error[2] = error[2].max(
                    ((s.tangential_force[0] + s.tangential_force[1]).dot(c.velocity)
                        - (s.prescribed_support_power - s.dissipated_power))
                        .abs(),
                );
                if s.dissipated_power < -1e-12 {
                    return Err("negative frictional dissipation".into());
                }
                choices.push(json!({"fraction":fraction,"acceleration":s.acceleration.to_array(),"loss_w":s.dissipated_power,"belt_power_w":s.prescribed_support_power}));
            }
        }
        let hull = family
            .vertices
            .iter()
            .map(|x| {
                yaw[0] = yaw[0].min(x.yaw_acceleration);
                yaw[1] = yaw[1].max(x.yaw_acceleration);
                fractions = [
                    fractions[0].min(x.fraction_b),
                    fractions[1].max(x.fraction_b),
                ];
                DVec2::new(x.fraction_b, x.yaw_acceleration)
            })
            .collect::<Vec<_>>();
        let scale = hull.iter().map(|p| p.y.abs()).fold(1f64, f64::max);
        let scale_v = |p: DVec2| DVec2::new(p.x, p.y / scale);
        let mut axes = (0..16)
            .map(|k| DVec2::from_angle(std::f64::consts::TAU * k as f64 / 16.))
            .collect::<Vec<_>>();
        for (i, p) in hull.iter().enumerate() {
            let d = scale_v(hull[(i + 1) % hull.len()]) - scale_v(*p);
            if d.length() > 1e-12 {
                axes.push(-d.perp().normalize());
            }
        }
        for n in axes {
            let normal = DVec2::new(n.x, n.y / scale);
            let primal = hull
                .iter()
                .map(|v| normal.dot(*v))
                .fold(f64::NEG_INFINITY, f64::max);
            let truth = dual
                .maximum(normal)
                .ok_or("no Cartesian dual certificate")?;
            error[3] = error[3].max((primal - truth).abs());
        }
        for v in &family.vertices {
            let mut torque = DVec3::ZERO;
            for (j, p) in v.site_centers.iter().enumerate() {
                if let Some(p) = p {
                    let w = if j == 0 {
                        1. - v.fraction_b
                    } else {
                        v.fraction_b
                    };
                    let traction =
                        -(c.velocity - c.pad_velocities[j]).normalize() * c.kinetic_friction[j];
                    torque += (*p - c.center)
                        .extend(-c.height)
                        .cross((-c.mass * c.gravity_normal * w) * traction.extend(1.));
                }
            }
            error[4] = error[4].max(torque.distance(iw.z_axis * v.yaw_acceleration));
        }
        if error.iter().any(|v| !v.is_finite() || *v > 1e-8) {
            return Err(format!("sliding field errors {error:?}, slice {slice:?}").into());
        }
        let a = c.gravity_tangent
            - c.gravity_normal * ((self.scene.v[0] - c.velocity).normalize() * self.mu[0]);
        let b = c.gravity_tangent
            - c.gravity_normal * ((self.scene.v[1] - c.velocity).normalize() * self.mu[1]);
        Ok(
            json!({"mode":"feasible","fraction_range":fractions,"yaw_range":yaw,"acceleration_diameter_m_s2":a.distance(b)*(fractions[1]-fractions[0]),"hull":hull.into_iter().map(|v|v.to_array()).collect::<Vec<_>>(),"zero_yaw_fraction":slice,"zero_yaw_examples":choices,"errors_zero_slice_torque_power_dual_hull_wrench":error}),
        )
    }
    fn source(self, i: Input) -> String {
        let src = self.scene.source(i, false);
        let a = src.find("    <model name=\"lift\">").unwrap();
        let b = src.find("    <model name=\"toe\">").unwrap();
        let field = |xml: &str, mu: f64| {
            xml.replace(
                "<mu>0</mu><mu2>0</mu2>",
                &format!("<mu>{mu}</mu><mu2>{mu}</mu2>"),
            )
        };
        field(&src[..a], 1.) + &field(&src[a..b], self.mu[0]) + &field(&src[b..], self.mu[1])
    }
    fn replay(self, i: Input, out: &Path, field: &Value) -> Result<()> {
        let src = self.source(i);
        fs::write(out.join("belts.sdf"), &src)?;
        let mut curves = vec![];
        for (tag, calls) in [("ordinary", 1), ("eight", 8), ("thirtytwo", 32)] {
            let mut sim = load(&src, true)?;
            let id = ids(&sim)?;
            let toe = bind(&sim, "default::toe::pad")?.1;
            let mut owner = sim.world_mut().resource_mut::<PhysicsBackend>();
            let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
            let mut rows = vec![];
            let mut tape = vec![];
            for tick in 0..=140 {
                let mut j = [0.; 2];
                if tick > 0 {
                    let r = n.step_contact_interval(Duration::from_micros(500), calls, 4)?;
                    for (k, pad) in [id.pad, toe].into_iter().enumerate() {
                        j[k] = r
                            .pairs
                            .iter()
                            .filter(|p| [p.a, p.b].contains(&pad) && [p.a, p.b].contains(&id.foot))
                            .map(|p| p.normal_impulse)
                            .sum();
                    }
                }
                rows.push(self.scene.capture(
                    n,
                    &id,
                    toe,
                    tick,
                    if tick == 0 { 0 } else { calls },
                    j,
                ));
                let (f, u, _) = departure::ingest(n, &id);
                let u = u.unwrap();
                let v = (DVec3::from(f.velocity) + DVec3::from(u.velocity)) * 0.5;
                tape.push(json!({"tick":tick,"impulses_ns":j,"center_velocity":v.to_array(),"spin":f.angular_velocity}));
            }
            self.scene.trace(out, tag, rows)?;
            curves.push(json!({"mode":tag,"ticks":tape}));
        }
        fs::write(
            out.join("public.json"),
            serde_json::to_vec(
                &json!({"physical_initial":field,"native":curves,"mu":self.mu,"pad_relative_velocity":self.scene.v.map(|v|v.to_array())}),
            )?,
        )?;
        let lanes = [
            ("ordinary", "One native solve"),
            ("eight", "Eight native solves"),
            ("thirtytwo", "32 native solves"),
        ]
        .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
        fs::write(
            out.join("comparison.json"),
            serde_json::to_vec_pretty(
                &json!({"title":"Two moving belts admit a range of sliding and yaw responses","pair_totals_only":true,"capture_ticks":{"first_response":1,"later_yaw":140},"lanes":lanes}),
            )?,
        )?;
        Ok(())
    }
}
pub(in super::super::super::super::super::super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let (i, p, _) = fixture(Case {
        active: 0,
        pitch: 0.18,
        opening: 0.3,
        yaw: 0.,
        pitch_rate: 0.6,
    });
    let mut rows = vec![];
    let mut demo = None;
    for x in [-0.12, -0.043, 0.014, 0.084, 0.16] {
        for y in [-0.055, 0.03] {
            for yaw in [-0.31, 0.35] {
                for skew in [-0.2, 0.19] {
                    for belts in 0..4 {
                        for principal in [true, false] {
                            let mut shape = p.shape;
                            shape.inertia = DMat3::from_cols(
                                DVec3::new(0.042, 0.002, if principal { 0. } else { -0.008 }),
                                DVec3::new(0.002, 0.053, if principal { 0. } else { 0.006 }),
                                DVec3::new(
                                    if principal { 0. } else { -0.008 },
                                    if principal { 0. } else { 0.006 },
                                    0.059,
                                ),
                            );
                            shape.gravity = DVec3::new(0.13, -0.08, -9.81);
                            let (vel, mu) = match belts {
                                0 => ([DVec2::ZERO; 2], [0.19; 2]),
                                1 => ([DVec2::ZERO; 2], [0.08, 0.63]),
                                2 => (
                                    [DVec2::new(-0.4, 0.19), DVec2::new(1.7, -0.31)],
                                    [0.22, 0.44],
                                ),
                                _ => (
                                    [DVec2::new(0.4, -0.65), DVec2::new(0.2, 0.96)],
                                    [0.46, 0.19],
                                ),
                            };
                            let trial = FixtureContact {
                                scene: Trial {
                                    body: Fixture {
                                        shape,
                                        start: State {
                                            c: DVec3::new(x, y, 0.13),
                                            v: DVec3::new(0.76, 0.14, 0.),
                                            motion: Motion {
                                                q: DQuat::from_rotation_z(yaw),
                                                w: DVec3::ZERO,
                                            },
                                        },
                                    },
                                    inclined: false,
                                    sites: [
                                        Rect {
                                            center: DVec2::new(-0.13, -0.009),
                                            half: DVec2::new(0.105, 0.142),
                                            yaw: skew,
                                        },
                                        Rect {
                                            center: DVec2::new(0.14, 0.025),
                                            half: DVec2::new(0.115, 0.135),
                                            yaw: -0.7 * skew,
                                        },
                                    ],
                                    v: vel,
                                },
                                mu,
                            };
                            let data = trial.measure()?;
                            if demo.is_none()
                                && belts == 2
                                && !principal
                                && data["mode"] == "feasible"
                                && data["acceleration_diameter_m_s2"].as_f64().unwrap() > 2.
                                && !data["zero_yaw_fraction"].is_null()
                            {
                                demo = Some((trial, data.clone()));
                            }
                            rows.push(json!({"x":x,"y":y,"yaw":yaw,"skew":skew,"belts":belts,"principal":principal,"result":data}));
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("sliding.json"), serde_json::to_vec(&rows)?)?;
    let (trial, data) = demo.ok_or("no physically diverse sliding demo")?;
    trial.replay(i, out, &data)
}
