// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! An independent virtual velocity field tests every continuous Coulomb disk.
//! Symmetric cases also admit a one-dimensional exact force-sharing interval.
#[path="capture/branches.rs"]
pub(crate) mod branches;
use super::*;
use gz_physics::{BeltCapture, BeltDualBound, BeltEquilibriumWitness};
fn certificate(
    c: CoplanarSlidingContact,
    p: [&[DVec2]; 2],
    mu: f64,
    tr: DVec2,
    y: BeltDualBound,
) -> [f64; 2] {
    let mut residual = 0f64;
    for (site, patch) in p.iter().enumerate() {
        for point in *patch {
            let arm = (*point - c.center).extend(-c.height);
            let velocity = y.linear + y.angular.cross(arm);
            let low = velocity.z
                + if site == 0 {
                    tr.dot(velocity.truncate())
                } else {
                    -mu * velocity.truncate().length()
                };
            residual = residual.max(if site == 0 { -low } else { y.objective - low });
        }
    }
    let target = y
        .linear
        .dot((c.gravity_tangent / c.gravity_normal).extend(1.));
    [residual, (target - y.target_work).abs()]
}
fn audit_witness(
    c: CoplanarSlidingContact,
    patches: [&[DVec2]; 2],
    mu: f64,
    k: DVec2,
    w: &BeltEquilibriumWitness,
) -> [f64; 5] {
    let nf = -c.mass * c.gravity_normal;
    let mut f = DVec3::ZERO;
    let mut m = DVec3::ZERO;
    let mut pressure = 0.;
    let mut traction = 0f64;
    for r in &w.reactions {
        let impulse = r.tangent_force.extend(r.normal_force);
        f += impulse;
        m += (r.position - c.center).extend(-c.height).cross(impulse);
        assert!(r.normal_force >= 0. && patches[r.pad].contains(&r.position));
        if r.pad == 1 {
            pressure += r.normal_force;
            traction = traction.max(r.tangent_force.length() - mu * r.normal_force);
        } else {
            traction = traction.max(r.tangent_force.distance(k * r.normal_force));
        }
    }
    let gravity = c.gravity_tangent.extend(c.gravity_normal) * c.mass;
    [
        f.distance(-gravity),
        m.length(),
        traction,
        (w.fraction_b - pressure / nf).abs(),
        (w.prescribed_support_power - w.dissipated_power - f.truncate().dot(c.velocity)).abs(),
    ]
}
fn restrict(r: &mut [f64; 2], constant: f64, gradient: f64) {
    if gradient > 0. {
        r[1] = r[1].min(-constant / gradient);
    } else if gradient < 0. {
        r[0] = r[0].max(-constant / gradient);
    } else if constant > 1e-12 {
        *r = [1., 0.];
    }
}
fn symmetric_range(c: CoplanarSlidingContact, patches: [&[DVec2]; 2], mu: f64) -> Option<[f64; 2]> {
    let k = c.gravity_tangent / c.gravity_normal;
    let dir = (c.velocity - c.pad_velocities[0]).normalize();
    let m = c.kinetic_friction[0];
    let mut r = linear_program(c.center - c.height * k, patches)?;
    let f = k.dot(dir) + m;
    restrict(&mut r, f, -m - mu);
    restrict(&mut r, -f, m - mu);
    (r[0] <= r[1] + 1e-12).then_some(r)
}
fn clip(poly: &[DVec2], n: DVec2, h: f64) -> Vec<DVec2> {
    let mut out = vec![];
    let Some(&last) = poly.last() else {
        return out;
    };
    let mut prev = last;
    let mut a = h - prev.dot(n);
    for &v in poly {
        let b = h - v.dot(n);
        if (a >= 0.) != (b >= 0.) {
            out.push(prev + (v - prev) * (a / (a - b)));
        }
        if b >= 0. {
            out.push(v);
        }
        a = b;
        prev = v;
    }
    out
}
impl FixtureContact {
    fn capture_field(self, symmetric: bool) -> Result<Value> {
        let (c, p, _) = self.model();
        let patches = [p[0].as_slice(), p[1].as_slice()];
        let mu = self.mu[1];
        let k = -(c.velocity - c.pad_velocities[0]).normalize() * self.mu[0];
        let direct = symmetric.then(|| symmetric_range(c, patches, mu));
        let result = c.stationary_second_belt(patches, mu)?;
        let mut e = [0f64; 8];
        match result {
            BeltCapture::Unresolved => Ok(json!({"mode":"unresolved","symmetric":direct})),
            BeltCapture::Infeasible {
                separation,
                certificate: y,
            } => {
                let q = certificate(c, patches, mu, k, y);
                if q.iter().any(|v| *v > 1e-7) || direct.flatten().is_some() || separation <= 0. {
                    return Err(format!(
                        "invalid negative certificate {q:?} {separation} {direct:?}"
                    )
                    .into());
                }
                Ok(
                    json!({"mode":"infeasible","separation":separation,"dual_errors":q,"symmetric":direct}),
                )
            }
            BeltCapture::Feasible(set) => {
                for w in &set.endpoints {
                    for (x, v) in e.iter_mut().zip(audit_witness(c, patches, mu, k, w)) {
                        *x = x.max(v);
                    }
                }
                for y in set.certificates {
                    for (x, v) in e[5..7].iter_mut().zip(certificate(c, patches, mu, k, y)) {
                        *x = x.max(v);
                    }
                }
                if let Some(truth) = direct {
                    let truth = truth.ok_or("unexpected feasible equilibrium on symmetric pads")?;
                    for (i, x) in truth.into_iter().enumerate() {
                        e[7] = e[7]
                            .max((x - set.inner[i]).abs())
                            .max((x - set.outer[i]).abs());
                    }
                }
                if e.iter().any(|x| !x.is_finite() || *x > 1e-7) {
                    return Err(format!("invalid primal/dual force certificate: {e:?}").into());
                }
                let mean = c.center - c.height * c.gravity_tangent / c.gravity_normal;
                let direction = k - c.gravity_tangent / c.gravity_normal;
                // Uniform tangential traction on each pad would require both
                // pressure centers on this line through the aggregate pressure.
                let no_uniform = direction.length() > 1e-12
                    && p[0]
                        .iter()
                        .map(|v| direction.perp_dot(*v - mean))
                        .reduce(f64::min)
                        .is_some_and(|x| x > 1e-12)
                    || direction.length() > 1e-12
                        && p[0]
                            .iter()
                            .map(|v| direction.perp_dot(*v - mean))
                            .reduce(f64::max)
                            .is_some_and(|x| x < -1e-12);
                let entries=set.endpoints.iter().map(|v|json!({"toe":v.fraction_b,"watts":v.dissipated_power,"belt_work_w":v.prescribed_support_power,"vertices":v.reactions.iter().map(|r|json!({"pad":r.pad,"xy":r.position.to_array(),"normal_n":r.normal_force,"tangent_n":r.tangent_force.to_array()})).collect::<Vec<_>>() })).collect::<Vec<_>>();
                Ok(
                    json!({"mode":"feasible","inner":set.inner,"outer":set.outer,"positive_heel_requires_distributed_static_traction":no_uniform&&set.outer[1]<1.-1e-9,"endpoints":entries,"errors_force_torque_cone_share_power_dual_bound_closedform":e,"symmetric":direct}),
                )
            }
        }
    }
    fn held_capture(self, i: Input, out: &Path) -> Result<()> {
        let (c, p, _) = self.model();
        let src = self.source(i).replace(
            "A yawing foot spans two finite, independently translating frictionless supports.",
            "A sole moves with the toe belt while the heel belt slides underneath.",
        );
        fs::write(out.join("capture.sdf"), &src)?;
        let horizon = 0.045;
        let ticks = (horizon / DT).round() as u64;
        let mut inner = p[0].clone();
        let rect = self.scene.sites[0];
        let x = DVec2::from_angle(rect.yaw);
        let d = (self.scene.v[0] - self.scene.v[1]) * horizon;
        for (axis, half) in [(x, rect.half.x), (x.perp(), rect.half.y)] {
            for n in [axis, -axis] {
                inner = clip(&inner, n, half + n.dot(rect.center + d));
            }
        }
        let BeltCapture::Feasible(set) = c.stationary_second_belt([&inner, &p[1]], self.mu[1])?
        else {
            return Err("no continuous pressure / shear witness".into());
        };
        let f = (set.inner[0] + set.inner[1]) * 0.5;
        let normal = -c.gravity_normal * c.mass * DT;
        let mut measurements = vec![];
        let seed = load(&src, true)?;
        let id = ids(&seed)?;
        let toe = bind(&seed, "default::toe::pad")?.1;
        let initial = self.scene.capture(engine(&seed), &id, toe, 0, 0, [0.; 2]);
        let mut rows = vec![];
        for tick in 0..=ticks {
            let mut r = initial.clone();
            r["tick"] = json!(tick);
            r["time_us"] = json!(tick * 500);
            let j = if tick == 0 {
                [0.; 2]
            } else {
                [(1. - f) * normal, f * normal]
            };
            r["support_normal_ns"] = json!(j);
            r["normal_impulse_ns"] = json!(j[0] + j[1]);
            r["solver_iterations"] = json!(0);
            for name in ["foot", "upper", "pad", "opponent"] {
                let xyz = |tag: &str| -> Result<DVec3> {
                    let p = &r[name][tag];
                    Ok(DVec3::new(
                        p[0].as_f64().ok_or("x")?,
                        p[1].as_f64().ok_or("y")?,
                        p[2].as_f64().ok_or("z")?,
                    ))
                };
                r[name]["position"] =
                    json!((xyz("position")? + tick as f64 * DT * xyz("velocity")?).to_array());
            }
            rows.push(r);
        }
        self.scene.trace(out, "stationary", rows)?;
        for (name, calls) in [("ordinary", 1), ("eight", 8)] {
            let mut sim = load(&src, true)?;
            let id = ids(&sim)?;
            let toe = bind(&sim, "default::toe::pad")?.1;
            let mut device = sim.world_mut().resource_mut::<PhysicsBackend>();
            let n = device.downcast_mut::<MultiFidelityBackend>().unwrap();
            let mut rows = vec![];
            let mut peak = [0f64; 2];
            let mut first = [0.; 2];
            for tick in 0..=ticks {
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
                let state = n.state(id.foot).ok_or("missing measured foot")?;
                let belt = n.state(toe).ok_or("missing measured toe")?;
                peak[0] = peak[0].max(
                    DVec3::from(state.velocity).distance(DVec3::from(belt.velocity)),
                );
                peak[1] = peak[1].max(DVec3::from(state.angular_velocity).length());
                if tick == 1 {
                    first = j;
                }
                rows.push(self.scene.capture(
                    n,
                    &id,
                    toe,
                    tick,
                    if tick > 0 { calls } else { 0 },
                    j,
                ));
            }
            self.scene.trace(out, name, rows)?;
            measurements.push(
                json!({"name":name,"first_j_ns":first,"max_foot_speed_relative_toe_and_spin":peak}),
            );
        }
        let lanes=[("ordinary","One native solve"),("eight","Eight native solves"),("stationary","One feasible sticking path")].map(|(l,n)|json!({"label":l,"name":n,"file":format!("traces/{l}.json"),"analytical":l=="stationary"}));
        fs::write(
            out.join("comparison.json"),
            serde_json::to_vec_pretty(
                &json!({"title":"A moving toe belt can hold a sole while the heel slides","pair_totals_only":true,"capture_ticks":{"first_interval":1,"sustained":ticks},"lanes":lanes}),
            )?,
        )?;
        fs::write(
            out.join("public.json"),
            serde_json::to_vec_pretty(
                &json!({"incoming":self.capture_field(false)?,"admissible_held_toe_fraction":set.inner,"held_interval_s":horizon,"held_toe_fraction":f,"native":measurements}),
            )?,
        )?;
        Ok(())
    }
}
pub(in super::super::super::super::super::super::super::super::super) fn run(
    out: &Path,
) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let (i, p, _) = fixture(Case {
        active: 0,
        pitch: 0.18,
        opening: 0.3,
        yaw: 0.,
        pitch_rate: 0.6,
    });
    let mut rows = vec![];
    let mut example = None;
    for sym in [true, false] {
        for angle in [0., 0.37, 1.29] {
            let axis = DVec2::from_angle(angle);
            for x in [-0.045, 0.019, 0.071] {
                for mu_b in [0.035, 0.18, 0.36, 0.65, 0.92] {
                    for side in [-1., 1.] {
                        for g in [-0.63, 0., 0.79] {
                            let cb = axis.rotate(DVec2::new(x, 0.));
                            let va = axis.rotate(DVec2::new(-0.27, 0.04));
                            let vb = axis.rotate(DVec2::new(0.43, 0.04));
                            let mut shape = p.shape;
                            shape.gravity = (axis * g).extend(-9.81);
                            let sites = if sym {
                                [
                                    Rect {
                                        center: axis.rotate(DVec2::new(-0.13, 0.)),
                                        half: DVec2::new(0.085, 0.17),
                                        yaw: angle,
                                    },
                                    Rect {
                                        center: axis.rotate(DVec2::new(0.12, 0.)),
                                        half: DVec2::new(0.095, 0.16),
                                        yaw: angle,
                                    },
                                ]
                            } else {
                                [
                                    Rect {
                                        center: axis.rotate(DVec2::new(-0.095, side * 0.093)),
                                        half: DVec2::new(0.081, 0.039),
                                        yaw: angle + side * 0.12,
                                    },
                                    Rect {
                                        center: axis.rotate(DVec2::new(0.10, -side * 0.024)),
                                        half: DVec2::new(0.085, 0.109),
                                        yaw: angle - side * 0.21,
                                    },
                                ]
                            };
                            let f = FixtureContact {
                                mu: [0.34, mu_b],
                                scene: Trial {
                                    body: Fixture {
                                        shape,
                                        start: State {
                                            c: cb.extend(0.13),
                                            v: vb.extend(0.),
                                            motion: Motion {
                                                q: DQuat::from_rotation_z(if sym {
                                                    angle
                                                } else {
                                                    angle + side * 0.13
                                                }),
                                                w: DVec3::ZERO,
                                            },
                                        },
                                    },
                                    sites,
                                    v: [va, vb],
                                    inclined: side > 0.,
                                },
                            };
                            let r = f.capture_field(sym)?;
                            if example.is_none()
                                && !sym
                                && side > 0.
                                && angle == 0.37
                                && x == -0.045
                                && mu_b == 0.92
                                && g == 0.
                                && r["positive_heel_requires_distributed_static_traction"] == true
                            {
                                example = Some(f);
                            }
                            rows.push(json!({"symmetric":sym,"rotation":angle,"center_coordinate":x,"static_mu":mu_b,"side":side,"gravity_tangent":g,"result":r}));
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("capture.json"), serde_json::to_vec(&rows)?)?;
    example
        .ok_or("missing distributed capture nominee")?
        .held_capture(i, out)
}
