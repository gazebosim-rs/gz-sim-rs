// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A shared incoming sliding solution reaches one belt's velocity. Two
//! different continuous motions then satisfy the same passive Coulomb law.
use super::*;
use gz_physics::SlidingLoadChoice;
const PRE: f64 = 0.00821381976;
const POST: f64 = 0.045;
fn swept(f: FixtureContact, velocity: DVec2, acc: DVec2, h: f64) -> [Vec<DVec2>; 2] {
    let (_, p, _) = f.model();
    std::array::from_fn(|i| {
        let rect = f.scene.sites[i];
        let x = DVec2::from_angle(rect.yaw);
        let mut result = p[i].clone();
        for (axis, half) in [(x, rect.half.x), (x.perp(), rect.half.y)] {
            for n in [axis, -axis] {
                let v = n.dot(f.scene.v[i] - velocity);
                let a = -n.dot(acc);
                let mut min = 0f64.min(v * h + 0.5 * a * h * h);
                let time = -v / a;
                if time > 0. && time < h {
                    min = min.min(0.5 * v * time);
                }
                result = clip(&result, n, half + n.dot(rect.center) + min - 1e-13);
            }
        }
        result
    })
}
fn pressure_motion(c: CoplanarSlidingContact, s: &SlidingLoadChoice) -> [f64; 4] {
    let mut m = DVec3::ZERO;
    for (k, point) in s.site_centers.iter().enumerate() {
        if let Some(p) = point {
            m += (*p - c.center)
                .extend(-c.height)
                .cross(s.tangential_force[k].extend(s.normal_force[k]));
        }
    }
    let v = s.tangential_force[0] + s.tangential_force[1];
    let n = s.normal_force.iter().sum::<f64>();
    [
        m.length(),
        (c.mass * (s.acceleration - c.gravity_tangent)).distance(v),
        (n + c.mass * c.gravity_normal).abs(),
        (s.prescribed_support_power - s.dissipated_power - v.dot(c.velocity)).abs(),
    ]
}
#[derive(Clone)]
struct Branch {
    fraction: f64,
    acc: DVec2,
    net_shear: [DVec2; 2],
}
impl FixtureContact {
    fn postbranches(self) -> Result<(Value, Option<(Branch, Branch)>)> {
        let (c, _, _) = self.model();
        let dir = (c.velocity - c.pad_velocities[0]).normalize();
        let normal = -c.gravity_normal * c.mass;
        let mu = self.mu[1];
        let held = swept(self, c.velocity, DVec2::ZERO, POST);
        let hp = [held[0].as_slice(), held[1].as_slice()];
        let cap = c.stationary_second_belt(hp, mu)?;
        let sticking = if let BeltCapture::Feasible(range) = cap {
            let weights = range.inner;
            let fraction = 0.5 * (weights[0] + weights[1]);
            let first = -dir * (1. - fraction) * normal * self.mu[0];
            Some(Branch {
                fraction,
                acc: DVec2::ZERO,
                net_shear: [first, -c.mass * c.gravity_tangent - first],
            })
        } else {
            None
        };
        let mut solutions = vec![];
        let mut errors = [0f64; 6];
        for i in 1..20 {
            let f = i as f64 / 20.;
            let drive = c.gravity_tangent + dir * (c.gravity_normal * (1. - f) * self.mu[0]);
            let resist = -c.gravity_normal * mu * f;
            if drive.length() <= resist + 1e-12 {
                continue;
            }
            let acc = drive.normalize() * (drive.length() - resist);
            if acc.perp_dot(dir).abs() > 1e-12
                || dir.dot(c.velocity - c.pad_velocities[0] + POST * acc) <= 0.
            {
                continue;
            }
            let material = swept(self, c.velocity, acc, POST);
            let Some(choice) = c.begin_second_belt_slip([&material[0], &material[1]], f)? else {
                continue;
            };
            let e = pressure_motion(c, &choice);
            for (k, v) in e.into_iter().enumerate() {
                errors[k] = errors[k].max(v);
            }
            errors[4] = errors[4].max(acc.distance(choice.acceleration));
            let t = POST;
            let disp = c.velocity * t + 0.5 * acc * t * t;
            let end = c.velocity + acc * t;
            let k = 0.5 * c.mass * (end.length_squared() - c.velocity.length_squared());
            let power = choice
                .tangential_force
                .iter()
                .enumerate()
                .map(|(k, f)| f.dot(c.pad_velocities[k]) * t)
                .sum::<f64>();
            let heat = choice
                .tangential_force
                .iter()
                .enumerate()
                .map(|(k, f)| -f.dot(disp - c.pad_velocities[k] * t))
                .sum::<f64>();
            errors[5] =
                errors[5].max((k - c.mass * c.gravity_tangent.dot(disp) - power + heat).abs());
            if heat < -1e-14 {
                return Err("negative heat during emerging slip".into());
            }
            solutions.push(Branch {
                fraction: f,
                acc,
                net_shear: choice.tangential_force,
            });
        }
        if errors.iter().any(|v| !v.is_finite() || *v > 3e-9) {
            return Err(format!("invalid finite second-belt branches {errors:?}").into());
        }
        let fastest = solutions
            .iter()
            .max_by(|a, b| a.acc.length().total_cmp(&b.acc.length()))
            .cloned();
        let news = json!({"admits_held_stick":sticking.is_some(),"sampled_held_slip_solutions":solutions.len(),"sampled_fractions":solutions.iter().map(|p|p.fraction).collect::<Vec<_>>(),"maximum_certified_slip_acceleration_m_s2":fastest.as_ref().map(|b|b.acc.length()),"errors_torque_force_normal_initial_work_acceleration_interval_energy":errors});
        Ok((news, sticking.zip(fastest)))
    }
    fn history(self) -> Result<(Self, Branch)> {
        let (c, _, _) = self.model();
        let dir = (c.velocity - c.pad_velocities[0]).normalize();
        if self.mu[0] != self.mu[1] {
            return Err("common incoming traction requires matching coefficients".into());
        }
        let acc = c.gravity_tangent + c.gravity_normal * self.mu[0] * dir;
        let v0 = c.velocity - acc * PRE;
        let c0 = c.center - c.velocity * PRE + 0.5 * acc * PRE * PRE;
        let mut initial = self;
        initial.scene.body.start.c = c0.extend(c.height);
        initial.scene.body.start.v = v0.extend(0.);
        for (i, r) in initial.scene.sites.iter_mut().enumerate() {
            r.center -= c.pad_velocities[i] * PRE;
        }
        let mat = swept(initial, v0, acc, PRE);
        let (ic, _, _) = initial.model();
        let traction = -self.mu[0] * dir;
        let target = ic.center - ic.height * traction;
        let cap = linear_program(target, [&mat[0], &mat[1]])
            .ok_or("swept incoming center outside available normal patch")?;
        let fraction = 0.5 * (cap[0] + cap[1]);
        let normal = -ic.mass * ic.gravity_normal;
        let forces = [
            (1. - fraction) * normal * traction,
            fraction * normal * traction,
        ];
        if (traction * (-ic.gravity_normal) + ic.gravity_tangent).distance(acc) > 1e-12
            || v0.distance(self.scene.v[1]) <= 0.
        {
            return Err("incorrect common acceleration".into());
        }
        Ok((
            initial,
            Branch {
                fraction,
                acc,
                net_shear: forces,
            },
        ))
    }
    fn dual_replay(self, i: Input, out: &Path, stationary: Branch, sliding: Branch) -> Result<()> {
        let (initial, pre) = self.history()?;
        let src = initial.source(i).replace(
            "A yawing foot spans two finite, independently translating frictionless supports.",
            "A sliding sole reaches the toe belt speed after a shared kinetic prehistory.",
        );
        fs::write(out.join("branches.sdf"), &src)?;
        let c0 = initial.scene.body.start.c.truncate();
        let v0 = initial.scene.body.start.v.truncate();
        let (c, _, _) = self.model();
        let ticks = ((PRE + POST) / DT).floor() as u64;
        let fr = frame(self.scene.inclined);
        let nf = -c.gravity_normal * c.mass;
        let mut sim = load(&src, true)?;
        let id = ids(&sim)?;
        let toe = bind(&sim, "default::toe::pad")?.1;
        let base = self.scene.capture(engine(&sim), &id, toe, 0, 0, [0.; 2]);
        let mut public = vec![];
        for (label, branch) in [("sticking", &stationary), ("slipping", &sliding)] {
            let mut frames = vec![];
            let mut totals = [0.; 3];
            let mut check = 0f64;
            for tick in 0..=ticks {
                let t = tick as f64 * DT;
                let h0 = (t - DT).max(0.);
                let a = t.min(PRE);
                let b = (t - PRE).max(0.);
                let cp =
                    c0 + v0 * a + 0.5 * pre.acc * a * a + c.velocity * b + 0.5 * branch.acc * b * b;
                let vel = v0 + pre.acc * a + branch.acc * b;
                let mut j = [0.; 2];
                let mut tangent = [DVec2::ZERO; 2];
                for (phase, start, end, pv) in [
                    (&pre, h0.min(PRE), a, v0),
                    (branch, (h0 - PRE).max(0.), b, c.velocity),
                ] {
                    let h = end - start;
                    let d = (pv + 0.5 * (start + end) * phase.acc) * h;
                    for k in 0..2 {
                        j[k] += nf
                            * h
                            * if k == 0 {
                                1. - phase.fraction
                            } else {
                                phase.fraction
                            };
                        tangent[k] += phase.net_shear[k] * h;
                        totals[0] -= phase.net_shear[k].dot(d - c.pad_velocities[k] * h);
                        totals[1] += phase.net_shear[k].dot(c.pad_velocities[k] * h);
                    }
                    totals[2] += c.mass * c.gravity_tangent.dot(d);
                }
                check = check.max(
                    (0.5 * c.mass * (vel.length_squared() - v0.length_squared())
                        - totals[2]
                        - totals[1]
                        + totals[0])
                        .abs(),
                );
                let mut f = base.clone();
                f["tick"] = json!(tick);
                f["time_us"] = json!(tick * 500);
                f["support_normal_ns"] = json!(j);
                f["normal_impulse_ns"] = json!(j[0] + j[1]);
                for k in ["foot", "upper", "pad", "opponent"] {
                    let arr = |n: &str| -> DVec3 {
                        let p = &base[k][n];
                        DVec3::new(
                            p[0].as_f64().unwrap(),
                            p[1].as_f64().unwrap(),
                            p[2].as_f64().unwrap(),
                        )
                    };
                    let (x, v) = if k == "foot" || k == "upper" {
                        (
                            arr("position") + fr.q * (COMMON * t + (cp - c0).extend(0.)),
                            fr.q * (COMMON + vel.extend(0.)),
                        )
                    } else {
                        (arr("position") + arr("velocity") * t, arr("velocity"))
                    };
                    f[k]["position"] = json!(x.to_array());
                    f[k]["velocity"] = json!(v.to_array());
                }
                frames.push(f);
            }
            if check > 1e-12 {
                return Err("bad continuous branch work".into());
            }
            initial.scene.trace(out, label, frames)?;
            public.push(json!({"mode":label,"toe_fraction_after":branch.fraction,"post_acceleration_m_s2":branch.acc.to_array(),"post_shear_forces_n":branch.net_shear.map(|v|v.to_array()),"dissipation_belt_gravity_j":totals,"energy_error_j":check}));
        }
        let mut owner = sim.world_mut().resource_mut::<PhysicsBackend>();
        let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
        let mut rows = vec![];
        for tick in 0..=ticks {
            let mut j = [0.; 2];
            if tick > 0 {
                let r = n.step_contact_interval(Duration::from_micros(500), 1, 4)?;
                for (k, p) in [id.pad, toe].into_iter().enumerate() {
                    j[k] = r
                        .pairs
                        .iter()
                        .filter(|z| [z.a, z.b].contains(&id.foot) && [z.a, z.b].contains(&p))
                        .map(|z| z.normal_impulse)
                        .sum();
                }
            }
            rows.push(
                initial
                    .scene
                    .capture(n, &id, toe, tick, u32::from(tick > 0), j),
            );
        }
        initial.scene.trace(out, "ordinary", rows)?;
        let post = ticks as f64 * DT - PRE;
        let d = 0.5 * sliding.acc.length() * post * post;
        fs::write(
            out.join("public.json"),
            serde_json::to_vec_pretty(
                &json!({"common_incoming_s":PRE,"pre_acceleration_m_s2":pre.acc.to_array(),"pre_toe_fraction":pre.fraction,"certified_post_duration_s":POST,"last_sample_s":ticks as f64*DT,"last_branch_distance_m":d,"last_branch_speed_difference_m_s":sliding.acc.length()*post,"branches":public}),
            )?,
        )?;
        let lanes=[("ordinary","One native solve"),("slipping","Feasible emerging slip"),("sticking","Feasible toe sticking")].map(|(l,n)|json!({"label":l,"name":n,"file":format!("traces/{l}.json"),"analytical":l!="ordinary"}));
        fs::write(
            out.join("comparison.json"),
            serde_json::to_vec_pretty(
                &json!({"title":"One incoming landing admits two continuous belt motions","pair_totals_only":true,"capture_ticks":{"shared_prehistory":(PRE/DT).floor() as u64,"catch":(PRE/DT).ceil() as u64,"diverged":ticks},"lanes":lanes}),
            )?,
        )?;
        Ok(())
    }
}
pub(in super::super::super::super::super::super::super::super::super::super) fn run(
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
    let mut demo = None;
    for mu in [0.16, 0.29, 0.41] {
        for heading in [-0.41, 0.32] {
            let axis = DVec2::from_angle(heading);
            for z in [0.10, 0.13, 0.17] {
                for x in [-0.065, -0.018, 0.026, 0.08] {
                    for skew in [false, true] {
                        for g in [-0.52, 0., 0.37] {
                            let mut shape = p.shape;
                            shape.gravity = (axis * g).extend(-9.81);
                            let vb = axis.rotate(DVec2::new(0.40, 0.04));
                            let va = axis.rotate(DVec2::new(-0.25, 0.04));
                            let f = FixtureContact {
                                mu: [mu, mu],
                                scene: Trial {
                                    body: Fixture {
                                        shape,
                                        start: State {
                                            c: axis.rotate(DVec2::new(x, 0.)).extend(z),
                                            v: vb.extend(0.),
                                            motion: Motion {
                                                q: DQuat::from_rotation_z(
                                                    heading + if skew { 0.13 } else { 0. },
                                                ),
                                                w: DVec3::ZERO,
                                            },
                                        },
                                    },
                                    sites: [
                                        Rect {
                                            center: axis.rotate(DVec2::new(
                                                -0.11,
                                                if skew { 0.018 } else { 0. },
                                            )),
                                            half: DVec2::new(0.098, 0.15),
                                            yaw: heading + if skew { 0.11 } else { 0. },
                                        },
                                        Rect {
                                            center: axis.rotate(DVec2::new(
                                                0.114,
                                                if skew { -0.012 } else { 0. },
                                            )),
                                            half: DVec2::new(0.105, 0.15),
                                            yaw: heading - if skew { 0.17 } else { 0. },
                                        },
                                    ],
                                    v: [va, vb],
                                    inclined: skew,
                                },
                            };
                            let (res, both) = f.postbranches()?;
                            if demo.is_none()
                                && z == 0.13
                                && mu == 0.29
                                && heading > 0.
                                && skew
                                && g == 0.
                                && x == -0.018
                                && let Some(b) = both.clone()
                            {
                                demo = Some((f, b));
                            }
                            rows.push(json!({"mu":mu,"heading":heading,"height":z,"x":x,"skew":skew,"tangential_gravity_m_s2":g,"result":res}));
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("branches.json"), serde_json::to_vec(&rows)?)?;
    let (f, (s, l)) = demo.ok_or("missing finite matched branches")?;
    f.dual_replay(i, out, s, l)
}
