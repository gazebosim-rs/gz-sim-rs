// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Authored equal-kinetic-traction contacts, checked by Cartesian momentum,
/*! work, pressure LP bounds and the complete initial acceleration family. */
#[path = "uniform/arrest.rs"]
pub(crate) mod arrest;
#[path = "uniform/parked.rs"]
pub(crate) mod parked;
use super::spatial::Trial;
use super::*;
#[derive(Clone, Copy)]
struct Campaign {
    case: Trial,
    mu: f64,
    axis: DVec2,
    speed: f64,
}
impl Campaign {
    fn source(self, i: Input, registered: bool) -> String {
        let src = self.case.source(i, false).replace("A yawing foot spans two finite, independently translating frictionless supports.", "A locked foot slides over two finite translating supports with equal kinetic traction.").replace(
            "<mu>0</mu><mu2>0</mu2>",
            &format!("<mu>{}</mu><mu2>{}</mu2>", self.mu, self.mu),
        );
        if registered {
            enrolled(&src).replace(
                "</share_coplanar_face>",
                "</share_coplanar_face><share_uniform_sliding>true</share_uniform_sliding>",
            )
        } else {
            src
        }
    }
    fn force(self, t: f64) -> (DVec2, DVec2, DVec2) {
        let f = self.case.body;
        let acceleration = f.shape.gravity.truncate() + self.axis * (self.mu * f.shape.gravity.z);
        let center = f.start.c.truncate() + f.start.v.truncate() * t + 0.5 * acceleration * t * t;
        let velocity = f.start.v.truncate() + acceleration * t;
        (center, velocity, center + self.axis * self.mu * f.start.c.z)
    }
    fn sites(self, t: f64) -> [Rect; 2] {
        std::array::from_fn(|k| Rect {
            center: self.case.sites[k].center + self.case.v[k] * t,
            ..self.case.sites[k]
        })
    }
    fn physical(self, t: f64) -> Option<[f64; 2]> {
        let (c, v, p) = self.force(t);
        if self.case.v.iter().any(|a| (v - *a).dot(self.axis) <= 0.) {
            return None;
        }
        let sole = Rect {
            center: c,
            half: DVec2::new(0.18, 0.10),
            yaw: self
                .case
                .body
                .start
                .motion
                .q
                .to_euler(glam::EulerRot::ZYX)
                .0,
        };
        let [a, b] = self.sites(t).map(|site| site.patch(sole, false).unwrap());
        linear_program(p, [&a, &b])
    }
    fn inspect(self, i: Input, save: Option<&Path>) -> Result<Value> {
        let f = self.case.body;
        let fr = frame(self.case.inclined);
        let src = self.source(i, true);
        let mut sim = load(&src, true)?;
        let id = ids(&sim)?;
        let toe = bind(&sim, "default::toe::pad")?.1;
        let mass = f.shape.mass;
        let ng = -f.shape.gravity.z;
        let normal = fr.q * DVec3::Z;
        let tangent = fr.q * (-self.mu * self.axis).extend(0.);
        let rate = f.shape.gravity.truncate().dot(self.axis) - ng * self.mu;
        let first_stop = self
            .case
            .v
            .iter()
            .map(|v| (self.speed - v.dot(self.axis)) / (-rate))
            .fold(f64::INFINITY, f64::min);
        let mut spans = 0;
        let mut end = "horizon";
        let mut last = 0;
        let mut errors = [0f64; 11];
        let mut diss = 0.;
        let mut work = 0.;
        let mut tape = vec![];
        let mut analytic = vec![];
        let mut prev = None;
        for tick in 0..=480 {
            let t = tick as f64 * DT;
            let mut j = [0.; 2];
            let mut calls = 0;
            if tick > 0 {
                sim.step_exact(1)?;
                let receipt = last_refinement(&sim).filter(|r| r.iteration == tick);
                if receipt.is_none_or(|r| r.interval.shared_faces.is_empty()) {
                    end = if t >= first_stop - 1e-14 {
                        "slip reaches zero"
                    } else if self.physical(t).is_none() {
                        "pressure leaves support"
                    } else {
                        "bound declines"
                    };
                    break;
                }
                let report = receipt.unwrap();
                calls = report.interval.native_steps;
                let start_time = t - DT;
                let mut declared = [DVec3::ZERO; 2];
                let mut dloss = 0.;
                let mut dwork = 0.;
                for span in &report.interval.shared_faces {
                    spans += 1;
                    let start = start_time + span.start.as_secs_f64();
                    let mut span_loss = 0.;
                    let mut span_work = 0.;
                    let h = span.duration.as_secs_f64();
                    let s0 = span
                        .slip_start_end
                        .ok_or("missing kinetic support speeds")?;
                    for (k, site) in span.loads.iter().enumerate() {
                        let s = usize::from(site.support == toe);
                        let load = site.selected_normal_impulse;
                        j[s] += load;
                        declared[s] += DVec3::from(site.tangent_impulse_on_foot);
                        errors[0] = errors[0].max(
                            DVec3::from(site.tangent_impulse_on_foot).distance(tangent * load),
                        );
                        for (t_index, at) in [start, start + h].into_iter().enumerate() {
                            let (_, v, expected_center) = self.force(at);
                            let speed = (v - self.case.v[s]).dot(self.axis);
                            errors[1] = errors[1].max((s0[t_index][k] - speed).abs());
                            let Some(bounds) = self.physical(at) else {
                                return Err(format!(
                                    "accepted unsupported kinetic interval at {at}"
                                )
                                .into());
                            };
                            let b = if span.loads[1].support == toe {
                                span.selected_fraction_b
                            } else {
                                1. - span.selected_fraction_b
                            };
                            errors[2] = errors[2].max((bounds[0] - b).max(b - bounds[1]));
                            if k == 0 {
                                let center = span
                                    .loads
                                    .iter()
                                    .filter_map(|site| {
                                        let p = if t_index == 0 {
                                            site.start_pressure_world
                                        } else {
                                            site.end_pressure_world
                                        }?;
                                        Some(
                                            (fr.q.conjugate() * (DVec3::from(p) - fr.shift)
                                                - COMMON * at)
                                                * site.selected_normal_impulse
                                                / span.total_normal_impulse,
                                        )
                                    })
                                    .sum::<DVec3>();
                                errors[3] =
                                    errors[3].max(center.distance(expected_center.extend(0.)));
                            }
                        }
                        let mean = (s0[0][k] + s0[1][k]) * 0.5;
                        span_loss += self.mu * mean * load;
                        span_work +=
                            (fr.q * (COMMON + self.case.v[s].extend(0.))).dot(tangent * load);
                    }
                    diss += span.dissipated_energy;
                    work += span.support_tangent_work;
                    dwork += span_work;
                    dloss += span_loss;
                    errors[4] = errors[4]
                        .max((span_work - span.support_tangent_work).abs())
                        .max((span_loss - span.dissipated_energy).abs());
                }
                for (k, pad) in [id.pad, toe].into_iter().enumerate() {
                    let p = report
                        .interval
                        .pairs
                        .iter()
                        .find(|v| [v.a, v.b].contains(&pad) && [v.a, v.b].contains(&id.foot))
                        .ok_or("missing signed pair")?;
                    let sign = if p.a == id.foot { 1. } else { -1. };
                    errors[5] = errors[5]
                        .max((p.normal_impulse - j[k]).abs())
                        .max(DVec3::from(p.normal_impulse_on_a).distance(sign * normal * j[k]));
                    errors[6] = errors[6].max(
                        DVec3::from(
                            p.tangent_impulse_on_a
                                .ok_or("missing selected kinetic reaction")?,
                        )
                        .distance(sign * declared[k]),
                    );
                }
                if save.is_some() {
                    analytic.push(json!({"tick":tick,"normal_ns":j,"world_tangent_ns":declared.map(|v|v.to_array()),"dissipation_j":dloss,"belt_work_j":dwork}));
                }
            }
            let n = engine(&sim);
            let (foot, u, _) = departure::ingest(n, &id);
            let upper = u.unwrap();
            let p = (DVec3::from(foot.position) + DVec3::from(upper.position)) * 0.5;
            let v = (DVec3::from(foot.velocity) + DVec3::from(upper.velocity)) * 0.5;
            let (cp, vel, _) = self.force(t);
            errors[7] =
                errors[7].max(p.distance(fr.shift + fr.q * (COMMON * t + cp.extend(f.start.c.z))));
            errors[8] = errors[8].max(v.distance(fr.q * (COMMON + vel.extend(0.))));
            errors[9] = errors[9].max(DVec3::from(foot.angular_velocity).length());
            if let Some((first_c, first_k)) = prev {
                let kinetic = 0.5 * mass * v.length_squared();
                let gravity = fr.q * f.shape.gravity;
                let prescribed_normal = mass * ng * COMMON.z * t;
                let e: f64 =
                    kinetic - first_k - mass * gravity.dot(p - first_c) - prescribed_normal - work
                        + diss;
                errors[10] = errors[10].max(e.abs());
            } else {
                prev = Some((p, 0.5 * mass * v.length_squared()));
            }
            if save.is_some() {
                tape.push(self.case.capture(n, &id, toe, tick, calls, j));
            }
            last = tick;
        }
        if spans == 0 || errors.iter().any(|v| !v.is_finite() || *v > 1e-8) {
            return Err(format!(
                "registered kinetic {:?}, ticks {last}, errors {errors:?}",
                self.case.body.start.c
            )
            .into());
        }
        let data = json!({"spans":spans,"last_registered_tick":last,"end":end,"first_zero_slip_s":first_stop,"dissipated_j":diss,"belt_work_j":work,"errors_traction_speed_share_center_work_normal_tangent_position_velocity_spin_energy":errors});
        if let Some(out) = save {
            self.case.trace(out, "shared", tape)?;
            for (tag, calls) in [("ordinary", 1), ("eight", 8)] {
                let mut s = load(&self.source(i, false), true)?;
                let id = ids(&s)?;
                let toe = bind(&s, "default::toe::pad")?.1;
                let mut owner = s.world_mut().resource_mut::<PhysicsBackend>();
                let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
                let mut rows = vec![];
                for tick in 0..=last {
                    let mut j = [0.; 2];
                    if tick > 0 {
                        let r = n.step_contact_interval(Duration::from_micros(500), calls, 4)?;
                        for (k, pad) in [id.pad, toe].into_iter().enumerate() {
                            j[k] = r
                                .pairs
                                .iter()
                                .filter(|p| {
                                    [p.a, p.b].contains(&pad) && [p.a, p.b].contains(&id.foot)
                                })
                                .map(|p| p.normal_impulse)
                                .sum();
                        }
                    }
                    rows.push(self.case.capture(
                        n,
                        &id,
                        toe,
                        tick,
                        if tick > 0 { calls } else { 0 },
                        j,
                    ));
                }
                self.case.trace(out, tag, rows)?;
            }
            fs::write(out.join("uniform.sdf"), src)?;
            fs::write(
                out.join("public.json"),
                serde_json::to_vec(&json!({"registered":data,"measurements":analytic}))?,
            )?;
            let lanes = [
                ("ordinary", "One native solve"),
                ("eight", "Eight native solves"),
                ("shared", "Registered uniform sliding"),
            ]
            .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
            fs::write(
                out.join("comparison.json"),
                serde_json::to_vec_pretty(
                    &json!({"title":"The foot slows as two moving pads share kinetic friction","pair_totals_only":true,"capture_ticks":{"last_registered":last},"lanes":lanes}),
                )?,
            )?;
        }
        Ok(data)
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
    for mu in [0.12, 0.24, 0.46] {
        for yaw in [-0.24, 0.29] {
            for heading in [-0.31, 0.43] {
                for speed in [0.63, 0.94] {
                    for active in [-1., 1.] {
                        for inclined in [false, true] {
                            let axis = DVec2::from_angle(heading) * active;
                            let mut shape = p.shape;
                            shape.inertia = DMat3::from_cols(
                                DVec3::new(0.042, 0.002, -0.008),
                                DVec3::new(0.002, 0.053, 0.006),
                                DVec3::new(-0.008, 0.006, 0.059),
                            );
                            shape.gravity = (axis * 0.16).extend(-9.81);
                            let trial = Campaign {
                                case: Trial {
                                    body: Fixture {
                                        shape,
                                        start: State {
                                            c: DVec3::new(0., 0.012, 0.13),
                                            v: (axis * speed).extend(0.),
                                            motion: Motion {
                                                q: DQuat::from_rotation_z(yaw),
                                                w: DVec3::ZERO,
                                            },
                                        },
                                    },
                                    inclined,
                                    sites: [
                                        Rect {
                                            center: DVec2::new(-0.14, 0.),
                                            half: DVec2::new(0.114, 0.16),
                                            yaw: 0.12,
                                        },
                                        Rect {
                                            center: DVec2::new(0.14, 0.018),
                                            half: DVec2::new(0.118, 0.15),
                                            yaw: -0.18,
                                        },
                                    ],
                                    v: [axis * (-0.12), axis * 0.27],
                                },
                                axis,
                                mu,
                                speed,
                            };
                            let r = trial.inspect(i, None)?;
                            if demo.is_none()
                                && mu == 0.46
                                && yaw > 0.
                                && heading > 0.
                                && active < 0.
                                && inclined
                                && r["end"] == "slip reaches zero"
                            {
                                demo = Some(trial);
                            }
                            rows.push(json!({"mu":mu,"yaw":yaw,"heading":heading,"speed":speed,"direction":active,"inclined":inclined,"result":r}));
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("uniform.json"), serde_json::to_vec(&rows)?)?;
    demo.ok_or("missing physical slip handoff")?
        .inspect(i, Some(out))?;
    Ok(())
}
