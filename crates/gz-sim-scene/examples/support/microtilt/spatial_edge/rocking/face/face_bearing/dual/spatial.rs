// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Withheld shared-pressure cases with yaw, unequal prescribed pad velocities,
//! off-diagonal inertia, an inclined frame and gravitational tangent force.
use super::*;
#[derive(Clone, Copy)]
pub(super) struct Trial {
    pub(super) body: Fixture,
    pub(super) sites: [Rect; 2],
    pub(super) v: [DVec2; 2],
    pub(super) inclined: bool,
}
impl Trial {
    fn pads(self, t: f64) -> [Rect; 2] {
        std::array::from_fn(|i| Rect {
            center: self.sites[i].center + self.v[i] * t,
            ..self.sites[i]
        })
    }
    pub(super) fn source(self, i: Input, enable: bool) -> String {
        let f = frame(self.inclined);
        let body = self.body;
        let s = observations::source_with(
            i,
            Sole {
                shape: body.shape,
                incoming: body.start,
            },
            body.shape.gravity,
            f,
            false,
            "ordinary",
        );
        let at = s.find("    <model name=\"lift\">").unwrap();
        let mut src=s[..at].replace("A rocking foot is momentarily level as the opposite edge arrives at its frictionless support.","A yawing foot spans two finite, independently translating frictionless supports.");
        for (k, site) in self.sites.into_iter().enumerate() {
            let name = ["lift", "toe"][k];
            let color = ["0.28 0.57 0.30", "0.15 0.40 0.75"][k];
            let pos = f.shift + f.q * site.center.extend(-0.06);
            let q = f.q * DQuat::from_rotation_z(site.yaw);
            let (yaw, pitch, roll) = q.to_euler(glam::EulerRot::ZYX);
            let size = site.half * 2.;
            let v = f.q * (COMMON + self.v[k].extend(0.));
            src.push_str(&format!(r#"    <model name="{name}"><pose>{:.17} {:.17} {:.17} {roll:.17} {pitch:.17} {yaw:.17}</pose>
      <plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>{:.17} {:.17} {:.17}</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>
      <link name="pad"><collision name="platform"><geometry><box><size>{:.17} {:.17} 0.12</size></box></geometry><surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision><visual name="platform"><geometry><box><size>{:.17} {:.17} 0.12</size></box></geometry><material><diffuse>{color} 1</diffuse></material></visual></link></model>
"#,pos.x,pos.y,pos.z,v.x,v.y,v.z,size.x,size.y,size.x,size.y));
        }
        src.push_str("  </world>\n</sdf>\n");
        if enable { enrolled(&src) } else { src }
    }
    fn oracle(self, t: f64) -> Result<(Option<[f64; 2]>, Load)> {
        let mut full = self.body;
        full.shape.pad_half = [10., 10.];
        let force = full.pressure(t, false)?;
        let s = self.body.state(t);
        let (yaw, _, _) = s.motion.q.to_euler(glam::EulerRot::ZYX);
        let sole = Rect {
            center: s.c.truncate(),
            half: DVec2::new(0.18, 0.1),
            yaw,
        };
        let [pa, pb] = self.pads(t).map(|p| p.patch(sole, false).unwrap());
        Ok((linear_program(force.center, [&pa, &pb]), force))
    }
    pub(super) fn measure(self, i: Input, save: Option<&Path>) -> Result<Value> {
        let fr = frame(self.inclined);
        let mut sim = load(&self.source(i, true), true)?;
        let id = ids(&sim)?;
        let toe = bind(&sim, "default::toe::pad")?.1;
        let mut spans = 0;
        let mut single = 0;
        let mut err = [0f64; 9];
        let mut split = [1f64, 0.];
        let mut both = 0;
        let mut frames = vec![];
        let mut last = 0;
        let mut end = "horizon";
        let mut true_possible = 0;
        for tick in 0..=240u64 {
            let time = tick as f64 * DT;
            let mut j = [0.; 2];
            let mut calls = 0;
            if tick > 0 {
                sim.step_exact(1)?;
                let r = last_refinement(&sim).filter(|r| r.iteration == tick);
                let mut dt = Duration::ZERO;
                let mut available = true;
                for fraction in [0., 0.5, 1.] {
                    available &= self.oracle(time - DT + fraction * DT)?.0.is_some();
                }
                if available {
                    true_possible += 1;
                }
                if let Some(report) = r {
                    calls = report.interval.native_steps;
                    for h in &report.interval.shared_faces {
                        spans += 1;
                        dt += h.duration;
                        let right = usize::from(h.loads[1].support == toe);
                        let alpha = h.loads[right].selected_normal_impulse / h.total_normal_impulse;
                        split = [split[0].min(alpha), split[1].max(alpha)];
                        if alpha > 1e-8 && alpha < 1. - 1e-8 {
                            both += 1;
                        }
                        let mut torque = [DVec3::ZERO; 3];
                        let begin = time - DT + h.start.as_secs_f64();
                        let width = h.duration.as_secs_f64();
                        for (idx, fraction) in [0., 0.5, 1.].into_iter().enumerate() {
                            let t = begin + width * fraction;
                            let (range, law) = self.oracle(t)?;
                            let Some([lo, hi]) = range else {
                                return Err(
                                    format!("committed an infeasible spatial load at {t}").into()
                                );
                            };
                            let cap = h.loads[right].normal_impulse_bounds;
                            err[0] = err[0].max((lo - alpha).max(alpha - hi));
                            err[1] = err[1].max(
                                (cap[0] - lo * h.total_normal_impulse)
                                    .max(hi * h.total_normal_impulse - cap[1]),
                            );
                            let s = self.body.state(t);
                            torque[idx] = (law.center.extend(0.) - s.c).cross(DVec3::Z * law.force);
                            if idx == 1 {
                                continue;
                            }
                            let mut center = DVec3::ZERO;
                            let sites = self.pads(t);
                            for site in h.loads {
                                if site.selected_normal_impulse <= 0. {
                                    continue;
                                }
                                let k = usize::from(site.support == toe);
                                let world = DVec3::from(if idx == 0 {
                                    site.start_pressure_world.unwrap()
                                } else {
                                    site.end_pressure_world.unwrap()
                                });
                                let local = fr.q.conjugate() * (world - fr.shift) - COMMON * t;
                                let pos = sites[k].inverse(local.truncate());
                                err[2] = err[2]
                                    .max((pos.abs() - sites[k].half).max_element())
                                    .max(local.z.abs());
                                center +=
                                    local * site.selected_normal_impulse / h.total_normal_impulse;
                            }
                            err[3] = err[3].max(center.distance(law.center.extend(0.)));
                        }
                        let exact = width * (torque[0] + 4. * torque[1] + torque[2]) / 6.;
                        err[4] =
                            err[4].max(exact.distance(
                                fr.q.conjugate() * DVec3::from(h.angular_impulse_about_com),
                            ));
                    }
                    for h in &report.interval.face_bearings {
                        single += 1;
                        dt += h.duration;
                    }
                    for (k, site) in [id.pad, toe].into_iter().enumerate() {
                        if let Some(p) =
                            report.interval.pairs.iter().find(|p| {
                                [p.a, p.b].contains(&site) && [p.a, p.b].contains(&id.foot)
                            })
                        {
                            j[k] = p.normal_impulse;
                            let vector = fr.q.conjugate()
                                * DVec3::from(p.normal_impulse_on_a)
                                * if p.a == id.foot { 1. } else { -1. };
                            if dt == Duration::from_micros(500) {
                                err[5] = err[5].max(vector.distance(DVec3::Z * j[k]));
                            }
                        }
                    }
                }
                if dt != Duration::from_micros(500) {
                    end = if available {
                        "conservative handoff"
                    } else {
                        "physical handoff"
                    };
                    break;
                }
                let (f, u, _) = departure::ingest(engine(&sim), &id);
                let upper = u.unwrap();
                let center = fr.q.conjugate()
                    * ((DVec3::from(f.position) + DVec3::from(upper.position)) * 0.5 - fr.shift)
                    - COMMON * time;
                let wanted = self.body.state(time);
                let v = fr.q.conjugate()
                    * ((DVec3::from(f.velocity) + DVec3::from(upper.velocity)) * 0.5)
                    - COMMON;
                err[6] = err[6].max(center.distance(wanted.c));
                err[7] = err[7].max(v.distance(wanted.v));
                err[8] = err[8].max(
                    (fr.q.conjugate() * DVec3::from(f.angular_velocity)).distance(wanted.motion.w),
                );
            }
            last = tick;
            if save.is_some() {
                frames.push(self.capture(engine(&sim), &id, toe, tick, calls, j));
            }
        }
        if err.iter().any(|v| !v.is_finite() || *v > 1e-8) {
            return Err(format!("spatial shared trial failed {err:?}").into());
        }
        if let Some(out) = save {
            if last == 0 {
                return Err("empty spatial replay".into());
            }
            self.trace(out, "shared", frames)?;
            for (tag, calls) in [("ordinary", 1), ("eight", 8)] {
                let mut s = load(&self.source(i, false), true)?;
                let id = ids(&s)?;
                let toe = bind(&s, "default::toe::pad")?.1;
                let mut owner = s.world_mut().resource_mut::<PhysicsBackend>();
                let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
                let mut frames = vec![];
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
                    frames.push(self.capture(
                        n,
                        &id,
                        toe,
                        tick,
                        if tick > 0 { calls } else { 0 },
                        j,
                    ));
                }
                self.trace(out, tag, frames)?;
            }
            fs::write(out.join("duo.sdf"), self.source(i, true))?;
            let lanes = [
                ("ordinary", "One native solve"),
                ("eight", "Eight native solves"),
                ("shared", "Registered spatial shared pressure"),
            ]
            .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
            fs::write(
                out.join("comparison.json"),
                serde_json::to_vec_pretty(
                    &json!({"title":"A yawing sole transfers load between two skewed moving pads","pair_totals_only":true,"capture_ticks":{"last_qualified":last},"lanes":lanes}),
                )?,
            )?;
        }
        Ok(
            json!({"spans":spans,"single":single,"last_supported_tick":last,"end":end,"eligible_samples":true_possible,"selected_fraction_toe":split,"positive_both":both,"fraction_outer_site_center_moment_vector_com_velocity_spin":err}),
        )
    }
    pub(super) fn trace(self, out: &Path, name: &str, frames: Vec<Value>) -> Result<()> {
        let pad = |p: Rect| [p.half.x, p.half.y, 0.06];
        let geometry = json!({"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":pad(self.sites[0]),"opponent_half_extents":pad(self.sites[1])});
        fs::write(
            out.join(format!("traces/{name}.json")),
            serde_json::to_vec(&json!({"geometry":geometry,"frames":frames}))?,
        )?;
        Ok(())
    }
    pub(super) fn capture(
        self,
        n: &MultiFidelityBackend,
        id: &Reading,
        toe: BodyId,
        tick: u64,
        calls: u32,
        j: [f64; 2],
    ) -> Value {
        let (f, u, p) = departure::ingest(n, id);
        let mut frame = row(tick, calls, j[0] + j[1], (f, u.unwrap(), p));
        frame["opponent"] = wire(n.state(toe).unwrap());
        frame["support_normal_ns"] = json!(j);
        frame
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
    let mut shape = p.shape;
    shape.inertia = DMat3::from_cols(
        DVec3::new(0.042, 0.002, -0.008),
        DVec3::new(0.002, 0.053, 0.006),
        DVec3::new(-0.008, 0.006, 0.059),
    );
    shape.gravity = DVec3::new(0.25, -0.12, -9.81);
    let mut results = Vec::new();
    let mut demo = None;
    for x in [-0.085, -0.012, 0.064] {
        for y in [-0.045, 0.04] {
            for yaw in [-0.37, 0.21, 0.59] {
                for spin in [-5., 0., 4.] {
                    for skew in [0., 0.23] {
                        for inclined in [false, true] {
                            let trial = Trial {
                                body: Fixture {
                                    shape,
                                    start: State {
                                        c: DVec3::new(x, y, 0.13),
                                        v: DVec3::new(0.45, -0.13, 0.),
                                        motion: Motion {
                                            q: DQuat::from_rotation_z(yaw),
                                            w: DVec3::Z * spin,
                                        },
                                    },
                                },
                                inclined,
                                sites: [
                                    Rect {
                                        center: DVec2::new(-0.14, -0.008),
                                        half: DVec2::new(0.105, 0.105),
                                        yaw: skew,
                                    },
                                    Rect {
                                        center: DVec2::new(0.15, 0.024),
                                        half: DVec2::new(0.11, 0.12),
                                        yaw: -0.7 * skew,
                                    },
                                ],
                                v: [DVec2::new(-0.18, 0.11), DVec2::new(0.075, -0.026)],
                            };
                            let v = trial.measure(i, None)?;
                            if demo.is_none()
                                && skew > 0.
                                && yaw > 0.
                                && spin > 0.
                                && inclined
                                && v["positive_both"].as_u64().unwrap() > 170
                            {
                                demo = Some(trial);
                            }
                            results.push(json!({"x":x,"y":y,"yaw":yaw,"spin":spin,"skew":skew,"inclined":inclined,"result":v}));
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("spatial.json"), serde_json::to_vec(&results)?)?;
    let plot = demo
        .ok_or("missing sustained spatial shared-pressure demo")?
        .measure(i, Some(out))?;
    fs::write(out.join("public.json"), serde_json::to_vec(&plot)?)?;
    Ok(())
}
