// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two translating belts stop, reaccelerate a settled foot in another
//! direction, then stop it again. The reference integrates its relative
//! velocity and kinetic energy without using registered contact receipts.
use super::*;
const TICKS: u64 = 220;
#[derive(Clone, Copy)]
struct MotionRef {
    c: DVec2,
    v: DVec2,
}
#[derive(Clone, Copy)]
struct Interval {
    end: MotionRef,
    slip: f64,
    axis: DVec2,
}
impl MotionRef {
    fn advance(self, belt: DVec2, a: f64, h: f64) -> Interval {
        let r = self.v - belt;
        let speed = r.length();
        let dir = r.normalize_or_zero();
        let t = (speed / a).min(h);
        let v = belt + dir * (speed - a * h).max(0.);
        let c = self.c + belt * h + r * t - 0.5 * a * t * t * dir;
        Interval {
            end: Self { c, v },
            slip: t,
            axis: dir,
        }
    }
}
fn change(tick: u64, old: DVec2, axis: DVec2, a: f64) -> Option<DVec2> {
    match tick {
        85 => Some(old + axis.perp() * (a * 0.011621537)),
        161 => Some(old + DVec2::from_angle(0.49).rotate(-axis) * (a * 0.017408771)),
        _ => None,
    }
}
impl Campaign {
    fn arrest_source(self, i: Input) -> String {
        self.source(i, true).replace(
            "</share_uniform_sliding>",
            "</share_uniform_sliding><settle_comoving_supports>true</settle_comoving_supports>",
        )
    }
    fn measure_arrest(self, i: Input, save: Option<&Path>) -> Result<Value> {
        let fr = frame(self.case.inclined);
        let body = self.case.body;
        let src = self.arrest_source(i);
        let mut sim = load(&src, true)?;
        let id = ids(&sim)?;
        let heel = bind(&sim, "default::lift::pad")?;
        let toe = bind(&sim, "default::toe::pad")?;
        let mut r = MotionRef {
            c: body.start.c.truncate(),
            v: body.start.v.truncate(),
        };
        let mut belt = self.case.v[0];
        let mut center = self.case.sites.map(|s| s.center);
        let a = self.mu * 9.81;
        let m = body.shape.mass;
        let spin = body.start.motion.q.to_euler(glam::EulerRot::ZYX).0;
        let mut tape = vec![];
        let mut samples = vec![];
        let mut err = [0f64; 11];
        let mut cumulative = [0.; 2];
        let mut phases = [0; 3];
        let mut stops = vec![];
        let mut prior_kinetic = true;
        for tick in 0..=TICKS {
            let t = tick as f64 * DT;
            let t0 = t - DT;
            let mut j = [0.; 2];
            let mut calls = 0;
            if tick > 0 {
                if let Some(new) = change(tick, belt, self.axis, a) {
                    belt = new;
                    let world = (fr.q * (COMMON + belt.extend(0.))).to_array();
                    for link in [heel.0, toe.0] {
                        gz_sim_scene::queue_link_world_velocity(&mut sim, link, world, [0.; 3])?;
                    }
                    prior_kinetic = true;
                }
                let truth = r.advance(belt, a, DT);
                sim.step_exact(1)?;
                let report = last_refinement(&sim)
                    .filter(|v| v.iteration == tick)
                    .ok_or_else(|| format!("absent Bevy arrest at {tick}"))?;
                let p = &report.interval;
                if p.shared_faces.is_empty() {
                    return Err(format!(
                        "generic handoff at tick {tick}, v {:?}, belt {belt:?}",
                        r.v
                    )
                    .into());
                }
                let mut jtan = [DVec3::ZERO; 2];
                let mut kinetic_seconds = 0.;
                let mut tau = 0.;
                let mut energy = [0.; 2];
                let mut ended = false;
                for s in &p.shared_faces {
                    let h = s.physical_duration_seconds;
                    err[0] = err[0].max((s.physical_start_seconds - tau).abs());
                    if s.comoving_rest {
                        ended = true;
                    } else {
                        if ended {
                            return Err(
                                "kinetic phase after arrest inside an unchanged public interval"
                                    .into(),
                            );
                        }
                        kinetic_seconds += h;
                    }
                    let local_axis = if s.comoving_rest {
                        DVec2::ZERO
                    } else {
                        truth.axis
                    };
                    let speed = s.slip_start_end.ok_or("missing Coulomb mode slip")?;
                    let share = if s.loads[1].support == toe.1 {
                        s.selected_fraction_b
                    } else {
                        1. - s.selected_fraction_b
                    };
                    for (side, at) in [tau, tau + h].into_iter().enumerate() {
                        let state = r.advance(belt, a, at).end;
                        let cp = state.c + local_axis * self.mu * body.start.c.z;
                        let sole = Rect {
                            center: state.c,
                            half: DVec2::new(0.18, 0.10),
                            yaw: spin,
                        };
                        let patches: [Vec<DVec2>; 2] = std::array::from_fn(|k| {
                            Rect {
                                center: center[k] + belt * at,
                                ..self.case.sites[k]
                            }
                            .patch(sole, false)
                            .unwrap()
                        });
                        let cap = linear_program(cp, [&patches[0], &patches[1]])
                            .ok_or("independent pressure infeasible")?;
                        err[1] = err[1].max((cap[0] - share).max(share - cap[1]));
                        let mut achieved = DVec3::ZERO;
                        for (k, site) in s.loads.iter().enumerate() {
                            let v = if side == 0 {
                                site.start_pressure_world
                            } else {
                                site.end_pressure_world
                            };
                            if let Some(w) = v {
                                achieved += (fr.q.conjugate() * (DVec3::from(w) - fr.shift)
                                    - COMMON * (t0 + at))
                                    * site.selected_normal_impulse
                                    / s.total_normal_impulse;
                            }
                            err[2] = err[2].max((speed[side][k] - (state.v - belt).length()).abs());
                        }
                        err[3] = err[3].max(achieved.distance(cp.extend(0.)));
                    }
                    for site in s.loads {
                        let k = usize::from(site.support == toe.1);
                        j[k] += site.selected_normal_impulse;
                        let impulse = DVec3::from(site.tangent_impulse_on_foot);
                        jtan[k] += impulse;
                        err[4] = err[4].max(impulse.distance(
                            fr.q
                                * (-self.mu * local_axis * site.selected_normal_impulse).extend(0.),
                        ));
                    }
                    energy[0] += s.dissipated_energy;
                    energy[1] += s.support_tangent_work;
                    tau += h;
                }
                err[0] = err[0]
                    .max((tau - DT).abs())
                    .max((kinetic_seconds - truth.slip).abs());
                if prior_kinetic && ended {
                    stops.push(t0 + kinetic_seconds);
                    prior_kinetic = false;
                }
                phases[if ended && kinetic_seconds > 1e-12 {
                    1
                } else if ended {
                    2
                } else {
                    0
                }] += 1;
                let jy = fr.q * (m * (truth.end.v - r.v)).extend(0.);
                err[5] = err[5]
                    .max((jtan[0] + jtan[1]).distance(jy))
                    .max((j[0] + j[1] - m * 9.81 * DT).abs());
                for (k, pad) in [heel.1, toe.1].into_iter().enumerate() {
                    let pair = p
                        .pairs
                        .iter()
                        .find(|p| [p.a, p.b].contains(&pad) && [p.a, p.b].contains(&id.foot))
                        .ok_or("missing pair")?;
                    let sign = if pair.a == id.foot { 1. } else { -1. };
                    err[5] = err[5].max(
                        jtan[k].distance(
                            sign * DVec3::from(
                                pair.tangent_impulse_on_a
                                    .ok_or("unknown registered friction impulse")?,
                            ),
                        ),
                    );
                }
                let lost = 0.5
                    * m
                    * ((r.v - belt).length_squared() - (truth.end.v - belt).length_squared());
                let work = jy.dot(fr.q * (COMMON + belt.extend(0.)));
                err[6] = err[6]
                    .max((energy[0] - lost).abs())
                    .max((energy[1] - work).abs());
                for k in 0..2 {
                    cumulative[k] += energy[k];
                    center[k] += belt * DT;
                }
                r = truth.end;
                calls = p.native_steps;
                if save.is_some() {
                    samples.push(json!({"tick":tick,"kinetic_seconds":kinetic_seconds,"rest_seconds":DT-kinetic_seconds,"normal_ns":j,"tangent_ns":jtan.map(|v|v.to_array()),"dissipated_j":energy[0],"belt_work_j":energy[1]}));
                }
            }
            let n = engine(&sim);
            let (f, u, _) = departure::ingest(n, &id);
            let u = u.unwrap();
            let cp = (DVec3::from(f.position) + DVec3::from(u.position)) * 0.5;
            let v = (DVec3::from(f.velocity) + DVec3::from(u.velocity)) * 0.5;
            err[7] = err[7]
                .max(cp.distance(fr.shift + fr.q * (COMMON * t + r.c.extend(body.start.c.z))));
            err[8] = err[8].max(v.distance(fr.q * (COMMON + r.v.extend(0.))));
            err[9] = err[9].max(DVec3::from(f.angular_velocity).length());
            let initial = (COMMON + body.start.v).length_squared();
            err[10] = err[10].max(
                (0.5 * m * (v.length_squared() - initial) - cumulative[1] + cumulative[0]).abs(),
            );
            if save.is_some() {
                tape.push(self.case.capture(n, &id, toe.1, tick, calls, j));
            }
        }
        if stops.len() != 3 || err.iter().any(|v| !v.is_finite() || *v > 1e-8) {
            return Err(format!("three arrests: {stops:?} e {err:?}").into());
        }
        let facts = json!({"spans_kinetic_mixed_rest":phases,"physical_stops_s":stops,"integrated_dissipation_and_belt_work_j":cumulative,"errors_clock_fraction_speed_center_siteimpulse_totalimpulse_work_COM_velocity_spin_energy":err});
        if let Some(out) = save {
            self.case.trace(out, "registered", tape)?;
            fs::write(out.join("arrest.sdf"), src)?;
            fs::write(
                out.join("public.json"),
                serde_json::to_vec(&json!({"field":facts,"samples":samples}))?,
            )?;
            for (label, calls) in [("ordinary", 1), ("eight", 8)] {
                let mut scene = load(&self.source(i, false), true)?;
                let id = ids(&scene)?;
                let toe = bind(&scene, "default::toe::pad")?.1;
                let mut device = scene.world_mut().resource_mut::<PhysicsBackend>();
                let n = device.downcast_mut::<MultiFidelityBackend>().unwrap();
                let mut frames = vec![];
                let mut belt = self.case.v[0];
                for tick in 0..=TICKS {
                    let mut j = [0.; 2];
                    if tick > 0 {
                        if let Some(new) = change(tick, belt, self.axis, a) {
                            belt = new;
                            let v = (fr.q * (COMMON + belt.extend(0.))).to_array();
                            for pad in [id.pad, toe] {
                                n.set_world_velocity(pad, v, [0.; 3])?;
                            }
                        }
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
                    frames.push(self.case.capture(
                        n,
                        &id,
                        toe,
                        tick,
                        if tick == 0 { 0 } else { calls },
                        j,
                    ));
                }
                self.case.trace(out, label, frames)?;
            }
            let lanes = [
                ("ordinary", "One native solve"),
                ("eight", "Eight native solves"),
                ("registered", "Registered sliding and arrest"),
            ]
            .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
            let ticks = stops
                .iter()
                .map(|t| (t / DT).ceil() as u64)
                .collect::<Vec<_>>();
            fs::write(
                out.join("comparison.json"),
                serde_json::to_vec_pretty(
                    &json!({"title":"Three sliding arrests under moving belt commands","pair_totals_only":true,"capture_ticks":{"first_arrest":ticks[0],"second_arrest":ticks[1],"third_arrest":ticks[2]},"lanes":lanes}),
                )?,
            )?;
        }
        Ok(facts)
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
    for mu in [0.12, 0.31, 0.46] {
        for heading in [-1.12, 0.43] {
            for inclined in [false, true] {
                for belt in [DVec2::ZERO, DVec2::new(0.12, -0.06)] {
                    for stop in [0.25e-9, 0.00021381976, 0.00049999975, 0.03721381976] {
                        let axis = DVec2::from_angle(heading);
                        let mut shape = p.shape;
                        shape.gravity = -9.81 * DVec3::Z;
                        shape.inertia = DMat3::from_cols(
                            DVec3::new(0.042, 0.002, -0.008),
                            DVec3::new(0.002, 0.053, 0.006),
                            DVec3::new(-0.008, 0.006, 0.059),
                        );
                        let state = State {
                            c: DVec3::new(0., 0.012, 0.13),
                            v: (belt + axis * (mu * 9.81 * stop)).extend(0.),
                            motion: Motion {
                                q: DQuat::from_rotation_z(0.29),
                                w: DVec3::ZERO,
                            },
                        };
                        let trial = Campaign {
                            axis,
                            mu,
                            speed: state.v.truncate().length(),
                            case: Trial {
                                body: Fixture {
                                    shape,
                                    start: state,
                                },
                                inclined,
                                sites: [
                                    Rect {
                                        center: DVec2::new(-0.13, 0.),
                                        half: DVec2::new(0.121, 0.16),
                                        yaw: 0.12,
                                    },
                                    Rect {
                                        center: DVec2::new(0.14, 0.018),
                                        half: DVec2::new(0.126, 0.15),
                                        yaw: -0.18,
                                    },
                                ],
                                v: [belt; 2],
                            },
                        };
                        let save = mu == 0.31
                            && heading > 0.
                            && inclined
                            && belt != DVec2::ZERO
                            && stop > 0.03;
                        let r = trial.measure_arrest(i, save.then_some(out))?;
                        rows.push(json!({"mu":mu,"heading":heading,"inclined":inclined,"belt_m_s":belt.to_array(),"initial_stop_s":stop,"result":r}));
                    }
                }
            }
        }
    }
    fs::write(out.join("arrests.json"), serde_json::to_vec(&rows)?)?;
    Ok(())
}
