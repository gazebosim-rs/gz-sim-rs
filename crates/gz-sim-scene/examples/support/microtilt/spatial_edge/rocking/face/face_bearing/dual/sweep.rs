// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Pressure can leave and return between identical public endpoints. A sealed
//! quadratic bound also admits inward motion from nanometer-scale margins.
use super::spatial::Trial;
use super::*;
pub(in super::super::super::super::super::super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let (i, base, _) = fixture(Case {
        active: 0,
        pitch: 0.18,
        opening: 0.3,
        yaw: 0.,
        pitch_rate: 0.6,
    });
    let mut rows = vec![];
    let mut demo = None;
    for strength in [0.125, 1.5, 11.] {
        for angle in [-1.12, 0.17, 2.42] {
            let dir = DVec2::from_angle(angle);
            for tilted in [false, true] {
                for scale in [0.1, 0.6, 0.96, 1.2, 2.5, 6.] {
                    for start_speed in [0., 0.5, 1.] {
                        let mut shape = base.shape;
                        shape.gravity = (-strength * dir).extend(-9.81);
                        let belt = dir.rotate(DVec2::new(0.19, -0.12));
                        let speed = start_speed * strength * DT;
                        let curvature = 0.125 * strength * DT * DT;
                        let initial_margin = scale * curvature;
                        let sites = [-0.075, 0.075].map(|y| Rect {
                            center: dir.rotate(DVec2::new(initial_margin - 0.075, y)),
                            half: DVec2::new(0.075, 0.045),
                            yaw: angle,
                        });
                        let f = Trial {
                            body: Fixture {
                                shape,
                                start: State {
                                    c: DVec3::new(0., 0., 0.13),
                                    v: (belt + dir * speed).extend(0.),
                                    motion: Motion {
                                        q: DQuat::from_rotation_z(angle),
                                        w: DVec3::ZERO,
                                    },
                                },
                            },
                            sites,
                            v: [belt; 2],
                            inclined: tilted,
                        };
                        let peak_time = (speed / strength).clamp(0., DT);
                        let progress = |t: f64| speed * t - 0.5 * strength * t * t;
                        let min = initial_margin - progress(peak_time);
                        let ends = initial_margin - progress(DT);
                        let coarse_fails = initial_margin.min(ends) <= curvature;
                        let mut sim = load(&f.source(i, true), true)?;
                        let id = ids(&sim)?;
                        sim.step_exact(1)?;
                        let report = last_refinement(&sim).map(|r| &r.interval);
                        let spans = report.map_or(&[][..], |r| r.shared_faces.as_slice());
                        let duration: f64 = spans.iter().map(|r| r.physical_duration_seconds).sum();
                        let held = (duration - DT).abs() < 1e-14;
                        if held != (min > 0.) {
                            return Err(format!("wrong swept pressure admission gravity {strength} margin {initial_margin} peak {min} held {held}").into());
                        }
                        let mut error = [0f64; 3];
                        if held {
                            let sum = spans.iter().map(|r| r.total_normal_impulse).sum::<f64>();
                            let (a, b, _) = departure::ingest(engine(&sim), &id);
                            let fr = frame(tilted);
                            let measured =
                                0.5 * (DVec3::from(a.position) + DVec3::from(b.unwrap().position));
                            let want = f.body.start.c
                                + f.body.start.v * DT
                                + (dir * (-0.5 * strength * DT * DT)).extend(0.);
                            error[0] = measured.distance(fr.shift + fr.q * (COMMON * DT + want));
                            error[1] = (sum - base.shape.mass * 9.81 * DT).abs();
                            for s in spans {
                                if s.loads.iter().any(|x| x.selected_normal_impulse <= 0.) {
                                    return Err("missing mandatory support load".into());
                                }
                                for t in [
                                    s.physical_start_seconds,
                                    s.physical_start_seconds + s.physical_duration_seconds,
                                ] {
                                    let cp = (belt * t + dir * progress(t)).extend(0.);
                                    let mut p = DVec3::ZERO;
                                    for l in s.loads {
                                        let pt = if t == s.physical_start_seconds {
                                            l.start_pressure_world
                                        } else {
                                            l.end_pressure_world
                                        };
                                        p += (fr.q.conjugate()
                                            * (DVec3::from(pt.unwrap()) - fr.shift)
                                            - COMMON * t)
                                            * l.selected_normal_impulse
                                            / s.total_normal_impulse;
                                    }
                                    error[2] = error[2].max(p.distance(cp));
                                }
                            }
                            if error.iter().any(|x| *x > 1e-10) {
                                return Err(format!("invalid edge pressure: {error:?}").into());
                            }
                        }
                        if demo.is_none()
                            && strength == 11.
                            && scale == 0.1
                            && angle == 0.17
                            && tilted
                            && start_speed == 0.
                        {
                            demo = Some(f);
                        }
                        rows.push(json!({"g_m_s2":strength,"x_axis_angle":angle,"inclined":tilted,"incoming_speed_factor":start_speed,"margin_factor":scale,"initial_margin_m":initial_margin,"end_margin_m":ends,"minimum_margin_m":min,"peak_time_s":peak_time,"earlier_symmetric_erosion_declines":coarse_fails,"full_registered_tick":held,"errors_COM_impulse_center":error}));
                    }
                }
            }
        }
    }
    fs::write(out.join("sweep.json"), serde_json::to_vec(&rows)?)?;
    demo.ok_or("no near-edge inward nominee")?
        .measure(i, Some(out))?;
    let mut m: Value = serde_json::from_slice(&fs::read(out.join("comparison.json"))?)?;
    m["title"] = json!("The foot accelerates inward from two nanometer-scale pressure margins");
    m["capture_ticks"] = json!({"near_edge":1,"inward":240});
    fs::write(out.join("comparison.json"), serde_json::to_vec_pretty(&m)?)?;
    Ok(())
}
