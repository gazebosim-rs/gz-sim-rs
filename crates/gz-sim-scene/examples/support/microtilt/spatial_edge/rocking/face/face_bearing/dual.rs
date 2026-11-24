// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Independent pressure feasibility on two finite coplanar pads.
#[path = "dual/sweep.rs"]
pub(crate) mod sweep;
#[path = "dual/sliding.rs"]
pub(in super::super::super::super::super::super::super) mod sliding;
#[path = "dual/spatial.rs"]
pub(in super::super::super::super::super::super::super) mod spatial;
#[path = "dual/uniform.rs"]
pub(in super::super::super::super::super::super::super) mod uniform;
use super::*;
use gz_physics::{FaceOverlap, coplanar_load_range};
#[derive(Clone, Copy)]
struct Rect {
    center: DVec2,
    half: DVec2,
    yaw: f64,
}
impl Rect {
    fn transform(self, p: DVec2) -> DVec2 {
        self.center + DVec2::from_angle(self.yaw).rotate(p)
    }
    fn inverse(self, p: DVec2) -> DVec2 {
        DVec2::from_angle(-self.yaw).rotate(p - self.center)
    }
    fn vertices(self) -> [DVec2; 4] {
        let h = self.half;
        [
            DVec2::new(-h.x, -h.y),
            DVec2::new(h.x, -h.y),
            h,
            DVec2::new(-h.x, h.y),
        ]
        .map(|p| self.transform(p))
    }
    fn patch(self, sole: Rect, production: bool) -> Result<Vec<DVec2>> {
        let raw = sole.vertices().map(|v| self.inverse(v));
        let p = if production {
            let clipped = FaceOverlap::clip(raw, self.half).ok_or("unresolved sole/pad polygon")?;
            clipped.vertices[..clipped.count as usize].to_vec()
        } else {
            points(raw, self.half)
        };
        Ok(p.into_iter().map(|v| self.transform(v)).collect())
    }
}
// Three scalar equations (unit load and its two moments) suffice. A basic
// feasible pressure distribution has no more than three positive vertices.
fn linear_program(c: DVec2, pads: [&[DVec2]; 2]) -> Option<[f64; 2]> {
    let vertices = pads
        .iter()
        .enumerate()
        .flat_map(|(k, p)| p.iter().map(move |v| (*v - c, k as f64)))
        .collect::<Vec<_>>();
    let mut bounds = [f64::INFINITY, f64::NEG_INFINITY];
    let mut candidate = |weights: &[(usize, f64)]| {
        if weights.iter().any(|&(_, x)| x < -1e-12) {
            return;
        }
        if weights
            .iter()
            .map(|&(i, x)| vertices[i].0 * x)
            .sum::<DVec2>()
            .length()
            > 1e-12
        {
            return;
        }
        let f = weights
            .iter()
            .map(|&(i, x)| vertices[i].1 * x)
            .sum::<f64>()
            .clamp(0., 1.);
        bounds[0] = bounds[0].min(f);
        bounds[1] = bounds[1].max(f);
    };
    for i in 0..vertices.len() {
        candidate(&[(i, 1.)]);
        for j in 0..i {
            let (a, b) = (vertices[i].0, vertices[j].0);
            let d = b - a;
            if d.length_squared() > 1e-24 {
                let u = -a.dot(d) / d.length_squared();
                candidate(&[(i, 1. - u), (j, u)]);
            }
            for k in 0..j {
                let e = vertices[k].0 - a;
                let det = d.perp_dot(e);
                if det.abs() < 1e-18 {
                    continue;
                }
                let u = (-a).perp_dot(e) / det;
                let v = d.perp_dot(-a) / det;
                candidate(&[(i, 1. - u - v), (j, u), (k, v)]);
            }
        }
    }
    bounds[0].is_finite().then_some(bounds)
}
fn field() -> Result<Value> {
    let mut rows = vec![];
    for x in [-0.28, -0.19, -0.095, -0.012, 0.046, 0.11, 0.21, 0.29] {
        for y in [-0.12, -0.035, 0.045, 0.12] {
            for yaw in [-0.54, -0.22, 0.08, 0.31, 0.72] {
                for shift in [-0.1, 0., 0.085] {
                    for skew in [-0.36, 0., 0.21] {
                        let sole = Rect {
                            center: DVec2::new(x, y),
                            half: DVec2::new(0.18, 0.1),
                            yaw,
                        };
                        let sites = [
                            Rect {
                                center: DVec2::new(-0.14 + shift, -0.019),
                                half: DVec2::new(0.113, 0.12),
                                yaw: skew,
                            },
                            Rect {
                                center: DVec2::new(0.17, 0.026),
                                half: DVec2::new(0.145, 0.10),
                                yaw: -0.5 * skew,
                            },
                        ];
                        let center = sole.transform(DVec2::new(0.009, -0.012));
                        let [pa, pb] = sites.map(|s| s.patch(sole, true).unwrap());
                        let [oa, ob] = sites.map(|s| s.patch(sole, false).unwrap());
                        let range = coplanar_load_range(center, [&pa, &pb])?.map(|r| r.fraction_b);
                        let oracle = linear_program(center, [&oa, &ob]);
                        if range.is_some() != oracle.is_some()
                            || range.zip(oracle).is_some_and(|(r, o)| {
                                (r[0] - o[0]).abs().max((r[1] - o[1]).abs()) > 2e-10
                            })
                        {
                            return Err(format!("load capacity mismatch {x} {y} {yaw} {shift} {skew}: {range:?} vs {oracle:?}").into());
                        }
                        rows.push(json!({"x":x,"y":y,"yaw":yaw,"shift":shift,"skew":skew,"polygon_sizes":[pa.len(),pb.len()],"fraction_toe":range,"linear_program":oracle}));
                    }
                }
            }
        }
    }
    Ok(rows.into())
}
const V: f64 = 0.32;
const COMMON: DVec3 = DVec3::new(0.17, -0.09, 0.22);
const N: f64 = 2. * 9.81;
fn heel(t: f64) -> (f64, f64) {
    if t < 0.42 {
        (-0.13 - 0.18 * t, -0.18)
    } else if t < 0.54 {
        (-0.13 - 0.18 * 0.42 + 0.9 * (t - 0.42), 0.9)
    } else {
        (-0.13 - 0.18 * 0.42 + 0.9 * 0.12 - 0.52 * (t - 0.54), -0.52)
    }
}
fn box_at(t: f64) -> [f64; 6] {
    let (left, _) = heel(t);
    let foot = -0.02 + V * t;
    [
        left - 0.125,
        left + 0.125,
        0.03,
        0.35,
        foot - 0.18,
        foot + 0.18,
    ]
}
fn shares(t: f64) -> Option<[f64; 2]> {
    let r = box_at(t);
    let area = |x0: f64, x1: f64| {
        let lo = x0.max(r[4]);
        let hi = x1.min(r[5]);
        if lo > hi {
            vec![]
        } else {
            vec![
                DVec2::new(lo, -0.10),
                DVec2::new(hi, -0.10),
                DVec2::new(hi, 0.10),
                DVec2::new(lo, 0.10),
            ]
        }
    };
    let a = area(r[0], r[1]);
    let b = area(r[2], r[3]);
    coplanar_load_range(DVec2::new(-0.02 + V * t, 0.), [&a, &b])
        .unwrap()
        .map(|p| p.fraction_b)
}
// Within an actuator interval every clipped bound and the pressure coordinate
// is affine until two endpoints cross. On each piece the active load bounds
// are monotone linear-fractional functions. Sum inward/outward rectangles
// over those pieces to cover the exact integrated continuum of pressure splits.
fn integrated(t0: f64, t1: f64) -> Result<[[f64; 2]; 2]> {
    let center = |t: f64| -0.02 + V * t;
    let mut a = box_at(t0).to_vec();
    a.push(center(t0));
    let mut b = box_at(t1).to_vec();
    b.push(center(t1));
    let mut breaks = vec![t0, t1];
    for i in 0..a.len() {
        for j in 0..i {
            let p = a[i] - a[j];
            let q = b[i] - b[j];
            if p * q < 0. {
                breaks.push(t0 + (t1 - t0) * p / (p - q));
            }
        }
    }
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    let mut total = [[0.; 2]; 2];
    for x in breaks.windows(2) {
        let [ta, tb] = [x[0], x[1]];
        let h = tb - ta;
        if h < 1e-16 {
            continue;
        }
        let middle = box_at((ta + tb) * 0.5);
        let positive = [
            middle[0].max(middle[4]) < middle[1].min(middle[5]),
            middle[2].max(middle[4]) < middle[3].min(middle[5]),
        ];
        let bound = |t: f64| -> Result<[f64; 2]> {
            let r = box_at(t);
            let c = center(t);
            match positive {
                [false, false] => Err("unsupported analytical COM".into()),
                [true, false] => Ok([0., 0.]),
                [false, true] => Ok([1., 1.]),
                [true, true] => {
                    let (lo, hi) = (r[0].max(r[4]), r[1].min(r[5]));
                    let (lo2, hi2) = (r[2].max(r[4]), r[3].min(r[5]));
                    Ok([
                        ((c - hi) / (hi2 - hi)).clamp(0., 1.),
                        ((c - lo) / (lo2 - lo)).clamp(0., 1.),
                    ])
                }
            }
        };
        let left = bound(ta)?;
        let right = bound(tb)?;
        let bmin = left[0].min(right[0]);
        let bmax = left[1].max(right[1]);
        total[0][0] += N * h * (1. - bmax);
        total[0][1] += N * h * (1. - bmin);
        total[1][0] += N * h * bmin;
        total[1][1] += N * h * bmax;
    }
    Ok(total)
}
fn source(f: Fixture, i: Input) -> String {
    let old = observations::source(
        i,
        Sole {
            shape: f.shape,
            incoming: f.start,
        },
        f.shape.gravity,
    );
    let at = old.find("    <model name=\"lift\">").unwrap();
    let mut src=old[..at].replace("A rocking foot is momentarily level as the opposite edge arrives at its frictionless support.","A rigid sole spans two finite, independently translating pads.");
    for (name, center, half, v, color) in [
        ("lift", -0.13, 0.125, 0.17 - 0.18, "0.25 0.55 0.31"),
        ("toe", 0.19, 0.16, 0.17, "0.12 0.45 0.80"),
    ] {
        let size = 2. * half;
        src.push_str(&format!(r#"    <model name="{name}"><pose>{center} 0 -0.06 0 0 0</pose>
      <plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>{v} -0.09 0.22</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>
      <link name="pad"><collision name="platform"><geometry><box><size>{size} 0.26 0.12</size></box></geometry><surface><friction><ode><mu>0</mu><mu2>0</mu2></ode></friction></surface></collision><visual name="platform"><geometry><box><size>{size} 0.26 0.12</size></box></geometry><material><diffuse>{color} 1</diffuse></material></visual></link></model>
"#));
    }
    src.push_str("  </world>\n</sdf>\n");
    src
}
fn enrolled(source: &str) -> String {
    let pairs=["default::lift::pad","default::toe::pad"].map(|pad|format!("<pair><foot>default::rocker::foot</foot><support>{pad}</support><follow_frictionless_face>true</follow_frictionless_face><share_coplanar_face>true</share_coplanar_face></pair>")).join("");
    source.replace("    <model name=\"rocker\">",&format!("    <plugin name=\"gz::sim::systems::PrimaryContactRefinement\" filename=\"gz-sim-primary-contact-refinement-system\"><native_steps>8</native_steps><solver_iterations>4</solver_iterations>{pairs}</plugin>\n    <model name=\"rocker\">"))
}
fn separation(f: BodyState, p: BodyState) -> f64 {
    let q = glam::DMat3::from_quat(DQuat::from_array(f.orientation));
    let half = q.x_axis.x.abs() * 0.18 + q.y_axis.x.abs() * 0.10 + q.z_axis.x.abs() * 0.06;
    f.position[0] - half - (p.position[0] + 0.125)
}
pub(in super::super::super::super::super::super::super) fn run(
    out: &Path,
    live: bool,
) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let (i, p, _) = fixture(Case {
        active: 0,
        pitch: 0.18,
        opening: 0.3,
        yaw: 0.,
        pitch_rate: 0.6,
    });
    let f = Fixture {
        shape: p.shape,
        start: State {
            c: DVec3::new(-0.02, 0., 0.13),
            v: DVec3::new(V, 0., 0.),
            motion: Motion {
                q: DQuat::IDENTITY,
                w: DVec3::ZERO,
            },
        },
    };
    let src = source(f, i);
    let field = field()?;
    fs::write(out.join("field.json"), serde_json::to_vec(&field)?)?;
    fs::write(
        out.join("duo.sdf"),
        if live { enrolled(&src) } else { src.clone() },
    )?;
    let mut measures = vec![];
    let mut captures = serde_json::Map::new();
    let third = if live {
        ("shared", 1)
    } else {
        ("thirtytwo", 32)
    };
    for (tag, calls) in [("ordinary", 1), ("eight", 8), third] {
        let scene = if tag == "shared" {
            enrolled(&src)
        } else {
            src.clone()
        };
        let mut sim = load(&scene, true)?;
        let id = ids(&sim)?;
        let toe = bind(&sim, "default::toe::pad")?.1;
        let heel_link = bind(&sim, "default::lift::pad")?.0;
        let mut rows = vec![];
        let mut receipts = vec![];
        let mut previous = "both required";
        for tick in 0..=if live { 1340u64 } else { 1200u64 } {
            let t = tick as f64 * DT;
            let mut j = [0.; 2];
            let mut normal_vectors = [DVec3::ZERO; 2];
            let mut registered = Value::Null;
            let mut count = if tick == 0 { 0 } else { calls };
            let mut separated_ns = 0.;
            let mut measured_native_gap = 0.0f64;
            let mut limits = [[0.; 2]; 2];
            if tick > 0 {
                let t0 = (tick - 1) as f64 * DT;
                let command = (tick == 841 || tick == 1081)
                    .then(|| (COMMON + DVec3::X * heel(t0 + 1e-10).1).to_array());
                let r = if tag == "shared" {
                    if let Some(v) = command {
                        gz_sim_scene::queue_link_world_velocity(&mut sim, heel_link, v, [0.; 3])?;
                    }
                    sim.step_exact(1)?;
                    let report = last_refinement(&sim)
                        .filter(|p| p.iteration == tick)
                        .ok_or_else(|| format!("no shared Bevy receipt at {tick}"))?;
                    let samples = &report.interval.shared_faces;
                    if samples.is_empty() {
                        return Err(format!("shared face was unavailable at {tick}").into());
                    }
                    let mut selected = [0.; 2];
                    let mut enclosure = [[0.; 2]; 2];
                    let mut active = [false; 2];
                    let mut max_torque = 0f64;
                    for group in samples {
                        for load in group.loads {
                            let k = usize::from(load.support == toe);
                            selected[k] += load.selected_normal_impulse;
                            for u in 0..2 {
                                enclosure[k][u] += load.normal_impulse_bounds[u];
                            }
                            active[k] = load.contact_at_end;
                            if load.selected_normal_impulse > 0.
                                && load.end_pressure_world.is_none()
                            {
                                return Err("missing loaded pressure witness".into());
                            }
                        }
                        max_torque =
                            max_torque.max(DVec3::from(group.angular_impulse_about_com).length());
                    }
                    registered = json!({"selected_ns":selected,"enclosing_ns":enclosure,"contact_at_end":active,"spans":samples.len(),"angular_impulse_nms":max_torque});
                    report.interval.clone()
                } else {
                    let mut owner = sim.world_mut().resource_mut::<PhysicsBackend>();
                    let native = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
                    if let Some(v) = command {
                        native.set_world_velocity(id.pad, v, [0.; 3])?;
                    }
                    let mut before = separation(
                        native.state(id.foot).unwrap(),
                        native.state(id.pad).unwrap(),
                    );
                    if (775..=804).contains(&tick)
                        || (884..=900).contains(&tick)
                        || (1200..=1234).contains(&tick)
                    {
                        native.observe_contact_interval(
                            Duration::from_micros(500),
                            calls,
                            4,
                            |sub| {
                                let gap = separation(
                                    sub.state(id.foot).unwrap(),
                                    sub.state(id.pad).unwrap(),
                                );
                                let j = sub
                                    .contact_patches()
                                    .iter()
                                    .filter(|p| {
                                        [p.a, p.b].contains(&id.pad)
                                            && [p.a, p.b].contains(&id.foot)
                                    })
                                    .flat_map(|p| p.points.iter())
                                    .map(|q| q.normal_impulse)
                                    .sum::<f64>();
                                if before > 1e-10 && gap > 1e-10 && j > 0. {
                                    separated_ns += j;
                                    measured_native_gap = measured_native_gap.max(before.min(gap));
                                }
                                before = gap;
                            },
                        )?
                    } else {
                        native.step_contact_interval(Duration::from_micros(500), calls, 4)?
                    }
                };
                count = r.native_steps;
                for (index, key) in [id.pad, toe].into_iter().enumerate() {
                    normal_vectors[index] = r
                        .pairs
                        .iter()
                        .filter(|p| [p.a, p.b].contains(&key) && [p.a, p.b].contains(&id.foot))
                        .map(|p| {
                            DVec3::from(p.normal_impulse_on_a)
                                * if p.a == id.foot { 1. } else { -1. }
                        })
                        .sum();
                    j[index] = r
                        .pairs
                        .iter()
                        .filter(|pair| {
                            [pair.a, pair.b].contains(&key) && [pair.a, pair.b].contains(&id.foot)
                        })
                        .map(|p| p.normal_impulse)
                        .sum();
                }
                limits = integrated(t0, t)?;
            }
            let native = engine(&sim);
            let (foot, upper, first) = departure::ingest(native, &id);
            let upper = upper.unwrap();
            let second = native.state(toe).ok_or("missing toe pad")?;
            let shares = shares(t).ok_or("absent analytical pressure")?;
            let kind = if shares[0] >= 1. - 1e-13 {
                "heel absent"
            } else if shares[1] < 1. - 1e-13 {
                "both required"
            } else {
                "heel optional"
            };
            let transition = kind != previous;
            if tag == "ordinary" && transition {
                captures.insert(format!("{}_{}", kind.replace(' ', "_"), tick), json!(tick));
            }
            let com = (DVec3::from(foot.position) + DVec3::from(upper.position)) * 0.5;
            let com_error = com.distance(f.state(t).c + COMMON * t);
            let heel_error =
                DVec3::from(first.position).distance(COMMON * t + DVec3::new(heel(t).0, 0., -0.06));
            let toe_error =
                DVec3::from(second.position).distance(COMMON * t + DVec3::new(0.19, 0., -0.06));
            if heel_error > 1e-9 || toe_error > 1e-9 {
                return Err(
                    format!("incorrect moving pad at {t}: {heel_error} {toe_error}").into(),
                );
            }
            receipts.push(json!({"tick":tick,"impulses_ns":j,"analytical_bounds_ns":limits,"instantaneous_fraction_toe":shares,"status":kind,"com_error_m":com_error,"spin_rad_s":DVec3::from(foot.angular_velocity).length(),"registered":registered,"normal_vectors_ns":normal_vectors.map(|v|v.to_array()),"heel_x_separation_m":separation(foot,first),"positive_separated_native_ns":separated_ns,"separated_native_gap_m":measured_native_gap}));
            if tick % 4 == 0
                || tick < 8
                || transition
                || captures.values().any(|s| s.as_u64() == Some(tick))
            {
                let mut v = row(tick, count, j.iter().sum(), (foot, upper, first));
                v["opponent"] = wire(second);
                v["support_normal_ns"] = json!(j);
                // Keep every lane on the same sampled clock.
                rows.push(v);
            }
            previous = kind;
        }
        let geom = json!({"foot_half_extents":[0.18,0.10,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.125,0.13,0.06],"opponent_half_extents":[0.16,0.13,0.06]});
        fs::write(
            out.join(format!("traces/{tag}.json")),
            serde_json::to_vec(&json!({"geometry":geom,"frames":rows}))?,
        )?;
        measures.push(json!({"mode":tag,"native_calls":calls,"ticks":receipts}));
    }
    fs::write(out.join("loads.json"), serde_json::to_vec(&measures)?)?;
    let lanes = [
        ("ordinary", "One native solve"),
        ("eight", "Eight native solves"),
        if live {
            ("shared", "Registered shared pressure")
        } else {
            ("thirtytwo", "32 native solves")
        },
    ]
    .map(|(l, n)| json!({"file":format!("traces/{l}.json"),"label":l,"name":n}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"The heel support releases and returns under a load shared by two finite pads","sampled_ticks":true,"pair_totals_only":true,"capture_ticks":captures,"lanes":lanes}),
        )?,
    )?;
    Ok(())
}
