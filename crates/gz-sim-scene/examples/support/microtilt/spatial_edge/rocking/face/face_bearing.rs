// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Spatial normal equations on the actual clipped contact polygon of a finite pad.
use super::super::transaction::frame;
#[path = "face_bearing/deck.rs"]
mod deck;
#[path = "face_bearing/dual.rs"]
pub(in super::super::super::super::super::super) mod dual;
#[path = "face_bearing/registered.rs"]
mod registered;
use super::*;
use gz_physics::{EdgeSupportState, FaceBearingAdvance, FrictionlessFaceSupport};

#[derive(Clone, Copy)]
struct Fixture {
    shape: Shape,
    start: State,
}
#[derive(Clone, Copy)]
struct Load {
    force: f64,
    center: DVec2,
    spin_acc: DVec3,
    area: f64,
    vertices: usize,
    margin: f64,
}
fn points(rect: [DVec2; 4], pad: DVec2) -> Vec<DVec2> {
    // Independent intersection by collecting corner inclusions and all edge
    // crossings, then ordering the convex hull. No half-plane clipping.
    let boundary = [
        DVec2::new(-pad.x, -pad.y),
        DVec2::new(pad.x, -pad.y),
        pad,
        DVec2::new(-pad.x, pad.y),
    ];
    let inside = |p: DVec2, q: [DVec2; 4]| {
        (0..4).all(|k| (q[(k + 1) % 4] - q[k]).perp_dot(p - q[k]) >= -1e-14)
    };
    let mut all = vec![];
    let mut push = |p: DVec2| {
        if !all.iter().any(|v: &DVec2| p.distance(*v) < 1e-11) {
            all.push(p);
        }
    };
    for p in rect {
        if inside(p, boundary) {
            push(p);
        }
    }
    for p in boundary {
        if inside(p, rect) {
            push(p);
        }
    }
    for k in 0..4 {
        let a = rect[k];
        let d = rect[(k + 1) % 4] - a;
        for j in 0..4 {
            let b = boundary[j];
            let e = boundary[(j + 1) % 4] - b;
            let cross = d.perp_dot(e);
            if cross.abs() < 1e-14 {
                continue;
            }
            let t = (b - a).perp_dot(e) / cross;
            let u = (b - a).perp_dot(d) / cross;
            if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
                push(a + t * d);
            }
        }
    }
    if !all.is_empty() {
        let center = all.iter().copied().sum::<DVec2>() / (all.len() as f64);
        all.sort_by(|a, b| {
            let da = *a - center;
            let db = *b - center;
            da.y.atan2(da.x).total_cmp(&db.y.atan2(db.x))
        });
    }
    all
}
impl Fixture {
    fn state(self, t: f64) -> State {
        let s = self.start;
        let g = self.shape.gravity.truncate().extend(0.);
        State {
            c: s.c + s.v * t + 0.5 * g * t * t,
            v: s.v + g * t,
            motion: Motion {
                q: DQuat::from_rotation_z(s.motion.w.z * t) * s.motion.q,
                w: s.motion.w,
            },
        }
    }
    fn model(self) -> FrictionlessFaceSupport {
        FrictionlessFaceSupport {
            mass: self.shape.mass,
            inertia_body: self.shape.inertia,
            corners: std::array::from_fn(|k| self.shape.vertices[k]),
            pad_axes: [DVec3::X, DVec3::Y],
            pad_half: self.shape.pad_half.into(),
            gravity: self.shape.gravity,
        }
    }
    fn coordinates(s: State) -> EdgeSupportState {
        EdgeSupportState {
            center: s.c,
            orientation: s.motion.q,
            velocity: s.v,
            angular_velocity: s.motion.w,
        }
    }
    fn pressure(self, t: f64, actual: bool) -> Result<Load> {
        let s = self.state(t);
        let n = DVec3::Z;
        let q = s.motion.q;
        let w = s.motion.w;
        let polygon = points(
            [0, 2, 3, 1].map(|k| (s.c + q * self.shape.vertices[k]).truncate()),
            self.shape.pad_half.into(),
        );
        if polygon.len() < 3 {
            return Err("zero face overlap".into());
        }
        let len = polygon.len();
        let area = (0..len)
            .map(|k| polygon[k].perp_dot(polygon[(k + 1) % len]))
            .sum::<f64>()
            * 0.5;
        let iw = DMat3::from_quat(q) * self.shape.inertia * DMat3::from_quat(q).transpose();
        let inv = iw.inverse();
        let gyro = inv * (-w.cross(iw * w));
        let m = self.shape.mass;
        let arms = polygon
            .iter()
            .map(|p| p.extend(0.) - s.c)
            .collect::<Vec<_>>();
        let torque = arms.iter().map(|r| r.cross(n)).collect::<Vec<_>>();
        let u = arms
            .iter()
            .map(|r| n.dot(self.shape.gravity + gyro.cross(*r) + w.cross(w.cross(*r))))
            .collect::<Vec<_>>();
        let mobility = |i: usize, j: usize| 1. / m + torque[i].dot(inv * torque[j]);
        let load = |rows: &[(usize, f64)]| {
            let force = rows.iter().map(|v| v.1).sum::<f64>();
            let cp = rows.iter().map(|(k, v)| polygon[*k] * (*v)).sum::<DVec2>() / force;
            let acc = gyro + inv * rows.iter().map(|(k, v)| torque[*k] * (*v)).sum::<DVec3>();
            let margin = (0..len)
                .map(|k| {
                    (polygon[(k + 1) % len] - polygon[k])
                        .normalize()
                        .perp_dot(cp - polygon[k])
                })
                .fold(f64::INFINITY, f64::min);
            Load {
                force,
                center: cp,
                spin_acc: acc,
                area,
                vertices: len,
                margin,
            }
        };
        let mut best = [0; 3];
        let mut twice_area = 0.;
        for i in 0..len {
            for j in i + 1..len {
                for k in j + 1..len {
                    let a = (polygon[j] - polygon[i]).perp_dot(polygon[k] - polygon[i]);
                    if a > twice_area {
                        twice_area = a;
                        best = [i, j, k];
                    }
                }
            }
        }
        let wmat = DMat3::from_cols_array_2d(&best.map(|j| best.map(|i| mobility(i, j))));
        let lambda = -(wmat.inverse() * DVec3::from_array(best.map(|i| u[i])));
        let full = load(&std::array::from_fn::<_, 3, _>(|i| (best[i], lambda[i])));
        if !actual || full.margin >= -1e-10 {
            return Ok(full);
        }
        let admissible = |rows: &[(usize, f64)]| {
            rows.iter().all(|x| x.1 >= -1e-10)
                && (0..len).all(|k| {
                    u[k] + rows
                        .iter()
                        .map(|(i, v)| mobility(k, *i) * (*v))
                        .sum::<f64>()
                        >= -1e-8
                })
        };
        for i in 0..len {
            let j = (i + 1) % len;
            let a = mobility(i, i);
            let b = mobility(i, j);
            let c = mobility(j, j);
            let det = a * c - b * b;
            if det > 1e-12 {
                let rows = [
                    (i, (b * u[j] - c * u[i]) / det),
                    (j, (b * u[i] - a * u[j]) / det),
                ];
                if admissible(&rows) {
                    return Ok(load(&rows));
                }
            }
            let rows = [(i, -u[i] / a)];
            if admissible(&rows) {
                return Ok(load(&rows));
            }
        }
        Err("no unilateral normal solution for finite polygon".into())
    }
    fn measured(self, horizon: f64) -> Result<Value> {
        let law = self.model();
        let begin = self.pressure(0., false)?;
        let bound = law.bind(Self::coordinates(self.start))?;
        if bound.is_none() && begin.margin > 1e-9 {
            return Err("missed loaded contact polygon".into());
        }
        if let Some(b) = bound {
            let mut worst = [0f64; 4];
            let mut below = 0.;
            let mut crossing = None;
            for k in 0..=32 {
                let t = horizon * f64::from(k) / 32.;
                let p = self.pressure(t, false)?;
                let v = b.sample(t)?;
                worst[0] = worst[0].max((v.normal_force - p.force).abs());
                worst[1] = worst[1].max(v.pad_center.distance(p.center));
                worst[2] = worst[2].max((v.overlap.area - p.area).abs());
                worst[3] = worst[3].max(p.spin_acc.length());
                if crossing.is_none() && p.margin <= 0. {
                    for _ in 0..50 {
                        let hi = crossing.unwrap_or(t);
                        let mid = (below + hi) * 0.5;
                        if self.pressure(mid, false)?.margin > 0. {
                            below = mid;
                        } else {
                            crossing = Some(mid);
                        }
                    }
                    crossing = Some(0.5 * (below + crossing.unwrap_or(t)));
                } else if crossing.is_none() {
                    below = t;
                }
            }
            if worst.iter().any(|e| !e.is_finite() || *e > 1e-8) {
                return Err(format!("inconsistent finite pressure {worst:?}").into());
            }
            let (exit, mode, edge) = match b.advance(horizon)? {
                FaceBearingAdvance::Supported(_) => (None, "supported", 0),
                FaceBearingAdvance::Exits(e) => (Some(e.seconds), "pressure exit", e.pad_edges),
                FaceBearingAdvance::Unresolved => (None, "unresolved", 0),
            };
            if crossing.is_some() != exit.is_some()
                || crossing
                    .zip(exit)
                    .is_some_and(|(a, b)| (a - b).abs() > 2e-10)
            {
                return Err(
                    format!("independent finite exit {crossing:?} vs {mode} {exit:?}").into(),
                );
            }
            let release = if let Some(t) = exit {
                let p = self.pressure(t + 1e-5, true)?;
                json!({"force_n":p.force,"angular_acceleration_rad_s2":p.spin_acc.to_array(),"active_cp_m":p.center.to_array()})
            } else {
                Value::Null
            };
            return Ok(
                json!({"mode":mode,"exit_s":exit,"physical_exit_s":crossing,"pad_edges":edge,"initial_pressure_margin_m":begin.margin,"initial_overlap_fraction":begin.area/(0.36*0.2),"initial_vertices":begin.vertices,"errors_force_cp_area_alpha":worst,"released":release,"force_n":begin.force}),
            );
        }
        Ok(
            json!({"mode":"unavailable","initial_pressure_margin_m":begin.margin,"initial_overlap_fraction":begin.area/(0.36*0.2)}),
        )
    }
}
fn write_trace(out: &Path, name: &str, rows: Vec<Value>) -> Result<()> {
    fs::write(
        out.join(format!("traces/{name}.json")),
        serde_json::to_vec(
            &json!({"geometry":{"foot_half_extents":[0.18,0.10,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":rows}),
        )?,
    )?;
    Ok(())
}
fn row(tick: u64, calls: u32, normal: f64, (f, u, p): (BodyState, BodyState, BodyState)) -> Value {
    json!({"tick":tick,"time_us":tick*500,"native_steps":calls,"solver_iterations":if calls==0{0}else{4},"selected":calls>1,"weak_start":false,"secondary_start":false,"normal_impulse_ns":normal,"foot":wire(f),"upper":wire(u),"pad":wire(p),"gap_m":super::super::native::contact_gaps(f,p).into_iter().fold(f64::INFINITY,f64::min)})
}
fn actual(out: &Path, i: Input, fx: Fixture, ticks: u64) -> Result<Value> {
    let src = observations::source_with(
        i,
        Sole {
            shape: fx.shape,
            incoming: fx.start,
        },
        fx.shape.gravity,
        frame(false),
        false,
        "ordinary",
    );
    fs::write(out.join("face.sdf"), &src)?;
    let mut records = vec![];
    for (tag, calls) in [("ordinary", 1), ("refined", 8)] {
        let mut sim = load(&src, true)?;
        let id = ids(&sim)?;
        let mut handle = sim.world_mut().resource_mut::<PhysicsBackend>();
        let e = handle.downcast_mut::<MultiFidelityBackend>().unwrap();
        let mut rows = vec![];
        let mut max_j = 0f64;
        let mut max_roll = 0f64;
        let mut total = 0.;
        for tick in 0..=ticks {
            let normal = if tick == 0 {
                0.
            } else {
                let r = e.step_contact_interval(Duration::from_micros(500), calls, 4)?;
                r.pairs
                    .iter()
                    .filter(|p| [p.a, p.b].contains(&id.foot) && [p.a, p.b].contains(&id.pad))
                    .map(|p| p.normal_impulse)
                    .sum::<f64>()
            };
            let (f, u, p) = departure::ingest(e, &id);
            if tick > 0 {
                max_j = max_j.max((normal + fx.shape.mass * fx.shape.gravity.z * DT).abs());
                max_roll = max_roll.max(DVec3::from(f.angular_velocity).truncate().length());
                total += normal;
            }
            rows.push(row(
                tick,
                if tick == 0 { 0 } else { calls },
                normal,
                (f, u.unwrap(), p),
            ));
        }
        records.push(json!({"mode":tag,"maximum_deviation_from_static_weight_ns":max_j,"total_ns":total,"maximum_roll_pitch_rate_rad_s":max_roll}));
        write_trace(out, tag, rows)?;
    }
    Ok(records.into())
}
fn reference(out: &Path, i: Input, fx: Fixture, ticks: u64) -> Result<()> {
    let bound = fx
        .model()
        .bind(Fixture::coordinates(fx.start))?
        .ok_or("missing initially supported face")?;
    let mut rows = vec![];
    for tick in 0..=ticks {
        let t = tick as f64 * DT;
        let x = bound.sample(t)?;
        rows.push(row(
            tick,
            0,
            if tick == 0 { 0. } else { x.normal_force * DT },
            super::super::replay::bodies(i, t, fx.state(t)),
        ));
    }
    write_trace(out, "pressure", rows)?;
    Ok(())
}
pub(in super::super::super::super::super::super) fn run(
    out: &Path,
    live: bool,
    tip: bool,
) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let (i, p, _) = fixture(Case {
        active: 0,
        pitch: 0.18,
        yaw: 0.,
        opening: 0.3,
        pitch_rate: 0.6,
    });
    let mut shape = p.shape;
    shape.inertia = DMat3::from_cols(
        DVec3::new(0.042, 0.002, -0.008),
        DVec3::new(0.002, 0.053, 0.006),
        DVec3::new(-0.008, 0.006, 0.059),
    );
    let mut data = vec![];
    let mut demo = None;
    let mut tips = vec![];
    let mut tip_demo = None;
    for yaw in [0., 0.36, 0.71] {
        for spin in [-5., 0., 3., 6.] {
            for x in [0.22, 0.32, 0.397, 0.425] {
                for direction in [-1., 1.] {
                    for vy in [-0.17, 0.07] {
                        let f = Fixture {
                            shape,
                            start: State {
                                c: DVec3::new(x * direction, 0.18, -shape.vertices[0].z),
                                v: DVec3::new(0.82 * direction, vy, 0.),
                                motion: Motion {
                                    q: DQuat::from_rotation_z(yaw),
                                    w: spin * DVec3::Z,
                                },
                            },
                        };
                        let mut row = f.measured(0.1)?;
                        if tip && let Some(c) = deck::case(f, &row) {
                            let mut study = c.compare()?;
                            if live {
                                let mut runs = Vec::new();
                                for (index, frac) in [0.25e-9, 0.000213819760, 0.00049999975]
                                    .into_iter()
                                    .enumerate()
                                {
                                    runs.push(deck::registered(
                                        i,
                                        c,
                                        &study,
                                        frac,
                                        index == 1,
                                        None,
                                    )?);
                                }
                                study["registered"] = runs.into();
                            }
                            if tip_demo.is_none()
                                && yaw > 0.
                                && spin != 0.
                                && study["max_tilt_rad"].as_f64().unwrap_or(0.) > 0.005
                            {
                                tip_demo = Some((c, study.clone()));
                            }
                            tips.push(study);
                        }
                        if live && !tip {
                            row["registered"] = json!([
                                registered::actual(i, f, &row, 0.1, false, true, false)?.0,
                                registered::actual(i, f, &row, 0.1, true, false, false)?.0
                            ]);
                        }
                        if demo.is_none()
                            && yaw == 0.71
                            && spin == 6.
                            && x == 0.32
                            && direction > 0.
                            && row["mode"] == "pressure exit"
                        {
                            demo = Some(f);
                        }
                        data.push(json!({"yaw":yaw,"spin":spin,"initial_x":x,"direction":direction,"velocity_y":vy,"result":row}));
                    }
                }
            }
        }
    }
    for spin in [-30., -15., 15., 30.] {
        for yaw in [1.35, 1.8, 2.6] {
            let mut orbit_shape = shape;
            let cross = -0.055 * (shape.mass * (-shape.gravity.z)) / (spin * spin);
            orbit_shape.inertia.z_axis.x = cross;
            orbit_shape.inertia.x_axis.z = cross;
            orbit_shape.inertia.z_axis.y = 0.;
            orbit_shape.inertia.y_axis.z = 0.;
            let f = Fixture {
                shape: orbit_shape,
                start: State {
                    c: DVec3::new(0.393, 0., 0.13),
                    v: DVec3::ZERO,
                    motion: Motion {
                        q: DQuat::from_rotation_z(yaw),
                        w: spin * DVec3::Z,
                    },
                },
            };
            let horizon = std::f64::consts::TAU / f64::abs(spin);
            let mut row = f.measured(horizon)?;
            if live && !tip {
                row["registered"] = json!([
                    registered::actual(i, f, &row, horizon, false, true, false)?.0,
                    registered::actual(i, f, &row, horizon, true, false, false)?.0
                ]);
            }
            if row["mode"] != "pressure exit" {
                return Err("hidden pressure excursion unresolved".into());
            }
            data.push(json!({"orbit":true,"yaw":yaw,"spin":spin,"horizon_s":horizon,"result":row}));
        }
    }
    if tip {
        fs::write(out.join("tips.json"), serde_json::to_vec(&tips)?)?;
        let (c, data) = tip_demo.ok_or("missing tipping demo")?;
        if live {
            deck::registered(i, c, &data, 0.000213819760, false, Some(out))?;
            return Ok(());
        }
        return deck::replay(out, i, c, &data);
    }
    fs::write(out.join("field.json"), serde_json::to_vec(&data)?)?;
    let f = demo.ok_or("missing partially supported yawing trajectory")?;
    let e = f.measured(0.1)?;
    if live {
        return registered::replay(out, i, f, &e);
    }
    let t = e["exit_s"].as_f64().unwrap();
    let ticks = (t / DT).floor() as u64;
    let n = actual(out, i, f, ticks)?;
    reference(out, i, f, ticks)?;
    fs::write(
        out.join("public.json"),
        serde_json::to_vec(&json!({"ticks":ticks,"physical":e,"natives":n}))?,
    )?;
    let lanes=[("ordinary","One native solve"),("refined","Eight native solves"),("pressure","Independently supported yawing face")].map(|(l,n)|json!({"label":l,"name":n,"file":format!("traces/{l}.json"),"analytical":l=="pressure"}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"A yawing sole approaches the outer limit of a finite support","pair_totals_only":true,"capture_ticks":{"last_supported":ticks},"lanes":lanes}),
        )?,
    )?;
    Ok(())
}
