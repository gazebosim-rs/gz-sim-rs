// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare the cyclic-coordinate solution of two-site bearing with Cartesian
//! Newton–Euler integration and the actual locked native foot.
use super::*;
#[path="two_bearing/registered.rs"]
pub(in super::super::super::super::super) mod registered;
use gz_physics::{
    BoundFrictionlessEdge, EdgeSupportSample, EdgeSupportState, FrictionlessEdgeSupport,
};
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Limit {
    Horizon,
    Loss(usize),
    FinitePad,
    Vertices(u16),
}
fn key(l: Limit) -> String {
    match l {
        Limit::Horizon => "horizon".into(),
        Limit::FinitePad => "finite pad".into(),
        Limit::Loss(i) => format!("unload {i}"),
        Limit::Vertices(bits) => format!("touch {bits:04x}"),
    }
}
fn state(s: EdgeSupportSample) -> State {
    State {
        c: s.state.center,
        v: s.state.velocity,
        motion: Motion {
            q: s.state.orientation,
            w: s.state.angular_velocity,
        },
    }
}
fn bound(a: Arc) -> Result<Option<BoundFrictionlessEdge>> {
    let Sites::Two(v) = a.sites else {
        return Err("requires two bearing sites".into());
    };
    let b = FrictionlessEdgeSupport {
        mass: a.shape.mass,
        inertia_body: a.shape.inertia,
        sites_body: v.map(|i| a.shape.vertices[i]),
        normal: DVec3::Z,
        plane_offset: 0.,
        gravity: a.shape.gravity,
    };
    let s = a.start;
    Ok(b.bind(EdgeSupportState {
        center: s.c,
        orientation: s.motion.q,
        velocity: s.v,
        angular_velocity: s.motion.w,
    })?)
}
fn margins(a: Arc, s: State, forces: [f64; 2]) -> [f64; 19] {
    let Sites::Two(active) = a.sites else {
        unreachable!()
    };
    let mut v = [f64::INFINITY; 19];
    v[0] = forces[0];
    v[1] = forces[1];
    for (i, r) in a.shape.vertices.into_iter().enumerate() {
        let p = s.c + s.motion.q * r;
        v[2] = v[2]
            .min(a.shape.pad_half[0] - p.x.abs() - 1e-8)
            .min(a.shape.pad_half[1] - p.y.abs() - 1e-8);
        if !active.contains(&i) {
            v[3 + i] = p.z;
        }
    }
    v
}
fn classify(a: Arc, t: State, loads: [f64; 2], horizon: bool) -> Result<Limit> {
    if horizon {
        return Ok(Limit::Horizon);
    }
    let Sites::Two(ids) = a.sites else {
        unreachable!()
    };
    let m = margins(a, t, loads);
    let mut out = vec![];
    for (j, k) in ids.into_iter().enumerate() {
        if m[j].abs() < 1e-7 {
            out.push(Limit::Loss(k));
        }
    }
    if m[2].abs() < 1e-8 {
        out.push(Limit::FinitePad);
    }
    let mask = m[3..]
        .iter()
        .enumerate()
        .filter(|(id, g)| g.abs() < 2e-9 && a.shape.point(*id, t).1.z < -1e-7)
        .fold(0u16, |m, (i, _)| m | 1 << i);
    if mask > 0 {
        out.push(Limit::Vertices(mask));
    }
    if let [one] = out.as_slice() {
        Ok(*one)
    } else {
        Err(format!("unresolved co-bearing mode transition: {out:?}").into())
    }
}
fn end(
    a: Arc,
    c: BoundFrictionlessEdge,
    h: f64,
    horizon: f64,
) -> Result<(f64, Limit, EdgeSupportSample)> {
    let mut cursor = c.trajectory();
    let mut t = 0.;
    let mut data = c.sample(0., h)?;
    while t < horizon {
        let dt = (horizon - t).min(h);
        let start = cursor;
        data = cursor.advance(dt, h)?;
        let m = margins(a, state(data), data.signed_force);
        if m.into_iter().any(|v| v <= 0.) {
            let mut lo = 0.;
            let mut hi = dt;
            for _ in 0..37 {
                let mid = 0.5 * (lo + hi);
                let v = {
                    let mut candidate = start;
                    candidate.advance(mid, h)?
                };
                if margins(a, state(v), v.signed_force)
                    .iter()
                    .any(|n| *n <= 0.)
                {
                    hi = mid
                } else {
                    lo = mid
                }
            }
            let event = {
                let mut candidate = start;
                candidate.advance(hi, h)?
            };
            return Ok((
                t + hi,
                classify(a, state(event), event.signed_force, false)?,
                event,
            ));
        }
        t += dt;
    }
    Ok((t, Limit::Horizon, data))
}
fn compare(a: Arc, known: BoundFrictionlessEdge) -> Result<(Value, Limit, f64)> {
    let horizon = 0.34;
    let one = end(a, known, 0.00025, horizon)?;
    let fine = end(a, known, 0.0000625, horizon)?;
    let spatial = a.next(0.000125, horizon)?;
    let reference = classify(
        a,
        spatial.state,
        a.load(spatial.state.motion)?,
        matches!(spatial.end, End::Horizon),
    )?;
    if reference != fine.1
        || one.1 != fine.1
        || (one.0 - fine.0).abs() > 3e-8
        || (spatial.time - fine.0).abs() > 3e-8
    {
        return Err(format!(
            "two-bearing transition {:?}/{:?}/{:?} at {:.9}/{:.9}/{:.9}",
            one.1, fine.1, reference, one.0, fine.0, spatial.time
        )
        .into());
    }
    let mut moments = vec![];
    let origin = known.sample(0., 1.)?;
    for u in [0., 0.25, 0.5, 0.75, 1.] {
        let t = fine.0 * u;
        let p = known.sample(t, 0.000125)?;
        let s = a.at(t, 0.000125)?;
        let forces = a.load(s.motion)?;
        let q = p.state.orientation;
        let qerr = 4. * ((q - s.motion.q).length().min((q + s.motion.q).length()) * 0.5).asin();
        let force = (0..2)
            .map(|j| (forces[j] - p.signed_force[j]).abs())
            .fold(0., f64::max);
        let normal = a.shape.mass * (p.state.velocity - a.start.v - a.shape.gravity * t);
        let residual = (normal - DVec3::Z * p.signed_impulse.iter().sum::<f64>()).length();
        let er = (p.mechanical_energy - origin.mechanical_energy).abs();
        if qerr
            .max(p.state.angular_velocity.distance(s.motion.w))
            .max(force)
            .max(residual)
            .max(er)
            > 5e-8
        {
            return Err(format!("constrained trajectory differs from spatial reference at {t:.8}: q {qerr:e} force {force:e} j {residual:e} energy {er:e}").into());
        }
        moments.push(json!({"time_s":t,"cyclic":state_wire(state(p)),"spatial":state_wire(s),"cyclic_force_n":p.signed_force,"reference_force_n":forces,"signed_impulse_ns":p.signed_impulse,"energy_delta_j":er,"momentum_residual_ns":residual,"q_error_rad":qerr,"w_error_rad_s":p.state.angular_velocity.distance(s.motion.w),"yaw_momentum":p.yaw_momentum}));
    }
    Ok((
        json!({"time_s":fine.0,"coarse_s":one.0,"spatial_s":spatial.time,"limit":key(fine.1),"samples":moments}),
        fine.1,
        fine.0,
    ))
}
fn varied(mut a: Arc, kind: &str) -> Arc {
    let Sites::Two(v) = a.sites else {
        unreachable!()
    };
    if kind == "natural" {
        return a;
    }
    let q = a.start.motion.q;
    let d = (q * (a.shape.vertices[v[1]] - a.shape.vertices[v[0]])).normalize();
    let (spin, roll, shift) = match kind {
        "triaxial" => (a.start.motion.w.z, a.start.motion.w.dot(d), DVec3::ZERO),
        "yawing" => (7., 1.1, DVec3::new(0.05, 0., 0.01)),
        "reversed" => (-4.5, -1.3, DVec3::new(-0.055, 0.01, -0.035)),
        "unload" => (12., -3.2, DVec3::new(0.088, -0.022, -0.02)),
        _ => unreachable!(),
    };
    a.shape.inertia = DMat3::from_cols(
        DVec3::new(0.034, 0.004, -0.003),
        DVec3::new(0.004, 0.051, 0.006),
        DVec3::new(-0.003, 0.006, 0.067),
    );
    a.shape.vertices = a.shape.vertices.map(|p| p - shift);
    a.start.c += q * shift;
    let w = d * roll + DVec3::Z * spin;
    a.start.motion.w = w;
    a.start.v.z = -w.cross(q * a.shape.vertices[v[0]]).z;
    a
}
/// Search predeclared tilted, spinning, eccentric fixtures. Record the
/// entire census, then re-solve each observed unloading event independently.
fn unload_field(out: &Path, base: Arc) -> Result<()> {
    let Sites::Two(keys) = base.sites else {
        unreachable!()
    };
    let mut census = vec![];
    let mut qualified = vec![];
    for shift in [-0.105, -0.045, 0.045, 0.105] {
        for roll in [-0.7, -0.36, 0.0, 0.36, 0.7] {
            for spin in [-15., -7., 0., 7., 15.] {
                for rate in [-8., -2., 0., 2., 8.] {
                    let mut a = varied(base, "triaxial");
                    let old = a.start.motion.q;
                    let local = (a.shape.vertices[keys[1]] - a.shape.vertices[keys[0]]).normalize();
                    let d = old * local;
                    let q = old * DQuat::from_axis_angle(local, roll);
                    let w = spin * DVec3::Z + rate * d;
                    a.shape.vertices = a.shape.vertices.map(|r| r - shift * local);
                    a.shape.pad_half = [0.75; 2];
                    a.start.motion = Motion { q, w };
                    let arm = q * a.shape.vertices[keys[0]];
                    a.start.c = DVec3::Z * (-arm.z);
                    a.start.v = DVec3::new(0.02, 0.01, -w.cross(arm).z);
                    let entry = |label: String| json!({"shift":shift,"roll_offset":roll,"spin":spin,"rate":rate,"result":label});
                    if margins(a, a.start, [1.; 2])[3..].iter().any(|v| *v < 1e-7) {
                        census.push(entry("outside starting geometry".into()));
                        continue;
                    }
                    let Some(c) = bound(a)? else {
                        census.push(entry("no initial compression".into()));
                        continue;
                    };
                    let (t, l, _) = end(a, c, 0.0005, 0.34)?;
                    census.push(entry(key(l)));
                    if let Limit::Loss(site) = l {
                        let (r, actual, when) = compare(a, c)?;
                        if actual != l || (when - t).abs() > 2e-7 {
                            return Err("independent unloading census disagrees".into());
                        }
                        qualified.push(json!({"shift":shift,"roll_offset":roll,"spin":spin,"rate":rate,"unloaded_vertex":site,"sites":keys,"inertia_body":a.shape.inertia.to_cols_array(),"result":r}));
                    }
                }
            }
        }
    }
    fs::write(
        out.join("unload-field.json"),
        serde_json::to_vec(&json!({"census":census,"releases":qualified}))?,
    )?;
    Ok(())
}
fn output(
    out: &Path,
    input: Input,
    a: Arc,
    law: BoundFrictionlessEdge,
    limit: Limit,
    t: f64,
) -> Result<()> {
    let ticks = ((t / DT).floor() as u64).min(210);
    if ticks < 6 {
        return Err("demo span too short".into());
    }
    let mut meta = vec![];
    for (label, calls) in [("ordinary", 1), ("refined", 8), ("cyclic", 0)] {
        let rows = if calls > 0 {
            native::native(input, a, calls, ticks, true, None)?.1
        } else {
            let mut cur = law.trajectory();
            let mut series = vec![];
            let mut prev = [0.; 2];
            let Sites::Two(keys) = a.sites else {
                unreachable!()
            };
            for tick in 0..=ticks {
                let s = cur.advance(if tick > 0 { DT } else { 0. }, 0.0000625)?;
                let (f, u, p) = replay::bodies(input, tick as f64 * DT, state(s));
                let mut loads = [0.; 4];
                for (k, id) in keys.into_iter().enumerate() {
                    loads[id] = s.signed_impulse[k] - prev[k];
                }
                prev = s.signed_impulse;
                series.push(json!({"tick":tick,"time_us":tick*500,"native_steps":0,"reference_steps":if tick>0{8}else{0},"solver_iterations":0,"selected":false,"weak_start":false,"secondary_start":false,"normal_impulse_ns":loads.iter().sum::<f64>(),"corner_ns":loads,"foot":wire(f),"upper":wire(u),"pad":wire(p),"gap_m":native::contact_gaps(f,p).into_iter().fold(f64::INFINITY,f64::min)}));
            }
            series
        };
        fs::write(
            out.join(format!("traces/{label}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":rows}),
            )?,
        )?;
        meta.push(json!({"lane":label,"last_tick":ticks,"last_relative":rows.last(),"initial_relative":rows.first()}));
    }
    let names=[("ordinary","One native call"),("refined","Eight native calls"),("cyclic","Constrained two-bearing trajectory")].map(|(l,n)|json!({"label":l,"name":n,"file":format!("traces/{l}.json"),"analytical":l=="cyclic"}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"Two vertices bear as the foot rolls and yaws","capture_ticks":{"last_co_bearing":ticks},"lanes":names}),
        )?,
    )?;
    let seed = Seed {
        state: PoseState {
            q: a.start.motion.q,
            c: a.start.c,
            v: a.start.v,
            w: a.start.motion.w,
        },
        ..input.seed
    };
    fs::write(
        out.join("two-bearing.sdf"),
        scene(seed, 0., false, "ordinary"),
    )?;
    fs::write(
        out.join("public.json"),
        serde_json::to_vec(&json!({"end":key(limit),"physical_end_s":t,"lanes":meta}))?,
    )?;
    Ok(())
}
pub(in super::super::super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    let mut demo = None;
    for active in [0, 1] {
        for pitch in [-0.18, 0.18] {
            for opening in [0.09, 0.3, 0.5] {
                for rate in [-3., -0.6, 0.6, 3.] {
                    for yaw in [0., 0.67] {
                        let (i, path, _) = fixture(Case {
                            active,
                            pitch,
                            yaw,
                            opening,
                            pitch_rate: rate,
                        });
                        if solve(i.edge).is_err() {
                            continue;
                        }
                        let e = path.next(0.000125, 0.4)?;
                        let End::Contact(new) = e.end else { continue };
                        let j = path.shape.recollide(e.state, path.sites.first(), new)?;
                        let Sites::Two(keys) = j.new_sites else {
                            continue;
                        };
                        for variant in ["natural", "triaxial", "yawing", "reversed", "unload"] {
                            let a = varied(
                                Arc {
                                    shape: path.shape,
                                    start: j.after,
                                    sites: j.new_sites,
                                },
                                variant,
                            );
                            let solver = bound(a)?;
                            let r = if let Some(s) = solver {
                                let (result, lim, t) = compare(a, s)?;
                                if demo.is_none()
                                    && variant == "natural"
                                    && yaw > 0.
                                    && t >= 0.012
                                    && t < 0.102
                                    && matches!(lim, Limit::Vertices(_))
                                {
                                    demo = Some((i, a, s, lim, t));
                                }
                                Some(result)
                            } else {
                                None
                            };
                            rows.push(json!({"incoming_active":active,"incoming_pitch":pitch,"opening":opening,"rate":rate,"yaw":yaw,"variant":variant,"sites":keys,"inertia_body":a.shape.inertia.to_cols_array(),"state":state_wire(a.start),"result":r}));
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("co-bearing.json"), serde_json::to_vec(&rows)?)?;
    let (i, a, s, lim, t) = demo.ok_or("no representative natural two-bearing return")?;
    output(out, i, a, s, lim, t)?;
    unload_field(out, a)
}
