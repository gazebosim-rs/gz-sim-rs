// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::super::super::super::transaction::Frame;
use super::super::transaction::frame;
use super::*;
use gz_physics_multifidelity::FacePressureReceipt;
fn intrinsic(shape: Shape) -> DMat3 {
    (shape.inertia - DMat3::from_diagonal(DVec3::new(1., 1., 0.) * (shape.mass * ARM.z.powi(2))))
        * 0.5
}
fn effective(sole: Sole) -> Sole {
    let m = intrinsic(sole.shape);
    let k = m * DVec3::X;
    let correction = DMat3::from_cols(k * k.x, k * k.y, k * k.z) * (1. / m.x_axis.x);
    Sole {
        shape: Shape {
            inertia: sole.shape.inertia - correction,
            ..sole.shape
        },
        ..sole
    }
}
fn energy_and_momentum(b: Bodies, shape: Shape) -> (DVec3, DVec3, f64) {
    let com = 0.5 * (DVec3::from(b.0.position) + DVec3::from(b.1.unwrap().position));
    let mut p = DVec3::ZERO;
    let mut l = DVec3::ZERO;
    let mut energy = 0.;
    for link in [b.0, b.1.unwrap()] {
        let q = DQuat::from_array(link.orientation);
        let m = 0.5 * shape.mass;
        let v = DVec3::from(link.velocity);
        let w = DVec3::from(link.angular_velocity);
        let h = q * (intrinsic(shape) * (q.conjugate() * w));
        p += m * v;
        l += h + (DVec3::from(link.position) - com).cross(m * v);
        energy += 0.5 * (m * v.length_squared() + w.dot(h));
    }
    (p, l, energy)
}
fn measure(r: FacePressureReceipt, l: Law, sole: Sole, f: Frame) -> Result<Value> {
    let s = sole.incoming;
    let normal = f.q * DVec3::Z;
    let cp = l.center.ok_or("unloaded pressure")?;
    let u = sole.incoming_normal();
    let mut speed = 0f64;
    let mut geometry = 0f64;
    let mut equiv = 0f64;
    let [lo, hi] = r.equivalent_corner_loads.shift_range;
    for k in 0..4 {
        speed = speed
            .max((r.before_normal[k] - u[k]).abs())
            .max((r.after_normal[k] - l.post[k]).abs());
        geometry = geometry
            .max((DVec3::from(r.points[k]) - f.shift - f.q * sole.shape.point(k, s).0).length());
    }
    let expected = f.q
        * (s.c + s.motion.q * DVec3::new(0.18 * cp.x, 0.1 * cp.y, sole.shape.vertices[0].z))
        + f.shift;
    geometry = geometry.max(expected.distance(r.center_of_pressure.into()));
    let nerr = normal.distance(r.normal.into());
    let cerr = cp.distance(DVec2::from(r.normalized_center));
    for delta in [lo, 0.5 * (lo + hi), hi] {
        let j = r.equivalent_corner_loads.at(delta).unwrap();
        let force = j.iter().sum::<f64>();
        let moment = sole
            .arms()
            .into_iter()
            .zip(j)
            .map(|(r, j)| r.cross(j * DVec3::Z))
            .sum::<DVec3>();
        equiv = equiv.max((force - l.total).abs()).max(
            (sole.incoming.motion.w + sole.iw().inverse() * moment - l.after.motion.w).length(),
        );
        if j.iter().any(|v| *v < -1e-12) {
            return Err("nonpositive feasible allocation".into());
        }
    }
    let loss = (r.dissipated_energy - l.loss).abs();
    let net = (l.total - r.normal_impulse).abs();
    if [geometry, speed, nerr, cerr, loss, net, equiv]
        .iter()
        .any(|v| !v.is_finite() || *v > 1e-9)
    {
        return Err(format!("native face mode {:?} site {geometry:e} port {speed:e} CoP {cerr:e} work {loss:e} endpoint {equiv:e}",l.mode).into());
    }
    Ok(
        json!({"mode":format!("{:?}",l.mode),"normal_ns":r.normal_impulse,"center_normalized":r.normalized_center,"normal_error_ns":net,"cop_error":cerr,"site_error_m":geometry,"port_error_m_s":speed,"work_error_j":loss,"equivalent_error":equiv,"shift_width_ns":hi-lo,"normal":r.normal,"world_cop_m":r.center_of_pressure,"lo_loads_ns":r.equivalent_corner_loads.at(lo),"hi_loads_ns":r.equivalent_corner_loads.at(hi),"outgoing_m_s":r.after_normal}),
    )
}
fn exact(i: Input, material: Sole, f: Frame, free: bool) -> Result<Value> {
    let source = observations::source_with(i, material, material.shape.gravity, f, free, "normal");
    let oracle = if free { effective(material) } else { material };
    let law = oracle.solve()?;
    let mut s = load(&source, true)?;
    let id = ids(&s)?;
    let mut owner = s.world_mut().resource_mut::<PhysicsBackend>();
    let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
    let before = departure::ingest(n, &id);
    let r = n
        .project_normal_face_impact(id.foot, id.pad)?
        .ok_or("direct face projection missing")?;
    let after = departure::ingest(n, &id);
    if n.project_normal_face_impact(id.foot, id.pad)?.is_some()
        || after != departure::ingest(n, &id)
    {
        return Err("normal face repeated a numerical impulse".into());
    }
    let mut data = measure(r, law, oracle, f)?;
    let (p0, h0, e0) = energy_and_momentum(before, material.shape);
    let (p1, h1, e1) = energy_and_momentum(after, material.shape);
    let impulse = r.normal_impulse * DVec3::from(r.normal);
    let center = (DVec3::from(after.0.position) + DVec3::from(after.1.unwrap().position)) * 0.5;
    let dp = (p1 - p0).distance(impulse);
    let dh = (h1 - h0).distance((DVec3::from(r.center_of_pressure) - center).cross(impulse));
    let de = (e1 - e0 - r.kinetic_energy_change).abs();
    if dp.max(dh).max(de) > 1e-10 {
        return Err("assembled energy or momentum does not equal normal pressure resultant".into());
    }
    let mut sim = load(&source, true)?;
    sim.step_exact(1)?;
    let receipt = last_refinement(&sim).ok_or("no live face receipt")?;
    if receipt.interval.face_pressure_impacts.as_slice() != [r]
        || receipt.requested_pairs[0].first_positive_end != Some(Duration::ZERO)
        || !receipt.interval.edge_impacts.is_empty()
    {
        return Err("Bevy and native face measurements differ".into());
    }
    data["linear_residual_ns"] = dp.into();
    data["angular_residual_nms"] = dh.into();
    data["energy_residual_j"] = de.into();
    data["free"] = free.into();
    data["frame"] = json!(f.q.to_array());
    data["public_normal_ns"] = receipt.requested_pairs[0].normal_impulse.into();
    Ok(data)
}
pub(super) fn cases(i: Input, sole: Sole) -> Result<Value> {
    let mut rows = vec![];
    for free in [false, true] {
        for tilted in [false, true] {
            rows.push(exact(i, sole, frame(tilted), free)?);
        }
    }
    Ok(rows.into())
}
pub(super) fn record(out: &Path, i: Input, sole: Sole, law: Law) -> Result<()> {
    let mut summary = vec![];
    for tag in ["ordinary", "refined", "normal"] {
        let src = observations::source_with(i, sole, sole.shape.gravity, frame(false), false, tag);
        if tag == "normal" {
            fs::write(out.join("face.sdf"), &src)?;
        }
        let mut sim = load(&src, true)?;
        let id = ids(&sim)?;
        let mut rows = vec![];
        let mut events = vec![];
        for tick in 0..=24 {
            let (calls, reaction) = if tick > 0 {
                sim.step_exact(1)?;
                if let Some(r) = last_refinement(&sim).filter(|r| r.iteration == tick) {
                    for h in &r.interval.face_pressure_impacts {
                        let mut d = measure(*h, law, sole, frame(false))?;
                        d["tick"] = tick.into();
                        d["at_ns"] = ((r.start_time + h.elapsed).as_nanos() as u64).into();
                        d["public_normal_ns"] = r.requested_pairs[0].normal_impulse.into();
                        d["last_native_ns"] =
                            r.requested_pairs[0].last_native_normal_impulse.into();
                        events.push(d);
                    }
                    (r.interval.native_steps, r.requested_pairs[0].normal_impulse)
                } else {
                    let n = engine(&sim);
                    (
                        1,
                        n.rigid_contact_observations()
                            .into_iter()
                            .filter(|c| {
                                [c.a, c.b].contains(&id.foot) && [c.a, c.b].contains(&id.pad)
                            })
                            .map(|c| c.normal_impulse)
                            .sum(),
                    )
                }
            } else {
                (0, 0.)
            };
            let (f, u, p) = departure::ingest(engine(&sim), &id);
            rows.push(json!({"tick":tick,"time_us":tick*500,"native_steps":calls,"solver_iterations":4,"selected":calls>1,"weak_start":false,"secondary_start":false,"normal_impulse_ns":reaction,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),"gap_m":super::super::native::contact_gaps(f,p).into_iter().fold(f64::INFINITY,f64::min)}));
        }
        summary.push(
            json!({"mode":tag,"events":events,"first_public_ns":rows[1]["normal_impulse_ns"]}),
        );
        fs::write(
            out.join(format!("traces/{tag}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":rows}),
            )?,
        )?;
    }
    let lanes = [
        ("ordinary", "Ordinary native solve"),
        ("refined", "Predictive refinement"),
        ("normal", "Registered pressure resultant"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"An off-center pressure impulse with many equivalent corner allocations","pair_totals_only":true,"capture_ticks":{"pressure_impulse":1},"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("public.json"), serde_json::to_vec(&summary)?)?;
    Ok(())
}
pub(super) fn flights(out: &Path, i: Input, s: Sole) -> Result<()> {
    let mut trials = vec![];
    for t_ns in [0.25, 0.8, 62000.37, 499999.8, 2138197.6] {
        for g in [0., -9.81] {
            for tilt in [false, true] {
                for free in [false, true] {
                    let f = frame(tilt);
                    let t = t_ns * 1e-9;
                    let mut cold = s;
                    let q = DQuat::from_rotation_z(0.28);
                    cold.incoming.motion = Motion { q, w: DVec3::ZERO };
                    cold.incoming.v = DVec3::new(0.13, -0.2, -1.27);
                    cold.incoming.c = DVec3::new(
                        0.,
                        0.,
                        -cold.shape.vertices[0].z - cold.incoming.v.z * t - 0.5 * g * t * t,
                    );
                    let src = observations::source_with(i, cold, g * DVec3::Z, f, free, "normal");
                    let mut sim = load(&src, true)?;
                    let id = ids(&sim)?;
                    let steps = ((t / DT).ceil() as u64).max(1);
                    let mut early = 0.;
                    let mut seen = false;
                    for tick in 1..=steps {
                        sim.step_exact(1)?;
                        let e = last_refinement(&sim).filter(|r| r.iteration == tick);
                        if let Some((r, h)) =
                            e.and_then(|r| r.interval.face_pressure_impacts.first().map(|h| (r, h)))
                        {
                            let stamp = (r.start_time + h.elapsed).as_secs_f64();
                            if early != 0. || tick != steps || (stamp - t).abs() > 1.001e-9 {
                                return Err(format!(
                                    "fractional face at {stamp} vs {t}, early {early}"
                                )
                                .into());
                            }
                            let expected = -cold.shape.mass * (cold.incoming.v.z + g * stamp);
                            if (h.normal_impulse - expected).abs() > 1e-10
                                || DVec2::from(h.normalized_center).length() > 2e-10
                            {
                                return Err("fractional face momentum incorrect".into());
                            }
                            trials.push(json!({"target_ns":t_ns,"actual_ns":((r.start_time+h.elapsed).as_nanos()as u64),"gravity":g,"tilted":tilt,"free":free,"normal_ns":h.normal_impulse,"expected_ns":expected,"prior_load_ns":early}));
                            seen = true;
                            break;
                        }
                        early += e.map_or_else(
                            || {
                                engine(&sim)
                                    .rigid_contact_observations()
                                    .into_iter()
                                    .filter(|q| {
                                        [q.a, q.b].contains(&id.foot)
                                            && [q.a, q.b].contains(&id.pad)
                                    })
                                    .map(|q| q.normal_impulse)
                                    .sum::<f64>()
                            },
                            |r| r.requested_pairs[0].normal_impulse,
                        );
                    }
                    if !seen {
                        return Err(format!(
                            "no registered normal face in fractional arrival {t_ns}"
                        )
                        .into());
                    }
                }
            }
        }
    }
    fs::write(out.join("flights.json"), serde_json::to_vec(&trials)?)?;
    Ok(())
}
