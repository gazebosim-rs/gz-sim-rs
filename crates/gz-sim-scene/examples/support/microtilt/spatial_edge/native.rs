// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a single native generalized commit with the independent assembly mechanics.
use super::*;
fn vector(v: DVec3) -> String {
    format!("{:.17} {:.17} {:.17}", v.x, v.y, v.z)
}
fn pose(p: DVec3, q: DQuat) -> String {
    let (yaw, pitch, roll) = q.to_euler(glam::EulerRot::ZYX);
    format!("{} {roll:.17} {pitch:.17} {yaw:.17}", vector(p))
}
fn replace_value(xml: &mut String, after: &str, key: &str, value: &str) {
    let open = format!("<{key}>");
    let close = format!("</{key}>");
    let at = xml.find(after).unwrap();
    let a = at + xml[at..].find(&open).unwrap() + open.len();
    let b = a + xml[a..].find(&close).unwrap();
    xml.replace_range(a..b, value);
}
#[derive(Clone, Copy)]
pub(super) struct Frame {
    pub q: DQuat,
    pub shift: DVec3,
    pub spin: f64,
}
pub(super) fn fixture(i: Input, frame: Frame, free: bool) -> String {
    let mut s=scene(i.seed,i.edge.friction[0],false,"ordinary").replace("<!-- A released locked 2 kg foot meets a rising finite support. The settling driver brakes the support to rest. -->","<!-- A translating deck meets two vertices of a tilted sliding sole at the initial boundary. -->");
    if free {
        s = s.replace(
            "<exact_revolute_locks>true</exact_revolute_locks>",
            "<exact_revolute_locks>false</exact_revolute_locks>",
        );
        s = s.replace(
            "<lower>0</lower><upper>0</upper>",
            "<lower>-2</lower><upper>2</upper>",
        );
    }
    let moved = |p: DVec3| frame.shift + frame.q * p;
    let deck = DVec3::from(i.seed.pad.position);
    let c = deck + PAD + i.seed.state.c;
    let foot = c - i.seed.state.q * ARM;
    let spin = frame.spin * DVec3::Z;
    let vel = i.seed.state.v + DVec3::from(i.seed.pad.velocity) + spin.cross(c - deck);
    replace_value(
        &mut s,
        "<model name=\"rocker\">",
        "pose",
        &pose(moved(foot), frame.q * i.seed.state.q),
    );
    replace_value(
        &mut s,
        "<model name=\"rocker\">",
        "linear_velocity",
        &vector(frame.q * vel),
    );
    replace_value(
        &mut s,
        "<model name=\"rocker\">",
        "angular_velocity",
        &vector(frame.q * (i.seed.state.w + spin)),
    );
    replace_value(
        &mut s,
        "<model name=\"lift\">",
        "pose",
        &pose(moved(deck), frame.q),
    );
    replace_value(
        &mut s,
        "<model name=\"lift\">",
        "linear_velocity",
        &vector(frame.q * DVec3::from(i.seed.pad.velocity)),
    );
    replace_value(
        &mut s,
        "<model name=\"lift\">",
        "angular_velocity",
        &vector(frame.q * spin),
    );
    s
}
pub(super) fn inertial(b: Bodies) -> (DVec3, DVec3, f64) {
    let links = [b.0, b.1.unwrap()];
    let center = (DVec3::from(links[0].position) + DVec3::from(links[1].position)) * 0.5;
    let mut p = DVec3::ZERO;
    let mut h = DVec3::ZERO;
    let mut energy = 0.;
    for l in links {
        let q = DQuat::from_array(l.orientation);
        let v = DVec3::from(l.velocity);
        let w = DVec3::from(l.angular_velocity);
        let inertia_w = q * (DVec3::new(0.0201, 0.0201, 0.025) * (q.conjugate() * w));
        p += v;
        h += inertia_w + (DVec3::from(l.position) - center).cross(v);
        energy += 0.5 * (v.length_squared() + w.dot(inertia_w));
    }
    (p, h, energy)
}
fn measure(i: Input, frame: Frame, free: bool) -> Result<Value> {
    let mut e = i.edge;
    if free {
        // The free child hinges through its center. Along its x hinge the
        // child's intrinsic axial inertia transmits no joint torque; the
        // remainder and both orbital terms stay in the root's point mobility.
        let axis = i.seed.state.q * DVec3::X;
        e.inertia -= DMat3::from_cols(axis * axis.x, axis * axis.y, axis * axis.z) * 0.0201;
    }
    let reference = e.solve();
    let mut s = load(&fixture(i, frame, free), true)?;
    let ids = ids(&s)?;
    let mut owner = s.world_mut().resource_mut::<PhysicsBackend>();
    let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
    let before = departure::ingest(n, &ids);
    let (p0, h0, k0) = inertial(before);
    let applied = n.project_sliding_edge_impact(ids.foot, ids.pad)?;
    let after = departure::ingest(n, &ids);
    let Some(a) = applied else {
        if before != after {
            return Err("declined edge operation mutated native state".into());
        }
        if reference.is_ok() {
            return Err(format!(
                "native declined certified edge µ={} free={free} spin={}",
                e.friction[0], frame.spin
            )
            .into());
        }
        return Ok(
            json!({"admitted":false,"reference_decline":format!("{:?}",reference.err().unwrap())}),
        );
    };
    let law = reference.map_err(|why| format!("native accepted oracle decline: {why:?}"))?;
    let (p1, h1, k1) = inertial(after);
    let foot = before.0;
    let center = (DVec3::from(foot.position) + DVec3::from(before.1.unwrap().position)) * 0.5;
    let mut impulses = [DVec3::ZERO; 2];
    let mut recorded = vec![];
    let mut errors = [0f64; 2];
    let mut dh = DVec3::ZERO;
    let mut sum_work = 0.;
    let mut sum_loss = 0.;
    let mut seen = [false; 2];
    for site in a.sites {
        let point = DVec3::from(site.point);
        let local = frame.q.conjugate() * (point - center);
        let idx = usize::from(local.distance(e.arm[1]) < local.distance(e.arm[0]));
        if seen[idx] || local.distance(e.arm[idx]) > 1e-12 {
            return Err("incorrect or duplicated native edge site".into());
        }
        seen[idx] = true;
        let j = DVec3::from(site.impulse_on_foot);
        impulses[idx] = j;
        errors[idx] = j.distance(frame.q * law.impulse[idx]);
        dh += (point - center).cross(j);
        if DVec3::from(site.before_relative_velocity).distance(frame.q * e.point(idx)) > 1e-11
            || DVec3::from(site.after_relative_velocity).distance(frame.q * law.point_velocity[idx])
                > 1e-11
        {
            return Err("native generalized point velocity differs from assembled edge".into());
        }
        sum_work += site.kinetic_energy_change;
        sum_loss += site.dissipated_energy;
        recorded.push(json!({"site":idx,"world_point_m":site.point,"world_normal":site.normal,"impulse_ns":site.impulse_on_foot,"incoming_m_s":site.before_relative_velocity,"outgoing_m_s":site.after_relative_velocity,"energy_j":site.kinetic_energy_change,"loss_j":site.dissipated_energy}));
    }
    let p = (p1 - p0).distance(impulses[0] + impulses[1]);
    let h = (h1 - h0).distance(dh);
    let k = (k1 - k0 - a.kinetic_energy_change).abs();
    let work = (sum_work - a.kinetic_energy_change).abs();
    let loss = (sum_loss - law.dissipated)
        .abs()
        .max((sum_loss - a.dissipated_energy).abs());
    if [p, h, k, work, loss, errors[0], errors[1]]
        .into_iter()
        .any(|d| d > 1e-10)
    {
        return Err(format!("unbalanced native edge p={p:e} h={h:e} k={k:e} work={work:e} loss={loss:e} sites={errors:?}").into());
    }
    if n.project_sliding_edge_impact(ids.foot, ids.pad)?.is_some()
        || departure::ingest(n, &ids) != after
    {
        return Err("edge was applied twice".into());
    }
    Ok(
        json!({"admitted":true,"sites":recorded,"oracle_sites_ns":law.impulse.map(|j|(frame.q*j).to_array()),"site_errors_ns":errors,"linear_residual_ns":p,"angular_residual_nms":h,"native_energy_residual_j":k,"site_work_residual_j":work,"loss_residual_j":loss,"total_energy_j":a.kinetic_energy_change,"total_loss_j":a.dissipated_energy,"native_contraction":a.contraction_bound,"frozen_body_frame":[frame.q.x,frame.q.y,frame.q.z,frame.q.w],"world_shift":frame.shift.to_array()}),
    )
}
fn record(out: &Path) -> Result<()> {
    let i = make(0.35, 0.67, 0.18, [0.4, 0.8], false);
    let frame = Frame {
        q: DQuat::IDENTITY,
        shift: DVec3::ZERO,
        spin: 0.,
    };
    let source = fixture(i, frame, false);
    fs::write(out.join("edge.sdf"), &source)?;
    let law = i.edge.solve().map_err(|e| format!("{e:?}"))?;
    let mut totals = vec![];
    for label in ["ordinary", "analytical", "native"] {
        let mut s = load(&source, true)?;
        let mut r = ids(&s)?;
        let mut frames = vec![];
        let mut first = None;
        for tick in 0..=40 {
            let mut corners = [0.; 4];
            if tick == 1 {
                if label == "native" {
                    let mut owner = s.world_mut().resource_mut::<PhysicsBackend>();
                    let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
                    let receipt = n
                        .project_sliding_edge_impact(r.foot, r.pad)?
                        .ok_or("native locked edge declined its demonstration")?;
                    for p in receipt.sites {
                        let local = i.seed.state.q.conjugate()
                            * (DVec3::from(p.point)
                                - DVec3::from(i.seed.pad.position)
                                - PAD
                                - i.seed.state.c);
                        let k = usize::from(local.x > 0.) * 2 + usize::from(local.y > 0.);
                        corners[k] += p.impulse_on_foot[2];
                    }
                    first = Some(
                        json!({"impulses_ns":receipt.sites.map(|p|p.impulse_on_foot),"kinetic_energy_change_j":receipt.kinetic_energy_change,"dissipated_j":receipt.dissipated_energy}),
                    );
                } else if label == "analytical" {
                    let post = Seed {
                        state: PoseState {
                            v: law.velocity,
                            w: law.spin,
                            ..i.seed.state
                        },
                        ..i.seed
                    };
                    s = load(&scene(post, 0.35, false, "ordinary"), true)?;
                    r = ids(&s)?;
                    for k in 0..2 {
                        corners[i.indices[k]] = law.normal[k];
                    }
                }
            }
            if tick > 0 {
                s.step_exact(1)?;
                let n = engine(&s);
                let q = DQuat::from_array(n.state(r.foot).unwrap().orientation);
                let native = points(&n.rigid_contact_patches(), &r, q)?;
                for k in 0..4 {
                    corners[k] += native[k].z;
                }
            }
            let (f, u, p) = departure::ingest(engine(&s), &r);
            frames.push(json!({"tick":tick,"time_us":tick*500,"native_steps":u32::from(tick>0),"solver_iterations":4,"selected":false,"weak_start":false,"secondary_start":false,"normal_impulse_ns":corners.iter().sum::<f64>(),"corner_ns":corners,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),"gap_m":f.position[2]-p.position[2]-0.06-q_up(f).abs().dot(DVec3::new(0.18,0.1,0.06))}));
        }
        fs::write(
            out.join(format!("traces/{label}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":frames}),
            )?,
        )?;
        totals.push(json!({"label":label,"initial_impulse":first,"first":frames[1]}));
    }
    fs::write(out.join("public.json"), serde_json::to_vec(&totals)?)?;
    let lanes = [
        ("ordinary", "Ordinary locked edge"),
        ("analytical", "Independent jump, then native"),
        ("native", "Native two-site transaction"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"Native coupled reaction at two sliding vertices","capture_ticks":{"first_interval":1},"lanes":lanes}),
        )?,
    )?;
    Ok(())
}
pub(in super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut results = vec![];
    for (family, load, rev, roll) in [
        ("outer-heavy", [0.4, 0.8], false, 0.18),
        ("inner-heavy", [0.75, 0.45], true, 0.18),
        ("other-edge", [0.65, 0.55], false, -0.18),
        ("reverse-slide", [0.5, 0.7], true, -0.18),
    ] {
        for mu in [0.08, 0.2, 0.35, 0.5] {
            for yaw in [0., 27., 68., 112.] {
                let i = make(mu, f64::to_radians(yaw), roll, load, rev);
                for free in [false, true] {
                    for (name, frame) in [
                        (
                            "base",
                            Frame {
                                q: DQuat::IDENTITY,
                                shift: DVec3::ZERO,
                                spin: 0.,
                            },
                        ),
                        (
                            "inclined",
                            Frame {
                                q: DQuat::from_euler(glam::EulerRot::ZYX, 0.34, -0.27, 0.42),
                                shift: DVec3::new(2.3, -0.8, 0.6),
                                spin: 0.,
                            },
                        ),
                        (
                            "spinning-pad",
                            Frame {
                                q: DQuat::from_euler(glam::EulerRot::ZYX, -0.4, 0.34, -0.18),
                                shift: DVec3::new(-1.1, 1.8, -0.2),
                                spin: 1.3,
                            },
                        ),
                    ] {
                        let value = measure(i, frame, free).map_err(|e| {
                            format!("edge {family} µ={mu} yaw={yaw} free={free} {name}: {e}")
                        })?;
                        results.push(json!({"case":family,"mu":mu,"yaw_deg":yaw,"free_child":free,"frame":name,"result":value}));
                    }
                }
            }
        }
    }
    fs::write(out.join("native.json"), serde_json::to_vec(&results)?)?;
    record(out)
}
