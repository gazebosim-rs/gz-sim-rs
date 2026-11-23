// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::super::super::super::{authored::xml, transaction::Frame};
use super::super::transaction::{donor, frame};
use super::*;
use gz_physics_multifidelity::ContactSolverIterations;
pub(super) fn source(i: Input, s: Sole, gravity: DVec3) -> String {
    source_with(i, s, gravity, frame(false), false, "ordinary")
}
pub(super) fn source_with(
    i: Input,
    s: Sole,
    gravity: DVec3,
    f: Frame,
    free: bool,
    policy: &str,
) -> String {
    let i = donor(i, s.incoming);
    let mut src = xml(
        i,
        f,
        free,
        f.q * gravity,
        if policy == "normal" { "edge" } else { policy },
    );
    if policy == "normal" {
        src = src.replace("project_edge", "project_frictionless_sole");
    }
    let tensor = (s.shape.inertia
        - DMat3::from_diagonal(DVec3::new(1., 1., 0.) * (s.shape.mass * ARM.z.powi(2))))
        * 0.5;
    let desired = format!(
        "<mass>{:.17}</mass><inertia><ixx>{:.17}</ixx><iyy>{:.17}</iyy><izz>{:.17}</izz><ixy>{:.17}</ixy><ixz>{:.17}</ixz><iyz>{:.17}</iyz></inertia>",
        s.shape.mass * 0.5,
        tensor.x_axis.x,
        tensor.y_axis.y,
        tensor.z_axis.z,
        tensor.x_axis.y,
        tensor.x_axis.z,
        tensor.y_axis.z
    );
    src = src.replace(
        "<mass>1</mass><inertia><ixx>0.0201</ixx><iyy>0.0201</iyy><izz>0.025</izz></inertia>",
        &desired,
    );
    src.replace("A translating deck meets two vertices of a tilted sliding sole at the initial boundary.","A rocking foot is momentarily level as the opposite edge arrives at its frictionless support.")
}
pub(super) fn isolated(i: Input, sole: Sole, law: Law) -> Result<Vec<Value>> {
    let src = source(i, sole, DVec3::ZERO);
    let mut data = vec![];
    for relax in [1, 8, 32] {
        let mut s = load(&src, true)?;
        let id = ids(&s)?;
        let mut owner = s.world_mut().resource_mut::<PhysicsBackend>();
        let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
        let r = n.step_contact_interval(
            Duration::from_nanos(1),
            1,
            ContactSolverIterations {
                temporal: 4,
                pgs: Some(1),
                relaxation: Some(relax),
            },
        )?;
        let f = sole.incoming;
        let mask = |x: DVec3| [x.x / 0.18, x.y / 0.1];
        let mut j = 0.;
        let mut pos = DVec3::ZERO;
        let mut site = [0.; 4];
        for p in n.rigid_contact_patches() {
            let sign = if p.a == id.foot && p.b == id.pad {
                1.
            } else if p.a == id.pad && p.b == id.foot {
                -1.
            } else {
                continue;
            };
            for a in &p.points {
                let lever = DVec3::from(if sign > 0. { a.lever_a } else { a.lever_b });
                let local = f.motion.q.conjugate() * lever;
                let quadrant = usize::from(local.x > 0.) * 2 + usize::from(local.y > 0.);
                j += a.normal_impulse;
                pos += local * a.normal_impulse;
                site[quadrant] += a.normal_impulse;
            }
        }
        let pair = r
            .pairs
            .iter()
            .find(|p| [p.a, p.b].contains(&id.foot) && [p.a, p.b].contains(&id.pad));
        if pair.is_none_or(|p| (p.normal_impulse - j).abs() > 1e-12) || j <= 0. {
            return Err("missing native rectangular reaction".into());
        }
        let after = Seed::read(departure::ingest(n, &id)).state;
        let cp = DVec2::from(mask(pos / j));
        let e = sqrt_center(Some(cp), law.center)?;
        let wrong = site
            .into_iter()
            .zip(law.post)
            .filter(|(_, v)| *v > 1e-9)
            .map(|(p, _)| p)
            .sum::<f64>();
        data.push(json!({"relaxation":relax,"normal_ns":j,"per_corner_ns":site,"center_normalized":cp.to_array(),"total_error_ns":(j-law.total).abs(),"pressure_center_normalized_error":e,"load_on_opening_vertices_ns":wrong,"outgoing_v_error_m_s":after.v.distance(law.after.v),"outgoing_w_error_rad_s":after.w.distance(law.after.motion.w)}));
    }
    Ok(data)
}
fn sqrt_center(a: Option<DVec2>, b: Option<DVec2>) -> Result<f64> {
    match (a, b) {
        (Some(p), Some(q)) => Ok(p.distance(q)),
        (None, None) => Ok(0.),
        _ => Err("resultant mismatch".into()),
    }
}
pub(super) fn record(out: &Path, i: Input, sole: Sole, law: Law) -> Result<()> {
    let Mode::Edge(a, b) = law.mode else {
        return Err("demo requires transfer to the opposite edge".into());
    };
    let follow = Arc {
        shape: sole.shape,
        start: law.after,
        sites: Sites::Two([a, b]),
    };
    let next = follow.next(0.0000625, 0.018)?;
    let safe = (next.time / DT).floor() as u64;
    let last = safe.min(32);
    if last < 2 {
        return Err("next change too soon to render unambiguously".into());
    }
    let i = donor(i, sole.incoming);
    let start = Arc {
        start: sole.incoming,
        ..follow
    };
    let mut public = vec![];
    for (label, calls) in [("ordinary", 1), ("refined", 8), ("pressure", 0)] {
        let mut rows = if calls > 0 {
            super::super::native::native(i, start, calls, last, true, None)?.1
        } else {
            let mut series = vec![];
            let mut m = law.after.motion;
            for tick in 0..=last {
                let mut j = if tick == 1 { law.total } else { 0. };
                let s = if tick > 0 {
                    for _ in 0..8 {
                        let (n, r) = follow.advance_loaded(m, DT / 8.)?;
                        if r.into_iter().any(|f| f < 0.) {
                            return Err("pressure follow crossed tension".into());
                        }
                        m = n;
                        j += r.iter().sum::<f64>();
                    }
                    follow.state(tick as f64 * DT, m)
                } else {
                    sole.incoming
                };
                let (f, u, p) = super::super::replay::bodies(i, tick as f64 * DT, s);
                series.push(json!({"tick":tick,"time_us":tick*500,"native_steps":0,"reference_steps":if tick==0{0}else{8},"solver_iterations":0,"selected":false,"weak_start":false,"secondary_start":false,"normal_impulse_ns":j,"foot":wire(f),"upper":wire(u),"pad":wire(p),"gap_m":super::super::native::contact_gaps(f,p).into_iter().fold(f64::INFINITY,f64::min)}));
            }
            series
        };
        for s in &mut rows {
            s.as_object_mut().unwrap().remove("corner_ns");
        }
        public.push(
            json!({"label":label,"first_tick_ns":rows[1]["normal_impulse_ns"],"last_tick":last}),
        );
        fs::write(
            out.join(format!("traces/{label}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.10,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":rows}),
            )?,
        )?;
    }
    fs::write(out.join("flat.sdf"), source(i, sole, sole.shape.gravity))?;
    let lanes=[("ordinary","One native call"),("refined","Eight native calls"),("pressure","Independent pressure resultant")].map(|(l,n)|json!({"label":l,"name":n,"file":format!("traces/{l}.json"),"analytical":l=="pressure"}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"The flat sole transfers its reaction to the opposite edge","pair_totals_only":true,"capture_ticks":{"flat_touchdown":1},"lanes":lanes}),
        )?,
    )?;
    fs::write(
        out.join("public.json"),
        serde_json::to_vec(
            &json!({"instant_ns":law.total,"cop_normalized":law.center.unwrap().to_array(),"new_edge":[a,b],"next_event_s":next.time,"lanes":public}),
        )?,
    )?;
    Ok(())
}
