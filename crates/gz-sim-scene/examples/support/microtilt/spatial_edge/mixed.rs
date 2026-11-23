// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Mixed stick/slide and feasible self-stress at two points of one rigid link.
use super::*;
use rapier3d_f64::na::{Matrix2, Matrix3, SMatrix, SVector};
type M5 = SMatrix<f64, 5, 5>;
type V5 = SVector<f64, 5>;
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Mode {
    BothStick,
    Stick(usize),
}
fn name(m: Mode) -> &'static str {
    match m {
        Mode::BothStick => "both-stick",
        Mode::Stick(0) => "first-sticks",
        Mode::Stick(_) => "second-sticks",
    }
}
struct Answer {
    mode: Mode,
    axis: DVec3,
    transverse: DVec3,
    force: V5,
    range: [f64; 2],
    beta: f64,
    exclusion: [f64; 2],
    velocity: DVec3,
    spin: DVec3,
}
impl Answer {
    /// The sole self-stress exchanges equal and opposite along-edge impulse.
    fn split(&self, x: f64) -> [DVec3; 2] {
        [
            self.axis * x + self.transverse * self.force[1] + DVec3::Z * self.force[2],
            self.axis * (self.force[0] - x)
                + self.transverse * self.force[3]
                + DVec3::Z * self.force[4],
        ]
    }
    fn closest_split(&self, measured: [DVec3; 2]) -> (f64, [DVec3; 2]) {
        let zero = self.split(0.);
        let x = (0.5
            * self
                .axis
                .dot((measured[0] - zero[0]) - (measured[1] - zero[1])))
        .clamp(self.range[0], self.range[1]);
        (x, self.split(x))
    }
}
fn model(mu: f64, yaw: f64, roll: f64, mode: Mode, reverse: bool, n: [f64; 2]) -> Input {
    let mut input = make(mu, yaw, roll, n, reverse);
    let e = (input.edge.arm[1] - input.edge.arm[0]).normalize();
    let t = DVec3::Z.cross(e);
    let sign = if reverse { -1. } else { 1. };
    let w = sign * 0.7 * e
        + if mode == Mode::BothStick {
            DVec3::ZERO
        } else {
            2.1 * sign * DVec3::Z
        };
    let index = if let Mode::Stick(i) = mode { i } else { 0 };
    let v = -w.cross(input.edge.arm[index]);
    let j = std::array::from_fn(|k| {
        let slip = v + w.cross(input.edge.arm[k]);
        let ti = if mode == Mode::BothStick {
            mu * n[k] * ([0.34, -0.18][k] * sign * e + [0.28, -0.37][k] * t)
        } else if k == index {
            mu * n[k] * (0.47 * sign * e - 0.31 * t)
        } else {
            -mu * n[k] * slip.normalize()
        };
        n[k] * DVec3::Z + ti
    });
    input.edge.velocity = v - (j[0] + j[1]) / input.edge.mass;
    input.edge.spin = w - input.edge.inertia.inverse()
        * (input.edge.arm[0].cross(j[0]) + input.edge.arm[1].cross(j[1]));
    input.seed.state.v = input.edge.velocity;
    input.seed.state.w = input.edge.spin;
    input.desired = j;
    input.after_v = v;
    input.after_w = w;
    input
}
fn solve(edge: Edge) -> Result<Answer> {
    let axis = (edge.arm[1] - edge.arm[0]).normalize();
    let t = DVec3::Z.cross(axis);
    let component = |v: [DVec3; 2]| {
        V5::from_row_slice(&[
            0.5 * axis.dot(v[0] + v[1]),
            t.dot(v[0]),
            v[0].z,
            t.dot(v[1]),
            v[1].z,
        ])
    };
    let before = edge.arm.map(|r| edge.velocity + edge.spin.cross(r));
    let u = component(before);
    if before.iter().any(|v| v.z >= 0.) || (before[0] - before[1]).dot(axis).abs() > 1e-13 {
        return Err("closing rigid edge required".into());
    }
    let mut exclusion = [0.; 2];
    for i in 0..2 {
        let j = 1 - i;
        let nt = |k| DVec3::new(edge.mobility(k, i, axis).z, edge.mobility(k, i, t).z, 0.);
        let lo = edge.mobility(i, i, DVec3::Z).z - edge.friction[i] * nt(i).length();
        if lo <= 0. {
            return Err("single-contact alternative unresolved".into());
        }
        let hi = edge.mobility(j, i, DVec3::Z).z + edge.friction[i] * nt(j).length();
        exclusion[j] = before[j].z + hi.max(0.) * (-before[i].z) / lo;
        if exclusion[j] >= -1e-13 {
            return Err("single-contact alternative unresolved".into());
        }
    }
    let bases = [
        [axis * 0.5, axis * 0.5],
        [t, DVec3::ZERO],
        [DVec3::Z, DVec3::ZERO],
        [DVec3::ZERO, t],
        [DVec3::ZERO, DVec3::Z],
    ];
    let h = M5::from_columns(&bases.map(|j| {
        component(std::array::from_fn(|i| {
            edge.mobility(i, 0, j[0]) + edge.mobility(i, 1, j[1])
        }))
    }));
    if h.cholesky().is_none() {
        return Err("nonpositive quotient mobility".into());
    }
    let n = [2, 4];
    let tang = [0, 1, 3];
    let nn = Matrix2::from_fn(|i, j| h[(n[i], n[j])]);
    let nt = SMatrix::<f64, 2, 3>::from_fn(|i, j| h[(n[i], tang[j])]);
    let a = nn.try_inverse().ok_or("singular normal block")? * nt;
    let s = Matrix3::from_fn(|i, j| h[(tang[i], tang[j])]) - nt.transpose() * a;
    let inv = s.try_inverse().ok_or("singular reduced tangent block")?;
    // Monotonicity of the subgradient of slip speed bounds any two admissible
    // responses. The opposing along-edge null impulse was eliminated above.
    let beta = (0..2)
        .map(|i| {
            let y = i + 1;
            let eigen =
                0.5 * (s[(0, 0)] + s[(y, y)] + (s[(0, 0)] - s[(y, y)]).hypot(2. * s[(0, y)]));
            edge.friction[i] * (eigen * (a.row(i) * inv * a.row(i).transpose())[(0, 0)]).sqrt()
        })
        .sum::<f64>();
    if !beta.is_finite() || beta >= 1. - 1e-12 {
        return Err("uniqueness bound unresolved".into());
    }
    let mut accepted = vec![];
    for (mode, sign) in [
        (Mode::BothStick, 0.),
        (Mode::Stick(0), 1.),
        (Mode::Stick(0), -1.),
        (Mode::Stick(1), 1.),
        (Mode::Stick(1), -1.),
    ] {
        let mut mat = h;
        let mut rhs = -u;
        if let Mode::Stick(i) = mode {
            let k = 1 - i;
            let y = 1 + 2 * k;
            mat.row_mut(y).fill(0.);
            mat[(y, y)] = 1.;
            mat[(y, y + 1)] = sign * edge.friction[k];
            rhs[y] = 0.;
        }
        let Some(force) = mat.lu().solve(&rhs) else {
            continue;
        };
        let post = u + h * force;
        if force[2] <= 0.
            || force[4] <= 0.
            || post[2].abs() > 1e-12
            || post[4].abs() > 1e-12
            || post[0].abs() > 1e-12
        {
            continue;
        }
        let x = match mode {
            Mode::BothStick => {
                if post.norm() > 1e-12 {
                    continue;
                }
                let radius = |i: usize| {
                    let normal = force[2 + 2 * i] * edge.friction[i];
                    let y = force[1 + 2 * i].abs();
                    if y > normal {
                        None
                    } else {
                        Some(((normal - y) * (normal + y)).sqrt())
                    }
                };
                let (Some(x0), Some(x1)) = (radius(0), radius(1)) else {
                    continue;
                };
                let lo = (-x0).max(force[0] - x1);
                let hi = x0.min(force[0] + x1);
                if hi < lo {
                    continue;
                }
                [lo, hi]
            }
            Mode::Stick(i) => {
                let k = 1 - i;
                if sign * post[1 + 2 * k] <= 1e-11
                    || post[1 + 2 * i].abs() > 1e-12
                    || force[0].hypot(force[1 + 2 * i]) > edge.friction[i] * force[2 + 2 * i]
                {
                    continue;
                }
                let x = if i == 0 { force[0] } else { 0. };
                [x, x]
            }
        };
        let mut sol = Answer {
            mode,
            axis,
            transverse: t,
            force,
            range: x,
            beta,
            exclusion,
            velocity: DVec3::ZERO,
            spin: DVec3::ZERO,
        };
        let j = sol.split((x[0] + x[1]) * 0.5);
        sol.velocity = edge.velocity + (j[0] + j[1]) / edge.mass;
        sol.spin = edge.spin
            + edge.inertia.inverse() * (edge.arm[0].cross(j[0]) + edge.arm[1].cross(j[1]));
        accepted.push(sol);
    }
    if accepted.len() != 1 {
        return Err("no unique certified sticking mode".into());
    }
    Ok(accepted.pop().unwrap())
}
fn fallback(i: Input) -> Result<Value> {
    let base = scene(i.seed, i.edge.friction[0], false, "relax8");
    let authored = base.replace(
        "</support>",
        "</support><project_sliding_edge>true</project_sliding_edge>",
    );
    let mut plain = load(&base, true)?;
    let mut registered = load(&authored, true)?;
    let a = ids(&plain)?;
    let b = ids(&registered)?;
    let mut gates = Vec::new();
    let mut delta = 0f64;
    for tick in 1..=3 {
        plain.step_exact(1)?;
        registered.step_exact(1)?;
        let x = last_refinement(&plain).filter(|r| r.iteration == tick);
        let y = last_refinement(&registered).filter(|r| r.iteration == tick);
        if y.is_some_and(|r| !r.interval.sliding_edge_impacts.is_empty()) {
            return Err("uncertified sliding transaction replaced native mixed contact".into());
        }
        let s =
            |sim: &Simulation, id: &Reading| Seed::read(departure::ingest(engine(sim), id)).state;
        delta = delta.max(near(s(&plain, &a), s(&registered, &b)));
        gates.push([
            x.map_or(1, |r| r.interval.native_steps),
            y.map_or(1, |r| r.interval.native_steps),
        ]);
    }
    if delta > 1e-12 || gates.iter().any(|g| g[0] != g[1]) {
        return Err(format!(
            "mixed decline modified ordinary native control: {delta:e}, {gates:?}"
        )
        .into());
    }
    Ok(json!({"paired_native_counts":gates,"max_state_difference":delta}))
}
fn public_view(out: &Path, native: bool) -> Result<()> {
    let mode = if native {
        Mode::BothStick
    } else {
        Mode::Stick(0)
    };
    let i = model(0.35, 0.67, 0.18, mode, false, [0.48, 0.72]);
    let law = solve(i.edge)?;
    let physical = law.split(law.range[0]);
    let mut summaries = vec![];
    for policy in if native {
        ["ordinary", "physical", "native"]
    } else {
        ["ordinary", "pgs32", "physical"]
    } {
        let src = scene(i.seed, 0.35, false, "ordinary");
        let mut s = load(&src, true)?;
        let mut r = ids(&s)?;
        if policy == "pgs32" {
            s.world_mut()
                .resource_mut::<PhysicsBackend>()
                .downcast_mut::<MultiFidelityBackend>()
                .unwrap()
                .set_contact_pgs_passes(32)?;
        }
        let mut rows = vec![];
        let mut observed = None;
        for tick in 0..=40 {
            let mut loads = [0.; 4];
            if tick == 1 && policy == "native" {
                let mut owner = s.world_mut().resource_mut::<PhysicsBackend>();
                let n = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
                let hit = n
                    .project_mixed_edge_impact(r.foot, r.pad)?
                    .ok_or("demonstration mixed-edge impulse declined")?;
                for k in 0..2 {
                    loads[i.indices[k]] = hit.impulses.normal[k];
                }
                observed = Some(
                    json!({"normal_ns":hit.impulses.normal,"transverse_ns":hit.impulses.transverse,"longitudinal_total_ns":hit.impulses.longitudinal_total,"first_longitudinal_range_ns":hit.impulses.first_longitudinal_range,"mode":format!("{:?}",hit.mode)}),
                );
            }
            if tick == 1 && policy == "physical" {
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
                    loads[i.indices[k]] += physical[k].z;
                }
            }
            if tick > 0 {
                s.step_exact(1)?;
                let n = engine(&s);
                let q = DQuat::from_array(n.state(r.foot).unwrap().orientation);
                let values = moments(&n.rigid_contact_patches(), &r, q, false)?.2;
                for k in 0..4 {
                    loads[k] += values[k];
                }
            }
            let (f, u, p) = departure::ingest(engine(&s), &r);
            rows.push(json!({"tick":tick,"time_us":tick*500,"native_steps":u32::from(tick>0),"solver_iterations":4,"selected":false,"weak_start":false,"secondary_start":false,"normal_impulse_ns":loads.iter().sum::<f64>(),"corner_ns":loads,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),"gap_m":f.position[2]-p.position[2]-0.06-q_up(f).abs().dot(DVec3::new(0.18,0.1,0.06))}));
        }
        fs::write(
            out.join(format!("traces/{policy}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":rows}),
            )?,
        )?;
        let mut summary = json!({"label":policy,"first":rows[1]});
        if let Some(hit) = observed {
            summary["initial_jump"] = hit;
        }
        summaries.push(summary);
        if policy == "ordinary" {
            fs::write(out.join("edge.sdf"), &src)?;
        }
    }
    let lanes = if native {
        [
            ("ordinary", "Ordinary locked edge"),
            ("physical", "Independent jump, then native"),
            ("native", "Set-valued native edge impact"),
        ]
    } else {
        [
            ("ordinary", "Ordinary locked edge"),
            ("pgs32", "32 fixed-time PGS passes"),
            ("physical", "Mixed physical jump, then native"),
        ]
    }
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":if native {"A native edge landing with two sticking vertices"} else {"One vertex sticks while the other slides"},"capture_ticks":{"first_interval":1},"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("public.json"), serde_json::to_vec(&summaries)?)?;
    Ok(())
}
pub(in super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut rows = vec![];
    let mut declines = vec![];
    for mode in [Mode::BothStick, Mode::Stick(0), Mode::Stick(1)] {
        for mu in [0.08, 0.2, 0.35, 0.5, 0.65, 0.8] {
            for (roll, reverse, n) in [(0.18, false, [0.48, 0.72]), (-0.18, true, [0.72, 0.48])] {
                for yaw_deg in [0., 27., 68., 112.] {
                    let input = model(mu, f64::to_radians(yaw_deg), roll, mode, reverse, n);
                    let s = match solve(input.edge) {
                        Ok(s) => s,
                        Err(e) => {
                            declines.push(json!({"mode":name(mode),"mu":mu,"roll":roll,"yaw_deg":yaw_deg,"reason":e.to_string(),"native_fallback":fallback(input)?}));
                            continue;
                        }
                    };
                    let (_, target) = s.closest_split(input.desired);
                    let residual = target[0]
                        .distance(input.desired[0])
                        .max(target[1].distance(input.desired[1]));
                    if s.mode != mode
                        || s.velocity.distance(input.after_v) > 1e-12
                        || s.spin.distance(input.after_w) > 1e-12
                        || residual > 1e-12
                    {
                        return Err("manufactured mixed edge state not recovered".into());
                    }
                    let mut native = vec![];
                    for rigid in [false, true] {
                        for pass in [1, 8, 32] {
                            let obs = native_points(input, rigid, pass)?;
                            let (x, nearest) = s.closest_split(obs.impulses);
                            let after = obs.end;
                            let measured = obs.impulses;
                            native.push(json!({"single":rigid,"passes":pass,"impulses_ns":measured.map(|j|j.to_array()),"post_point_m_s":input.edge.arm.map(|r|(after.v+after.w.cross(r)).to_array()),"closest_along_ns":x,"site_errors_to_set_ns":[measured[0].distance(nearest[0]),measured[1].distance(nearest[1])],"spin_error_rad_s":after.w.distance(s.spin),"angular_balance_nms":obs.angular_balance,"total_error_ns":(measured[0]+measured[1]).distance(nearest[0]+nearest[1])}));
                        }
                    }
                    // The registered sliding-only transaction must decline these
                    // modes without changing either link velocity or pose.
                    let mut test = load(&scene(input.seed, mu, false, "ordinary"), true)?;
                    let ids = ids(&test)?;
                    let mut p = test.world_mut().resource_mut::<PhysicsBackend>();
                    let solver = p.downcast_mut::<MultiFidelityBackend>().unwrap();
                    let before = departure::ingest(solver, &ids);
                    if solver
                        .project_sliding_edge_impact(ids.foot, ids.pad)?
                        .is_some()
                        || departure::ingest(solver, &ids) != before
                    {
                        return Err(
                            "sliding-only native transaction accepted a sticking site".into()
                        );
                    }
                    rows.push(json!({"mode":name(mode),"mu":mu,"roll":roll,"yaw_deg":yaw_deg,"axis":s.axis.to_array(),"edge_transverse":s.transverse.to_array(),"normal_loads_ns":[s.force[2],s.force[4]],"identifiable":[s.force[0],s.force[1],s.force[2],s.force[3],s.force[4]],"first_along_interval_ns":s.range,"beta":s.beta,"singleton_other_upper_m_s":s.exclusion,"manufactured_ns":input.desired.map(|j|j.to_array()),"arms_m":input.edge.arm.map(|v|v.to_array()),"before":input.seed.state.wire(),"vpost":s.velocity.to_array(),"wpost":s.spin.to_array(),"endpoint_splits":[s.split(s.range[0]).map(|v|v.to_array()),s.split(s.range[1]).map(|v|v.to_array())],"native":native}));
                }
            }
        }
    }
    fs::write(out.join("mixed.json"), serde_json::to_vec(&rows)?)?;
    fs::write(
        out.join("declines.json"),
        serde_json::to_vec_pretty(&declines)?,
    )?;
    public_view(out, false)
}

#[path = "native_mixed.rs"]
pub(in super::super::super) mod native;

#[path = "authored_mixed.rs"]
pub(in super::super::super) mod authored;
