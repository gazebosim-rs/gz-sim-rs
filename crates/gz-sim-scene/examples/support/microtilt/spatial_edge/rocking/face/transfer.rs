// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fractional public-clock transfer between opposite frictionless sole edges.
#[path = "transfer/registered.rs"]
mod registered;
use super::*;
use gz_physics::{
    EdgeSupportState, FrictionlessEdgeSupport, FrictionlessSoleArc, SoleBearingAdvance,
};
fn physical(s: EdgeSupportState) -> State {
    State {
        c: s.center,
        v: s.velocity,
        motion: Motion {
            q: s.orientation,
            w: s.angular_velocity,
        },
    }
}
fn coordinates(s: State) -> EdgeSupportState {
    EdgeSupportState {
        center: s.c,
        velocity: s.v,
        orientation: s.motion.q,
        angular_velocity: s.motion.w,
    }
}
fn sole(a: Arc) -> FrictionlessSoleArc {
    FrictionlessSoleArc {
        mass: a.shape.mass,
        inertia_body: a.shape.inertia,
        corners: std::array::from_fn(|k| a.shape.vertices[k]),
        normal: DVec3::Z,
        plane_offset: 0.,
        gravity: a.shape.gravity,
    }
}
fn edge(a: Arc) -> Result<gz_physics::BoundFrictionlessEdge> {
    let Sites::Two(keys) = a.sites else {
        return Err("two-point bearing required".into());
    };
    Ok(FrictionlessEdgeSupport {
        mass: a.shape.mass,
        inertia_body: a.shape.inertia,
        sites_body: keys.map(|k| a.shape.vertices[k]),
        normal: DVec3::Z,
        plane_offset: 0.,
        gravity: a.shape.gravity,
    }
    .bind(coordinates(a.start))?
    .ok_or("unbound two-bearing donor")?)
}
fn relative(s: State, t: State) -> [f64; 4] {
    [
        s.c.distance(t.c),
        s.v.distance(t.v),
        s.motion.w.distance(t.motion.w),
        4. * (0.5
            * (s.motion.q - t.motion.q)
                .length()
                .min((s.motion.q + t.motion.q).length()))
        .asin(),
    ]
}
fn integrate(a: Arc, mut m: Motion, t: f64) -> Result<([f64; 2], Motion)> {
    let steps = (t / 0.00003125).ceil().max(1.) as u32;
    let h = t / f64::from(steps);
    let mut loads = [0.; 2];
    for _ in 0..steps {
        let (next, j) = a.advance_loaded(m, h)?;
        m = next;
        for k in 0..2 {
            loads[k] += j[k];
        }
    }
    Ok((loads, m))
}
struct Donor {
    path: Arc,
    arrival: Arrival,
    impact: Law,
    follow: Arc,
}
impl Donor {
    fn new(path: Arc, end: Arrival) -> Result<Option<Self>> {
        if (end.state.motion.q * DVec3::Z - DVec3::Z).length() > 1e-9 {
            return Ok(None);
        }
        let physical = Sole {
            shape: path.shape,
            incoming: end.state,
        };
        let impact = physical.solve()?;
        let Mode::Edge(i, k) = impact.mode else {
            return Ok(None);
        };
        let follow = Arc {
            shape: path.shape,
            start: impact.after,
            sites: Sites::Two([i, k]),
        };
        if follow.load(impact.after.motion)?.iter().any(|v| *v <= 0.) {
            return Ok(None);
        }
        Ok(Some(Self {
            path,
            arrival: end,
            impact,
            follow,
        }))
    }
    fn cut(&self, phase: f64) -> Result<(EdgeSupportState, Value, SoleBearingAdvance)> {
        let a = self.path;
        let start_time = self.arrival.time - phase;
        let start = edge(a)?.sample(start_time, 0.0000625)?.state;
        let Sites::Two(keys) = a.sites else {
            unreachable!()
        };
        let accepted = sole(a).advance(start, keys, DT)?;
        let result = accepted.ok_or_else(|| {
            format!(
                "declined physical transfer at {} s, phase {phase:e}",
                self.arrival.time
            )
        })?;
        let Some((transfer, end)) = result.transfer else {
            return Err(format!("missing transition, phase {phase:e}").into());
        };
        let (pre, _) = integrate(a, physical(start).motion, phase)?;
        let (post, endmotion) = integrate(self.follow, self.impact.after.motion, DT - phase)?;
        let state = self.follow.state(DT - phase, endmotion);
        let reference = relative(physical(end.end.state), state);
        let time = (transfer.seconds - phase).abs();
        let pressure = (transfer.pressure.resultant - self.impact.total).abs();
        let Sites::Two(next_keys) = self.follow.sites else {
            unreachable!()
        };
        let lookup = |idx: usize| next_keys.iter().position(|k| *k == idx).unwrap();
        let j = (0..2)
            .map(|k| {
                (result.first.end.signed_impulse[k] - pre[k])
                    .abs()
                    .max((end.end.signed_impulse[k] - post[lookup(end.vertices[k])]).abs())
            })
            .fold(0., f64::max);
        let net = a.shape.mass * (state.v - physical(start).v - a.shape.gravity * DT);
        let total = result.first.end.signed_impulse.iter().sum::<f64>()
            + transfer.pressure.resultant
            + end.end.signed_impulse.iter().sum::<f64>();
        let momentum = net.distance(total * DVec3::Z);
        if time > 2e-11
            || [pressure, j, momentum]
                .into_iter()
                .chain(reference)
                .any(|v| v > 1e-9 || !v.is_finite())
        {
            return Err(format!(
                "fractional transfer error {time:e} s J {j:e}, P {momentum:e}, state {reference:?}"
            )
            .into());
        }
        let data = json!({"phase_s":phase,"event_s":transfer.seconds,"event_bracket_s":transfer.numerical_bracket,"old":keys,"new":end.vertices,"pressure_ns":transfer.pressure.resultant,"independent_pressure_ns":self.impact.total,"first_smooth_ns":result.first.end.signed_impulse,"next_smooth_ns":end.end.signed_impulse,"reference_first_ns":pre,"reference_next_ns":end.vertices.map(|id|post[lookup(id)]),"max_smooth_error_ns":j,"momentum_residual_ns":momentum,"center_velocity_spin_orientation_error":reference,"end":state_wire(physical(end.end.state)),"independent_end":state_wire(state)});
        Ok((start, data, result))
    }
}
fn replay(out: &Path, input: Input, d: &Donor) -> Result<()> {
    let phase = 0.000213819760;
    let (start, _, _) = d.cut(phase)?;
    let input = super::super::transaction::donor(input, physical(start));
    let path = Arc {
        start: physical(start),
        ..d.path
    };
    let Sites::Two(keys) = d.path.sites else {
        unreachable!()
    };
    let mut full = vec![];
    for (label, calls) in [("ordinary", 1), ("refined", 8), ("resolved", 0)] {
        let mut events = vec![];
        let mut rows = if calls != 0 {
            super::super::native::native(input, path, calls, 20, true, None)?.1
        } else {
            let mut v = start;
            let mut ids = keys;
            let mut data = vec![];
            for tick in 0..=20 {
                let mut normal = 0.;
                if tick > 0 {
                    let advance = sole(path)
                        .advance(v, ids, DT)?
                        .ok_or("unresolved replay span")?;
                    normal = advance.first.end.signed_impulse.iter().sum();
                    if let Some((e, p)) = advance.transfer {
                        normal += e.pressure.resultant + p.end.signed_impulse.iter().sum::<f64>();
                        ids = p.vertices;
                        events.push(json!({"tick":tick,"time_us":((tick-1)as f64*DT+e.seconds)*1e6,"normal_ns":e.pressure.resultant,"new":ids}));
                    }
                    v = advance.end();
                }
                let (f, u, p) = super::super::replay::bodies(input, tick as f64 * DT, physical(v));
                data.push(json!({"tick":tick,"time_us":tick*500,"native_steps":0,"solver_iterations":0,"selected":false,"weak_start":false,"secondary_start":false,"normal_impulse_ns":normal,"foot":wire(f),"upper":wire(u),"pad":wire(p),"gap_m":super::super::native::contact_gaps(f,p).into_iter().fold(f64::INFINITY,f64::min)}));
            }
            data
        };
        for r in &mut rows {
            r.as_object_mut().unwrap().remove("corner_ns");
        }
        full.push(
            json!({"label":label,"first_tick_ns":rows[1]["normal_impulse_ns"],"events":events}),
        );
        fs::write(
            out.join(format!("traces/{label}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.10,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":rows}),
            )?,
        )?;
    }
    let lanes=[("ordinary","One native call"),("refined","Eight native calls"),("resolved","Resolved opposite-edge transfer")].map(|(l,n)|json!({"label":l,"name":n,"file":format!("traces/{l}.json"),"analytical":l=="resolved"}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"The bearing crosses the flat sole inside one actuator interval","pair_totals_only":true,"capture_ticks":{"opposite_edge_transfer":1},"lanes":lanes}),
        )?,
    )?;
    fs::write(
        out.join("transfer.sdf"),
        scene(input.seed, 0., false, "ordinary"),
    )?;
    fs::write(out.join("public.json"), serde_json::to_vec(&full)?)?;
    Ok(())
}
pub(in super::super::super::super::super::super) fn run(
    out: &Path,
    native: bool,
    bench: bool,
) -> Result<()> {
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
                        let e = path.next(0.0000625, 0.4)?;
                        let End::Contact(new) = e.end else {
                            continue;
                        };
                        for variant in ["natural", "triaxial"] {
                            let mut shape = path.shape;
                            if variant == "triaxial" {
                                shape.inertia = DMat3::from_cols(
                                    DVec3::new(0.034, 0.004, -0.003),
                                    DVec3::new(0.004, 0.051, 0.006),
                                    DVec3::new(-0.003, 0.006, 0.055),
                                );
                            }
                            let j = shape.recollide(e.state, path.sites.first(), new)?;
                            let Sites::Two(_) = j.new_sites else {
                                continue;
                            };
                            let a = Arc {
                                shape,
                                start: j.after,
                                sites: j.new_sites,
                            };
                            let third = a.next(0.0000625, 0.35)?;
                            let Some(d) = Donor::new(a, third)? else {
                                continue;
                            };
                            if bench {
                                if variant == "natural" && yaw > 0. {
                                    return registered::cost(out, i, &d);
                                }
                                continue;
                            }
                            for phase_ns in [0.25, 0.84, 62500., 213819.76, 499999.78] {
                                let (start, mut data, _) = d.cut(phase_ns * 1e-9)?;
                                if native {
                                    data["registered"] =
                                        registered::check(i, &d, start, phase_ns * 1e-9)?;
                                }
                                rows.push(json!({"active":active,"pitch":pitch,"opening":opening,"rate":rate,"yaw":yaw,"mass":variant,"rocking_time_s":e.time+third.time,"result":data}));
                            }
                            if demo.is_none() && variant == "natural" && yaw > 0. {
                                demo = Some((i, d));
                            }
                        }
                    }
                }
            }
        }
    }
    fs::write(out.join("transfers.json"), serde_json::to_vec(&rows)?)?;
    let (i, d) = demo.ok_or("missing transfer donor")?;
    if native {
        registered::replay(out, i, &d)
    } else {
        replay(out, i, &d)
    }
}
