// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Test live isolated ceiling treatment without supplying impact time, pose or impulse.
use super::*;
#[path = "inertia_upper_return.rs"]
pub(crate) mod returns;
use gz_physics_multifidelity::{BoxContactActions, OpposingCornerReceipt};
pub(crate) fn hit_record(
    r: OpposingCornerReceipt,
    base: u64,
    start: BodyState,
    end: BodyState,
) -> Value {
    let h = r.point;
    let i = h.impact;
    json!({"application_ns":base+i.elapsed.as_nanos()as u64,"physical_ns":base as f64 + h.pulse_start.as_nanos()as f64 + h.after_cut_s*1e9,
        "start_ns":base+h.pulse_start.as_nanos()as u64,"duration_ns":h.pulse_duration.as_nanos()as u64,"corner":4+h.corner,
        "timing_error_ns":r.timing_error_s*1e9,"other_gap_m":r.minimum_other_gap_m,"footprint_margin_m":r.minimum_footprint_margin_m,
        "outward_accel_m_s2":h.outward_acceleration,"point":i.point,"normal":i.normal,"impulse":i.impulse_on_foot,"gap_m":i.gap,
        "before_relative_velocity":i.before_relative_velocity,"after_relative_velocity":i.after_relative_velocity,"sticking":i.sticking,
        "mu":i.friction,"loss_j":i.dissipated_energy,"delta_energy_j":i.kinetic_energy_change,"incoming":body_view(r.incoming),"outgoing":body_view(r.outgoing),"start_state":body_view(start),"end_state":body_view(end)})
}
pub(crate) fn enable(s: &str) -> String {
    s.replace(
        "</guard_opponent>",
        "</guard_opponent><project_opponent_corner>true</project_opponent_corner>",
    )
}

fn raw(
    i: &Impact,
    g: &Gate,
    xml: &str,
    t: f64,
    x: Flight,
    h: u64,
    project: bool,
) -> crate::Result<Value> {
    let mut world = restart(i, g, xml, t, x, false, None)?;
    let ids = world.ids;
    let gate = world.gate;
    let mut phy = world.sim.world_mut().resource_mut::<PhysicsBackend>();
    let e = phy.downcast_mut::<MultiFidelityBackend>().unwrap();
    let initial = e.state(ids.foot).unwrap();
    let pair = [(ids.foot, ids.pad)];
    let triple = [(ids.foot, ids.pad, gate)];
    let mut observed = vec![];
    let out = e.advance_box_contacts(
        Duration::from_nanos(h),
        1,
        4.into(),
        BoxContactActions {
            guard_separated_flight: &pair,
            integrate_free_rotation: &pair,
            integrate_ballistic_translation: &pair,
            opposing_box_guards: &triple,
            project_opposing_corners: if project { &triple } else { &[] },
            ..Default::default()
        },
        Some(&mut |s| {
            observed.push(s.opposing_corner_impact);
        }),
    )?;
    let impact = out.interval.opposing_corner_impacts.first().copied();
    assert_eq!(observed, [impact]);
    assert!(out.interval.opposing_corner_impacts.len() <= 1);
    let scalar = |a: BodyId, b: BodyId| {
        out.interval
            .pairs
            .iter()
            .filter(|p| (p.a, p.b) == (a.min(b), a.max(b)))
            .map(|p| p.normal_impulse)
            .sum::<f64>()
    };
    let load = scalar(ids.foot, gate);
    if let Some(hit) = impact {
        assert!(hit.point.impact.support == gate);
        assert!(
            (load
                - DVec3::from(hit.point.impact.impulse_on_foot)
                    .dot(hit.point.impact.normal.into()))
            .abs()
                < 1e-10
        );
    }
    let end = e.state(ids.foot).unwrap();
    let change = M
        * (DVec3::from(end.velocity)
            - DVec3::from(initial.velocity)
            - i.r.gravity * (h as f64 * 1e-9));
    Ok(
        json!({"momentum_impulse":change.to_array(),"cut_ns":t*1e9,"span_ns":h,"upper":impact.map(|p|hit_record(p,0,initial,e.state(ids.foot).unwrap())),"lower_ns":scalar(ids.foot,ids.pad),"upper_ns":load,"end":body_view(e.state(ids.foot).unwrap()),"special":out.flight.unwrap().opposing_corner_calls}),
    )
}
fn brief(
    i: &Impact,
    g: &Gate,
    run: &Value,
    frames: &[Value],
    reference: Option<Crossing>,
    follow: Option<&Continued>,
) -> Value {
    let v = run["native_intervals"].as_array().unwrap();
    let hits: Vec<_> = v
        .iter()
        .filter_map(|s| {
            s["upper_projection"]
                .as_object()
                .map(|_| s["upper_projection"].clone())
        })
        .collect();
    let first = v.iter().find(|s| s["opponent_ns"].as_f64().unwrap() > 0.);
    let r = &i.r;
    let mut final_error = None;
    if let (Some(hit), Some(arc)) = (reference, follow) {
        let mut pos = None;
        for (tick, f) in frames.iter().enumerate() {
            let t = tick as f64 * 0.0005;
            if t + 1e-15 < hit.at || t > arc.through + 1e-15 {
                continue;
            }
            let mut exact = hit.after();
            let dt = t - hit.at;
            let n = (dt / RK_H).ceil().max(1.) as u32;
            for _ in 0..n {
                exact.advance(r, dt / f64::from(n));
            }
            let p: DVec3 = serde_json::from_value::<[f64; 3]>(f["foot"]["position"].clone())
                .unwrap()
                .into();
            let q = DQuat::from_array(
                serde_json::from_value(f["foot"]["orientation"].clone()).unwrap(),
            );
            pos = Some(
                json!({"at_ns":t*1e9,"com_m":(p+q*ARM).distance(exact.com),"orientation_rad":angular_distance(q,exact.q),
                "spin_rad_s":DVec3::from(serde_json::from_value::<[f64;3]>(f["foot"]["angular_velocity"].clone()).unwrap()).distance(r.body.omega(exact.q,exact.l))}),
            );
        }
        final_error = pos;
    }
    json!({"first_upper":first,"projections":hits,"native_calls":v.len(),"gate_ns":g.at*1e9,"qualified_endpoint":final_error,
        "primary_projections":run["applications"],"total_upper_ns":v.iter().map(|s|s["opponent_ns"].as_f64().unwrap()).sum::<f64>()})
}
pub(crate) fn write(path: &Path) -> crate::Result<()> {
    fs::create_dir_all(path)?;
    let mut cases = vec![];
    for i in inputs() {
        let mut rows = vec![];
        for delay in [0, 8, 20000, 750000, 1100000] {
            let g = gate(&i, delay as f64 * 1e-9);
            for mu in [0., 0.35, 0.8] {
                let xml = source(&i, &g, mu);
                let treatment = enable(&xml);
                let oracle = upper(&i, &g, i.mu.min(mu)).ok();
                let arc = oracle
                    .map(|hit| follow(&i, faces(&i.r, &g), [i.mu, i.mu.min(mu)], hit, WINDOW));
                let mut cold = vec![];
                if let Some(hit) = oracle {
                    let speed = Bounds::new(i.r.body, hit.after()).w * 0.32
                        + (hit.after().v - g.velocity)
                            .length()
                            .max((hit.after().v - i.r.pad_v).length())
                        + 1.;
                    let dur = (1e9 * (0.125 * hit.other_gap / speed).min(0.5e-6))
                        .floor()
                        .clamp(1., 500.) as u64;
                    {
                        for fraction in [0., 0.375, 0.875] {
                            let t = hit.at - fraction * dur as f64 * 1e-9;
                            let x = from_primary(&i.r, t);
                            let mut sample = raw(&i, &g, &treatment, t, x, dur, true)?;
                            sample["unprojected"] = raw(&i, &g, &xml, t, x, dur, false)?;
                            cold.push(sample);
                        }
                    }
                    if i.r.id == 1160
                        && delay == 1100000
                        && mu == 0.35
                        && let Some(next) = arc.as_ref().and_then(|a| a.next)
                        && next.surface == 0
                    {
                        views::projection(&path.join("upper"), &i, &g, mu, hit, next)?;
                    }
                } else {
                    let t = i.r.arrival - 10e-9;
                    cold.push(raw(
                        &i,
                        &g,
                        &treatment,
                        t,
                        from_primary(&i.r, t),
                        200,
                        true,
                    )?);
                }
                let mut native = vec![];
                for (enabled, s) in [(false, &xml), (true, &treatment)] {
                    let (run, frames) = measure_gate(&i, s, true)?;
                    native.push(json!({"enabled":enabled,"data":brief(&i,&g,&run,&frames,oracle,arc.as_ref())}));
                }
                rows.push(json!({"delay_ns":delay,"gate_mu":mu,"gate_center0":g.center.to_array(),"gate_velocity":g.velocity.to_array(),"reference":oracle.map(Crossing::data),"next":arc.as_ref().and_then(|a|a.next).map(Crossing::data),"follow":arc.as_ref().map(|a|a.reason),"qualified_duration_ns":arc.as_ref().map(|a|(a.through-g.at)*1e9),"cold":cold,"public":native}));
            }
        }
        cases.push(json!({"id":i.r.id,"mode":MODES[i.mode],"primary":law(&i),"tensor":i.r.body.tensor.to_cols_array(),"pad_velocity":i.r.pad_v.to_array(),"cases":rows}));
    }
    write_json(&path.join("projection.json"), &json!({"donors":cases}))
}
pub(crate) fn cost(path: &Path) -> crate::Result<()> {
    fs::create_dir_all(path)?;
    let mut out = vec![];
    for (k, i) in inputs().into_iter().enumerate() {
        for delay in [0, 750000, 1100000] {
            let g = gate(&i, delay as f64 * 1e-9);
            let xml = source(&i, &g, 0.35);
            let mut runs = vec![];
            for s in [xml.clone(), enable(&xml)] {
                let (mut sim, ids) = load(&s, false)?;
                sim.step_exact(8)?;
                let end = state(backend(&sim), ids)?;
                costed(&s, end)?;
                runs.push((s, end));
            }
            for rep in 0..6 {
                for z in 0..2 {
                    let n = (k + rep + z) % 2;
                    let sample = costed(&runs[n].0, runs[n].1)?;
                    out.push(json!({"id":i.r.id,"mode":MODES[i.mode],"delay_ns":delay,"enabled":n==1,"rep":rep,"gate_tick":(g.at/0.0005).ceil()as u64,"primary_tick":(i.r.arrival/0.0005).ceil()as u64,"wall_ns":sample.wall,"cpu_ns":sample.cpu}));
                }
            }
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":out}))
}
