// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two independently ordered impacts within one Bevy actuator interval.
use super::*;
use gz_physics_multifidelity::OpposingReturnReceipt;
pub(crate) fn record(r: OpposingReturnReceipt, base: u64, end: BodyState) -> Value {
    let mut v = hit_record(r.lower, base, r.branch_start, end);
    v["corner"] = json!(r.lower.point.corner);
    v["upper_corner"] = json!(r.previous_upper_corner + 4);
    v["previous_upper_ns"] = json!(base as f64 + r.previous_upper_s * 1e9);
    v["prior_upper_gaps_m"] = json!(r.previous_upper_gaps_m);
    v["native_start"] = body_view(r.native_start);
    v["native_start_ns"] = json!(base + r.native_start_elapsed.as_nanos() as u64);
    v
}
pub(crate) fn enroll(xml: &str) -> String {
    xml.replace(
        "</project_opponent_corner>",
        "</project_opponent_corner><project_opponent_return>true</project_opponent_return>",
    )
}
struct Trial {
    start: f64,
    dt: u64,
    calls: u32,
    enabled: bool,
}
fn native(i: &Impact, g: &Gate, xml: &str, trial: Trial) -> crate::Result<Value> {
    let mut cut = restart(
        i,
        g,
        xml,
        trial.start,
        from_primary(&i.r, trial.start),
        false,
        None,
    )?;
    let ids = cut.ids;
    let gate = cut.gate;
    let mut backend = cut.sim.world_mut().resource_mut::<PhysicsBackend>();
    let e = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
    let pair = [(ids.foot, ids.pad)];
    let triple = [(ids.foot, ids.pad, gate)];
    let actions = BoxContactActions {
        guard_separated_flight: &pair,
        integrate_free_rotation: &pair,
        integrate_ballistic_translation: &pair,
        opposing_box_guards: &triple,
        project_opposing_corners: &triple,
        project_opposing_returns: if trial.enabled { &triple } else { &[] },
        ..Default::default()
    };
    let initial = e.state(ids.foot).unwrap();
    let mut previous = initial;
    let mut calls = vec![];
    let out=e.advance_box_contacts(Duration::from_nanos(trial.dt),trial.calls,4.into(),actions,Some(&mut |s|{
        let end=s.state(ids.foot).unwrap();
        let reached=s.opposing_return_impact.copied();
        let mut pad=Wrench::default();let mut roof=Wrench::default();
        for p in s.contact_patches(){if !(p.a==ids.foot || p.b==ids.foot){continue;}
            let dest=if p.a==gate||p.b==gate{&mut roof}else{&mut pad};dest.add(&p,ids.foot,DQuat::from_array(end.orientation));
        }
        let upper=s.opposing_corner_impact.map(|h|hit_record(h,0,previous,reached.map_or(end,|r|r.branch_start)));
        calls.push(json!({"from_ns":(s.elapsed-s.duration).as_nanos()as u64,"to_ns":s.elapsed.as_nanos()as u64,"upper":upper,"lower":reached.map(|r|record(r,0,end)),"native_pad":pad.data(),"native_gate":roof.data(),"end":body_view(end)}));previous=end;
    }))?;
    assert_eq!(out.interval.native_steps, trial.calls);
    let pair_load = |body| {
        let key = (ids.foot.min(body), ids.foot.max(body));
        out.interval.pairs.iter().find(|p|(p.a,p.b)==key).map(|p|{let sign=if key.0==ids.foot{1.}else{-1.};json!({"normal_ns":p.normal_impulse,"normal_on_foot":(sign*DVec3::from(p.normal_impulse_on_a)).to_array(),"tangent_on_foot":p.tangent_impulse_on_a.map(|j|(sign*DVec3::from(j)).to_array()),"first_ns":p.first_positive_end.map(|t|t.as_nanos()as u64)})})
    };
    Ok(
        json!({"start_ns":trial.start*1e9,"dt_ns":trial.dt,"n":trial.calls,"enabled":trial.enabled,"start":body_view(initial),"calls":calls,"pad":pair_load(ids.pad),"gate":pair_load(gate)}),
    )
}
pub(in super::super::super) fn advances(i: &Impact, x: Crossing, t: f64) -> Flight {
    let mut s = x.after();
    let dt = t - x.at;
    let n = (dt / RK_H).ceil().max(1.) as u32;
    for _ in 0..n {
        s.advance(&i.r, dt / f64::from(n));
    }
    s
}
fn compact(
    i: &Impact,
    run: &Value,
    frames: &[Value],
    next: Option<Crossing>,
    end: Option<&Continued>,
) -> Value {
    let spans = run["native_intervals"].as_array().unwrap();
    let events = spans
        .iter()
        .filter(|s| !s["upper_projection"].is_null() || !s["return_projection"].is_null())
        .cloned()
        .collect::<Vec<_>>();
    let mut state = None;
    if let (Some(hit), Some(window)) = (next, end) {
        for (k, f) in frames.iter().enumerate() {
            let t = k as f64 * 0.0005;
            if t + 1e-16 < hit.at || t > window.through + 1e-16 {
                continue;
            }
            let exact = advances(i, hit, t);
            let q = DQuat::from_array(
                serde_json::from_value(f["foot"]["orientation"].clone()).unwrap(),
            );
            let c = DVec3::from(
                serde_json::from_value::<[f64; 3]>(f["foot"]["position"].clone()).unwrap(),
            ) + q * ARM;
            let w = DVec3::from(
                serde_json::from_value::<[f64; 3]>(f["foot"]["angular_velocity"].clone()).unwrap(),
            );
            state = Some(
                json!({"at_ns":t*1e9,"com_m":c.distance(exact.com),"attitude_rad":angular_distance(q,exact.q),"spin_rad_s":w.distance(i.r.body.omega(exact.q,exact.l))}),
            );
        }
    }
    let mut first_gate=false;let mut after_upper=Vec::new();
    for s in spans{
        first_gate |= s["opponent_ns"].as_f64().unwrap()>0.;
        if first_gate && s["primary_ns"].as_f64().unwrap()>0.{after_upper.push(s.clone());}
    }
    json!({"calls":spans.len(),"events":events,"positive_pad_after_first_gate":after_upper,"state_after_return":state,"lower_applications":run["applications"]})
}
pub(crate) fn write(path: &Path) -> crate::Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    for i in inputs() {
        let mut cases = vec![];
        for delay in [0, 8, 20000, 750000, 1100000] {
            let g = gate(&i, delay as f64 * 1e-9);
            for gate_mu in [0., 0.35, 0.8] {
                let mu = i.mu.min(gate_mu);
                let xml = enable(&source(&i, &g, gate_mu));
                let ordered = enroll(&xml);
                let planes = faces(&i.r, &g);
                let first = upper(&i, &g, mu).ok();
                let after = first.map(|f| follow(&i, planes, [i.mu, mu], f, WINDOW));
                let next = after.as_ref().and_then(|a| a.next);
                let further = next.map(|f| follow(&i, planes, [i.mu, mu], f, WINDOW));
                let mut controls = vec![];
                if let (Some(upper), Some(second), Some(post)) = (first, next, &further) {
                    let separation = second.at - upper.at;
                    let guard = (post.through - second.at).clamp(0., 0.5e-6) * 0.4;
                    for fraction in [0., 0.375, 0.875] {
                        let lead = (fraction * separation).min(12e-6);
                        let t = upper.at - lead;
                        let dt = ((second.at - t + guard.max(4e-9)) * 1e9).ceil() as u64;
                        for count in [1, 4] {
                            for enabled in [false, true] {
                                controls.push(native(
                                    &i,
                                    &g,
                                    &ordered,
                                    Trial {
                                        start: t,
                                        dt,
                                        calls: count,
                                        enabled,
                                    },
                                )?);
                            }
                        }
                    }
                } else if first.is_none() {
                    controls.push(native(
                        &i,
                        &g,
                        &ordered,
                        Trial {
                            start: i.r.arrival - 10e-9,
                            dt: 200,
                            calls: 1,
                            enabled: true,
                        },
                    )?);
                }
                let mut public = vec![];
                for (enabled, s) in [(false, &xml), (true, &ordered)] {
                    let (run, frames) = measure_gate(&i, s, true)?;
                    public.push(json!({"enabled":enabled,"data":compact(&i,&run,&frames,next,further.as_ref())}));
                }
                cases.push(json!({"delay_ns":delay,"gate_mu":gate_mu,"gate_center0":g.center.to_array(),"gate_velocity":g.velocity.to_array(),"upper":first.map(Crossing::data),"follow":after.as_ref().map(|a|a.reason),"next":next.map(Crossing::data),"later":further.as_ref().and_then(|a|a.next).map(Crossing::data),"post_return_through_ns":further.as_ref().map(|a|a.through*1e9),"post_return_reason":further.as_ref().map(|a|a.reason),"cold":controls,"public":public}));
            }
        }
        rows.push(json!({"id":i.r.id,"mode":MODES[i.mode],"tensor":i.r.body.tensor.to_cols_array(),"primary":law(&i),"pad_velocity":i.r.pad_v.to_array(),"cases":cases}));
    }
    write_json(&path.join("returns.json"), &json!({"donors":rows}))
}
pub(crate) fn cost(path: &Path) -> crate::Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    for (index, i) in inputs().into_iter().enumerate() {
        for delay in [0, 750000, 1100000] {
            let g = gate(&i, delay as f64 * 1e-9);
            let coarse = enable(&source(&i, &g, 0.35));
            let mut worlds = vec![];
            for xml in [coarse.clone(), enroll(&coarse)] {
                let (mut sim, ids) = load(&xml, false)?;
                sim.step_exact(8)?;
                let end = state(backend(&sim), ids)?;
                costed(&xml, end)?;
                worlds.push((xml, end));
            }
            for rep in 0..6 {
                for z in 0..2 {
                    let k = (index + rep + z) % 2;
                    let t = costed(&worlds[k].0, worlds[k].1)?;
                    rows.push(json!({"id":i.r.id,"mode":MODES[i.mode],"delay_ns":delay,"gate_tick":(g.at/0.0005).ceil()as u64,"enabled":k==1,"rep":rep,"wall_ns":t.wall,"cpu_ns":t.cpu}));
                }
            }
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":rows}))
}

pub(crate) fn write_view(path: &Path) -> crate::Result<()> {
    let i = inputs()
        .into_iter()
        .find(|i| i.r.id == 16 && MODES[i.mode] == "slide")
        .ok_or("missing paired fixture")?;
    let g = gate(&i, 0.00075);
    let mu = 0.8;
    let hit = upper(&i, &g, i.mu.min(mu))?;
    let second = follow(&i, faces(&i.r, &g), [i.mu, i.mu.min(mu)], hit, WINDOW)
        .next
        .ok_or("missing lower return")?;
    views::ordered::write(path, &i, &g, mu, hit, second)
}
