// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::super::registered::native_state;
use super::*;
pub(in super::super) fn compare(
    i: Input,
    c: Case,
    physical: &Value,
    frac: f64,
    tilted: bool,
    save: Option<&Path>,
) -> Result<Value> {
    let fr = frame(tilted);
    let f = Fixture {
        start: c.f.state(c.clock - frac),
        ..c.f
    };
    let src=super::super::registered::source(i,f,tilted,true)
        .replace("</follow_frictionless_face>","</follow_frictionless_face><follow_frictionless_deck_edge>true</follow_frictionless_deck_edge>");
    let mut sim = load(&src, true)?;
    let id = ids(&sim)?;
    let reference = Deck::from(f, c.edge);
    let endpoint = physical["duration_s"].as_f64().unwrap() + frac;
    let physical_loss = physical["end"] != "horizon";
    let last = (endpoint / DT).floor() as u64 + u64::from(physical_loss);
    let mut raw = Vec::new();
    let mut handoff = None;
    let mut event = None;
    let mut spans = 0;
    let mut max_domains = 0;
    let mut subdivided = 0;
    let mut edge_end = 0.;
    let mut state = c.start();
    let mut errors = [0f64; 8];
    let mut min_force = f64::INFINITY;
    let mut max_transverse = 0.0f64;
    let mut actual_sum = 0.;
    for tick in 0..=last {
        let t = tick as f64 * DT;
        let mut native = 0;
        let mut impulse = 0.;
        if tick > 0 {
            sim.step_exact(1)?;
            let h = if tick == 1 { DT - frac } else { DT };
            let mut expected_force = 0.;
            let mut dv = DVec3::ZERO;
            let mut dl = DVec3::ZERO;
            if tick == 1 {
                let cp0 = f.pressure(0., false)?;
                let pm = f.pressure(frac * 0.5, false)?;
                let cp1 = f.pressure(frac, false)?;
                let n = DVec3::Z * cp0.force;
                let torque = |v: Load, s: State| (v.center.extend(0.) - s.c).cross(n);
                dl = (torque(cp0, f.state(0.))
                    + 4. * torque(pm, f.state(frac / 2.))
                    + torque(cp1, f.state(frac)))
                    * (frac / 6.);
                expected_force = frac * cp0.force;
                dv = frac * n;
            }
            for _ in 0..8 {
                let (p, j, jv, jw) = reference.advance(state, h / 8.);
                state = p;
                expected_force += j;
                dv += jv;
                dl += jw;
            }
            let report = last_refinement(&sim).filter(|r| r.iteration == tick);
            let (mut vec_j, mut ang_j, mut duration) = (DVec3::ZERO, DVec3::ZERO, Duration::ZERO);
            if let Some(r) = report {
                native = r.interval.native_steps;
                impulse = r.requested_pairs[0].normal_impulse;
                for span in &r.interval.deck_edge_bearings {
                    if span.pad_edge as usize != c.edge {
                        return Err("wrong material edge".into());
                    }
                    if let Some(pair) = span.face_exit_bracket_seconds {
                        let at = (r.start_time + span.start).as_secs_f64() + span.face_seconds;
                        if event.replace(at).is_some()
                            || pair[0] > span.face_seconds
                            || pair[1] < span.face_seconds
                        {
                            return Err("invalid repeated face-to-line switch".into());
                        }
                    }
                    spans += 1;
                    max_domains = max_domains.max(span.certified_time_domains);
                    subdivided += u64::from(span.certified_time_domains > 1);
                    duration += span.duration;
                    edge_end = (r.start_time + span.start + span.duration).as_secs_f64();
                    let j = fr.q.conjugate() * DVec3::from(span.impulse_on_foot);
                    vec_j += j;
                    ang_j += fr.q.conjugate() * DVec3::from(span.angular_impulse_about_com);
                    max_transverse = max_transverse.max(j.truncate().length());
                    min_force = min_force
                        .min(span.end_equivalent_force[0].min(span.end_equivalent_force[1]));
                    if span.normal_force_bounds[0] <= 0.
                        || span.end_normal_force < span.normal_force_bounds[0] - 1e-10
                        || span.end_normal_force > span.normal_force_bounds[1] + 1e-10
                    {
                        return Err("force outside admission bounds".into());
                    }
                }
                for span in &r.interval.face_bearings {
                    duration += span.duration;
                    vec_j += DVec3::Z * span.normal_impulse;
                    ang_j += fr.q.conjugate() * DVec3::from(span.angular_impulse_about_com);
                }
                if duration == Duration::from_micros(500) {
                    let pair = r
                        .interval
                        .pairs
                        .iter()
                        .find(|p| [p.a, p.b].contains(&id.foot) && [p.a, p.b].contains(&id.pad))
                        .ok_or("missing measured vector")?;
                    let sign = if pair.a == id.foot { 1. } else { -1. };
                    let delivered = fr.q.conjugate() * DVec3::from(pair.normal_impulse_on_a) * sign;
                    errors[0] = errors[0].max(delivered.distance(vec_j));
                }
            } else {
                native = 1;
                impulse = engine(&sim)
                    .rigid_contact_observations()
                    .iter()
                    .filter(|p| [p.a, p.b].contains(&id.foot) && [p.a, p.b].contains(&id.pad))
                    .map(|p| p.normal_impulse)
                    .sum();
            }
            if duration != Duration::from_micros(500) {
                handoff.get_or_insert(tick);
            }
            if handoff.is_none() {
                if t > endpoint + 2e-10 {
                    return Err("constrained after first physical loss".into());
                }
                let found = native_state(&sim, &id, fr);
                errors[1] = errors[1].max((impulse - expected_force).abs());
                errors[2] = errors[2].max(vec_j.distance(dv));
                errors[3] = errors[3].max(ang_j.distance(dl));
                errors[4] = errors[4].max(state.c.distance(found.c));
                errors[5] = errors[5].max(state.v.distance(found.v));
                errors[6] = errors[6].max(state.motion.w.distance(found.motion.w));
                let dq = state.motion.q.conjugate() * found.motion.q;
                errors[7] = errors[7].max(dq.xyz().length() * 2.);
            }
            actual_sum += impulse;
        }
        if save.is_some() {
            let (shoe, upper, pad) = departure::ingest(engine(&sim), &id);
            raw.push(row(tick, native, impulse, (shoe, upper.unwrap(), pad)));
        }
    }
    let end = handoff.unwrap_or(last + 1);
    if event.is_none()
        || (event.unwrap() - frac).abs() > 2e-10
        || spans == 0
        || errors.iter().any(|v| !v.is_finite() || *v > 2e-8)
        || min_force < 0.
    {
        return Err(format!("deck transition fraction {frac} frame {tilted}, edge {}, event {event:?}, spans {spans} handoff {handoff:?}, physical {endpoint}, meters/impulses/errors {errors:?}",c.edge).into());
    }
    let data = json!({"positive_spans":spans,"timing_subdivided_spans":subdivided,"max_certified_domains":max_domains,"face_exit_s":event,"fraction_s":frac,"tilted":tilted,"pressure_end_s":endpoint,"qualified_through_s":edge_end,"first_ordinary_tick":end,"first_possible_ordinary_tick":last + u64::from(!physical_loss),"min_equivalent_force_n":min_force,"max_horizontal_tick_impulse_ns":max_transverse,"errors_vector_receipt_scalar_vector_angular_com_v_w_q":errors,"total_ns":actual_sum});
    if let Some(out) = save {
        let controls = super::super::actual(out, i, f, last)?;
        write_trace(out, "registered", raw)?;
        fs::write(out.join("face.sdf"), src)?;
        fs::write(
            out.join("public.json"),
            serde_json::to_vec(
                &json!({"physical":physical,"registered":data,"controls":controls}),
            )?,
        )?;
        let lanes = [
            ("ordinary", "One native solve"),
            ("refined", "Eight native solves"),
            ("registered", "Registered face-to-pad-edge support"),
        ]
        .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
        fs::write(
            out.join("comparison.json"),
            serde_json::to_vec_pretty(
                &json!({"title":"The pressure moves from a finite face to a tipping pad edge","pair_totals_only":true,"lanes":lanes,"capture_ticks":{"face_to_edge":1,"last_qualified":(edge_end/DT).round() as u64,"first_ordinary":end.min(last)}}),
            )?,
        )?;
    }
    Ok(data)
}
