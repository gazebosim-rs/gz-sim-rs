// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
pub(super) use super::super::super::two_bearing::registered::native_state;
use super::*;
use gz_physics_multifidelity::FaceBearingReceipt;
pub(super) fn source(i: Input, f: Fixture, tilted: bool, standalone: bool) -> String {
    let s = observations::source_with(
        i,
        Sole {
            shape: f.shape,
            incoming: f.start,
        },
        f.shape.gravity,
        frame(tilted),
        false,
        "normal",
    );
    let existing = "<project_frictionless_sole>true</project_frictionless_sole>";
    let flag = "<follow_frictionless_face>true</follow_frictionless_face>";
    s.replace("A rocking foot is momentarily level as the opposite edge arrives at its frictionless support.", "A yawing articulated sole moves across a finite, uniformly translating frictionless pad.").replace(
        existing,
        if standalone {
            flag.into()
        } else {
            format!("{existing}{flag}")
        }
        .as_str(),
    )
}
fn check_receipt(
    f: Fixture,
    h: &FaceBearingReceipt,
    local_start: f64,
    tilted: bool,
    err: &mut [f64; 7],
) -> Result<()> {
    let frame = frame(tilted);
    let normal = frame.q * DVec3::Z;
    let t0 = local_start + h.start.as_secs_f64();
    let t1 = t0 + h.duration.as_secs_f64();
    let pad_v = frame.q * DVec3::new(0.17, -0.09, 0.22);
    let mut torque = [DVec3::ZERO; 3];
    for (k, t) in [t0, 0.5 * (t0 + t1), t1].into_iter().enumerate() {
        let p = f.pressure(t, false)?;
        let actual = frame.shift + pad_v * t + frame.q * p.center.extend(0.);
        if k != 1 {
            let (pt, patch) = if k == 0 {
                (h.start_pressure_world, h.start_overlap)
            } else {
                (h.end_pressure_world, h.end_overlap)
            };
            err[0] = err[0].max(actual.distance(pt.into()));
            err[1] = err[1].max((patch.area - p.area).abs());
        }
        torque[k] = frame.q * (p.center.extend(0.) - f.state(t).c).cross(DVec3::Z * p.force);
        err[2] = err[2].max((p.force - h.normal_force).abs());
    }
    let j = h.duration.as_secs_f64() * (torque[0] + 4. * torque[1] + torque[2]) / 6.;
    err[3] = err[3].max(j.distance(h.angular_impulse_about_com.into()));
    if h.normal_force <= 0. || DVec3::from(h.normal).distance(normal) > 1e-13 {
        return Err("incorrect finite pressure normal".into());
    }
    Ok(())
}
pub(super) fn actual(
    i: Input,
    f: Fixture,
    physical: &Value,
    horizon: f64,
    tilted: bool,
    standalone: bool,
    save: bool,
) -> Result<(Value, Vec<Value>)> {
    let exit = physical["exit_s"].as_f64();
    let good = physical["mode"] != "unavailable";
    let t = exit.unwrap_or(horizon);
    let last = if good {
        (t / DT).floor() as u64
            + if exit.is_some() {
                if save { 4 } else { 1 }
            } else {
                0
            }
    } else {
        1
    };
    let mut sim = load(&source(i, f, tilted, standalone), true)?;
    let id = ids(&sim)?;
    let mut handoff = None;
    let mut spans = 0;
    let mut errors = [0f64; 7];
    let mut frames = vec![];
    let mut phases = vec![];
    let mut last_end = 0.;
    for tick in 0..=last {
        let (calls, j) = if tick == 0 {
            (0, 0.)
        } else {
            sim.step_exact(1)?;
            if let Some(r) = last_refinement(&sim).filter(|r| r.iteration == tick) {
                let mut duration = Duration::ZERO;
                let mut sum = 0.;
                for h in &r.interval.face_bearings {
                    if !good {
                        return Err("compressive pressure unexpectedly accepted".into());
                    }
                    check_receipt(f, h, r.start_time.as_secs_f64(), tilted, &mut errors)?;
                    duration += h.duration;
                    sum += h.normal_impulse;
                    spans += 1;
                    last_end = (r.start_time + h.start + h.duration).as_secs_f64();
                    if save {
                        phases.push(json!({"tick":tick,"start_s":(r.start_time+h.start).as_secs_f64(),"coverage":h.end_overlap.area/0.072,"force_n":h.normal_force,"sole_cp":h.normalized_sole_center,"world_cp":h.end_pressure_world}));
                    }
                }
                if duration < Duration::from_micros(500) {
                    handoff.get_or_insert(tick);
                } else if (r.requested_pairs[0].normal_impulse - sum).abs() > 1e-11 {
                    return Err("registered face impulse mixed with native duplicate".into());
                }
                (r.interval.native_steps, r.requested_pairs[0].normal_impulse)
            } else {
                handoff.get_or_insert(tick);
                (
                    1,
                    engine(&sim)
                        .rigid_contact_observations()
                        .iter()
                        .filter(|p| [p.a, p.b].contains(&id.foot) && [p.a, p.b].contains(&id.pad))
                        .map(|p| p.normal_impulse)
                        .sum(),
                )
            }
        };
        if good && (tick as f64) * DT <= t {
            let s = native_state(&sim, &id, frame(tilted));
            let oracle = f.state(tick as f64 * DT);
            errors[4] = errors[4].max(s.c.distance(oracle.c));
            errors[5] = errors[5].max(s.v.distance(oracle.v));
            errors[6] = errors[6].max(s.motion.w.distance(oracle.motion.w));
        }
        if save {
            let (shoe, upper, pad) = departure::ingest(engine(&sim), &id);
            frames.push(row(tick, calls, j, (shoe, upper.unwrap(), pad)));
        }
    }
    if good {
        let expected = exit.map(|ts| (ts / DT).ceil() as u64);
        if handoff != expected || errors.iter().any(|v| !v.is_finite() || *v > 1e-9) {
            let mut scratch = load(&source(i, f, tilted, standalone), true)?;
            let mut owner = scratch.world_mut().resource_mut::<PhysicsBackend>();
            let native = owner.downcast_mut::<MultiFidelityBackend>().unwrap();
            let pair = [(id.foot, id.pad)];
            let screen = native.frictionless_face_now(id.foot, id.pad)?;
            let direct = native.advance_box_contacts(
                Duration::from_micros(500),
                1,
                native.ordinary_contact_iterations(),
                gz_physics_multifidelity::BoxContactActions {
                    face_bearings: &pair,
                    ..Default::default()
                },
                None,
            )?;
            return Err(format!("finite face handoff {handoff:?} vs {expected:?}, errors {errors:?}, tilted {tilted}, fixture {:?} spin {:?}, screen {screen}, direct {} spans, Bevy {spans}",f.start.c,f.start.motion.w,direct.interval.face_bearings.len()).into());
        }
    } else if spans != 0 {
        return Err("noncompressive face was committed".into());
    }
    Ok((
        json!({"tilted":tilted,"standalone":standalone,"committed":spans,"first_native_handoff_tick":handoff,"last_supported_endpoint_s":last_end,"errors_cp_area_force_moment_com_velocity_spin":errors,"phases":phases}),
        frames,
    ))
}
pub(super) fn replay(out: &Path, i: Input, f: Fixture, raw: &Value) -> Result<()> {
    let (data, frames) = actual(i, f, raw, 0.1, false, true, true)?;
    let n = super::actual(out, i, f, frames.len() as u64 - 1)?;
    write_trace(out, "registered", frames)?;
    fs::write(out.join("face.sdf"), source(i, f, false, true))?;
    fs::write(
        out.join("public.json"),
        serde_json::to_vec(&json!({"physical":raw,"registered":data,"controls":n}))?,
    )?;
    let lanes = [
        ("ordinary", "One native solve"),
        ("refined", "Eight native solves"),
        ("registered", "Registered finite face pressure"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"A yawing articulated sole hands an overhanging contact back to the native solver","pair_totals_only":true,"capture_ticks":{"first_native_handoff":data["first_native_handoff_tick"]},"lanes":lanes}),
        )?,
    )?;
    Ok(())
}
