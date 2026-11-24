// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Authored Bevy arrival times, feasible normal/tangent set and native fallback.
use super::super::{
    authored::{record_flight, xml},
    transaction::Frame,
};
use super::*;
use gz_physics_multifidelity::MixedEdgeImpactReceipt;
const AUTHORED: &str = include_str!("../../../../../../examples/multifidelity_mixed_edge.sdf");
fn check(hit: &MixedEdgeImpactReceipt, law: &Answer, rot: DQuat) -> Result<f64> {
    let mut error = 0f64;
    for (k, along) in hit
        .impulses
        .first_longitudinal_range
        .into_iter()
        .enumerate()
    {
        let native = hit
            .split_world(along)
            .ok_or("invalid native edge impulse set")?;
        let truth = law.split(law.range[k]);
        for i in 0..2 {
            error = error.max(DVec3::from(native[i]).distance(rot * truth[i]));
        }
    }
    if error > 1e-11 {
        return Err(format!("native endpoints differ from physical by {error:e} N s").into());
    }
    Ok(error)
}
fn prepare(i: Input, free: bool) -> Input {
    let mut i = i;
    if free {
        let axis = i.seed.state.q * DVec3::X;
        i.edge.inertia -= DMat3::from_cols(axis * axis.x, axis * axis.y, axis * axis.z) * 0.0201;
        i.edge.spin = i.after_w
            - i.edge.inertia.inverse()
                * (i.edge.arm[0].cross(i.desired[0]) + i.edge.arm[1].cross(i.desired[1]));
        i.seed.state.w = i.edge.spin;
    }
    i
}
fn exact(i: Input, frame: Frame, free: bool) -> Result<Value> {
    let src = xml(i, frame, free, frame.q * DVec3::new(0., 0., -9.81), "mixed");
    let mut sim = load(&src, true)?;
    sim.step_exact(1)?;
    let rec = last_refinement(&sim);
    let hit = rec.and_then(|r| r.interval.mixed_edge_impacts.first());
    match (solve(i.edge), hit) {
        (Ok(law), Some(h)) => {
            let rec = rec.unwrap();
            let pair = &rec.requested_pairs[0];
            if !h.elapsed.is_zero() || pair.first_positive_end != Some(Duration::ZERO) {
                return Err("cold mixed impact timing changed".into());
            }
            Ok(
                json!({"admitted":true,"mode":name(law.mode),"calls":rec.interval.native_steps,"endpoints_error_ns":check(h,&law,frame.q)?,"range_ns":h.impulses.first_longitudinal_range,"normal_ns":h.impulses.normal,"pair_scalar_ns":pair.normal_impulse,"last_native_ns":pair.last_native_normal_impulse}),
            )
        }
        (Err(why), None) => {
            let control = xml(
                i,
                frame,
                free,
                frame.q * DVec3::new(0., 0., -9.81),
                if frame.spin == 0. && frame.q == DQuat::IDENTITY {
                    "refined"
                } else {
                    "coupled"
                },
            );
            let mut plain = load(&control, true)?;
            plain.step_exact(1)?;
            // Only an edge-aware authored policy supports the inclined,
            // spinning configuration. Its strictly sliding mode also declines
            // these sticking cases; verify the shared native-only control.
            if last_refinement(&plain).is_some_and(|r| !r.interval.sliding_edge_impacts.is_empty())
            {
                return Err("control applied a separate impulse".into());
            }
            let a = departure::ingest(engine(&sim), &ids(&sim)?);
            let b = departure::ingest(engine(&plain), &ids(&plain)?);
            if a != b {
                return Err("declined mixed projection changed ordinary refined state".into());
            }
            Ok(json!({"admitted":false,"unchanged":true,"reason":why.to_string()}))
        }
        _ => Err("authored native gate disagrees with independent mixed mechanics".into()),
    }
}
fn airborne(mu: f64, yaw: f64, mode: Mode, t: f64, g: DVec3) -> Input {
    let mut i = model(mu, yaw, 0.18, mode, false, [0.48, 0.72]);
    let (x, y) = match mode {
        Mode::BothStick => (0.05, 0.4),
        Mode::Stick(0) => (0.02, 0.08),
        Mode::Stick(_) => (-0.02, 0.08),
    };
    i.edge.velocity = DQuat::from_rotation_z(yaw) * DVec3::new(x, y, -1.27);
    i.edge.spin = DVec3::ZERO;
    i.seed.state.v = i.edge.velocity;
    i.seed.state.w = DVec3::ZERO;
    i.seed.state.c.z -= i.edge.velocity.z * t + 0.5 * g.z * t * t;
    i
}
fn flight(i: Input, frame: Frame, gravity: DVec3, t: f64) -> Result<Value> {
    let mut s = load(&xml(i, frame, false, frame.q * gravity, "mixed"), true)?;
    let ticks = (t / DT).ceil() as u64;
    let mut early = 0.;
    for k in 1..=ticks {
        s.step_exact(1)?;
        let rec = last_refinement(&s).filter(|r| r.iteration == k);
        if let Some((r, h)) =
            rec.and_then(|r| r.interval.mixed_edge_impacts.first().map(|h| (r, h)))
        {
            let a = (r.start_time + h.elapsed).as_secs_f64();
            if early != 0. || k != ticks || (a - t).abs() > 1.001e-9 {
                return Err(format!(
                    "incorrect mixed arrival: early={early} actual={a} expected={t}"
                )
                .into());
            }
            let mut e = i.edge;
            e.velocity += gravity * a;
            let law = solve(e)?;
            let p = &r.requested_pairs[0];
            if p.first_positive_end != Some(h.elapsed) {
                return Err("projected mixed impact missing from pair receipt".into());
            }
            return Ok(
                json!({"mode":name(law.mode),"expected_ns":t*1e9,"actual_ns":(r.start_time+h.elapsed).as_nanos()as u64,"tick":k,"prior_reaction_ns":early,"calls":r.interval.native_steps,"endpoints_error_ns":check(h,&law,frame.q)?,"range_ns":h.impulses.first_longitudinal_range,"normal_ns":h.impulses.normal,"pair_scalar_ns":p.normal_impulse,"last_native_ns":p.last_native_normal_impulse}),
            );
        } else if let Some(r) = rec {
            early += r
                .interval
                .pairs
                .iter()
                .map(|p| p.normal_impulse)
                .sum::<f64>();
        } else {
            early += engine(&s)
                .rigid_contact_observations()
                .iter()
                .map(|p| p.normal_impulse)
                .sum::<f64>();
        }
    }
    Err(format!("no mixed edge in Bevy flight at {t}, prior measured {early}").into())
}
pub(in super::super::super::super) fn run(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let base = Frame {
        q: DQuat::IDENTITY,
        shift: DVec3::ZERO,
        spin: 0.,
    };
    let moved = Frame {
        q: DQuat::from_euler(glam::EulerRot::ZYX, -0.4, 0.34, -0.18),
        shift: DVec3::new(-1.1, 1.8, -0.2),
        spin: 0.,
    };
    let mut authored = load(AUTHORED, true)?;
    authored.step_exact(1)?;
    let sample = last_refinement(&authored).ok_or("example SDF did not select touchdown")?;
    let [h] = sample.interval.mixed_edge_impacts.as_slice() else {
        return Err("example SDF did not produce one mixed impact".into());
    };
    if h.mode != gz_physics::EdgeStickMode::BothStick
        || (h.impulses.first_longitudinal_range[1] - 0.16128).abs() > 1e-12
    {
        return Err("unexpected public authored example".into());
    }
    let mut exacts = vec![];
    let mut flights = vec![];
    for mode in [Mode::BothStick, Mode::Stick(0), Mode::Stick(1)] {
        for mu in [0.08, 0.2, 0.35, 0.5, 0.65, 0.8] {
            for (roll, inv, load) in [(0.18, false, [0.48, 0.72]), (-0.18, true, [0.72, 0.48])] {
                for yaw in [0., 0.47, 1.19, 1.95] {
                    for free in [false, true] {
                        let i = prepare(model(mu, yaw, roll, mode, inv, load), free);
                        for (frame, label) in [
                            (base, "base"),
                            (Frame { spin: 1.3, ..moved }, "inclined spinning"),
                        ] {
                            let r = exact(i, frame, free).map_err(|e| {
                                format!("{} µ={mu} free={free} {label}: {e}", name(mode))
                            })?;
                            exacts.push(json!({"mu":mu,"mode":name(mode),"free":free,"yaw":yaw,"roll":roll,"frame":label,"result":r}));
                        }
                    }
                }
            }
        }
        for ns in [
            0.25, 1.25, 62500.37, 249999.4, 499999.34, 500001.61, 638197.61, 2138197.72,
        ] {
            let t = ns * 1e-9;
            for g in [DVec3::ZERO, DVec3::new(0.35, -0.23, -9.81)] {
                for yaw in [0., 0.7] {
                    let i = airborne(0.2, yaw, mode, t, g);
                    for (frame, label) in [(base, "base"), (moved, "inclined")] {
                        let r = flight(i, frame, g, t).map_err(|e| {
                            format!("{} ns={ns} yaw={yaw} g={g:?} {label}: {e}", name(mode))
                        })?;
                        if r["mode"] != name(mode) {
                            return Err(format!(
                                "unexpected mode on flight {:?} mode {}: {r}",
                                mode,
                                name(mode)
                            )
                            .into());
                        }
                        flights.push(json!({"mode":name(mode),"yaw":yaw,"g":g.to_array(),"frame":label,"result":r}));
                    }
                }
            }
        }
    }
    fs::write(out.join("tangent.json"), serde_json::to_vec(&exacts)?)?;
    fs::write(out.join("flights.json"), serde_json::to_vec(&flights)?)?;
    let t = 0.00213819765;
    let g = DVec3::new(0.35, -0.23, -9.81);
    record_flight(out, airborne(0.2, 0.5, Mode::Stick(0), t, g), g, true)
}
