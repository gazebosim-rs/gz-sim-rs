// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::super::super::super::super::transaction::Frame;
use super::super::super::{transaction::frame, two_bearing::registered::native_state};
use super::*;
use gz_physics_multifidelity::{BoxContactActions, ContactIntervalReceipt};

fn source(i: Input, d: &Donor, x: EdgeSupportState, f: Frame, mode: &str) -> String {
    let sole = Sole {
        shape: d.path.shape,
        incoming: physical(x),
    };
    let src = observations::source_with(
        i,
        sole,
        sole.shape.gravity,
        f,
        false,
        if mode == "transfer" { "normal" } else { mode },
    );
    if mode == "transfer" {
        src.replace(
            "</project_frictionless_sole>",
            "</project_frictionless_sole><follow_frictionless_edge>true</follow_frictionless_edge>",
        )
    } else {
        src
    }
}
fn site(raw: u8) -> usize {
    usize::from(2 * (raw & 1) + (raw >> 1 & 1))
}
fn check_receipt(
    r: &ContactIntervalReceipt,
    d: &Donor,
    start: EdgeSupportState,
    phase: f64,
) -> Result<Value> {
    let [event] = r.sole_transfers.as_slice() else {
        return Err(format!(
            "expected one sole transfer, got {} face {} smooth {}, {} native calls",
            r.sole_transfers.len(),
            r.face_pressure_impacts.len(),
            r.two_bearing_edges.len(),
            r.native_steps
        )
        .into());
    };
    if r.face_pressure_impacts.as_slice() != [event.pressure] || !r.edge_impacts.is_empty() {
        return Err("inconsistent face attribution".into());
    }
    let p = event.pressure;
    let t = p.physical_elapsed_seconds;
    if (t - phase).abs() > 2e-11
        || p.elapsed.as_nanos() != (t * 1e9).ceil() as u128
        || !(event.event_bracket_seconds[0]..=event.event_bracket_seconds[1]).contains(&t)
    {
        return Err(format!("clock {t:e} vs {phase:e}, public {:?}", p.elapsed).into());
    }
    let mut pre = [0.; 4];
    let mut post = [0.; 4];
    for (dest, span) in [(&mut pre, event.before), (&mut post, event.after)] {
        for k in 0..2 {
            if span.normal_force_bounds[k][0] <= 0. {
                return Err("uncertified compression".into());
            }
            dest[site(span.sole_vertices[k])] += span.normal_impulses[k];
        }
    }
    for s in &r.two_bearing_edges {
        let dest = if s.start + s.duration <= event.start {
            &mut pre
        } else if s.start >= event.start + event.duration {
            &mut post
        } else {
            return Err("overlapping support segments".into());
        };
        for k in 0..2 {
            dest[site(s.sole_vertices[k])] += s.normal_impulses[k];
        }
    }
    let (pre0, _) = integrate(d.path, physical(start).motion, phase)?;
    let (post0, _) = integrate(d.follow, d.impact.after.motion, DT - phase)?;
    let (Sites::Two(a), Sites::Two(b)) = (d.path.sites, d.follow.sites) else {
        unreachable!()
    };
    let expected = |keys: [usize; 2], j: [f64; 2]| {
        let mut values = [0.; 4];
        values[keys[0]] = j[0];
        values[keys[1]] = j[1];
        values
    };
    let err = |(l, r): (f64, f64)| (l - r).abs();
    let loads = pre
        .into_iter()
        .zip(expected(a, pre0))
        .chain(post.into_iter().zip(expected(b, post0)))
        .map(err)
        .fold(0., f64::max);
    let pressure = (d.impact.total - p.normal_impulse).abs();
    let cop = d
        .impact
        .center
        .unwrap()
        .distance(DVec2::from(p.normalized_center));
    let rate = pre.iter().chain(&post).sum::<f64>() + p.normal_impulse;
    let pair = r
        .pairs
        .iter()
        .find(|v| [v.a, v.b].contains(&p.foot) && [v.a, v.b].contains(&p.support))
        .ok_or("missing pressure in public pair")?;
    if loads.max(pressure).max(cop) > 1e-9
        || (pair.normal_impulse - rate).abs() > 1e-11
        || !pair.active_at_end
        || pair.first_positive_end != Some(Duration::ZERO)
    {
        return Err(format!(
            "subspan loads {loads:e} hit {pressure:e} cop {cop:e} public {pair:?} expected {rate:e}"
        )
        .into());
    }
    Ok(
        json!({"time_s":t,"time_error_s":(t-phase).abs(),"digital_ns":p.elapsed.as_nanos() as u64,"event_bracket_s":event.event_bracket_seconds,"pressure_ns":p.normal_impulse,"pressure_error_ns":pressure,"cop_error":cop,"before_equivalent_ns":pre,"after_equivalent_ns":post,"max_endpoint_error_ns":loads,"begin_s":event.start.as_secs_f64(),"public_normal_ns":pair.normal_impulse,"calls":r.native_steps}),
    )
}
pub(super) fn check(i: Input, d: &Donor, start: EdgeSupportState, phase: f64) -> Result<Value> {
    let mut rows = vec![];
    for tilt in [false, true] {
        let f = frame(tilt);
        let src = source(i, d, start, f, "transfer");
        let mut sim = load(&src, true)?;
        let id = ids(&sim)?;
        sim.step_exact(1)?;
        let r = last_refinement(&sim).ok_or("missing registered initial bearing")?;
        let mut report = check_receipt(&r.interval, d, start, phase)
            .map_err(|e| format!("authored tilt {tilt} phase {phase:e}: {e}"))?;
        let (_, motion) = integrate(d.follow, d.impact.after.motion, DT - phase)?;
        let expected = d.follow.state(DT - phase, motion);
        let delta = relative(native_state(&sim, &id, f), expected);
        if delta.iter().any(|v| !v.is_finite() || *v > 1e-9) {
            return Err(format!("Bevy public endpoint {delta:?}").into());
        }
        report["state_errors"] = json!(delta);
        let mut native = load(&src, true)?;
        let ids = ids(&native)?;
        let mut handle = native.world_mut().resource_mut::<PhysicsBackend>();
        let engine = handle.downcast_mut::<MultiFidelityBackend>().unwrap();
        let pair = [(ids.foot, ids.pad)];
        let mut grids = vec![];
        // Exercise a deliberately different native subdivision from the authored scene.
        let calls = if r.interval.native_steps == 1 { 8 } else { 1 };
        let alt = engine.advance_box_contacts(
            Duration::from_micros(500),
            calls,
            4.into(),
            BoxContactActions {
                two_bearing_edges: &pair,
                frictionless_sole_impacts: &pair,
                ..Default::default()
            },
            None,
        )?;
        grids.push(
            check_receipt(&alt.interval, d, start, phase)
                .map_err(|e| format!("alternate tilt {tilt} phase {phase:e}: {e}"))?,
        );
        report["alternate_grid"] = grids.into();
        report["tilted"] = tilt.into();
        rows.push(report);
    }
    Ok(rows.into())
}
pub(super) fn replay(out: &Path, i: Input, d: &Donor) -> Result<()> {
    let phase = 0.000213819760;
    let (start, _, _) = d.cut(phase)?;
    let mut summary = vec![];
    for mode in ["ordinary", "refined", "transfer"] {
        let src = source(i, d, start, frame(false), mode);
        if mode == "transfer" {
            fs::write(out.join("transfer.sdf"), &src)?;
        }
        let mut sim = load(&src, true)?;
        let id = ids(&sim)?;
        let mut frames = vec![];
        let mut events = vec![];
        for tick in 0..=32 {
            let (calls, normal) = if tick == 0 {
                (0, 0.)
            } else {
                sim.step_exact(1)?;
                if let Some(r) = last_refinement(&sim).filter(|r| r.iteration == tick) {
                    for e in &r.interval.sole_transfers {
                        events.push(json!({"tick":tick,"event_time_us":(r.start_time.as_secs_f64()+e.pressure.physical_elapsed_seconds)*1e6,"event_ns":e.pressure.normal_impulse,"cop":e.pressure.normalized_center,"prior":e.before.normal_impulses,"next":e.after.normal_impulses,"public_ns":r.requested_pairs[0].normal_impulse}));
                    }
                    (r.interval.native_steps, r.requested_pairs[0].normal_impulse)
                } else {
                    let n = engine(&sim);
                    (
                        1,
                        n.rigid_contact_observations()
                            .iter()
                            .filter(|p| {
                                [p.a, p.b].contains(&id.foot) && [p.a, p.b].contains(&id.pad)
                            })
                            .map(|p| p.normal_impulse)
                            .sum(),
                    )
                }
            };
            let (f, u, p) = departure::ingest(engine(&sim), &id);
            frames.push(json!({"tick":tick,"time_us":tick*500,"native_steps":calls,"solver_iterations":4,"selected":calls>1,"weak_start":false,"secondary_start":false,"normal_impulse_ns":normal,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),"gap_m":super::super::super::native::contact_gaps(f,p).into_iter().fold(f64::INFINITY,f64::min)}));
        }
        summary.push(
            json!({"mode":mode,"events":events,"first_public_ns":frames[1]["normal_impulse_ns"]}),
        );
        fs::write(
            out.join(format!("traces/{mode}.json")),
            serde_json::to_vec(
                &json!({"geometry":{"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.41,0.41,0.06]},"frames":frames}),
            )?,
        )?;
    }
    let lanes = [
        ("ordinary", "Ordinary native solve"),
        ("refined", "Predictive refinement"),
        ("transfer", "Registered sole transfer"),
    ]
    .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
    fs::write(
        out.join("comparison.json"),
        serde_json::to_vec_pretty(
            &json!({"title":"A registered sole transfers its load to the opposite edge during one public step","pair_totals_only":true,"capture_ticks":{"registered_transfer":1},"lanes":lanes}),
        )?,
    )?;
    fs::write(out.join("public.json"), serde_json::to_vec(&summary)?)?;
    Ok(())
}
pub(super) fn cost(out: &Path, i: Input, d: &Donor) -> Result<()> {
    use std::{hint::black_box, time::Instant};
    let mut rows = vec![];
    let arc = edge(d.path)?;
    let model = sole(d.path);
    let Sites::Two(sites) = d.path.sites else {
        unreachable!()
    };
    for rep in 0..72 {
        let mut row = json!({"repeat":rep});
        for k in 0..4 {
            let phase = [0.000213819760, 0.002213819760][(k + rep) % 2];
            let native = (k / 2 + rep / 2) % 2 != 0;
            let start = arc.sample(d.arrival.time - phase, 0.0000625)?.state;
            let key = format!(
                "{}_{}_ns",
                if native { "bevy" } else { "mechanics" },
                if phase < DT { "crossing" } else { "smooth" }
            );
            let measured = if native {
                // A complete warm public tick, starting one public step before
                // the timed cut. Scene creation and initialization are untimed.
                let warm = arc.sample(d.arrival.time - phase - DT, 0.0000625)?.state;
                let mut sim = load(&source(i, d, warm, frame(false), "transfer"), true)?;
                sim.step_exact(1)?;
                let begin = Instant::now();
                sim.step_exact(1)?;
                let ns = begin.elapsed().as_nanos() as u64;
                let r = last_refinement(&sim).ok_or("missing warm receipt")?;
                let count = r.interval.sole_transfers.len();
                if count != usize::from(phase < DT) || r.iteration != 2 {
                    return Err("warm benchmark membership changed".into());
                }
                ns
            } else {
                let begin = Instant::now();
                for _ in 0..8 {
                    let advance = black_box(model)
                        .advance(black_box(start), sites, DT)?
                        .ok_or("unresolved cost fixture")?;
                    if black_box(advance).transfer.is_some() != (phase < DT) {
                        return Err("incorrect crossing benchmark".into());
                    }
                }
                (begin.elapsed().as_nanos() / 8) as u64
            };
            row[key] = measured.into();
        }
        rows.push(row);
    }
    fs::write(out.join("cost.json"), serde_json::to_vec(&rows)?)?;
    Ok(())
}
