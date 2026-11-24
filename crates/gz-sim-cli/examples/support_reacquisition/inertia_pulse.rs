// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare fixed controls and a live, guarded post-arrival pulse over two disjoint sets of cuts.
use super::*;
use gz_sim_scene::primary_contact::{last_box_timing, set_refined_observer};
use std::sync::{Arc, Mutex};
const MODES: [&str; 6] = ["baseline", "16ns", "64ns", "256ns", "1000ns", "measured"];
pub(super) fn xml(r: &Physical, calls: u32, mode: &str) -> (Profile, String) {
    let (p, text) = cubic::configured(r, calls, true);
    if mode == "baseline" {
        (p, text)
    } else {
        (p,text.replace("</refine_rotation_timing>",&format!("</refine_rotation_timing><rotation_contact_pulse>{mode}</rotation_contact_pulse>")))
    }
}
pub(super) fn ledger(r: &Physical, text: &str) -> Result<Value> {
    let (mut sim, ids) = sdf_features::load_finite_box(text, false)?;
    let samples = Arc::new(Mutex::new(Vec::new()));
    let shared = samples.clone();
    set_refined_observer(&mut sim, move |_, s| {
        let cut = gyro::Cut {
            foot: s.state(ids.foot).unwrap(),
            pad: s.state(ids.pad).unwrap(),
            ns: s.elapsed.as_nanos() as u64,
        };
        let load = s
            .contact_patches()
            .iter()
            .filter(|p| (p.a, p.b) == (ids.foot, ids.pad) || (p.b, p.a) == (ids.foot, ids.pad))
            .flat_map(|p| &p.points)
            .map(|p| p.normal_impulse)
            .sum::<f64>();
        shared.lock().unwrap().push((cut, load));
    })?;
    let mut pulses = vec![];
    let mut loads = vec![];
    let mut last = None;
    for tick in 0..8u64 {
        let start = gyro::Cut::bodies(backend(&sim), ids, 0);
        sim.step_exact(1)?;
        let mut cuts = std::mem::take(&mut *samples.lock().unwrap());
        if cuts.is_empty() {
            let e = backend(&sim);
            cuts.push((
                gyro::Cut::bodies(e, ids, 500_000),
                sdf::unrefined(e, ids).0.into_iter().sum(),
            ));
        }
        if let Some(live) = last_box_timing(&sim).filter(|x| x.sim_time == sim.elapsed()) {
            for p in &live.timing.pulses {
                let cut = if p.cut_ns == 0 {
                    start
                } else {
                    cuts.iter()
                        .find(|(x, _)| x.ns == p.cut_ns)
                        .ok_or("pulse cut missing")?
                        .0
                };
                let mut oracle = None;
                if let Some(span) = p.applied_ns {
                    assert!(cuts.iter().any(|(x, _)| x.ns == p.cut_ns + span));
                    let (ix, sec) = gyro::Ballistic::new(r.body, cut)
                        .crossing(span as f64 * 1e-9)?
                        .ok_or("pulse missing physical arrival")?;
                    assert_eq!(ix, usize::from(p.corner));
                    assert!(sec * 1e9 <= p.arrival_by_ns.unwrap() as f64 + 0.001);
                    oracle = Some(sec * 1e9);
                }
                pulses.push(json!({"tick":tick+1,"cut_ns":p.cut_ns,"corner":p.corner,"arrives_by_ns":p.arrival_by_ns,"oracle_local_ns":oracle,"applied_ns":p.applied_ns,"decline":p.decline.map(|d|format!("{d:?}"))}));
            }
        }
        for (cut, load) in cuts {
            if load > 1e-12 {
                let t = (tick * 500_000 + cut.ns) as f64 * 1e-9;
                let want = r.state_branch(t, r.post);
                let spin = DVec3::from(cut.foot.angular_velocity);
                let q = DQuat::from_array(cut.foot.orientation);
                let l = r.body.momentum(q, spin);
                loads.push(json!({"end_ns":tick*500_000+cut.ns,"normal_ns":load,"angular_momentum_error_nms":l.distance(r.post.l),
                    "spin_error_rad_s":spin.distance(want.foot.angular_velocity.into())}));
                last = Some((cut, t, l, q));
            }
        }
    }
    let (cut, t, l, mut q) = last.ok_or("no positive native impact")?;
    let n = ((WINDOW - t) / RK_H).ceil().max(1.) as usize;
    let h = (WINDOW - t) / n as f64;
    for _ in 0..n {
        q = advance_q(r.body, q, l, h);
    }
    let w = r.body.omega(q, l);
    let exact = r.state(WINDOW);
    let native = backend(&sim).state(ids.foot).unwrap();
    let center = DVec3::from(cut.foot.position)
        + DQuat::from_array(cut.foot.orientation) * ARM
        + DVec3::from(cut.foot.velocity) * (WINDOW - t);
    Ok(
        json!({"pulses":pulses,"loads":loads,"last_load_ns":(t*1e9).round()as u64,
        "free_propagation_spin_residual_rad_s":w.distance(native.angular_velocity.into()),
        "oracle_from_outgoing_spin_error_rad_s":w.distance(exact.foot.angular_velocity.into()),
        "oracle_from_outgoing_com_error_m":center.distance(exact.com())}),
    )
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut catalog = vec![];
    let mut cases = vec![];
    for id in 0..2 * CASES {
        let r = Physical::new(id);
        let cert = r.certify();
        catalog.push(json!({"id":id,"excluded":cert.reason}));
        if cert.reason.is_some() {
            continue;
        }
        let mut trials = vec![];
        for calls in [4, 8, 16] {
            for mode in MODES {
                let (p, text) = xml(&r, calls, mode);
                let (row, frames, _) = scene::run(&r, false, p, &text)?;
                let audit = ledger(&r, &text)?;
                if id == 153 && calls == 8 && matches!(mode, "baseline" | "measured") {
                    separated::single_rigid_trace(path, mode, &frames)?;
                    fs::write(path.join(format!("{mode}.sdf")), text)?;
                }
                trials.push(json!({"calls":calls,"mode":mode,"native_calls":row["native_calls"],"metrics":row["metrics"],"first_ns":row["first_positive_end_ns"],"early_ns":row["preimpact_measured_ns"],"ledger":audit}));
            }
        }
        if id == 153 {
            separated::single_rigid_trace(path, "analytic", &r.trace())?;
        }
        cases.push(json!({"id":id,"held_out":id>=CASES,"arrival_ns":(r.arrival*1e9).round()as u64,"j_ns":r.j,"corner":r.corner,"trials":trials}));
    }
    write_json(
        &path.join("pulse.json"),
        &json!({"catalog":catalog,"cases":cases,"view_id":153}),
    )?;
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"title":"Resolving the first native contact pulse","lanes":[
        {"label":"baseline","name":"Four-microsecond contact pulse","file":"traces/baseline.json"},
        {"label":"measured","name":"Pulse from measured arrival bound","file":"traces/measured.json"},
        {"label":"analytic","name":"Independent Euler flight and impact","file":"traces/analytic.json","analytical":true}]}),
    )?;
    adverse(path)
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    for id in CASES..2 * CASES {
        let r = Physical::new(id);
        if r.certify().reason.is_some() {
            continue;
        }
        let mut fixtures = vec![];
        for mode in MODES {
            let (p, text) = xml(&r, 8, mode);
            let end = scene::run(&r, false, p, &text)?.2;
            scene::measure(&text, false, end)?;
            fixtures.push((mode, text, end));
        }
        let mut times: [Vec<[u64; 8]>; 6] = Default::default();
        for rep in 0..6 {
            for pos in 0..6 {
                let i = (pos + rep) % 6;
                let i = if id % 2 == 0 { i } else { 5 - i };
                times[i].push(scene::measure(&fixtures[i].1, false, fixtures[i].2)?);
            }
        }
        for ((mode, _, _), raw) in fixtures.into_iter().zip(times) {
            rows.push(json!({"id":id,"calls":8,"mode":mode,"primary_tick":(r.arrival*2000.).ceil()as u64,"tick_ns":raw}));
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":rows}))
}
fn adverse(path: &Path) -> Result<()> {
    use gz_physics_multifidelity::{BoxContactActions, RotationContactPulse};
    let mut data = vec![];
    for family in 0..6 {
        for phase in 0..8 {
            let r = cubic::adverse_fixture(family, phase);
            for calls in [4, 8, 16] {
                for mode in MODES {
                    let (_, text) = xml(&r, calls, mode);
                    let (mut sim, ids) = sdf_features::load_finite_box(&text, false)?;
                    let mut phy = sim.world_mut().resource_mut::<PhysicsBackend>();
                    let e = phy.downcast_mut::<MultiFidelityBackend>().unwrap();
                    let pair = [(ids.foot, ids.pad)];
                    let config = match mode {
                        "baseline" => None,
                        "measured" => Some(RotationContactPulse::Measured),
                        m => Some(RotationContactPulse::Fixed(
                            m.trim_end_matches("ns").parse()?,
                        )),
                    };
                    let request = config.map(|m| (pair[0], m));
                    let a = BoxContactActions {
                        fresh_features: &pair,
                        prospective_timing: &pair,
                        guard_separated_flight: &pair,
                        integrate_free_rotation: &pair,
                        refine_rotation_timing: &pair,
                        rotation_contact_pulses: request.as_slice(),
                        ..Default::default()
                    };
                    let out = e.advance_box_contacts(
                        Duration::from_micros(500),
                        calls,
                        4.into(),
                        a,
                        None,
                    )?;
                    let t = out.timing.unwrap();
                    assert_eq!(
                        t.durations.iter().sum::<Duration>(),
                        Duration::from_micros(500)
                    );
                    assert_eq!(t.durations.len(), calls as usize);
                    if family <= 2 || family == 4 {
                        assert!(t.crossings.is_empty() && t.pulses.is_empty());
                    }
                    data.push(json!({"family":(["face","edge","subroundoff","near_edge","grazing","isolated"][family]),"phase":phase,"calls":calls,"mode":mode,
                        "witnesses":t.crossings.len(),"applied":t.pulses.iter().filter(|p|p.applied_ns.is_some()).count(),
                        "declines":t.pulses.iter().filter_map(|p|p.decline.map(|d|format!("{d:?}"))).collect::<Vec<_>>()}));
                }
            }
        }
    }
    write_json(&path.join("adverse.json"), &json!({"rows":data}))
}
