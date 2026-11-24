// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare a measured free rigid body's next corner crossing to its analytical initial-value problem.
use super::*;
use gz_physics::BodyState;
use std::sync::{Arc, Mutex};
#[derive(Clone, Copy)]
pub(super) struct Cut {
    pub(super) foot: BodyState,
    pub(super) pad: BodyState,
    pub(super) ns: u64,
}
impl Cut {
    pub(super) fn bodies(e: &MultiFidelityBackend, ids: Bodies, ns: u64) -> Self {
        Self {
            foot: e.state(ids.foot).unwrap(),
            pad: e.state(ids.pad).unwrap(),
            ns,
        }
    }
    fn serialize(self) -> Value {
        json!({"ns":self.ns,"foot":Body::from(self.foot),"pad":Body::from(self.pad)})
    }
    fn com(self) -> DVec3 {
        DVec3::from(self.foot.position) + DQuat::from_array(self.foot.orientation) * ARM
    }
}
pub(super) struct Ballistic {
    tensor: ReferenceBody,
    normal: DVec3,
    center: DVec3,
    relative: DVec3,
    q: DQuat,
    w: DVec3,
    omega: f64,
    alpha: f64,
}
impl Ballistic {
    pub(super) fn new(b: ReferenceBody, cut: Cut) -> Self {
        let q = DQuat::from_array(cut.foot.orientation);
        let n = DQuat::from_array(cut.pad.orientation) * DVec3::Z;
        let w = q.conjugate() * DVec3::from(cut.foot.angular_velocity);
        let l = b.tensor * w;
        let omega = (w.dot(l) / b.lower).sqrt().min(l.length() / b.lower);
        Self {
            tensor: b,
            normal: n,
            center: cut.com() - DVec3::from(cut.pad.position) - n * PAD_HALF[2],
            relative: DVec3::from(cut.foot.velocity) - DVec3::from(cut.pad.velocity),
            q,
            w,
            omega,
            alpha: 0.5 * (b.upper - b.lower) / b.lower * omega.powi(2),
        }
    }
    /// The oracle integrates explicit body-frame spin plus attitude; independently
    /// bound all corners on each 2 µs bracket before accepting its first root.
    pub(super) fn crossing(&self, horizon: f64) -> Result<Option<(usize, f64)>> {
        let mut q = self.q;
        let mut w = self.w;
        let mut time = 0.;
        let arms = corners().map(|p| p - ARM);
        let gap =
            |q: DQuat, t: f64, r: DVec3| (self.center + self.relative * t + q * r).dot(self.normal);
        while time < horizon {
            let h = (horizon - time).min(2e-6);
            let (end, ew) = advance_euler(self.tensor, q, w, h);
            let mut events = vec![];
            for (i, &r) in arms.iter().enumerate() {
                let z = gap(q, time, r);
                let zend = gap(end, time + h, r);
                if z <= 0. {
                    events.push((i, time));
                    continue;
                }
                let u = (self.relative + (q * w).cross(q * r)).dot(self.normal);
                let amax = (self.alpha + self.omega * self.omega) * r.length();
                if z + u.min(0.) * h - 0.5 * amax * h * h > 1e-14 || u - amax * h > 0. {
                    continue;
                }
                if u + amax * h >= 0. {
                    return Err(
                        format!("uncertain first free corner root, gap {z} rate {u}").into(),
                    );
                }
                if zend > 0. {
                    continue;
                }
                let (mut lo, mut hi) = (0., h);
                for _ in 0..43 {
                    let mid = (lo + hi) * 0.5;
                    let qm = advance_euler(self.tensor, q, w, mid).0;
                    if gap(qm, time + mid, r) > 0. {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                events.push((i, time + (lo + hi) * 0.5));
            }
            if let Some(event) = events.into_iter().min_by(|a, b| a.1.total_cmp(&b.1)) {
                return Ok(Some(event));
            }
            q = end;
            w = ew;
            time += h;
        }
        Ok(None)
    }
}
fn actual(r: &Physical, text: &str) -> Result<Value> {
    let (mut sim, ids) = sdf_features::load_finite_box(text, false)?;
    let mut before = Cut::bodies(backend(&sim), ids, 0);
    let collected = Arc::new(Mutex::new(Vec::new()));
    let share = collected.clone();
    gz_sim_scene::primary_contact::set_refined_observer(&mut sim, move |t, step| {
        let ns = t.start_time.as_nanos() as u64 + step.elapsed.as_nanos() as u64;
        let cut = Cut {
            foot: step.state(ids.foot).unwrap(),
            pad: step.state(ids.pad).unwrap(),
            ns,
        };
        let load = step
            .contact_patches()
            .into_iter()
            .filter(|p| (p.a, p.b) == (ids.foot, ids.pad) || (p.b, p.a) == (ids.foot, ids.pad))
            .flat_map(|p| p.points)
            .map(|x| x.normal_impulse)
            .sum::<f64>();
        share.lock().unwrap().push((cut, load));
    })?;
    for _ in 0..8 {
        sim.step_exact(1)?;
        let end = sim.elapsed().as_nanos() as u64;
        let mut samples = std::mem::take(&mut *collected.lock().unwrap());
        if samples.is_empty() {
            let e = backend(&sim);
            let load = sdf::unrefined(e, ids).0.into_iter().sum();
            samples.push((Cut::bodies(e, ids, end), load));
        }
        for (cut, load) in samples {
            if load > 1e-12 {
                let crossing = Ballistic::new(r.body, before).crossing(0.001)?;
                let reference = r.state_branch(before.ns as f64 * 1e-9, r.pre);
                let q = DQuat::from_array(before.foot.orientation);
                let w = DVec3::from(before.foot.angular_velocity);
                let l = r.body.momentum(q, w);
                let l0 = r.pre.l;
                let e0 = 0.5 * r.body.omega(r.q, l0).dot(l0);
                return Ok(
                    json!({"begin":before.serialize(),"end":cut.serialize(),"first_native_impulse_ns":load,
                    "first_end_ns":cut.ns,"first_loaded_start_ns":before.ns,"start_gap_m":lowest_gap(before.foot,before.pad),
                    "cut_momentum_drift_nms":l.distance(l0),"cut_energy_drift_j":0.5*w.dot(l)-e0,
                    "cut_orientation_error_rad":angular_distance(q,DQuat::from_array(reference.foot.orientation)),
                    "measured_crossing_ns":crossing.map(|(_,t)|before.ns as f64+1e9*t),
                    "measured_corner":crossing.map(|(i,_)|i)}),
                );
            }
            before = cut;
        }
    }
    Err("missing rigid landing".into())
}
fn ghost(r: &Physical, calls: u32) -> Result<Value> {
    let (_, text) = separated::configured(r, false, 8, false);
    let (mut sim, ids) = sdf_features::load_finite_box(&text, false)?;
    let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
    let e = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
    e.set_self_collision_group(ids.foot, Some(901))?;
    e.set_self_collision_group(ids.pad, Some(901))?;
    let elapsed = (r.arrival * 2000.).floor() as u64 * 500_000;
    for _ in 0..elapsed / 500_000 {
        let receipts = e.step_contact_interval(Duration::from_micros(500), calls, 4)?;
        assert!(receipts.pairs.iter().all(|p| p.normal_impulse == 0.));
    }
    let cut = Cut::bodies(e, ids, elapsed);
    let q = DQuat::from_array(cut.foot.orientation);
    let w = DVec3::from(cut.foot.angular_velocity);
    let l = r.body.momentum(q, w);
    let e0 = r.pre.l.dot(r.body.omega(r.q, r.pre.l)) * 0.5;
    let exact = r.state(elapsed as f64 * 1e-9);
    let crossing = Ballistic::new(r.body, cut).crossing(0.001)?;
    Ok(
        json!({"calls":calls,"cut":cut.serialize(),"remaining_reference_ns":r.arrival*1e9-elapsed as f64,
        "measured_crossing_ns":crossing.map(|(_,dt)|elapsed as f64+dt*1e9),"measured_corner":crossing.map(|(i,_)|i),
        "orientation_error_rad":angular_distance(q,DQuat::from_array(exact.foot.orientation)),
        "angular_momentum_error_nms":l.distance(r.pre.l),"energy_error_j":0.5*w.dot(l)-e0,
        "com_error_m":cut.com().distance(exact.com()),"angular_velocity_error_rad_s":w.distance(exact.foot.angular_velocity.into())}),
    )
}
pub(crate) fn diagnose(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut cases = vec![];
    for id in 0..CASES {
        let r = Physical::new(id);
        if r.certify().reason.is_some() {
            continue;
        }
        let (_, xml) = separated::configured(&r, false, 8, true);
        let mut controls = vec![];
        for calls in [1, 2, 4, 8, 16] {
            controls.push(ghost(&r, calls)?);
        }
        let (sim, ids) = sdf_features::load_finite_box(&xml, false)?;
        let first = Cut::bodies(backend(&sim), ids, 0);
        let oracle = Ballistic::new(r.body, first)
            .crossing(0.0032)?
            .ok_or("missing analytical crossing")?;
        assert_eq!(oracle.0, r.corner);
        assert!(
            (r.arrival - oracle.1).abs() < 3e-12,
            "cold analytical crossing"
        );

        cases.push(
            json!({"id":id,"reference_ns":(r.arrival*1e9).round()as u64,"reference_corner":r.corner,
            "cold_recheck_ns":oracle.1*1e9,"ghost":controls,"guarded":actual(&r,&xml)?}),
        );
    }
    write_json(&path.join("cuts.json"), &json!({"cases":cases}))
}

pub(super) fn configured(r: &Physical, n: u32, precise: bool) -> (Profile, String) {
    let (p, mut xml) = separated::configured(r, false, n, true);
    if precise {
        xml = xml.replace(
            "</guard_separated_flight>",
            "</guard_separated_flight><integrate_free_rotation>true</integrate_free_rotation>",
        );
    }
    (p, xml)
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut cases = vec![];
    for id in 0..CASES {
        let r = Physical::new(id);
        if r.certify().reason.is_some() {
            continue;
        }
        let mut trials = vec![];
        for n in [4, 8] {
            for precise in [false, true] {
                let (p, xml) = configured(&r, n, precise);
                let (mut row, frames, _) = scene::run(&r, false, p, &xml)?;
                row["precise_rotation"] = precise.into();
                row["first_cut"] = actual(&r, &xml)?;
                if id == 153 && n == 8 {
                    let name = if precise { "integrated" } else { "guard_only" };
                    separated::single_rigid_trace(path, name, &frames)?;
                    fs::write(path.join(format!("{name}.sdf")), xml)?;
                }
                trials.push(row);
            }
        }
        if id == 153 {
            separated::single_rigid_trace(path, "analytic", &r.trace())?;
        }
        cases.push(json!({"id":id,"trials":trials}));
    }
    write_json(
        &path.join("flight.json"),
        &json!({"cases":cases,"view_id":153}),
    )?;
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"title":"Torque-free rotation before a triaxial strike","lanes":[
        {"label":"guard_only","name":"Native gyro with separation guard","file":"traces/guard_only.json"},
        {"label":"integrated","name":"Conservative free rigid rotation","file":"traces/integrated.json"},
        {"label":"analytic","name":"Independent Euler flight and impact","file":"traces/analytic.json","analytical":true}]}),
    )
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut rows = vec![];
    for id in 0..CASES {
        let r = Physical::new(id);
        if r.certify().reason.is_some() {
            continue;
        }
        let mut variants = vec![];
        for n in [4, 8] {
            for precise in [false, true] {
                let (p, xml) = configured(&r, n, precise);
                let end = scene::run(&r, false, p, &xml)?.2;
                scene::measure(&xml, false, end)?;
                variants.push((n, precise, xml, end));
            }
        }
        let mut times: [Vec<[u64; 8]>; 4] = Default::default();
        for rep in 0..8 {
            for pos in 0..4 {
                let ix = (rep + pos) % 4;
                let i = if (rep / 4 + id) % 2 == 0 { ix } else { 3 - ix };
                times[i].push(scene::measure(&variants[i].2, false, variants[i].3)?);
            }
        }
        for ((calls, precise, _, _), raw) in variants.into_iter().zip(times) {
            rows.push(json!({"id":id,"calls":calls,"precise_rotation":precise,"primary_tick":(r.arrival*2000.).ceil()as usize,"tick_ns":raw}));
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":rows}))
}

pub(crate) fn frame_impact(path: &Path) -> Result<()> {
    let survey: Value = serde_json::from_slice(&fs::read(path.join("frames.json"))?)?;
    let sample = &survey["featured"];
    let matrix = |k: &str| -> Result<DMat3> {
        Ok(DMat3::from_cols_array(&serde_json::from_value(
            sample[k].clone(),
        )?))
    };
    let tensor = matrix("expected_tensor")?;
    let old = matrix("effective_old_tensor")?;
    let authored = matrix("authored_tensor")?;
    let axes = DQuat::from_array(serde_json::from_value(sample["authored_frame"].clone())?);
    let b = ReferenceBody {
        member: DVec3::new(0.041, 0.041, 0.072),
        principal_frame: axes,
        tensor,
        inverse: tensor.inverse(),
        lower: 0.041 * (1. - 1e-12),
        upper: 0.072 * (1. + 1e-12),
    };
    // Fixed selection criterion. Take the first certified 6 rad/s arrival in
    // the existing donor sequence to avoid high-spin time-integration confounds.
    let r = (0..CASES)
        .map(|i| Physical::with_body(i, b))
        .find(|r| {
            (r.body.omega(r.q, r.pre.l).length() - 6.).abs() < 1e-10 && r.certify().reason.is_none()
        })
        .ok_or("no certified moderate-spin donor")?;
    let (profile, base) = configured(&r, 8, true);
    let original = scene::tensor(tensor, ARM, DQuat::IDENTITY, M);
    fs::create_dir_all(path.join("traces"))?;
    let mut trials = vec![];
    for (label, material, frame) in [
        ("legacy", old, DQuat::IDENTITY),
        ("corrected", authored, axes),
    ] {
        if base.matches(&original).count() != 1 {
            return Err("missing authored inertial block".into());
        }
        let text = base.replace(&original, &scene::tensor(material, ARM, frame, M));
        let (mut row, frames, _) = scene::run(&r, false, profile, &text)?;
        row["label"] = label.into();
        trials.push(row);
        separated::single_rigid_trace(path, label, &frames)?;
        fs::write(path.join(format!("{label}.sdf")), text)?;
    }
    separated::single_rigid_trace(path, "analytic", &r.trace())?;
    write_json(
        &path.join("impact.json"),
        &json!({"survey_id":sample["id"],"reference":scene::summary(&r,&r.certify()),"trials":trials}),
    )?;
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"title":"Rotated oblate inertia at a corner strike","lanes":[
        {"label":"legacy","name":"Measured legacy effective inertia","file":"traces/legacy.json"},
        {"label":"corrected","name":"Corrected authored inertia","file":"traces/corrected.json"},
        {"label":"analytic","name":"Analytical authored tensor","file":"traces/analytic.json","analytical":true}]}),
    )
}
