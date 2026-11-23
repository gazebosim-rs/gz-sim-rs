// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Render the actual native callbacks from one authored Bevy update on its exact clock.
use super::*;
type State = [BodyState; 3];
struct Sample {
    time: u64,
    state: State,
    load: [[f64; 4]; 2],
    upper: Option<Value>,
    lower: Option<Value>,
}
fn native(xml: &str) -> crate::Result<Vec<Sample>> {
    let (mut sim, ids) = load(xml, false)?;
    let gate = id_named(&sim, "default::opponent::gate")?;
    let e = backend(&sim);
    let initial = [
        e.state(ids.foot).unwrap(),
        e.state(ids.pad).unwrap(),
        e.state(gate).unwrap(),
    ];
    let stream = Arc::new(Mutex::new(vec![Sample {
        time: 0,
        state: initial,
        load: [[0.; 4]; 2],
        upper: None,
        lower: None,
    }]));
    let writer = stream.clone();
    set_refined_observer(&mut sim, move |_, s| {
        let f = s.state(ids.foot).unwrap();
        let pad = s.state(ids.pad).unwrap();
        let roof = s.state(gate).unwrap();
        let mut samples = writer.lock().unwrap();
        let entry = samples.last().unwrap().state[0];
        let restored = s.opposing_return_impact.copied();
        let mut w = pair_loads(
            ids,
            gate,
            f,
            s.contact_patches(),
            s.rigid_point_impact,
            s.opposing_corner_impact,
        );
        if let Some(r) = restored {
            let h = r.lower.point;
            let j = DVec3::from(h.impact.impulse_on_foot).dot(h.impact.normal.into());
            w[0].scalar += j;
            w[0].corners[usize::from(h.corner)] += j;
        }
        let load = w.map(|w| std::array::from_fn(|k| w.corners[k] + w.corners[4 + k]));
        samples.push(Sample {
            time: s.elapsed.as_nanos() as u64,
            state: [f, pad, roof],
            load,
            upper: s.opposing_corner_impact.map(|r| {
                super::projected::hit_record(r, 0, entry, restored.map_or(f, |r| r.branch_start))
            }),
            lower: restored.map(|r| super::projected::returns::record(r, 0, f)),
        });
    })?;
    sim.step_exact(1)?;
    let samples = std::mem::take(&mut *stream.lock().unwrap());
    if samples.len() != 9 {
        return Err(format!(
            "expected eight selected native callbacks, got {}",
            samples.len() - 1
        )
        .into());
    }
    assert_eq!(
        samples.last().unwrap().time,
        sim.elapsed().as_nanos() as u64
    );
    Ok(samples)
}
fn exact(i: &Impact, g: &Gate, hit: Crossing, second: Crossing, time: f64) -> State {
    let x = if time < second.at {
        super::reference(i, hit, second, time)
    } else {
        super::projected::returns::advances(i, second, time)
    };
    [
        at_foot(&i.r, x),
        pose(i.r.pad_v * time - i.r.n * PAD_HALF[2], i.r.pad_q, i.r.pad_v),
        pose(g.center + g.velocity * time, i.r.pad_q, g.velocity),
    ]
}
pub(in super::super) fn write(
    path: &Path,
    i: &Impact,
    g: &Gate,
    gate_mu: f64,
    hit: Crossing,
    second: Crossing,
) -> crate::Result<()> {
    let gap = second.at - hit.at;
    let start = hit.at - 0.08 * gap;
    let dt = 8 * (1.24 * gap * 1e9 / 8.).ceil() as u64;
    let at = from_primary(&i.r, start);
    let xml = super::projected::enable(&initial_sdf(i, g, gate_mu, start, at, dt, true));
    let ordered = super::projected::returns::enroll(&xml);
    let control = native(&xml)?;
    let policy = native(&ordered)?;
    let times = |v: &[Sample]| v.iter().map(|s| s.time).collect::<Vec<_>>();
    if times(&control) != times(&policy) {
        return Err(format!(
            "native sample times differ: {:?} {:?}",
            times(&control),
            times(&policy)
        )
        .into());
    }
    if policy.iter().filter(|s| s.upper.is_some()).count() != 1
        || policy.iter().filter(|s| s.lower.is_some()).count() != 1
    {
        return Err(format!(
            "expected both scheduled impulses: {} upper and {} lower",
            policy.iter().filter(|s| s.upper.is_some()).count(),
            policy.iter().filter(|s| s.lower.is_some()).count()
        )
        .into());
    }
    let end = super::follow(i, faces(&i.r, g), [i.mu, i.mu.min(gate_mu)], second, WINDOW).through;
    assert!(start + dt as f64 * 1e-9 < end);
    let mut analytic = vec![];
    let mut last = start;
    for sample in &control {
        let t = start + sample.time as f64 * 1e-9;
        let mut w = [[0.; 4]; 2];
        for e in [hit, second] {
            if last < e.at && e.at <= t {
                w[e.surface][e.corner % 4] += e.law.impulse.dot(e.plane.frame * DVec3::Z);
            }
        }
        analytic.push(Sample {
            time: sample.time,
            state: exact(i, g, hit, second, t),
            load: w,
            upper: None,
            lower: None,
        });
        last = t;
    }
    for (side, face) in [(0, "bottom"), (1, "top")] {
        let dir = path.join(face);
        fs::create_dir_all(dir.join("traces"))?;
        fs::write(dir.join("upper-only.sdf"), &xml)?;
        fs::write(dir.join("ordered.sdf"), &ordered)?;
        let geom = json!({"foot_half_extents":FOOT_HALF,"foot_com_local":ARM.to_array(),"pad_half_extents":PAD_HALF,"opponent_half_extents":HALF_GATE});
        let mut lanes = vec![];
        let mut scores = vec![];
        for (key, name, v) in [
            ("upper", "Projected ceiling impact", &control),
            ("ordered", "Projected ceiling and lower return", &policy),
            ("analytic", "Independent ordered impulses", &analytic),
        ] {
            let mut data = vec![];
            let mut row = vec![];
            let analytical = key == "analytic";
            for (k, s) in v.iter().enumerate() {
                let [foot, pad, gate] = s.state;
                let true_state = analytic[k].state[0];
                let a = Flight::from_state(&i.r, foot);
                let b = Flight::from_state(&i.r, true_state);
                let mut frame = snap(
                    k as u64,
                    1,
                    u32::from(k > 0 && !analytical),
                    foot,
                    pad,
                    gate,
                    s.load[side],
                );
                frame["time_ns"] = json!(s.time);
                frame["time_us"] = json!(s.time / 1000);
                frame["selected"] = json!(k > 0 && !analytical);
                data.push(frame);
                row.push(json!({"time_ns":s.time,"com_m":a.com.distance(b.com),"spin_rad_s":DVec3::from(foot.angular_velocity).distance(true_state.angular_velocity.into()),"pad_ns":s.load[0],"gate_ns":s.load[1],"upper":s.upper,"lower":s.lower}));
            }
            let file = format!("traces/{key}.json");
            write_json(&dir.join(&file), &json!({"geometry":geom,"frames":data}))?;
            let mut desc = json!({"label":key,"name":name,"file":file});
            if analytical {
                desc["analytical"] = json!(true);
            }
            lanes.push(desc);
            scores.push(json!({"lane":key,"samples":row}));
        }
        let tick = |t: f64| {
            control
                .iter()
                .position(|s| start + s.time as f64 * 1e-9 >= t)
                .unwrap()
        };
        write_json(
            &dir.join("comparison.json"),
            &json!({"title":if side==1{"The upper strike within one actuator update"}else{"The return to the lower pad in the same actuator update"},"single_body":true,"interval_samples":true,"public_dt_ns":dt,"load_face":face,"capture_ticks":{"upper_strike":tick(hit.at),"lower_return":tick(second.at)},"lanes":lanes}),
        )?;
        write_json(
            &dir.join("scores.json"),
            &json!({"id":i.r.id,"mode":MODES[i.mode],"start_ns":start*1e9,"actuator_dt_ns":dt,"time_ns":times(&control),"upper":hit.data(),"lower":second.data(),"qualified_through_ns":end*1e9,"lanes":scores}),
        )?;
    }
    Ok(())
}
