// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure a physical impulse across successive actuator receipts.
use super::*;

const LABELS: [&str; 4] = ["clipped", "carry", "uniform8_r32", "uniform256_r32"];
fn trial(c: Case, t: Duration, id: usize, capture: bool) -> Result<(Vec<[f64; 4]>, Vec<Value>)> {
    let Rig {
        mut native,
        foot,
        upper,
        pad,
        velocity: incoming,
    } = Rig::arriving(c, Model::Locked, t)?;
    let window = ContactWindow {
        before: Duration::from_micros(8),
        after: Duration::from_micros(8),
        max_native_dt: Duration::from_micros(4),
        ordinary: budget(1),
        refined: budget(32),
    };
    let target = c.analytic()?;
    let desired = DVec3::new(target.impulse[0], 0., target.impulse[1]);
    let b = |n: &MultiFidelityBackend| {
        links(n.state(foot).unwrap(), upper.map(|u| n.state(u).unwrap()))
    };
    let center =
        |b: [BodyState; 2]| 0.5 * (DVec3::from(b[0].position) + DVec3::from(b[1].position));
    let initial = center(b(&native));
    let mut frames = vec![];
    let mut values = vec![];
    let intervals = if capture { 16 } else { 2 };
    for tick in 0..=intervals {
        let mut j = 0.;
        let mut calls = 0;
        if tick > 0 {
            let start = DT * (tick - 1);
            let segments = if tick > 2 {
                window.schedule(DT, &[])?
            } else {
                match id {
                    0 if tick == 2 => window.schedule(DT, &[])?,
                    0 | 1 => window.schedule_at(start, DT, &[t])?,
                    2 | 3 => vec![ContactSegment {
                        duration: DT,
                        native_steps: if id == 2 { 8 } else { 256 },
                        iterations: budget(32),
                    }],
                    _ => unreachable!(),
                }
            };
            let (r, _) = native.step_contact_schedule(DT, &segments, &[])?;
            calls = r.native_steps;
            j = r
                .pairs
                .iter()
                .filter(|p| (p.a, p.b) == (foot.min(pad), foot.max(pad)))
                .map(|p| p.normal_impulse)
                .sum();
            let state = b(&native);
            let h = DT * tick;
            let exact = initial
                + incoming * h.as_secs_f64()
                + 0.5 * desired * h.saturating_sub(t).as_secs_f64();
            values.push([
                (2. * (velocity(state) - incoming)).distance(desired),
                DVec3::from(state[0].angular_velocity).distance(DVec3::Y * target.pitch_rate),
                center(state).distance(exact),
                f64::from(calls),
            ]);
        }
        if capture {
            let s = b(&native);
            frames.push(json!({"tick":tick,"time_us":tick*500,"foot":as_json(s[0]),"upper":as_json(s[1]),"pad":as_json(native.state(pad).unwrap()),"normal_impulse_ns":j,"native_steps":calls,"solver_iterations":4}));
        }
    }
    Ok((values, frames))
}
pub(crate) fn study(out: &Path) -> Result<()> {
    fs::create_dir_all(out.join("traces"))?;
    let mut summary = vec![];
    let mut lanes = vec![];
    for phase in [484_375, 499_999] {
        let t = Duration::from_nanos(phase);
        let mut samples = vec![vec![]; LABELS.len()];
        for k in 0..FIELD {
            let c = Case::at(k);
            for (id, &label) in LABELS.iter().enumerate() {
                let save = phase == 499_999 && k == 1410 && id != 2;
                let (measure, frames) = trial(c, t, id, save)?;
                samples[id].push(measure);
                if save {
                    write_json(
                        &out.join("traces").join(format!("{label}.json")),
                        &json!({"geometry":{"foot_half_extents":FOOT,"upper_half_extents":TOP,"pad_half_extents":PAD},"frames":frames}),
                    )?;
                    lanes.push(json!({"label":label,"name":match id{0=>"Clipped at public boundary",1=>"Window across public boundary",_=>"Uniform 256 over both ticks"},"file":format!("traces/{label}.json")}));
                }
            }
        }
        for (id, label) in LABELS.iter().enumerate() {
            for tick in 0..2 {
                let mut row = json!({"phase_ns":phase,"profile":label,"tick":tick+1,"cases":FIELD});
                for (j, key) in [
                    "cumulative_impulse_error_ns",
                    "spin_error_rad_s",
                    "com_error_m",
                    "calls",
                ]
                .iter()
                .enumerate()
                {
                    let mut v: Vec<_> = samples[id]
                        .iter()
                        .enumerate()
                        .map(|(k, r)| (r[tick][j], k))
                        .collect();
                    v.sort_unstable_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
                    row[key] = json!({"median":v[FIELD.div_ceil(2)-1].0,"p95":v[(95*FIELD).div_ceil(100)-1].0,"max":v.last().unwrap().0,"worst":v.last().unwrap().1});
                }
                summary.push(row);
            }
        }
        write_json(
            &out.join("summary.json"),
            &json!({"cases_per_arrival":FIELD,"rows":summary}),
        )?;
    }
    write_json(
        &out.join("comparison.json"),
        &json!({"title":"Landing one nanosecond before the actuator boundary","pair_totals_only":true,"case":1410,"phase_ns":499_999,"lanes":lanes}),
    )
}
