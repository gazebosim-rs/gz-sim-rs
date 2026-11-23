// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Physical checks of pressure-center projection through the public native interval.
use super::*;
use gz_physics_multifidelity::BoxContactActions;

const PHASES: [u64; 11] = [
    0, 1, 15_625, 62_499, 62_501, 138_197, 250_000, 312_501, 437_499, 484_375, 499_999,
];
const SCHEMA: &str = "projected-planar-patch-v1";
#[derive(Serialize, Deserialize)]
struct Row {
    case: usize,
    phase_ns: u64,
    calls: u32,
    projections: usize,
    bearing: Option<String>,
    clock_error_ns: Option<u64>,
    impulse_error_ns: Option<f64>,
    pressure_error_m: Option<f64>,
    work_error_j: Option<f64>,
    cone_excess_ns: Option<f64>,
    total_impulse_error_ns: f64,
    spin_error_rad_s: f64,
    com_error_m: f64,
    native_extra_ns: f64,
}
fn trial(c: Case, t: Duration, k: usize, calls: u32, save: bool) -> Result<(Row, Vec<Value>)> {
    let Rig {
        mut native,
        foot,
        upper,
        pad,
        velocity: incoming,
    } = Rig::arriving(c, Model::Locked, t)?;
    let pair = [(foot, pad)];
    let target = c.analytic()?;
    let desired = DVec3::new(target.impulse[0], 0., target.impulse[1]);
    let bodies = |n: &MultiFidelityBackend| {
        links(n.state(foot).unwrap(), upper.map(|u| n.state(u).unwrap()))
    };
    let com = |b: [BodyState; 2]| 0.5 * (DVec3::from(b[0].position) + DVec3::from(b[1].position));
    let initial = com(bodies(&native));
    let mut val = None;
    let mut frames = vec![];
    for tick in 0..=if save { 32 } else { 1 } {
        let mut normal = 0.;
        let mut used = 0;
        if tick > 0 {
            let r = if tick == 1 {
                native
                    .advance_box_contacts(
                        DT,
                        calls.max(1),
                        4.into(),
                        BoxContactActions {
                            planar_impacts: if calls == 0 { &[] } else { &pair },
                            ..Default::default()
                        },
                        None,
                    )?
                    .interval
            } else {
                native.step_contact_interval(DT, 1, 4)?
            };
            used = r.native_steps;
            normal = r
                .pairs
                .iter()
                .filter(|v| (v.a, v.b) == (foot.min(pad), foot.max(pad)))
                .map(|q| q.normal_impulse)
                .sum();
            if tick == 1 {
                let h = r.planar_impacts.first();
                let b = bodies(&native);
                let expect_com =
                    initial + incoming * DT.as_secs_f64() + 0.5 * desired * (DT - t).as_secs_f64();
                let p = h.map(|s| s.resultant);
                let bearing = h.map(|s| format!("{:?}", s.bearing));
                if h.is_some_and(|h| {
                    h.bearing != target.bearing
                        && (h.pressure_x - target.center_of_pressure).abs() > 1e-8
                }) {
                    return Err(format!("load classification {k}").into());
                }
                val = Some(Row {
                    case: k,
                    phase_ns: t.as_nanos() as u64,
                    calls: used,
                    projections: r.planar_impacts.len(),
                    bearing,
                    clock_error_ns: p.map(|s| s.elapsed.abs_diff(t).as_nanos() as u64),
                    impulse_error_ns: p.map(|s| DVec3::from(s.impulse_on_foot).distance(desired)),
                    pressure_error_m: h.map(|s| (s.pressure_x - target.center_of_pressure).abs()),
                    work_error_j: p.map(|s| (s.dissipated_energy - target.dissipated_energy).abs()),
                    cone_excess_ns: p.map(|s| {
                        (s.impulse_on_foot[0].hypot(s.impulse_on_foot[1])
                            - c.mu * s.impulse_on_foot[2])
                            .max(0.)
                    }),
                    total_impulse_error_ns: (2. * (velocity(b) - incoming)).distance(desired),
                    spin_error_rad_s: DVec3::from(b[0].angular_velocity)
                        .distance(DVec3::Y * target.pitch_rate),
                    com_error_m: com(b).distance(expect_com),
                    native_extra_ns: normal - p.map_or(0., |s| s.impulse_on_foot[2]),
                });
            }
        }
        if save {
            let [a, b] = bodies(&native);
            frames.push(json!({"tick":tick,"time_us":tick*500,"foot":as_json(a),"upper":as_json(b),"pad":as_json(native.state(pad).unwrap()),"normal_impulse_ns":normal,"native_steps":used,"solver_iterations":4}));
        }
    }
    Ok((val.unwrap(), frames))
}
pub(super) fn study(out: &Path, first: usize, count: usize) -> Result<()> {
    let end = first
        .checked_add(count)
        .filter(|v| *v <= FIELD * PHASES.len() && count > 0)
        .ok_or("invalid patch range")?;
    let dir = out.join("batches");
    fs::create_dir_all(&dir)?;
    for start in (first..end).step_by(256) {
        let stop = (start + 256).min(end);
        let dest = dir.join(format!("batch-{start:06}-{stop:06}.json"));
        if dest.exists() {
            let v: Value = serde_json::from_reader(fs::File::open(&dest)?)?;
            if v["format"] != SCHEMA
                || v["start"] != start
                || v["end"] != stop
                || v["rows"].as_array().map(Vec::len) != Some(2 * (stop - start))
            {
                return Err("invalid patch batch".into());
            }
            continue;
        }
        let mut rows = vec![];
        for i in start..stop {
            let k = i / PHASES.len();
            let c = Case::at(k);
            let phase = PHASES[(i % PHASES.len() + k) % PHASES.len()];
            for n in [2, 8] {
                rows.push(trial(c, Duration::from_nanos(phase), k, n, false)?.0);
            }
        }
        let tmp = dest.with_extension(format!("partial-{}", std::process::id()));
        write_json(
            &tmp,
            &json!({"format":SCHEMA,"start":start,"end":stop,"rows":rows}),
        )?;
        fs::rename(tmp, dest)?;
    }
    if first == 0 {
        fs::create_dir_all(out.join("traces"))?;
        let c = Case {
            offset: -0.42,
            mu: 0.6,
            slip: 1.3,
            closing: 1.4,
            pad_first: false,
        };
        let mut lanes = vec![];
        for (id, name, n) in [
            ("ordinary", "Ordinary native solve", 0),
            ("two", "Two calls with the patch projection", 2),
            ("eight", "Eight calls with the patch projection", 8),
        ] {
            let (r, frames) = trial(c, Duration::from_nanos(499_999), 0, n, true)?;
            write_json(
                &out.join("traces").join(format!("{id}.json")),
                &json!({"geometry":{"foot_half_extents":FOOT,"upper_half_extents":TOP,"pad_half_extents":PAD},"frames":frames}),
            )?;
            lanes.push(
                json!({"label":id,"name":name,"file":format!("traces/{id}.json"),"measured":r}),
            );
        }
        write_json(
            &out.join("comparison.json"),
            &json!({"title":"An offset planar edge lands one nanosecond before the public boundary","pair_totals_only":true,"fixture":c,"lanes":lanes}),
        )?;
    }
    Ok(())
}
