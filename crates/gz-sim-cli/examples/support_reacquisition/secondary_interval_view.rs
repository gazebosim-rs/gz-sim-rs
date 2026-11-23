// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! True nested native timestamps and the mechanical trajectory over one landing.
use super::*;
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum IntervalView {
    Native,
    Transfer,
    Spatial,
    Launch,
}
pub(crate) fn write(dir: &Path, k: usize, stroke: usize, mode: IntervalView) -> Result<()> {
    if k >= 1250 || !(1..=3).contains(&stroke) {
        return Err("invalid interval view fixture".into());
    }
    let f = survey::fixture(k);
    let c = survey::cuts(f)?.0[stroke - 1].ok_or("missing launch donor")?;
    let d = laboratory(f, &c)?.ok_or("no incident line")?;
    let transfer = mode != IntervalView::Native;
    let h = (DT - d.clock_offset).as_nanos() as u64;
    if h < if transfer { 8 } else { 8192 } {
        return Err("too few nanoseconds for the nested 8,192-call comparison".into());
    }
    let mut traces = vec![];
    let mut initial = None;
    let mut stamps = vec![];
    let modes = if mode == IntervalView::Launch {
        [
            (
                "spatial8",
                8,
                "Following skew edges with established slip",
                Action::Spatial,
            ),
            (
                "launch8",
                8,
                "Resolving the launch from static friction",
                Action::Launch,
            ),
        ]
    } else if mode == IntervalView::Spatial {
        [
            (
                "transfer8",
                8,
                "Internal transfers and fixed material points",
                Action::Transfer,
            ),
            (
                "spatial8",
                8,
                "Sliding along both skew edges",
                Action::Spatial,
            ),
        ]
    } else if transfer {
        [
            (
                "bound8",
                8,
                "Following points until the next feature",
                Action::Both,
            ),
            (
                "transfer8",
                8,
                "Resolving internal changes of bearing",
                Action::Transfer,
            ),
        ]
    } else {
        [
            ("native8", 8, "Eight ordinary native calls", Action::Native),
            (
                "native8192",
                8192,
                "8,192 ordinary native calls",
                Action::Native,
            ),
        ]
    };
    let mut actual = Value::Null;
    for (id, n, label, action) in modes {
        let mut frames = vec![];
        let b = path_with_samples(
            f,
            &c,
            &d,
            p(id, b(n, 1), b(8, 1)),
            action,
            c.tick + 1,
            Some(&mut frames),
        )?;
        if matches!(action, Action::Transfer | Action::Spatial | Action::Launch) {
            actual = json!({"initial_impact":b.impulse,"spans":b.vertices,"transfers":b.transfers});
        }
        let ns = frames
            .iter()
            .map(|v| v["time_ns"].as_u64().ok_or("no sample time"))
            .collect::<std::result::Result<Vec<_>, _>>()?;
        if stamps.is_empty() {
            stamps = ns;
            initial = Some(frames[0].clone())
        } else if stamps != ns || initial.as_ref() != Some(&frames[0]) {
            return Err("nested native boundaries changed".into());
        }
        let j = frames
            .iter()
            .map(|a| a["normal_impulse_ns"].as_f64().ok_or("no normal receipt"))
            .sum::<std::result::Result<f64, _>>()?;
        if (j - b.ticks[0].normal).abs() > 2e-10 {
            return Err("observed scalar load differs from fast native path".into());
        }
        traces.push((id, label, false, frames));
    }
    let mut analytical = vec![initial.unwrap()];
    let mut normal = 0.;
    let mut j = DVec3::ZERO;
    let mut mechanics = Value::Null;
    for i in 1..=8 {
        let elapsed = stamps[i] - stamps[0];
        if elapsed != i as u64 * h / 8 {
            return Err("native bins not nested on physical clock".into());
        }
        let (z, t) = physical::mechanics_at(&d, f, &c, 512, elapsed as f64 * 1e-9)?;
        let t = t.ok_or("unresolved intermediate mechanical state")?;
        let load = DVec3::from_array(serde_json::from_value(z["impulse_ns"].clone())?);
        let ns = stamps[i];
        let mut a = t.as_json();
        a["tick"] = json!(i);
        a["time_ns"] = json!(ns);
        a["time_us"] = json!(ns / 1000);
        a["native_steps"] = json!(0);
        a["solver_iterations"] = json!(0);
        a["normal_impulse_ns"] = json!(t.normal - normal);
        a["world_impulse_ns"] = json!((load - j).to_array());
        if t.normal < normal {
            return Err("negative reaction in mechanical bin".into());
        }
        normal = t.normal;
        j = load;
        analytical.push(a);
        mechanics = z;
    }
    traces.push((
        "mechanics",
        "Piecewise rigid-body contact reference",
        true,
        analytical,
    ));
    let folder = dir.join(format!("case-{k:04}-stroke-{stroke}"));
    fs::create_dir_all(folder.join("traces"))?;
    write_json(&folder.join("mechanical_events.json"), &mechanics)?;
    if transfer {
        write_json(&folder.join("delivered_events.json"), &actual)?;
    }
    let mut lanes = vec![];
    for (id, label, reference, frames) in traces {
        let file = format!("traces/{id}.json");
        write_json(
            &folder.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"label":id,"name":label,"file":file,"analytical":reference}));
    }
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":if mode==IntervalView::Launch{"A loaded point starts sliding from rest"}else if mode==IntervalView::Spatial{"A sliding contact migrates along two skew material edges"}else if transfer{"Measured changes of bearing inside eight native calls"}else{"Eight matched observations during one landing interval"},"public_dt_ns":500_000,"interval_samples":true,
        "sampled_ticks":true,"pair_totals_only":true,"clock_origin_ns":stamps[0],"lanes":lanes}),
    )
}
