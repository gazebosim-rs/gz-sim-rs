// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Retain the live primary graph after a full first-tick intervention, then
//! repeat selected boundary states on fresh graphs through three commands.
use super::super::super::finite::{geometry, kinematics, load::complete};
use super::super::secondary::physical::{state, thaw};
use super::whole::{Method, first};
use super::*;
const NAMES: [&str; 7] = [
    "endpoint8_warm512",
    "launch2_warm512",
    "launch8_warm512",
    "retimed_launch8_warm512",
    "endpoint8_cold512",
    "launch8_cold512",
    "mechanics_cold512",
];
const COMPACT: [&str; 7] = [
    "subdivided8_warm512",
    "subdivided2_warm512",
    "compact8_warm512",
    "compact64_warm512",
    "subdivided8_cold512",
    "compact8_cold512",
    "mechanics_cold512",
];
fn vector(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(v.clone())?))
}
pub(super) fn mechanical(f: Fixture, c: &ReboundFrame) -> Result<(Value, Tick)> {
    let flight = kinematics(f, c)?.0;
    let g = geometry(flight)?;
    let t = DT.as_secs_f64();
    let a = complete::at(flight, &g, t, 512)?;
    let b = complete::at(flight, &g, t, 128)?;
    if a["end"] != "original_clock"
        || b["end"] != "original_clock"
        || vector(&a["j"])?.distance(vector(&b["j"])?) > 1e-5
        || vector(&a["com"])?.distance(vector(&b["com"])?) > 1e-7
    {
        return Err("unqualified primary mechanical control".into());
    }
    let drive = command(f, c.tick) * DVec3::Z;
    let w = vector(&a["omega"])?;
    let center = vector(&a["com"])? + t * drive;
    let v = vector(&a["velocity"])? + drive;
    let foot = vector(&a["foot"])? + t * drive;
    let q = serde_json::from_value(a["orientation"].clone())?;
    let rigid = |p: DVec3| Body {
        position: p.to_array(),
        orientation: q,
        velocity: (v + w.cross(p - center)).to_array(),
        angular_velocity: w.to_array(),
    };
    let (foot, upper) = (rigid(foot), rigid(2. * center - foot));
    let mut pad = c.pad;
    pad.position = (DVec3::from(pad.position) + t * drive).to_array();
    pad.velocity = drive.to_array();
    let tick = Tick {
        tick: c.tick + 1,
        foot,
        upper,
        pad,
        normal: a["jn"].as_f64().ok_or("no scalar pressure")?,
        extra: 0.,
        tangent: vector(&a["j"])? - vector(&a["normal_vector"])?,
        first: Some(Duration::from_nanos(
            g["hi_ns"].as_u64().ok_or("no primary clock")?,
        )),
        gap: supported_gap(state(foot), state(pad)),
        calls: 0,
    };
    Ok((a, tick))
}
pub(crate) fn write(dir: &Path, start: usize, count: usize, compact: bool) -> Result<()> {
    let names = if compact { COMPACT } else { NAMES };
    let last = start
        .checked_add(count)
        .filter(|x| count > 0 && *x <= 50)
        .ok_or("invalid nominal range")?;
    for sub in ["cases", "frames", "trajectories"] {
        fs::create_dir_all(dir.join(sub))?;
    }
    for k in start..last {
        let name = format!("case-{k:03}.json");
        let f = fixture(source(k, true));
        let c = survey::cuts(f)?.0[0].ok_or("missing nominal primary")?;
        let (exact, tick) = mechanical(f, &c)?;
        let mut runs = vec![];
        let mut receipts = vec![];
        let modes = if compact {
            [
                (8, Method::Subdivided),
                (2, Method::Subdivided),
                (8, Method::Compact),
                (64, Method::Compact),
            ]
        } else {
            [
                (8, Method::Endpoint),
                (2, Method::Launch),
                (8, Method::Launch),
                (8, Method::Retimed),
            ]
        };
        for (n, m) in modes {
            let (r, p) = first(f, &c, m, n, 512, DURATION)?;
            receipts.push(r);
            runs.push(p);
        }
        for p in [runs[0][0], runs[2][0], tick] {
            runs.push(thaw(f, p)?);
        }
        let mut row = vec![];
        let mut series = vec![];
        let mut saved = vec![];
        let mut times = std::collections::BTreeSet::new();
        times.extend(c.tick + 1..=c.tick + 15);
        times.insert(DURATION);
        for path in &runs {
            for event in events(path)["strokes"]
                .as_array()
                .ok_or("missing strokes")?
            {
                if let Some(at) = event["first_native_end_ns"].as_u64() {
                    let t = at.div_ceil(500000);
                    times.extend(t.saturating_sub(3)..=t + 8);
                }
            }
        }
        for (i, p) in runs.iter().enumerate() {
            let final_tick = p.last().ok_or("empty primary continuation")?;
            let sums = [0, 2, 6]
                .map(|ix| compare(p, &runs[ix]))
                .into_iter()
                .collect::<Result<Vec<_>>>()?;
            row.push(json!({"profile":names[i],"first":p[0].as_json(),"end":final_tick.as_json(),"outcomes":events(p),"native_calls":p.iter().map(|x|u64::from(x.calls)).sum::<u64>(),"comparisons":sums,"contact_receipt":receipts.get(i)}));
            series.push(
                p.iter()
                    .map(|x| {
                        (
                            x.com().to_array(),
                            x.foot.angular_velocity,
                            x.normal,
                            x.tangent.to_array(),
                        )
                    })
                    .collect::<Vec<_>>(),
            );
            saved.push(
                p.iter()
                    .filter(|x| x.tick % 16 == 0 || times.contains(&x.tick))
                    .map(Tick::as_json)
                    .collect::<Vec<_>>(),
            );
        }
        let first = json!({"tick":c.tick,"time_us":c.tick*STEP_US,"time_ns":c.tick*500000,"foot":c.foot,"upper":c.upper,"pad":c.pad,"native_steps":0,"normal_impulse_ns":0.});
        write_json(
            &dir.join("trajectories").join(&name),
            &json!({"profiles":names,"first_tick":c.tick+1,"series":series}),
        )?;
        write_json(
            &dir.join("frames").join(&name),
            &json!({"initial":first,"paths":saved}),
        )?;
        let file = dir.join("cases").join(name);
        let tmp = file.with_extension("partial");
        write_json(
            &tmp,
            &json!({"format":if compact{"compact-primary-three-stroke-v1"}else{"full-primary-three-stroke-v1"},"case":k,"source":source(k,true),"fixture":f,"cut":snapshot(&c),"cut_tick":c.tick,"end_tick":DURATION,"mechanics":exact,"paths":row}),
        )?;
        fs::rename(tmp, file)?;
        eprintln!("completed original primary continuation {k}");
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, k: usize) -> Result<()> {
    let name = format!("case-{k:03}.json");
    let raw: Value = serde_json::from_reader(fs::File::open(dir.join("cases").join(&name))?)?;
    let f: Value = serde_json::from_reader(fs::File::open(dir.join("frames").join(&name))?)?;
    let folder = dir.join(format!("case-{k:03}"));
    fs::create_dir_all(folder.join("traces"))?;
    let mut lanes = vec![];
    let compact = raw["format"] == "compact-primary-three-stroke-v1";
    let names = if compact { COMPACT } else { NAMES };
    for (i, label) in [
        (
            0,
            if compact {
                "Eight native calls, each material interval certified"
            } else {
                "Only the primary impulse, then 512 native calls"
            },
        ),
        (
            2,
            if compact {
                "Coalesce a wholly certified tail, then 512 calls"
            } else {
                "Primary and subsequent transfers, then 512 native calls"
            },
        ),
        (
            6,
            "Integrated first tick, then 512 calls from a fresh native graph",
        ),
    ] {
        let mut frames = vec![f["initial"].clone()];
        let p = f["paths"][i].as_array().ok_or("missing poses")?;
        if p.first() != Some(&raw["paths"][i]["first"]) || p.last() != Some(&raw["paths"][i]["end"])
        {
            return Err("pose endpoints disagree with simulation".into());
        }
        frames.extend_from_slice(p);
        let file = format!("traces/{}.json", names[i]);
        write_json(
            &folder.join(&file),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        lanes.push(json!({"label":names[i],"name":label,"file":file,"analytical_ticks":if i==6{vec![raw["paths"][6]["first"]["tick"].clone()]}else{vec![]}}));
    }
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":"Three outer-support returns after the complete primary landing","pair_totals_only":true,"sampled_ticks":true,"lanes":lanes}),
    )
}
