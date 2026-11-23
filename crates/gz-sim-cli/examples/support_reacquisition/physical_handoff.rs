// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Delay the return to the ordinary native solver along the same physical path.
//! Each treatment reconstructs the same cold graph at its own public cutoff.
use super::{
    super::secondary::physical::{state, thaw_with_grid},
    *,
};
fn v(s: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(s.clone())?))
}
pub(super) fn qualified(a: &Value, b: &Value) -> Result<bool> {
    if a["end"] != "original_clock" || b["end"] != "original_clock" {
        return Ok(false);
    }
    let x = a["events"].as_array().ok_or("missing reference events")?;
    let y = b["events"].as_array().ok_or("missing reference events")?;
    Ok(x.len() == y.len()
        && x.iter()
            .zip(y)
            .all(|(p, q)| ["mode", "feature", "stuck"].iter().all(|k| p[k] == q[k]))
        && v(&a["j"])?.distance(v(&b["j"])?) <= 1e-5
        && v(&a["com"])?.distance(v(&b["com"])?) <= 1e-7
        && b["arc_energy_defect"]
            .as_f64()
            .ok_or("missing energy defect")?
            .abs()
            <= 1e-7)
}
pub(super) fn prefix(f: Fixture, c: &ReboundFrame, doc: &Value) -> Result<Vec<Tick>> {
    let (mechanics, first) = super::whole_long::mechanical(f, c)?;
    let refs = doc["references"]
        .as_array()
        .ok_or("missing mechanical history")?;
    let norm: Value = serde_json::from_slice(&serde_json::to_vec(&mechanics)?)?;
    if refs[0]["fine"] != norm {
        return Err("different first physical landing".into());
    }
    let drive = v(&doc["drive"])?;
    let mut ticks = vec![first];
    for (i, r) in refs.iter().enumerate().skip(1) {
        let (a, b) = (&r["coarse"], &r["fine"]);
        if !qualified(a, b)? {
            break;
        }
        let at = (i + 1) as u64;
        if r["at"] != at || drive.z != command(f, c.tick + at - 1) {
            return Err("foreign actuator command".into());
        }
        let t = at as f64 * DT.as_secs_f64();
        let center = v(&b["com"])? + t * drive;
        let w = v(&b["omega"])?;
        let speed = v(&b["velocity"])? + drive;
        let foot = v(&b["foot"])? + t * drive;
        let orientation = serde_json::from_value(b["orientation"].clone())?;
        let body = |p: DVec3| Body {
            position: p.to_array(),
            orientation,
            angular_velocity: w.to_array(),
            velocity: (speed + w.cross(p - center)).to_array(),
        };
        let (foot, upper) = (body(foot), body(2. * center - foot));
        let mut pad = c.pad;
        pad.velocity = drive.to_array();
        pad.position = (DVec3::from(c.pad.position) + t * drive).to_array();
        let prev = &refs[i - 1]["fine"];
        let j = v(&b["j"])? - v(&prev["j"])?;
        let normal_vec = v(&b["normal_vector"])? - v(&prev["normal_vector"])?;
        let normal = b["jn"].as_f64().ok_or("missing pressure")?
            - prev["jn"].as_f64().ok_or("missing previous pressure")?;
        if normal < -1e-11 {
            return Err("decreasing mechanical pressure".into());
        }
        ticks.push(Tick {
            tick: c.tick + at,
            foot,
            upper,
            pad,
            normal,
            extra: 0.,
            tangent: j - normal_vec,
            first: None,
            gap: supported_gap(state(foot), state(pad)),
            calls: 0,
        });
    }
    Ok(ticks)
}
pub(crate) fn write(
    output: &Path,
    episodes: &Path,
    start: usize,
    count: usize,
    grid: u32,
) -> Result<()> {
    if !matches!(grid, 512 | 16384 | 32768)
        || count == 0
        || start.checked_add(count).is_none_or(|x| x > 50)
    {
        return Err("invalid return-to-native study".into());
    }
    let folder = output.join(format!("grid-{grid}"));
    fs::create_dir_all(folder.join("cases"))?;
    for k in start..start + count {
        let f = fixture(source(k, true));
        let c = survey::cuts(f)?.0[0].ok_or("missing primary cut")?;
        let name = format!("case-{k:03}");
        let doc: Value = serde_json::from_reader(fs::File::open(
            episodes.join("cases").join(format!("{name}.json")),
        )?)?;
        let cut: Value = serde_json::from_slice(&serde_json::to_vec(&snapshot(&c))?)?;
        if doc["cut"] != cut || doc["source"] != source(k, true) {
            return Err("different primary donor".into());
        }
        let physical = prefix(f, &c, &doc)?;
        fs::create_dir_all(folder.join(&name).join("traces"))?;
        let mut records = Vec::new();
        for &n in if grid == 512 {
            &[1, 2, 8, 26][..]
        } else {
            &[8][..]
        } {
            if n > physical.len() {
                continue;
            }
            let native = thaw_with_grid(f, physical[n - 1], grid)?;
            let mut ticks = physical[..n - 1].to_vec();
            ticks.extend(native);
            let file = format!("traces/physical-{n}.json");
            write_json(
                &folder.join(&name).join(&file),
                &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":ticks.iter().map(Tick::as_json).collect::<Vec<_>>()}),
            )?;
            records.push(json!({"physical_ticks":n,"native_calls":ticks.iter().map(|t|u64::from(t.calls)).sum::<u64>(),"end":ticks.last().unwrap().as_json(),"outcomes":events(&ticks),"file":file}));
            eprintln!("physical cutoff {n}, grid {grid}, donor {k}");
        }
        let dst = folder.join("cases").join(format!("{name}.json"));
        let tmp = dst.with_extension("partial");
        write_json(
            &tmp,
            &json!({"format":"matched-physical-handoff-v1","case":k,"source":source(k,true),"cut":cut,"end":DURATION,"grid":grid,"qualified_prefix":physical.len(),"paths":records}),
        )?;
        fs::rename(tmp, dst)?;
    }
    Ok(())
}
