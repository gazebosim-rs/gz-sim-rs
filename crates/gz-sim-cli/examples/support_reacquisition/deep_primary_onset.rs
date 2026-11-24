// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Compare fine native primary landings on nested executed steps. The same
//! registered world and ordinary solver settings produce each complete history.
use super::super::super::finite::{geometry, kinematics};
use super::*;
use std::io::{BufWriter, Write};

pub(crate) const COMMON: usize = 8192;
const BUDGETS: [u32; 3] = [8192, 16384, 32768];
#[derive(Clone, Copy)]
struct Observed {
    ns: u64,
    foot: Body,
    upper: Body,
    pad: Body,
    normal: f64,
    impulse: DVec3,
}
impl Observed {
    fn center(self) -> DVec3 {
        0.5 * (DVec3::from(self.foot.position) + DVec3::from(self.upper.position))
    }
    fn velocity(self) -> DVec3 {
        0.5 * (DVec3::from(self.foot.velocity) + DVec3::from(self.upper.velocity))
    }
    fn fields(self) -> [f64; 13] {
        let (c, v, w, j) = (
            self.center(),
            self.velocity(),
            self.foot.angular_velocity,
            self.impulse,
        );
        [
            c.x,
            c.y,
            c.z,
            v.x,
            v.y,
            v.z,
            w[0],
            w[1],
            w[2],
            j.x,
            j.y,
            j.z,
            self.normal,
        ]
    }
}
struct Run {
    samples: Vec<Observed>,
    first: Value,
    receipt: Value,
}
fn observe(f: Fixture, cut: &ReboundFrame, n: u32) -> Result<Run> {
    let mut rig = restart(f, ADAPTIVE, cut)?;
    let lane = &mut rig.case.lane;
    if lane.tier()? != Dynamic {
        return Err("matched cut is not dynamic".into());
    }
    let [a, u, d] = [lane.foot, lane.upper, lane.pad];
    let mut res = lane.sim.world_mut().resource_mut::<PhysicsBackend>();
    let native = res
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("missing native solver")?;
    let speed = command(f, cut.tick);
    if rig.last_speed != speed {
        native.set_world_velocity(d, [0., 0., speed], [0.; 3])?;
    }
    observe_grid(native, [a, u, d], n)
}
fn observe_grid(native: &mut MultiFidelityBackend, [a, u, d]: [BodyId; 3], n: u32) -> Result<Run> {
    let initial = Observed {
        ns: 0,
        foot: native.state(a).ok_or("foot")?.into(),
        upper: native.state(u).ok_or("upper")?.into(),
        pad: native.state(d).ok_or("pad")?.into(),
        normal: 0.,
        impulse: DVec3::ZERO,
    };
    let mut samples = Vec::with_capacity(COMMON + 1);
    samples.push(initial);
    let mut calls = 0;
    let mut ok = true;
    let mut first = Value::Null;
    let mut sum = DVec3::ZERO;
    let mut amount = 0.;
    let mut extra = 0.;
    let settings = ContactSolverIterations {
        temporal: 4,
        pgs: None,
        relaxation: Some(1),
    };
    let receipt=native.observe_contact_interval(DT,n,settings,|s|{
        calls+=1;
        let mut local=DVec3::ZERO;let mut normal=0.;let mut sites=vec![];
        for p in s.contact_patches(){
            let selected=(p.a,p.b)==(a.min(d),a.max(d));
            for site in p.points {
                if site.normal_impulse<=0.{continue;}
                if !selected {extra+=site.normal_impulse;continue;}
                let Some(t)=site.tangent_impulse_on_a else{ok=false;continue};
                let sign=if p.a==a{1.}else{-1.};let axis=sign*DVec3::from(p.normal_on_a);
                let j=sign*DVec3::from(t)+axis*site.normal_impulse;
                if first.is_null(){sites.push(json!({"normal":axis.to_array(),"j_ns":j.to_array(),"normal_ns":site.normal_impulse,"lever_m":if p.a==a{site.lever_a}else{site.lever_b}}));}
                local+=j;normal+=site.normal_impulse;
            }
        }
        let ns=s.elapsed.as_nanos()as u64;
        if first.is_null() && normal>0.{first=json!({"start_ns":ns-s.duration.as_nanos()as u64,"end_ns":ns,"call":calls,"j_ns":local.to_array(),"normal_ns":normal,"sites":sites});}
        sum+=local;amount+=normal;
        if calls%(n/COMMON as u32)==0 {
            let(Some(foot),Some(upper),Some(pad))=(s.state(a),s.state(u),s.state(d)) else{ok=false;return;};
            samples.push(Observed{ns,foot:foot.into(),upper:upper.into(),pad:pad.into(),normal:amount,impulse:sum});
        }
    })?;
    let pair = receipt
        .pairs
        .iter()
        .find(|p| (p.a, p.b) == (a.min(d), a.max(d)));
    let (j, normal, first_end) = if let Some(pair) = pair {
        let sign = if pair.a == a { 1. } else { -1. };
        (
            sign * (DVec3::from(pair.normal_impulse_on_a)
                + DVec3::from(pair.tangent_impulse_on_a.ok_or("missing friction")?)),
            pair.normal_impulse,
            pair.first_positive_end,
        )
    } else {
        (DVec3::ZERO, 0., None)
    };
    if !ok
        || calls != n
        || receipt.native_steps != n
        || samples.len() != COMMON + 1
        || extra > 1e-13
        || (amount - normal).abs() > 2e-10
        || sum.distance(j) > 2e-10
        || first["end_ns"].as_u64() != first_end.map(|t| t.as_nanos() as u64)
    {
        return Err("observed interval disagrees with public physics".into());
    }
    Ok(Run {
        samples,
        first,
        receipt: json!({"public_foot":Body::from(native.state(a).ok_or("foot")?),"public_upper":Body::from(native.state(u).ok_or("upper")?),"public_pad":Body::from(native.state(d).ok_or("pad")?),"calls":n,"j_ns":j.to_array(),"normal_ns":normal,"other_ns":extra}),
    })
}
pub(crate) fn write(dir: &Path, deep: &Path, start: usize, count: usize) -> Result<()> {
    write_grid(dir, deep, start, count, false)
}
pub(crate) fn write_grid(
    dir: &Path,
    deep: &Path,
    start: usize,
    count: usize,
    successor: bool,
) -> Result<()> {
    let end = start
        .checked_add(count)
        .filter(|v| count > 0 && *v <= 50)
        .ok_or("invalid deep onset range")?;
    for sub in ["cases", "bins", "frames"] {
        fs::create_dir_all(dir.join(sub))?;
    }
    for k in start..end {
        let name = format!("case-{k:03}");
        let f = fixture(source(k, true));
        let cut = survey::cuts(f)?.0[0].ok_or("missing primary cut")?;
        let norm =
            |v: &Value| -> Result<Value> { Ok(serde_json::from_slice(&serde_json::to_vec(v)?)?) };
        let (long, boundary, mechanics) = if successor {
            let (m, t) = super::whole_long::mechanical(f, &cut)?;
            (Value::Null, Some(t), m)
        } else {
            let long: Value = serde_json::from_reader(fs::File::open(
                deep.join("cases").join(format!("{name}.json")),
            )?)?;
            if long["source"] != source(k, true) || long["result"]["cut"] != norm(&snapshot(&cut))?
            {
                return Err("deep history uses another donor".into());
            }
            (long, None, Value::Null)
        };
        let base_tick = boundary.map_or(cut.tick, |t| t.tick);
        let mut runs = vec![];
        let mut labels = vec![];
        let mut clocks = std::collections::BTreeSet::from([0, COMMON]);
        for (i, n) in BUDGETS.into_iter().enumerate() {
            let r = if let Some(t) = boundary {
                let (mut sim, ids) = super::super::secondary::physical::cold_boundary(t)?;
                let mut resource = sim.world_mut().resource_mut::<PhysicsBackend>();
                let native = resource
                    .downcast_mut::<MultiFidelityBackend>()
                    .ok_or("requires registered world")?;
                let next = command(f, t.tick);
                if next != t.pad.velocity[2] {
                    native.set_world_velocity(ids[2], [0., 0., next], [0.; 3])?;
                }
                observe_grid(native, ids, n)?
            } else {
                observe(f, &cut, n)?
            };
            let last = *r.samples.last().unwrap();
            let control = deep
                .join(&name)
                .join("traces")
                .join(format!("native{n}.json"));
            let archived = if successor && control.exists() {
                let doc: Value = serde_json::from_reader(fs::File::open(control)?)?;
                if doc["frames"][0] != norm(&boundary.unwrap().as_json())? {
                    return Err("physical cut mismatch".into());
                }
                Some(doc["frames"][1].clone())
            } else if !successor {
                Some(long["result"]["paths"][i]["first"].clone())
            } else {
                None
            };
            if let Some(original) = &archived
                && (original["foot"] != norm(&r.receipt["public_foot"])?
                    || original["upper"] != norm(&r.receipt["public_upper"])?
                    || original["pad"] != norm(&r.receipt["public_pad"])?
                    || (original["normal_impulse_ns"].as_f64().ok_or("no load")? - last.normal)
                        .abs()
                        > 2e-10
                    || original["first_local_ns"] != r.first["end_ns"])
            {
                return Err(format!(
                    "fine observer changed original first public state at {k}/{n}"
                )
                .into());
            }
            for (j, s) in r.samples.iter().enumerate() {
                if s.ns != j as u64 * 500000 / COMMON as u64 {
                    return Err("non-nested grids".into());
                }
            }
            let event = r
                .samples
                .partition_point(|s| s.ns < r.first["end_ns"].as_u64().unwrap_or(0));
            clocks.extend(event.saturating_sub(3)..=(event + 10).min(COMMON));
            labels.push(json!({"first":r.first,"total":r.receipt}));
            runs.push(r);
        }
        let geom = geometry(kinematics(f, &cut)?.0)?;
        let mut pairs = vec![];
        for (i, j) in [(0, 1), (1, 2), (0, 2)] {
            let (a, b) = (&runs[i], &runs[j]);
            let mut first = [None; 8];
            let mut pre = [0.0_f64; 3];
            let mut peak = [0.0_f64; 3];
            let onset = a.first["end_ns"]
                .as_u64()
                .unwrap_or(u64::MAX)
                .min(b.first["end_ns"].as_u64().unwrap_or(u64::MAX));
            for (t, (x, y)) in a.samples.iter().zip(&b.samples).enumerate() {
                let d = x.center().distance(y.center());
                let v = x.velocity().distance(y.velocity());
                let impulse = x.impulse.distance(y.impulse);
                let spin =
                    DVec3::from(x.foot.angular_velocity).distance(y.foot.angular_velocity.into());
                let raw = [d, v, impulse];
                for (m, e) in raw.into_iter().enumerate() {
                    if x.ns < onset {
                        pre[m] = pre[m].max(e);
                    }
                    peak[m] = peak[m].max(e);
                }
                for (o, yes) in first.iter_mut().zip([
                    d > 1e-12,
                    d > 1e-9,
                    d > 1e-7,
                    d > 1e-6,
                    impulse > 1e-8,
                    impulse > 1e-3,
                    spin > 1e-4,
                    v > 1e-4,
                ]) {
                    if yes && o.is_none() {
                        *o = Some(x.ns);
                        clocks.extend(t.saturating_sub(1)..=(t + 1).min(COMMON));
                    }
                }
            }
            pairs.push(json!({"budgets":[BUDGETS[i],BUDGETS[j]],"first_common_bin_ns":first,"unloaded_peak_com_velocity_impulse":pre,"peak_com_velocity_impulse":peak}));
        }
        for j in 1..32 {
            clocks.insert(COMMON * j / 32);
        }
        let path = dir.join("bins").join(format!("{name}.bin"));
        let tmp = path.with_extension("partial");
        let mut file = BufWriter::new(fs::File::create(&tmp)?);
        for r in &runs {
            for s in &r.samples {
                for x in s.fields() {
                    file.write_all(&x.to_le_bytes())?;
                }
            }
        }
        file.flush()?;
        drop(file);
        fs::rename(tmp, path)?;
        let mut frames = vec![];
        for (i, r) in runs.iter().enumerate() {
            let mut track = vec![];
            let mut prev = 0;
            for &j in &clocks {
                let s = r.samples[j];
                let p = r.samples[prev];
                let ns = base_tick * 500000 + s.ns;
                track.push(json!({"tick":j,"time_us":ns/1000,"time_ns":ns,"foot":s.foot,"upper":s.upper,"pad":s.pad,"native_steps":(j-prev)*(BUDGETS[i] as usize/COMMON),"solver_iterations":4,"normal_impulse_ns":s.normal-p.normal,"world_impulse_ns":(s.impulse-p.impulse).to_array()}));
                prev = j;
            }
            frames.push(track);
        }
        write_json(
            &dir.join("frames").join(format!("{name}.json")),
            &json!(frames),
        )?;
        let output = dir.join("cases").join(format!("{name}.json"));
        let tmp = output.with_extension("partial");
        write_json(
            &tmp,
            &json!({"format":if successor{"physical-successor-onset-v1"}else{"nested-native-primary-onset-v1"},"boundary":boundary.map(|t|t.as_json()),"primary_mechanics":mechanics,"clock_origin_ns":base_tick*500000,"case":k,"source":source(k,true),"cut":snapshot(&cut),"fixture":f,"geometry":geom,"lanes":labels,"pairs":pairs,"keys":["1pm_com","1nm_com","100nm_com","1um_com","1e-8_Ns","1mNs","1e-4_spin","1e-4_speed"],"common_bins":COMMON}),
        )?;
        fs::rename(tmp, output)?;
        eprintln!("checked nested native grids {k}");
    }
    Ok(())
}
pub(crate) fn replay(dir: &Path, k: usize) -> Result<()> {
    let f: Value = serde_json::from_reader(fs::File::open(
        dir.join("frames").join(format!("case-{k:03}.json")),
    )?)?;
    let doc: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:03}.json")),
    )?)?;
    let title = if doc["format"] == "physical-successor-onset-v1" {
        "First native tick after an identical physical landing"
    } else {
        "First loaded native calls at the preceding primary landing"
    };
    let folder = dir.join(format!("case-{k:03}"));
    fs::create_dir_all(folder.join("traces"))?;
    let mut lanes = vec![];
    for (i, n) in BUDGETS.into_iter().enumerate() {
        let name = format!("traces/grid-{n}.json");
        write_json(
            &folder.join(&name),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":f[i]}),
        )?;
        lanes.push(
            json!({"label":format!("native{n}"),"name":format!("{n} native calls"),"file":name}),
        );
    }
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":title,"pair_totals_only":true,"sampled_ticks":true,"interval_samples":true,"public_dt_ns":500000,"clock_origin_ns":f[0][0]["time_ns"],"lanes":lanes}),
    )
}

/// Add independent physical histories on the same native bin ends. Use two
/// meshes at predeclared times after the first primary onset and at the clocks
/// saved for the within-tick native replay.
pub(crate) fn controls(dir: &Path, start: usize, count: usize) -> Result<()> {
    let last = start
        .checked_add(count)
        .filter(|x| count > 0 && *x <= 50)
        .ok_or("invalid deep onset range")?;
    fs::create_dir_all(dir.join("controls"))?;
    for k in start..last {
        let name = format!("case-{k:03}.json");
        let raw: Value = serde_json::from_reader(fs::File::open(dir.join("cases").join(&name))?)?;
        let saved: Value =
            serde_json::from_reader(fs::File::open(dir.join("frames").join(&name))?)?;
        let f = fixture(source(k, true));
        let c = survey::cuts(f)?.0[0].ok_or("no primary")?;
        let flight = kinematics(f, &c)?.0;
        let geo = geometry(flight)?;
        let successor = raw["format"] == "physical-successor-onset-v1";
        let offset = if successor { 500000u64 } else { 0 };
        let origin = c.tick * 500000 + offset;
        let arrival = if successor {
            if command(f, c.tick) != command(f, c.tick + 1) {
                return Err("mechanical successor crossed a command".into());
            }
            let end = super::super::super::finite::load::complete::at(flight, &geo, 0.001, 512)?;
            end["events"]
                .as_array()
                .ok_or("missing physical event list")?
                .iter()
                .filter_map(|p| p["at_s"].as_f64())
                .find(|s| *s > 0.0005)
                .map_or(0, |s| ((s - 0.0005) * 1e9).ceil() as u64)
        } else {
            geo["hi_ns"].as_u64().ok_or("unresolved geometric onset")?
        };
        let as_bin = |ns: u64| {
            ns.saturating_mul(COMMON as u64)
                .div_ceil(500000)
                .min(COMMON as u64)
        };
        let mut selected = std::collections::BTreeSet::new();
        let mut offsets = vec![];
        for after in [
            0, 64, 128, 256, 512, 1000, 2000, 4000, 8000, 16000, 32000, 64000,
        ] {
            let bin = as_bin(arrival + after);
            selected.insert(bin);
            offsets.push((after, bin));
        }
        for row in saved[0].as_array().ok_or("no replay clocks")? {
            selected.insert(row["tick"].as_u64().ok_or("no bin")?);
        }
        let mut samples = std::collections::BTreeMap::new();
        for bin in selected {
            let ns = 500000 * bin / COMMON as u64;
            let t = (ns + offset) as f64 / 1e9;
            let paths = [128, 512]
                .map(|n| super::super::super::finite::load::complete::at(flight, &geo, t, n))
                .into_iter()
                .collect::<Result<Vec<_>>>()?;
            samples.insert(bin, json!({"ns":ns,"paths":paths}));
        }
        let p = dir.join("controls").join(&name);
        let tmp = p.with_extension("partial");
        write_json(
            &tmp,
            &json!({"case":k,"source":source(k,true),"origin_ns":origin,"primary_offset_ns":offset,"geometry":geo,"deck_v":[0.,0.,command(f,c.tick)],"initial":saved[0][0],"raw_cut":raw["cut"],"offsets_ns_and_bins":offsets,"samples":samples}),
        )?;
        fs::rename(tmp, p)?;
    }
    Ok(())
}
pub(crate) fn physical_replay(dir: &Path, k: usize) -> Result<()> {
    replay(dir, k)?;
    let name = format!("case-{k:03}.json");
    let d: Value = serde_json::from_reader(fs::File::open(dir.join("controls").join(&name))?)?;
    let f: Value = serde_json::from_reader(fs::File::open(dir.join("frames").join(&name))?)?;
    let folder = dir.join(format!("case-{k:03}-mechanics"));
    fs::create_dir_all(folder.join("traces"))?;
    let mut lanes = vec![];
    let input = dir.join(format!("case-{k:03}"));
    for (i, n) in [(1, 16384), (2, 32768)] {
        let name = format!("traces/grid-{n}.json");
        fs::copy(input.join(&name), folder.join(&name))?;
        lanes.push(json!({"label":format!("native{n}"),"name":format!("{n} ordinary native calls"),"file":name}));
        if f[i][0] != f[0][0] {
            return Err("unequal native initial state".into());
        }
    }
    let vec3 =
        |v: &Value| -> Result<DVec3> { Ok(DVec3::from_array(serde_json::from_value(v.clone())?)) };
    let mut frames = vec![f[0][0].clone()];
    let offset = d["primary_offset_ns"].as_u64().unwrap_or(0);
    let start = &d["samples"]["0"]["paths"][1];
    let mut sum = if offset != 0 {
        vec3(&start["j"])?
    } else {
        DVec3::ZERO
    };
    let mut normal = if offset != 0 {
        start["jn"].as_f64().ok_or("no initial pressure")?
    } else {
        0.
    };
    let origin = d["origin_ns"].as_u64().ok_or("no original clock")?;
    let deck = vec3(&d["deck_v"])?;
    for time in f[0].as_array().ok_or("no sampled poses")?.iter().skip(1) {
        let bin = time["tick"].as_u64().ok_or("no common bin")?;
        let s = &d["samples"][bin.to_string()];
        let refn = &s["paths"][1];
        if refn["end"] != "flight" && refn["end"] != "original_clock" {
            return Err("missing physical sample".into());
        }
        let coarse = &s["paths"][0];
        if coarse["end"] != refn["end"]
            || vec3(&coarse["com"])?.distance(vec3(&refn["com"])?) > 1e-7
            || vec3(&coarse["j"])?.distance(vec3(&refn["j"])?) > 1e-5
        {
            return Err("physical replay sample differs between meshes".into());
        }
        let ns = s["ns"].as_u64().ok_or("no clock")?;
        let t = (ns + offset) as f64 / 1e9;
        let p = vec3(&refn["foot"])? + t * deck;
        let center = vec3(&refn["com"])? + t * deck;
        let w = vec3(&refn["omega"])?;
        let v = vec3(&refn["velocity"])? + deck;
        let b = |p: DVec3| json!({"position":p.to_array(),"orientation":refn["orientation"],"velocity":(v+w.cross(p-center)).to_array(),"angular_velocity":w.to_array()});
        let mut pad = f[0][0]["pad"].clone();
        pad["position"] = json!((vec3(&pad["position"])? + (ns as f64 / 1e9) * deck).to_array());
        pad["velocity"] = d["deck_v"].clone();
        let j = vec3(&refn["j"])?;
        let n = refn["jn"].as_f64().ok_or("no normal integral")?;
        frames.push(json!({"tick":bin,"time_us":(origin+ns)/1000,"time_ns":origin+ns,"foot":b(p),"upper":b(2.*center-p),"pad":pad,"native_steps":0,"solver_iterations":0,"normal_impulse_ns":n-normal,"world_impulse_ns":(j-sum).to_array()}));
        sum = j;
        normal = n;
    }
    write_json(
        &folder.join("traces/mechanics.json"),
        &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
    )?;
    lanes.push(json!({"label":"mechanics","name":"Piecewise rigid contact from matched flight","file":"traces/mechanics.json","analytical":true}));
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":if offset==0{"Finest native primary landings and independently integrated contact"}else{"First native tick after the same physical landing, with continuous mechanics"},"pair_totals_only":true,"sampled_ticks":true,"interval_samples":true,"public_dt_ns":500000,"clock_origin_ns":origin,"lanes":lanes}),
    )
}
