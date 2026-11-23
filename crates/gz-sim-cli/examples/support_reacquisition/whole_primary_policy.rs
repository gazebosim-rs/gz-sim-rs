// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Run separated primary donors through a complete landing in one live world.
use super::super::super::finite::{geometry, kinematics, load::complete};
use super::*;
use gz_physics_multifidelity::BoxContactActions;
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Method {
    Native,
    Endpoint,
    Spatial,
    Launch,
    Retimed,
    Subdivided,
    Compact,
    Directed { retimed: bool, compact: bool },
    SubdividedRetimed,
}
impl Method {
    pub(super) fn directed(self, on: bool) -> Self {
        if !on {
            return self;
        }
        match self {
            Self::Compact | Self::Subdivided | Self::SubdividedRetimed => Self::Directed {
                retimed: self == Self::SubdividedRetimed,
                compact: self == Self::Compact,
            },
            _ => self,
        }
    }
    fn subdivision(self, on: bool) -> Self {
        match (self, on) {
            (Self::Launch, true) => Self::Subdivided,
            (Self::Retimed, true) => Self::SubdividedRetimed,
            _ => self,
        }
    }
}
const MODES: [(&str, u32, Method); 10] = [
    ("native2", 2, Method::Native),
    ("launch2", 2, Method::Launch),
    ("native8", 8, Method::Native),
    ("endpoint8", 8, Method::Endpoint),
    ("spatial8", 8, Method::Spatial),
    ("launch8", 8, Method::Launch),
    ("retimed_launch8", 8, Method::Retimed),
    ("launch64", 64, Method::Launch),
    ("native4096", 4096, Method::Native),
    ("native8192", 8192, Method::Native),
];
pub(super) fn first(
    f: Fixture,
    cut: &ReboundFrame,
    method: Method,
    n: u32,
    tail: u32,
    last: u64,
) -> Result<(Value, Vec<Tick>)> {
    observed(f, cut, method, n, tail, last, None)
}
pub(super) fn observed(
    f: Fixture,
    cut: &ReboundFrame,
    method: Method,
    n: u32,
    tail: u32,
    last: u64,
    samples: Option<&mut Vec<Value>>,
) -> Result<(Value, Vec<Tick>)> {
    let mut rig = restart(f, ADAPTIVE, cut)?;
    let mut speed = rig.last_speed;
    let lane = &mut rig.case.lane;
    if lane.tier()? != Dynamic {
        return Err("expected matched dynamic cut".into());
    }
    let ids = [lane.foot, lane.upper, lane.pad];
    let [a, b, d] = ids;
    let mut res = lane.sim.world_mut().resource_mut::<PhysicsBackend>();
    let native = res
        .downcast_mut::<MultiFidelityBackend>()
        .ok_or("missing solver")?;
    let next = command(f, cut.tick);
    if next != speed {
        native.set_world_velocity(d, [0., 0., next], [0.; 3])?;
    }
    speed = next;
    let pair = [(a, d)];
    let active = !matches!(method, Method::Native | Method::Endpoint);
    let launch = active && method != Method::Spatial;
    let subdivide = matches!(
        method,
        Method::Subdivided | Method::SubdividedRetimed | Method::Compact | Method::Directed { .. }
    );
    let compact = matches!(
        method,
        Method::Compact | Method::Directed { compact: true, .. }
    );
    let retimed = matches!(
        method,
        Method::Retimed | Method::SubdividedRetimed | Method::Directed { retimed: true, .. }
    );
    let use_pair = |on: bool| if on { pair.as_slice() } else { &[] };
    let actions = BoxContactActions {
        finite_point_impacts: &pair,
        point_transfers: use_pair(active),
        sliding_skew_edges: use_pair(active),
        slip_launches: use_pair(launch),
        subdivide_point_bounds: use_pair(subdivide),
        compact_point_tails: use_pair(compact),
        retime_finite_transfer: use_pair(retimed),
        target_point_bounds: use_pair(matches!(method, Method::Directed { .. })),
        ..Default::default()
    };
    let params = ContactSolverIterations {
        temporal: 4,
        pgs: None,
        relaxation: Some(1),
    };
    let r = if let Some(frames) = samples {
        let origin = cut.tick * 500000;
        let init = json!({"tick":cut.tick,"time_ns":origin,"time_us":origin/1000,"foot":Body::from(native.state(a).ok_or("foot")?),"upper":Body::from(native.state(b).ok_or("upper")?),"pad":Body::from(native.state(d).ok_or("pad")?),"native_steps":0,"normal_impulse_ns":0.});
        super::super::secondary::bins::run(
            native,
            gz_physics_multifidelity::ContactSegment {
                duration: DT,
                native_steps: n,
                iterations: params,
            },
            ids,
            &init,
            (method != Method::Native).then_some(actions),
            frames,
        )?
        .0
    } else if method == Method::Native {
        native.step_contact_interval(DT, n, params)?
    } else {
        native
            .advance_box_contacts(DT, n, params, actions, None)?
            .interval
    };
    let mut normal = 0.;
    let mut tangent = DVec3::ZERO;
    let mut j = DVec3::ZERO;
    let mut first = None;
    let mut extra = 0.;
    for p in &r.pairs {
        if (p.a, p.b) == (a.min(d), a.max(d)) {
            let sign = if p.a == a { 1. } else { -1. };
            normal += p.normal_impulse;
            let t = sign
                * DVec3::from(
                    p.tangent_impulse_on_a
                        .ok_or("missing signed primary friction")?,
                );
            tangent += t;
            j += t + sign * DVec3::from(p.normal_impulse_on_a);
            first = p.first_positive_end;
        } else {
            extra += p.normal_impulse;
        }
    }
    let omitted = r.compact_point_tail.map_or(0, |s| s.omitted_native_steps);
    if r.native_steps.checked_add(omitted) != Some(n) || extra > 1e-12 {
        return Err("public grid or contact pair changed".into());
    }
    let foot = native.state(a).ok_or("foot")?;
    let pad = native.state(d).ok_or("pad")?;
    let tick = Tick {
        tick: cut.tick + 1,
        foot: foot.into(),
        upper: native.state(b).ok_or("upper")?.into(),
        pad: pad.into(),
        normal,
        tangent,
        first,
        extra,
        gap: supported_gap(foot, pad),
        calls: r.native_steps,
    };
    let mut spans = vec![];
    let mut hits = vec![];
    for h in &r.finite_point_impacts {
        let s = h.resultant;
        hits.push(json!({"at_s":s.elapsed.as_secs_f64(),"j":s.impulse_on_foot,"point":s.point,"normal":s.normal,"feature":format!("{:?}",h.feature),"stuck":s.sticking}));
    }
    for s in r.sliding_vertices.iter().chain(&r.sliding_skew_edges) {
        spans.push((
            s.start.as_nanos() as u64,
            s.duration.as_nanos() as u64,
            if s.slip_growth.is_some() {
                "growing_slip"
            } else {
                "sliding"
            },
        ));
    }
    for s in &r.sticking_points {
        spans.push((
            s.start.as_nanos() as u64,
            s.duration.as_nanos() as u64,
            "sticking",
        ));
    }
    for s in &r.point_transfers {
        spans.push((
            s.start.as_nanos() as u64,
            s.duration.as_nanos() as u64,
            "transfer",
        ));
        for p in &s.intervals {
            if let Some(e) = p.impact {
                hits.push(json!({"at_s":s.start.as_secs_f64()+e.local_bracket[0],"j":e.impulse_on_foot,"point":e.point,"normal":e.normal,"stuck":e.sticking}));
            }
        }
    }
    spans.sort_by_key(|s| s.0);
    hits.sort_by(|a, b| {
        a["at_s"]
            .as_f64()
            .unwrap()
            .total_cmp(&b["at_s"].as_f64().unwrap())
    });
    let mut resolved_j = DVec3::ZERO;
    let mut resolved_normal = 0.;
    for h in &r.finite_point_impacts {
        let x = h.resultant;
        let j = DVec3::from(x.impulse_on_foot);
        resolved_j += j;
        resolved_normal += j.dot(x.normal.into());
    }
    for s in r.sliding_vertices.iter().chain(&r.sliding_skew_edges) {
        resolved_j += DVec3::from(s.impulse_on_foot);
        resolved_normal += s.normal_impulse;
    }
    for s in &r.sticking_points {
        resolved_j += DVec3::from(s.impulse_on_foot);
        resolved_normal += s.normal_impulse;
    }
    for s in &r.point_transfers {
        resolved_j += DVec3::from(s.impulse_on_foot);
        resolved_normal += s.normal_impulse;
    }
    let mut row = json!({"first":tick.as_json(),"j":j.to_array(),"spans":spans,"hits":hits,"resolved_j":resolved_j.to_array(),"resolved_normal":resolved_normal});
    if compact {
        row["compact"]=json!(r.compact_point_tail.map(|s|json!({"start_ns":s.start.as_nanos(),"duration_ns":s.duration.as_nanos(),"omitted":s.omitted_native_steps})));
    }
    let mut ticks = vec![tick];
    let mut unexpected = vec![];
    for i in cut.tick + 2..=last {
        let v = command(f, i - 1);
        ticks.push(propagate_one(
            native,
            ids,
            i,
            super::super::b(tail, 1),
            (speed != v).then_some(v),
            None,
            &mut unexpected,
        )?);
        speed = v;
    }
    Ok((row, ticks))
}
fn collect(f: Fixture, subdivide: bool, directed: bool) -> Result<Value> {
    let (cuts, disjoint) = survey::cuts(f)?;
    let mut trials = vec![];
    for (k, c) in cuts.into_iter().enumerate() {
        let Some(c) = c else { continue };
        let (flight, defect) = kinematics(f, &c)?;
        if defect > 1e-8 {
            return Err("nonrigid launch".into());
        }
        let geo = geometry(flight)?;
        let mut paths = vec![];
        for (id, n, m) in MODES {
            let (mut row, _) = first(
                f,
                &c,
                m.subdivision(subdivide).directed(directed),
                n,
                n,
                c.tick + 1,
            )?;
            row["profile"] = json!(id);
            paths.push(row);
        }
        let refs = [128, 512]
            .map(|n| complete::at(flight, &geo, DT.as_secs_f64(), n))
            .into_iter()
            .collect::<Result<Vec<_>>>()?;
        trials.push(json!({"stroke":k+1,"cut":snapshot(&c),"geometry":geo,"deck_v":[0.,0.,command(f,c.tick)],"mechanics":refs,"paths":paths}));
    }
    Ok(json!({"fixture":f,"disjoint":disjoint,"trials":trials}))
}
pub(crate) fn write(
    dir: &Path,
    start: usize,
    count: usize,
    subdivide: bool,
    directed: bool,
) -> Result<()> {
    survey::write_cases(
        dir,
        start,
        count,
        if directed {
            "directed-original-primary-v1"
        } else if subdivide {
            "subdivided-original-primary-v1"
        } else {
            "continuous-original-primary-v1"
        },
        |f| collect(f, subdivide, directed),
    )
}
fn xy(v: &Value) -> Result<DVec3> {
    Ok(DVec3::from_array(serde_json::from_value(v.clone())?))
}
pub(crate) fn replay(dir: &Path, k: usize, stroke: usize, control: Option<&Path>) -> Result<()> {
    if k >= 1250 || !(1..=3).contains(&stroke) {
        return Err("invalid primary replay".into());
    }
    let d: Value = serde_json::from_reader(fs::File::open(
        dir.join("cases").join(format!("case-{k:04}.json")),
    )?)?;
    let t = d["result"]["trials"]
        .as_array()
        .ok_or("missing cuts")?
        .iter()
        .find(|z| z["stroke"] == stroke)
        .ok_or("missing selected landing")?;
    let f = survey::fixture(k);
    let c = survey::cuts(f)?.0[stroke - 1].ok_or("missing primary")?;
    let geo = &t["geometry"];
    let origin = c.tick * 500000;
    let folder = dir.join(format!("case-{k:04}-stroke-{stroke}"));
    let inside = folder.join("interval");
    for p in [&folder, &inside] {
        fs::create_dir_all(p.join("traces"))?;
    }
    let mut lanes = vec![];
    let mut window = vec![];
    let mut clocks = vec![];
    let mut initial = Value::Null;
    let save = |p: &Path,
                id: &str,
                label: &str,
                frames: Vec<Value>,
                analytic: bool|
     -> Result<Value> {
        let name = format!("traces/{id}.json");
        write_json(
            &p.join(&name),
            &json!({"geometry":{"foot_half_extents":FOOT_HALF,"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":PAD_HALF},"frames":frames}),
        )?;
        Ok(json!({"label":id,"name":label,"file":name,"analytical":analytic}))
    };
    let prior: Option<Value> = control
        .map(|p| -> Result<Value> {
            Ok(serde_json::from_reader(fs::File::open(
                p.join("cases").join(format!("case-{k:04}.json")),
            )?)?)
        })
        .transpose()?;
    let base = prior
        .as_ref()
        .map(|a| {
            a["result"]["trials"]
                .as_array()
                .ok_or("no baseline trials")?
                .iter()
                .find(|z| z["stroke"] == stroke)
                .ok_or("missing control landing")
        })
        .transpose()?;
    if base.is_some_and(|b| b["cut"] != t["cut"] || b["mechanics"] != t["mechanics"]) {
        return Err("control starts at another physical state".into());
    }
    let format = d["format"].as_str().ok_or("missing primary format")?;
    let directed = format == "directed-original-primary-v1";
    let subdivided = directed || format == "subdivided-original-primary-v1";
    let base_split = prior
        .as_ref()
        .is_some_and(|p| p["format"] == "subdivided-original-primary-v1");

    let left = if let Some(b) = base {
        (
            5,
            "control8",
            if base_split {
                "Dyadically subdivided material bounds"
            } else {
                "Uniform material bounds"
            },
            512,
            base_split,
            b,
        )
    } else {
        (
            3,
            "endpoint8",
            "Project the primary endpoint",
            512,
            false,
            t,
        )
    };
    let right = if subdivided {
        (
            5,
            "subdivided8",
            if directed {
                "Target the approaching material feature"
            } else {
                "Subdivide the material bounds"
            },
            512,
            true,
            t,
        )
    } else {
        (
            5,
            "launch8",
            "Follow primary and later changes of bearing",
            512,
            false,
            t,
        )
    };
    for (index, id, label, suffix, split, source) in [
        left,
        right,
        (
            9,
            "native8192",
            "Continuous 8,192 native calls",
            8192,
            false,
            t,
        ),
    ] {
        let (profile, n, m) = MODES[index];
        let mut samples = vec![];
        let (mut r, path) = observed(
            f,
            &c,
            m.subdivision(split)
                .directed(directed && std::ptr::eq(source, t)),
            n,
            suffix,
            c.tick + 26,
            (index != 9).then_some(&mut samples),
        )?;
        r["profile"] = json!(profile);
        if serde_json::from_slice::<Value>(&serde_json::to_vec(&r)?)? != source["paths"][index] {
            return Err("replay changed first public interval".into());
        }
        if !samples.is_empty() {
            let ns = samples
                .iter()
                .map(|s| s["time_ns"].as_u64().unwrap())
                .collect::<Vec<_>>();
            if clocks.is_empty() {
                clocks = ns;
                initial = samples[0].clone();
            } else if ns != clocks || initial != samples[0] {
                return Err("different native sample times".into());
            }
            window.push(save(&inside, id, label, samples, false)?);
        }
        let mut first = initial.clone();
        first["tick"] = json!(c.tick);
        let mut frames = vec![first];
        frames.extend(path.iter().map(Tick::as_json));
        lanes.push(save(&folder, id, label, frames, false)?);
    }
    let frames = physical_samples(f, &c, geo, &clocks, initial)?;
    window.push(save(
        &inside,
        "mechanics",
        "Piecewise rigid motion from the preceding primary cut",
        frames,
        true,
    )?);
    write_json(
        &inside.join("comparison.json"),
        &json!({"title":"Matched flight and changes of bearing over the entire primary landing","interval_samples":true,"public_dt_ns":500000,"sampled_ticks":true,"pair_totals_only":true,"clock_origin_ns":origin,"lanes":window}),
    )?;
    write_json(
        &folder.join("comparison.json"),
        &json!({"title":"Outer-support trajectory after the preceding primary landing","pair_totals_only":true,"lanes":lanes}),
    )
}

pub(super) fn physical_samples(
    f: Fixture,
    c: &ReboundFrame,
    geo: &Value,
    clocks: &[u64],
    initial: Value,
) -> Result<Vec<Value>> {
    let origin = c.tick * 500000;
    let v = DVec3::Z * command(f, c.tick);
    let mut frames = vec![initial];
    let mut prev_j = DVec3::ZERO;
    let mut prev_n = 0.;
    let flight = kinematics(f, c)?.0;
    for (i, &ns) in clocks.iter().enumerate().skip(1) {
        let dt = (ns - origin) as f64 / 1e9;
        let a = complete::at(flight, geo, dt, 512)?;
        if a["end"] != "original_clock" && a["end"] != "flight" {
            return Err("unresolved requested physical sample".into());
        }
        let center = xy(&a["com"])? + dt * v;
        let w = xy(&a["omega"])?;
        let com_v = xy(&a["velocity"])? + v;
        let foot = xy(&a["foot"])? + dt * v;
        let orient: [f64; 4] = serde_json::from_value(a["orientation"].clone())?;
        let rigid = |p: DVec3| Body {
            position: p.to_array(),
            orientation: orient,
            velocity: (com_v + w.cross(p - center)).to_array(),
            angular_velocity: w.to_array(),
        };
        let j = xy(&a["j"])?;
        let n = a["jn"].as_f64().ok_or("unknown reference normal")?;
        let mut pad = c.pad;
        pad.position = (DVec3::from(c.pad.position) + v * dt).to_array();
        pad.velocity = v.to_array();
        frames.push(json!({"tick":i,"time_ns":ns,"time_us":ns/1000,"foot":rigid(foot),"upper":rigid(2.*center-foot),"pad":pad,"native_steps":0,"solver_iterations":0,"normal_impulse_ns":n-prev_n,"world_impulse_ns":(j-prev_j).to_array()}));
        prev_j = j;
        prev_n = n;
    }
    Ok(frames)
}
