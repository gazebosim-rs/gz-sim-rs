// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Public multi-object scenes, competitors and a real third body entering after touchdown.
use super::*;
use gz_physics::DynamicBackend;
#[path = "inertia_opponent.rs"]
pub(crate) mod named;

const HALF_GATE: [f64; 3] = [0.48, 0.42, 0.025];
fn deck(center: DVec3, q: DQuat, v: DVec3, half: [f64; 3]) -> String {
    let geom = format!(
        "<geometry><box><size>{}</size></box></geometry>",
        scene::coords(2. * DVec3::from(half))
    );
    format!(
        r#"<model name="opponent"><pose>{} {}</pose><plugin name="gz::sim::systems::PrescribedMotion" filename="gz-sim-prescribed-motion-system"><linear_velocity>{}</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin><link name="gate"><collision name="bar">{geom}</collision><visual name="bar">{geom}</visual></link></model>"#,
        scene::coords(center),
        scene::rpy(q),
        scene::coords(v)
    )
}
fn insert(xml: &str, models: &str) -> String {
    xml.replace("</world>", &format!("{models}</world>"))
}
fn clutter(xml: &str, n: usize) -> String {
    let mut models = deck(
        DVec3::new(0.90, -0.26, 0.18),
        DQuat::from_rotation_z(0.22),
        DVec3::new(-1.5, 0.4, 0.),
        [0.2, 0.19, 0.09],
    );
    for j in 0..n {
        let theta = j as f64 * 2.399_963_229_728_653;
        let r = 0.83 + 0.16 * (j % 4) as f64;
        let z = 0.09 * ((j % 5) as f64 - 2.);
        models.push_str(&format!(r#"<model name="shelf{j}"><static>true</static><pose>{} {} {z} 0 0 {theta}</pose><link name="table"><collision name="board"><geometry><box><size>0.22 0.19 0.07</size></box></geometry></collision></link></model>"#,r*theta.cos(),r*theta.sin()));
    }
    insert(xml, &models)
}
fn policy(i: &Impact, clear: bool) -> String {
    let text = Study::Departure.text(i, 8, true, false);
    if clear {
        text.replace(
            "<project_rigid_corner>true",
            "<project_rigid_corner>clear_neighbors",
        )
    } else {
        text
    }
}
fn cases() -> Vec<Impact> {
    let mut out = vec![];
    for (lo, hi) in [(0, 384), (1344, 1800)] {
        let mut count = [0; 4];
        for id in lo..hi {
            for (k, num) in count.iter_mut().enumerate() {
                if *num >= 16 {
                    continue;
                }
                let i = manufactured(id, k);
                if i.unique_margin > 1e-8
                    && i.loss > 0.
                    && i.r.closing > 0.05
                    && i.r.certify().reason.is_none()
                {
                    out.push(i);
                    *num += 1;
                }
            }
            if count == [16; 4] {
                break;
            }
        }
        assert_eq!(count, [16; 4]);
    }
    out
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut rows = vec![];
    for i in cases() {
        let mut runs = vec![];
        for count in [0, 12, 48] {
            for clear in [false, true] {
                let base = policy(&i, clear);
                let xml = if count > 0 {
                    clutter(&base, count)
                } else {
                    base
                };
                let (r, _) = run(&i, PROFILES[1], &xml)?;
                runs.push(json!({"static_shelves":count,"clear_neighbors":clear,"run":r}));
            }
        }
        rows.push(json!({"reference":law(&i),"trials":runs}));
    }
    write_json(&path.join("study.json"), &json!({"cases":rows}))?;
    handoffs(path)?;
    competitors(path)
}
fn process_cpu_ns() -> Option<u64> {
    #[cfg(target_os = "linux")]
    {
        let t = rustix::time::clock_gettime(rustix::time::ClockId::ProcessCPUTime);
        Some(t.tv_sec as u64 * 1_000_000_000 + t.tv_nsec as u64)
    }
    #[cfg(not(target_os = "linux"))]
    {
        None
    }
}
struct Sample {
    wall: Vec<u64>,
    cpu: Vec<Option<u64>>,
}
fn costed(text: &str, end: State) -> Result<Sample> {
    let (mut sim, ids) = load(text, false)?;
    let mut sample = Sample {
        wall: Vec::with_capacity(8),
        cpu: Vec::with_capacity(8),
    };
    for _ in 0..8 {
        let cpu = process_cpu_ns();
        let start = Instant::now();
        sim.step_exact(1)?;
        sample.wall.push(start.elapsed().as_nanos() as u64);
        sample.cpu.push(
            process_cpu_ns()
                .zip(cpu)
                .map(|(after, before)| after - before),
        );
    }
    if state(backend(&sim), ids)? != end {
        return Err("timed obstacle state mismatch".into());
    }
    Ok(sample)
}
pub(crate) fn cost(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    let mut out = vec![];
    for i in cases() {
        let mut lanes = vec![];
        for count in [0, 12, 48] {
            for clear in [false, true] {
                let base = policy(&i, clear);
                let xml = if count > 0 {
                    clutter(&base, count)
                } else {
                    base
                };
                let (mut sim, ids) = load(&xml, false)?;
                sim.step_exact(8)?;
                let end = state(backend(&sim), ids)?;
                clocked(&xml, 8, end)?;
                lanes.push((count, clear, xml, end));
            }
        }
        let mut times: [Vec<Sample>; 6] = Default::default();
        for rep in 0..5 {
            for j in 0..6 {
                let k = (i.r.id + i.mode + rep + if rep % 2 == 0 { j } else { 5 - j }) % 6;
                times[k].push(costed(&lanes[k].2, lanes[k].3)?);
            }
        }
        for ((n, clear, _, _), measured) in lanes.into_iter().zip(times) {
            out.push(json!({"id":i.r.id,"mode":MODES[i.mode],"shelves":n,"clear":clear,"steps_ns":measured.iter().map(|s|&s.wall).collect::<Vec<_>>(),"process_cpu_ns":measured.iter().map(|s|&s.cpu).collect::<Vec<_>>()}));
        }
    }
    write_json(&path.join("cost.json"), &json!({"rows":out}))
}
fn id_named(sim: &Simulation, name: &str) -> Result<BodyId> {
    #[allow(deprecated)]
    sim.world()
        .iter_entities()
        .find_map(|e| {
            (e.get::<Name>()?.0 == name)
                .then(|| e.get::<BodyHandle>().map(|h| h.id()))
                .flatten()
        })
        .ok_or_else(|| format!("missing {name}").into())
}
fn body_view(s: BodyState) -> Value {
    json!({"position":s.position,"orientation":s.orientation,"velocity":s.velocity,"angular_velocity":s.angular_velocity})
}
struct Gate {
    xml: String,
    center: DVec3,
    velocity: DVec3,
    at: f64,
    speed: f64,
}
// The lowest point of the ceiling lies at the highest vertex of the
// reference foot at the chosen crossing. Its downward speed exceeds the
// translational and angular bounds on every point before that crossing.
fn gate(i: &Impact, delay: f64) -> Gate {
    let r = &i.r;
    let at = r.arrival + delay;
    let s = r.state(at);
    let q = DQuat::from_array(s.foot.orientation);
    let foot = DVec3::from(s.foot.position);
    let cap = corners()
        .map(|mut p| {
            p.z = FOOT_HALF[2];
            (q * p).dot(r.n)
        })
        .into_iter()
        .fold(f64::NEG_INFINITY, f64::max);
    let mut bound = 0f64;
    for b in [r.pre, r.post] {
        let v = (b.v - r.pad_v).length() + r.gravity.length() * WINDOW;
        let spin = r.body.omega(r.q, b.l);
        let w = (spin.dot(b.l) / r.body.lower)
            .sqrt()
            .min(b.l.length() / r.body.lower);
        bound = bound.max(v + 0.32 * w);
    }
    let speed = bound + 7.;
    let velocity = r.pad_v - speed * r.n;
    let center = foot + (cap + HALF_GATE[2]) * r.n;
    Gate {
        center: center - at * velocity,
        velocity,
        xml: deck(center - at * velocity, r.pad_q, velocity, HALF_GATE),
        at,
        speed,
    }
}
#[derive(Clone, Copy, Default)]
struct ProjectedContacts {
    primary: Option<gz_physics_multifidelity::RigidPointImpactReceipt>,
    upper: Option<gz_physics_multifidelity::OpposingCornerReceipt>,
    returned: Option<gz_physics_multifidelity::OpposingReturnReceipt>,
}
struct Observed {
    end: u64,
    foot: BodyState,
    gate: BodyState,
    projection: Option<gz_physics_multifidelity::RigidPointImpactReceipt>,
    upper: Option<gz_physics_multifidelity::OpposingCornerReceipt>,
    returned:Option<gz_physics_multifidelity::OpposingReturnReceipt>,
    corners: [f64; 4],
    opponent: f64,
    hit: Option<(u64, DVec3)>,
}
fn observe(
    cut: gyro::Cut,
    ids: Bodies,
    gate: BodyId,
    gate_state: BodyState,
    patches: Vec<RigidContactPatch>,
    points:ProjectedContacts,
) -> Observed {
    let ProjectedContacts{primary:hit,upper,returned}=points;
    let mut opponent: f64 = patches
        .iter()
        .filter(|p| (p.a == ids.foot && p.b == gate) || (p.a == gate && p.b == ids.foot))
        .flat_map(|p| p.points.iter())
        .map(|p| p.normal_impulse)
        .sum();
    if let Some(h) = upper {
        assert_eq!(h.point.impact.support, gate);
        opponent += DVec3::from(h.point.impact.impulse_on_foot).dot(h.point.impact.normal.into());
    }
    let mut corners = measured(cut, ids, patches).corners;
    if let Some(r)=returned{let h=r.lower.point;
        assert_eq!(h.impact.support,ids.pad);
        corners[usize::from(h.corner)]+=DVec3::from(h.impact.impulse_on_foot).dot(h.impact.normal.into());
    }
    let projected = hit.map(|h| {
        let impulse = DVec3::from(h.impact.impulse_on_foot);
        corners[usize::from(h.corner)] += impulse.dot(h.impact.normal.into());
        (h.impact.elapsed.as_nanos() as u64, impulse)
    });
    Observed {
        end: cut.ns,
        foot: cut.foot,
        gate: gate_state,
        projection: hit,
        upper,
        returned,
        corners,
        opponent,
        hit: projected,
    }
}
fn collect_gate(i: &Impact, xml: &str) -> Result<(Value, Vec<Value>)> {
    measure_gate(i, xml, false)
}
fn measure_gate(i: &Impact, xml: &str, audit: bool) -> Result<(Value, Vec<Value>)> {
    let (mut sim, ids) = load(xml, false)?;
    let gate = id_named(&sim, "default::opponent::gate")?;
    let stream = Arc::new(Mutex::new(Vec::new()));
    let seen = stream.clone();
    set_refined_observer(&mut sim, move |_, s| {
        let cut = gyro::Cut {
            foot: s.state(ids.foot).unwrap(),
            pad: s.state(ids.pad).unwrap(),
            ns: s.elapsed.as_nanos() as u64,
        };
        seen.lock().unwrap().push(observe(
            cut,
            ids,
            gate,
            s.state(gate).unwrap(),
            s.contact_patches(),
            ProjectedContacts{primary:s.rigid_point_impact,upper:s.opposing_corner_impact,returned:s.opposing_return_impact.copied()},
        ));
    })?;
    let snapshot = |sim: &Simulation, t: u64, count: u32, corner: [f64; 4]| -> Result<Value> {
        let mut v = serde_json::to_value(frame(
            t,
            state(backend(sim), ids)?,
            count,
            count > 1,
            corner,
        ))?;
        v.as_object_mut().unwrap().remove("upper");
        v["opponent"] = body_view(backend(sim).state(gate).unwrap());
        Ok(v)
    };
    let mut frames = vec![snapshot(&sim, 0, 0, [0.; 4])?];
    let mut ticks = vec![];
    let mut applications = vec![];
    let mut first_gate = None;
    let mut spans = Vec::new();
    let mut previous_gate = backend(&sim).state(gate).unwrap();
    let mut previous = backend(&sim).state(ids.foot).unwrap();
    for t in 1..=8 {
        sim.step_exact(1)?;
        let engine = backend(&sim);
        let mut steps = std::mem::take(&mut *stream.lock().unwrap());
        if steps.is_empty() {
            steps.push(observe(
                gyro::Cut::bodies(engine, ids, 500_000),
                ids,
                gate,
                engine.state(gate).unwrap(),
                engine.rigid_contact_patches(),
                gz_sim_scene::primary_contact::last_opposing_corner(&sim)
                    .filter(|r|r.sim_time==sim.elapsed()).map_or_else(ProjectedContacts::default,|r|ProjectedContacts{primary:None,upper:r.interval.opposing_corner_impacts.first().copied(),returned:r.interval.opposing_return_impacts.first().copied()}),
            ));
        }
        if audit {
            let log = last_box_flight(&sim).filter(|v| v.sim_time == sim.elapsed());
            let guards = log.and_then(|v| v.flight.opposing.as_ref());
            if let Some(guards) = guards {
                assert_eq!(guards.len(), steps.len());
            }
            let mut start = 0;
            let mut foot_state = previous;
            for (n, s) in steps.iter().enumerate() {
                let cert = guards.and_then(|v| v[n]);
                if let Some(c) = cert {
                    assert_eq!(c.opponent, gate);
                }
                spans.push(named::inspect(
                    i,
                    (t - 1) * 500000,
                    start,
                    s,
                    foot_state,
                    previous_gate,
                    cert,
                ));
                start = s.end;
                foot_state = s.foot;
                previous_gate = s.gate;
            }
        }
        let mut primary = [0.; 4];
        let mut gate_normal = 0.;
        for s in &steps {
            for (k, x) in s.corners.iter().enumerate() {
                primary[k] += x;
            }
            if s.opponent > 0. {
                first_gate.get_or_insert((t - 1) * 500_000 + s.end);
            }
            gate_normal += s.opponent;
            if let Some((elapsed, j)) = s.hit {
                applications.push(json!({"at_ns":(t-1)*500000+elapsed,"world":j.to_array()}));
            }
        }
        let log = last_box_timing(&sim).filter(|l| l.sim_time == sim.elapsed());
        if let Some(log) = log {
            for (id, j) in [(ids.pad, primary.iter().sum()), (gate, gate_normal)] {
                let native = log
                    .interval
                    .pairs
                    .iter()
                    .find(|x| (x.a, x.b) == (id.min(ids.foot), id.max(ids.foot)))
                    .map_or(0., |p| p.normal_impulse);
                assert!((native - j).abs() < 1e-10);
            }
        }
        let current = engine.state(ids.foot).unwrap();
        let impulse = M
            * (DVec3::from(current.velocity)
                - DVec3::from(previous.velocity)
                - i.r.gravity * 0.0005);
        let exact = i.r.state(t as f64 * 0.0005);
        let actual = state(engine, ids)?;
        ticks.push(json!({"tick":t,"calls":steps.len(),"primary_normal_ns":primary.iter().sum::<f64>(),"opponent_normal_ns":gate_normal,"mean_opponent_force_n":gate_normal/0.0005,"net_foot_impulse":impulse.to_array(),"unopposed_reference_com_m":exact.com().distance(actual.com()),"timing":log.map(|l|json!({"decline":l.timing.decline.map(|d|format!("{d:?}")),"decline_after":l.timing.decline_after,"crossings":l.timing.crossings.len(),"cubic_queries":l.timing.cubic_queries,"pulses":l.timing.pulses.iter().map(|p|json!({"cut":p.cut_ns,"applied":p.applied_ns,"decline":p.decline.map(|d|format!("{d:?}"))})).collect::<Vec<_>>(),"point_attempts":l.timing.point_attempts.as_ref().map(|v|v.iter().map(|(at,d)|json!([at,d.map(|d|format!("{d:?}"))])).collect::<Vec<_>>())})),
                    "guards":last_box_flight(&sim).filter(|v|v.sim_time==sim.elapsed()).map(|v|v.flight.guarded.clone()), "spans":log.and_then(|s|s.timing.departure_calls.as_ref()).map(|spans|spans.iter().map(|(a,h)|[(t-1)*500000+a.as_nanos()as u64,(t-1)*500000+( *a+*h).as_nanos()as u64]).collect::<Vec<_>>())}));
        if audit {
            let sweep = gz_sim_scene::primary_contact::last_box_sweep(&sim)
                .filter(|s| s.sim_time == sim.elapsed());
            if let Some(record) = gz_sim_scene::primary_contact::last_opposing_corner(&sim)
                .filter(|v| v.sim_time == sim.elapsed())
            {
                let pair = (ids.foot.min(gate), ids.foot.max(gate));
                let total = record
                    .interval
                    .pairs
                    .iter()
                    .filter(|p| (p.a, p.b) == pair)
                    .map(|p| p.normal_impulse)
                    .sum::<f64>();
                assert!((total - gate_normal).abs() < 1e-9);
            }
            ticks.last_mut().unwrap()["public_sweep"] = json!(
                sweep
                    .and_then(|s| s.pairs.iter().find(|p| (p.0, p.1) == (ids.foot, ids.pad)))
                    .map(|p| format!("{:?}", p.2))
            );
        }
        frames.push(snapshot(&sim, t, steps.len() as u32, primary)?);
        previous = current;
    }
    let mut result =
        json!({"applications":applications,"first_opponent_ns":first_gate,"ticks":ticks});
    if audit {
        result["native_intervals"] = json!(spans);
    }
    Ok((result, frames))
}
fn handoffs(path: &Path) -> Result<()> {
    let mut rows = vec![];
    for (id, mode) in [(1160, 3), (1353, 3)]
        .into_iter()
        .chain(cases().into_iter().take(30).map(|i| (i.r.id, i.mode)))
    {
        let i = manufactured(id, mode);
        for delay in [-0.00018, 0.00075, 0.0011] {
            let g = gate(&i, delay);
            let mut trials = vec![];
            for clear in [false, true] {
                let text = insert(&policy(&i, clear), &g.xml);
                let (row, frames) = collect_gate(&i, &text)?;
                if id == 20 && mode == 3 && delay == 0.00075 {
                    let name = if clear { "neighbors" } else { "isolated" };
                    fs::write(path.join(format!("{name}.sdf")), &text)?;
                    write_json(
                        &path.join("traces").join(format!("{name}.json")),
                        &json!({"geometry":{"foot_half_extents":FOOT_HALF,"pad_half_extents":PAD_HALF,"foot_com_local":ARM.to_array(),"opponent_half_extents":HALF_GATE},"frames":frames}),
                    )?;
                }
                trials.push(json!({"clear":clear,"run":row}));
            }
            rows.push(json!({"id":id,"mode":MODES[mode],"primary_ns":(i.r.arrival*1e9).round()as u64,"gate_ns":(g.at*1e9).round()as u64,"delay_ns":(delay*1e9).round()as i64,"gate_closing_speed_bound":g.speed,"reference":law(&i),"trials":trials}));
        }
    }
    write_json(&path.join("handoff.json"), &json!({"cases":rows}))?;
    write_json(
        &path.join("comparison.json"),
        &json!({"single_body":true,"title":"Handing off from the sole to a moving ceiling","capture_ticks":{"primary_landing":6,"first_ceiling_contact":7},"lanes":[{"label":"isolated","name":"Two-collider-only projection","file":"traces/isolated.json"},{"label":"neighbors","name":"Swept third-body clearance","file":"traces/neighbors.json"}]}),
    )
}
fn competitors(path: &Path) -> Result<()> {
    let mut rows = vec![];
    for (id, mode) in cases().into_iter().take(24).map(|i| (i.r.id, i.mode)) {
        let case = manufactured(id, mode);
        for masked in [false, true] {
            let mut trials = vec![];
            for clear in [false, true] {
                let text = compact::with_competitor(&policy(&case, clear), 0.0515);
                let (mut sim, ids) = load(&text, false)?;
                let witness = id_named(&sim, "default::payload::block")?;
                let floor = id_named(&sim, "default::payload_base::base")?;
                if masked {
                    let mut phy = sim.world_mut().resource_mut::<PhysicsBackend>();
                    let e = phy.downcast_mut::<MultiFidelityBackend>().unwrap();
                    for id in [ids.foot, witness] {
                        e.set_self_collision_group(id, Some(19))?;
                    }
                }
                let mut ticks = vec![];
                let mut projected = 0;
                for t in 1..=8 {
                    let force = [0.4, 0.1, -0.8];
                    let mut phy = sim.world_mut().resource_mut::<PhysicsBackend>();
                    phy.downcast_mut::<MultiFidelityBackend>()
                        .unwrap()
                        .add_world_wrench(witness, force, [0.; 3])?;
                    sim.step_exact(1)?;
                    let e = backend(&sim);
                    let log = last_box_timing(&sim).filter(|l| l.sim_time == sim.elapsed());
                    let pair = (floor.min(witness), floor.max(witness));
                    let normal = if let Some(l) = log {
                        projected += l.interval.rigid_point_impacts.len();
                        l.interval
                            .pairs
                            .iter()
                            .find(|r| (r.a, r.b) == pair)
                            .map_or(0., |p| p.normal_impulse)
                    } else {
                        e.rigid_contact_observations()
                            .iter()
                            .filter(|r| (r.a, r.b) == pair)
                            .map(|r| r.normal_impulse)
                            .sum()
                    };
                    ticks.push(json!({"tick":t,"force_n":force,"witness":body_view(e.state(witness).unwrap()),"floor_ns":normal,"foot":body_view(e.state(ids.foot).unwrap()),"grid_ns":log.map(|l|l.timing.durations.iter().map(|h|h.as_nanos()as u64).collect::<Vec<_>>())}));
                }
                trials.push(json!({"clear":clear,"projected":projected,"ticks":ticks}));
            }
            rows.push(json!({"id":id,"mode":MODES[mode],"masked":masked,"trials":trials}));
        }
    }
    write_json(&path.join("active.json"), &json!({"cases":rows}))
}
