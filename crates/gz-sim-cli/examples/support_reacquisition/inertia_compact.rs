// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Measure controlled landing work from the same certified initial states.
use super::*;

const PROFILES: [(u32, bool); 6] = [
    (3, false),
    (4, false),
    (8, false),
    (16, false),
    (8, true),
    (16, true),
];
fn configured(r: &Physical, n: u32, on: bool) -> (Profile, String) {
    let (p, mut text) = xml(r, n, true);
    if on {
        text = text.replace(
            "</detect_separated_crossings>",
            "</detect_separated_crossings><compact_impact_budget>true</compact_impact_budget>",
        );
    }
    (p, text)
}
pub(crate) fn write(path: &Path) -> Result<()> {
    fs::create_dir_all(path.join("traces"))?;
    let mut all = vec![];
    let mut feature = None;
    for id in (0..320).chain(640..960) {
        let r = Physical::new(id);
        if r.certify().reason.is_some() {
            continue;
        }
        let mut trials = vec![];
        for (n, on) in PROFILES {
            let (p, text) = configured(&r, n, on);
            let (row, _, _) = scene::run(&r, false, p, &text)?;
            let nominations:Vec<_>=row["ticks"].as_array().unwrap().iter().filter(|t|t["calls"]!=1).map(|t|json!({"tick":t["tick"],"calls":t["calls"],"compact":t["compact"],"end_ns":t["native_end_ns"],"loads":t["native_normal_ns"]})).collect();
            if feature.is_none()
                && id >= 640
                && on
                && n == 8
                && nominations.iter().any(|e| e["compact"]["complete"] == true)
            {
                feature = Some(id);
            }
            trials.push(json!({"budget":n,"compact":on,"native_calls":row["native_calls"],"first_ns":row["first_positive_end_ns"],"early_ns":row["preimpact_measured_ns"],"metrics":row["metrics"],"nominated":nominations}));
        }
        all.push(json!({"id":id,"arrival_ns":(r.arrival*1e9).round()as u64,"trials":trials}));
    }
    write_json(
        &path.join("single.json"),
        &json!({"cases":all,"view_id":feature}),
    )?;
    if let Some(id) = feature {
        let r = Physical::new(id);
        for (name, on) in [("full", false), ("compact", true)] {
            let (p, text) = configured(&r, 8, on);
            let (_, frames, _) = scene::run(&r, false, p, &text)?;
            separated::single_rigid_trace(path, name, &frames)?;
            fs::write(path.join(format!("{name}.sdf")), text)?;
        }
        separated::single_rigid_trace(path, "analytic", &r.trace())?;
        write_json(
            &path.join("comparison.json"),
            &json!({"single_body":true,"title":"Certifying the separated tail of a rotating landing","lanes":[{"name":"Full eight-call landing","label":"full","file":"traces/full.json"},{"name":"Guarded compact landing","label":"compact","file":"traces/compact.json"},{"name":"Independent Euler and contact","label":"analytic","file":"traces/analytic.json","analytical":true}]}),
        )?;
    }
    let mut returns = vec![];
    for id in 0..DONORS {
        let r = donor(id);
        let h = History::solve(&r, RK_H);
        let n = ((h.end + 1e-13) * 2000.).floor() as u64;
        if h.events
            .iter()
            .filter(|e| e.time_s <= n as f64 * 0.0005)
            .count()
            < 2
        {
            continue;
        }
        let mut trials = vec![];
        for (budget, on) in PROFILES {
            let (_, text) = configured(&r, budget, on);
            let (row, _) = run(&r, &h, &text, n)?;
            trials.push(json!({"budget":budget,"compact":on,"trial":row}));
        }
        returns.push(json!({"id":id,"events":h.events,"ticks":n,"trials":trials}));
    }
    write_json(&path.join("returns.json"), &json!({"cases":returns}))?;
    contention(path, 0)
}
pub(crate) fn cost(folder: &Path) -> Result<()> {
    fs::create_dir_all(folder)?;
    let data: Value = serde_json::from_slice(&fs::read(
        folder
            .parent()
            .ok_or("need output subdirectory")?
            .join("returns.json"),
    )?)?;
    let mut rows = vec![];
    for c in data["cases"].as_array().unwrap() {
        let id = c["id"].as_u64().unwrap() as usize;
        let n = c["ticks"].as_u64().unwrap();
        let r = donor(id);
        let mut controls = vec![];
        for (budget, on) in PROFILES {
            let (_, text) = configured(&r, budget, on);
            let (mut sim, ids) = sdf_features::load_finite_box(&text, false)?;
            sim.step_exact(n)?;
            let end = state(backend(&sim), ids)?;
            clocked(&text, n, end)?;
            controls.push((budget, on, text, end));
        }
        let mut measurements: [Vec<Vec<u64>>; 6] = Default::default();
        for rep in 0..4 {
            for k in 0..controls.len() {
                let i = (if rep % 2 == 0 {
                    k
                } else {
                    controls.len() - 1 - k
                } + id
                    + rep)
                    % controls.len();
                measurements[i].push(clocked(&controls[i].2, n, controls[i].3)?);
            }
        }
        for ((budget, on, _, _), times) in controls.into_iter().zip(measurements) {
            rows.push(json!({"id":id,"budget":budget,"compact":on,"steps_ns":times}));
        }
    }
    write_json(&folder.join("cost.json"), &json!({"rows":rows}))?;
    contention(folder, 12)
}

pub(super) fn with_competitor(text: &str, z: f64) -> String {
    let models = format!(
        r#"<model name="payload"><pose>4 0 {z} 0 0 0</pose>
      <plugin name="gz::sim::systems::InitialVelocity" filename="gz-sim-initial-velocity-system"><linear_velocity>0.2 0 -1</linear_velocity><angular_velocity>0 0 0</angular_velocity></plugin>
      <link name="block"><inertial><mass>2</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
        <collision name="box"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link></model>
      <model name="payload_base"><static>true</static><pose>4 0 -0.05 0 0 0</pose>
        <link name="base"><collision name="box"><geometry><box><size>0.6 0.6 0.1</size></box></geometry></collision></link></model>"#
    );
    text.replace("</world>", &(models + "</world>"))
}
struct CompetingRun {
    path: Vec<(BodyState, BodyState, BodyState, Vec<Duration>, f64)>,
    elapsed: Vec<u64>,
    selected: Vec<u64>,
}
fn competing_run(text: &str, enabled: bool) -> Result<CompetingRun> {
    let (mut sim, ids) = sdf_features::load_finite_box(text, false)?;
    let body = |name: &str| -> Result<BodyId> {
        #[allow(deprecated)]
        sim.world()
            .iter_entities()
            .find_map(|e| {
                (e.get::<Name>()?.0 == name)
                    .then(|| e.get::<BodyHandle>().map(|b| b.id()))
                    .flatten()
            })
            .ok_or_else(|| format!("missing {name}").into())
    };
    let w = body("default::payload::block")?;
    let b = body("default::payload_base::base")?;
    // The two remote moving bodies cannot exchange impulses. Their own
    // supports remain collidable in the same native world.
    {
        use gz_physics::DynamicBackend;
        let mut backend = sim.world_mut().resource_mut::<PhysicsBackend>();
        let native = backend.downcast_mut::<MultiFidelityBackend>().unwrap();
        for id in [w, ids.foot] {
            native.set_self_collision_group(id, Some(19))?;
        }
    }
    let mut out = CompetingRun {
        path: vec![],
        elapsed: vec![],
        selected: vec![],
    };
    for tick in 1..=8u64 {
        let t = Instant::now();
        sim.step_exact(1)?;
        out.elapsed.push(t.elapsed().as_nanos() as u64);
        let native = backend(&sim);
        let stamp = sim.elapsed();
        let time = last_box_timing(&sim).filter(|x| x.sim_time == stamp);
        let (durations, reaction) = if let Some(selected) = time {
            out.selected.push(tick);
            assert_eq!(selected.interval.native_steps, 8);
            let mark = selected.timing.compact;
            assert_eq!(
                mark.map(|x| x.decline),
                enabled.then_some(Some(
                    gz_physics_multifidelity::CompactDecline::AdditionalDynamicBody
                ))
            );
            (
                selected.timing.durations.clone(),
                selected
                    .interval
                    .pairs
                    .iter()
                    .filter(|c| (c.a == w && c.b == b) || (c.a == b && c.b == w))
                    .map(|c| c.normal_impulse)
                    .sum(),
            )
        } else {
            (
                vec![Duration::from_micros(500)],
                native
                    .rigid_contact_observations()
                    .iter()
                    .filter(|c| (c.a == w && c.b == b) || (c.a == b && c.b == w))
                    .map(|c| c.normal_impulse)
                    .sum(),
            )
        };
        out.path.push((
            native.state(ids.foot).unwrap(),
            native.state(ids.pad).unwrap(),
            native.state(w).unwrap(),
            durations,
            reaction,
        ));
    }
    assert!(out.path.iter().map(|t| t.4).sum::<f64>() > 1.);
    Ok(out)
}
fn contention(path: &Path, repetitions: usize) -> Result<()> {
    let mut results = vec![];
    for id in (0..320)
        .filter(|id| Physical::new(*id).certify().reason.is_none())
        .take(32)
    {
        let r = Physical::new(id);
        // The second free box lands within +/-0.2 ms of the primary impact.
        let z = 0.05 + r.arrival + 0.0004 * (radical(id + 17, 13) - 0.5);
        let controls = [false, true].map(|on| with_competitor(&configured(&r, 8, on).1, z));
        let mut baseline = competing_run(&controls[0], false)?;
        let guarded = competing_run(&controls[1], true)?;
        assert_eq!(guarded.path, baseline.path);
        assert_eq!(guarded.selected, baseline.selected);
        let mut times: [Vec<Vec<u64>>; 2] = Default::default();
        for rep in 0..repetitions {
            for k in 0..2 {
                let i = (k + rep + id) % 2;
                let test = competing_run(&controls[i], i != 0)?;
                assert_eq!(test.path, baseline.path);
                times[i].push(test.elapsed);
            }
        }
        let path:Vec<_>=baseline.path.drain(..).map(|(foot,pad,other,durations,j)|json!({"foot":Body::from(foot),"pad":Body::from(pad),"other":Body::from(other),"native_ns":durations.into_iter().map(|d|d.as_nanos()as u64).collect::<Vec<_>>(),"other_impulse_ns":j})).collect();
        results.push(json!({"id":id,"selected":baseline.selected,"shared_exact_path":path,"full_step_ns":times[0],"compact_step_ns":times[1]}));
    }
    write_json(&path.join("contending.json"), &json!({"cases":results}))
}
