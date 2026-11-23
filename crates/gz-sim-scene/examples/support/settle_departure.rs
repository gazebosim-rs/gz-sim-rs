// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Preserve the full native history through first release, then branch the public authored policy.
use super::*;
use gz_sim_scene::{primary_contact::set_enabled, queue_link_world_wrench};
const ENDURANCE: &str = include_str!("../../../../examples/multifidelity_primary_endurance.sdf");
const RELEASE: u64 = 1472;
const STOP: u64 = 1732;
const TARGET: [u64; 2] = [2120, 2184];
const END: u64 = 3732;
pub(super) fn source(mu: Option<f64>, policy: &str) -> String {
    let mut s = ENDURANCE.to_owned();
    if let Some(mu) = mu {
        s=s.replace("</geometry></collision>",&format!("</geometry><surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface></collision>"));
    }
    if policy == "ordinary" {
        let a = s
            .find("    <plugin name=\"gz::sim::systems::PrimaryContactRefinement\"")
            .unwrap();
        let b = a + s[a..].find("</plugin>").unwrap() + 9;
        s.replace_range(a..b, "");
    }
    if policy == "relax32" {
        s = s.replace("<relaxation_iterations>8", "<relaxation_iterations>32");
    }
    if policy == "refresh8" {
        s = s.replace(
            "</support>",
            "</support><refresh_box_features>true</refresh_box_features>",
        );
    }
    s
}
pub(super) fn cmd(t: u64) -> f64 {
    if t < 1400 {
        0.
    } else {
        let k = t - 1400;
        if k < 48 {
            -1.25
        } else if k < 72 {
            0.
        } else if k < 212 {
            0.30
        } else if k < 332 {
            0.30 * (1. - ((k - 212) as f64 + 0.5) / 120.)
        } else {
            0.
        }
    }
}
pub(super) fn ingest(
    s: &MultiFidelityBackend,
    ids: &Reading,
) -> (BodyState, Option<BodyState>, BodyState) {
    (
        s.state(ids.foot).unwrap(),
        ids.upper.and_then(|id| s.state(id)),
        s.state(ids.pad).unwrap(),
    )
}
fn prefix(s: &MultiFidelityBackend, ids: &Reading) -> Result<Value> {
    let (f, u, p) = ingest(s, ids);
    let warm = s.native_contact_warmstart(ids.foot, ids.pad)?;
    Ok(
        json!({"foot":wire(f),"upper":u.map(wire),"pad":wire(p),"warm_points":warm.points,"warm_normal_ns":warm.normal_ns,"warm_tangent_ns":warm.tangent_on_a_ns}),
    )
}
fn run(
    mu: Option<f64>,
    per_point: bool,
    policy: &str,
    matched: bool,
) -> Result<(Value, Vec<Value>)> {
    let xml = source(mu, policy);
    let mut sim = load_world_with_runtime(
        &xml,
        "mixed-settling.sdf",
        "default",
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: None,
        },
        WorldRuntimeOptions {
            rapier_per_point_friction: per_point,
            ..Default::default()
        },
    )?;
    let bindings = [("rocker", "lift"), ("edge_rocker", "edge_lift")].map(|(m, l)| {
        let (pad_link, pad) = bind(&sim, &format!("default::{l}::pad")).unwrap();
        (
            Reading {
                foot: bind(&sim, &format!("default::{m}::foot")).unwrap().1,
                upper: Some(bind(&sim, &format!("default::{m}::upper")).unwrap().1),
                pad,
            },
            pad_link,
        )
    });
    let (witness, _) = bind(&sim, "default::witness::body")?;
    if matched && policy != "ordinary" {
        set_enabled(&mut sim, false)?;
    }
    let mut initial = Value::Null;
    let mut m: [Vec<Metric>; 2] = Default::default();
    let mut outcomes: [Vec<String>; 2] = Default::default();
    let mut requests = vec![];
    let mut traces = vec![];
    let mut previous = 0.;
    let dt = sim.step_duration();
    for boundary in 0..END {
        if boundary == RELEASE {
            let n = sim
                .world()
                .resource::<PhysicsBackend>()
                .downcast_ref::<MultiFidelityBackend>()
                .unwrap();
            initial = json!([prefix(n, &bindings[0].0)?, prefix(n, &bindings[1].0)?]);
            if matched && policy != "ordinary" {
                set_enabled(&mut sim, true)?;
            }
        }
        let tick = boundary + 1;
        let force = if tick % 4 < 2 { 24. } else { -16. };
        queue_link_world_wrench(&mut sim, witness, [force, 0., 0.], [0.; 3])?;
        if boundary == 1400 {
            sim.set_paused(true);
            assert!(sim.tick()?.dt.is_zero());
        }
        let speed = cmd(boundary);
        if speed != previous {
            for (_, link) in &bindings {
                queue_link_world_velocity(&mut sim, *link, [0., 0., speed], [0.; 3])?;
            }
            previous = speed;
        }
        sim.step_exact(1)?;
        let n = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .unwrap();
        let receipt = last_refinement(&sim).filter(|r| r.iteration == tick);
        if let Some(r) = receipt {
            requests.push(json!({"tick":tick,"pairs":r.requested_pairs.iter().map(|p|p.foot_link).collect::<Vec<_>>()}));
        }
        for (i, (ids, _)) in bindings.iter().enumerate() {
            if tick >= STOP {
                let result = n.validate_articulation_demotion(ids.foot, dt);
                m[i].push(metric(n, ids, ingest(n, ids), result.is_ok()));
                outcomes[i].push(result.err().map(|e| e.to_string()).unwrap_or_default());
            }
            if i == 0
                && mu.is_none()
                && !per_point
                && matches!(policy, "ordinary" | "relax8" | "refresh8")
                && (RELEASE..=2300).contains(&tick)
                && (tick % 8 == 0 || (1520..=1540).contains(&tick))
            {
                let (f, u, p) = ingest(n, ids);
                let key = (ids.foot.min(ids.pad), ids.foot.max(ids.pad));
                let impulse = receipt.map_or_else(
                    || {
                        n.rigid_contact_observations()
                            .iter()
                            .filter(|p| (p.a, p.b) == key)
                            .map(|p| p.normal_impulse)
                            .sum()
                    },
                    |r| {
                        r.interval
                            .pairs
                            .iter()
                            .filter(|p| (p.a, p.b) == key)
                            .map(|p| p.normal_impulse)
                            .sum::<f64>()
                    },
                );
                let selected =
                    receipt.is_some_and(|r| r.requested_pairs.iter().any(|p| p.foot == ids.foot));
                traces.push(json!({"tick":tick,"time_us":tick*500,"native_steps":receipt.map_or(1,|r|r.interval.native_steps),"solver_iterations":4,
                    "selected":selected,"weak_start":false,"secondary_start":false,"foot":wire(f),"upper":wire(u.unwrap()),"pad":wire(p),
                    "normal_impulse_ns":impulse,"gap_m":f.position[2]-p.position[2]-0.06-q_up(f).abs().dot(DVec3::new(0.18,0.10,0.06))}));
            }
        }
    }
    let mut rigs = vec![];
    for (i, rows) in m.iter().enumerate() {
        let at = (TARGET[i] - STOP) as usize;
        let target = rows[at];
        let next = &rows[at..=at + 40];
        let rotation_path: f64 = next.windows(2).map(|p| quat_angle(p[0].q, p[1].q)).sum();
        let net = quat_angle(target.q, next.last().unwrap().q);
        let translation = next
            .iter()
            .map(|p| (p.com - target.com).length())
            .fold(0., f64::max);
        let gate = rows
            .windows(40)
            .position(|w| w.iter().all(|m| m.accept))
            .map(|i| STOP + i as u64);
        let after_target = rows[at..]
            .windows(40)
            .position(|w| w.iter().all(|m| m.accept))
            .map(|j| TARGET[i] + j as u64);
        let samples = [0, 40, 160, 400, 800, 1600, 1960]
            .map(|j| json!({"tick":STOP+j as u64,"sample":rows[j].describe()}));
        let reasons = outcomes[i].iter().filter(|s| !s.is_empty()).fold(
            BTreeMap::<String, usize>::new(),
            |mut a, s| {
                *a.entry(s.clone()).or_default() += 1;
                a
            },
        );
        rigs.push(json!({"rig":i,"target_tick":TARGET[i],"at_attempt":target.describe(),"gate_reason":outcomes[i][at],
            "future_20ms_rotation_path_rad":rotation_path,"future_20ms_net_rotation_rad":net,"future_20ms_max_com_move_m":translation,
            "first_20ms_eligible_tick":gate,"first_20ms_eligible_after_attempt":after_target,"parked_gate_failures":reasons,"park_samples":samples}));
    }
    Ok((
        json!({"mu":mu,"per_point":per_point,"policy":policy,"matched":matched,"prefix_tick":RELEASE,"prefix":initial,"refined":requests,"rigs":rigs}),
        traces,
    ))
}
pub(super) fn execute(dir: &std::path::Path) -> Result<()> {
    let mut data = vec![];
    for matched in [false, true] {
        let name = if matched { "matched" } else { "uninterrupted" };
        let out = dir.join(name);
        fs::create_dir_all(out.join("traces"))?;
        for mu in [None, Some(0.), Some(0.2), Some(0.8)] {
            for per_point in [false, true] {
                for policy in POLICIES {
                    let (row, trace) = run(mu, per_point, policy, matched)?;
                    data.push(row);
                    if !trace.is_empty() {
                        let geometry = json!({"foot_half_extents":[0.18,0.1,0.06],"upper_half_extents":[0.05,0.06,0.06],"pad_half_extents":[0.33,0.18,0.06]});
                        fs::write(
                            out.join(format!("traces/{policy}.json")),
                            serde_json::to_vec(&json!({"geometry":geometry,"frames":trace}))?,
                        )?;
                    }
                }
            }
        }
        let lanes = [
            ("ordinary", "Ordinary solve"),
            ("relax8", "8 relaxation passes"),
            ("refresh8", "8 passes + fresh geometry"),
        ]
        .map(|(l, n)| json!({"label":l,"name":n,"file":format!("traces/{l}.json")}));
        fs::write(
            out.join("comparison.json"),
            serde_json::to_vec_pretty(
                &json!({"title":format!("First mixed lift return · {name} native histories"),
            "sampled_ticks":true,"capture_ticks":{"demotion_gate":2120},"independent_histories":!matched,"pair_totals_only":true,"lanes":lanes}),
            )?,
        )?;
    }
    fs::write(dir.join("crossover.json"), serde_json::to_vec(&data)?)?;
    Ok(())
}
