// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Operator withdrawal of one support beneath a cohort of settled two-link feet.
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, PhysicsBackend};
use gz_physics_multifidelity::{ArticulationTier, MultiFidelityBackend};
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    BodyHandle, Name, PhysicsEngine,
    fidelity_zone::{self, FidelityZone},
    load_world_with_sdf_step, model_fidelity,
    primary_contact::last_refinement,
    queue_link_world_velocity, reset_loaded_world, support_loss_promotion,
};
use serde_json::{Value, json};
use std::{collections::BTreeMap, fs, path::Path, time::Duration};
use tungstenite::Message;

const XML: &str = include_str!("../../../../examples/multifidelity_shared_batch.sdf");
const STEP: Duration = Duration::from_micros(500);
const STATION: usize = 1;
// Cooldown ends at 140 ms. Four eligible half-millisecond intervals precede demotion.
const PARK: u64 = 285;
const WITHDRAW: u64 = 300;
const END: u64 = 600;

#[derive(Clone, Copy)]
struct Foot {
    model: u64,
    toe_entity: u64,
    bodies: [BodyId; 4],
}

fn bindings(sim: &Simulation) -> [Foot; 4] {
    #[allow(deprecated)]
    let lookup = |name: String| {
        let e = sim
            .world()
            .iter_entities()
            .find(|e| e.get::<Name>().is_some_and(|n| n.0 == name))
            .unwrap();
        (
            e.get::<EntityId>().unwrap().get(),
            e.get::<BodyHandle>().map(BodyHandle::id),
        )
    };
    std::array::from_fn(|i| {
        let toe = lookup(format!("default::toe_{i}::pad"));
        Foot {
            model: lookup(format!("default::rocker_{i}")).0,
            toe_entity: toe.0,
            bodies: [
                lookup(format!("default::rocker_{i}::foot")).1.unwrap(),
                lookup(format!("default::rocker_{i}::upper")).1.unwrap(),
                lookup(format!("default::lift_{i}::pad")).1.unwrap(),
                toe.1.unwrap(),
            ],
        }
    })
}

fn scene() -> Simulation {
    load_world_with_sdf_step(
        XML,
        "cohort.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn states(sim: &Simulation, feet: &[Foot; 4]) -> [[BodyState; 4]; 4] {
    let backend = sim.world().resource::<PhysicsBackend>();
    feet.map(|f| f.bodies.map(|b| backend.state(b).unwrap()))
}

fn withdraw(sim: &mut Simulation, feet: &[Foot; 4]) {
    let start = sim.elapsed();
    let before = states(sim, feet);
    let normal = DQuat::from_array(before[STATION][3].orientation) * DVec3::Z;
    queue_link_world_velocity(
        sim,
        feet[STATION].toe_entity,
        (-1.8 * normal).to_array(),
        [0.; 3],
    )
    .unwrap();
    sim.tick().unwrap();
    assert_eq!(sim.elapsed(), start);
    assert_eq!(states(sim, feet), before);
}

fn loads(sim: &Simulation, f: Foot) -> [f64; 2] {
    let backend = sim.world().resource::<PhysicsBackend>();
    let solver = backend.downcast_ref::<MultiFidelityBackend>().unwrap();
    [f.bodies[2], f.bodies[3]].map(|pad| {
        let pair = |a, b| (a == f.bodies[0] && b == pad) || (a == pad && b == f.bodies[0]);
        if let Some(r) = last_refinement(sim).filter(|r| r.sim_time == sim.elapsed()) {
            r.interval
                .pairs
                .iter()
                .filter(|p| pair(p.a, p.b))
                .map(|p| p.normal_impulse)
                .sum()
        } else {
            solver
                .rigid_contact_observations()
                .iter()
                .filter(|p| pair(p.a, p.b))
                .map(|p| p.normal_impulse)
                .sum()
        }
    })
}

fn names(i: usize) -> [String; 4] {
    [
        format!("default::rocker_{i}::foot::sole"),
        format!("default::rocker_{i}::upper::v"),
        format!("default::lift_{i}::pad::platform"),
        format!("default::toe_{i}::pad::platform"),
    ]
}

#[test]
fn paused_support_withdrawal_wakes_only_its_loaded_foot_and_reset_repeats_the_cut() {
    let mut policy = scene();
    let mut dynamic = scene();
    let feet = bindings(&policy);
    let control = bindings(&dynamic);
    for f in feet {
        fidelity_zone::configure_with_eligibility_dwell(
            &mut policy,
            f.model,
            vec![FidelityZone {
                center: [99., 0., 0.],
                promote_radius: 0.2,
                demote_radius: 1.,
            }],
            Duration::from_millis(140),
            Duration::from_millis(2),
        )
        .unwrap();
    }
    let sdf = sdformat::load_str(XML, "cohort.sdf", &sdformat::NoResources).unwrap();
    let mut frames = Vec::new();
    let mut reference = Vec::new();
    let mut captures = BTreeMap::new();
    let mut first_epoch = Vec::new();
    let mut evidence = Vec::new();
    let mut displacement_m = [0.0_f64; 4];
    let mut impulse_error_ns = [0.0_f64; 2];
    let mut rotation_rad = [0.0_f64; 4];
    for epoch in 0..2 {
        if epoch == 1 {
            reset_loaded_world(&mut policy, |_| ()).unwrap();
            reset_loaded_world(&mut dynamic, |_| ()).unwrap();
        }
        policy.set_paused(true);
        dynamic.set_paused(true);
        for tick in 0..=END {
            if tick == WITHDRAW + 1 {
                withdraw(&mut policy, &feet);
                withdraw(&mut dynamic, &control);
            }
            if tick > 0 {
                policy.step_exact(1).unwrap();
                dynamic.step_exact(1).unwrap();
            }
            let measured = states(&policy, &feet);
            let baseline = states(&dynamic, &control);
            let parked = feet
                .map(|f| model_fidelity(&policy, f.model).unwrap() == ArticulationTier::Prescribed);
            assert_eq!(
                parked,
                std::array::from_fn(|i| tick >= PARK && (i != STATION || tick <= WITHDRAW)),
                "epoch {epoch}, tick {tick}"
            );
            let reaction = loads(&policy, feet[STATION]);
            let expected = loads(&dynamic, control[STATION]);
            if epoch == 0 {
                first_epoch.push((measured, parked, reaction));
            } else {
                assert_eq!(first_epoch[tick as usize], (measured, parked, reaction));
            }
            if [PARK - 1, PARK, WITHDRAW, WITHDRAW + 1, WITHDRAW + 2, END].contains(&tick)
                && epoch == 0
            {
                evidence.push(json!({"tick":tick,"parked":parked,"normal_impulse_ns":reaction,"control_ns":expected,"foot_position_m":measured[STATION][0].position,"control_position_m":baseline[STATION][0].position}));
            }
            if tick == PARK - 1 {
                for pair in feet.map(|f| loads(&policy, f)) {
                    assert!(pair.iter().all(|j| *j > 0.), "{pair:?}");
                    assert!(
                        (pair.iter().sum::<f64>() - 2. * 9.81 * STEP.as_secs_f64()).abs() < 1e-12
                    );
                }
            }
            if tick == WITHDRAW + 1 {
                let wake =
                    support_loss_promotion::last_promotion(&policy, feet[STATION].model).unwrap();
                assert_eq!(
                    (wake.epoch, wake.iteration, wake.link, wake.support),
                    (
                        epoch,
                        tick,
                        feet[STATION].bodies[0],
                        feet[STATION].bodies[3]
                    )
                );
                assert_eq!(wake.start_time, STEP * WITHDRAW as u32);
                assert!(reaction[0] > 0., "{reaction:?}");
                assert_eq!(reaction[1], 0.);
                for f in feet
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| *i != STATION)
                    .map(|(_, f)| f)
                {
                    assert!(support_loss_promotion::last_promotion(&policy, f.model).is_none());
                }
            }
            for (i, observed) in measured.iter().enumerate() {
                let err =
                    DVec3::from(observed[0].position).distance(baseline[i][0].position.into());
                displacement_m[i] = displacement_m[i].max(err);
                let a = DQuat::from_array(observed[0].orientation).normalize();
                let b = DQuat::from_array(baseline[i][0].orientation).normalize();
                let chord = (a - b).length().min((a + b).length());
                let rotation = 4. * (0.5 * chord).asin();
                rotation_rad[i] = rotation_rad[i].max(rotation);
            }
            if tick > WITHDRAW {
                assert_eq!(measured[STATION], baseline[STATION]);
                assert_eq!(reaction, expected);
                for (i, (a, b)) in reaction.into_iter().zip(expected).enumerate() {
                    impulse_error_ns[i] = impulse_error_ns[i].max((a - b).abs());
                }
            }
            if epoch == 0
                && (tick % 10 == 0 || [PARK - 1, PARK, WITHDRAW + 1, WITHDRAW + 2].contains(&tick))
            {
                for (at, label) in [
                    (0, "start"),
                    (PARK, "park"),
                    (WITHDRAW + 1, "wake"),
                    (END, "final"),
                ] {
                    if tick == at {
                        captures.insert(label, frames.len());
                    }
                }
                frames.push(
                    gz_sim_cli::primitive_frame(&policy, &sdf, "default", frames.len() as u64 + 1)
                        .unwrap(),
                );
                let poses: BTreeMap<_, _> = measured
                    .into_iter()
                    .enumerate()
                    .flat_map(|(i, group)| {
                        names(i).into_iter().zip(group).map(|(name, b)| {
                            (
                                name,
                                json!({"translation":b.position,"rotation":b.orientation}),
                            )
                        })
                    })
                    .collect();
                reference.push(json!({"time_ns":policy.elapsed().as_nanos(),"poses":poses}));
            }
        }
    }
    evidence.push(json!({"max_displacement_m":displacement_m,"max_rotation_rad":rotation_rad,"max_impulse_error_ns":impulse_error_ns}));
    assert!(displacement_m.into_iter().all(|d| d < 0.0001));
    assert!(rotation_rad.into_iter().all(|r| r < 0.0001));
    let initial = first_epoch[WITHDRAW as usize].0;
    let final_state = first_epoch.last().unwrap().0;
    let travel =
        DVec3::from(initial[STATION][3].position).distance(final_state[STATION][3].position.into());
    assert!((travel - 1.8 * 0.150).abs() < 1e-12);
    for i in [0, 2, 3] {
        assert_eq!(initial[i], final_state[i]);
    }
    let tmp = tempfile::tempdir().unwrap();
    let output = std::env::var_os("GZ_FOOT_COHORT_REPLAY");
    let folder = output.as_ref().map(Path::new).unwrap_or(tmp.path());
    fs::create_dir_all(folder).unwrap();
    gz_sim_cli::export_archive(&folder.join("replay"), BTreeMap::new(), frames).unwrap();
    fs::write(
        folder.join("visual-reference.json"),
        serde_json::to_vec(&reference).unwrap(),
    )
    .unwrap();
    fs::write(
        folder.join("measurements.json"),
        serde_json::to_vec_pretty(&evidence).unwrap(),
    )
    .unwrap();
    check_cold_replay(&folder.join("replay"), &reference, feet[STATION].model);
    let visual_map: BTreeMap<_, _> = (0..4).flat_map(names).map(|n| (n.clone(), n)).collect();
    let index = json!({
        "kind": "authored-archive-v1", "dt_ns": 1, "camera": "perspective",
        "source_poses": "../visual-reference.json", "source_visual_poses": visual_map,
        "captures": captures
    });
    fs::write(
        folder.join("index.json"),
        serde_json::to_vec(&index).unwrap(),
    )
    .unwrap();
}

fn check_cold_replay(archive: &Path, reference: &[Value], model: u64) {
    let (mut process, mut socket) = super::player(archive);
    let mut frame = super::read_tuple(&mut socket, "publication_credit");
    for (i, truth) in reference.iter().enumerate() {
        assert_eq!(frame[2]["frame"], i + 1);
        assert_eq!(frame[2]["recorded_nanoseconds"], truth["time_ns"]);
        let visuals = frame[0]["visuals"].as_array().unwrap();
        assert_eq!(visuals.len(), 16);
        for v in visuals {
            let name = v["name"].as_str().unwrap();
            for field in ["translation", "rotation"] {
                let numbers = |values: &Value| {
                    values
                        .as_array()
                        .unwrap()
                        .iter()
                        .map(|x| x.as_f64().unwrap())
                        .collect::<Vec<_>>()
                };
                let actual = numbers(&v[field]);
                let expected = numbers(&truth["poses"][name][field]);
                assert_eq!(actual.len(), expected.len());
                assert!(
                    actual
                        .iter()
                        .zip(expected)
                        .all(|(x, y)| (x - y).abs() < 1e-12),
                    "{name} {field} at {i}"
                );
            }
        }
        if i == 0 {
            socket.send(Message::text(json!({"version":1,"kind":"lifecycle","request_id":1,"action":"remove","model_id":model,"world_id":frame[0]["world_id"],"epoch":frame[0]["epoch"]}).to_string())).unwrap();
            let refused = super::read_tuple(&mut socket, "lifecycle_ack");
            assert_eq!(refused[0]["visuals"], frame[0]["visuals"]);
            assert_eq!(refused[3]["ok"], false);
            assert_eq!(refused[3]["error"], "recorded playback is read-only");
            socket
                .send(Message::text(
                    json!({"version":1,"kind":"control","request_id":2,"action":"resume"})
                        .to_string(),
                ))
                .unwrap();
            assert_eq!(super::read_tuple(&mut socket, "ack")[3]["ok"], true);
        }
        if i + 1 < reference.len() {
            socket
                .send(Message::text(
                    json!({"version":1,"kind":"publication_ready","token":frame[3]["token"]})
                        .to_string(),
                ))
                .unwrap();
            frame = super::read_tuple(&mut socket, "publication_credit");
        }
    }
    assert_eq!(frame[2]["finished"], true);
    socket
        .send(Message::text(
            json!({"version":1,"kind":"control","request_id":3,"action":"reset"}).to_string(),
        ))
        .unwrap();
    let rewind = super::read_tuple(&mut socket, "ack");
    assert_eq!(rewind[2]["frame"], 1);
    assert_eq!(rewind[3]["ok"], true);
    assert_eq!(rewind[0]["epoch"], 1);
    socket.close(None).unwrap();
    rustix::process::kill_process(
        rustix::process::Pid::from_child(process.0.as_ref().unwrap()),
        rustix::process::Signal::TERM,
    )
    .unwrap();
    assert!(process.wait(super::TIMEOUT).status.success());
}
