// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

const ORIGINAL: &str = include_str!("../fixtures/m18_joint_state.sdf");
fn model_source(index: usize, rate: &str) -> String {
    let begin = ORIGINAL.find("  <model ").unwrap();
    let end = ORIGINAL.rfind("</model>").unwrap() + "</model>".len();
    ORIGINAL[begin..end]
        .replace("name=\"arm\"", &format!("name=\"arm{index}\""))
        .replace(
            "<pose>0 0 1 0 0 0</pose>",
            &format!("<pose>0 {} 1 0 0 0</pose>", index * 4),
        )
        .replace(
            "<topic>/m18/joint_state</topic>",
            &format!("<topic>/m57/{index}/joint_state</topic>"),
        )
        .replace(
            "<update_rate>100</update_rate>",
            &format!("<update_rate>{rate}</update_rate>"),
        )
}
fn world_source(count: usize) -> String {
    let models = (0..count)
        .map(|index| model_source(index, "0"))
        .collect::<String>();
    format!(
        "<sdf version='1.12'><world name='default'><gravity>0 0 0</gravity><physics name='step' type='ode'><max_step_size>0.001</max_step_size></physics>{models}</world></sdf>"
    )
}
fn load(source: &str) -> Result<Simulation, LoadError> {
    load_world_with_sdf_step(
        source,
        "m57-private.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
}
fn owner(sim: &Simulation, index: usize) -> Entity {
    sim.world()
        .iter_entities()
        .find(|node| {
            node.get::<Name>()
                .is_some_and(|name| name.0 == format!("default::arm{index}"))
        })
        .unwrap()
        .id()
}

#[test]
fn owner_bound_and_selected_slot_bounds_are_independent_closed_admission_limits() {
    let eight = load(&world_source(8)).unwrap();
    let owners = eight
        .world()
        .iter_entities()
        .filter(|node| node.contains::<Publisher>())
        .collect::<Vec<_>>();
    assert_eq!(owners.len(), 8);
    assert_eq!(
        owners
            .iter()
            .map(|node| node.get::<Publisher>().unwrap().publication.joints.len())
            .sum::<usize>(),
        16
    );
    assert!(matches!(
        load(&world_source(9)),
        Err(LoadError::Unsupported {
            feature: "multiple joint-state models",
            ..
        })
    ));
    let malformed_last = world_source(8).replace(
        "<topic>/m57/7/joint_state</topic>",
        "<topic>/bad topic</topic>",
    );
    assert!(load(&malformed_last).is_err());
    // This is the existing configuration resource boundary, not a new 1024-joint
    // physical topology or a claim that all maxima combine into one solver case.
    let mut model = sdformat::load_str(ORIGINAL, "m18.sdf", &sdformat::NoResources)
        .unwrap()
        .worlds
        .remove(0)
        .models
        .remove(0);
    let template = model.joints[0].clone();
    model.joints = (0..128)
        .map(|index| {
            let mut joint = template.clone();
            joint.name = format!("selected_{index}");
            joint
        })
        .collect();
    let mut slots = 0;
    for index in 0..8 {
        model.name = format!("bounded{index}");
        model.scoped_name = format!("default::bounded{index}");
        let plan = parse(&model).unwrap().unwrap();
        assert_eq!(plan.names.len(), 128);
        slots += plan.names.len();
    }
    assert_eq!(slots, 1024);
    let mut overflow = template;
    overflow.name = "selected_128".to_owned();
    model.joints.push(overflow);
    assert!(matches!(
        parse(&model),
        Err(LoadError::Unsupported {
            feature: "joint-state joint count",
            ..
        })
    ));
}

#[test]
fn per_owner_sampler_fault_keeps_global_terminal_contract_without_peer_order_assumptions() {
    for failure in 0..3 {
        let mut sim = load(&world_source(2)).unwrap();
        sim.step_exact(1).unwrap();
        let owners = [owner(&sim, 0), owner(&sim, 1)];
        assert!(owners.iter().all(|entity| {
            sim.world()
                .get::<JointStateSample>(*entity)
                .unwrap()
                .revision
                == 1
        }));
        let victim = owners[1];
        match failure {
            0 => sim
                .world_mut()
                .get_mut::<JointStatePublication>(victim)
                .unwrap()
                .topic
                .push_str("/changed"),
            1 => {
                sim.world_mut()
                    .get_mut::<Publisher>(victim)
                    .unwrap()
                    .revision = u64::MAX
            }
            _ => {
                let joint_id = sim
                    .world()
                    .get::<Publisher>(victim)
                    .unwrap()
                    .publication
                    .joints[0]
                    .id;
                let joint = gz_sim_core::entity_by_id(sim.world(), joint_id).unwrap();
                sim.world_mut()
                    .get_mut::<JointSample>(joint)
                    .unwrap()
                    .state
                    .position = f64::NAN;
            }
        }
        // Physics would replace a forged JointSample; directly exercise the
        // unchanged publication boundary, as in the original M18 control.
        sample(sim.world_mut());
        assert!(sim.world().resource::<StepFault>().error().is_some());
        assert!(sim.world().get::<JointStateSample>(victim).is_none());
        let peer = sim
            .world()
            .get::<JointStateSample>(owners[0])
            .unwrap()
            .clone();
        assert!((1..=2).contains(&peer.revision));
        let peer_bits = peer
            .joints
            .iter()
            .map(|value| (value.id, value.position.to_bits(), value.velocity.to_bits()))
            .collect::<Vec<_>>();
        let clock = (sim.elapsed(), sim.epoch());
        assert_eq!(
            sim.step_exact(1).unwrap_err(),
            gz_sim_core::StepError::Faulted
        );
        let mut observed = false;
        assert!(matches!(
            reset_loaded_world(&mut sim, |_| observed = true),
            Err(SceneResetError::Clock(gz_sim_core::StepError::Faulted))
        ));
        assert!(!observed);
        assert_eq!((sim.elapsed(), sim.epoch()), clock);
        let after = sim.world().get::<JointStateSample>(owners[0]).unwrap();
        assert_eq!(
            (after.time, after.epoch, after.revision),
            (peer.time, peer.epoch, peer.revision)
        );
        assert_eq!(after.pose.translation, peer.pose.translation);
        assert_eq!(after.pose.rotation, peer.pose.rotation);
        assert_eq!(
            after
                .joints
                .iter()
                .map(|value| (value.id, value.position.to_bits(), value.velocity.to_bits()))
                .collect::<Vec<_>>(),
            peer_bits
        );
    }
}
