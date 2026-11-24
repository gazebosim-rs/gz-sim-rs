// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{JointSample, Name, PhysicsEngine, load_world_with_sdf_step, set_joint_force};

const BASE: &str = include_str!("../../../examples/worlds/local_reduced_wrist_slider.sdf");
const PLUGIN: &str = r#"<plugin name="gz::sim::systems::ApplyJointForce" filename="gz-sim-apply-joint-force-system"><joint_name>slider</joint_name></plugin>"#;

fn xml(lower: f64, upper: f64, powered: bool) -> String {
    let mut sdf = BASE.replace("-0.02405 0.12", "3.0 0.12").replace(
        "<xyz>1 0 0</xyz>",
        &format!("<xyz>1 0 0</xyz><limit><lower>{lower}</lower><upper>{upper}</upper></limit>"),
    );
    if powered {
        sdf = sdf.replace(
            "<link name=\"wrist\">",
            &format!("{PLUGIN}<link name=\"wrist\">"),
        );
    }
    sdf
}

fn load(sdf: &str) -> Simulation {
    load_world_with_sdf_step(
        sdf,
        "wrist.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap()
}

fn slider(sim: &Simulation) -> (u64, JointSample) {
    #[allow(deprecated)]
    let joint = sim
        .world()
        .iter_entities()
        .find(|node| {
            node.get::<Name>()
                .is_some_and(|n| n.0 == "default::wrist_rig::slider")
        })
        .unwrap();
    (
        joint.get::<EntityId>().unwrap().get(),
        *joint.get::<JointSample>().unwrap(),
    )
}

#[test]
fn the_loaded_native_stop_holds_and_releases_in_both_directions() {
    for side in [-1., 1.] {
        let bounds = if side > 0. { [-0.01, 0.] } else { [0., 0.01] };
        let mut sim = load(&xml(bounds[0], bounds[1], true));
        let (id, initial) = slider(&sim);
        let run = |sim: &mut Simulation| {
            set_joint_force(sim, id, 2. * side).unwrap();
            sim.step_exact(100).unwrap();
            let loaded = slider(sim).1.state;
            println!("side {side} loaded {loaded:?}");
            assert!(loaded.position.abs() < 1e-8 && loaded.velocity.abs() < 1e-4);
            set_joint_force(sim, id, -2. * side).unwrap();
            sim.step_exact(100).unwrap();
            let away = slider(sim).1.state;
            println!("side {side} released {away:?}");
            assert!(side * away.position < -1e-5 && side * away.velocity < -0.02);
            [loaded, away]
        };
        sim.step_exact(10).unwrap();
        let path = run(&mut sim);
        sim.reset().unwrap();
        assert_eq!(slider(&sim).1.state, initial.state);
        sim.step_exact(10).unwrap();
        assert_eq!(slider(&sim).1.state, initial.state);
        assert_eq!(run(&mut sim), path);
    }
}

#[test]
fn unpowered_slider_is_passive_and_joint_force_has_one_owner() {
    let sdf = xml(-0.001, 0.001, true);
    let mut sim = load(&sdf);
    let (id, initial) = slider(&sim);
    sim.step_exact(200).unwrap();
    assert_eq!(slider(&sim).1.state, initial.state);
    set_joint_force(&mut sim, id, 2.).unwrap();
    sim.step_exact(1500).unwrap();
    let st = slider(&sim).1.state;
    println!("interior reached positive stop {st:?}");
    assert!((st.position - 0.001).abs() < 1e-7 && st.velocity.abs() < 0.001);
    for bad in [
        sdf.replace(PLUGIN, &format!("{PLUGIN}{PLUGIN}")),
        sdf.replace(PLUGIN, &PLUGIN.replace("slider", "hinge")),
        sdf.replace("<upper>0.001</upper>", "<upper>-0.002</upper>"),
    ] {
        assert!(
            load_world_with_sdf_step(
                &bad,
                "bad.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::MultiFidelity
            )
            .is_err()
        );
    }
}
