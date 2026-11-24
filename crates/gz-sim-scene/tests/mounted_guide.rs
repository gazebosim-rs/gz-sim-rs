// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_physics::PhysicsBackend;
use gz_sim_scene::{
    PhysicsEngine, load_local_replay_world, load_world_with_sdf_step, reset_loaded_world,
};
const WORLD: &str = include_str!("../../../examples/worlds/local_robot_serial_transfer.sdf");
const SETTING: &str =
    r#"<natural_frequency_hz joint="default::gripper::travel">100000</natural_frequency_hz>"#;
fn plugin(body: &str) -> String {
    format!(
        r#"<plugin name="gz::sim::systems::MountedPrismaticGuide" filename="gz-sim-mounted-prismatic-guide-system">{body}</plugin>"#
    )
}
fn world(body: &str) -> String {
    WORLD.replace("</world>", &format!("{body}</world>"))
}
#[test]
fn named_guide_restores_the_same_local_state_on_both_world_construction_paths() {
    let xml = world(&plugin(SETTING));
    let normal = load_world_with_sdf_step(
        &xml,
        "guide.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::MultiFidelity,
    )
    .unwrap();
    let replay = load_local_replay_world(&xml, "guide.sdf", "default", [24; 32]).unwrap();
    let mut states = Vec::new();
    for mut s in [normal, replay] {
        for run in 0..2 {
            s.step_exact(10).unwrap();
            #[allow(deprecated)]
            let ids = s
                .world()
                .iter_entities()
                .filter_map(|e| e.get::<gz_sim_scene::BodyHandle>().map(|b| b.id()));
            states.push(
                ids.map(|id| s.world().resource::<PhysicsBackend>().state(id).unwrap())
                    .collect::<Vec<_>>(),
            );
            if run == 0 {
                reset_loaded_world(&mut s, |_| ()).unwrap();
            }
        }
    }
    assert!(states[0].len() >= 6 && states.windows(2).all(|pair| pair[0] == pair[1]));
}
#[test]
fn reject_wrong_guide_or_malformed_frequency_before_stepping() {
    let valid = plugin(SETTING);
    for authored in [
        plugin(""),
        plugin("<wrong/>"),
        plugin(&SETTING.repeat(2)),
        valid.repeat(2),
        plugin(&SETTING.replace("joint=", "link=")),
        plugin(&SETTING.replace("travel", "spindle")),
        plugin(&SETTING.replace("travel", "left")),
        plugin(&SETTING.replace("travel", "close_right")),
        plugin(&SETTING.replace("100000", "")),
        plugin(&SETTING.replace("100000", "NaN")),
        plugin(&SETTING.replace("100000", "garbled")),
        plugin(&SETTING.replace("100000", "1000001")),
        plugin(&SETTING.replace("100000", "<value>100000</value>")),
    ] {
        assert!(
            load_local_replay_world(&world(&authored), "invalid.sdf", "default", [0; 32]).is_err(),
            "{authored}"
        );
    }
    let empty = world(&valid);
    assert!(
        load_world_with_sdf_step(
            &empty,
            "bad.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Tpe
        )
        .is_err()
    );
}

#[test]
fn two_robot_guide_frequencies_are_independent_and_the_unselected_robot_stays_ordinary() {
    let robot = |name: &str, x: f64| {
        format!(
            r#"<model name="{name}"><pose>{x} 0 0 0 0 0</pose><link name="slider"><inertial><mass>1</mass><inertia><ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz></inertia></inertial><collision name="box"><geometry><box><size>0.1 0.1 0.1</size></box></geometry></collision></link><joint name="rail" type="prismatic"><parent>world</parent><child>slider</child><axis><xyz>1 0 0</xyz><limit><lower>-1</lower><upper>1</upper></limit></axis></joint></model>"#
        )
    };
    let models = format!(
        "{}{}{}",
        robot("one", 0.),
        robot("two", 1.),
        robot("three", 2.)
    );
    let measure = |settings: &str| {
        let xml = format!(
            r#"<sdf version="1.12"><world name="default"><gravity>0 0 -9.81</gravity><physics name="p" type="ignored"><max_step_size>0.001</max_step_size></physics>{models}{settings}</world></sdf>"#
        );
        let mut s = load_local_replay_world(&xml, "two-robots.sdf", "default", [0; 32]).unwrap();
        s.step_exact(160).unwrap();
        ["one", "two", "three"].map(|name| {
            let name = format!("default::{name}::slider");
            #[allow(deprecated)]
            let id = s
                .world()
                .iter_entities()
                .find(|e| e.get::<gz_sim_scene::Name>().is_some_and(|n| n.0 == name))
                .unwrap()
                .get::<gz_sim_scene::BodyHandle>()
                .unwrap()
                .id();
            s.world()
                .resource::<PhysicsBackend>()
                .state(id)
                .unwrap()
                .position[2]
        })
    };
    let settings = |a: u32, b: u32| {
        plugin(&format!(
            r#"<natural_frequency_hz joint="default::one::rail">{a}</natural_frequency_hz><natural_frequency_hz joint="default::two::rail">{b}</natural_frequency_hz>"#
        ))
    };
    let normal = measure("");
    let ab = measure(&settings(100, 100000));
    let ba = measure(&settings(100000, 100));
    assert!(ab[0].abs() > 1e3 * ab[1].abs());
    assert_eq!(ab[0], ba[1]);
    assert_eq!(ab[1], ba[0]);
    assert_eq!(ab[2], normal[2]);
    assert_eq!(ba[2], normal[2]);
    println!("transverse deflection under 9.81 N: {ab:?} m");
}
