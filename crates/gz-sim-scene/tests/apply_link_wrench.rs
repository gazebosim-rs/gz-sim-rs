// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation};
use gz_sim_scene::{
    APPLY_LINK_WRENCH_CAPACITY, ApplyLinkWrenchConfiguration, ApplyLinkWrenchError,
    ApplyLinkWrenchMode::{Once, Persistent},
    Name, PhysicsEngine, Velocity, clear_apply_link_wrench, load_world_with_sdf_step,
    queue_apply_link_wrench, queue_link_world_wrench, remove_model, reset_loaded_world,
};
use std::time::Duration;
const XML: &str = include_str!("../../../examples/m35_apply_link_wrench.sdf");
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m35.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn id(sim: &Simulation, name: &str) -> u64 {
    sim.world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|v| v.0 == name))
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get()
}
fn velocity(sim: &Simulation, name: &str) -> ([f64; 3], [f64; 3]) {
    let v = sim
        .world()
        .iter_entities()
        .find(|e| e.get::<Name>().is_some_and(|v| v.0 == name))
        .unwrap()
        .get::<Velocity>()
        .unwrap();
    (v.linear, v.angular)
}
fn close(actual: f64, expected: f64) {
    assert!((actual - expected).abs() <= 1e-12, "{actual} != {expected}");
}
#[test]
fn actual_once_persistent_addition_and_paused_clear_keep_distinct_lifetimes() {
    let mut sim = load(XML);
    let body = id(&sim, "default::rig::body");
    queue_apply_link_wrench(&mut sim, body, [4.0, 0.0, 0.0], [0.0, 0.0, 2.0], Once).unwrap();
    for _ in 0..3 {
        assert!(sim.tick().unwrap().dt.is_zero());
    }
    assert_eq!(velocity(&sim, "default::rig::body"), ([0.0; 3], [0.0; 3]));
    sim.step_exact(1).unwrap();
    close(velocity(&sim, "default::rig::body").0[0], 0.002);
    close(velocity(&sim, "default::rig::body").1[2], 0.002);
    sim.step_exact(1).unwrap();
    close(velocity(&sim, "default::rig::body").0[0], 0.002);
    queue_apply_link_wrench(&mut sim, body, [1.0, 0.0, 0.0], [0.0, 0.0, 2.0], Persistent).unwrap();
    queue_apply_link_wrench(&mut sim, body, [3.0, 0.0, 0.0], [0.0, 0.0, 4.0], Persistent).unwrap();
    sim.step_exact(2).unwrap();
    close(velocity(&sim, "default::rig::body").0[0], 0.006);
    close(velocity(&sim, "default::rig::body").1[2], 0.014);
    queue_apply_link_wrench(&mut sim, body, [2.0, 0.0, 0.0], [0.0; 3], Once).unwrap();
    clear_apply_link_wrench(&mut sim, body).unwrap();
    // Even an addition after the queued clear is removed by clear-first maintenance.
    queue_apply_link_wrench(&mut sim, body, [8.0, 0.0, 0.0], [0.0; 3], Persistent).unwrap();
    sim.set_paused(true);
    sim.tick().unwrap();
    queue_apply_link_wrench(&mut sim, body, [2.0, 0.0, 0.0], [0.0; 3], Persistent).unwrap();
    sim.step_exact(1).unwrap();
    close(velocity(&sim, "default::rig::body").0[0], 0.008);
    sim.step_exact(1).unwrap();
    close(velocity(&sim, "default::rig::body").0[0], 0.009);
    clear_apply_link_wrench(&mut sim, body).unwrap();
    sim.tick().unwrap();
    sim.step_exact(2).unwrap();
    close(velocity(&sim, "default::rig::body").0[0], 0.009);
    close(velocity(&sim, "default::other::body").0[0], 0.0);
}
#[test]
fn bounds_nonfinite_and_ordered_existing_pending_sums_are_atomic() {
    let mut sim = load(XML);
    let body = id(&sim, "default::rig::body");
    queue_apply_link_wrench(&mut sim, body, [2.0, 0.0, 0.0], [0.0; 3], Once).unwrap();
    for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        assert!(
            queue_apply_link_wrench(&mut sim, body, [value, 0.0, 0.0], [0.0; 3], Once).is_err()
        );
        assert!(
            queue_apply_link_wrench(&mut sim, body, [0.0; 3], [0.0, value, 0.0], Persistent)
                .is_err()
        );
    }
    sim.step_exact(1).unwrap();
    close(velocity(&sim, "default::rig::body").0[0], 0.001);
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    queue_link_world_wrench(&mut sim, body, [-f64::MAX, 0.0, 0.0], [0.0; 3]).unwrap();
    queue_apply_link_wrench(&mut sim, body, [f64::MAX, 0.0, 0.0], [0.0; 3], Once).unwrap();
    // (-MAX + MAX) + MAX is finite; pre-aggregating the two system entries is not.
    queue_apply_link_wrench(&mut sim, body, [f64::MAX, 0.0, 0.0], [0.0; 3], Persistent).unwrap();
    sim.reset().unwrap();
    for _ in 0..APPLY_LINK_WRENCH_CAPACITY {
        queue_apply_link_wrench(&mut sim, body, [-0.0, 0.0, 0.0], [0.0; 3], Once).unwrap();
        queue_apply_link_wrench(&mut sim, body, [0.0; 3], [0.0; 3], Persistent).unwrap();
        clear_apply_link_wrench(&mut sim, body).unwrap();
    }
    assert!(matches!(
        queue_apply_link_wrench(&mut sim, body, [0.0; 3], [0.0; 3], Once),
        Err(ApplyLinkWrenchError::Capacity)
    ));
    assert!(matches!(
        queue_apply_link_wrench(&mut sim, body, [0.0; 3], [0.0; 3], Persistent),
        Err(ApplyLinkWrenchError::Capacity)
    ));
    assert!(matches!(
        clear_apply_link_wrench(&mut sim, body),
        Err(ApplyLinkWrenchError::Capacity)
    ));
    sim.step_exact(1).unwrap();
    assert_eq!(velocity(&sim, "default::rig::body"), ([0.0; 3], [0.0; 3]));
    let other = id(&sim, "default::other::body");
    queue_apply_link_wrench(&mut sim, body, [2.0, 0.0, 0.0], [0.0; 3], Once).unwrap();
    queue_apply_link_wrench(&mut sim, other, [f64::MAX, 0.0, 0.0], [0.0; 3], Once).unwrap();
    // A subsequent ordinary direct input is part of the final all-target shadow.
    queue_link_world_wrench(&mut sim, other, [f64::MAX, 0.0, 0.0], [0.0; 3]).unwrap();
    assert!(sim.step_exact(1).is_err());
    assert_eq!(velocity(&sim, "default::rig::body"), ([0.0; 3], [0.0; 3]));
    assert_eq!(velocity(&sim, "default::other::body"), ([0.0; 3], [0.0; 3]));
}
#[test]
fn target_identity_removal_reset_and_absence_do_not_retarget_commands() {
    let mut sim = load(XML);
    let body = id(&sim, "default::rig::body");
    let model = id(&sim, "default::rig");
    assert!(queue_apply_link_wrench(&mut sim, model, [1.0; 3], [0.0; 3], Once).is_err());
    assert!(queue_apply_link_wrench(&mut sim, u64::MAX, [1.0; 3], [0.0; 3], Persistent).is_err());
    queue_apply_link_wrench(&mut sim, body, [100.0, 0.0, 0.0], [0.0; 3], Persistent).unwrap();
    queue_apply_link_wrench(&mut sim, body, [100.0, 0.0, 0.0], [0.0; 3], Once).unwrap();
    remove_model(&mut sim, model).unwrap();
    assert!(clear_apply_link_wrench(&mut sim, body).is_err());
    sim.step_exact(1).unwrap();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    assert_eq!(id(&sim, "default::rig::body"), body);
    sim.step_exact(1).unwrap();
    assert_eq!(velocity(&sim, "default::rig::body"), ([0.0; 3], [0.0; 3]));
    let bare=XML.replace("  <plugin name=\"gz::sim::systems::ApplyLinkWrench\" filename=\"gz-sim-apply-link-wrench-system\"/>\n","");
    let mut absent = load(&bare);
    let body = id(&absent, "default::rig::body");
    assert!(
        !absent
            .world()
            .iter_entities()
            .any(|e| e.contains::<ApplyLinkWrenchConfiguration>())
    );
    assert!(matches!(
        queue_apply_link_wrench(&mut absent, body, [1.0; 3], [0.0; 3], Once),
        Err(ApplyLinkWrenchError::Unconfigured)
    ));
    let offcentre = XML.replace(
        "<inertial><mass>",
        "<inertial><pose>0.1 0 0 0 0 0</pose><mass>",
    );
    assert!(matches!(
        load_world_with_sdf_step(
            &offcentre,
            "m35-offset-com.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier,
        ),
        Err(gz_sim_scene::LoadError::Unsupported { entity, feature: "offset center of mass" })
            if entity == "default::rig::body"
    ));
    let body = id(&sim, "default::rig::body");
    queue_link_world_wrench(&mut sim, body, [1.0; 3], [0.0; 3]).unwrap();
}
#[test]
fn plugin_configuration_is_strict_and_tpe_does_not_gain_wrench_support() {
    let original = "<plugin name=\"gz::sim::systems::ApplyLinkWrench\" filename=\"gz-sim-apply-link-wrench-system\"/>";
    for fields in [
        "<verbose>false</verbose>",
        "<persistent/>",
        "<topic/>",
        "<topic>/x<!--tail-->bad</topic>",
        "<topic>/x&amp;y<!--tail-->bad</topic>",
        "<topic><![CDATA[/x]]></topic>",
        "<topic>/x<bad/></topic>",
        "<topic>/x</topic><topic>/y</topic>",
        "<topic>/x</topic><topic_clear>/x</topic_clear>",
        "<topic bad='1'>/x</topic>",
        "<foreign/>",
    ] {
        let replacement = format!(
            "<plugin name='gz::sim::systems::ApplyLinkWrench' filename='gz-sim-apply-link-wrench-system'>{fields}</plugin>"
        );
        let xml = XML.replace(original, &replacement);
        assert!(
            load_world_with_sdf_step(
                &xml,
                "bad-m35.sdf",
                "default",
                &sdformat::NoResources,
                PhysicsEngine::Rapier
            )
            .is_err(),
            "{fields}"
        );
    }
    assert!(
        load_world_with_sdf_step(
            XML,
            "m35-tpe.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Tpe
        )
        .is_err()
    );
    let mut sim = load(XML);
    assert_eq!(sim.step_duration(), Duration::from_millis(1));
    sim.set_paused(true);
    assert!(sim.tick().unwrap().dt.is_zero());
}

#[test]
fn direct_scene_spawn_reactivates_only_the_configured_system_at_the_boundary() {
    let mut sim = load(XML);
    let sdf = sdformat::load_str(XML, "m35.sdf", &sdformat::NoResources).unwrap();
    let template =
        gz_sim_scene::prepare_model_template(&sim, &sdf, &sdf.worlds[0].models[0]).unwrap();
    for name in ["default::rig", "default::other"] {
        let identity = id(&sim, name);
        remove_model(&mut sim, identity).unwrap();
    }
    assert!(!gz_sim_scene::apply_link_wrench_active(&sim));
    let model = gz_sim_scene::spawn_model_template(
        &mut sim,
        &template,
        gz_sim_scene::WorldPose {
            translation: [0.0, 6.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        },
    )
    .unwrap();
    assert!(gz_sim_scene::apply_link_wrench_active(&sim));
    let model_entity = gz_sim_core::entity_by_id(sim.world(), model).unwrap();
    let body = sim
        .world()
        .iter_entities()
        .find(|e| {
            e.get::<bevy_ecs::hierarchy::ChildOf>()
                .is_some_and(|parent| parent.parent() == model_entity)
                && e.get::<gz_sim_scene::EntityKind>() == Some(&gz_sim_scene::EntityKind::Link)
        })
        .unwrap()
        .get::<EntityId>()
        .unwrap()
        .get();
    queue_apply_link_wrench(&mut sim, body, [2.0, 0.0, 0.0], [0.0; 3], Once).unwrap();
    sim.step_exact(1).unwrap();
    let entity = gz_sim_core::entity_by_id(sim.world(), body).unwrap();
    close(
        sim.world().get::<Velocity>(entity).unwrap().linear[0],
        0.001,
    );
}

#[test]
fn world_axes_signed_zero_and_unsupported_articulation_preserve_old_input_bounds() {
    let xml = XML
        .replace(
            "<model name=\"rig\">",
            "<model name=\"rig\"><pose>0 0 0 0 0 1.5707963267948966</pose>",
        )
        .replace(
            "<inertial><mass>",
            "<inertial><pose>-0 0 -0 0 0 0</pose><mass>",
        );
    let mut sim = load(&xml);
    let body = id(&sim, "default::rig::body");
    queue_apply_link_wrench(&mut sim, body, [2.0, -0.0, 0.0], [-0.0, 0.0, 1.0], Once).unwrap();
    sim.step_exact(1).unwrap();
    let (linear, angular) = velocity(&sim, "default::rig::body");
    close(linear[0], 0.001);
    close(linear[1], 0.0);
    close(angular[2], 0.001);
    let static_xml = XML.replace(
        "<model name=\"rig\">",
        "<model name=\"rig\"><static>true</static>",
    );
    let mut fixed = load(&static_xml);
    let body = id(&fixed, "default::rig::body");
    assert!(queue_apply_link_wrench(&mut fixed, body, [1.0; 3], [0.0; 3], Once).is_err());
    let robot = include_str!("../../../examples/m8_diff_drive.sdf");
    let plugin = "<plugin name='gz::sim::systems::ApplyLinkWrench' filename='gz-sim-apply-link-wrench-system'/>";
    let xml = robot.replace(
        "<world name=\"default\">",
        &format!("<world name=\"default\">{plugin}"),
    );
    let mut jointed = load(&xml);
    let body = id(&jointed, "default::robot::base_link");
    assert!(queue_apply_link_wrench(&mut jointed, body, [1.0; 3], [0.0; 3], Once).is_err());
}

#[test]
fn topic_overrides_and_xml_entities_are_exact_and_duplicates_fail_before_load() {
    let plugin = "<plugin name=\"gz::sim::systems::ApplyLinkWrench\" filename=\"gz-sim-apply-link-wrench-system\"/>";
    let custom = "<plugin name='gz::sim::systems::ApplyLinkWrench' filename='gz-sim-apply-link-wrench-system'><topic>/m35/a&amp;b</topic><topic_persistent>/m35/persistent</topic_persistent><topic_clear>/m35/clear</topic_clear></plugin>";
    let sim = load(&XML.replace(plugin, custom));
    let config = sim
        .world()
        .iter_entities()
        .find_map(|e| e.get::<ApplyLinkWrenchConfiguration>())
        .unwrap();
    assert_eq!(
        (
            &*config.topic,
            &*config.persistent_topic,
            &*config.clear_topic
        ),
        ("/m35/a&b", "/m35/persistent", "/m35/clear")
    );
    for encoded in ["/m35/a&#38;b", "&#47;m35/a&amp;b"] {
        let sim = load(&XML.replace(plugin, &custom.replace("/m35/a&amp;b", encoded)));
        let config = sim
            .world()
            .iter_entities()
            .find_map(|e| e.get::<ApplyLinkWrenchConfiguration>())
            .unwrap();
        assert_eq!(config.topic, "/m35/a&b");
    }
    let duplicate = XML.replace(plugin, &format!("{plugin}{plugin}"));
    assert!(
        load_world_with_sdf_step(
            &duplicate,
            "m35-duplicate.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
    let attribute=XML.replace(plugin,"<plugin name='gz::sim::systems::ApplyLinkWrench' filename='gz-sim-apply-link-wrench-system' extra='ignored'/>");
    assert!(
        load_world_with_sdf_step(
            &attribute,
            "m35-attribute.sdf",
            "default",
            &sdformat::NoResources,
            PhysicsEngine::Rapier
        )
        .is_err()
    );
}
