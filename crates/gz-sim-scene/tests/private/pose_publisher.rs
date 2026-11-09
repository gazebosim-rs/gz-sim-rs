// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use sdformat::NoResources;
const XML: &str = include_str!("../fixtures/m27_pose_publisher.sdf");
fn model() -> Model {
    sdformat::load_str(XML, "m27.sdf", &NoResources)
        .unwrap()
        .worlds
        .remove(0)
        .models
        .remove(0)
}
fn configured(contents: &str) -> Model {
    let mut m = model();
    m.plugins[0].xml = format!(
        "<plugin filename='{}' name='{}'>{contents}</plugin>",
        m.plugins[0].filename, m.plugins[0].name
    );
    m
}
fn load(xml: &str) -> Simulation {
    load_world_with_sdf_step(
        xml,
        "m27.sdf",
        "default",
        &NoResources,
        PhysicsEngine::Rapier,
    )
    .unwrap()
}
fn owner(sim: &Simulation) -> Entity {
    sim.world()
        .iter_entities()
        .find(|e| e.contains::<PosePublication>())
        .unwrap()
        .id()
}
fn absent_xml() -> String {
    let start = XML.find("    <plugin ").unwrap();
    let end = start + XML[start..].find("</plugin>").unwrap() + 9;
    format!("{}{}", &XML[..start], &XML[end..])
}
#[test]
fn pose_configuration_preserves_native_defaults_and_checked_frequency_without_sensor_clamping() {
    let mut absent = model();
    absent.plugins.clear();
    assert!(parse(&absent).unwrap().is_none());
    let p = parse(&configured(
        "<use_pose_vector_msg>true</use_pose_vector_msg>",
    ))
    .unwrap()
    .unwrap();
    assert!(!p.model && p.links);
    assert_eq!(p.topic, "/model/rig/pose");
    assert_eq!(p.period, Duration::ZERO);
    let p=parse(&configured("<use_pose_vector_msg>true</use_pose_vector_msg><publish_link_pose>false</publish_link_pose><publish_model_pose>true</publish_model_pose><update_frequency>3</update_frequency><topic>/custom/pose</topic>")).unwrap().unwrap();
    assert!(p.model && !p.links);
    assert_eq!(p.period, Duration::from_nanos(333_333_333));
    assert_eq!(p.topic, "/custom/pose");
    for rate in [-1.0, -1e300, 0.0, 2e9] {
        assert_eq!(period(rate), Some(Duration::ZERO));
    }
    assert_eq!(period(100.0), Some(Duration::from_millis(10)));
    // Native computes the reciprocal before conversion; reassociation loses a ns.
    assert_eq!(period(90909090.90909092), Some(Duration::from_nanos(11)));
    let boundary=parse(&configured("<use_pose_vector_msg>true</use_pose_vector_msg><update_frequency>90909090.90909092</update_frequency>")).unwrap().unwrap();
    assert_eq!(boundary.period, Duration::from_nanos(11));
    assert!(period(1.1e-10).is_some());
    for rate in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, 1e-10, 1e-300] {
        assert!(period(rate).is_none());
    }
}
#[test]
fn pose_configuration_refuses_unsupported_publication_without_dropping_scene_entities() {
    for content in [
        "",
        "<use_pose_vector_msg>false</use_pose_vector_msg>",
        "<use_pose_vector_msg>true</use_pose_vector_msg><publish_link_pose>false</publish_link_pose>",
        "<use_pose_vector_msg>true</use_pose_vector_msg><static_publisher>true</static_publisher><publish_model_pose>true</publish_model_pose><publish_link_pose>false</publish_link_pose>",
        "<use_pose_vector_msg>true</use_pose_vector_msg><static_update_frequency>1</static_update_frequency>",
    ] {
        assert!(parse(&configured(content)).is_err(), "{content}");
    }
    for option in [
        "publish_nested_model_pose",
        "publish_visual_pose",
        "publish_collision_pose",
        "publish_sensor_pose",
    ] {
        assert!(
            parse(&configured(&format!(
                "<use_pose_vector_msg>true</use_pose_vector_msg><{option}>true</{option}>"
            )))
            .is_err()
        );
        assert!(
            parse(&configured(&format!(
                "<use_pose_vector_msg>true</use_pose_vector_msg><{option}>false</{option}>"
            )))
            .is_ok()
        );
    }
    for bad in [
        "<unknown/>",
        "<topic>/a</topic><topic>/b</topic>",
        "<topic>/bad topic</topic>",
        "<topic>/</topic>",
        "<topic>/a//b</topic>",
        "<topic>/bad@x</topic>",
        "<topic>/bad~x</topic>",
        "<topic>/bad:=x</topic>",
        "<topic/>",
        "<update_frequency>NaN</update_frequency>",
        "<update_frequency>x</update_frequency>",
        "<publish_model_pose>1</publish_model_pose>",
        "<topic a='x'>/a</topic>",
        "<topic><n>/a</n></topic>",
        "<topic><![CDATA[/a]]></topic>",
        "<topic xmlns='x'>/a</topic>",
    ] {
        assert!(
            parse(&configured(&format!(
                "<use_pose_vector_msg>true</use_pose_vector_msg>{bad}"
            )))
            .is_err(),
            "{bad}"
        );
    }
    let mut m = configured("<use_pose_vector_msg>true</use_pose_vector_msg>");
    m.plugins.push(m.plugins[0].clone());
    assert!(parse(&m).is_err());
    let mut m = model();
    m.plugins[0].xml = "<plugin".into();
    assert!(parse(&m).is_err());
    let mut m = model();
    m.plugins[0].xml = m.plugins[0].xml.replace("<plugin ", "<plugin xmlns='x' ");
    assert!(parse(&m).is_err());
    let mut m = model();
    m.plugins[0].xml.push_str(&" ".repeat(65536));
    assert!(parse(&m).is_err());
    let mut m = model();
    m.scoped_name = "default::outer::rig".into();
    assert!(parse(&m).is_err());
    let mut many = configured("<use_pose_vector_msg>true</use_pose_vector_msg>");
    let link = many.links[0].clone();
    many.links = (0..128)
        .map(|i| {
            let mut v = link.clone();
            v.name = format!("l{i}");
            v
        })
        .collect();
    assert!(parse(&many).is_ok());
    many.links.push(link);
    assert!(parse(&many).is_err());
    let sim = load(XML);
    assert!(
        sim.world()
            .iter_entities()
            .any(|e| e.get::<EntityKind>() == Some(&EntityKind::Visual))
    );
    assert_eq!(
        sim.world()
            .get::<PosePublication>(owner(&sim))
            .unwrap()
            .poses
            .len(),
        3
    );
}
#[test]
fn pose_plan_domain_is_flat_rapier_and_one_publisher_static_models_are_not_static_streams() {
    let sim = load(&absent_xml());
    let m = model();
    let p = parse(&m).unwrap().unwrap();
    p.validate(&m, &sim, Some("rapier"), (true, false)).unwrap();
    p.validate(&m, &sim, Some("rapier"), (true, true)).unwrap();
    for engine in [None, Some("dart"), Some("tpe")] {
        assert!(p.validate(&m, &sim, engine, (true, false)).is_err());
    }
    assert!(
        p.validate(&m, &sim, Some("rapier"), (false, false))
            .is_err()
    );
    let mut nested = model();
    nested.models.push(model());
    assert!(
        p.validate(&nested, &sim, Some("rapier"), (true, false))
            .is_err()
    );
    let live = load(XML);
    assert!(
        p.validate(&m, &live, Some("rapier"), (true, false))
            .is_err()
    );
}
#[test]
fn pose_cadence_skips_all_paused_ticks_but_equal_unpaused_and_rewind_are_eligible() {
    let mut sim = load(XML);
    let entity = owner(&sim);
    sim.tick().unwrap();
    assert!(sim.world().get::<PoseSample>(entity).is_none());
    sim.step_exact(9).unwrap();
    assert!(sim.world().get::<PoseSample>(entity).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().get::<PoseSample>(entity).unwrap().time,
        Duration::from_millis(10)
    );
    let revision = sim.world().get::<PoseSample>(entity).unwrap().revision;
    sim.tick().unwrap();
    sim.tick().unwrap();
    assert_eq!(
        sim.world().get::<PoseSample>(entity).unwrap().revision,
        revision
    );
    // Equal simulation time is not the same as paused. Exercise the true native
    // PostUpdate boundary with an unpaused zero-dt update, without changing physics.
    sim.world_mut().resource_mut::<UpdateInfo>().paused = false;
    sample(sim.world_mut());
    assert_eq!(
        sim.world().get::<PoseSample>(entity).unwrap().revision,
        revision + 1
    );
    sim.reset().unwrap();
    assert!(sim.world().get::<PoseSample>(entity).is_none());
    sim.tick().unwrap();
    assert!(sim.world().get::<PoseSample>(entity).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(
        sim.world().get::<PoseSample>(entity).unwrap().time,
        Duration::from_millis(1)
    );
    let identity = sim.world().get::<EntityId>(entity).unwrap().get();
    reset_loaded_world(&mut sim, |_| ()).unwrap();
    let restored = gz_sim_core::entity_by_id(sim.world(), identity).unwrap();
    assert_ne!(entity, restored);
    assert!(sim.world().get::<PoseSample>(restored).is_none());
    sim.step_exact(9).unwrap();
    assert!(sim.world().get::<PoseSample>(restored).is_none());
    sim.step_exact(1).unwrap();
    assert_eq!(sim.world().get::<PoseSample>(restored).unwrap().revision, 1);
}
#[test]
fn pose_invalid_local_frame_retires_whole_previous_sample_and_reports_terminal_fault() {
    let xml = XML.replace(
        "<update_frequency>100</update_frequency>",
        "<update_frequency>0</update_frequency>",
    );
    for case in 0..8 {
        let mut sim = load(&xml);
        let entity = owner(&sim);
        sim.step_exact(1).unwrap();
        assert!(sim.world().get::<PoseSample>(entity).is_some());
        let frame = sim
            .world()
            .get::<PosePublication>(entity)
            .unwrap()
            .poses
            .last()
            .unwrap()
            .identity;
        let link = gz_sim_core::entity_by_id(sim.world(), frame).unwrap();
        match case {
            0 => {
                sim.world_mut()
                    .get_mut::<LocalPose>(link)
                    .unwrap()
                    .0
                    .translation[0] = f64::NAN
            }
            1 => {
                sim.world_mut()
                    .get_mut::<LocalPose>(link)
                    .unwrap()
                    .0
                    .rotation = [0.0; 4]
            }
            2 => {
                sim.world_mut().entity_mut(link).remove::<LocalPose>();
            }
            3 => {
                sim.world_mut().entity_mut(link).remove::<EntityId>();
            }
            4 => {
                sim.world_mut().entity_mut(link).insert(EntityKind::Visual);
            }
            5 => {
                sim.world_mut()
                    .get_mut::<PosePublication>(entity)
                    .unwrap()
                    .topic = "/forged".into()
            }
            6 => {
                sim.world_mut()
                    .get_mut::<Publisher>(entity)
                    .unwrap()
                    .revision = u64::MAX
            }
            _ => {
                sim.world_mut()
                    .entity_mut(entity)
                    .remove::<PosePublication>();
            }
        }
        sample(sim.world_mut());
        assert!(sim.world().get::<PoseSample>(entity).is_none());
        assert!(sim.world().resource::<StepFault>().error().is_some());
    }
}
#[test]
fn pose_absence_last_removal_and_loaded_restore_release_and_reinstall_only_owned_systems() {
    let absent = load(&absent_xml());
    let mut live = load(XML);
    let base = absent.scheduled_system_counts();
    assert_eq!(live.scheduled_system_counts(), (base.0 + 1, base.1 + 1));
    assert!(!absent.world().contains_resource::<Installed>());
    let entity = owner(&live);
    let id = live.world().get::<EntityId>(entity).unwrap().get();
    remove_model(&mut live, id).unwrap();
    assert!(!live.world().contains_resource::<Installed>());
    assert!(
        !live
            .world()
            .iter_entities()
            .any(|e| e.contains::<Publisher>() || e.contains::<PoseSample>())
    );
    retire_if_unused(&mut live).unwrap();
    reset_loaded_world(&mut live, |_| ()).unwrap();
    assert!(live.world().contains_resource::<Installed>());
    let restored = gz_sim_core::entity_by_id(live.world(), id).unwrap();
    live.step_exact(10).unwrap();
    assert!(live.world().get::<PoseSample>(restored).is_some());
}
