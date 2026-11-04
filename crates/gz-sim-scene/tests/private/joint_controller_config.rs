// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

const XML: &str = include_str!("../../../../examples/m9_joint_controllers.sdf");
const POSITION: &str = "<joint_name>base_hinge</joint_name><use_velocity_commands>true</use_velocity_commands><cmd_max>2</cmd_max>";
const VELOCITY: &str = "<joint_name>tip_hinge</joint_name>";

fn model() -> Model {
    sdformat::load_str(XML, "m9-config.sdf", &sdformat::NoResources)
        .unwrap()
        .worlds
        .remove(0)
        .models
        .into_iter()
        .find(|model| model.name == "arm")
        .unwrap()
}

fn body(model: &mut Model, index: usize, contents: &str) {
    let plugin = &mut model.plugins[index];
    plugin.xml = format!(
        "<plugin filename='{}' name='{}'>{contents}</plugin>",
        plugin.filename, plugin.name
    );
}

fn refused(model: &Model, feature: &'static str) {
    assert!(
        matches!(parse(model), Err(LoadError::Unsupported { entity, feature: actual })
        if entity == model.scoped_name && actual == feature)
    );
}

#[test]
fn actual_demo_defaults_topics_and_caps_bind_joint_names_not_vector_order() {
    let mut model = model();
    let actual = parse(&model).unwrap().unwrap();
    assert_eq!(actual.len(), 2);
    assert_eq!(actual[0].joint, "base_hinge");
    assert_eq!(actual[0].publication.model_name, "arm");
    assert_eq!(actual[0].publication.joint_name, "base_hinge");
    assert_eq!(actual[0].publication.mode, JointControllerMode::Position);
    assert_eq!(
        actual[0].publication.topic,
        "/model/arm/joint/base_hinge/0/cmd_pos"
    );
    assert_eq!(
        (
            actual[0].max_velocity,
            actual[0].max_force,
            actual[0].initial_target
        ),
        (2.0, 3.0, 0.0)
    );
    assert_eq!(actual[1].joint, "tip_hinge");
    assert_eq!(actual[1].publication.mode, JointControllerMode::Velocity);
    assert_eq!(
        actual[1].publication.topic,
        "/model/arm/joint/tip_hinge/cmd_vel"
    );
    assert_eq!(
        (
            actual[1].max_velocity,
            actual[1].max_force,
            actual[1].initial_target
        ),
        (4.0, 3.0, 0.0)
    );
    body(&mut model, 0, POSITION);
    body(&mut model, 1, VELOCITY);
    model.joints[0].axis.as_mut().unwrap().velocity = 1.5;
    model.joints[1].axis.as_mut().unwrap().effort = 7.0;
    model.joints.reverse();
    model.links.reverse();
    model.plugins.reverse();
    let reordered = parse(&model).unwrap().unwrap();
    assert_eq!(reordered[0].joint, "tip_hinge");
    assert_eq!(
        (reordered[0].max_velocity, reordered[0].max_force),
        (4.0, 7.0)
    );
    assert_eq!(reordered[1].joint, "base_hinge");
    assert_eq!(reordered[1].max_velocity, 1.5);
    assert_eq!(reordered[1].initial_target, 0.0);
}

#[test]
fn velocity_mode_and_abs_position_mode_do_not_silently_admit_pid_or_indexed_dofs() {
    for value in ["false", "0"] {
        let mut model = model();
        body(
            &mut model,
            1,
            &format!("{VELOCITY}<use_force_commands>{value}</use_force_commands>"),
        );
        assert_eq!(
            parse(&model).unwrap().unwrap()[1].publication.mode,
            JointControllerMode::Velocity
        );
    }
    for value in ["true", "1", "garbage", ""] {
        let mut model = model();
        body(
            &mut model,
            1,
            &format!("{VELOCITY}<use_force_commands>{value}</use_force_commands>"),
        );
        refused(&model, "joint controller mode");
    }
    for flag in ["true", "1"] {
        let mut model = model();
        body(
            &mut model,
            0,
            &format!(
                "<joint_name>base_hinge</joint_name><use_velocity_commands>{flag}</use_velocity_commands><cmd_max>2</cmd_max><joint_index>0</joint_index>"
            ),
        );
        assert_eq!(
            parse(&model).unwrap().unwrap()[0].publication.mode,
            JointControllerMode::Position
        );
    }
    for mode in [
        "",
        "<use_velocity_commands>false</use_velocity_commands>",
        "<use_velocity_commands>0</use_velocity_commands>",
        "<use_velocity_commands>invalid</use_velocity_commands>",
    ] {
        let mut model = model();
        body(
            &mut model,
            0,
            &format!("<joint_name>base_hinge</joint_name>{mode}<cmd_max>2</cmd_max>"),
        );
        refused(&model, "joint controller mode");
    }
    for index in ["1", "-1", "0.5", "NaN", "", "4294967296"] {
        let mut model = model();
        body(
            &mut model,
            0,
            &format!("{POSITION}<joint_index>{index}</joint_index>"),
        );
        refused(&model, "joint controller mode");
    }
    let mut model = model();
    body(
        &mut model,
        1,
        &format!("{VELOCITY}<joint_index>0</joint_index>"),
    );
    refused(&model, "joint controller configuration");
    body(&mut model, 1, VELOCITY);
    body(
        &mut model,
        0,
        "<joint_name>base_hinge</joint_name><use_velocity_commands>true</use_velocity_commands>",
    );
    refused(&model, "joint controller limits");
    for cap in ["0", "-1", "NaN", "inf", "garbage", ""] {
        body(
            &mut model,
            0,
            &format!(
                "<joint_name>base_hinge</joint_name><use_velocity_commands>true</use_velocity_commands><cmd_max>{cap}</cmd_max>"
            ),
        );
        refused(&model, "joint controller limits");
    }
}

#[test]
fn initial_targets_preserve_continuous_positions_and_bound_velocity_by_axis_limits() {
    for target in ["-4", "4", "0"] {
        let mut model = model();
        body(
            &mut model,
            1,
            &format!("{VELOCITY}<initial_velocity>{target}</initial_velocity>"),
        );
        assert_eq!(
            parse(&model).unwrap().unwrap()[1].initial_target,
            target.parse::<f64>().unwrap()
        );
    }
    for target in ["4.001", "-4.001", "NaN", "inf", "garbage", ""] {
        let mut model = model();
        body(
            &mut model,
            1,
            &format!("{VELOCITY}<initial_velocity>{target}</initial_velocity>"),
        );
        refused(&model, "initial joint controller target");
    }
    // Native ABS compares continuous positions; admission must not wrap to +/-pi.
    for target in ["11.75", "-11.75", "0", "1.7976931348623157e308"] {
        let mut model = model();
        body(
            &mut model,
            0,
            &format!("{POSITION}<initial_position>{target}</initial_position>"),
        );
        assert_eq!(
            parse(&model).unwrap().unwrap()[0].initial_target,
            target.parse::<f64>().unwrap()
        );
    }
    for target in ["NaN", "-inf", "garbage", ""] {
        let mut model = model();
        body(
            &mut model,
            0,
            &format!("{POSITION}<initial_position>{target}</initial_position>"),
        );
        refused(&model, "initial joint controller target");
    }
    for value in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        for effort in [false, true] {
            let mut model = model();
            let axis = model.joints[0].axis.as_mut().unwrap();
            if effort {
                axis.effort = value;
            } else {
                axis.velocity = value;
            }
            refused(&model, "joint controller limits");
        }
    }
    let mut model = model();
    model.plugins.remove(1);
    let unconfigured = model.joints[1].axis.as_mut().unwrap();
    unconfigured.velocity = f64::INFINITY;
    unconfigured.effort = f64::INFINITY;
    assert_eq!(parse(&model).unwrap().unwrap().len(), 1);
}

#[test]
fn closed_scalar_xml_rejects_unsupported_fields_repetition_attributes_and_split_text() {
    for (index, required, fields) in [
        (
            0,
            POSITION,
            vec![
                "p_gain",
                "i_gain",
                "d_gain",
                "cmd_min",
                "cmd_offset",
                "use_force_commands",
                "initial_velocity",
                "sub_topic",
                "namespace",
                "actuator_number",
                "disable_braking",
            ],
        ),
        (
            1,
            VELOCITY,
            vec![
                "p_gain",
                "cmd_max",
                "use_velocity_commands",
                "initial_position",
                "sub_topic",
                "namespace",
                "actuator_number",
            ],
        ),
    ] {
        for field in fields {
            let mut model = model();
            body(
                &mut model,
                index,
                &format!("{required}<{field}>0</{field}>"),
            );
            refused(&model, "joint controller configuration");
        }
    }
    for (index, required, field) in [
        (0, POSITION, "joint_name"),
        (0, POSITION, "cmd_max"),
        (0, POSITION, "use_velocity_commands"),
        (0, POSITION, "initial_position"),
        (0, POSITION, "joint_index"),
        (0, POSITION, "topic"),
        (1, VELOCITY, "joint_name"),
        (1, VELOCITY, "initial_velocity"),
        (1, VELOCITY, "use_force_commands"),
    ] {
        let mut model = model();
        // Repetition is rejected even when the two lexical values are identical.
        body(
            &mut model,
            index,
            &format!("{required}<{field}>0</{field}><{field}>0</{field}>"),
        );
        refused(&model, "joint controller configuration");
    }
    for scalar in [
        "<joint_name value='tip_hinge'>tip_hinge</joint_name>",
        "<joint_name><value>tip_hinge</value></joint_name>",
        "<joint_name>tip<!--split-->_hinge</joint_name>",
        "<joint_name>tip<![CDATA[_hinge]]></joint_name>",
        "<?instruction x?><joint_name>tip_hinge</joint_name>",
        "<joint_name xmlns='urn:other'>tip_hinge</joint_name>",
    ] {
        let mut model = model();
        body(&mut model, 1, scalar);
        refused(&model, "joint controller configuration");
    }
    let mut model = model();
    let original = model.plugins[1].xml.clone();
    for xml in [
        original.replace("<plugin ", "<plugin extra='0' "),
        original.replace("<plugin ", "<plugin xmlns:x='urn:other' "),
        original
            .replace("<plugin ", "<other ")
            .replace("</plugin>", "</other>"),
        original.replace("gz-sim-joint-controller-system", "wrong-library"),
        "<plugin".into(),
    ] {
        model.plugins[1].xml = xml;
        refused(&model, "joint controller configuration");
    }
}

#[test]
fn recognized_plugin_dispatch_is_exact_and_cannot_overlap_joints_or_exceed_two() {
    let mut model = model();
    let mut unrelated = model.plugins[1].clone();
    unrelated.name = "x".into();
    unrelated.filename = "x".into();
    assert!(!is_plugin(&unrelated));
    model.plugins.push(unrelated.clone());
    assert_eq!(parse(&model).unwrap().unwrap().len(), 2);
    model.plugins = vec![unrelated];
    assert!(parse(&model).unwrap().is_none());
    model.plugins.clear();
    assert!(parse(&model).unwrap().is_none());
    let mut model = self::model();
    model.plugins[0].filename = "libgz-sim-joint-position-controller-system.so".into();
    assert!(!is_plugin(&model.plugins[0]));
    assert_eq!(parse(&model).unwrap().unwrap().len(), 1);
    let mut model = self::model();
    body(&mut model, 1, "<joint_name>base_hinge</joint_name>");
    refused(&model, "overlapping joint controllers");
    let mut model = self::model();
    model.plugins.push(model.plugins[1].clone());
    refused(&model, "joint controller count");
    for contents in [
        "",
        "<joint_name>missing</joint_name>",
        "<joint_name>arm::tip_hinge</joint_name>",
    ] {
        let mut model = self::model();
        body(&mut model, 1, contents);
        refused(&model, "joint controller joint name");
    }
}

#[test]
fn direct_world_rooted_chain_and_native_local_frame_domain_are_explicit() {
    for mutate in [
        (|m: &mut Model| m.is_static = true) as fn(&mut Model),
        |m| m.raw_namespace = Some("namespace".into()),
        |m| m.models.push(m.clone()),
    ] {
        let mut model = model();
        mutate(&mut model);
        refused(&model, "joint controller model domain");
    }
    for mutate in [
        (|m: &mut Model| {
            m.links.pop();
        }) as fn(&mut Model),
        |m| {
            m.joints.pop();
        },
        |m| m.links[1].name = m.links[0].name.clone(),
        |m| m.joints[1].name = m.joints[0].name.clone(),
        |m| m.joints[0].parent = "default".into(),
        |m| m.joints[1].parent = "world".into(),
        |m| m.joints[1].parent = "tip".into(),
        |m| m.joints[1].child = "base".into(),
        |m| m.canonical_link = "missing".into(),
        |m| m.canonical_link = "tip".into(),
    ] {
        let mut model = model();
        mutate(&mut model);
        refused(&model, "joint controller chain");
    }
    for mutate in [
        (|m: &mut Model| m.links[0].pose.relative_to = "arm".into()) as fn(&mut Model),
        |m| m.links[0].pose.transform.rotation = [0.0, 0.0, 1.0, 0.0],
        |m| m.links[0].pose.transform.translation[0] = f64::INFINITY,
        |m| m.joints[0].kind = JointKind::Fixed,
        |m| m.joints[0].axis = None,
        |m| m.joints[0].axis2 = m.joints[0].axis.clone(),
        |m| m.joints[0].axis.as_mut().unwrap().xyz = [0.0, 0.0, -1.0],
        |m| m.joints[0].axis.as_mut().unwrap().expressed_in = "base".into(),
        |m| m.joints[0].pose.transform.translation[0] = 0.1,
        |m| m.joints[0].pose.transform.rotation = [0.0, 0.0, 1.0, 0.0],
        |m| m.joints[0].pose.relative_to = "tip".into(),
        |m| m.joints[1].child = "missing".into(),
    ] {
        let mut model = model();
        mutate(&mut model);
        refused(&model, "joint controller frames");
    }
    let mut model = model();
    model.canonical_link.clear(); // Native canonical-link fallback is the first direct link.
    for joint in &mut model.joints {
        joint.pose.relative_to.clear();
    }
    assert_eq!(parse(&model).unwrap().unwrap().len(), 2);
}

#[test]
fn absolute_topics_preserve_units_and_never_fan_out_between_distinct_controllers() {
    let mut model = model();
    body(
        &mut model,
        0,
        &format!("{POSITION}<topic>/arm/position</topic>"),
    );
    body(
        &mut model,
        1,
        &format!("{VELOCITY}<topic>/arm/velocity</topic>"),
    );
    let actual = parse(&model).unwrap().unwrap();
    assert_eq!(actual[0].publication.topic, "/arm/position");
    assert_eq!(actual[1].publication.topic, "/arm/velocity");
    for topic in [
        "".into(),
        "relative".into(),
        "/".into(),
        "/a//b".into(),
        "/a b".into(),
        "/a~b".into(),
        "/a@b".into(),
        "/a:=b".into(),
        format!("/{}", "a".repeat(65535)),
    ] {
        body(&mut model, 1, &format!("{VELOCITY}<topic>{topic}</topic>"));
        refused(&model, "joint controller topics");
    }
    body(
        &mut model,
        1,
        &format!("{VELOCITY}<topic>/arm/position</topic>"),
    );
    refused(&model, "joint controller topics");
    // Same-mode fanout is also outside the initial contract.
    model.plugins[0].name = model.plugins[1].name.clone();
    model.plugins[0].filename = model.plugins[1].filename.clone();
    body(
        &mut model,
        0,
        "<joint_name>base_hinge</joint_name><topic>/shared</topic>",
    );
    body(&mut model, 1, &format!("{VELOCITY}<topic>/shared</topic>"));
    refused(&model, "joint controller topics");
}
