// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

const XML: &str = include_str!("../../../../examples/m8_diff_drive.sdf");
const JOINTS: &str =
    "<left_joint>left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>";

fn model() -> Model {
    sdformat::load_str(XML, "m8-config.sdf", &sdformat::NoResources)
        .unwrap()
        .worlds
        .remove(0)
        .models
        .into_iter()
        .find(|model| model.name == "robot")
        .unwrap()
}

fn body(model: &mut Model, contents: &str) {
    model.plugins[0].xml = format!(
        "<plugin filename='gz-sim-diff-drive-system' name='gz::sim::systems::DiffDrive'>{contents}</plugin>"
    );
}

fn refused(model: &Model, feature: &'static str) {
    assert!(
        matches!(parse(model), Err(LoadError::Unsupported { entity, feature: actual })
        if entity == model.scoped_name && actual == feature)
    );
}

#[test]
fn defaults_and_explicit_demo_bind_names_limits_and_native_canonical_frames() {
    let mut model = model();
    let actual = parse(&model).unwrap().unwrap();
    assert_eq!(
        (actual.left.as_str(), actual.right.as_str()),
        ("left_wheel_joint", "right_wheel_joint")
    );
    assert_eq!(
        (actual.radius, actual.separation, actual.rate),
        (0.2, 0.5, 50.0)
    );
    assert_eq!((actual.linear, actual.angular), ([-1.0, 1.0], [-2.0, 2.0]));
    assert_eq!(
        (actual.wheel_speed, actual.wheel_force),
        ([10.0; 2], [5.0; 2])
    );
    body(&mut model, JOINTS);
    model.canonical_link.clear(); // Native fallback selects the first direct link.
    let default = parse(&model).unwrap().unwrap();
    assert_eq!(
        (default.radius, default.separation, default.rate),
        (0.2, 1.0, 50.0)
    );
    assert_eq!(
        (default.linear, default.angular),
        (
            [f64::NEG_INFINITY, f64::INFINITY],
            [f64::NEG_INFINITY, f64::INFINITY]
        )
    );
    assert_eq!(default.publication.model_name, "robot");
    assert_eq!(default.publication.topic, "/model/robot/cmd_vel");
    assert_eq!(default.publication.odom_topic, "/model/robot/odometry");
    assert_eq!(default.publication.frame_id, "robot/odom");
    assert_eq!(default.publication.child_frame_id, "robot/base_link");
    model.canonical_link = "base_link".into();
    model.joints[0].axis.as_mut().unwrap().velocity = 3.0;
    model.joints[1].axis.as_mut().unwrap().effort = 7.0;
    model.joints.reverse();
    model.links.reverse();
    // Geometry is a configured parameter, never inferred from spacing or shapes.
    body(
        &mut model,
        &format!(
            "{JOINTS}<wheel_radius>0.3</wheel_radius><wheel_separation>9</wheel_separation><odom_publish_frequency>25</odom_publish_frequency>"
        ),
    );
    let reordered = parse(&model).unwrap().unwrap();
    assert_eq!(
        (reordered.radius, reordered.separation, reordered.rate),
        (0.3, 9.0, 25.0)
    );
    assert_eq!(
        (reordered.wheel_speed, reordered.wheel_force),
        ([3.0, 10.0], [5.0, 7.0])
    );
}

#[test]
fn native_general_velocity_limits_are_overridden_per_axis_and_zero_must_remain_admitted() {
    let mut model = model();
    body(
        &mut model,
        &format!(
            "{JOINTS}<min_velocity>-4</min_velocity><max_velocity>5</max_velocity><min_linear_velocity>-2</min_linear_velocity><max_angular_velocity>3</max_angular_velocity>"
        ),
    );
    let actual = parse(&model).unwrap().unwrap();
    assert_eq!((actual.linear, actual.angular), ([-2.0, 5.0], [-4.0, 3.0]));
    for contents in [
        "<min_velocity>0</min_velocity><max_velocity>0</max_velocity>",
        "<min_linear_velocity>-1</min_linear_velocity>",
        "<max_angular_velocity>2</max_angular_velocity>",
    ] {
        body(&mut model, &format!("{JOINTS}{contents}"));
        assert!(parse(&model).is_ok());
    }
    // Native accepts some positive-only/negative-only ranges; M8 explicitly defers them.
    for contents in [
        "<min_velocity>1</min_velocity>",
        "<max_velocity>-1</max_velocity>",
        "<min_linear_velocity>1</min_linear_velocity><max_linear_velocity>2</max_linear_velocity>",
        "<min_angular_velocity>-2</min_angular_velocity><max_angular_velocity>-1</max_angular_velocity>",
        "<min_velocity>2</min_velocity><max_velocity>1</max_velocity>",
    ] {
        body(&mut model, &format!("{JOINTS}{contents}"));
        refused(&model, "DiffDrive velocity limits");
    }
    for name in [
        "min_velocity",
        "max_velocity",
        "min_linear_velocity",
        "max_linear_velocity",
        "min_angular_velocity",
        "max_angular_velocity",
    ] {
        for value in ["NaN", "inf", "-inf", "", "1 2", "wrong"] {
            body(&mut model, &format!("{JOINTS}<{name}>{value}</{name}>"));
            refused(&model, "DiffDrive velocity limits");
        }
    }
}

#[test]
fn closed_plugin_xml_rejects_unknown_repeated_namespaced_and_lexically_split_fields() {
    let mut model = model();
    body(&mut model, JOINTS);
    let original = model.plugins[0].xml.clone();
    for raw in [
        "<".into(),
        original
            .replace("<plugin ", "<wrong ")
            .replace("</plugin>", "</wrong>"),
        original.replace("systems::DiffDrive", "systems::Other"),
        original.replace("gz-sim-diff-drive-system", "other"),
        original.replace("<plugin ", "<plugin extra='x' "),
        original.replace("<plugin ", "<plugin xmlns:e='urn:e' "),
        original.replace("<left_joint>", "<left_joint extra='x'>"),
        original.replace("<left_joint>", "<left_joint xmlns='urn:e'>"),
        original.replace("left_wheel_joint</left_joint>", "<nested/></left_joint>"),
        original.replace(
            "left_wheel_joint</left_joint>",
            "left<!--split-->_wheel_joint</left_joint>",
        ),
        original.replace(
            "left_wheel_joint</left_joint>",
            "<![CDATA[left_wheel_joint]]></left_joint>",
        ),
        original.replace("</plugin>", "text</plugin>"),
        original.replace("</plugin>", "<?instruction value?></plugin>"),
    ] {
        model.plugins[0].xml = raw;
        refused(&model, "DiffDrive configuration");
    }
    for field in [
        "wheel_radius",
        "wheel_separation",
        "odom_publish_frequency",
        "topic",
        "odom_topic",
        "frame_id",
        "child_frame_id",
        "min_velocity",
        "max_velocity",
        "min_linear_velocity",
        "max_linear_velocity",
        "min_angular_velocity",
        "max_angular_velocity",
        "left_joint",
        "right_joint",
    ] {
        body(
            &mut model,
            &format!("{JOINTS}<{field}>1</{field}><{field}>1</{field}>"),
        );
        refused(&model, "DiffDrive configuration");
    }
    for field in [
        "min_acceleration",
        "max_acceleration",
        "min_jerk",
        "max_jerk",
        "min_linear_acceleration",
        "max_angular_jerk",
        "enable_topic",
        "tf_topic",
        "namespace",
        "unknown",
    ] {
        body(&mut model, &format!("{JOINTS}<{field}>1</{field}>"));
        refused(&model, "DiffDrive configuration");
    }
    body(&mut model, &format!("<!--configuration-->{JOINTS}"));
    assert!(parse(&model).is_ok());
}

#[test]
fn plugin_dispatch_preserves_unrelated_systems_and_refuses_duplicate_drive_owners() {
    let mut model = model();
    let drive = model.plugins[0].clone();
    assert!(is_plugin(&drive));
    model.plugins[0].name = "gz::sim::systems::ApplyJointForce".into();
    assert!(!is_plugin(&model.plugins[0]));
    assert!(parse(&model).unwrap().is_none());
    model.plugins[0] = drive.clone();
    model.plugins[0].filename = "other".into();
    assert!(!is_plugin(&model.plugins[0]));
    assert!(parse(&model).unwrap().is_none());
    model.plugins[0] = drive.clone();
    let mut other = drive.clone();
    other.name = "x".into();
    other.filename = "x".into();
    model.plugins.push(other);
    assert!(parse(&model).unwrap().is_some());
    model.plugins.push(drive);
    refused(&model, "duplicate DiffDrive system");
    model.plugins.clear();
    assert!(parse(&model).unwrap().is_none());
}

#[test]
fn wheel_geometry_and_rate_require_finite_positive_representable_values() {
    let mut model = model();
    for field in ["wheel_radius", "wheel_separation", "odom_publish_frequency"] {
        for value in ["0", "-1", "NaN", "inf", "-inf", "", "wrong"] {
            body(&mut model, &format!("{JOINTS}<{field}>{value}</{field}>"));
            refused(
                &model,
                if field == "odom_publish_frequency" {
                    "DiffDrive rate"
                } else {
                    "DiffDrive geometry"
                },
            );
        }
    }
    for field in ["wheel_radius", "wheel_separation"] {
        body(&mut model, &format!("{JOINTS}<{field}>5e-324</{field}>"));
        refused(&model, "DiffDrive geometry");
    }
    for missing in [
        "",
        "<left_joint>left_wheel_joint</left_joint>",
        "<right_joint>right_wheel_joint</right_joint>",
        "<left_joint/><right_joint>right_wheel_joint</right_joint>",
        "<left_joint>left_wheel_joint</left_joint><right_joint>left_wheel_joint</right_joint>",
        "<left_joint>nested::left_wheel_joint</left_joint><right_joint>right_wheel_joint</right_joint>",
        "<left_joint>missing</left_joint><right_joint>right_wheel_joint</right_joint>",
    ] {
        body(&mut model, missing);
        refused(&model, "DiffDrive wheel pair");
    }
}

#[test]
fn wheel_topology_frames_and_motor_limits_cannot_silently_broaden_the_pair_domain() {
    let original = model();
    for mutate in [
        (|m: &mut Model| m.is_static = true) as fn(&mut Model),
        |m| m.raw_namespace = Some("robot".into()),
        |m| m.models.push(m.clone()),
    ] {
        let mut model = original.clone();
        mutate(&mut model);
        refused(&model, "DiffDrive model domain");
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
        |m| m.canonical_link = "missing".into(),
        |m| m.links[0].pose.relative_to = "other".into(),
        |m| m.links[0].pose.transform.translation[0] = 1.0,
        |m| m.joints[0].kind = JointKind::Fixed,
        |m| m.joints[0].parent = "world".into(),
        |m| m.joints[0].child = "missing".into(),
        |m| m.joints[0].child = "base_link".into(),
        |m| m.joints[1].child = m.joints[0].child.clone(),
        |m| m.joints[0].axis2 = m.joints[0].axis.clone(),
        |m| m.joints[0].axis = None,
        |m| m.joints[0].pose.transform.translation[0] = 0.1,
        |m| m.joints[0].pose.transform.rotation = [1.0, 0.0, 0.0, 0.0],
        |m| m.joints[0].pose.relative_to = "base_link".into(),
        |m| m.links[1].pose.relative_to = "base_link".into(),
        |m| m.links[1].pose.transform.rotation = [1.0, 0.0, 0.0, 0.0],
        |m| m.links[1].pose.transform.translation[0] = 0.1,
        |m| m.links[1].pose.transform.translation[2] = 0.1,
        |m| m.links[1].pose.transform.translation[1] = 0.3,
        |m| m.links[1].pose.transform.translation[1] = -0.25,
        |m| m.links[1].pose.transform.translation[1] = 0.0,
        |m| m.links[1].pose.transform.translation[0] = f64::NAN,
    ] {
        let mut model = original.clone();
        mutate(&mut model);
        refused(&model, "DiffDrive wheel pair");
    }
    for mutate in [
        (|m: &mut Model| m.joints[0].axis.as_mut().unwrap().xyz = [0.0, -1.0, 0.0])
            as fn(&mut Model),
        |m| m.joints[0].axis.as_mut().unwrap().expressed_in = "__model__".into(),
    ] {
        let mut model = original.clone();
        mutate(&mut model);
        refused(&model, "DiffDrive wheel limits");
    }
    for value in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
        for index in 0..2 {
            let mut model = original.clone();
            model.joints[index].axis.as_mut().unwrap().effort = value;
            refused(&model, "DiffDrive wheel limits");
            let mut model = original.clone();
            model.joints[index].axis.as_mut().unwrap().velocity = value;
            refused(&model, "DiffDrive wheel limits");
        }
    }
    let mut model = original;
    for joint in &mut model.joints {
        joint.pose.relative_to.clear();
    }
    assert!(parse(&model).is_ok()); // The default joint pose frame is its child.
}

#[test]
fn topics_remain_absolute_distinct_and_frames_use_native_empty_value_fallbacks() {
    let mut model = model();
    body(
        &mut model,
        &format!(
            "{JOINTS}<topic>/custom/cmd</topic><odom_topic>/custom/odom</odom_topic><frame_id>custom/odom</frame_id><child_frame_id>custom/base</child_frame_id>"
        ),
    );
    let p = parse(&model).unwrap().unwrap().publication;
    assert_eq!(
        (
            p.topic.as_str(),
            p.odom_topic.as_str(),
            p.frame_id.as_str(),
            p.child_frame_id.as_str()
        ),
        ("/custom/cmd", "/custom/odom", "custom/odom", "custom/base")
    );
    body(
        &mut model,
        &format!("{JOINTS}<frame_id/><child_frame_id> \n\t </child_frame_id>"),
    );
    let p = parse(&model).unwrap().unwrap().publication;
    assert_eq!(
        (p.frame_id.as_str(), p.child_frame_id.as_str()),
        ("robot/odom", "robot/base_link")
    );
    for field in ["topic", "odom_topic"] {
        for value in [
            "",
            "/",
            "relative",
            "/bad topic",
            "/bad//topic",
            "/bad@topic",
            "/bad~topic",
            "/bad:=topic",
        ] {
            body(&mut model, &format!("{JOINTS}<{field}>{value}</{field}>"));
            refused(&model, "DiffDrive topics");
        }
        body(
            &mut model,
            &format!("{JOINTS}<{field}>/{}</{field}>", "x".repeat(65535)),
        );
        refused(&model, "DiffDrive topics");
    }
    body(
        &mut model,
        &format!("{JOINTS}<topic>/same</topic><odom_topic>/same</odom_topic>"),
    );
    refused(&model, "DiffDrive topics");
}
