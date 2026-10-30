// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

#[test]
fn plugin_scalar_text_matches_native_or_rejects_lexical_splits() {
    let fixture: serde_json::Value =
        serde_json::from_str(include_str!("../fixtures/joint_force_text.json")).unwrap();
    for row in fixture["observations"].as_array().unwrap() {
        let xml = row["xml"].as_str().unwrap();
        let document = roxmltree::Document::parse(xml).unwrap();
        let actual = joint_text(xml, document.root_element());
        if row["supported"] == true {
            assert_eq!(row["stderr"], "");
            let lines: Vec<_> = row["stdout"].as_str().unwrap().lines().collect();
            assert_eq!(lines[0], "1 0");
            let hex = lines[1];
            let bytes: Vec<_> = hex
                .as_bytes()
                .chunks_exact(2)
                .map(|v| u8::from_str_radix(std::str::from_utf8(v).unwrap(), 16).unwrap())
                .collect();
            assert_eq!(actual.unwrap().as_bytes(), bytes, "{}", row["name"]);
        } else {
            assert!(actual.is_err(), "{}", row["name"]);
        }
    }
}

#[test]
fn parser_checks_raw_xml_instead_of_trusting_plugin_metadata() {
    let source = "<sdf version='1.12'><model name='m'><joint name='hinge' type='revolute'><parent>world</parent><child>l</child></joint><link name='l'/><plugin name='gz::sim::systems::ApplyJointForce' filename='gz-sim-apply-joint-force-system'><joint_name>hinge</joint_name></plugin></model></sdf>";
    let mut document = sdformat::load_str(source, "raw.sdf", &sdformat::NoResources).unwrap();
    let valid = document.models[0].plugins[0].xml.clone();
    for xml in [
        "<".into(),
        valid
            .replace("<plugin ", "<wrong ")
            .replace("</plugin>", "</wrong>"),
        valid.replace("name='gz::sim::systems::ApplyJointForce'", "name='other'"),
        valid.replace(
            "filename='gz-sim-apply-joint-force-system'",
            "filename='other'",
        ),
        valid.replace("<plugin ", "<plugin xmlns:extra='urn:extra' "),
        valid.replace("<plugin ", "<plugin extra='x' "),
        valid.replace("<joint_name>", "<joint_name xmlns='urn:extra'>"),
        valid.replace("<joint_name>", "<joint_name extra='x'>"),
        valid.replace("joint_name", "different"),
        valid.replace("hinge</joint_name>", "<nested/></joint_name>"),
        valid.replace("hinge</joint_name>", "</joint_name>"),
        valid.replace("<joint_name>hinge</joint_name>", "text"),
        valid.replace("<joint_name>hinge</joint_name>", ""),
    ] {
        let model = &mut document.models[0];
        model.plugins[0].xml = xml.clone();
        assert!(parse(model).is_err(), "{xml}");
    }
}
