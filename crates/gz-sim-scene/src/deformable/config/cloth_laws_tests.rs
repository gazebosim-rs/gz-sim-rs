// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;

fn xml(extra: &str) -> String {
    format!(
        r#"<cloth columns="2" substeps="1" iterations="4" radius="0.01" self_contact="false" friction="0" stretch="0.001" shear="0.002" bend="0.03">
        <particle position="0 0 0" mass="1"/><particle position="1 0 0" mass="1"/>
        <particle position="0 1 0" mass="1"/><particle position="1 1 0" mass="1"/>
        {extra}</cloth>"#
    )
}
fn load(s: &str) -> Result<SoftBody, String> {
    let doc = roxmltree::Document::parse(s).unwrap();
    soft_body(doc.root_element(), [0.; 3], Duration::from_millis(1))
}

#[test]
fn overrides_apply_to_the_shared_diagonal_and_preserve_other_rest_laws() {
    let distance = r#"<distance_compliance particles="1 0" value="0.04"/>"#;
    let hinge = r#"<hinge_compliance particles="3 0" value="0.06"/>"#;
    let original = load(&xml("")).unwrap();
    let custom = load(&xml(&format!("{distance}{hinge}"))).unwrap();
    assert_eq!(custom.particles(), original.particles());
    for (a, b) in custom.constraints().iter().zip(original.constraints()) {
        let selected = [a.a, a.b] == [0, 1];
        assert_eq!(a.rest_length, b.rest_length);
        assert_eq!(a.compliance, if selected { 0.04 } else { b.compliance });
    }
    assert_eq!(custom.constraints().len(), 5);
    assert_eq!(custom.bending_constraints().len(), 1);
    assert_eq!(custom.bending_constraints()[0].compliance, 0.06);
    assert_eq!(
        custom.bending_constraints()[0].rest_angle,
        original.bending_constraints()[0].rest_angle
    );
}

#[test]
fn duplicate_invalid_and_unreferenced_laws_fail_before_simulation() {
    let good = r#"<hinge_compliance particles="0 3" value="0.04"/>"#;
    for (src, msg) in [
        (
            xml(&format!("{good}{}", good.replace("0 3", "3 0"))),
            "duplicate",
        ),
        (
            xml(&good.replace("0 3", "0 1")),
            "unknown distance edge or hinge",
        ),
        (
            xml(&good.replace("0 3", "0 0")),
            "unknown distance edge or hinge",
        ),
        (xml(&good.replace("0 3", "0 1 3")), "exactly two"),
        (xml(&good.replace("0 3", "-1 3")), "two particle indices"),
        (xml(&good.replace("0.04", "NaN")), "invalid or duplicate"),
        (xml(&good.replace("0.04", "-0.1")), "invalid or duplicate"),
        (xml(&good.replace("0.04", "bad")), "invalid value"),
        (xml(&good.replace(r#"value="0.04""#, "")), "missing value"),
        (
            xml(&good.replace(r#"particles="0 3""#, "")),
            "missing particles",
        ),
        (xml(&good.replace("value=", "other=")), "unexpected content"),
        (
            xml(good).replace(r#"stretch="0.001""#, r#"stretch="-1""#),
            "invalid cloth compliance",
        ),
        (
            xml(good).replace(r#"columns="2""#, r#"columns="0""#),
            "invalid soft body topology",
        ),
        (
            xml("").replace(r#"columns="2""#, r#"columns="0""#),
            "invalid soft body topology",
        ),
    ] {
        assert!(load(&src).err().unwrap().contains(msg), "{msg}");
    }
}
