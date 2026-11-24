// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::Result;
/// Vary physical parameters of the same authored robot. For a heavier rigid
/// parcel at the same dimensions, scale its entire inertia tensor with mass.
pub fn world(base: &str, mass_scale: f64, mirror: bool, friction: Option<f64>) -> Result<String> {
    let doc = roxmltree::Document::parse(base)?;
    let parcel = doc
        .descendants()
        .find(|n| n.has_tag_name("model") && n.attribute("name") == Some("parcel"))
        .unwrap();
    let inertial = parcel
        .descendants()
        .find(|n| n.has_tag_name("inertial"))
        .unwrap();
    let mut edits = Vec::new();
    for node in doc.descendants().filter(|n| n.is_text()) {
        let parent = node.parent().unwrap();
        let tag = parent.tag_name().name();
        let value = match tag {
            "mu" | "mu2" => friction.map(|mu| mu.to_string()),
            "pose" if mirror && parent.parent() == Some(inertial) => {
                Some("-0.0045 0 0 0 0 0".to_string())
            }
            _ if mass_scale != 1.
                && parent.ancestors().any(|n| n == inertial)
                && matches!(tag, "mass" | "ixx" | "iyy" | "izz" | "ixy" | "iyz" | "ixz") =>
            {
                Some((node.text().unwrap().parse::<f64>()? * mass_scale).to_string())
            }
            _ => None,
        };
        if let Some(value) = value {
            edits.push((node.range(), value));
        }
    }
    let mut xml = base.to_owned();
    for (range, value) in edits.into_iter().rev() {
        xml.replace_range(range, &value);
    }
    Ok(xml)
}
