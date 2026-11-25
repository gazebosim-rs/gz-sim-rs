// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Physical load measurements for a user-supplied serial workcell.
use super::*;
use roxmltree::Node;
use std::{borrow::Cow, ops::Range};

/// Keep every authored joint, pose and material outside the selected parcel and finger faces.
pub(super) fn configure(
    xml: &str,
    mass: Option<f64>,
    friction: Option<f64>,
) -> Result<Cow<'_, str>> {
    if mass.is_none() && friction.is_none() {
        return Ok(Cow::Borrowed(xml));
    }
    if mass.is_some_and(|v| !v.is_finite() || v <= 0.) {
        return Err("parcel mass must be finite and positive".into());
    }
    if friction.is_some_and(|v| !v.is_finite() || v < 0.) {
        return Err("grip friction must be finite and nonnegative".into());
    }

    let doc = roxmltree::Document::parse(xml)?;
    let world = named(doc.root_element(), "world", Some("default"))?;
    let link = |m, l| named(named(world, "model", Some(m))?, "link", Some(l));
    let parcel = link("parcel", "box")?;
    let mut edits: Vec<(Range<usize>, String)> = Vec::new();
    let mut replace = |node: Node<'_, '_>, value: f64| -> Result<()> {
        let text = node
            .first_child()
            .filter(Node::is_text)
            .ok_or("expected a numeric SDF element")?;
        if !value.is_finite() {
            return Err("scaled parcel inertia is outside the finite range".into());
        }
        edits.push((text.range(), value.to_string()));
        Ok(())
    };
    if let Some(mass) = mass {
        let body = named(parcel, "inertial", None)?;
        let mass_node = named(body, "mass", None)?;
        let old: f64 = mass_node.text().unwrap_or_default().trim().parse()?;
        if !old.is_finite() || old <= 0. {
            return Err("authored parcel mass must be finite and positive".into());
        }
        replace(mass_node, mass)?;
        let tensor = named(body, "inertia", None)?;
        for axis in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"] {
            let component = if matches!(axis, "ixx" | "iyy" | "izz") {
                named(tensor, axis, None)?
            } else if let Some(n) = tensor.children().find(|n| n.has_tag_name(axis)) {
                n
            } else {
                continue; // Omitted cross terms are zero.
            };
            let previous: f64 = component.text().unwrap_or_default().trim().parse()?;
            replace(component, previous * (mass / old))?;
        }
    }
    if let Some(friction) = friction {
        for (link, collision) in [
            (parcel, "box"),
            (link("gripper", "left")?, "left"),
            (link("gripper", "right")?, "right"),
        ] {
            let mut node = named(link, "collision", Some(collision))?;
            for element in ["surface", "friction", "ode"] {
                node = named(node, element, None)?;
            }
            for coef in ["mu", "mu2"] {
                replace(named(node, coef, None)?, friction)?;
            }
        }
    }
    edits.sort_unstable_by_key(|(range, _)| std::cmp::Reverse(range.start));
    let mut result = xml.to_owned();
    for (at, value) in edits {
        result.replace_range(at, &value);
    }
    Ok(Cow::Owned(result))
}

fn named<'a, 'i>(parent: Node<'a, 'i>, tag: &str, name: Option<&str>) -> Result<Node<'a, 'i>> {
    parent
        .children()
        .find(|n| n.has_tag_name(tag) && name.is_none_or(|s| n.attribute("name") == Some(s)))
        .ok_or_else(|| {
            format!("missing workcell element {tag} {}", name.unwrap_or(""))
                .trim_end()
                .to_owned()
                .into()
        })
}

/// The receiving face in this workcell is horizontal. Its load is the vertical
/// component of parcel weight, measured in the same world axes as the sensor.
pub(super) fn weight(sdf: &sdformat::Sdf) -> Result<f64> {
    let world = sdf
        .worlds
        .iter()
        .find(|world| world.name == "default")
        .ok_or("missing serial workcell")?;
    let link = world
        .models
        .iter()
        .find(|model| model.name == "parcel")
        .and_then(|model| model.links.iter().find(|link| link.name == "box"))
        .ok_or("missing parcel inertial")?;
    Ok(-world.gravity[2] * link.inertial.mass)
}

#[derive(Debug, PartialEq, Serialize)]
pub(super) struct Loads {
    pub(super) weight_n: f64,
    min_pad_normal_n: [f64; 2],
    mean_pad_normal_n: [f64; 2],
    first_missing_bilateral_grip_ms: Option<usize>,
    tray_loaded_traverse_ticks: usize,
    shelf_loaded_traverse_ticks: usize,
}
impl Loads {
    pub(super) fn new(weight_n: f64) -> Self {
        Self {
            weight_n,
            min_pad_normal_n: [f64::INFINITY; 2],
            mean_pad_normal_n: [0.; 2],
            first_missing_bilateral_grip_ms: None,
            tray_loaded_traverse_ticks: 0,
            shelf_loaded_traverse_ticks: 0,
        }
    }
    pub(super) fn observe(&mut self, tick: usize, reading: &cycle::Reading, intervals: usize) {
        for (i, force) in reading.pad.into_iter().enumerate() {
            self.min_pad_normal_n[i] = self.min_pad_normal_n[i].min(force);
            self.mean_pad_normal_n[i] += force / intervals as f64;
        }
        if !reading.both_pads_loaded {
            self.first_missing_bilateral_grip_ms.get_or_insert(tick);
        }
        self.tray_loaded_traverse_ticks += usize::from(reading.face_loaded_any[0]);
        self.shelf_loaded_traverse_ticks += usize::from(reading.face_loaded_any[1]);
    }
}

#[cfg(test)]
#[path = "payload/tests.rs"]
pub(super) mod tests;
