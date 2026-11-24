// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::{Plugin, triple};
use gz_physics::trajectory::{PathEnd, TranslationSpline, Waypoint};
use roxmltree::Node;
use std::time::Duration;
fn contents<'a, 'b>(n: Node<'a, 'b>) -> impl Iterator<Item = Node<'a, 'b>> {
    n.children().filter(|v| {
        !v.is_comment() && !(v.is_text() && v.text().is_some_and(|t| t.trim().is_empty()))
    })
}
fn attrs(n: Node<'_, '_>, tag: &str, keys: &[&str]) -> bool {
    n.is_element()
        && n.has_tag_name(tag)
        && n.namespaces().next().is_none()
        && n.attributes().len() == keys.len()
        && keys.iter().all(|k| n.attribute(*k).is_some())
}
pub(super) fn parse(plugin: &Plugin) -> Result<TranslationSpline, &'static str> {
    let doc = roxmltree::Document::parse(&plugin.xml).map_err(|_| "invalid trajectory XML")?;
    let root = doc.root_element();
    if !attrs(root, "plugin", &["name", "filename"])
        || root.attribute("name") != Some(&plugin.name)
        || root.attribute("filename") != Some(&plugin.filename)
    {
        return Err("invalid trajectory plugin");
    }
    let mut children = contents(root);
    let node = children.next().ok_or("missing trajectory")?;
    if children.next().is_some() || !attrs(node, "trajectory", &["end"]) {
        return Err("expected one trajectory with an end policy");
    }
    let end = match node.attribute("end") {
        Some("hold") => PathEnd::Hold,
        Some("repeat") => PathEnd::Repeat,
        _ => return Err("trajectory end must be hold or repeat"),
    };
    let mut points = Vec::new();
    for k in contents(node) {
        if !attrs(k, "waypoint", &["time_ns", "displacement", "velocity"])
            || contents(k).next().is_some()
        {
            return Err("expected waypoint time_ns, displacement and velocity");
        }
        let time = Duration::from_nanos(
            k.attribute("time_ns")
                .and_then(|v| v.parse::<u64>().ok())
                .ok_or("invalid waypoint time_ns")?,
        );
        let vector = |name| {
            triple(k.attribute(name).unwrap()).ok_or("expected three finite waypoint components")
        };
        points.push(Waypoint {
            time,
            displacement: vector("displacement")?,
            velocity: vector("velocity")?,
        });
    }
    TranslationSpline::new(&points, end)
}
