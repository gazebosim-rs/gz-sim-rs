// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Optional material-law overrides on the existing triangular cloth grid.
use super::*;

pub(super) fn is_law(node: Node<'_, '_>) -> bool {
    node.has_tag_name("distance_compliance") || node.has_tag_name("hinge_compliance")
}

pub(super) fn build(
    node: Node<'_, '_>,
    particles: Vec<Particle>,
    columns: usize,
    stretch: f64,
    shear: f64,
    bend: f64,
    config: SolverConfig,
) -> Result<SoftBody, String> {
    let mut laws = node.children().filter(|n| is_law(*n)).peekable();
    if laws.peek().is_none() {
        return SoftBody::cloth(particles, columns, stretch, shear, bend, config)
            .map_err(|e| e.to_string());
    }
    if [stretch, shear, bend]
        .into_iter()
        .any(|c| !c.is_finite() || c < 0.)
    {
        return Err("invalid cloth compliance".into());
    }
    let mut distance = BTreeMap::new();
    let mut hinge = BTreeMap::new();
    for law in laws {
        fields(law, &["particles", "value"], false)?;
        let mut ids = attribute(law, "particles")?.split_whitespace();
        let mut pair = [0; 2];
        for id in &mut pair {
            *id = ids
                .next()
                .and_then(|s| s.parse::<usize>().ok())
                .ok_or("expected two particle indices")?;
        }
        if ids.next().is_some() {
            return Err("expected exactly two particle indices".into());
        }
        pair.sort_unstable();
        let value: f64 = number(law, "value")?;
        let table = if law.has_tag_name("distance_compliance") {
            &mut distance
        } else {
            &mut hinge
        };
        if !value.is_finite() || value < 0. || table.insert(pair, value).is_some() {
            return Err("invalid or duplicate cloth compliance override".into());
        }
    }
    let body = SoftBody::cloth_with_material_laws(
        particles,
        columns,
        config,
        |a, b| {
            distance.remove(&[a.min(b), a.max(b)]).unwrap_or_else(|| {
                if a / columns != b / columns && a % columns != b % columns {
                    shear
                } else {
                    stretch
                }
            })
        },
        |[a, b, _, _]| hinge.remove(&[a.min(b), a.max(b)]).unwrap_or(bend),
    )
    .map_err(|e| e.to_string())?;
    if !distance.is_empty() || !hinge.is_empty() {
        return Err("cloth compliance refers to an unknown distance edge or hinge".into());
    }
    Ok(body)
}

#[cfg(test)]
#[path = "cloth_laws_tests.rs"]
mod tests;
