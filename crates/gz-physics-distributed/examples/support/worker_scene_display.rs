// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Display the isolated fixture worlds compactly while preserving geometry and motion within each.
use crate::{Result, primitive_replay_view::View};
use gz_physics_distributed::physics::{Body, ContactImpulse, GroundSupport, Primitive};
use serde::Deserialize;
use serde_json::json;
use std::collections::BTreeMap;
#[derive(Deserialize)]
pub struct Frame {
    pub tick: u64,
    pub bodies: Vec<Body>,
    pub owners: Vec<u8>,
    pub pairs: Vec<ContactImpulse>,
    pub floors: Vec<GroundSupport>,
    pub loaded_pairs: [usize; 2],
}
pub const OWNER_COLORS: [[f64; 4]; 8] = [
    [0.14, 0.48, 0.86, 1.],
    [0.98, 0.50, 0.11, 1.],
    [0.62, 0.30, 0.76, 1.],
    [0.09, 0.65, 0.45, 1.],
    [0.87, 0.27, 0.32, 1.],
    [0.16, 0.67, 0.70, 1.],
    [0.87, 0.76, 0.30, 1.],
    [0.60, 0.62, 0.69, 1.],
];

pub struct BodyLayer {
    shifts: BTreeMap<u64, [f64; 2]>,
    last: [Option<u8>; 62],
    generation: [u64; 62],
    arm: usize,
    label: &'static str,
    dy: f64,
}
impl BodyLayer {
    pub fn new(groups: &[Vec<Body>], arm: usize, label: &'static str, dy: f64) -> Self {
        let shifts = groups
            .iter()
            .flat_map(|g| {
                let [x, y, _] = g[0].position;
                g.iter().map(move |b| (b.id, [-0.56 * x, -0.56 * y]))
            })
            .collect();
        Self {
            shifts,
            last: [None; 62],
            generation: [0; 62],
            arm,
            label,
            dy,
        }
    }
    pub fn draw(&mut self, v: &mut View, f: &Frame, map: &[u8]) -> Result<()> {
        if map.len() != f.bodies.len() {
            return Err("ownership count differs from body count".into());
        }
        for (b, &owner) in f.bodies.iter().zip(map) {
            let i = usize::try_from(b.id)?.checked_sub(1).ok_or("zero ID")?;
            let prior = self.last.get_mut(i).ok_or("unknown fixture body")?;
            if prior.is_some_and(|old| old != owner) {
                self.generation[i] += 1;
            }
            *prior = Some(owner);
            let [dx, dy] = self.shifts[&b.id];
            let [x, y, z] = b.position;
            let shape = match b.primitive {
                Primitive::Box => json!({"kind":"box","size":b.size}),
                Primitive::Sphere => json!({"kind":"sphere","radius":b.size[0]/2.}),
                _ => return Err("unexpected fixture shape".into()),
            };
            v.add(
                30000
                    + 2 * (self.arm as u64 * 1000
                        + self.generation[i] * 2000
                        + u64::from(owner) * 100
                        + b.id),
                &format!("{}_body_{}_worker_{owner}", self.label, b.id),
                [x + dx, y + dy + self.dy, z],
                b.orientation,
                shape,
                *OWNER_COLORS.get(owner as usize).ok_or("bad worker slot")?,
            );
        }
        Ok(())
    }
}
