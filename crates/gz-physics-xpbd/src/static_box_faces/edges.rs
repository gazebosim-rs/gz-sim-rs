// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Finite cuboid ridges against unique material edges of the triangular sheet.
use super::{Bounds, Row, cell_faces};
use crate::{
    Particle, XpbdError,
    surface::{first_entry_mode, geometry::edge_edge},
};
use glam::{DQuat, DVec3};
use std::{collections::BTreeMap, sync::Arc};

#[derive(Clone, Copy)]
pub(super) struct Edge {
    ends: [usize; 2],
    wing: usize,
    other: Option<usize>,
}
pub(super) fn topology(count: usize, columns: usize) -> Arc<[Edge]> {
    let mut edges = BTreeMap::new();
    for cell in columns + 1..count {
        if cell % columns != 0 {
            for f in cell_faces(cell, columns) {
                for i in 0..3 {
                    let (a, b, c) = (f[i], f[(i + 1) % 3], f[(i + 2) % 3]);
                    let ends = [a.min(b), a.max(b)];
                    edges
                        .entry(ends)
                        .and_modify(|e: &mut Edge| e.other = Some(c))
                        .or_insert(Edge {
                            ends,
                            wing: c,
                            other: None,
                        });
                }
            }
        }
    }
    edges.into_values().collect()
}

pub(super) struct Ridge {
    end: [DVec3; 2],
    translation: DVec3,
    axis: usize,
    signs: DVec3,
    index: usize,
}
impl Ridge {
    pub(super) fn all(
        center: DVec3,
        half: DVec3,
        q: DQuat,
        translation: DVec3,
    ) -> impl Iterator<Item = Self> {
        (0..12).map(move |index| {
            let axis = index / 4;
            let mut signs = DVec3::ZERO;
            signs[(axis + 1) % 3] = if index & 1 == 0 { -1. } else { 1. };
            signs[(axis + 2) % 3] = if index & 2 == 0 { -1. } else { 1. };
            let middle = center + q * (signs * half);
            let dir = q * (DVec3::AXES[axis] * half[axis]);
            Self {
                end: [middle - dir, middle + dir],
                translation,
                axis,
                signs,
                index,
            }
        })
    }
    pub(super) fn overlaps(&self, b: &Bounds) -> bool {
        let [a, z] = self.end;
        !a.min(z).cmpgt(b.hi).any() && !a.max(z).cmplt(b.lo).any()
    }
    fn feature(&self, s: f64, n: DVec3) -> Option<usize> {
        let mut signs = self.signs;
        if s < 1e-10 {
            signs[self.axis] = -1.;
        } else if s > 1. - 1e-10 {
            signs[self.axis] = 1.;
        } else if n[self.axis].abs() > 1e-8 {
            return None;
        }
        if (signs * n).min_element() < -1e-8 {
            return None;
        }
        if signs[self.axis] == 0. {
            Some(8 + self.index)
        } else {
            Some(
                usize::from(signs.x > 0.)
                    | (usize::from(signs.y > 0.) << 1)
                    | (usize::from(signs.z > 0.) << 2),
            )
        }
    }
    pub(super) fn sweep(
        &self,
        edge: Edge,
        world_to_local: DQuat,
        p: &[Particle],
        x: &[DVec3],
        radius: f64,
        queries: &mut usize,
    ) -> Result<Option<(usize, Row)>, XpbdError> {
        let [i, j] = edge.ends;
        if p[i].inverse_mass == 0. && p[j].inverse_mass == 0. {
            return Ok(None);
        }
        let a = [
            self.end[0],
            self.end[1],
            p[i].position.into(),
            p[j].position.into(),
        ];
        let z = [a[0], a[1], x[i] - self.translation, x[j] - self.translation];
        if !self.overlaps(&Bounds::swept(
            [(a[2], z[2]), (a[3], z[3])].into_iter(),
            radius,
        )) {
            return Ok(None);
        }
        let eps =
            1e-10 * radius + 128. * f64::EPSILON * a[0].abs().max(a[1].abs()).max_element().max(1.);
        let initial = edge_edge(&a);
        let (w, n, t) = if initial.delta.length() <= eps {
            if -initial.weights[2] <= 1e-10 || -initial.weights[3] <= 1e-10 {
                // This exact endpoint is owned by the particle-to-box sweep.
                return Ok(None);
            }
            let direction = a[1] - a[0];
            let crossing = direction.cross(a[3] - a[2]);
            let n = if let Some(mut n) = crossing.try_normalize() {
                if (world_to_local * n).dot(self.signs) < 0. {
                    n = -n;
                }
                n
            } else {
                // Coincident parallel edges have no geometric normal. For
                // zero skin, oppose only their incident transverse velocity.
                if radius > eps {
                    return Err(XpbdError::UnresolvedSurface);
                }
                let axis = direction.normalize();
                let v = (z[2] - a[2]).lerp(z[3] - a[3], -initial.weights[3]);
                let Some(n) = (axis * v.dot(axis) - v).try_normalize() else {
                    return Ok(None);
                };
                n
            };
            (initial.weights, n, 0.)
        } else {
            let Some(hit) =
                first_entry_mode(a, z, false, radius.max(16. * eps), eps, queries, true)?
            else {
                return Ok(None);
            };
            (hit.weights, -hit.normal, hit.time)
        };
        if -w[2] <= 1e-10 || -w[3] <= 1e-10 {
            return Ok(None);
        }
        let Some(feature) = self.feature(w[1], world_to_local * n) else {
            return Ok(None);
        };
        let pos = |id: usize| DVec3::from(p[id].position).lerp(x[id], t);
        let incident = pos(i).lerp(pos(j), -w[3]);
        // An edge cannot support a contact hidden by either incident triangle.
        if [Some(edge.wing), edge.other]
            .into_iter()
            .flatten()
            .any(|k| (pos(k) - incident).dot(n) < -eps)
        {
            return Ok(None);
        }
        Ok(Some((
            feature,
            Row {
                ids: [i, j, edge.wing],
                other_wing: edge.other,
                weights: [-w[2], -w[3], 0.],
                normal: n,
                witness: self.end[0].lerp(self.end[1], w[1]) + self.translation,
                lambda: 0.,
            },
        )))
    }
}

#[cfg(test)]
#[path = "../../tests/private/static_box_edges.rs"]
mod tests;
