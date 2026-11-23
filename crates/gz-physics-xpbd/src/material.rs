// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{Particle, XpbdError, coupled::Receiver};
use glam::{DMat3, DVec3};
use std::collections::{BTreeMap, BTreeSet};

const MAX_RING: usize = 24;

/// Use both incident triangles at a shared edge and the whole incident ring
/// at a vertex. Thus the offset torque does not depend on which face owns
/// an exactly tied closest point.
pub(super) fn patches(faces: &[[usize; 3]]) -> Result<BTreeMap<[usize; 3], Vec<usize>>, XpbdError> {
    let mut map = BTreeMap::<[usize; 3], BTreeSet<usize>>::new();
    for &[a, b, c] in faces {
        for key in [
            [a, b, c],
            [a, b, usize::MAX],
            [b, c, usize::MAX],
            [a, c, usize::MAX],
            [a, usize::MAX, usize::MAX],
            [b, usize::MAX, usize::MAX],
            [c, usize::MAX, usize::MAX],
        ] {
            let ring = map.entry(key).or_default();
            ring.extend([a, b, c]);
            if ring.len() > MAX_RING {
                return Err(XpbdError::SurfaceBudget);
            }
        }
    }
    Ok(map
        .into_iter()
        .map(|(k, v)| (k, v.into_iter().collect()))
        .collect())
}

/// A material patch transports an offset point by its least-squares rigid
/// angular velocity. The transpose maps a skin impulse to nodal impulses
/// preserving both the total impulse and the thickness-offset couple.
pub(super) struct Skin<'a> {
    ids: &'a [usize],
    weights: [f64; MAX_RING],
    arms: [DVec3; MAX_RING],
    inverse: DMat3,
    offset: DVec3,
}
impl<'a> Skin<'a> {
    pub fn new(
        ids: &'a [usize],
        face: [usize; 3],
        bary: [f64; 3],
        at: impl Fn(usize) -> DVec3,
        offset: DVec3,
    ) -> Result<Self, XpbdError> {
        let origin = at(ids[0]);
        let center = ids.iter().map(|&i| at(i) - origin).sum::<DVec3>() / ids.len() as f64;
        let mut arms = [DVec3::ZERO; MAX_RING];
        let mut weights = [0.; MAX_RING];
        let mut inertia = DMat3::ZERO;
        for (i, &id) in ids.iter().enumerate() {
            let a = at(id) - origin - center;
            arms[i] = a;
            if let Some(j) = face.iter().position(|&j| j == id) {
                weights[i] = bary[j];
            }
            inertia +=
                DMat3::IDENTITY * a.length_squared() - DMat3::from_cols(a * a.x, a * a.y, a * a.z);
        }
        let scale = inertia.x_axis.x.max(inertia.y_axis.y).max(inertia.z_axis.z);
        let scaled = inertia / scale;
        if !scale.is_finite() || scale <= 0. || scaled.determinant() < 1e-12 {
            return Err(XpbdError::UnresolvedSurface);
        }
        Ok(Self {
            ids,
            weights,
            arms,
            inverse: scaled.inverse() / scale,
            offset,
        })
    }
    pub fn nodal(&self, j: DVec3) -> impl Iterator<Item = (usize, DVec3)> + '_ {
        let torque = self.inverse * self.offset.cross(j);
        self.ids
            .iter()
            .enumerate()
            .map(move |(i, &id)| (id, self.weights[i] * j + torque.cross(self.arms[i])))
    }
    pub fn velocity(&self, p: &[Particle]) -> DVec3 {
        let mut v = DVec3::ZERO;
        let mut spin = DVec3::ZERO;
        for (i, &id) in self.ids.iter().enumerate() {
            let vi = DVec3::from(p[id].velocity);
            v += self.weights[i] * vi;
            spin += self.arms[i].cross(vi);
        }
        v + (self.inverse * spin).cross(self.offset)
    }
    pub fn apply(&self, j: DVec3, p: &mut [Particle]) {
        for (i, j) in self.nodal(j) {
            let q = &mut p[i];
            q.velocity = (DVec3::from(q.velocity) + q.inverse_mass * j).to_array();
        }
    }
    pub fn inverse_along(
        &self,
        p: &[Particle],
        r: &Receiver,
        arm: DVec3,
        d: DVec3,
    ) -> Result<f64, XpbdError> {
        let w = self
            .nodal(d)
            .map(|(id, j)| p[id].inverse_mass * j.length_squared())
            .sum::<f64>();
        r.directional_inverse_mass(w + r.inverse_mass, arm, d)
    }
}
