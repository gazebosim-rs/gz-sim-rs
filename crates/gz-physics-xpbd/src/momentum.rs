// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Torque-free structural impulses on a substep, followed by a velocity
//! projection for ideal (zero compliance) constraints.
use super::{DihedralConstraint, DistanceConstraint, Particle, SoftBody, XpbdError, bending};
use glam::DVec3;

#[derive(Clone)]
pub(super) struct MomentumRows {
    pub distance: Vec<DVec3>,
    pub bending: Vec<[DVec3; 4]>,
    hard_distance: Vec<usize>,
    hard_bending: Vec<usize>,
}
impl MomentumRows {
    pub fn new(body: &SoftBody) -> Self {
        Self {
            distance: vec![DVec3::ZERO; body.constraints.len()],
            bending: vec![[DVec3::ZERO; 4]; body.bending.len()],
            hard_distance: body
                .constraints
                .iter()
                .enumerate()
                .filter_map(|(i, c)| (c.compliance == 0.).then_some(i))
                .collect(),
            hard_bending: body
                .bending
                .iter()
                .enumerate()
                .filter_map(|(i, c)| (c.compliance == 0.).then_some(i))
                .collect(),
        }
    }
    pub fn substeps(&self, body: &SoftBody, dt: std::time::Duration) -> Result<u32, XpbdError> {
        let h = dt.as_secs_f64();
        let mut count = f64::from(body.config.substeps);
        // A hard distance projects along its starting separation. Limit free
        // relative travel before that line is nearly perpendicular to the new
        // gradient. A zero-length temporary chord uses its authored scale.
        for &i in &self.hard_distance {
            let c = body.constraints[i];
            let (a, b) = (body.particles[c.a], body.particles[c.b]);
            let length = DVec3::from(a.position).distance(b.position.into());
            let scale = if length > 0. { length } else { c.rest_length };
            let relative = DVec3::from(a.velocity) - DVec3::from(b.velocity);
            count = count.max((4. * h * relative.length() / scale).ceil());
        }
        // A thin triangle can have a high hinge rate even when its longest
        // edges move little. Measure the derivative of the signed dihedral.
        for &i in &self.hard_bending {
            let c = body.bending[i];
            let ps = c.vertices.map(|index| body.particles[index]);
            let Some((_, g)) = bending::geometry(ps.map(|p| p.position.into())) else {
                return Err(XpbdError::Numerical);
            };
            let v0 = DVec3::from(ps[0].velocity);
            let rate: f64 = ps
                .iter()
                .zip(g)
                .map(|(p, g)| g.dot(DVec3::from(p.velocity) - v0))
                .sum();
            count = count.max((4. * h * rate.abs()).ceil());
        }
        if !count.is_finite() || count > f64::from(body.config.substeps.max(1024)) {
            return Err(XpbdError::UnresolvedStructure);
        }
        Ok(count as u32)
    }
    pub fn prepare(
        &mut self,
        particles: &[Particle],
        distance: &[DistanceConstraint],
        rest: &[DVec3],
        hinges: &[DihedralConstraint],
    ) -> Result<(), XpbdError> {
        for ((n, c), r) in self.distance.iter_mut().zip(distance).zip(rest) {
            *n = (DVec3::from(particles[c.a].position) - DVec3::from(particles[c.b].position))
                .try_normalize()
                .unwrap_or(*r);
        }
        for (g, c) in self.bending.iter_mut().zip(hinges) {
            *g = bending::geometry(c.vertices.map(|i| DVec3::from(particles[i].position)))
                .ok_or(XpbdError::Numerical)?
                .1;
        }
        Ok(())
    }
    pub fn finish(
        &mut self,
        x: &[DVec3],
        p: &mut [Particle],
        edges: &[DistanceConstraint],
        hinges: &[DihedralConstraint],
        iterations: u32,
    ) -> Result<(), XpbdError> {
        if self.hard_distance.is_empty() && self.hard_bending.is_empty() {
            return Ok(());
        }
        // The RATTLE velocity impulse acts at the endpoint. Its endpoint
        // gradient has zero total torque about those same particle positions.
        for &index in &self.hard_distance {
            let c = edges[index];
            self.distance[index] = (x[c.a] - x[c.b])
                .try_normalize()
                .unwrap_or(self.distance[index]);
        }
        for &index in &self.hard_bending {
            let c = &hinges[index];
            self.bending[index] = bending::geometry(c.vertices.map(|i| x[i]))
                .ok_or(XpbdError::Numerical)?
                .1;
        }
        for _ in 0..iterations {
            let mut changed = false;
            for &index in &self.hard_distance {
                let c = edges[index];
                let n = self.distance[index];
                changed |= project_velocity(p, [c.a, c.b], [n, -n])?;
            }
            for &index in &self.hard_bending {
                changed |= project_velocity(p, hinges[index].vertices, self.bending[index])?;
            }
            if !changed {
                break;
            }
        }
        Ok(())
    }
}

fn project_velocity<const N: usize>(
    p: &mut [Particle],
    indices: [usize; N],
    gradient: [DVec3; N],
) -> Result<bool, XpbdError> {
    let mut rate = 0.;
    let mut inverse = 0.;
    let mut scale = 0.;
    // Relative velocities remove the Galilean frame from the stopping criterion.
    let base = DVec3::from(p[indices[0]].velocity);
    for (&i, g) in indices.iter().zip(gradient) {
        let v = DVec3::from(p[i].velocity) - base;
        rate += g.dot(v);
        inverse += p[i].inverse_mass * g.length_squared();
        scale += g.length() * v.length();
    }
    if !rate.is_finite() || !inverse.is_finite() {
        return Err(XpbdError::Numerical);
    }
    if inverse == 0. || rate.abs() <= 32. * f64::EPSILON * scale {
        return Ok(false);
    }
    let impulse = -rate / inverse;
    for (i, g) in indices.into_iter().zip(gradient) {
        p[i].velocity = (DVec3::from(p[i].velocity) + g * (impulse * p[i].inverse_mass)).to_array();
    }
    Ok(true)
}
