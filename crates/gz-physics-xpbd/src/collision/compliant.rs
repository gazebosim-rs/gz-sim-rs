// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Elastic particle contacts against prescribed rigid pads.
use super::*;

/// Particle skin may compress up to the rigid core. Refuse unresolved core
/// crossings and pass-throughs instead of selecting a far-side contact normal.
pub(super) fn projection(sweep: &Sweep) -> Result<Option<Projection>, XpbdError> {
    let (start, _, _) = sweep.geometry.witness(sweep.q0.conjugate() * sweep.start);
    let (end, normal, _) = sweep.geometry.witness(sweep.q1.conjugate() * sweep.end);
    let travel =
        sweep.start.distance(sweep.end) + sweep.angular_travel().length() * sweep.geometry.bound();
    if !travel.is_finite() || !start.is_finite() || !end.is_finite() {
        return Err(XpbdError::Numerical);
    }
    if start.min(end) < 0.
        || (start <= travel
            && (Sweep {
                radius: 0.,
                ..*sweep
            })
            .cast()?
            .is_some())
    {
        return Err(XpbdError::UnresolvedSweep);
    }
    if start >= sweep.radius && end >= sweep.radius && start - travel <= sweep.radius {
        let tolerance = 1e-10 * sweep.geometry.extent().max(sweep.radius);
        let inner = Sweep {
            radius: (sweep.radius - tolerance).max(0.),
            ..*sweep
        };
        if inner.cast()?.is_some() {
            return Err(XpbdError::UnresolvedSweep);
        }
    }
    Ok(Some(Projection {
        normal: sweep.q1 * normal,
        depth: sweep.radius - end,
        initial_overlap: (sweep.radius - start).max(0.),
        event: None,
    }))
}

impl Contacts {
    pub(super) fn project_compliant(
        &mut self,
        (i, particle): (usize, &mut Particle),
        x: &mut DVec3,
        boundaries: &[PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
        time: Interval,
        active: &mut Vec<(usize, Projection, f64)>,
    ) -> Result<(), XpbdError> {
        let start = DVec3::from(particle.position);
        active.clear();
        for (j, b) in boundaries
            .iter()
            .enumerate()
            .filter(|(_, b)| b.normal_compliance > 0.)
        {
            let Some(hit) = b.projection(start, *x, time, false)? else {
                continue;
            };
            let old = self
                .index
                .get(&(i, j))
                .map_or(0., |&k| self.items[k].projected_impulse);
            if hit.depth > 0. || old > 0. {
                active.push((j, hit, old));
            }
        }
        // Jacobi updates use the same geometric trial for opposing pads. The
        // number of active rows bounds their shared translational response.
        let inverse = active.len() as f64 * particle.inverse_mass;
        for &(j, ref hit, old) in active.iter() {
            let b = &boundaries[j];
            let compliance = b.normal_compliance;
            let impulse = ((time.h * hit.depth - compliance * old)
                / (inverse * time.h * time.h + compliance))
                .max(-old);
            let applied = impulse * hit.normal;
            correct(
                x,
                particle,
                time.h * particle.inverse_mass * applied,
                time.h,
            );
            reactions[j].react(start - b.origin, applied)?;
            if let Some(loads) = &mut self.pad_loads {
                loads.react(j, i, applied, DVec3::ZERO)?;
            }
            self.record(i, j, hit, impulse, None);
        }
        if !active.is_empty() && self.friction > 0. {
            self.stick_compliant((i, particle), x, boundaries, reactions, time, active)?;
        }
        Ok(())
    }
    /// Project static slip in the moving pad frame. Each pad sees the same
    /// trial position. Opposing jaws share the sticking correction in proportion
    /// to their normal loads, subject to each contact's own Coulomb limit.
    fn stick_compliant(
        &mut self,
        (i, particle): (usize, &mut Particle),
        x: &mut DVec3,
        boundaries: &[PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
        time: Interval,
        active: &[(usize, Projection, f64)],
    ) -> Result<(), XpbdError> {
        let trial = *x;
        let start = DVec3::from(particle.position);
        let squeeze: f64 = active
            .iter()
            .map(|(j, _, _)| self.items[self.index[&(i, *j)]].projected_impulse.max(0.))
            .sum();
        for (j, _, _) in active {
            let j = *j;
            let boundary = &boundaries[j];
            let index = self.index[&(i, j)];
            let contact = &mut self.items[index];
            let load = contact.projected_impulse.max(0.);
            let mut impulse = DVec3::ZERO;
            if load > 0. {
                let (q0, q1) = boundary
                    .receiver
                    .as_ref()
                    .and_then(|r| r.orientations())
                    .ok_or(XpbdError::InvalidBoundary)?;
                let (c0, c1) = boundary.centers(time);
                let target = c1 + q1 * (q0.conjugate() * (start - c0));
                let delta =
                    (target - trial) * ((load / squeeze) / (particle.inverse_mass * time.h));
                let accumulated = contact.tangent_impulse + delta;
                let tangent = accumulated - contact.normal * accumulated.dot(contact.normal);
                impulse = tangent.clamp_length_max(self.friction * load);
            }
            let applied = impulse - contact.tangent_impulse;
            contact.tangent_impulse = impulse;
            correct(
                x,
                particle,
                time.h * particle.inverse_mass * applied,
                time.h,
            );
            reactions[j].react(start - boundary.origin, applied)?;
            if let Some(loads) = &mut self.pad_loads {
                loads.react(j, i, DVec3::ZERO, applied)?;
            }
        }
        Ok(())
    }
}
