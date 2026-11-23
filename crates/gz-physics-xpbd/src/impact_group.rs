// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Event-time frictionless contact for an oriented finite receiver and an unloaded cable.
use crate::{
    BoundaryImpulse, SoftBody, XpbdError,
    collision::{Interval, PreparedBoundary},
};
use glam::DVec3;

type Point = (usize, usize);
const MAX_CONTACTS: usize = 4;
const MAX_HARD_ROWS: usize = 12;

pub(super) struct Event {
    pub(super) head_ns: u128,
    points: Vec<(Point, DVec3)>,
}

// The trial uses only copies of rigid motion and ballistic point predictions.
// The head is advanced without contact only after proving that the cable is
// unloaded, all earlier paths are clear and no non-transient contact is present.
pub(super) fn locate(
    bodies: &[&mut SoftBody],
    boundaries: &[PreparedBoundary],
    elapsed: u128,
    ns: u128,
) -> Result<Option<Event>, XpbdError> {
    let [boundary] = boundaries else {
        return Ok(None);
    };
    if boundary.material_capsule() && bodies.iter().any(|b| b.uses_capsule_material_contact()) {
        return Ok(None);
    }
    let Some(receiver) = &boundary.receiver else {
        return Ok(None);
    };
    let isolated_capsule = matches!(
        boundary.oriented_shape(),
        Some(crate::oriented::OrientedShape::Capsule(_))
    ) && receiver.isolated_particle;
    if !boundary.is_oriented()
        || !receiver.can_rotate()
        || (receiver.isolated_particle && !isolated_capsule)
    {
        return Ok(None);
    }
    if ns < 2
        || bodies.iter().any(|b| {
            b.config.friction != 0.
                || b.config.gravity != [0.; 3]
                || !b.bending.is_empty()
                || b.config.self_contact
        })
    {
        return Ok(None);
    }
    let h = ns as f64 * 1e-9;
    let t0 = elapsed as f64 * 1e-9;
    let t1 = (elapsed + ns) as f64 * 1e-9;
    let mut trial = boundary.clone();
    let receiver = trial.receiver.as_mut().unwrap();
    receiver.begin(h, t1, true)?;
    let (q0, q1) = receiver.orientations().ok_or(XpbdError::InvalidBoundary)?;
    if q0 == q1 || q0 == -q1 {
        return Ok(None);
    }
    let mut events = Vec::new();
    let mut transient = false;
    let mut ordinary = false;
    for (bi, body) in bodies.iter().enumerate() {
        for (pi, p) in body
            .scratch
            .iter()
            .enumerate()
            .filter(|(_, p)| p.inverse_mass > 0.)
        {
            let start = DVec3::from(p.position);
            let velocity = DVec3::from(p.velocity) + h * DVec3::from(body.config.gravity);
            let interval = Interval {
                t0,
                t1,
                h,
                radius: body.config.particle_radius,
            };
            if let Some(hit) = trial.projection(start, start + h * velocity, interval, true)? {
                if let Some(event) = hit.event {
                    transient |= event.transient;
                    // Retain the earliest four entries plus one overflow witness.
                    // A dense cloud cannot allocate a correspondingly large manifold.
                    let entry = (event.time, ((bi, pi), hit.normal));
                    let index = events.partition_point(|(t, _)| *t <= event.time);
                    if index <= MAX_CONTACTS {
                        events.insert(index, entry);
                        events.truncate(MAX_CONTACTS + 1);
                    }
                } else {
                    ordinary = true;
                }
            }
        }
    }
    if events.is_empty() {
        return Ok(None);
    }
    if ordinary
        || bodies.iter().any(|b| {
            b.scratch.iter().any(|p| p.inverse_mass <= 0.)
                || b.constraints.iter().any(|c| {
                    let (a, b) = (&b.scratch[c.a], &b.scratch[c.b]);
                    let scale = c.rest_length;
                    let length = DVec3::from(a.position).distance(DVec3::from(b.position));
                    let relative = DVec3::from(a.velocity).distance(DVec3::from(b.velocity));
                    (length - scale).abs() > 1e-10 * scale || h * relative > 1e-10 * scale
                })
        })
    {
        return Ok(None);
    }
    let time = events[0].0 * ns as f64;
    // Stay on the separated side of the rounded feature. Starting the
    // remaining interval at an overlapping surface can spuriously transport
    // that surface's tangent plane after a brief impact has already left.
    let head_ns = (time.floor() as u128).saturating_sub(1);
    let points: Vec<_> = events
        .into_iter()
        .take_while(|(t, _)| (t * ns as f64 - time).abs() <= 0.5)
        .map(|(_, p)| p)
        .collect();
    // A point can remain supported by the barrel after first impact. Resolving
    // that impulse at its clock and continuing the coupled motion captures the
    // subsequent contact trajectory. An entry within one nanosecond is already
    // handled by the ordinary contact projection.
    if isolated_capsule && head_ns <= 1 {
        return Ok(None);
    }
    if !isolated_capsule && !transient && points.len() < 2 {
        return Ok(None);
    }
    if !transient && (head_ns == 0 || points.len() > MAX_CONTACTS) {
        return Ok(None);
    }
    if head_ns == 0 || head_ns >= ns || points.len() > MAX_CONTACTS {
        return Err(XpbdError::UnresolvedSweep);
    }
    Ok(Some(Event { head_ns, points }))
}

#[derive(Clone, Copy)]
struct Term {
    point: Point,
    axis: DVec3,
}

struct Row {
    terms: Vec<Term>,
    translation: DVec3,
    rotation: DVec3,
    speed: f64,
    // Canonical geometric order makes the linear solve independent of which
    // body, particle or distance row a caller happened to register first.
    key: [DVec3; 2],
}

fn order(a: DVec3, b: DVec3) -> std::cmp::Ordering {
    a.x.total_cmp(&b.x)
        .then_with(|| a.y.total_cmp(&b.y))
        .then_with(|| a.z.total_cmp(&b.z))
}
fn sort_rows(rows: &mut [Row]) {
    rows.sort_by(|a, b| order(a.key[0], b.key[0]).then_with(|| order(a.key[1], b.key[1])));
}

pub(super) fn resolve(
    bodies: &mut [&mut SoftBody],
    boundary: &mut PreparedBoundary,
    reactions: &mut [Vec<BoundaryImpulse>],
    event: &Event,
) -> Result<(), XpbdError> {
    let geometry = boundary
        .oriented_shape()
        .ok_or(XpbdError::InvalidBoundary)?;
    let r = boundary
        .receiver
        .as_mut()
        .ok_or(XpbdError::InvalidBoundary)?;
    let q = r.orientations().ok_or(XpbdError::InvalidBoundary)?.1;
    let center = boundary.origin + boundary.velocity * r.end_time + r.end_offset;
    let linear = boundary.velocity + r.linear;
    let omega = r.angular_velocity();
    let mut rows = Vec::new();
    for &((bi, pi), normal) in &event.points {
        let p = &bodies[bi].scratch[pi];
        let pos = DVec3::from(p.position);
        let arm = pos - center;
        let (signed, local, _) = geometry.witness(q.conjugate() * arm);
        let radius = bodies[bi].config.particle_radius;
        let sampled_normal = q * local;
        let relative = DVec3::from(p.velocity) - linear - omega.cross(arm);
        // The separated clock boundary lies at most two nanoseconds before impact.
        let tolerance = 1e-9 * (1. + geometry.bound() + radius)
            + 2.1e-9 * (relative.length() + omega.length() * radius)
            + 64. * f64::EPSILON * pos.abs().max_element();
        if (signed - radius).abs() > tolerance
            || sampled_normal.distance(normal) * radius > 3. * tolerance
            || relative.dot(normal) >= 0.
        {
            return Err(XpbdError::UnresolvedSweep);
        }
        rows.push(Row {
            terms: vec![Term {
                point: (bi, pi),
                axis: normal,
            }],
            translation: -normal,
            rotation: -arm.cross(normal),
            speed: relative.dot(normal),
            key: [pos, normal],
        });
    }
    sort_rows(&mut rows);
    let contacts = rows.len();
    for (bi, body) in bodies.iter().enumerate() {
        if !event.points.iter().any(|&((i, _), _)| i == bi) {
            continue;
        }
        // A finite-compliance distance has a finite force and no instantaneous
        // impulse. An ideal bilateral distance enforces equal axial speeds.
        for c in body.constraints.iter().filter(|c| c.compliance == 0.) {
            if rows.len() - contacts >= MAX_HARD_ROWS {
                return Err(XpbdError::UnresolvedSweep);
            }
            let (mut a, mut b) = (c.a, c.b);
            if order(
                body.scratch[a].position.into(),
                body.scratch[b].position.into(),
            )
            .is_gt()
            {
                std::mem::swap(&mut a, &mut b);
            }
            let (pa, pb) = (&body.scratch[a], &body.scratch[b]);
            let key = [DVec3::from(pa.position), DVec3::from(pb.position)];
            let axis = (key[0] - key[1])
                .try_normalize()
                .ok_or(XpbdError::UnresolvedSweep)?;
            rows.push(Row {
                terms: vec![
                    Term {
                        point: (bi, a),
                        axis,
                    },
                    Term {
                        point: (bi, b),
                        axis: -axis,
                    },
                ],
                translation: DVec3::ZERO,
                rotation: DVec3::ZERO,
                speed: (DVec3::from(pa.velocity) - DVec3::from(pb.velocity)).dot(axis),
                key,
            });
        }
    }
    sort_rows(&mut rows[contacts..]);
    let n = rows.len();
    let mut matrix = vec![vec![0.; n]; n];
    for i in 0..n {
        for j in 0..=i {
            let (a, b) = (&rows[i], &rows[j]);
            let mut w = r.inverse_mass * a.translation.dot(b.translation)
                + a.rotation.dot(r.inverse_inertia(b.rotation));
            for t in &a.terms {
                for u in &b.terms {
                    if t.point == u.point {
                        w += bodies[t.point.0].scratch[t.point.1].inverse_mass * t.axis.dot(u.axis);
                    }
                }
            }
            if !w.is_finite() {
                return Err(XpbdError::Numerical);
            }
            matrix[i][j] = w;
            matrix[j][i] = w;
        }
    }
    let velocities: Vec<_> = rows.iter().map(|row| row.speed).collect();
    let impulses = inelastic(&matrix, &velocities, contacts).ok_or(XpbdError::UnresolvedSweep)?;
    for (i, (row, j)) in rows.iter().zip(impulses).enumerate() {
        for term in &row.terms {
            let (bi, pi) = term.point;
            let p = &mut bodies[bi].scratch[pi];
            let impulse = term.axis * j;
            let v = DVec3::from(p.velocity) + p.inverse_mass * impulse;
            if !v.is_finite() {
                return Err(XpbdError::Numerical);
            }
            p.velocity = v.to_array();
            if i < contacts {
                let x = DVec3::from(p.position);
                r.apply_velocity_impulse(x - center, -impulse);
                reactions[bi][0].react(x - boundary.origin, impulse)?;
            }
        }
    }
    Ok(())
}

// Solve unilateral normal complementarity with bilateral cable rows using
// an exact active-set search. A maximum of four normal rows bounds it at
// 16 small symmetric solves. Fail closed on singular/inconsistent systems.
pub(super) fn inelastic(k: &[Vec<f64>], v: &[f64], contacts: usize) -> Option<Vec<f64>> {
    let scale: Vec<_> = (0..v.len()).map(|i| k[i][i].sqrt()).collect();
    if scale.iter().any(|&s| !s.is_finite() || s <= 0.) {
        return None;
    }
    for mask in (0..1usize << contacts).rev() {
        let active: Vec<_> = (0..v.len())
            .filter(|&i| i >= contacts || mask & (1 << i) != 0)
            .collect();
        let n = active.len();
        let mut l = vec![vec![0.; n]; n];
        let mut good = true;
        for (a, &i) in active.iter().enumerate() {
            for (b, &j) in active[..=a].iter().enumerate() {
                let z =
                    k[i][j] / scale[i] / scale[j] - (0..b).map(|c| l[a][c] * l[b][c]).sum::<f64>();
                if a == b {
                    if !z.is_finite() || z < 1e-12 {
                        good = false;
                        break;
                    }
                    l[a][a] = z.sqrt();
                } else {
                    l[a][b] = z / l[b][b];
                }
            }
            if !good {
                break;
            }
        }
        if !good {
            continue;
        }
        let mut x = vec![0.; n];
        for (a, &i) in active.iter().enumerate() {
            x[a] = (-v[i] / scale[i] - (0..a).map(|b| l[a][b] * x[b]).sum::<f64>()) / l[a][a];
        }
        for a in (0..n).rev() {
            x[a] = (x[a] - (a + 1..n).map(|b| l[b][a] * x[b]).sum::<f64>()) / l[a][a];
        }
        let mut impulses = vec![0.; v.len()];
        for (&i, y) in active.iter().zip(x) {
            impulses[i] = y / scale[i];
        }
        if impulses.iter().any(|x| !x.is_finite()) || impulses[..contacts].iter().any(|&j| j < 0.) {
            continue;
        }
        if (0..v.len()).all(|i| {
            let mut gap = v[i];
            let mut mag = v[i].abs();
            for (a, j) in k[i].iter().zip(&impulses) {
                gap += a * j;
                mag += (a * j).abs();
            }
            let tolerance = 1e-11 * mag.max(1e-12);
            gap.is_finite()
                && if i >= contacts || mask & (1 << i) != 0 {
                    gap.abs() <= tolerance
                } else {
                    gap >= -tolerance
                }
        }) {
            return Some(impulses);
        }
    }
    None
}

/// Unequal particle impulses can start exciting internal cable forces after
/// impact. Resolve the remainder on a local spatial scale before returning to
/// the requested substep clock. Uniformly translating cables need no refinement.
pub(super) fn remaining_step(
    bodies: &[&mut SoftBody],
    boundary: &PreparedBoundary,
    ns: u128,
) -> Result<u128, XpbdError> {
    let h = ns as f64 * 1e-9;
    let single_capsule = matches!(
        boundary.oriented_shape(),
        Some(crate::oriented::OrientedShape::Capsule(_))
    ) && boundary
        .receiver
        .as_ref()
        .is_some_and(|r| r.isolated_particle);
    if !single_capsule
        && !bodies.iter().any(|body| {
            body.constraints.iter().any(|c| {
                let speed = DVec3::from(body.scratch[c.a].velocity)
                    .distance(body.scratch[c.b].velocity.into());
                speed * h > 1e-8 * body.config.particle_radius
            })
        })
    {
        return Ok(ns);
    }
    let r = boundary
        .receiver
        .as_ref()
        .ok_or(XpbdError::InvalidBoundary)?;
    let spin = r.angular_velocity().length()
        * boundary
            .oriented_shape()
            .ok_or(XpbdError::InvalidBoundary)?
            .bound();
    let center = boundary.velocity + r.linear;
    let mut slices = 1.;
    for b in bodies {
        let speed = b
            .scratch
            .iter()
            .map(|p| DVec3::from(p.velocity).distance(center))
            .fold(0., f64::max)
            + spin;
        let scale = match boundary.oriented_shape() {
            Some(crate::oriented::OrientedShape::Capsule(c)) => {
                b.config.particle_radius.max(0.1 * c.radius)
            }
            _ => b.config.particle_radius,
        };
        slices = f64::max(slices, (4. * speed * h / scale).ceil());
    }
    if !slices.is_finite() || slices > 512. || slices > ns as f64 {
        return Err(XpbdError::UnresolvedSweep);
    }
    Ok(ns.div_ceil(slices as u128))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_redundant_normal_opens_while_the_inextensible_row_stays_satisfied() {
        let k = vec![vec![1., 0.8, 0.1], vec![0.8, 1., 0.], vec![0.1, 0., 1.]];
        let before = [-1., -0.1, 0.2];
        let j = inelastic(&k, &before, 2).unwrap();
        assert!(j[0] > 0. && j[1] == 0. && j[2] < 0.);
        let after: Vec<_> = k
            .iter()
            .zip(before)
            .map(|(row, v)| row.iter().zip(&j).fold(v, |s, (k, j)| s + k * j))
            .collect();
        assert!(after[0].abs() < 1e-13 && after[1] > 0. && after[2].abs() < 1e-13);
    }
    #[test]
    fn dependent_bilateral_constraints_refuse_a_singular_event() {
        let k = vec![vec![1., 0., 0.], vec![0., 1., 1.], vec![0., 1., 1.]];
        assert!(inelastic(&k, &[-1., 0., 0.], 1).is_none());
    }
}
