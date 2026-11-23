// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Exact angular arc with certified chord envelopes for a finite capsule core.
use super::{
    Candidate, Particle, PreparedBoundary, XpbdError,
    bounds::Bounds,
    geometry::{Guide, Stencil},
};
use crate::{
    collision::Interval,
    oriented::OrientedShape,
    surface::{
        first_entry_mode,
        geometry::{edge_edge, vertex_face},
        material_sweep,
    },
};
use glam::{DQuat, DVec3};

#[derive(Clone, Copy)]
pub(super) struct Trajectory {
    pub linear: Guide,
    rotation: DVec3,
    h: f64,
    finite: bool,
}
impl Trajectory {
    pub fn new(b: &PreparedBoundary, dt: Interval) -> Result<Option<Self>, XpbdError> {
        if let Some(cap) = b.prescribed_capsule() {
            return Ok(Some(Self {
                linear: Guide {
                    center: b.origin + dt.t0 * b.velocity,
                    delta: dt.h * b.velocity,
                    velocity: b.velocity,
                    arm: cap.axis * cap.half_length,
                    gap: cap.radius + dt.radius,
                },
                rotation: DVec3::ZERO,
                h: dt.h,
                finite: false,
            }));
        }
        let Some(OrientedShape::Capsule(cap)) = b.oriented_shape() else {
            return Ok(None);
        };
        let r = b.receiver.as_ref().ok_or(XpbdError::InvalidBoundary)?;
        let (q0, q1) = r.orientations().ok_or(XpbdError::InvalidBoundary)?;
        let arm = q0 * (DVec3::Z * cap.half_length);
        let q = (q1 * q0.inverse()).normalize();
        let mut w = if q.w < 0. { -q } else { q }.to_scaled_axis();
        if w.cross(arm).length() <= 128. * f64::EPSILON * arm.length() * w.length() {
            w = DVec3::ZERO;
        }
        Ok(Some(Self {
            linear: Guide {
                center: b.origin + dt.t0 * b.velocity + r.start_offset,
                delta: dt.h * b.velocity + r.end_offset - r.start_offset,
                velocity: b.velocity + r.linear,
                arm,
                gap: cap.radius + dt.radius,
            },
            rotation: w,
            h: dt.h,
            finite: true,
        }))
    }
    pub fn bounds(self) -> Bounds {
        if self.rotation == DVec3::ZERO {
            return Bounds::swept_guide(self.linear);
        }
        // The core lies in a sphere of radius half_length about the moving center.
        Bounds::swept_guide(Guide {
            arm: DVec3::splat(self.linear.arm.length()),
            ..self.linear
        })
    }
    pub(super) fn arm(self, t: f64) -> DVec3 {
        DQuat::from_scaled_axis(t * self.rotation) * self.linear.arm
    }
    fn travel(self, s: Stencil, x: &[DVec3], p: &[Particle]) -> f64 {
        let relative = |i: usize| x[i] - DVec3::from(p[i].position) - self.linear.delta;
        let ids: &[usize] = match &s {
            Stencil::Edge(ids) => ids,
            Stencil::FaceCap(ids, _) => ids,
        };
        ids.iter().map(|&i| relative(i).length()).fold(0., f64::max)
            + self.rotation.cross(self.linear.arm).length()
    }
    pub fn candidates(
        self,
        s: Stencil,
        x: &[DVec3],
        p: &[Particle],
        queries: &mut usize,
    ) -> Result<[Option<Candidate>; 2], XpbdError> {
        let g = self.linear;
        let first = if self.rotation == DVec3::ZERO {
            s.candidate(x, p, g, queries)?
        } else {
            self.rotating(s, x, p, queries)?
        };
        let Some(hit) = first else {
            return Ok([None, None]);
        };
        // Keep the translational/rotational generalized impulse and evolving
        // lever arm local to a fraction of the contact thickness on impact.
        // Failure retries from the committed state at a finer shared clock.
        if self.finite && self.travel(s, x, p) > 0.20 * g.gap {
            return Err(XpbdError::UnresolvedSweep);
        }
        if self.rotation == DVec3::ZERO {
            Ok(s.manifold(hit, x, p, g))
        } else {
            Ok([Some(hit), None])
        }
    }
    pub(super) fn endpoint(
        &self,
        stencil: Stencil,
        x: &[DVec3],
        p: &[Particle],
    ) -> Result<Option<Candidate>, XpbdError> {
        let g = self.linear;
        let start = stencil.coords(|i| DVec3::from(p[i].position) - g.center, g.arm);
        let end = stencil.coords(|i| x[i] - (g.center + g.delta), self.arm(1.));
        let witness = match stencil {
            Stencil::Edge(_) => edge_edge,
            Stencil::FaceCap(_, _) => vertex_face,
        };
        let old = witness(&start).delta.length();
        let w = witness(&end);
        let d = w.delta.length();
        if !d.is_finite() {
            return Err(XpbdError::Numerical);
        }
        let Some(direction) = w.delta.try_normalize() else {
            return Ok(None);
        };
        let (feature, weights, part, normal) = stencil.describe(w.weights, direction);
        let eps = 1e-10 * g.gap + 128. * f64::EPSILON * (g.center.abs().max_element() + d);
        // This witness is only considered as an additional, opposing row of
        // an already detected impact. Its depth may be negative.
        let time = if old <= g.gap + eps { 0. } else { 1. };
        Ok(Some(Candidate {
            feature,
            weights,
            part,
            normal,
            time,
            depth: g.gap - d,
            overlap: (g.gap - old).max(0.),
            tolerance: eps,
        }))
    }
    fn rotating(
        self,
        s: Stencil,
        x: &[DVec3],
        p: &[Particle],
        queries: &mut usize,
    ) -> Result<Option<Candidate>, XpbdError> {
        let g = self.linear;
        let at = |t: f64| {
            s.coords(
                |i| DVec3::from(p[i].position).lerp(x[i], t) - (g.center + t * g.delta),
                self.arm(t),
            )
        };
        let face = matches!(s, Stencil::FaceCap(..));
        let witness = |a: &[DVec3; 4]| if face { vertex_face(a) } else { edge_edge(a) };
        let a0 = at(0.);
        let end = at(1.);
        let eps = 128.
            * f64::EPSILON
            * a0.iter()
                .chain(&end)
                .map(|p| p.length())
                .fold(g.gap, f64::max)
            + 1e-10 * g.gap;
        let angle = self.rotation.length();
        let orbit = self.rotation.cross(g.arm).length() / angle;
        // The maximum analytic arc-to-chord distance over a slab is
        // 2 r sin²(theta/4). Inflating the capsule offset by this amount
        // proves a negative chord cast is clear for the actual rotating core.
        let mut stack = vec![(0., 1.)];
        let mut examined = 0;
        while let Some((lo, hi)) = stack.pop() {
            examined += 1;
            if examined > 4096 {
                return Err(XpbdError::UnresolvedSweep);
            }
            let a = at(lo);
            let b = at(hi);
            let width = hi - lo;
            let sagitta = 2. * orbit * (angle * width * 0.25).sin().powi(2);
            let outer = g.gap + sagitta;
            if first_entry_mode(a, b, face, outer, eps, queries, true)?.is_none() {
                continue;
            }
            if sagitta > 2e-8 * g.gap {
                let mid = (lo + hi) * 0.5;
                stack.push((mid, hi));
                stack.push((lo, mid));
                continue;
            }
            let velocity = std::array::from_fn(|i| (b[i] - a[i]) / (width * self.h));
            let Some(hit) = material_sweep(a, b, velocity, face, outer, eps, queries)? else {
                continue;
            };
            let mut time = lo + width * hit.time;
            // Polish a transverse entry on the actual (unchorded) core. A
            // slab may begin inside the spatial envelope by a few roundoff
            // units, so bracket one slab on either side of the candidate.
            if time > 0. {
                let mut left = (time - width).max(0.);
                let mut right = (time + width).min(1.);
                let gap = |u| witness(&at(u)).delta.length() - g.gap;
                if gap(left) > 0. && gap(right) <= 0. {
                    for _ in 0..36 {
                        let mid = 0.5 * (left + right);
                        if gap(mid) > 0. {
                            left = mid;
                        } else {
                            right = mid;
                        }
                    }
                    *queries += 36;
                    if *queries > 65536 {
                        return Err(XpbdError::UnresolvedSweep);
                    }
                    time = right;
                }
            }

            let q = at(time);
            let w = witness(&q);
            let d = w.delta.length();
            let normal = w.delta.try_normalize().ok_or(XpbdError::UnresolvedSweep)?;
            let (feature, weights, part, incident) = s.describe(w.weights, normal);
            let axial = match s {
                Stencil::Edge(_) => -w.weights[3] + w.weights[2],
                Stencil::FaceCap(_, sign) => f64::from(sign),
            };
            let end_arm = self.arm(1.);
            let swing = DQuat::from_rotation_arc(self.arm(time).normalize(), end_arm.normalize());
            let n = swing * incident;
            let mut c = Candidate {
                feature,
                weights,
                part,
                normal: n,
                time,
                depth: 0.,
                overlap: (g.gap - witness(&a0).delta.length()).max(0.),
                tolerance: eps,
            };
            let sep = c.point(|i| x[i] - (g.center + g.delta)) - axial * end_arm;
            c.depth = (g.gap - sep.dot(n)).max(0.);
            // Do not apply an impulse to the outer approximation itself.
            // Its witness must be approaching or enter the true contact plane.
            let at_spin = self.rotation.cross(self.arm(time) * axial) / self.h;
            let closing = -(c.point(|i| DVec3::from(p[i].velocity)) - g.delta / self.h - at_spin)
                .dot(incident);
            if c.depth > eps || closing > eps / self.h && d <= g.gap + 2. * sagitta + 2. * eps {
                return Ok(Some(c));
            }
        }
        Ok(None)
    }
}

/// Certify the entire accepted curved path of both receivers for this
/// shared substep. The native pair bridge separately checks the native free
/// and commit paths. XPBD has no competing rigid-rigid contact owner.
pub(crate) fn check_receiver_pair(bs: &[PreparedBoundary], dt: Interval) -> Result<(), XpbdError> {
    let mut paths = bs.iter().filter(|b| {
        b.receiver.is_some() && matches!(b.oriented_shape(), Some(OrientedShape::Capsule(_)))
    });
    let (Some(a), Some(b), None) = (paths.next(), paths.next(), paths.next()) else {
        return Ok(());
    };
    let a = Trajectory::new(a, dt)?.unwrap();
    let b = Trajectory::new(b, dt)?.unwrap();
    let d0 = b.linear.center - a.linear.center;
    let dv = b.linear.delta - a.linear.delta;
    let minimum = a.linear.gap + b.linear.gap; // radius == 0 for the rigid centerline test
    let speed = dv.length()
        + a.rotation.cross(a.linear.arm).length()
        + b.rotation.cross(b.linear.arm).length();
    let eps = 128.
        * f64::EPSILON
        * (d0.length() + minimum + a.linear.arm.length() + b.linear.arm.length());
    let clearance = |t| {
        let arm = a.arm(t);
        let other = b.arm(t);
        let center = d0 + t * dv;
        edge_edge(&[-arm, arm, center - other, center + other])
            .delta
            .length()
            - minimum
    };
    if clearance(0.) <= eps || clearance(1.) <= eps {
        return Err(XpbdError::SimultaneousCapsules);
    }
    let mut stack = vec![(0., 1., 0)];
    let mut queries = 0;
    while let Some((low, high, depth)) = stack.pop() {
        let t = 0.5 * (low + high);
        let gap = clearance(t);
        if !gap.is_finite() {
            return Err(XpbdError::Numerical);
        }
        if gap <= eps {
            return Err(XpbdError::SimultaneousCapsules);
        }
        if gap > 0.5 * (high - low) * speed + eps {
            continue;
        }
        queries += 1;
        if depth == 28 || queries > 4096 {
            return Err(XpbdError::UnresolvedSweep);
        }
        stack.extend([(t, high, depth + 1), (low, t, depth + 1)]);
    }
    Ok(())
}

impl super::CapsuleMaterial {
    /// Check the actual end pose of the receiver against the whole material
    /// skin. An unresolved endpoint retries the public transaction before any
    /// particle, receiver correction or load is published.
    pub(crate) fn check_finite_endpoint(
        &self,
        x: &[DVec3],
        bs: &[PreparedBoundary],
        skin: f64,
    ) -> Result<(), XpbdError> {
        for b in bs {
            let Some(OrientedShape::Capsule(geometry)) = b.oriented_shape() else {
                continue;
            };
            let r = b.receiver.as_ref().ok_or(XpbdError::InvalidBoundary)?;
            let (_, q) = r.orientations().ok_or(XpbdError::InvalidBoundary)?;
            let center = b.origin + b.velocity * r.end_time + r.end_offset;
            let arm = q * (DVec3::Z * geometry.half_length);
            let gap = geometry.radius + skin;
            let tolerance = 1e-8 * (geometry.half_length + gap)
                + 128. * f64::EPSILON * center.abs().max_element();
            for s in &self.mesh.stencils {
                if let Stencil::FaceCap(ids, -1) = s
                    && super::geometry::pierces(ids.map(|i| x[i]), center, arm)
                {
                    return Err(XpbdError::UnresolvedSweep);
                }
                let p = s.coords(|i| x[i] - center, arm);
                let d = match s {
                    Stencil::Edge(_) => edge_edge(&p),
                    Stencil::FaceCap(_, _) => vertex_face(&p),
                }
                .delta
                .length();
                if !d.is_finite() {
                    return Err(XpbdError::Numerical);
                }
                if d < gap - tolerance {
                    return Err(XpbdError::UnresolvedSweep);
                }
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use parry3d_f64::{
        math::Pose,
        query::{self, NonlinearRigidMotion},
        shape::{Capsule, Segment, Shape, Triangle},
    };
    #[test]
    fn rotating_capsules_match_nonlinear_parry_and_deforming_feature_distances() {
        for shear in [false, true] {
            let (mut hits, mut misses, mut error) = (0, 0, 0f64);
            let mut cast_error = 0f64;
            for i in 0..180 {
                let t = i as f64;
                let turn = DQuat::from_rotation_z(0.42 * t)
                    * DQuat::from_rotation_y(0.16 * (t * 0.22).cos());
                let start = [
                    DVec3::new(-0.47, -0.29, 0.58),
                    DVec3::new(0.45, -0.01, 0.58),
                    DVec3::new(-0.16, 0.46, 0.58),
                ]
                .map(|v| turn * v);
                let linear = DVec3::new(0.12 * (t * 0.22).cos(), 0.08 * (t * 0.36).sin(), -1.25);
                let v = std::array::from_fn::<_, 3, _>(|j| {
                    let z = t + 1.97 * j as f64;
                    linear
                        + if shear {
                            DVec3::new(
                                0.2 * (0.31 * z).sin(),
                                0.13 * (0.72 * z).cos(),
                                0.19 * (0.58 * z).sin(),
                            )
                        } else {
                            DVec3::ZERO
                        }
                });
                let end = std::array::from_fn::<_, 3, _>(|j| start[j] + v[j]);
                let p = std::array::from_fn::<_, 3, _>(|j| Particle {
                    position: start[j].to_array(),
                    velocity: v[j].to_array(),
                    inverse_mass: 1.,
                });
                let center = DVec3::new(0.52 * (0.31 * t).cos(), 0.49 * (0.46 * t).sin(), 0.);
                let arm = 0.26
                    * DVec3::new((0.51 * t).cos(), 0.5 + (0.38 * t).sin(), (0.41 * t).cos())
                        .normalize();
                let rotation =
                    DVec3::new((0.62 * t).cos(), -0.5 + (0.57 * t).sin(), (0.71 * t).sin())
                        .normalize()
                        * (0.2 + 1.8 * (0.33 * t).sin().abs());
                let motion = DVec3::new(0.19 * (0.33 * t).cos(), -0.16 * (0.55 * t).sin(), 0.04);
                let g = Guide {
                    center,
                    arm,
                    delta: motion,
                    velocity: motion,
                    gap: 0.10,
                };
                let trial = Trajectory {
                    linear: g,
                    rotation,
                    h: 1.,
                    finite: true,
                };
                let face = i % 2 == 0;
                let mut stencils = vec![Stencil::Edge([0, 1])];
                if face {
                    stencils.extend([
                        Stencil::Edge([0, 2]),
                        Stencil::Edge([1, 2]),
                        Stencil::FaceCap([0, 1, 2], -1),
                        Stencil::FaceCap([0, 1, 2], 1),
                    ]);
                }
                let mut got: Option<f64> = None;
                let mut queries = 0;
                for s in stencils {
                    if let Some(c) = trial
                        .rotating(s, &end, &p, &mut queries)
                        .unwrap_or_else(|e| panic!("case {i} shear={shear}: {e}"))
                    {
                        got = Some(got.map_or(c.time, |t| t.min(c.time)));
                    }
                }
                let cast = if !shear {
                    let seg = Segment::new(start[0], start[1]);
                    let tri = Triangle::new(start[0], start[1], start[2]);
                    let shape: &dyn Shape = if face { &tri } else { &seg };
                    let rigid = NonlinearRigidMotion::new(
                        Pose::from_parts(center, DQuat::IDENTITY),
                        DVec3::ZERO,
                        motion,
                        rotation,
                    );
                    let soft =
                        NonlinearRigidMotion::new(Pose::IDENTITY, DVec3::ZERO, linear, DVec3::ZERO);
                    query::cast_shapes_nonlinear(
                        &rigid,
                        &Capsule::new(-arm, arm, g.gap),
                        &soft,
                        shape,
                        0.,
                        1.,
                        true,
                    )
                    .unwrap()
                    .map(|h| h.time_of_impact)
                } else {
                    None
                };
                let expected = {
                    let distance = |u: f64| {
                        let c = center + u * motion;
                        let a = DQuat::from_scaled_axis(u * rotation) * arm;
                        let core = Segment::new(c - a, c + a);
                        let pts = std::array::from_fn::<_, 3, _>(|j| start[j] + u * v[j]);
                        let seg = Segment::new(pts[0], pts[1]);
                        let tri = Triangle::new(pts[0], pts[1], pts[2]);
                        let shape: &dyn Shape = if face { &tri } else { &seg };
                        query::distance(&Pose::IDENTITY, shape, &Pose::IDENTITY, &core)
                            .unwrap()
                            .distance
                            - g.gap
                    };
                    let mut first = None;
                    for n in 1..=640 {
                        let mut hi = n as f64 / 640.;
                        if distance(hi) <= 0. {
                            let mut lo = (n - 1) as f64 / 640.;
                            for _ in 0..36 {
                                let mid = 0.5 * (hi + lo);
                                if distance(mid) > 0. {
                                    lo = mid;
                                } else {
                                    hi = mid;
                                }
                            }
                            first = Some(hi);
                            break;
                        }
                    }
                    first
                };
                if !shear {
                    match (cast, expected) {
                        (Some(a), Some(b)) => cast_error = cast_error.max((a - b).abs()),
                        (None, None) => {}
                        v => panic!(
                            "independent nonlinear cast vs dense Parry disagreement at {i}: {v:?}"
                        ),
                    }
                }
                match (got, expected) {
                    (Some(a), Some(b)) => {
                        hits += 1;
                        error = error.max((a - b).abs());
                    }
                    (None, None) => misses += 1,
                    v => panic!("case {i} shear={shear}: {v:?}"),
                }
            }
            println!(
                "180 angular sweeps shear={shear}: {hits} hits, {misses} misses, max normalized true-distance TOI error={error:e}, independent nonlinear cast vs distance={cast_error:e}"
            );
            assert!(hits > 35 && misses > 35 && error < 2e-6);
        }
    }
}
