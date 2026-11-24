// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Fixed-orientation box features meeting the interior or boundary of a moving cloth triangle.
mod edges;
use super::{
    BoundaryImpulse, Particle, PreparedBoundary, XpbdError,
    collision::Interval,
    correct,
    material::Skin,
    surface::{first_entry, geometry::vertex_face},
    topology::cell_faces,
};
use glam::{DQuat, DVec3};
use std::{
    collections::BTreeMap,
    sync::{Arc, OnceLock},
};

#[derive(Clone)]
struct Row {
    ids: [usize; 3],
    weights: [f64; 3],
    normal: DVec3,
    witness: DVec3,
    other_wing: Option<usize>,
    lambda: f64,
}
impl Row {
    fn blend(&self, values: impl Fn(usize) -> DVec3) -> DVec3 {
        self.ids
            .iter()
            .zip(self.weights)
            .map(|(&i, w)| w * values(i))
            .sum()
    }
    fn mass_inverse(&self, p: &[Particle]) -> f64 {
        self.ids
            .iter()
            .zip(self.weights)
            .map(|(&i, w)| w * w * p[i].inverse_mass)
            .sum()
    }
}
#[derive(Default, Clone)]
pub(super) struct Faces {
    rotated_endpoints: Vec<DVec3>,
    edges: OnceLock<Arc<[edges::Edge]>>,
    rows: BTreeMap<(usize, usize, [usize; 3]), Row>,
}
struct Bounds {
    lo: DVec3,
    hi: DVec3,
}
impl Bounds {
    fn swept(points: impl Iterator<Item = (DVec3, DVec3)>, thickness: f64) -> Self {
        let mut lo = DVec3::splat(f64::INFINITY);
        let mut hi = DVec3::splat(f64::NEG_INFINITY);
        for (start, end) in points {
            lo = lo.min(start).min(end);
            hi = hi.max(start).max(end);
        }
        let pad = DVec3::splat(
            thickness + 128. * f64::EPSILON * lo.abs().max(hi.abs()).max_element().max(1.),
        );
        Self {
            lo: lo - pad,
            hi: hi + pad,
        }
    }
    fn contains(&self, corner: DVec3) -> bool {
        corner.cmpge(self.lo).all() && corner.cmple(self.hi).all()
    }
}

impl Faces {
    pub fn clear(&mut self) {
        self.rows.clear();
    }
    pub fn project(
        &mut self,
        columns: usize,
        x: &mut [DVec3],
        p: &mut [Particle],
        surfaces: &[PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
        dt: Interval,
    ) -> Result<(), XpbdError> {
        let mut boxes = surfaces
            .iter()
            .enumerate()
            .filter_map(|(i, b)| b.one_way_box(dt.t0).map(|(half, q)| (i, b, half, q)))
            .peekable();
        if boxes.peek().is_none() {
            return Ok(());
        }
        let mut queries = 0;
        let mut stationary_bounds = None;
        for (b, boundary, half, q) in boxes {
            // Cast the material relative to the box at interval start. Store the
            // end witness back in world space for position and angular reactions.
            let center = boundary.origin + dt.t0 * boundary.velocity;
            let offset = (dt.t1 - dt.t0) * boundary.velocity;
            let spin = boundary.box_spin();
            let rotating = spin != DVec3::ZERO;
            let rotation = DQuat::from_scaled_axis(dt.h * spin);
            // In the co-rotating frame each material vertex traces a curve.
            // Bound the chord error via |r′′| ≤ ω²|r| + 2ω|v|, then cast
            // against the padded skin. Project against the actual endpoint skin.
            let end_in_start_frame = &mut self.rotated_endpoints;
            end_in_start_frame.clear();
            let mut cast_radius = dt.radius;
            if rotating {
                // The rotating box stays inside this swept sphere. Distant
                // sheets need neither a feature cast nor an arc tolerance.
                let world = stationary_bounds.get_or_insert_with(|| {
                    Bounds::swept(
                        p.iter()
                            .zip(x.iter())
                            .map(|(p, &end)| (p.position.into(), end)),
                        dt.radius,
                    )
                });
                let end_center = center + offset;
                let extent = DVec3::splat(half.length());
                if (center.min(end_center) - extent).cmpgt(world.hi).any()
                    || (center.max(end_center) + extent).cmplt(world.lo).any()
                {
                    continue;
                }
                let angle = dt.h * spin.length();
                let mut max_error = 0_f64;
                for (p, &end) in p.iter().zip(x.iter()) {
                    let a = DVec3::from(p.position) - center;
                    let b = end - (center + offset);
                    let error = (angle * angle * a.length().max(b.length())
                        + 2. * angle * (b - a).length())
                        / 8.;
                    max_error = max_error.max(error);
                    end_in_start_frame.push(center + rotation.conjugate() * b);
                }
                if max_error > (0.001 * dt.radius).min(1e-6) {
                    return Err(XpbdError::UnresolvedSweep);
                }
                cast_radius += max_error;
            }
            let ends = if rotating {
                end_in_start_frame.as_slice()
            } else {
                &*x
            };
            let drift = if rotating { DVec3::ZERO } else { offset };
            let to_world = |mut row: Row| {
                if rotating {
                    row.normal = rotation * row.normal;
                    row.witness = (center + offset) + rotation * (row.witness - center);
                }
                row
            };
            let bounds = || {
                Bounds::swept(
                    p.iter()
                        .zip(ends.iter())
                        .map(|(p, &x)| (p.position.into(), x - drift)),
                    cast_radius,
                )
            };
            let moving_bounds;
            let all = if !rotating && offset == DVec3::ZERO {
                stationary_bounds.get_or_insert_with(bounds)
            } else {
                moving_bounds = bounds();
                &moving_bounds
            };
            for corner_index in 0..8 {
                let signs = DVec3::from_array(std::array::from_fn(|i| {
                    if corner_index & (1 << i) == 0 {
                        -1.
                    } else {
                        1.
                    }
                }));
                let corner = center + q * (half * signs);
                if !all.contains(corner) {
                    continue;
                }
                for end in (columns + 1)..p.len() {
                    if end % columns == 0 {
                        continue;
                    }
                    for ids in cell_faces(end, columns) {
                        let key = (b, corner_index, ids);
                        if self.rows.contains_key(&key) {
                            continue;
                        }
                        let a = ids.map(|i| DVec3::from(p[i].position));
                        let z = ids.map(|i| ends[i] - drift);
                        if !Bounds::swept(a.into_iter().zip(z), cast_radius).contains(corner) {
                            continue;
                        }
                        let eps = 1e-9 * dt.radius
                            + 128. * f64::EPSILON * corner.abs().max_element().max(1.);
                        let start = [corner, a[0], a[1], a[2]];
                        let at_start = vertex_face(&start);
                        let (signed, n) = if at_start.delta.length() <= eps {
                            let Some(mut n) = (a[1] - a[0]).cross(a[2] - a[0]).try_normalize()
                            else {
                                return Err(XpbdError::UnresolvedSurface);
                            };
                            if (corner - center).dot(n) < 0. {
                                n = -n;
                            }
                            (at_start.weights, n)
                        } else {
                            // A positive cast skin gives a well-defined incoming
                            // normal even for zero-thickness cloth crossing the box.
                            let Some(hit) = first_entry(
                                start,
                                [corner, z[0], z[1], z[2]],
                                true,
                                cast_radius.max(16. * eps),
                                eps,
                                &mut queries,
                            )?
                            else {
                                continue;
                            };
                            (hit.weights, -hit.normal)
                        };
                        let weights = [-signed[1], -signed[2], -signed[3]];
                        // Other faces of this convex box can lie behind the incident
                        // corner. Only normals in its outward normal cone support cloth.
                        if weights.iter().any(|&w| w <= 1e-9)
                            || ((q.conjugate() * n) * signs).min_element() < -1e-8
                        {
                            continue;
                        }
                        let row = Row {
                            ids,
                            weights,
                            normal: n,
                            witness: corner + drift,
                            other_wing: None,
                            lambda: 0.,
                        };
                        if row.mass_inverse(p) > 0. {
                            self.rows.insert(key, to_world(row));
                        }
                    }
                }
            }
            for ridge in edges::Ridge::all(center, half, q, drift) {
                if !ridge.overlaps(all) {
                    continue;
                }
                let edges = self.edges.get_or_init(|| edges::topology(p.len(), columns));
                for &edge in edges.iter() {
                    if let Some((feature, row)) =
                        ridge.sweep(edge, q.conjugate(), p, ends, cast_radius, &mut queries)?
                        && row.mass_inverse(p) > 0.
                    {
                        self.rows
                            .entry((b, feature, row.ids))
                            .or_insert_with(|| to_world(row));
                    }
                }
            }
        }
        for (&(b, _, _), row) in &mut self.rows {
            let point = row.blend(|i| x[i]);
            let depth = dt.radius - (point - row.witness).dot(row.normal);
            let next = (row.lambda + depth / row.mass_inverse(p)).max(0.);
            let delta = next - row.lambda;
            row.lambda = next;
            for (&i, w) in row.ids.iter().zip(row.weights) {
                let dx = row.normal * (delta * w * p[i].inverse_mass);
                correct(&mut x[i], &mut p[i], dx, dt.h);
            }
            // The coupled displacement and velocity correction changes angular
            // momentum about the material point at the start of this substep.
            let start = row.blend(|i| p[i].position.into());
            reactions[b].react(start - surfaces[b].origin, row.normal * (delta / dt.h))?;
        }
        Ok(())
    }
    pub fn finish(
        &self,
        x: &[DVec3],
        p: &mut [Particle],
        surfaces: &[PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
        cfg: super::SolverConfig,
        (h, t1): (f64, f64),
    ) -> Result<(), XpbdError> {
        let (mu, radius) = (cfg.friction, cfg.particle_radius);
        for (&(b, _, _), row) in &self.rows {
            let center = row.blend(|i| x[i]);
            let n = row.normal;
            if row.lambda == 0.
                && (center - row.witness).dot(n)
                    > radius + 1e-9 * radius + 128. * f64::EPSILON * center.length()
            {
                continue;
            }
            let surface_velocity = surfaces[b].velocity
                + surfaces[b]
                    .box_spin()
                    .cross(center - radius * n - (surfaces[b].origin + t1 * surfaces[b].velocity));
            let closing = (surface_velocity - row.blend(|i| p[i].velocity.into()))
                .dot(n)
                .max(0.);
            let normal_j = closing / row.mass_inverse(p);
            for (&i, w) in row.ids.iter().zip(row.weights) {
                p[i].velocity =
                    (DVec3::from(p[i].velocity) + w * p[i].inverse_mass * n * normal_j).to_array();
            }
            let ring = [
                row.ids[0],
                row.ids[1],
                row.ids[2],
                row.other_wing.unwrap_or(row.ids[2]),
            ];
            let patch = if row.other_wing.is_some() {
                &ring[..]
            } else {
                &ring[..3]
            };
            let skin = Skin::new(patch, row.ids, row.weights, |i| x[i], -radius * n)?;
            let relative = skin.velocity(p) - surface_velocity;
            let slip = relative - n * relative.dot(n);
            let mut tangent = DVec3::ZERO;
            if let Some(dir) = slip.try_normalize() {
                let inverse: f64 = skin
                    .nodal(dir)
                    .map(|(i, j)| p[i].inverse_mass * j.length_squared())
                    .sum();
                let cap = mu * (row.lambda / h + normal_j);
                if inverse > 0. {
                    tangent = -dir * (slip.length() / inverse).min(cap);
                    for (i, j) in skin.nodal(tangent) {
                        p[i].velocity =
                            (DVec3::from(p[i].velocity) + p[i].inverse_mass * j).to_array();
                    }
                }
            }
            reactions[b].react(
                center - radius * n - surfaces[b].origin,
                n * normal_j + tangent,
            )?;
        }
        Ok(())
    }
}
