// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Partial plantar support at the deck's +X edge and a separate rigid shank contact.
use super::*;

#[derive(Clone, Copy, Default, Debug)]
pub struct ShankContact {
    pub impulse: [f64; 3],
    pub angular_impulse: [f64; 3],
    pub normal_ns: f64,
    /// Largest signed geometric gap at the end of a substep carrying a positive impulse.
    pub max_loaded_gap_m: Option<f64>,
    pub peak_points: usize,
    pub first_loaded_offset_s: Option<f64>,
}
#[derive(Clone, Copy)]
pub(super) struct EdgeGeometry {
    side_clearance: f64,
    shank_gap: f64,
    core_height: f64,
    shank_face: f64,
    link_clearance: f64,
    least_outward_speed: f64,
    normal_alignment: f64,
}
impl EdgeGeometry {
    pub(super) fn validate_start(self) -> Result<(), PhysicsError> {
        if self.side_clearance <= 0.
            || self.core_height <= 0.
            || self.link_clearance <= 0.
            || self.normal_alignment <= 0.
        {
            return Err(unsupported(
                "toe exit requires clearance from both side rims, the rear wall and the rigid sole core",
            ));
        }
        Ok(())
    }
    pub(super) fn validate_sweep(
        self,
        end: Self,
        reach: f64,
        angular_travel: f64,
        edge: f64,
    ) -> Result<(), PhysicsError> {
        if !reach.is_finite()
            || self.side_clearance.min(end.side_clearance) <= reach
            || self.core_height.min(end.core_height) <= reach
            || self.link_clearance.min(end.link_clearance) <= 2. * reach
            || self.normal_alignment.min(end.normal_alignment) <= angular_travel
        {
            return Err(unsupported(
                "toe exit cannot certify rim, sole core or self-contact clearance",
            ));
        }
        if edge <= 0. && self.least_outward_speed.min(end.least_outward_speed) <= 0. {
            return Err(unsupported(
                "inward sole motion across a finite sidewall is unresolved",
            ));
        }
        Ok(())
    }
}
impl Shapes {
    pub(super) fn edge_geometry(self, solver: &RapierBackend, s: &Sample) -> EdgeGeometry {
        let deck = self.deck.orientation.inverse();
        let half = self.deck.half_extents;
        let foot = deck * (Vector::from(s.lo.position) - self.deck.center);
        let rotation = deck * Rotation::from_array(s.lo.orientation);
        let ext = |r: Rotation, h: Vector| {
            (r * Vector::X).abs() * h.x + (r * Vector::Y).abs() * h.y + (r * Vector::Z).abs() * h.z
        };
        let h = self.foot.half_extents;
        let shoe = ext(rotation, h);
        // The upper half of the foot is the rigid substrate; only the plantar
        // coating may penetrate a panel top when the toe extends past its rim.
        let core = ext(rotation, Vector::new(h.x, h.y, 0.));
        let shank = deck * (Vector::from(s.hi.position) - self.deck.center);
        let upper = ext(
            deck * Rotation::from_array(s.hi.orientation),
            self.shank_half,
        );
        let lower_col = &solver.world.colliders[self.rows.foot];
        let upper_col = &solver.world.colliders[self.shank_collider];
        // Both shapes are admitted cuboids. A failed exact-distance query fails closed.
        let link_clearance = rapier3d_f64::parry::query::distance(
            lower_col.position(),
            lower_col.shape(),
            upper_col.position(),
            upper_col.shape(),
        )
        .map_or(0., |d| d.distance);
        let spin = Vector::from(s.lo.angular_velocity);
        let min_speed = (deck * Vector::from(s.lo.velocity)).x - spin.length() * h.length();
        EdgeGeometry {
            side_clearance: (half.y - foot.y.abs() - shoe.y).min(half.x + foot.x - shoe.x),
            shank_gap: s.height,
            core_height: foot.z - core.z - half.z,
            shank_face: (half.x - shank.x.abs() - upper.x).min(half.y - shank.y.abs() - upper.y),
            link_clearance,
            least_outward_speed: min_speed,
            normal_alignment: (rotation * Vector::Z).z,
        }
    }

    pub(super) fn read_shank(
        self,
        solver: &RapierBackend,
        c: CoatedAnkle,
        [initial, end]: [EdgeGeometry; 2],
        reach: f64,
        elapsed: f64,
        out: &mut ShankContact,
    ) -> Result<(), PhysicsError> {
        if reach >= 2. * self.deck.half_extents.z.min(self.shank_half.min_element()) {
            return Err(unsupported(
                "shank sweep can traverse the finite support thickness",
            ));
        }
        if initial.shank_face.min(end.shank_face) <= reach {
            return Err(unsupported(
                "hard shank contact requires the interior of the finite top face",
            ));
        }
        let support = &solver.world.colliders[self.rows.pad];
        let upper = &solver.world.colliders[self.shank_collider];
        if upper.friction() != 0. || support.friction() != 0. {
            return Err(unsupported(
                "this shank strike requires explicitly frictionless rigid contact",
            ));
        }
        let n = self.deck.orientation * Vector::Z;
        let mut loaded = 0;
        for patch in solver.rigid_contact_patches() {
            if (patch.a, patch.b) != (c.shank.min(c.floor), c.shank.max(c.floor)) {
                continue;
            }
            let first = patch.a == c.shank;
            let normal = Vector::from(patch.normal_on_a) * (if first { 1. } else { -1. });
            for point in patch.points.into_iter().filter(|p| p.normal_impulse > 0.) {
                if normal.distance(n) > 1e-8 {
                    return Err(unsupported("non-top rigid shank response"));
                }
                let j = point.normal_impulse * normal;
                let arm = Vector::from(if first { point.lever_a } else { point.lever_b });
                out.impulse = (Vector::from(out.impulse) + j).into();
                out.angular_impulse = (Vector::from(out.angular_impulse) + arm.cross(j)).into();
                out.normal_ns += point.normal_impulse;
                out.first_loaded_offset_s.get_or_insert(elapsed);
                loaded += 1;
            }
        }
        if loaded > 0 {
            // Tiny roundoff is permitted; a solver-predicted row with both
            // endpoints physically separated cannot be counted as a hard strike.
            if initial.shank_gap.min(end.shank_gap) > 1e-9 {
                return Err(unsupported("rigid shank row precedes geometric contact"));
            }
            out.max_loaded_gap_m = Some(
                out.max_loaded_gap_m
                    .map_or(end.shank_gap, |g| g.max(end.shank_gap)),
            );
        }
        out.peak_points = out.peak_points.max(loaded);
        Ok(())
    }
}
