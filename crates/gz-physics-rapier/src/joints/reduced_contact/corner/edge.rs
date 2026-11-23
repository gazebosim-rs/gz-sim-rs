// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two co-level vertices of a cold cuboid edge on the interior of a support.
use super::*;
use gz_physics::EdgeMobility;
mod bearing;
pub(crate) use bearing::{BearingModes, BearingResult};
pub use bearing::{
    DeckEdgeBearingReceipt, FaceBearingReceipt, FractionalBearingSpan, RockingLimitReceipt,
    SharedFaceReceipt, SharedSupportLoad, SharedSupportRequest, SoleTransferReceipt,
    TwoBearingReceipt,
};
mod response;
pub(super) use response::EdgeMode;
pub use response::{EdgeImpactReceipt, FrictionlessRecontactReceipt, OpeningEdgeImpactReceipt};
mod mixed;
pub use mixed::MixedEdgeImpactReceipt;

/// One joint-space commit at two distinct sites in a common contact frame.
/// The two signed site works sum to the assembled kinetic energy change.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SlidingEdgeImpactReceipt {
    pub sites: [CornerImpactReceipt; 2],
    pub kinetic_energy_change: f64,
    pub dissipated_energy: f64,
    pub contraction_bound: f64,
    pub iterate_error_bound: f64,
}
struct EdgeSites {
    boxes: RegisteredBoxes,
    sites: [ArticulatedContactSite; 2],
    world: [Vector; 2],
    vertices: [u8; 2],
    deck_velocity: [Vector; 2],
    gaps: [f64; 2],
    basis: [Vector; 3],
}
impl EdgeSites {
    fn incoming(
        &self,
        measured: &ArticulatedContactSample,
        velocity_increment: Vector,
    ) -> [Vector; 2] {
        std::array::from_fn(|i| {
            Vector::from_array(measured.velocities[i]) - self.deck_velocity[i] + velocity_increment
        })
    }

    fn mobility(
        measured: &ArticulatedContactSample,
        basis: [Vector; 3],
    ) -> Result<EdgeMobility, PhysicsError> {
        let mut matrix = [[[[0.; 3]; 3]; 2]; 2];
        for (i, row) in matrix.iter_mut().enumerate() {
            for (j, coeff) in row.iter_mut().enumerate() {
                let m = measured
                    .mobility(i, j)
                    .ok_or(PhysicsError::InvalidValue)?
                    .map(Vector::from_array);
                *coeff = std::array::from_fn(|k| {
                    std::array::from_fn(|l| {
                        basis[k].dot(Vector::from_array(m.map(|r| r.dot(basis[l]))))
                    })
                });
            }
        }
        Ok(matrix)
    }
}
impl RapierBackend {
    fn cold_edge_sites(
        &self,
        foot: BodyId,
        support: BodyId,
        displacement: Vector,
        allowance: f64,
    ) -> Result<Option<EdgeSites>, PhysicsError> {
        let Some(b) = self.registered_boxes(foot, support)? else {
            return Ok(None);
        };
        let h = b.sole_half;
        let p = b.pad_half;
        let scale = 1. + h.max_element() + p.max_element();
        let tol = 64.
            * f64::EPSILON
            * (scale
                + b.sole_pose.translation.abs().max_element()
                + b.pad_pose.translation.abs().max_element());
        let clear = (1e-8 * scale).max(32. * (tol + allowance));
        let mut found = [(0u8, Vector::ZERO, 0.); 2];
        let mut count = 0;
        for index in 0..8u8 {
            let sign = |bit: u8| if index & (1 << bit) == 0 { -1. } else { 1. };
            let world = b.sole_pose * Vector::new(sign(0) * h.x, sign(1) * h.y, sign(2) * h.z);
            let q = b.pad_pose.inverse_transform_point(world + displacement);
            let gap = q.z - p.z;
            if gap > clear {
                continue;
            }
            if gap.abs() > tol + allowance
                || q.x.abs() + clear >= p.x
                || q.y.abs() + clear >= p.y
                || count == 2
            {
                return Ok(None);
            }
            found[count] = (index, world, gap);
            count += 1;
        }
        if count != 2
            || (found[0].0 ^ found[1].0).count_ones() != 1
            || (found[0].2 - found[1].2).abs() > tol
        {
            return Ok(None);
        }
        let world = found.map(|p| p.1);
        let base = self.state(foot).ok_or(PhysicsError::MissingBody)?;
        let shoe = &self.world.bodies[b.foot];
        let deck = &self.world.bodies[b.pad];
        let sites = world.map(|p| ArticulatedContactSite {
            body: foot,
            local_point: (shoe.rotation().inverse() * (p - Vector::from_array(base.position)))
                .to_array(),
        });
        let deck_state = self.state(support).ok_or(PhysicsError::MissingBody)?;
        let deck_velocity = world.map(|p| {
            Vector::from_array(deck_state.velocity)
                + Vector::from_array(deck_state.angular_velocity).cross(p - deck.center_of_mass())
        });
        let basis = [Vector::X, Vector::Y, Vector::Z].map(|u| b.pad_pose.rotation * u);
        Ok(Some(EdgeSites {
            boxes: b,
            sites,
            world,
            deck_velocity,
            vertices: found.map(|r| r.0),
            gaps: found.map(|x| x.2),
            basis,
        }))
    }
}
