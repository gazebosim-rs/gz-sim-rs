// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Frozen point mobility and signed impulses on one native passive articulated tree.
use super::*;
pub(super) const MAX_SITES: usize = gz_physics_multifidelity::MAX_ARTICULATED_CONTACT_SITES;
type V = [f64; 3];
type K = [[f64; 3]; 3];
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArticulatedPointSite {
    /// Stable public tree link identity.
    pub link: u64,
    /// Metres from the link-frame origin, expressed in that frame.
    pub local_point: V,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContactPointBasis {
    pub world_points: Vec<V>,
    pub velocities: Vec<V>,
    pub dofs: usize,
    pub links: usize,
    blocks: Vec<K>,
}
impl ContactPointBasis {
    /// World XYZ velocity at target per world XYZ impulse at source, in kg⁻¹.
    pub fn mobility(&self, target: usize, source: usize) -> Option<K> {
        let n = self.world_points.len();
        if target >= n || source >= n {
            return None;
        }
        self.blocks.get(target * n + source).copied()
    }
    pub(super) fn from_native(s: &gz_physics_multifidelity::ArticulatedContactSample) -> Self {
        let n = s.world_points.len();
        Self {
            world_points: s.world_points.clone(),
            velocities: s.velocities.clone(),
            dofs: s.dofs,
            links: s.links,
            blocks: (0..n)
                .flat_map(|t| {
                    (0..n).map(move |u| s.mobility(t, u).expect("native square mobility"))
                })
                .collect(),
        }
    }
    pub(super) fn check(&self, n: usize, links: usize) -> Result<()> {
        self.check_dimensions(n, links, links + 5)
    }
    pub(super) fn check_dimensions(&self, n: usize, links: usize, dofs: usize) -> Result<()> {
        if !(1..=MAX_SITES).contains(&n)
            || self.links != links
            || self.dofs != dofs
            || self.world_points.len() != n
            || self.velocities.len() != n
            || self.blocks.len() != n * n
            || !self
                .world_points
                .iter()
                .chain(&self.velocities)
                .flatten()
                .chain(self.blocks.iter().flatten().flatten())
                .all(|v| v.is_finite())
        {
            return Err(invalid(
                "invalid dimensions or nonfinite articulated mobility",
            ));
        }
        Ok(())
    }
    pub(super) fn seal(
        &self,
        digest: [u8; 32],
        generation: u64,
        tick: u64,
        sites: &[ArticulatedPointSite],
    ) -> [u8; 32] {
        let mut h = Sha256::new();
        h.update(b"Gazebo contact sample v1");
        bincode::serialize_into(&mut h, &(digest, generation, tick, sites, self))
            .expect("bounded mobility hashes to SHA-256");
        h.finalize().into()
    }
}
/// Only the checked native state, original ordered sites and active lease may use this sample.
#[derive(Clone, Debug)]
pub struct ContactPointCertificate {
    pub(super) epoch: u64,
    pub(super) tick: u64,
    pub(super) generation: u64,
    pub(super) source: [u8; 32],
    pub(super) session: uuid::Uuid,
    pub(super) seal: [u8; 32],
    pub(super) sites: Vec<ArticulatedPointSite>,
    pub(super) basis: ContactPointBasis,
}
impl ContactPointCertificate {
    pub fn epoch(&self) -> u64 {
        self.epoch
    }
    pub fn tick(&self) -> u64 {
        self.tick
    }
    pub fn generation(&self) -> u64 {
        self.generation
    }
    pub fn sites(&self) -> &[ArticulatedPointSite] {
        &self.sites
    }
    pub fn basis(&self) -> &ContactPointBasis {
        &self.basis
    }
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ContactImpulseReceipt {
    /// Native point velocities after the instantaneous response and before time integration.
    pub after_velocities: Vec<V>,
    /// Native spatial kinetic energy change caused by the point impulses, in joules.
    pub kinetic_energy_change: f64,
}
