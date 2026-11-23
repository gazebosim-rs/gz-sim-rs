// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_physics_multifidelity::ArticulatedContactSite;
impl TreeWorld {
    pub(in crate::floating_tree) fn contact_spheres(
        &self,
        links: &[u64],
    ) -> Result<Vec<([f64; 3], f64)>> {
        if links.is_empty() || links.len() > crate::floating_tree::contact::MAX_SITES {
            return Err(invalid("invalid native sphere query count"));
        }
        let b = self.sim.world().resource::<PhysicsBackend>();
        let native = b
            .downcast_ref::<MultiFidelityBackend>()
            .expect("owned contact tree");
        links
            .iter()
            .map(|id| {
                let i = self
                    .links
                    .binary_search_by_key(id, |x| x.0)
                    .map_err(|_| invalid("unknown contact sphere"))?;
                Ok(native.articulated_sphere_geometry(self.links[i].1)?)
            })
            .collect()
    }

    fn point_sites(&self, sites: &[ArticulatedPointSite]) -> Result<Vec<ArticulatedContactSite>> {
        if sites.is_empty() || sites.len() > crate::floating_tree::contact::MAX_SITES {
            return Err(invalid("invalid native point count"));
        }
        sites
            .iter()
            .map(|s| {
                let i = self
                    .links
                    .binary_search_by_key(&s.link, |x| x.0)
                    .map_err(|_| invalid("unknown native contact link"))?;
                Ok(ArticulatedContactSite {
                    body: self.links[i].1,
                    local_point: s.local_point,
                })
            })
            .collect()
    }
    pub(in crate::floating_tree) fn point_basis(
        &mut self,
        sites: &[ArticulatedPointSite],
    ) -> Result<ContactPointBasis> {
        let native_sites = self.point_sites(sites)?;
        let mut b = self.sim.world_mut().resource_mut::<PhysicsBackend>();
        let native = b
            .downcast_mut::<MultiFidelityBackend>()
            .expect("owned native contact tree");
        let tx = native.prepare_articulated_contacts(&native_sites)?;
        Ok(ContactPointBasis::from_native(tx.sample()))
    }
    pub(in crate::floating_tree) fn commit_point_impulses(
        &mut self,
        sites: &[ArticulatedPointSite],
        impulses: &[[f64; 3]],
        seal: [u8; 32],
        source: [u8; 32],
        generation: u64,
        tick: u64,
    ) -> Result<ContactImpulseReceipt> {
        let native_sites = self.point_sites(sites)?;
        let mut b = self.sim.world_mut().resource_mut::<PhysicsBackend>();
        let native = b
            .downcast_mut::<MultiFidelityBackend>()
            .expect("owned native contact tree");
        let tx = native.prepare_articulated_contacts(&native_sites)?;
        let basis = ContactPointBasis::from_native(tx.sample());
        if basis.seal(source, generation, tick, sites) != seal {
            return Err(invalid(
                "native point mobility or physical source changed before commit",
            ));
        }
        let r = tx.commit(impulses)?;
        Ok(ContactImpulseReceipt {
            after_velocities: r.after_velocities,
            kinetic_energy_change: r.kinetic_energy_change,
        })
    }
}
