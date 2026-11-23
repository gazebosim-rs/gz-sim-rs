// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Keep the sphere pair's native contact graph resident while the floating hinge crosses owners.
use super::super::grounded::{graph_codec, validate_contacts};
use super::*;
const ISLAND_MAGIC: &[u8; 8] = b"GZASP001";

impl RapierBackend {
    fn sphere_cut_clear(&self, a: BodyId, b: BodyId) -> Result<bool, PhysicsError> {
        let collider = |id| {
            let rb = &self.world.bodies[self.bodies[&id].0];
            &self.world.colliders[rb.colliders()[0]]
        };
        let left = collider(a);
        let right = collider(b);
        let radius = |c: &Collider| {
            c.shape()
                .as_ball()
                .map(|v| v.radius)
                .ok_or_else(unsupported)
        };
        let margin = self.world.integration_parameters.prediction_distance()
            + radius(left)?
            + radius(right)?;
        let handle = |id| self.world.bodies[self.bodies[&id].0].colliders()[0];
        let held = self
            .world
            .contact_pair(handle(a), handle(b))
            .is_some_and(|pair| {
                pair.has_any_active_contact()
                    || pair
                        .manifolds()
                        .iter()
                        .any(|m| !m.points.is_empty() || !m.data.solver_contacts.is_empty())
            });
        Ok(!held
            && left
                .position()
                .translation
                .distance(right.position().translation)
                > margin)
    }

    fn set_island_members(&mut self, ids: [BodyId; 2], enabled: bool) {
        for id in ids {
            let handle = self.bodies[&id].0;
            let collider = self.world.bodies[handle].colliders()[0];
            self.world.bodies[handle].set_enabled(enabled);
            self.world.colliders[collider].set_enabled(enabled);
        }
    }
    /// Split a full graph into either half; only sphere–hinge edges have to drain.
    /// The pair retains its original broad phase, narrow phase and contact graph.
    pub fn extract_sphere_island(&mut self, keep_spheres: bool) -> Result<(), PhysicsError> {
        let obs = self.pair_observer.ok_or_else(unsupported)?;
        let layout = self.revolute_layout(Some(Companion::Pair(obs.spheres)))?;
        if !obs
            .spheres
            .into_iter()
            .all(|id| self.world.bodies[self.bodies[&id].0].is_enabled())
        {
            return Err(unsupported());
        }
        if !self.joints.open {
            for link in obs.links {
                for sphere in obs.spheres {
                    if !self.sphere_cut_clear(link, sphere)? {
                        return Err(unsupported());
                    }
                }
            }
        }
        if keep_spheres {
            self.set_island_members(obs.links, false);
            // The native removal journal is skipped by Rapier's serializer.
            // Keep the disabled tree until the first resident step, where its
            // removal event can be consumed before the physics solve.
            self.joints = Default::default();
            // No joint, pending drive or enabled hinge link can enter the pair's native step.
            self.joints.close();
            self.world.integration_parameters.dt = STEP.as_secs_f64();
            self.pair_observer.as_mut().unwrap().island_construction = Some(layout.construction);
        } else {
            self.set_island_members(obs.spheres, false);
            // The next full pipeline step drains both retired sphere edges and
            // persistent island membership when it consumes the disable flags.
        }
        Ok(())
    }
    pub(crate) fn apply_sphere_island_topology(&mut self) {
        if let Some(obs) = self.pair_observer
            && obs.island_construction.is_some()
            && self.world.multibody_joints.iter().next().is_some()
        {
            self.world
                .multibody_joints
                .remove_multibody_articulations(self.bodies[&obs.links[0]].0, false);
        }
    }

    pub fn sphere_island_drained(&self) -> Result<bool, PhysicsError> {
        let observer = self.pair_observer.ok_or_else(unsupported)?;
        if observer.island_construction.is_none() {
            return Err(unsupported());
        }
        self.sphere_cut_clear(observer.spheres[0], observer.spheres[1])
    }
    pub fn checkpoint_sphere_island(&self) -> Result<Vec<u8>, PhysicsError> {
        let o = self.pair_observer.ok_or_else(unsupported)?;
        let binding = o.island_construction.ok_or_else(unsupported)?;
        validate_contacts(&self.world)?;
        let data = graph_codec()
            .serialize(&(binding, o.initial_velocity, &self.world))
            .map_err(|_| invalid())?;
        let mut bytes = Vec::with_capacity(data.len() + 40);
        bytes.extend_from_slice(ISLAND_MAGIC);
        bytes.extend_from_slice(&Sha256::digest(&data));
        bytes.extend_from_slice(&data);
        Ok(bytes)
    }
    fn read_sphere_island(&self, bytes: &[u8]) -> Result<(PhysicsWorld, [f64; 3]), PhysicsError> {
        let o = self.pair_observer.ok_or_else(unsupported)?;
        let companion = Companion::Pair(o.spheres);
        let binding = match o.island_construction {
            Some(v) => v,
            None => self.revolute_layout(Some(companion))?.construction,
        };
        let (header, payload) = bytes.split_at_checked(40).ok_or_else(invalid)?;
        if bytes.len() > GROUNDED_REVOLUTE_LIMIT
            || &header[..8] != ISLAND_MAGIC
            || header[8..] != Sha256::digest(payload)[..]
        {
            return Err(invalid());
        }
        let (construction, vel, world): ([u8; 32], [f64; 3], PhysicsWorld) =
            graph_codec().deserialize(payload).map_err(|_| invalid())?;
        if construction != binding
            || !finite(vel)
            || world.gravity != Vector::ZERO
            || world.integration_parameters
                != (IntegrationParameters {
                    dt: STEP.as_secs_f64(),
                    ..self.world.integration_parameters
                })
            || world.bodies.len() != 4
            || world.colliders.len() != 4
            || !self.is_detached_or_pending_tree(&world, o)
            || !world.impulse_joints.is_empty()
            || !world.soft_bodies.is_empty()
        {
            return Err(invalid());
        }
        self.validate_companion_bodies(&world, companion, true)?;
        validate_contacts(&world)?;
        for link in o.links {
            let handle = world.bodies[self.bodies[&link].0].colliders()[0];
            if world.narrow_phase.contact_pairs_with(handle).any(|p| {
                p.has_any_active_contact()
                    || p.manifolds()
                        .iter()
                        .any(|m| !m.points.is_empty() || !m.data.solver_contacts.is_empty())
            }) {
                return Err(invalid());
            }
        }
        Ok((world, vel))
    }
    fn is_detached_or_pending_tree(&self, world: &PhysicsWorld, observer: PairObserver) -> bool {
        let mut records = world.multibody_joints.iter();
        if records.next().is_none() {
            return true;
        }
        let [root, child] = observer.links.map(|id| self.bodies[&id].0);
        records.next().is_none()
            && world
                .multibody_joints
                .get(MultibodyJointHandle(child.0))
                .is_some_and(|(tree, index)| {
                    index == 1
                        && tree.num_links() == 2
                        && tree.link(0).is_some_and(|l| l.rigid_body_handle() == root)
                        && tree.link(1).is_some_and(|l| l.rigid_body_handle() == child)
                })
    }

    /// Restore into a freshly configured pair template.
    pub fn restore_sphere_island(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        let o = self.pair_observer.ok_or_else(unsupported)?;
        if o.island_construction.is_some() || !self.joints.open {
            return Err(unsupported());
        }
        let binding = self
            .revolute_layout(Some(Companion::Pair(o.spheres)))?
            .construction;
        let (world, vel) = self.read_sphere_island(bytes)?;
        self.world = world;
        self.joints = Default::default();
        self.joints.close();
        let o = self.pair_observer.as_mut().unwrap();
        o.initial_velocity = vel;
        o.island_construction = Some(binding);
        self.refresh_max_boundary_spin();
        Ok(())
    }
    /// Continue the loaded A/B graph and insert the authoritative hinge tree at its stable handles.
    /// Keeping the pair as receiver preserves its entire manifold and solver ordering metadata.
    pub fn merge_sphere_island(&mut self, bytes: &[u8]) -> Result<(), PhysicsError> {
        let o = self.pair_observer.ok_or_else(unsupported)?;
        if o.island_construction.is_some()
            || o.spheres
                .iter()
                .any(|id| self.world.bodies[self.bodies[id].0].is_enabled())
        {
            return Err(unsupported());
        }
        let (mut joined, vel) = self.read_sphere_island(bytes)?;
        for link in o.links {
            let handle = self.bodies[&link].0;
            let source = &self.world.bodies[handle];
            let collider = source.colliders()[0];
            joined.bodies[handle].copy_from(source);
            joined.colliders[collider].set_enabled(true);
        }
        joined
            .bodies
            .propagate_modified_body_positions_to_colliders(&mut joined.colliders);
        // Register a fresh chain in the receiver's persistent island graph.
        // The topology and handles are identical, so the full native tree can
        // replace the constructor's zeroed tree without losing generalized motion.
        let [parent, child] = o.links.map(|id| self.bodies[&id].0);
        // An immediate merge can precede the pair's first advancing interval.
        joined
            .multibody_joints
            .remove_multibody_articulations(parent, false);
        let handle = MultibodyJointHandle(child.0);
        let (tree, index) = self
            .world
            .multibody_joints
            .get(handle)
            .ok_or_else(invalid)?;
        let data = tree.link(index).ok_or_else(invalid)?.joint.data;
        let inserted = joined
            .multibody_joints
            .insert(parent, child, data, false)
            .ok_or_else(invalid)?;
        if inserted != handle {
            return Err(invalid());
        }
        *joined
            .multibody_joints
            .get_mut(inserted)
            .ok_or_else(invalid)?
            .0 = tree.clone();
        // A cold hinge must still run its first-step velocity initialization.
        joined.integration_parameters = self.world.integration_parameters;
        self.world = joined;
        self.pair_observer.as_mut().unwrap().initial_velocity = vel;
        self.refresh_max_boundary_spin();
        Ok(())
    }
}
