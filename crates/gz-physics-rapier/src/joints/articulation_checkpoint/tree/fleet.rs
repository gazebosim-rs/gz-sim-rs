// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Two complete grippers, their independent payloads, and every native contact between them.
use super::*;
const MAGIC: &[u8; 8] = b"GZFCPL01";
const SHEAR_MAGIC: &[u8; 8] = b"GZFCPL02";
/// Maximum same-build native graph for two 3–8-link articulated grippers and two spherical payloads.
pub const GRASP_FLEET_IMAGE_LIMIT: usize = 2 * GRASP_TREE_IMAGE_LIMIT;
fn graph_error(stage: &str) -> PhysicsError {
    PhysicsError::Native {
        engine: "multifidelity",
        message: format!("invalid coupled grasp-fleet {stage}"),
    }
}
fn fleet_graph_codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit((GRASP_FLEET_IMAGE_LIMIT - 40) as u64)
        .reject_trailing_bytes()
}
impl RapierBackend {
    pub(super) fn fleet_layout(
        &self,
        groups: &[(BodyId, BodyId); 2],
    ) -> Result<[TreeLayout; 2], PhysicsError> {
        let mut layouts = Vec::with_capacity(2);
        let mut seen = std::collections::BTreeSet::new();
        let mut previous = None;
        for &(root, payload) in groups {
            if !seen.insert(root) || !seen.insert(payload) {
                return Err(unsupported_tree());
            }
            let (root_handle, .., group) =
                self.bodies.get(&root).ok_or(PhysicsError::MissingBody)?;
            let group = group.ok_or_else(unsupported_tree)?;
            if previous.is_some_and(|p| p >= group) {
                return Err(unsupported_tree());
            }
            previous = Some(group);
            let layout =
                self.scoped_tree_layout(true, Some(TreeExternal::Grasp(payload)), Some(group))?;
            if layout.root != *root_handle {
                return Err(unsupported_tree());
            }
            if !self.joints.open {
                self.check_tree_dynamics(&layout)
                    .map_err(|_| graph_error("generalized dynamics"))?;
            }
            layouts.push(layout);
        }
        let expected_joints: usize = layouts.iter().map(|l| l.joints.len()).sum();
        if self.bodies.len() != expected_joints + 4 || self.joints.records.len() != expected_joints
        {
            return Err(unsupported_tree());
        }
        // Links in each original tree share an explicit self-collision exclusion.
        // Every cross-tree or external-body contact is admitted to this common graph.
        for pair in self
            .world
            .narrow_phase
            .contact_pairs()
            .filter(|p| p.has_any_active_contact())
        {
            let group = |h| {
                self.world
                    .colliders
                    .get(h)
                    .and_then(|c| c.parent())
                    .and_then(|rb| self.bodies.values().find(|b| b.0 == rb))
                    .map(|b| b.4)
                    .ok_or_else(invalid)
            };
            let (a, b) = (group(pair.collider1)?, group(pair.collider2)?);
            if a.is_some() && a == b {
                return Err(invalid());
            }
        }
        super::super::grounded::validate_contacts(&self.world)?;
        layouts.try_into().map_err(|_| invalid())
    }
    fn fleet_fingerprint(layouts: &[TreeLayout; 2]) -> [u8; 32] {
        let mut hash = Sha256::new();
        hash.update(MAGIC);
        for layout in layouts {
            hash.update(layout.construction);
        }
        hash.finalize().into()
    }
    /// Capture two motorized grippers and every native inter-assembly warm contact.
    /// Supply (root, payload) in increasing authored model ID order.
    pub fn checkpoint_grasp_fleet(
        &self,
        groups: &[(BodyId, BodyId); 2],
    ) -> Result<Vec<u8>, PhysicsError> {
        if self.joints.open || self.world.integration_parameters.dt != STEP.as_secs_f64() {
            return Err(unsupported_tree());
        }
        let layouts = self.fleet_layout(groups)?;
        let mut payload = Self::fleet_fingerprint(&layouts).to_vec();
        let magic = if let Some(o) = self.grasp_payload_observer {
            let mut parts = [groups[0].1, groups[1].1];
            parts.sort_unstable();
            if o.parts != parts {
                return Err(unsupported_tree());
            }
            append(&mut payload, &(o.incoming_velocity, o.external_impulse))?;
            SHEAR_MAGIC
        } else {
            MAGIC
        };
        for l in &layouts {
            self.append_tree_motion(l, &mut payload)?;
        }
        fleet_graph_codec()
            .serialize_into(&mut payload, &self.world)
            .map_err(|_| invalid())?;
        if payload.len() + 40 > GRASP_FLEET_IMAGE_LIMIT {
            return Err(invalid());
        }
        let mut bytes = Vec::with_capacity(payload.len() + 40);
        bytes.extend_from_slice(magic);
        bytes.extend_from_slice(&Sha256::digest(&payload));
        bytes.extend_from_slice(&payload);
        Ok(bytes)
    }
    /// Restore an exact locally trusted same-build graph into identically authored fresh construction.
    pub fn restore_grasp_fleet(
        &mut self,
        groups: &[(BodyId, BodyId); 2],
        bytes: &[u8],
    ) -> Result<(), PhysicsError> {
        if !self.joints.open || !self.joints.efforts.is_empty() {
            return Err(unsupported_tree());
        }
        if !(72..=GRASP_FLEET_IMAGE_LIMIT).contains(&bytes.len())
            || ![MAGIC.as_slice(), SHEAR_MAGIC.as_slice()].contains(&&bytes[..8])
            || bytes[8..40] != Sha256::digest(&bytes[40..])[..]
        {
            return Err(invalid());
        }
        let initial = self
            .fleet_layout(groups)
            .map_err(|_| graph_error("destination layout"))?;
        if bytes[40..72] != Self::fleet_fingerprint(&initial) {
            return Err(graph_error("construction fingerprint"));
        }
        let mut rest = &bytes[72..];
        let observer = if &bytes[..8] == SHEAR_MAGIC {
            let mut parts = [groups[0].1, groups[1].1];
            parts.sort_unstable();
            let (incoming_velocity, external_impulse): ([f64; 3], [f64; 3]) =
                codec().deserialize_from(&mut rest).map_err(|_| invalid())?;
            if !incoming_velocity
                .iter()
                .chain(&external_impulse)
                .all(|v| v.is_finite())
            {
                return Err(invalid());
            }
            let configured = self.grasp_payload_observer.ok_or_else(unsupported_tree)?;
            if configured.parts != parts {
                return Err(invalid());
            }
            Some(GraspPayloadObserver {
                parts,
                incoming_velocity,
                external_impulse,
            })
        } else {
            None
        };
        let mut motion = Vec::with_capacity(2);
        for tree in &initial {
            let mut rows = Vec::with_capacity(tree.joints.len());
            for _ in &tree.joints {
                let row: JointMotion =
                    codec().deserialize_from(&mut rest).map_err(|_| invalid())?;
                row.validate()?;
                rows.push(row);
            }
            motion.push(rows);
        }
        let graph: PhysicsWorld = fleet_graph_codec()
            .deserialize(rest)
            .map_err(|_| graph_error("serialized graph"))?;
        let mut params = self.world.integration_parameters;
        params.dt = STEP.as_secs_f64();
        if graph.gravity != self.world.gravity
            || graph.bodies.len() != self.bodies.len()
            || graph.colliders.len() != self.bodies.len()
            || graph.integration_parameters != params
        {
            return Err(invalid());
        }
        self.validate_companion_bodies(&graph, Companion::Pair([groups[0].1, groups[1].1]), false)
            .map_err(|_| graph_error("restored collision bodies"))?;
        let mut staged = Self::new_multifidelity(self.world.gravity.to_array())?;
        staged.owner = Arc::clone(&self.owner);
        staged.allow_isolation = self.allow_isolation;
        staged.apply_initial(self.prepare_initial(&self.initial_state())?)?;
        staged.world = graph;
        staged.grasp_payload_observer = observer;
        staged.joints.open = false;
        let actual = staged
            .fleet_layout(groups)
            .map_err(|_| graph_error("restored layout"))?;
        for ((a, b), rows) in actual.iter().zip(&initial).zip(motion) {
            if a.construction != b.construction || a.joints != b.joints {
                return Err(invalid());
            }
            staged
                .install_graph_motion(a, rows)
                .map_err(|_| graph_error("restored generalized coordinates"))?;
        }
        if staged.checkpoint_grasp_fleet(groups)? != bytes {
            return Err(graph_error("re-encoded graph"));
        }
        *self = staged;
        Ok(())
    }
}
