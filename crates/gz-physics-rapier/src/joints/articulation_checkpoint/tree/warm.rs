// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Retain a sphere pair's solver anchors, warm impulses and contact recycling phase
//! when two independently solved gripper assemblies begin sharing one native world.
use super::*;
use rapier3d_f64::geometry::{
    ContactManifold, ContactManifoldData, PairContacts, RigidPairContacts,
};

const MAGIC: &[u8; 8] = b"GZFWAM01";
pub const WARM_GRASP_MEMBER_IMAGE_LIMIT: usize = 16 * 1024;

fn reject(stage: &str) -> PhysicsError {
    PhysicsError::Native {
        engine: "multifidelity",
        message: format!("unsupported warm grasp graph remap: {stage}"),
    }
}
#[derive(Serialize, Deserialize)]
struct Recycle {
    pos12: Pose,
    rot1: Rotation,
    rot2: Rotation,
    max_extent: f64,
    max_drift: f64,
}
#[derive(Serialize, Deserialize)]
struct Pair {
    a: u8,
    b: u8,
    manifold: Option<ContactManifold>,
    recycle: Option<Recycle>,
}
#[derive(Serialize, Deserialize)]
struct Member {
    core: Vec<u8>,
    pairs: Vec<Pair>,
}

fn copy_contact_data(target: &mut ContactManifoldData, source: &ContactManifoldData) {
    // Body handles, solver color, graph position and body indices are assigned
    // by the destination graph. All physical outputs and cached anchors travel.
    target.solver_flags = source.solver_flags;
    target.normal = source.normal;
    target.solver_contacts = source.solver_contacts.clone();
    target.relative_dominance = source.relative_dominance;
    target.user_data = source.user_data;
    target.friction = source.friction;
    target.restitution = source.restitution;
}

/// Rapier exposes the solver manifolds, but its same-build snapshot is the public
/// interface for the recycling reference. Check the layout against the serializer's
/// actual public fields and retain its native color/body/workspace bytes verbatim.
fn rigid_segments(
    r: &RigidPairContacts,
) -> Result<(Vec<u8>, usize, Option<Recycle>), PhysicsError> {
    if !r.solver_clusters.is_empty() || r.manifolds.len() > 1 {
        return Err(reject("clustered contacts"));
    }
    let b = codec().serialize(r).map_err(|_| invalid())?;
    let prefix = codec()
        .serialize(&(&r.manifolds, &r.solver_clusters))
        .map_err(|_| invalid())?;
    let mut rest = b
        .strip_prefix(prefix.as_slice())
        .ok_or_else(|| reject("native pair schema"))?;
    let (_, _, workspace): (u8, [u32; 2], Option<()>) =
        codec().deserialize_from(&mut rest).map_err(|_| invalid())?;
    if workspace.is_some() {
        return Err(reject("sphere-pair query workspace"));
    }
    let offset = b.len() - rest.len();
    let recycle = codec()
        .deserialize(rest)
        .map_err(|_| reject("native recycling schema"))?;
    Ok((b, offset, recycle))
}
fn set_recycle(r: &mut RigidPairContacts, value: &Option<Recycle>) -> Result<(), PhysicsError> {
    let (mut b, at, _) = rigid_segments(r)?;
    b.truncate(at);
    append(&mut b, value)?;
    *r = codec().deserialize(&b).map_err(|_| invalid())?;
    Ok(())
}
fn graph_codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .with_limit(GRASP_FLEET_IMAGE_LIMIT as u64)
        .reject_trailing_bytes()
}
fn pack(m: &Member) -> Result<Vec<u8>, PhysicsError> {
    let mut data = Vec::from(MAGIC.as_slice());
    data.extend_from_slice(&[0; 32]);
    append(&mut data, m)?;
    if data.len() > WARM_GRASP_MEMBER_IMAGE_LIMIT {
        return Err(invalid());
    }
    let digest = Sha256::digest(&data[40..]);
    data[8..40].copy_from_slice(&digest);
    Ok(data)
}
fn unpack(bytes: &[u8]) -> Result<Member, PhysicsError> {
    if !(40..=WARM_GRASP_MEMBER_IMAGE_LIMIT).contains(&bytes.len())
        || &bytes[..8] != MAGIC
        || bytes[8..40] != Sha256::digest(&bytes[40..])[..]
    {
        return Err(invalid());
    }
    let value: Member = codec().deserialize(&bytes[40..]).map_err(|_| invalid())?;
    if value.core.len() > CLEAR_GRASP_MEMBER_IMAGE_LIMIT
        || value.pairs.len() > MAX_GROUNDED_TREE_LINKS
    {
        return Err(invalid());
    }
    Ok(value)
}
impl RapierBackend {
    fn export_warm_group(&self, root: BodyId, part: BodyId) -> Result<Member, PhysicsError> {
        let layout = self.clear_layout(root, part)?;
        let handles = self.clear_handles(&layout, part)?;
        let tail = *handles.last().ok_or_else(invalid)?;
        let mut pairs = Vec::new();
        for p in self
            .world
            .narrow_phase
            .contact_pairs()
            .filter(|p| p.collider1 == tail || p.collider2 == tail)
        {
            let ord = |h| handles.iter().position(|i| *i == h).map(|i| i as u8);
            let (Some(a), Some(b)) = (ord(p.collider1), ord(p.collider2)) else {
                if p.has_any_active_contact() {
                    return Err(reject("warm contact with another assembly"));
                }
                continue;
            };
            let rigid = p.rigid().ok_or_else(invalid)?;
            let (_, _, recycle) = rigid_segments(rigid)?;
            if recycle.is_none() && rigid.manifolds.is_empty() {
                continue;
            }
            let manifold = rigid.manifolds.first().map(|m| {
                let mut saved = m.clone();
                let mut portable = ContactManifoldData::default();
                copy_contact_data(&mut portable, &saved.data);
                saved.data = portable;
                saved
            });
            pairs.push(Pair {
                a,
                b,
                manifold,
                recycle,
            });
        }
        Ok(Member {
            core: self.encode_clear_member(root, part)?,
            pairs,
        })
    }
    /// Capture bounded warm sphere contacts for this assembly in native topology order.
    pub fn checkpoint_warm_grasp_member(
        &self,
        root: BodyId,
        payload: BodyId,
    ) -> Result<Option<Vec<u8>>, PhysicsError> {
        if self.joints.open || self.bodies.len() != self.joints.records.len() + 2 {
            return Err(unsupported_tree());
        }
        super::super::grounded::validate_contacts(&self.world)?;
        let member = self.export_warm_group(root, payload)?;
        if !member
            .pairs
            .iter()
            .any(|p| p.manifold.as_ref().is_some_and(|m| !m.points.is_empty()))
        {
            return Ok(None);
        }
        pack(&member).map(Some)
    }

    /// Rebuild the destination's native pair graph without advancing simulation time,
    /// then bind each retained manifold and recycling reference to its actual graph row.
    pub fn prepare_warm_grasp_fleet(
        &mut self,
        groups: &[(BodyId, BodyId); 2],
        packets: [&[u8]; 2],
    ) -> Result<(), PhysicsError> {
        let warm = packets.map(|p| p.get(..8) == Some(MAGIC));
        if !warm.contains(&true) {
            return Err(reject("expected a retained warm member"));
        }
        let member = |bytes: &[u8]| {
            if bytes.get(..8) == Some(b"GZFCLR01") && bytes.len() <= CLEAR_GRASP_MEMBER_IMAGE_LIMIT
            {
                Ok(Member {
                    core: bytes.to_vec(),
                    pairs: Vec::new(),
                })
            } else {
                unpack(bytes)
            }
        };
        let members = [member(packets[0])?, member(packets[1])?];
        let mut staged =
            self.stage_grasp_members(groups, [&members[0].core, &members[1].core], false)?;
        let w = &mut staged.world;
        let saved_broad = w.broad_phase.clone();
        w.narrow_phase = NarrowPhase::new();
        let prediction = w.integration_parameters.prediction_distance();
        CollisionPipeline::new().step(
            prediction,
            &mut w.islands,
            &mut w.broad_phase,
            &mut w.narrow_phase,
            &mut w.bodies,
            &mut w.colliders,
            &crate::collision::Groups,
            &(),
        );
        w.broad_phase = saved_broad;
        let mut graph = w.narrow_phase.contact_graph().clone();
        let mut expected = BTreeSet::new();
        let mut ordered = Vec::new();
        for (&(root, payload), member) in groups.iter().zip(&members) {
            let l = staged.clear_layout(root, payload)?;
            let handles = staged.clear_handles(&l, payload)?;
            let tail = handles.len() - 1;
            for p in &member.pairs {
                let (ia, ib) = (usize::from(p.a), usize::from(p.b));
                if ia >= handles.len()
                    || ib >= handles.len()
                    || ia == ib
                    || (ia != tail && ib != tail)
                {
                    return Err(reject("pair role"));
                }
                let (a, b) = (handles[ia], handles[ib]);
                ordered.push((a, b));
                let pair_key = if a.into_raw_parts() < b.into_raw_parts() {
                    (a.into_raw_parts(), b.into_raw_parts())
                } else {
                    (b.into_raw_parts(), a.into_raw_parts())
                };
                if !expected.insert(pair_key) {
                    return Err(reject("duplicate member contact"));
                }
                let edges = graph
                    .raw_graph()
                    .raw_edges()
                    .iter()
                    .find(|e| e.weight.collider1 == a && e.weight.collider2 == b)
                    .map(|e| (e.source(), e.target()));
                let Some((n1, n2)) = edges else {
                    return Err(reject("oriented destination contact missing"));
                };
                let (_, _, dst) = graph.interaction_pair_mut(n1, n2).ok_or_else(invalid)?;
                let PairContacts::Rigid(ref mut rigid) = dst.contacts else {
                    return Err(invalid());
                };
                match (&p.manifold, rigid.manifolds.as_mut_slice()) {
                    (Some(saved), [dest]) => {
                        if saved.points.len() > 1
                            || saved.data.solver_contacts.len() > saved.points.len()
                        {
                            return Err(invalid());
                        }
                        let mut data = dest.data.clone();
                        copy_contact_data(&mut data, &saved.data);
                        *dest = saved.clone();
                        dest.data = data;
                    }
                    (None, []) => {}
                    _ => return Err(reject("destination manifold count")),
                }
                if let Some(r) = &p.recycle {
                    let a = &staged.world.colliders[a];
                    let b = &staged.world.colliders[b];
                    let extent = |c: &Collider| {
                        let aabb = c.shape().compute_local_aabb();
                        aabb.mins.length().max(aabb.maxs.length())
                    };
                    let params = &staged.world.integration_parameters;
                    let drift = params.normalized_contact_recycle_distance * params.length_unit;
                    let expected_drift = if rigid.has_any_active_contact() {
                        drift
                    } else {
                        drift.min(prediction)
                    };
                    if !r.pos12.translation.is_finite()
                        || rotation(r.pos12.rotation.to_array()).is_err()
                        || rotation(r.rot1.to_array()).is_err()
                        || rotation(r.rot2.to_array()).is_err()
                        || r.max_extent != extent(a).max(extent(b))
                        || r.max_drift != expected_drift
                    {
                        return Err(reject("recycling reference"));
                    }
                }
                set_recycle(rigid, &p.recycle)?;
            }
        }
        for p in graph.interactions().filter(|p| p.has_any_active_contact()) {
            let a = p.collider1.into_raw_parts();
            let b = p.collider2.into_raw_parts();
            if !expected.contains(&(a.min(b), a.max(b))) {
                return Err(reject("unrepresented destination contact"));
            }
        }
        // Only replace the typed contact graph prefix. All graph bookkeeping, graph
        // indices, destination allocation and island ownership come from native detection.
        let original = graph_codec()
            .serialize(&staged.world.narrow_phase)
            .map_err(|_| invalid())?;
        let old = graph_codec()
            .serialize(staged.world.narrow_phase.contact_graph())
            .map_err(|_| invalid())?;
        let suffix = original
            .strip_prefix(old.as_slice())
            .ok_or_else(|| reject("native narrow-phase graph schema"))?;
        // Generic multibody rows are solved in edge order after a graph rebuild.
        // Only permute the active row slots. Empty candidate edges retain their
        // native handles and indices. Each active slot has one generic row and
        // all colliders have the same empty event mask, preserving the indexed
        // destination hints across this permutation.
        let existing = graph.raw_graph().raw_edges();
        if staged
            .world
            .colliders
            .iter()
            .any(|(_, c)| !c.active_events().is_empty())
            || existing.iter().any(|edge| {
                edge.weight.has_any_active_contact() && edge.weight.solver_manifolds().len() != 1
            })
        {
            return Err(reject("nonuniform solver edges"));
        }
        let mut active = Vec::new();
        for (a, b) in ordered {
            let edge = existing
                .iter()
                .find(|edge| edge.weight.collider1 == a && edge.weight.collider2 == b)
                .ok_or_else(invalid)?;
            if edge.weight.has_any_active_contact() {
                active.push(edge);
            }
        }
        let mut active = active.into_iter();
        let mut raw = graph.raw_graph().clone();
        for i in (0..existing.len()).rev() {
            raw.remove_edge(rapier3d_f64::geometry::TemporaryInteractionIndex::new(
                i as u32,
            ));
        }
        for original in existing {
            let row = if original.weight.has_any_active_contact() {
                active.next().ok_or_else(invalid)?
            } else {
                original
            };
            raw.add_edge(row.source(), row.target(), row.weight.clone());
        }
        if active.next().is_some() {
            return Err(reject("unmatched active solver row"));
        }
        let mut merged = graph_codec().serialize(&raw).map_err(|_| invalid())?;
        merged.extend_from_slice(suffix);
        staged.world.narrow_phase = graph_codec().deserialize(&merged).map_err(|_| invalid())?;
        super::super::grounded::validate_contacts(&staged.world)?;
        for ((&(root, part), original), has_warm) in groups.iter().zip(packets).zip(warm) {
            let exported = staged.export_warm_group(root, part)?;
            let actual = if has_warm {
                pack(&exported)?
            } else {
                if exported
                    .pairs
                    .iter()
                    .any(|p| p.manifold.as_ref().is_some_and(|m| !m.points.is_empty()))
                {
                    return Err(reject("previously clear member gained a contact"));
                }
                exported.core
            };
            if actual != original {
                return Err(reject("materialized prior readback"));
            }
        }
        *self = staged;
        Ok(())
    }
}
