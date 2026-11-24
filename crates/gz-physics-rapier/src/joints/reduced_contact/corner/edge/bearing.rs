// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Advance an isolated, frictionless locked assembly along its two-vertex
//! bearing. All other native bodies keep their normal public clock.
mod transfer;
use super::super::sustained::RigidTreeMotion;
use super::*;
mod batch;
use gz_physics::{EdgeSupportState, FrictionlessEdgeSupport};
pub use transfer::{
    SharedFaceReceipt, SharedSupportLoad, SharedSupportRequest, DeckEdgeBearingReceipt, FaceBearingReceipt, FractionalBearingSpan, RockingLimitReceipt, SoleTransferReceipt,
};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TwoBearingReceipt {
    pub foot: BodyId,
    pub support: BodyId,
    pub start: Duration,
    pub duration: Duration,
    pub normal: [f64; 3],
    /// Cuboid vertex IDs. Bits 0, 1 and 2 select the positive local x, y and z sides.
    pub sole_vertices: [u8; 2],
    pub start_points: [[f64; 3]; 2],
    pub end_points: [[f64; 3]; 2],
    /// Equivalent endpoint normal impulses of the line bearing. A continuous
    /// pressure density along that edge is not individually observed.
    pub normal_impulses: [f64; 2],
    pub normal_force_bounds: [[f64; 2]; 2],
    pub angular_impulse_about_com: [f64; 3],
    pub integration_steps: u32,
}
// The common smooth span is produced on every bearing step. Keep it inline
// so only the less frequent topology-changing receipts allocate.
#[allow(clippy::large_enum_variant)]
pub(crate) enum BearingResult {
    Smooth(TwoBearingReceipt),
    Transfer(Box<SoleTransferReceipt>),
    Rocking(Box<RockingLimitReceipt>),
    Face(Box<FaceBearingReceipt>),
    Deck(Box<DeckEdgeBearingReceipt>),
    Shared(Box<SharedFaceReceipt>,Option<Box<SharedFaceReceipt>>,ColliderHandle),
}
pub(crate) struct PreparedBearing {
    pub colliders: (ColliderHandle, ColliderHandle),
    motion: RigidTreeMotion,
    receipt: BearingResult,
}
impl PreparedBearing {
    pub(crate) fn second_pad(&self)->Option<ColliderHandle>{
        if let BearingResult::Shared(_,_,pad)= &self.receipt {Some(*pad)}else{None}
    }
    pub(crate) fn commit(self, backend: &mut RapierBackend) -> BearingResult {
        self.motion.commit(backend);
        self.record(backend)
    }
    pub(crate) fn commit_batch(items:Vec<Self>,backend:&mut RapierBackend)->Vec<BearingResult>{
        if items.is_empty(){return Vec::new();}
        for item in &items{item.motion.commit_links(backend);}
        backend.world.bodies.propagate_modified_body_positions_to_colliders(&mut backend.world.colliders);
        items.into_iter().map(|item|item.record(backend)).collect()
    }
    fn record(self,backend:&mut RapierBackend)->BearingResult{
        if let BearingResult::Shared(load,None,second)=&self.receipt && load.comoving_rest
            && let Some(evidence)=crate::articulation_support::ParkedSharedSupport::capture(backend,self.motion.root,self.colliders.0,[self.colliders.1,*second],load){
            backend.parked_shared.push(evidence);
        }
        self.receipt
    }
}
#[derive(Clone, Copy)]
pub(crate) struct BearingModes<'a> {
    pub batch:Option<&'a crate::approach_clearance::SharedNeighborhood>,
    pub edge: bool,
    pub sole: bool,
    pub rocking: bool,
    pub face: bool,
    pub deck: bool,
    pub shared:Option<SharedSupportRequest>,
}
impl RapierBackend {
    pub(crate) fn parked_supports_still_clear(&self,index:MultibodyIndex,foot:BodyId,supports:[BodyId;2],dt:Duration)->Result<bool,PhysicsError>{
        if self.solved_shared_groups.len()>1{
            return self.shared_group_remains_clear(dt);
        }
        let Some(first)=self.registered_boxes(foot,supports[0])? else{return Ok(false);};
        let Some(second)=self.registered_boxes(foot,supports[1])? else{return Ok(false);};
        let Some(tree)=self.world.multibody_joints.get_multibody(index) else{return Ok(false);};
        let Some((_,com))=self.locked_mass_center(tree,&self.world.bodies[first.foot])else{return Ok(false);};
        if tree.ndofs()!=6{return Ok(false);}
        let gravity=self.world.gravity;
        let tol=512.*f64::EPSILON*(1.+com.length()+first.pad_half.length()+second.pad_half.length());
        self.swept_locked_supports(tree,index,first,Some(second),com,gravity,tol,0.,0.,dt,None)
    }
    /// Co-level, normally stationary bottom face. The finite-polygon solver
    /// checks positive pressure, actual overlap and swept clearance before commit.
    pub fn frictionless_face_now(&self, foot: BodyId, pad: BodyId) -> Result<bool, PhysicsError> {
        self.coplanar_face_now(foot,pad,false)
    }
    pub fn coplanar_face_now(&self,foot:BodyId,pad:BodyId,allow_kinetic:bool)->Result<bool,PhysicsError>{
        let Some(b) = self.registered_boxes(foot, pad)? else {
            return Ok(false);
        };
        if b.friction != 0. && !allow_kinetic || self.world.bodies[b.pad].angvel() != Vector::ZERO {
            return Ok(false);
        }
        let n = b.pad_pose.rotation * Vector::Z;
        if (b.sole_pose.rotation * Vector::Z - n).length() > 512. * f64::EPSILON {
            return Ok(false);
        }
        let top = b.pad_pose.translation + n * b.pad_half.z;
        let bottom = b.sole_pose.translation - n * b.sole_half.z;
        if n.dot(bottom - top).abs() > 512. * f64::EPSILON * (1. + bottom.length() + top.length()) {
            return Ok(false);
        }
        let Some((v, w)) = self.finite_incoming_twist(b.foot) else {
            return Ok(false);
        };
        let relative = v - self.world.bodies[b.pad].linvel();
        let eps = 512. * f64::EPSILON;
        Ok(w.cross(n).length() <= eps * (1. + w.length())
            && relative.dot(n).abs() <= eps * (1. + relative.length()))
    }

    /// A geometric and velocity screen for an explicitly registered, initially
    /// bearing sole. Swept clearance, force ownership and positive load are
    /// validated again using the actual native duration before changing poses.
    pub fn frictionless_bearing_now(
        &self,
        foot: BodyId,
        pad: BodyId,
        allow_flat: bool,
    ) -> Result<bool, PhysicsError> {
        let Some(b) = self.registered_boxes(foot, pad)? else {
            return Ok(false);
        };
        let shoe = &self.world.bodies[b.foot];
        let deck = &self.world.bodies[b.pad];
        if b.friction != 0. || deck.angvel() != Vector::ZERO {
            return Ok(false);
        }
        let Some((v, w)) = self.finite_incoming_twist(b.foot) else {
            return Ok(false);
        };
        let n = b.pad_pose.rotation * Vector::Z;
        let top = b.pad_pose.translation + b.pad_half.z * n;
        let c = shoe.center_of_mass();
        let tol = 512.
            * f64::EPSILON
            * (1. + c.length() + top.length() + b.sole_half.length() + b.pad_half.length());
        let mut count = 0;
        let mut bits = 0usize;
        for k in 0..4 {
            let p = b.sole_pose
                * Vector::new(
                    if k & 2 == 0 {
                        -b.sole_half.x
                    } else {
                        b.sole_half.x
                    },
                    if k & 1 == 0 {
                        -b.sole_half.y
                    } else {
                        b.sole_half.y
                    },
                    -b.sole_half.z,
                );
            let gap = n.dot(p - top);
            if gap < -tol {
                return Ok(false);
            }
            let arm = p - c;
            let rel = v - deck.linvel();
            let utol = 512. * f64::EPSILON * (1. + rel.length() + w.length() * arm.length());
            if gap.abs() <= tol && n.dot(rel + w.cross(arm)).abs() <= utol {
                let lp = b.pad_pose.inverse_transform_point(p);
                if lp.x.abs() >= b.pad_half.x || lp.y.abs() >= b.pad_half.y {
                    return Ok(false);
                }
                count += 1;
                bits ^= k;
            }
        }
        Ok(count == 2 && bits.count_ones() == 1 || allow_flat && count == 4)
    }
    pub(crate) fn prepare_two_bearing(
        &self,
        foot: BodyId,
        pad: BodyId,
        start: Duration,
        dt: Duration,
        modes: BearingModes<'_>,
    ) -> Result<Option<PreparedBearing>, PhysicsError> {
        if dt.is_zero()
            || !self.held_wrenches.is_empty()
            || !self.joints.efforts.is_empty()
            || !self.spinning_assemblies.is_empty()
            || self.free_coast.is_some()
            || self.curves.authored()
            || self.conserved_rotor.is_some()
        {
            return Ok(None);
        }
        let resolve = modes.sole || modes.face;
        let sites = if resolve {
            None
        } else {
            self.cold_edge_sites(foot, pad, Vector::ZERO, 0.)?
        };
        let b = if let Some(e) = &sites {
            e.boxes
        } else if resolve {
            let Some(b) = self.registered_boxes(foot, pad)? else {
                return Ok(None);
            };
            b
        } else {
            return Ok(None);
        };
        let world = &self.world;
        let deck = &world.bodies[b.pad];
        let shoe = &world.bodies[b.foot];
        if b.friction != 0. && !modes.shared.is_some_and(|p|p.uniform_kinetic) || deck.angvel() != Vector::ZERO {
            return Ok(None);
        }
        let owner = world
            .multibody_joints
            .rigid_body_link(b.foot)
            .expect("registered articulated body")
            .multibody;
        let tree = world
            .multibody_joints
            .get_multibody(owner)
            .ok_or(PhysicsError::InvalidJoint)?;
        if tree.ndofs() != 6
            || tree
                .armature()
                .iter()
                .chain(tree.damping().iter())
                .any(|v| *v != 0.)
            || !tree.couplings().is_empty()
        {
            return Ok(None);
        }
        let Some((mass, com)) = self.locked_mass_center(tree, shoe) else {
            return Ok(None);
        };
        let Some((v, angular)) = self.finite_incoming_twist(b.foot) else {
            return Ok(None);
        };
        let gravity = world.gravity * shoe.gravity_scale();
        let velocity = v + angular.cross(com - shoe.center_of_mass());
        let relative = velocity - deck.linvel();
        let root = tree.root().rigid_body_handle();
        let old = *world.bodies[root].position();
        let q = old.rotation;
        let local = [Vector::X, Vector::Y, Vector::Z]
            .map(|axis| q.inverse() * self.locked_inertia_action(tree, com, q * axis));
        let inertia = rapier3d_f64::parry::math::Matrix::from_cols(local[0], local[1], local[2]);
        let n = b.pad_pose.rotation * Vector::Z;
        let top = b.pad_pose.translation + n * b.pad_half.z;
        let tolerance = 512.
            * f64::EPSILON
            * (1. + com.length() + top.length() + b.sole_half.length() + b.pad_half.length());
        let initial = EdgeSupportState {
            center: com - top,
            orientation: q,
            velocity: relative,
            angular_velocity: angular,
        };
        if resolve {
            return transfer::Frame {
                boxes: b,
                owner,
                root,
                old,
                com,
                deck_velocity: deck.linvel(),
                start: initial,
                mass,
                inertia,
                gravity,
                normal: n,
                top,
                tolerance,
                foot,
                pad,
            }
            .prepare(self, start, dt, modes);
        }
        let sites = sites.expect("matched discrete two-vertex support");
        let arms = sites.world.map(|p| q.inverse() * (p - com));
        let body = FrictionlessEdgeSupport {
            mass,
            inertia_body: inertia,
            sites_body: arms,
            normal: n,
            plane_offset: 0.,
            gravity,
        };
        let Some(arc) = body.bind(initial)? else {
            return Ok(None);
        };
        let h = dt.as_secs_f64();
        let Some(envelope) = arc.enclose(h)? else {
            return Ok(None);
        };
        if envelope.force_bounds.iter().any(|v| v[0] <= 0.) {
            return Ok(None);
        }
        let sole = b.sole_half;
        for k in 0..8 {
            let sign = |bit: u32| if k & (1 << bit) == 0 { -1. } else { 1. };
            let p = b.sole_pose * Vector::new(sign(0) * sole.x, sign(1) * sole.y, sign(2) * sole.z);
            let body = q.inverse() * (p - com);
            if !sites.world.contains(&p) && envelope.height(body)[0] <= tolerance {
                return Ok(None);
            }
            let pad = b.pad_pose.inverse_transform_point(p);
            let reach = tolerance
                + h * (envelope.center_speed_bound + envelope.angular_speed_bound * body.length());
            if pad.x.abs() + reach >= b.pad_half.x || pad.y.abs() + reach >= b.pad_half.y {
                return Ok(None);
            }
        }
        if !self.swept_locked_neighborhood(
            tree,
            owner,
            b,
            com,
            gravity,
            tolerance,
            envelope.center_speed_bound,
            envelope.angular_speed_bound,
            dt,
        )? {
            return Ok(None);
        }
        let Some((sample, steps)) = arc.resolved_sample(h)? else {
            return Ok(None);
        };
        let state = sample.state;
        let expected = mass * n.dot(state.velocity - relative - h * gravity);
        if sample.signed_impulse.iter().any(|v| *v <= 0.)
            || (sample.signed_impulse.iter().sum::<f64>() - expected).abs()
                > 1e-10 * (1. + expected.abs())
        {
            return Ok(None);
        }
        let center = top + deck.linvel() * h + state.center;
        let rotation = state.orientation * q.inverse();
        let pose = Pose::from_parts(
            center + rotation * (old.translation - com),
            state.orientation,
        );
        let momentum = |q: Rotation, w: Vector| q * (inertia * (q.inverse() * w));
        Ok(Some(PreparedBearing {
            motion: RigidTreeMotion {
                owner,
                root,
                root_pose: pose,
                center,
                center_velocity: state.velocity + deck.linvel(),
                angular: state.angular_velocity,
            },
            colliders: b.colliders,
            receipt: BearingResult::Smooth(TwoBearingReceipt {
                foot,
                support: pad,
                start,
                duration: dt,
                normal: n.to_array(),
                sole_vertices: sites.vertices,
                start_points: sites.world.map(|v| v.to_array()),
                end_points: arms.map(|r| (center + state.orientation * r).to_array()),
                normal_impulses: sample.signed_impulse,
                normal_force_bounds: envelope.force_bounds,
                angular_impulse_about_com: (momentum(state.orientation, state.angular_velocity)
                    - momentum(q, angular))
                .to_array(),
                integration_steps: steps,
            }),
        }))
    }
}
