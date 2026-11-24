// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Certified cold impact of a spherical-inertia locked assembly on a finite deck.
//! One material point can belong to a vertex/face or skew edge/edge pair.
use super::sustained::RigidTreeMotion;
use super::*;
use gz_physics::{FiniteBoxArrival, FiniteBoxFlight, FlightBox, SeparatingFeature};
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteImpactReceipt {
    pub feature: SeparatingFeature,
    /// Causal change of native grid after this impulse. Time is relative to
    /// the impact. All native contacts remain enabled over this suffix.
    pub next_feature_boundary: Option<(gz_physics::FeatureLimit, Duration)>,
    pub resultant: CornerImpactReceipt,
    pub segment: Option<FiniteEndpointCertificate>,
}
#[derive(Clone, Copy)]
pub(super) struct RigidFlight {
    initial: RigidTreeMotion,
    acceleration: Vector,
}
impl RigidFlight {
    pub(super) fn at(self, t: Duration) -> RigidTreeMotion {
        let s = self.initial;
        let t = t.as_secs_f64();
        let rotation = Rotation::from_scaled_axis(s.angular * t);
        let center = s.center + (s.center_velocity + 0.5 * t * self.acceleration) * t;
        RigidTreeMotion {
            root_pose: Pose::from_parts(
                center + rotation * (s.root_pose.translation - s.center),
                (rotation * s.root_pose.rotation).normalize(),
            ),
            center,
            center_velocity: s.center_velocity + self.acceleration * t,
            ..s
        }
    }
}
impl RegisteredBoxes {
    pub(super) fn flight_geometry(&self) -> FiniteBoxFlight {
        let bx = |p: Pose, half: Vector| FlightBox {
            center: p.translation,
            orientation: p.rotation,
            half_extent: half,
        };
        FiniteBoxFlight {
            moving: bx(self.sole_pose, self.sole_half),
            support: bx(self.pad_pose, self.pad_half),
            center_of_mass: Default::default(),
            relative_velocity: Default::default(),
            angular_velocity: Default::default(),
            acceleration: Default::default(),
        }
    }
}
impl RapierBackend {
    pub(super) fn finite_incoming_twist(&self, h: RigidBodyHandle) -> Option<(Vector, Vector)> {
        if self.joints.construction_open() {
            let rb = &self.world.bodies[h];
            Some((rb.linvel(), rb.angvel()))
        } else {
            self.generalized_link_twist(h)
        }
    }
    fn finite_candidate(
        &self,
        foot: BodyId,
        pad: BodyId,
        allowance: f64,
        certified: Option<gz_physics::FiniteContactWitness>,
    ) -> Result<Option<(SeparatingFeature, Candidate)>, PhysicsError> {
        let Some(b) = self.registered_boxes(foot, pad)? else {
            return Ok(None);
        };
        let geometry = b.flight_geometry();
        let current = match certified {
            Some(edge @ gz_physics::FiniteContactWitness::SkewEdges(_)) => edge.measure(geometry.moving,geometry.support,allowance),
            _ => geometry.contact_geometry(allowance).or_else(|| certified?.measure(geometry.moving,geometry.support,allowance))
        };
        let Some((feature, n, [site, _], gap)) = current
        else {
            return Ok(None);
        };
        let normal = Vector::from_array(n.to_array());
        let point = Vector::from_array(site.to_array());
        let shoe = &self.world.bodies[b.foot];
        let deck = &self.world.bodies[b.pad];
        let origin =
            Vector::from_array(self.state(foot).ok_or(PhysicsError::MissingBody)?.position);
        let e = Vector::from_array(n.any_orthonormal_vector().to_array());
        Ok(Some((
            feature,
            Candidate {
                site: ArticulatedContactSite {
                    body: foot,
                    local_point: (shoe.rotation().inverse() * (point - origin)).to_array(),
                },
                colliders: b.colliders,
                world_point: point,
                gap,
                basis: [e, normal.cross(e), normal],
                deck_velocity: deck.linvel() + deck.angvel().cross(point - deck.center_of_mass()),
                friction: b.friction,
            },
        )))
    }
    /// Detect an isolated point whose two material velocities agree in the normal
    /// direction to roundoff. This only nominates a cold bearing for refinement.
    /// The integrator still proves the load, material law and swept geometry.
    pub fn tangent_material_point(&self, foot: BodyId, pad: BodyId) -> Result<bool, PhysicsError> {
        let Some((_, site)) = self.finite_candidate(foot,pad,0.,None)? else {return Ok(false);};
        let handle=self.bodies[&foot].0;
        let Some((v,w))=self.finite_incoming_twist(handle) else{return Ok(false);};
        let lever=site.world_point-self.world.bodies[handle].center_of_mass();
        let relative=v+w.cross(lever)-site.deck_velocity;
        let tolerance=256.*f64::EPSILON*(1.+v.length()+w.length()*lever.length()+site.deck_velocity.length());
        Ok(relative.dot(site.basis[2]).abs()<=tolerance)
    }
    pub fn finite_point_closing(&self, foot: BodyId, pad: BodyId) -> Result<bool, PhysicsError> {
        let Some((_, c)) = self.finite_candidate(foot, pad, 0., None)? else {
            return Ok(false);
        };
        let h = self.bodies[&foot].0;
        let Some((v, w)) = self.finite_incoming_twist(h) else {
            return Ok(false);
        };
        Ok(
            (v + w.cross(c.world_point - self.world.bodies[h].center_of_mass()) - c.deck_velocity)
                .dot(c.basis[2])
                < 0.,
        )
    }
    pub(crate) fn apply_finite_at(
        &mut self,
        foot: BodyId,
        pad: BodyId,
        elapsed: Duration,
        allowance: f64,
        certified:Option<gz_physics::FiniteContactWitness>,
    ) -> Result<Option<FiniteImpactReceipt>, PhysicsError> {
        let Some((feature, site)) = self.finite_candidate(foot, pad, allowance, certified)? else {
            return Ok(None);
        };
        Ok(self
            .apply_point_candidate(site, foot, pad, elapsed)?
            .map(|resultant| FiniteImpactReceipt {
                feature,
                resultant,
                next_feature_boundary: None,
                segment: None,
            }))
    }
    pub(crate) fn finite_transfer_hint(
        &self,
        hit: FiniteImpactReceipt,
    ) -> Result<Option<(gz_physics::FeatureLimit, f64)>, PhysicsError> {
        let h = hit.resultant;
        let Some(b) = self.registered_boxes(h.foot, h.support)? else {
            return Ok(None);
        };
        let shoe = &self.world.bodies[b.foot];
        let Some((v, w)) = self.generalized_link_twist(b.foot) else {
            return Ok(None);
        };
        let site = gz_physics::FinitePointSite {
            normal: h.normal.into(),
            point: h.point.into(),
            feature: hit.feature,
        };
        let geometry = b.flight_geometry();
        let center = shoe.center_of_mass();
        let speed = v - self.world.bodies[b.pad].linvel();
        // Both the sole and the segment's endpoint belong to one rigid link.
        // Only the instantaneous link twist enters this first-order estimate.
        let s = if let Some(cert) = hit.segment {
            let Some((_, segment)) = geometry.incident_segment_now(0.) else {
                return Ok(None);
            };
            if segment.moving_points.map(|v| v.to_array()) != cert.points {
                return Ok(None);
            }
            let state = gz_physics::FiniteBoxFlight {
                center_of_mass: center,
                relative_velocity: speed,
                angular_velocity: w,
                ..geometry
            };
            let Some((_, sample)) =
                gz_physics::FiniteFeatureTrack::outgoing_endpoint(state, segment, cert.loaded_end)
            else {
                return Ok(None);
            };
            sample
        } else {
            let track = gz_physics::FiniteFeatureTrack::new(geometry, site)?;
            let Some(s) = track.sample(geometry.moving, center, speed, w) else {
                return Ok(None);
            };
            if s.margins().iter().any(|m| m.distance <= 0.) {
                return Ok(None);
            }
            s
        };
        Ok(s.approaching_limit())
    }
    pub fn finite_arrival(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
    ) -> Result<Option<Duration>, PhysicsError> {
        self.finite_arrival_certified(foot,pad,dt,false)
    }
    pub fn finite_arrival_certified(&self,foot:BodyId,pad:BodyId,dt:Duration,certify_face_vertex:bool)->Result<Option<Duration>,PhysicsError>{
        self.finite_arrival_with_witnesses(foot,pad,dt,certify_face_vertex,false)
    }
    pub fn finite_arrival_with_witnesses(&self,foot:BodyId,pad:BodyId,dt:Duration,certify_face_vertex:bool,certify_skew:bool)->Result<Option<Duration>,PhysicsError>{
        Ok(self
            .finite_flight(foot, pad, dt, certify_face_vertex,certify_skew)?
            .filter(|f| f.arrives)
            .map(|f| f.at))
    }
    pub(crate) fn finite_flight(
        &self,
        foot: BodyId,
        pad: BodyId,
        dt: Duration,
        certify_vertex:bool,
        certify_skew:bool,
    ) -> Result<Option<CornerFlight>, PhysicsError> {
        self.finite_flight_mode(foot,pad,dt,certify_vertex,certify_skew,None)
    }
    pub fn fractional_point_arrival(&self,foot:BodyId,pad:BodyId,dt:Duration)->Result<Option<Duration>,PhysicsError>{
        let mut staged=None;
        Ok(self.finite_flight_mode(foot,pad,dt,false,false,Some(&mut staged))?.filter(|f| f.arrives).map(|f|f.at))
    }
    pub(crate) fn finite_flight_mode(&self,foot:BodyId,pad:BodyId,dt:Duration,certify_vertex:bool,certify_skew:bool,
        prepare:Option<&mut Option<Box<PreparedFractional>>>)->Result<Option<CornerFlight>,PhysicsError>{
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
        let Some(b) = self.registered_boxes(foot, pad)? else {
            return Ok(None);
        };
        let world = &self.world;
        let shoe = &world.bodies[b.foot];
        let deck = &world.bodies[b.pad];
        if deck.angvel() != Vector::ZERO {
            return Ok(None);
        }
        let owner = world
            .multibody_joints
            .rigid_body_link(b.foot)
            .unwrap()
            .multibody;
        let tree = world
            .multibody_joints
            .get_multibody(owner)
            .ok_or(PhysicsError::InvalidJoint)?;
        if tree.ndofs() != 6
            || !tree.couplings().is_empty()
            || tree.armature().iter().any(|q| *q != 0.)
            // initialize_velocities clears construction-time Rapier drag.
            || (!self.joints.construction_open() && tree.damping().iter().any(|q| *q != 0.))
        {
            return Ok(None);
        }
        let Some((mass, center, inertia)) = self.spherical_mass_properties(tree, shoe) else {
            return Ok(None);
        };
        let Some((v, angular)) = self.finite_incoming_twist(b.foot) else {
            return Ok(None);
        };
        let velocity = v + angular.cross(center - shoe.center_of_mass());
        let acceleration = world.gravity * shoe.gravity_scale();
        let mut g = b.flight_geometry();
        g.center_of_mass = center;
        g.relative_velocity = velocity - deck.linvel();
        g.angular_velocity = angular;
        g.acceleration = acceleration;
        let mut finite_witness=None;
        let mut staged=None;
        let (at, arrives, allowance) = match g.first_crossing(dt)? {
            FiniteBoxArrival::Crossing(c) => {
                if prepare.is_some() {
                    let material=gz_physics::SphericalPointMaterial{mass,inertia,friction:b.friction};
                    staged=g.fractional_point_path(c,material)?;
                    if staged.is_none(){return Ok(None);}
                } else if certify_skew && matches!(c.feature,SeparatingFeature::EdgePair(..)) {
                    let material=gz_physics::SphericalPointMaterial{mass,inertia,friction:b.friction};
                    let Some(edge)=g.single_skew_impact_in_bracket(c,material)? else{return Ok(None);};
                    finite_witness=Some(gz_physics::FiniteContactWitness::SkewEdges(edge));
                } else if certify_vertex {
                    finite_witness=g.isolated_face_vertex(c).map(gz_physics::FiniteContactWitness::FaceVertex);
                }
                if c.points.is_none() && finite_witness.is_none() && staged.is_none() { return Ok(None); }
                let delta = (c.latest - c.earliest).as_secs_f64();
                let v = g.relative_velocity.length()
                    + g.acceleration.length() * c.latest.as_secs_f64()
                    + angular.length()
                        * (b.sole_pose.translation.distance(center) + b.sole_half.length());
                (c.latest, true, 1.000001 * delta * v)
            }
            FiniteBoxArrival::Separated => (dt, false, 0.),
            _ => return Ok(None),
        };
        if let Err(e) = self.check_contact_tree_geometry(tree, Some(b.colliders), true, true) {
            return match e {
                PhysicsError::Unsupported { .. } => Ok(None),
                _ => Err(e),
            };
        }
        let t = at.as_secs_f64();
        let mut sweeps = Vec::new();
        for l in tree.links() {
            let h = l.rigid_body_handle();
            let rb = &world.bodies[h];
            if !rb.is_dynamic()
                || !rb.is_enabled()
                || l.joint.kinematic
                || !rb.locked_axes().is_empty()
                || rb.user_force() != Vector::ZERO
                || rb.user_torque() != Vector::ZERO
                || rb.linear_damping() != 0.
                || rb.angular_damping() != 0.
                || rb.gravity_scale() * world.gravity != acceleration
                || world.impulse_joints_with(h).next().is_some()
            {
                return Ok(None);
            }
            if self.joints.construction_open() {
                let expected = velocity + angular.cross(rb.center_of_mass() - center);
                let eps = 256. * f64::EPSILON;
                if rb.linvel().distance(expected) > eps * (1. + expected.length())
                    || rb.angvel().distance(angular) > eps * (1. + angular.length())
                {
                    return Ok(None);
                }
            }
            for &ch in rb.colliders() {
                let c = &world.colliders[ch];
                let pose =
                    *rb.position() * c.position_wrt_parent().ok_or(PhysicsError::InvalidValue)?;
                let bound = c.shape().compute_local_bounding_sphere();
                let reach = (pose * bound.center - center).length() + bound.radius;
                // Include the deviation from the unimpeded sweep over the subnanosecond
                // supported trajectory, bounded by total kinetic energy in the deck frame.
                let material_margin=staged.as_ref().map_or(0.,|path|{
                    let span=t-path.crossing_bracket[0];
                    let energy_speed=g.relative_velocity.length().hypot((inertia/mass).sqrt()*angular.length())+t*acceleration.length();
                    let changes=path.follow.iter().map(|s|s.motion.initial_translation.abs()).sum::<f64>();
                    changes+2.*span*energy_speed*(1.+reach*(mass/inertia).sqrt())
                });
                let margin = material_margin+world.integration_parameters.prediction_distance()
                    + angular.length() * reach * t
                    + 0.5 * acceleration.length() * t * t
                    + allowance;
                if ch != b.colliders.0
                    && crate::collision::can_exchange_impulses(
                        c,
                        &world.colliders[b.colliders.1],
                        deck.body_type(),
                    )
                    && !query::distance(
                        &pose,
                        c.shape(),
                        &b.pad_pose,
                        world.colliders[b.colliders.1].shape(),
                    )
                    .is_ok_and(|q| {
                        q.distance
                            > margin + c.contact_skin() + (velocity - deck.linvel()).length() * t
                    })
                {
                    return Ok(None);
                }
                sweeps.push((
                    c,
                    crate::approach_clearance::ApproachSweep {
                        pose,
                        velocity,
                        dt: at,
                        margin,
                    },
                ));
            }
        }
        if !self.approach_tree_neighbors_clear(&sweeps, |h| {
            h == b.pad
                || world
                    .multibody_joints
                    .rigid_body_link(h)
                    .is_some_and(|l| l.multibody == owner)
        })? {
            return Ok(None);
        }
        let root = tree.root().rigid_body_handle();
        let initial = RigidTreeMotion {
            owner,
            root,
            root_pose: *world.bodies[root].position(),
            center,
            center_velocity: velocity,
            angular,
        };
        if let (Some(out),Some(path))=(prepare,staged){
            *out=Some(Box::new(PreparedFractional::new(initial,b.sole_pose.rotation,deck.linvel(),(foot,pad),at,path)));
        }
        Ok(Some(CornerFlight {
            finite_witness,
            foot,
            support: pad,
            at,
            arrives,
            allowance,
            colliders: b.colliders,
            geometry: ProjectionGeometry::Finite,
            ballistic: None,
            rigid: Some(RigidFlight {
                initial,
                acceleration,
            }),
        }))
    }
}
