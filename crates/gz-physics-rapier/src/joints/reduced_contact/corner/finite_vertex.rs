// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Sustain one moving material vertex sliding on a finite translating support.
use super::{sustained::RigidTreeMotion, *};
use gz_physics::{FiniteFeatureArc, FinitePointSite, SeparatingFeature, SphericalPointMaterial};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SlidingPointReceipt {
    pub feature: SeparatingFeature,
    pub normal_impulse: f64,
    pub normal_impulse_on_foot: [f64; 3],
    pub foot: BodyId,
    pub support: BodyId,
    pub start: Duration,
    pub duration: Duration,
    pub normal: [f64; 3],
    pub point_start: [f64; 3],
    pub point_end: [f64; 3],
    pub impulse_on_foot: [f64; 3],
    pub angular_impulse_about_com: [f64; 3],
    pub friction_work: f64,
    pub minimum_slip_speed: f64,
    pub slip_growth:Option<gz_physics::SlipGrowth>,
    pub end_material_velocity:[f64;3],
    pub minimum_inward_acceleration: f64,
    pub subdivisions: u32,
}
pub type SlidingVertexReceipt = SlidingPointReceipt;
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StickingPointReceipt {
    pub foot: BodyId,
    pub support: BodyId,
    pub start: Duration,
    pub duration: Duration,
    pub feature: SeparatingFeature,
    pub point_start: [f64; 3],
    pub point_end: [f64; 3],
    pub normal_start: [f64; 3],
    pub normal_end: [f64; 3],
    pub normal_impulse: f64,
    pub normal_impulse_on_foot: [f64; 3],
    pub impulse_on_foot: [f64; 3],
    pub angular_impulse_about_com: [f64; 3],
    pub minimum_normal_force: f64,
    pub minimum_cone_margin: f64,
    pub subdivisions: u32,
}
#[derive(Clone, Copy)]
pub(crate) struct PointFollowModes {
    pub remember_endpoint: bool,
    pub before_line: bool,
    pub vertex: bool,
    pub sticking: bool,
    pub transfer: bool,
    pub skew: bool,
    pub launch:bool,
    pub subdivide:bool,
    pub directed:bool,
}
/// Material load committed over one native interval. Transfer totals already
/// include the instantaneous impulses recorded in their individual pieces.
#[derive(Clone, Debug, PartialEq)]
pub enum PointSpanLoad {
    Sliding(SlidingVertexReceipt),
    Sticking(StickingPointReceipt),
    Transfer(BoundTransferReceipt),
}
impl PointSpanLoad {
    /// Ordered (foot, support), unsigned normal impulse and signed world impulse on the foot.
    pub fn reaction(&self) -> ((BodyId, BodyId), f64, [f64; 3]) {
        match self {
            Self::Sliding(s) => ((s.foot,s.support),s.normal_impulse,s.impulse_on_foot),
            Self::Sticking(s) => ((s.foot,s.support),s.normal_impulse,s.impulse_on_foot),
            Self::Transfer(s) => ((s.foot,s.support),s.normal_impulse,s.impulse_on_foot),
        }
    }
}
/// Site at the end of a proven material path, valid only for the next
/// contiguous native interval of the same pair inside this public call.
#[derive(Clone, Copy)]
pub(crate) struct MaterialEndpoint {
    pub site: FinitePointSite,
    pub sticking: bool,
    pub limit: Option<(gz_physics::FeatureLimit, f64)>,
}
impl From<&FractionalPointReceipt> for MaterialEndpoint {
    fn from(r: &FractionalPointReceipt) -> Self {
        Self {
            site: FinitePointSite { feature: r.end_feature, point: r.end_point.into(), normal: r.end_normal.into() },
            sticking: r.end_sticking,
            limit: r.approaching_limit,
        }
    }
}
pub(crate) struct PreparedVertex {
    pub(crate) endpoint: Option<MaterialEndpoint>,
    pub(crate) colliders: (ColliderHandle, ColliderHandle),
    motion: RigidTreeMotion,
    receipt: PointSpanLoad,
}
impl PreparedVertex {
    pub(crate) fn duration(&self) -> Duration {
        match &self.receipt {
            PointSpanLoad::Sliding(s) => s.duration,
            PointSpanLoad::Sticking(s) => s.duration,
            PointSpanLoad::Transfer(s) => s.duration,
        }
    }
    pub(crate) fn commit(self, backend: &mut RapierBackend) -> PointSpanLoad {
        self.motion.commit(backend);
        self.receipt
    }
}
impl RapierBackend {
    /// Coarsening a public native grid also coarsens unrelated bodies. Only
    /// the selected pad and members of the same free tree may move in this world.
    /// The usual swept contact check still tests every fixed obstacle.
    pub(crate) fn isolated_point_world(&self, foot:BodyId,pad:BodyId)->bool {
        let (Some(a),Some(b))=(self.bodies.get(&foot),self.bodies.get(&pad)) else{return false};
        let links=&self.world.multibody_joints;
        let Some(owner)=links.rigid_body_link(a.0) else{return false};
        self.world.bodies.iter().all(|(handle,body)|body.is_fixed() || handle==b.0 || links.rigid_body_link(handle).is_some_and(|l|l.multibody==owner.multibody))
    }
    pub(crate) fn prepare_bound_point(
        &self,
        pair: (BodyId,BodyId),
        start: Duration,
        dt: Duration,
        hit: Option<FiniteImpactReceipt>,
        endpoint: Option<MaterialEndpoint>,
        modes: PointFollowModes,
    ) -> Result<Option<PreparedVertex>, PhysicsError> {
        let (foot,pad)=pair;
        let PointFollowModes {
            remember_endpoint, before_line, vertex: slide,
            sticking: stick,
            transfer,
            skew,launch,subdivide,directed,
        } = modes;
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
        let w = &self.world;
        let shoe = &w.bodies[b.foot];
        let deck = &w.bodies[b.pad];
        if deck.angvel() != Vector::ZERO {
            return Ok(None);
        }
        let owner = w
            .multibody_joints
            .rigid_body_link(b.foot)
            .unwrap()
            .multibody;
        let tree = w
            .multibody_joints
            .get_multibody(owner)
            .ok_or(PhysicsError::InvalidJoint)?;
        if tree.ndofs() != 6
            || !tree.couplings().is_empty()
            || tree.armature().iter().any(|q| *q != 0.)
            || tree.damping().iter().any(|q| *q != 0.)
        {
            return Ok(None);
        }
        let Some((mass, center, inertia)) = self.spherical_mass_properties(tree, shoe) else {
            return Ok(None);
        };
        let material = SphericalPointMaterial {
            mass,
            inertia,
            friction: b.friction,
        };
        let Some((v, angular)) = self.generalized_link_twist(b.foot) else {
            return Ok(None);
        };
        let gravity = w.gravity * shoe.gravity_scale();
        let speed = v + angular.cross(center - shoe.center_of_mass()) - deck.linvel();
        let f = gz_physics::FiniteBoxFlight {
            center_of_mass: center,
            relative_velocity: speed,
            angular_velocity: angular,
            acceleration: gravity,
            ..b.flight_geometry()
        };
        let scale = 1.
            + b.sole_pose.translation.length()
            + b.pad_pose.translation.length()
            + b.sole_half.length()
            + b.pad_half.length();
        let tol = 1024. * f64::EPSILON * scale;

        let (site, arc, stuck) = if let Some(r) = endpoint {
            (r.site, FiniteFeatureArc::new(f,r.site,material,r.sticking,tol)?, r.sticking)
        } else if let Some(hit) = hit {
            let h = hit.resultant;
            if (h.sticking && !stick && !launch) || (!h.sticking && !slide && !skew && !launch) {
                return Ok(None);
            }
            let site = FinitePointSite {
                feature: hit.feature,
                point: h.point.into(),
                normal: h.normal.into(),
            };
            if let Some(line) = hit.segment {
                let Some((_, s)) = f.incident_segment_now(0.) else {
                    return Ok(None);
                };
                if line.points != s.moving_points.map(|p| p.to_array()) {
                    return Ok(None);
                }
                (
                    site,
                    FiniteFeatureArc::after_line_endpoint(
                        f,
                        s,
                        line.loaded_end,
                        material,
                        h.sticking,
                        tol,
                    )?,
                    h.sticking,
                )
            } else {
                if !h.sticking
                    && !(slide && matches!(site.feature, SeparatingFeature::SupportFace(_))
                        || (skew || launch) && matches!(site.feature, SeparatingFeature::EdgePair(..)))
                {
                    return Ok(None);
                }
                (
                    site,
                    FiniteFeatureArc::new(f, site, material, h.sticking, h.gap.abs() + tol)?,
                    h.sticking,
                )
            }
        } else {
            let Some((feature, n, [p, _], _)) = f.contact_geometry(tol) else {
                return Ok(None);
            };
            let site = FinitePointSite {
                feature,
                normal: n,
                point: p,
            };
            let stuck = (stick || launch)
                && (speed + angular.cross(p - center)).length()
                    <= 256.
                        * f64::EPSILON
                        * (1. + speed.length() + angular.length() * p.distance(center));
            if !stuck
                && !(slide && matches!(feature, SeparatingFeature::SupportFace(_))
                    || (skew || launch) && matches!(feature, SeparatingFeature::EdgePair(..)))
            {
                return Ok(None);
            }
            (
                site,
                FiniteFeatureArc::new(f, site, material, stuck, tol)?,
                stuck,
            )
        };
        let Some(arc) = arc else { return Ok(None) };
        let dt = if before_line {
            let Some(h) = arc.vertex_handoff_hint(dt.as_secs_f64())? else { return Ok(None) };
            h
        } else {
            dt
        };
        let translation = dt.as_secs_f64() * deck.linvel();
        let span = if stuck && stick {
            arc.sticking_point_span(dt.as_secs_f64())?
        } else {
            None
        };
        let mut sliding = if !stuck && slide {
            if before_line {arc.sliding_vertex_approach(dt.as_secs_f64())?}
            else {arc.sliding_vertex_span(dt.as_secs_f64())?}
        } else {
            None
        };
        if sliding.is_none() && !stuck && skew {
            sliding = arc.sliding_skew_span(dt.as_secs_f64())?;
        }
        if sliding.is_none() && span.is_none() && launch {
            sliding=arc.sliding_skew_launch_span(dt.as_secs_f64())?;
        }
        let (step, linear_bound, spin_bound, shift, receipt) = if let Some(span) = span {
            let step = span.motion;
            let receipt = StickingPointReceipt {
                foot,
                support: pad,
                start,
                duration: dt,
                feature: step.site.site.feature,
                point_start: site.point.to_array(),
                point_end: (step.site.site.point + translation).to_array(),
                normal_start: site.normal.to_array(),
                normal_end: step.site.site.normal.to_array(),
                normal_impulse: step.normal_impulse,
                normal_impulse_on_foot: step.normal_impulse_vector.to_array(),
                impulse_on_foot: step.impulse.to_array(),
                angular_impulse_about_com: step.angular_impulse.to_array(),
                minimum_normal_force: span.minimum_normal_force,
                minimum_cone_margin: span.minimum_cone_margin,
                subdivisions: span.subdivisions,
            };
            (
                step,
                span.center_speed_bound,
                span.angular_speed_bound,
                step.initial_translation.abs(),
                PointSpanLoad::Sticking(receipt),
            )
        } else if let Some(span) = sliding {
            let step = span.motion;
            let receipt = SlidingPointReceipt {
                feature: step.site.site.feature,
                normal_impulse: step.normal_impulse,
                normal_impulse_on_foot: step.normal_impulse_vector.to_array(),
                foot,
                support: pad,
                start,
                duration: dt,
                normal: step.site.site.normal.to_array(),
                point_start: site.point.to_array(),
                point_end: (step.site.site.point + translation).to_array(),
                impulse_on_foot: step.impulse.to_array(),
                angular_impulse_about_com: step.angular_impulse.to_array(),
                friction_work: step.friction_work,
                minimum_slip_speed: span.minimum_slip_speed,
                slip_growth:span.slip_growth,
                end_material_velocity:(step.velocity+step.angular_velocity.cross(step.site.site.point-step.center)).to_array(),
                minimum_inward_acceleration: span.minimum_inward_acceleration,
                subdivisions: span.subdivisions,
            };
            (
                step,
                span.center_speed_bound,
                span.angular_speed_bound,
                step.initial_translation.abs(),
                PointSpanLoad::Sliding(receipt),
            )
        } else if transfer
            && let Some(path) = if directed {arc.directed_point_transfers(dt.as_secs_f64(),skew,launch)?} else if subdivide {arc.subdivided_point_transfers(dt.as_secs_f64(),skew,launch)?} else if launch {arc.resolved_launch_transfers(dt.as_secs_f64())?} else if skew {
                arc.resolved_spatial_transfers(dt.as_secs_f64())?
            } else {
                arc.resolved_point_transfers(dt.as_secs_f64())?
            }
        {
            if before_line && path.pieces.iter().any(|piece| piece.impact.is_some() || piece.motion.site.site.feature != site.feature) {
                return Ok(None);
            }
            let step = path.pieces.last().expect("qualified complete path").motion;
            (
                step,
                path.center_speed_bound,
                path.angular_speed_bound,
                path.translation_allowance,
                PointSpanLoad::Transfer(BoundTransferReceipt::from_path(
                    foot,
                    pad,
                    start,
                    dt,
                    path,
                    deck.linvel(),
                )),
            )
        } else {
            return Ok(None);
        };
        if !self.swept_locked_neighborhood(
            tree,
            owner,
            b,
            center,
            gravity,
            tol + shift,
            linear_bound,
            spin_bound,
            dt,
        )? {
            return Ok(None);
        }
        let next = step.center + translation;
        let root = tree.root().rigid_body_handle();
        let old = *w.bodies[root].position();
        let dq = step.moving.orientation * b.sole_pose.rotation.inverse();
        let motion = RigidTreeMotion {
            owner,
            root,
            root_pose: Pose::from_parts(
                next + dq * (old.translation - center),
                (dq * old.rotation).normalize(),
            ),
            center: next,
            center_velocity: step.velocity + deck.linvel(),
            angular: step.angular_velocity,
        };
        let endpoint = remember_endpoint.then(|| {
        let sticking = match &receipt {
            PointSpanLoad::Sliding(_) => false,
            PointSpanLoad::Sticking(_) => true,
            PointSpanLoad::Transfer(path) => path.intervals.last().expect("qualified path").sticking,
        };
        MaterialEndpoint {
            site: FinitePointSite { point: step.site.site.point + translation, ..step.site.site },
            sticking,
            limit: step.site.approaching_limit(),
        }});
        Ok(Some(PreparedVertex {
            endpoint,
            colliders: b.colliders,
            motion,
            receipt,
        }))
    }
}
