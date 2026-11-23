// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Shared f64 construction and state values, and the dynamic backend contract.

mod backend;
mod damped_area_foundation;
mod elastic_area_foundation;
mod elastic_layers;
pub mod flat_landing;
pub mod sole_balance;
pub mod sole_calibration;
pub mod sole_contact;
pub mod sole_supports;
pub mod sole_turntable;
pub use damped_area_foundation::{DampedAreaFoundation, DampedAreaSample};
pub use elastic_area_foundation::{ElasticAreaFoundation, ElasticAreaIntegral, ElasticAreaSample};
mod elastic_line_foundation;
pub use elastic_line_foundation::{ElasticLineFoundation, ElasticLineIntegral, ElasticLineSample};
mod normal_approach;
pub use normal_approach::NormalApproach;
mod interval;
mod line_closure;
pub use line_closure::{LineClosure, LineClosureSearch, exclude_closed_line};
mod line_wrench;
pub use line_wrench::LineWrench;
mod line_sticking;
pub use line_sticking::{LineStickingResponse, StickingLine};
mod segment_transfer;
pub use segment_transfer::{
    SegmentPointImpact, generalized_segment_impact, generalized_segment_work_impact,
};
mod finite_feature_arc;
pub use finite_feature_arc::{
    BoundPointImpact, BoundPointPath, BoundPointPiece, FiniteArcEnd, FiniteArcResult,
    FiniteFeatureArc, SlidingVertexSpan, SlipGrowth, StickingPointSpan,
};
mod finite_feature_track;
pub use finite_feature_track::{
    EdgeFaceFeature, FeatureLimit, FeatureMargin, FiniteContactSegment, FiniteFeatureSample,
    FiniteFeatureTrack,
};
mod finite_point_load;
pub use finite_point_load::{
    FiniteLoadMode, FinitePointLoad, FinitePointSite, SphericalPointMaterial,
};
mod finite_box_flight;
mod finite_edge_crossing;
mod finite_face_vertex;
mod fractional_point_path;
pub use finite_box_flight::{
    BoxCrossing, FiniteBoxArrival, FiniteBoxFlight, FlightBox, SeparatingFeature,
};
pub use finite_edge_crossing::{FiniteContactWitness, SkewEdgeCrossing};
pub use finite_face_vertex::FaceVertexCrossing;
pub use fractional_point_path::{FractionalPointPath, SphericalImpactState};
mod planar_edge_arc;
mod planar_edge_pivot;
pub use planar_edge_arc::{DeckDirection, PlanarEdgeAdvance, PlanarEdgeArc};
mod planar_edge;
pub use planar_edge::{EdgeMotion, PlanarEdgeContact, PlanarEdgeReaction};
mod planar_impact;
pub use planar_impact::{Bearing, PlanarImpactResponse, PlanarPatchImpact};
mod frictionless_edge_arc;
pub use frictionless_edge_arc::{
    BoundFrictionlessEdge, EdgeSupportEnvelope, EdgeSupportSample, EdgeSupportState,
    EdgeSupportTrajectory, FrictionlessEdgeSupport,
};
mod coplanar_supports;
mod deck_edge_support;
pub use coplanar_supports::{
    BeltCapture, BeltCaptureRange, BeltDualBound, BeltEquilibriumWitness, BeltVertexReaction,
    CoplanarSlidingContact, SlidingAccelerationSet, SlidingAccelerationVertex, SlidingLoadChoice,
    SlidingLoadSet, SupportLoadRange, coplanar_load_range,
};
pub use deck_edge_support::{BoundDeckEdge, DeckEdgeEnvelope, DeckEdgeSample, DeckEdgeTrajectory};
mod frictionless_face_support;
pub use frictionless_face_support::{
    BoundFaceSupport, BoundSharedFace, ComovingSupportStop, CoplanarPad, FaceBearingAdvance,
    FaceBearingSample, FaceOverlap, FacePressureExit, FrictionlessFaceSupport, SharedFaceStep,
    SharedSiteLoad, UniformSlidingStep,
};
mod planar_sole_rocking;
pub use planar_sole_rocking::{
    PlanarRockingAdvance, PlanarRockingImpact, PlanarSoleRocking, RockingCycle, RockingLimit,
    RockingSample,
};
mod frictionless_sole_arc;
pub use frictionless_sole_arc::{
    FrictionlessSoleArc, SoleBearingAdvance, SoleBearingSpan, SolePressureTransfer,
};
mod rectangular_normals;
pub use rectangular_normals::{
    NormalCornerEquivalents, RectangularNormalContact, RectangularNormalResponse,
};
mod two_normals;
pub use two_normals::{NormalSiteState, TwoNormalContact, TwoNormalResponse};
mod opening_edge;
pub use opening_edge::{OpeningEdgeImpact, OpeningEdgeResponse};
mod mixed_edge;
pub use mixed_edge::{EdgeImpulseSet, EdgeStickMode, MixedEdgeImpact, MixedEdgeResponse};
mod edge_impact;
pub use edge_impact::{EdgeMobility, SlidingEdgeImpact, SlidingEdgeResponse};
mod alternating_normals;
pub use alternating_normals::{AlternatingNormalInput, AlternatingNormalLimit};
mod point_impact;
pub use point_impact::{PointImpact, PointImpactResponse, PointImpactSensitivity};
mod substeps;
pub use substeps::NativeStepGrid;
pub mod joint_drive;
pub mod posture;
pub mod trajectory;
pub use backend::{
    BodyId, BodyStateLookup, DynamicBackend, InitialPhysicsState, JointId, JointPositionServo,
    JointState, JointVelocityMotor, PhysicsBackend, PhysicsError, PhysicsReset,
    PreparedPhysicsReset, RestoredPhysics, install, install_with_step,
};

#[derive(Debug, Clone, Copy)]
pub enum Shape {
    Sphere {
        radius: f64,
    },
    /// Radius in X/Y and half-height along Gazebo Z.
    Cylinder {
        radius: f64,
        half_height: f64,
    },
    /// Spherical caps around a Z-axis segment of length twice `half_length`.
    Capsule {
        radius: f64,
        half_length: f64,
    },
    Box {
        half_extents: [f64; 3],
    },
}

/// Fixed local Z-up plane. `size` is the SDF XY metadata; collision extent is
/// backend-specific. `position` and XYZW `orientation` locate the link frame.
#[derive(Debug, Clone, Copy)]
pub struct PlaneSpec {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub size: [f64; 2],
}

/// Initial state, in meters, kilograms, and meters per second. `None` mass is fixed.
#[derive(Debug, Clone, Copy)]
pub struct BodySpec {
    pub shape: Shape,
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub mass: Option<f64>,
}

/// One mesh collider in a link-local rigid frame. Vertices already include
/// glTF node transforms, axis conversion and SDF scale. Geometry is borrowed only
/// during admission; the backend owns its validated construction shape for reset.
#[derive(Debug, Clone, Copy)]
pub struct MeshBodySpec<'a> {
    pub vertices: &'a [[f64; 3]],
    pub triangles: &'a [[u32; 3]],
    pub collision_position: [f64; 3],
    pub collision_orientation: [f64; 4],
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    /// None admits a static triangle surface; Some admits a closed convex solid.
    pub mass: Option<f64>,
}

/// One primitive collision in a link's local frame, in meters and XYZW rotation.
/// It does not define a separate body, mass or public link identity.
#[derive(Debug, Clone, Copy)]
pub struct CollisionSpec {
    pub shape: Shape,
    pub position: [f64; 3],
    pub orientation: [f64; 4],
}

/// A link's inertial frame, relative to its body frame. Tensor order is xx,yy,zz,xy,xz,yz.
#[derive(Debug, Clone, Copy)]
pub struct Inertia {
    pub center: [f64; 3],
    pub orientation: [f64; 4],
    pub tensor: [f64; 6],
}

/// Rotational state. Quaternions use [x,y,z,w]; angular velocity is in world axes.
#[derive(Debug, Clone, Copy)]
pub struct RotationSpec {
    pub orientation: [f64; 4],
    pub angular_velocity: [f64; 3],
}

impl Default for RotationSpec {
    fn default() -> Self {
        Self {
            orientation: [0.0, 0.0, 0.0, 1.0],
            angular_velocity: [0.0; 3],
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyState {
    /// Body origin in world axes.
    pub position: [f64; 3],
    /// Center-of-mass velocity in world axes; differs from origin velocity for offset COM.
    pub velocity: [f64; 3],
    pub orientation: [f64; 4],
    pub angular_velocity: [f64; 3],
}

/// Extension identity shared by simulator physics installers.
/// Claimed outside mutable ECS resources so removing a resource cannot reinstall
/// a second stepping system in the same simulation.
pub struct PhysicsInstallation;
