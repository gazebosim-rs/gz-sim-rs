// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Instantaneous point impulses on a passive native articulation. Every site
//! on the same tree shares one joint-space inertia and generalized velocity.
use super::*;
mod corner;
#[cfg(test)]
mod locked_tests;
mod trial;
pub use corner::PointSpanLoad;
pub(crate) use corner::{BearingModes, BearingResult};
pub use corner::{
    BoundPointInterval, BoundTransferReceipt, CornerImpactReceipt, DeckEdgeBearingReceipt,
    EdgeImpactReceipt, FaceBearingReceipt, FacePressureReceipt, FiniteEndpointCertificate,
    FiniteImpactReceipt, FractionalBearingSpan, FractionalPointEvent, FractionalPointReceipt,
    FrictionlessRecontactReceipt, InternalPointImpact, MixedEdgeImpactReceipt,
    OpeningEdgeImpactReceipt, PlanarImpactReceipt, RockingLimitReceipt, SharedFaceReceipt,
    SharedSupportLoad, SharedSupportRequest, SlidingEdgeImpactReceipt, SlidingPointReceipt,
    SlidingVertexReceipt, SoleTransferReceipt, StickingPointReceipt, SustainedEdgeReceipt,
    TwoBearingReceipt,
};
pub(crate) use corner::{
    CornerFlight, MaterialEndpoint, PointFollowModes, PreparedFractional, ProjectionGeometry,
};
use rapier3d_f64::{
    na::{DMatrix, DVector},
    parry::{bounding_volume::BoundingVolume, query},
};
pub use trial::ArticulatedNativeInterval;

/// Maximum number of point sites in a single dense native transaction.
pub const MAX_ARTICULATED_CONTACT_SITES: usize = 16;
const MAX_LINKS: usize = 32;
type V = [f64; 3];

/// Point in the public link frame, in metres from its origin (not its COM).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArticulatedContactSite {
    pub body: BodyId,
    pub local_point: V,
}

/// Certified world-space motion of a selected native collider center before
/// material impact. Angular velocity is signed radians per second about a
/// fixed world line through `pivot`, directed by the unit `axis`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ArticulatedSweep {
    Translation {
        velocity: [f64; 3],
    },
    Revolution {
        pivot: [f64; 3],
        axis: [f64; 3],
        angular_speed: f64,
    },
}

/// Response at one frozen native configuration. Each 3×3 block maps a world
/// impulse in N s at the source site to world velocity in m/s at the target.
#[derive(Clone, Debug, PartialEq)]
pub struct ArticulatedContactSample {
    pub world_points: Vec<V>,
    pub velocities: Vec<V>,
    pub dofs: usize,
    pub links: usize,
    blocks: Vec<[[f64; 3]; 3]>,
}
impl ArticulatedContactSample {
    /// Reconstitute read-only reporting data exported by a managed solver. Only a
    /// transaction borrowing the physical backend can apply impulses.
    pub fn from_measurement(
        world_points: Vec<V>,
        velocities: Vec<V>,
        dofs: usize,
        links: usize,
        blocks: Vec<[[f64; 3]; 3]>,
    ) -> Result<Self, PhysicsError> {
        let n = world_points.len();
        if !(1..=MAX_ARTICULATED_CONTACT_SITES).contains(&n)
            || n != velocities.len()
            || blocks.len() != n * n
            || !(1..=MAX_LINKS).contains(&links)
            || !(1..=6 + links).contains(&dofs)
            || !world_points
                .iter()
                .chain(&velocities)
                .flatten()
                .chain(blocks.iter().flatten().flatten())
                .all(|v| v.is_finite())
        {
            return Err(PhysicsError::InvalidValue);
        }
        Ok(Self {
            world_points,
            velocities,
            dofs,
            links,
            blocks,
        })
    }
    /// Rows are target velocity XYZ. Columns are source impulse XYZ.
    pub fn mobility(&self, target: usize, source: usize) -> Option<[[f64; 3]; 3]> {
        let n = self.world_points.len();
        (target < n && source < n).then(|| self.blocks[n * target + source])
    }
}

/// Native point velocities following one atomic batch. Work includes the
/// kinetic energy of the entire reduced tree and the half-impulse term.
#[derive(Clone, Debug, PartialEq)]
pub struct ArticulatedImpulseReport {
    pub before: ArticulatedContactSample,
    pub after_velocities: Vec<V>,
    pub kinetic_energy_change: f64,
}

/// Holds an exclusive native solver borrow until this frozen configuration
/// receives its impulses or is dropped. No stale operator can be committed.
pub struct ArticulatedContactTransaction<'a> {
    backend: &'a mut RapierBackend,
    operator: FrozenOperator,
}
struct FrozenOperator {
    index: MultibodyIndex,
    sample: ArticulatedContactSample,
    velocity: DVector<f64>,
    jacobian: DMatrix<f64>,
    weighted: DMatrix<f64>,
    links: Vec<(RigidBodyHandle, DMatrix<f64>)>,
}

/// An exact construction lock removes its coordinate from J and M. An
/// ordinary zero-width or active stop still owns a unilateral native row.
fn passive_contact_coordinate(link: &rapier3d_f64::dynamics::MultibodyLink) -> bool {
    link.joint.ndofs() == 1
        || (link.joint.ndofs() == 0
            && link.joint.data.user_data == EXACT_REVOLUTE_LOCK
            && link.joint.data.locked_axes == JointAxesMask::LOCKED_FIXED_AXES
            && link.joint.data.limit_axes.is_empty())
}
fn unsupported(capability: &'static str) -> PhysicsError {
    PhysicsError::Unsupported {
        engine: "multifidelity",
        capability,
    }
}

fn xyz(a: &DMatrix<f64>, row: usize, col: usize) -> Vector {
    Vector::new(a[(row, col)], a[(row + 1, col)], a[(row + 2, col)])
}
fn vectors(v: &DVector<f64>) -> Vec<V> {
    v.as_slice()
        .chunks_exact(3)
        .map(|c| [c[0], c[1], c[2]])
        .collect()
}

impl RapierBackend {
    /// Current center and radius of one attached, enabled sphere collider in
    /// world metres. Its native local pose may be offset from the link COM.
    pub fn articulated_sphere_geometry(&self, id: BodyId) -> Result<([f64; 3], f64), PhysicsError> {
        let e = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
        if self.world.multibody_joints.rigid_body_link(e.0).is_none() {
            return Err(unsupported(
                "material impacts require articulated sphere links",
            ));
        }
        let body = &self.world.bodies[e.0];
        let [h] = body.colliders() else {
            return Err(unsupported(
                "material contact requires a single articulated sphere collider",
            ));
        };
        let collider = &self.world.colliders[*h];
        let sphere = collider
            .shape()
            .as_ball()
            .filter(|_| body.is_enabled() && collider.is_enabled() && !collider.is_sensor())
            .ok_or_else(|| {
                unsupported("material contact requires an enabled spherical fingertip")
            })?;
        let pose = *body.position()
            * collider
                .position_wrt_parent()
                .ok_or(PhysicsError::InvalidValue)?;
        Ok((pose.translation.to_array(), sphere.radius))
    }

    /// Certify uniform translation in zero gravity, or a stationary world-
    /// supported tree whose revolutes carry no gravitational generalized load.
    pub fn articulated_translation(
        &self,
        sites: &[BodyId],
        dt: Duration,
    ) -> Result<[f64; 3], PhysicsError> {
        match self.articulated_sweep(sites, dt)? {
            ArticulatedSweep::Translation { velocity } => Ok(velocity),
            ArticulatedSweep::Revolution { .. } => Err(unsupported(
                "a turning native pad requires its circular trajectory",
            )),
        }
    }

    /// The complete isolated tree either translates or consists of one passive
    /// world revolute with constant angular momentum about its fixed axis.
    pub fn articulated_sweep(
        &self,
        sites: &[BodyId],
        dt: Duration,
    ) -> Result<ArticulatedSweep, PhysicsError> {
        self.sweep_with_support(sites, dt, None)
    }
    fn sweep_with_support(
        &self,
        sites: &[BodyId],
        dt: Duration,
        support: Option<(ColliderHandle, ColliderHandle)>,
    ) -> Result<ArticulatedSweep, PhysicsError> {
        let Some(&first) = sites.first() else {
            return Err(PhysicsError::InvalidValue);
        };
        if sites.len() > MAX_ARTICULATED_CONTACT_SITES || dt.is_zero() {
            return Err(PhysicsError::InvalidValue);
        }
        let handle = self.bodies.get(&first).ok_or(PhysicsError::MissingBody)?.0;
        let index = self
            .world
            .multibody_joints
            .rigid_body_link(handle)
            .ok_or_else(|| unsupported("continuous impact requires a native passive tree"))?
            .multibody;
        let tree = self
            .world
            .multibody_joints
            .get_multibody(index)
            .ok_or(PhysicsError::InvalidJoint)?;
        let translation = self.world.bodies[handle].linvel();
        if !(2..=MAX_LINKS).contains(&tree.num_links())
            || (support.is_none() && tree.num_links() != self.world.bodies.len())
            || self.curves.authored()
            || !self.held_wrenches.is_empty()
            || !self.joints.efforts.is_empty()
            || !tree.couplings().is_empty()
            || tree.frictions().iter().any(|f| *f != 0.)
            || !self.spinning_assemblies.is_empty()
            || self.free_coast.is_some()
            || self.conserved_rotor.is_some()
        {
            return Err(unsupported(
                "continuous material approach requires an isolated, passive native tree",
            ));
        }
        // The native solver lazily removes the initial floating-root slots for
        // a newly mounted tree. Read its real fixed-base configuration without
        // mutating the owner or advancing time just to refresh kinematics.
        let grounded = tree
            .links()
            .any(|l| l.is_root() && self.world.bodies[l.rigid_body_handle()].is_fixed());
        let mut prepared;
        let tree = if grounded {
            prepared = tree.clone();
            prepared.forward_kinematics(&self.world.bodies, true);
            &prepared
        } else {
            tree
        };
        for &id in sites {
            let h = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?.0;
            if self
                .world
                .multibody_joints
                .rigid_body_link(h)
                .is_none_or(|m| m.multibody != index)
            {
                return Err(unsupported(
                    "all swept spheres must belong to the same native tree",
                ));
            }
        }
        let turning = tree.links().any(|l| {
            self.world.bodies[l.rigid_body_handle()].angvel().length() * dt.as_secs_f64()
                > 64. * f64::EPSILON
        });
        if turning
            && !(grounded
                && tree.num_links() == 2
                && tree.ndofs() == 1
                && (self.joints.construction_open() || tree.damping()[0] == 0.))
        {
            return Err(unsupported(
                "a rotating material receiver requires one isolated undamped world revolute",
            ));
        }
        for link in tree.links() {
            let body = &self.world.bodies[link.rigid_body_handle()];
            if !body.is_enabled()
                || !body.locked_axes().is_empty()
                || link.joint.kinematic
                || self
                    .world
                    .impulse_joints_with(link.rigid_body_handle())
                    .next()
                    .is_some()
                || (link.is_root()
                    && !(body.is_dynamic() && link.joint.ndofs() == 6
                        || body.is_fixed() && link.joint.ndofs() == 0))
                || (!link.is_root()
                    && (!body.is_dynamic()
                        || !passive_contact_coordinate(link)
                        || !link.joint.data.motor_axes.is_empty()))
                || body.user_force() != Vector::ZERO
                || body.user_torque() != Vector::ZERO
                || (body.linear_damping() != 0. && body.linvel() != Vector::ZERO)
                || (body.angular_damping() != 0. && body.angvel() != Vector::ZERO)
                || (!turning
                    && (body.linvel() - translation).length()
                        > 128. * f64::EPSILON * (1. + translation.length()))
            {
                return Err(unsupported(
                    "a forced or rotating native tree has no certified linear sweep",
                ));
            }
            if !link.is_root()
                && let Some(limit) = link.joint.data.limits(JointAxis::AngX)
            {
                let angle = link.joint.coords()[3];
                if !(angle > limit.min + 1e-10 && angle < limit.max - 1e-10) {
                    return Err(unsupported(
                        "an engaged joint stop needs a coupled contact solve",
                    ));
                }
            }
        }
        self.require_clear_contact_tree(tree, support, true)?;
        if turning {
            return self.world_revolute_sweep(tree, dt);
        }
        // Registered box flight handles a uniform free-fall acceleration.
        // The public material sweep still certifies only constant velocity.
        if self.world.gravity != Vector::ZERO && (support.is_none() || grounded) {
            self.require_supported_gravity(tree, dt)?;
        }
        Ok(ArticulatedSweep::Translation {
            velocity: translation.to_array(),
        })
    }

    fn world_revolute_sweep(
        &self,
        tree: &Multibody,
        dt: Duration,
    ) -> Result<ArticulatedSweep, PhysicsError> {
        let l = tree.link(1).ok_or(PhysicsError::InvalidJoint)?;
        let body = &self.world.bodies[l.rigid_body_handle()];
        if l.joint.data.locked_axes != JointAxesMask::LOCKED_REVOLUTE_AXES {
            return Err(unsupported("circular sweep requires a revolute coordinate"));
        }
        let root = &self.world.bodies[tree.root().rigid_body_handle()];
        let frame = *root.position() * l.joint.data.local_frame1;
        let pivot = frame.translation;
        let axis = frame.rotation * Vector::X;
        let angular_speed = if self.joints.construction_open() {
            body.angvel().dot(axis)
        } else {
            tree.generalized_velocity()[0]
        };
        let omega = angular_speed * axis;
        let expected = omega.cross(body.center_of_mass() - pivot);
        let round = 256. * f64::EPSILON * (1. + expected.length() + angular_speed.abs());
        if (body.linvel() - expected).length() > round
            || (body.angvel() - omega).length() > round
            || (self.world.gravity != Vector::ZERO
                && axis.cross(self.world.gravity).length()
                    > 64. * f64::EPSILON * self.world.gravity.length())
        {
            return Err(unsupported(
                "native angular and center velocities must agree with a gravity-balanced world pivot",
            ));
        }
        let angle = l.joint.coords()[3];
        let end = angle + dt.as_secs_f64() * angular_speed;
        if let Some(limit) = l.joint.data.limits(JointAxis::AngX)
            && !(angle.min(end) > limit.min + 1e-10 && angle.max(end) < limit.max - 1e-10)
        {
            return Err(unsupported(
                "a swept native revolute would encounter its stop inside this tick",
            ));
        }
        Ok(ArticulatedSweep::Revolution {
            pivot: pivot.to_array(),
            axis: axis.to_array(),
            angular_speed,
        })
    }

    /// All admitted rotational axes are parallel to gravity, so potential
    /// energy is constant over the complete allowed configuration space.
    /// Cross-check this using the actual per-link COM Jacobians and weights.
    fn require_supported_gravity(
        &self,
        native: &Multibody,
        dt: Duration,
    ) -> Result<(), PhysicsError> {
        let incompatible = || {
            unsupported(
                "continuous gravity approach requires a stationary world-supported tree with gravity-parallel passive revolutes",
            )
        };
        if !native
            .links()
            .any(|l| l.is_root() && self.world.bodies[l.rigid_body_handle()].is_fixed())
        {
            return Err(incompatible());
        }
        let tree = native;
        let gravity = self.world.gravity;
        let direction = gravity.try_normalize().ok_or(PhysicsError::InvalidValue)?;
        for link in tree.links() {
            let body = &self.world.bodies[link.rigid_body_handle()];
            if body.is_fixed() {
                continue;
            }
            // Fixed-base assembly introduces roundoff-sized velocities when
            // projecting gravity. Bound actual motion over this interval.
            if body.linvel().length() * dt.as_secs_f64()
                > 64. * f64::EPSILON * (1. + body.center_of_mass().abs().max_element())
            {
                return Err(incompatible());
            }
            let j = tree.body_jacobian(link.link_id());
            let weight = body.mass() * body.gravity_scale() * gravity;
            if !weight.is_finite() {
                return Err(PhysicsError::InvalidValue);
            }
            for i in 0..tree.ndofs() {
                let displacement = Vector::new(j[(0, i)], j[(1, i)], j[(2, i)]);
                let axis = Vector::new(j[(3, i)], j[(4, i)], j[(5, i)]);
                if axis.cross(direction).length() > 64. * f64::EPSILON * axis.length()
                    || displacement.dot(weight).abs()
                        > 128. * f64::EPSILON * displacement.length() * weight.length()
                {
                    return Err(incompatible());
                }
            }
        }
        Ok(())
    }

    /// Build J M⁻¹ Jᵀ for a batch of material points on one floating or
    /// world-grounded reduced tree. A speculative read can drop the transaction.
    /// Active motors, stops, other joint constraints and native rigid contact
    /// require those rows to participate in the same simultaneous solve.
    pub fn prepare_articulated_contacts(
        &mut self,
        sites: &[ArticulatedContactSite],
    ) -> Result<ArticulatedContactTransaction<'_>, PhysicsError> {
        self.prepare_contact_sites(sites, None)
    }

    fn prepare_contact_sites(
        &mut self,
        sites: &[ArticulatedContactSite],
        certified_pair: Option<(ColliderHandle, ColliderHandle)>,
    ) -> Result<ArticulatedContactTransaction<'_>, PhysicsError> {
        let operator = self.frozen_contact_operator(sites, certified_pair, false)?;
        Ok(ArticulatedContactTransaction {
            backend: self,
            operator,
        })
    }
    fn frozen_contact_operator(
        &self,
        sites: &[ArticulatedContactSite],
        certified_pair: Option<(ColliderHandle, ColliderHandle)>,
        allow_unloaded: bool,
    ) -> Result<FrozenOperator, PhysicsError> {
        if sites.is_empty()
            || sites.len() > MAX_ARTICULATED_CONTACT_SITES
            || sites.iter().any(|s| !finite(s.local_point))
        {
            return Err(PhysicsError::InvalidValue);
        }
        let lookup = |id: BodyId| {
            let e = self.bodies.get(&id).ok_or(PhysicsError::MissingBody)?;
            self.world
                .multibody_joints
                .rigid_body_link(e.0)
                .copied()
                .ok_or_else(|| {
                    unsupported("a native reduced tree is required for each contact site")
                })
        };
        let ids: Vec<_> = sites
            .iter()
            .map(|s| lookup(s.body))
            .collect::<Result<_, _>>()?;
        let index = ids[0].multibody;
        if ids.iter().any(|m| m.multibody != index) {
            return Err(unsupported(
                "all articulated contact sites must belong to one tree",
            ));
        }
        let native = self
            .world
            .multibody_joints
            .get_multibody(index)
            .ok_or(PhysicsError::InvalidJoint)?;
        if !(2..=MAX_LINKS).contains(&native.num_links())
            || !native.couplings().is_empty()
            || native.frictions().iter().any(|v| *v != 0.)
        {
            return Err(unsupported(
                "point impulses require a supported connected passive revolute tree",
            ));
        }
        // Prepare using a private snapshot. Query-only reads and rejected
        // impulses preserve even the solver's internal position/mass caches.
        let mut tree = native.clone();
        tree.forward_kinematics(&self.world.bodies, true);
        let n = tree.ndofs();
        if n == 0 || n > MAX_LINKS + 5 {
            return Err(unsupported("unsupported articulated contact dimension"));
        }
        let mut mass = DMatrix::<f64>::from_diagonal(tree.armature());
        let mut links = Vec::with_capacity(tree.num_links());
        for l in tree.links() {
            let h = l.rigid_body_handle();
            let body = &self.world.bodies[h];
            if !body.is_enabled()
                || !body.locked_axes().is_empty()
                || l.joint.kinematic
                || self.world.impulse_joints_with(h).next().is_some()
                || (l.is_root()
                    && !(body.is_dynamic() && l.joint.ndofs() == 6
                        || body.is_fixed() && l.joint.ndofs() == 0))
                || (!l.is_root()
                    && (!body.is_dynamic()
                        || !passive_contact_coordinate(l)
                        || !l.joint.data.motor_axes.is_empty()))
            {
                return Err(unsupported(
                    "articulated point impulses require passive free coordinates and an enabled fixed or floating root",
                ));
            }
            if !l.is_root()
                && let Some(range) = l.joint.data.limits(JointAxis::AngX)
            {
                let q = l.joint.coords()[3];
                if !(q > range.min + 1e-10 && q < range.max - 1e-10) {
                    return Err(unsupported(
                        "an engaged revolute stop requires a simultaneous contact solve",
                    ));
                }
            }
            let j = tree.body_jacobian(l.link_id());
            let j = DMatrix::from_column_slice(6, n, j.as_slice());
            if body.is_dynamic() {
                let props = &body.mass_properties().local_mprops;
                let inertia = props.inv_principal_inertia.recip();
                let frame = *body.rotation() * props.principal_inertia_local_frame;
                let m = body.mass();
                if !m.is_finite() || m <= 0. || !inertia.is_finite() || inertia.min_element() <= 0.
                {
                    return Err(PhysicsError::InvalidValue);
                }
                // Spatial kinetic energy is simplest at the COM, where the
                // link's native translational and rotational Jacobians live.
                for b in 0..n {
                    let v = m * xyz(&j, 0, b);
                    let w = frame * (inertia * (frame.inverse() * xyz(&j, 3, b)));
                    for a in 0..=b {
                        let entry = xyz(&j, 0, a).dot(v) + xyz(&j, 3, a).dot(w);
                        mass[(a, b)] += entry;
                        if a != b {
                            mass[(b, a)] += entry;
                        }
                    }
                }
            }
            links.push((h, j));
        }
        self.require_clear_contact_tree(&tree, certified_pair, allow_unloaded)?;
        let velocity = if self.joints.construction_open() {
            super::reduced::initial_generalized_velocity(&tree, &self.world.bodies)
        } else {
            tree.generalized_velocity().into_owned()
        };
        let mut jacobian = DMatrix::<f64>::zeros(sites.len() * 3, n);
        let mut points = Vec::with_capacity(sites.len());
        for (i, (site, member)) in sites.iter().zip(&ids).enumerate() {
            let (handle, j) = &links[member.id];
            let body = &self.world.bodies[*handle];
            let state = self.state(site.body).ok_or(PhysicsError::MissingBody)?;
            let point = Vector::from_array(state.position)
                + *body.rotation() * Vector::from_array(site.local_point);
            let arm = point - body.center_of_mass();
            for b in 0..n {
                let col = xyz(j, 0, b) + xyz(j, 3, b).cross(arm);
                for a in 0..3 {
                    jacobian[(3 * i + a, b)] = col[a];
                }
            }
            points.push(point.to_array());
        }
        let decomposition = mass.cholesky().ok_or(PhysicsError::InvalidValue)?;
        let weighted = decomposition.solve(&jacobian.transpose());
        let response = &jacobian * &weighted;
        let velocities = &jacobian * &velocity;
        if weighted
            .iter()
            .chain(response.iter())
            .chain(velocities.iter())
            .any(|v| !v.is_finite())
            || points.iter().any(|&p| !finite(p))
        {
            return Err(PhysicsError::InvalidValue);
        }
        let mut blocks = Vec::with_capacity(sites.len() * sites.len());
        for i in 0..sites.len() {
            for j in 0..sites.len() {
                blocks.push(std::array::from_fn(|a| {
                    std::array::from_fn(|b| response[(3 * i + a, 3 * j + b)])
                }));
            }
        }
        let sample = ArticulatedContactSample {
            world_points: points,
            velocities: vectors(&velocities),
            dofs: n,
            links: tree.num_links(),
            blocks,
        };
        Ok(FrozenOperator {
            index,
            sample,
            velocity,
            jacobian,
            weighted,
            links,
        })
    }

    fn require_clear_contact_tree(
        &self,
        tree: &Multibody,
        certified_pair: Option<(ColliderHandle, ColliderHandle)>,
        allow_unloaded: bool,
    ) -> Result<(), PhysicsError> {
        self.check_contact_tree_geometry(tree, certified_pair, allow_unloaded, false)
    }

    // A swept clearance certificate can subsume the external current-pose
    // check. Existing active rows and internal pairs must still be inspected.
    fn check_contact_tree_geometry(
        &self,
        tree: &Multibody,
        certified_pair: Option<(ColliderHandle, ColliderHandle)>,
        allow_unloaded: bool,
        swept_external: bool,
    ) -> Result<(), PhysicsError> {
        self.check_contact_tree_pairs(
            tree,
            certified_pair.as_slice(),
            allow_unloaded,
            swept_external,
        )
    }
    fn check_contact_tree_pairs(
        &self,
        tree: &Multibody,
        certified_pairs: &[(ColliderHandle, ColliderHandle)],
        allow_unloaded: bool,
        swept_external: bool,
    ) -> Result<(), PhysicsError> {
        let world = &self.world;
        for l in tree.links() {
            let bh = l.rigid_body_handle();
            let body = &world.bodies[bh];
            if !body.is_dynamic() {
                continue;
            }
            for &h in body.colliders() {
                let a = &world.colliders[h];
                if !a.is_enabled() || a.is_sensor() {
                    continue;
                }
                if world.narrow_phase.contact_pairs_with(h).any(|p| {
                    if !p.has_any_active_contact() {
                        return false;
                    }
                    let named = certified_pairs.iter().any(|&(a, b)| {
                        (p.collider1 == a && p.collider2 == b)
                            || (p.collider1 == b && p.collider2 == a)
                    });
                    !(allow_unloaded
                        && named
                        && p.manifolds().iter().flat_map(|m| &m.points).all(|point| {
                            let q = &point.data;
                            q.impulse == 0.
                                && q.tangent_impulse.norm_squared() == 0.
                                && q.warmstart_impulse == 0.
                                && q.warmstart_tangent_impulse.norm_squared() == 0.
                                && q.warmstart_twist_impulse == 0.
                                && q.warmstart_tangent_world == Vector::ZERO
                        }))
                }) {
                    return Err(unsupported(
                        "an active native rigid contact requires a simultaneous articulated solve",
                    ));
                }
                // Narrow phase can describe the previous completed step or be
                // empty just after construction. Check actual current poses too.
                let pa =
                    *body.position() * a.position_wrt_parent().ok_or(PhysicsError::InvalidValue)?;
                let ba = a.shape().compute_aabb(&pa).loosened(1e-10);
                let check = |(other, b): (ColliderHandle, &Collider)| {
                    if certified_pairs.contains(&(h, other))
                        || other == h
                        || b.parent() == Some(bh)
                        || !b.is_enabled()
                        || b.is_sensor()
                        || !a.collision_groups().test(b.collision_groups())
                        || !a.solver_groups().test(b.solver_groups())
                        || a.user_data != 0 && a.user_data == b.user_data
                    {
                        return Ok(());
                    }
                    if let Some(other_link) = b
                        .parent()
                        .and_then(|h| world.multibody_joints.rigid_body_link(h))
                        && world
                            .multibody_joints
                            .rigid_body_link(bh)
                            .is_some_and(|m| m.multibody == other_link.multibody)
                        && ((l.parent_id() == Some(other_link.id)
                            && !l.joint.data.contacts_enabled)
                            || tree.link(other_link.id).is_some_and(|q| {
                                q.parent_id() == Some(l.link_id()) && !q.joint.data.contacts_enabled
                            }))
                    {
                        return Ok(());
                    }
                    let pb = if let Some(parent) = b.parent() {
                        *world.bodies[parent].position()
                            * b.position_wrt_parent().ok_or(PhysicsError::InvalidValue)?
                    } else {
                        *b.position()
                    };
                    if ba.intersects(&b.shape().compute_aabb(&pb)) {
                        let d = query::distance(&pa, a.shape(), &pb, b.shape())
                            .map_err(|_| {
                                unsupported(
                                    "unsupported native geometry at an articulated contact site",
                                )
                            })?
                            .distance;
                        if !d.is_finite() || d <= 1e-10 {
                            return Err(unsupported(
                                "touching native colliders require a simultaneous articulated solve",
                            ));
                        }
                    }
                    Ok(())
                };
                if swept_external {
                    tree.links()
                        .flat_map(|l| world.bodies[l.rigid_body_handle()].colliders())
                        .map(|&h| (h, &world.colliders[h]))
                        .try_for_each(check)?;
                } else {
                    world.colliders.iter().try_for_each(check)?;
                }
            }
        }
        Ok(())
    }
}

impl ArticulatedContactTransaction<'_> {
    pub fn sample(&self) -> &ArticulatedContactSample {
        &self.operator.sample
    }

    /// Apply world impulses on the links at the sampled points, in N s.
    /// Evaluates all resulting native link velocities before mutating any.
    pub fn commit(self, impulses: &[V]) -> Result<ArticulatedImpulseReport, PhysicsError> {
        if impulses.len() != self.operator.sample.world_points.len()
            || impulses.iter().any(|&j| !finite(j))
        {
            return Err(PhysicsError::InvalidValue);
        }
        let count = impulses.len();
        let impulse = DVector::from_iterator(3 * count, impulses.iter().flatten().copied());
        let next = &self.operator.velocity + &self.operator.weighted * &impulse;
        let after = &self.operator.jacobian * &next;
        let before = &self.operator.jacobian * &self.operator.velocity;
        let change = 0.5 * impulse.dot(&(&after + before));
        if !change.is_finite() || next.iter().chain(after.iter()).any(|v| !v.is_finite()) {
            return Err(PhysicsError::InvalidValue);
        }
        let mut states = Vec::with_capacity(self.operator.links.len());
        for (body, j) in &self.operator.links {
            let v = j * &next;
            if v.iter().any(|x| !x.is_finite()) {
                return Err(PhysicsError::InvalidValue);
            }
            states.push((
                *body,
                Vector::new(v[0], v[1], v[2]),
                Vector::new(v[3], v[4], v[5]),
            ));
        }
        if impulses.iter().any(|&j| j != [0.; 3]) {
            let world = &mut self.backend.world;
            let tree = world
                .multibody_joints
                .get_multibody_mut(self.operator.index)
                .expect("exclusively borrowed registered tree");
            // A newly constructed world anchor still occupies the six default
            // root slots before its first native FK. Preparation already ran
            // this deterministic transition on the private copy. Install it
            // only after the entire accepted impulse and every link are checked.
            if tree.ndofs() != next.len() {
                tree.forward_kinematics(&world.bodies, true);
            }
            debug_assert_eq!(tree.ndofs(), next.len());
            tree.generalized_velocity_mut().copy_from(&next);
            for (h, v, w) in states {
                let b = &mut world.bodies[h];
                if b.is_dynamic() {
                    b.set_linvel(v, true);
                    b.set_angvel(w, true);
                }
            }
        }
        Ok(ArticulatedImpulseReport {
            before: self.operator.sample,
            after_velocities: vectors(&after),
            kinetic_energy_change: change,
        })
    }
}
