// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Finite-mass centered rigid receivers in the shared deformable substep.
use super::{Boundary, BoundaryImpulse, PreparedBoundary, SoftBody, XpbdError};
use super::{PrincipalRigidMotion, tensor::TensorMotion};
use glam::{DQuat, DVec3};
use std::time::Duration;
mod shell_retry;
use shell_retry::ShellRetry;

/// World-frame mobility of a free sphere with isotropic inertia about its COM.
#[derive(Clone, Copy, Debug)]
pub struct SphereMobility {
    /// In kg⁻¹.
    pub inverse_mass: f64,
    /// In (kg m²)⁻¹.
    pub inverse_inertia: f64,
}

/// Mobility about the receiver COM. A principal inertia tensor needs the
/// native orientation trajectory to express its mobility in world axes.
#[derive(Clone, Copy, Debug)]
pub enum RigidMobility {
    Isotropic(SphereMobility),
    Principal(PrincipalRigidMotion),
    /// Centered box moving between native link poses. Its contacts affect the material
    /// and accumulate a world-frame wrench for the native owner. No free-body correction
    /// is returned. The boundary supplies chord velocity and ending angular velocity.
    PrescribedCuboid {
        /// Normal indentation in metres per newton for each contacting particle.
        /// Zero selects hard contact.
        normal_compliance: f64,
        start_orientation: [f64; 4],
        end_orientation: [f64; 4],
        start_angular_velocity: [f64; 3],
    },
    /// A free capsule under a known constant world force. The material and
    /// native COM advance on the same integer clock when they exchange load.
    ForceAware {
        rotation: PrincipalRigidMotion,
        start_velocity: [f64; 3],
        acceleration: [f64; 3],
    },
}
impl From<SphereMobility> for RigidMobility {
    fn from(value: SphereMobility) -> Self {
        Self::Isotropic(value)
    }
}
/// Corrections to the independently integrated rigid end state, in world axes.
#[derive(Clone, Copy, Debug)]
pub struct ReceiverMotion {
    pub translation_correction: [f64; 3],
    /// Left-multiply onto the uncoupled end orientation. Components are xyzw.
    pub rotation_correction: [f64; 4],
    pub linear_velocity_correction: [f64; 3],
    pub angular_velocity_correction: [f64; 3],
}

#[derive(Clone)]
pub(super) struct Receiver {
    forcing: Option<LinearForcing>,
    received_impulse: bool,
    pub inverse_mass: f64,
    pub start_offset: DVec3,
    pub end_offset: DVec3,
    pub linear: DVec3,
    spin: Spin,
    pub end_time: f64,
    pub isolated_particle: bool,
    angular_path: f64,
}

#[derive(Clone)]
struct LinearForcing {
    start_velocity: DVec3,
    acceleration: DVec3,
    native_chord: DVec3,
    end_velocity: DVec3,
}
#[derive(Clone)]
enum Spin {
    Isotropic {
        inverse_inertia: f64,
        base: DVec3,
        change: DVec3,
        rotation: DQuat,
        base_rotation: DQuat,
    },
    Principal(Box<TensorMotion>),
}

impl Receiver {
    pub(super) fn new(
        mobility: RigidMobility,
        base: DVec3,
        dt: f64,
        isolated_particle: bool,
        native_chord: DVec3,
    ) -> Result<Self, XpbdError> {
        let positive = |w: f64| w.is_finite() && w > 0. && w.recip().is_finite();
        let mut forcing = None;
        if let RigidMobility::ForceAware {
            start_velocity,
            acceleration,
            ..
        } = mobility
        {
            let v = DVec3::from(start_velocity);
            let a = DVec3::from(acceleration);
            let end = v + dt * a;
            if !v.is_finite() || !a.is_finite() || !end.is_finite() {
                return Err(XpbdError::InvalidBoundary);
            }
            if a != DVec3::ZERO {
                forcing = Some(LinearForcing {
                    start_velocity: v,
                    acceleration: a,
                    native_chord,
                    end_velocity: end,
                });
            }
        }
        let (inverse_mass, spin) = match mobility {
            RigidMobility::Isotropic(m) if positive(m.inverse_inertia) => (
                m.inverse_mass,
                Spin::Isotropic {
                    inverse_inertia: m.inverse_inertia,
                    base,
                    change: DVec3::ZERO,
                    rotation: DQuat::IDENTITY,
                    base_rotation: DQuat::IDENTITY,
                },
            ),
            RigidMobility::PrescribedCuboid {
                normal_compliance: _,
                start_orientation,
                end_orientation,
                start_angular_velocity,
            } => (
                0.,
                Spin::Principal(Box::new(TensorMotion::prescribed(
                    start_orientation,
                    end_orientation,
                    start_angular_velocity,
                    base,
                    dt,
                )?)),
            ),
            RigidMobility::Isotropic(_) => return Err(XpbdError::InvalidBoundary),
            RigidMobility::Principal(m) | RigidMobility::ForceAware { rotation: m, .. } => (
                m.inverse_mass,
                Spin::Principal(Box::new(TensorMotion::new(m, base, dt)?)),
            ),
        };
        if !matches!(mobility, RigidMobility::PrescribedCuboid { .. }) && !positive(inverse_mass) {
            return Err(XpbdError::InvalidBoundary);
        }
        Ok(Self {
            inverse_mass,
            spin,
            forcing,
            received_impulse: false,
            start_offset: DVec3::ZERO,
            end_offset: DVec3::ZERO,
            linear: DVec3::ZERO,
            end_time: 0.,
            isolated_particle,
            angular_path: 0.,
        })
    }

    fn begin_translation(&mut self, velocity: &mut DVec3, t0: f64, t1: f64) {
        if let Some(f) = &self.forcing {
            let next = f.start_velocity + t1 * f.acceleration;
            // start_offset/end_offset were expressed relative to the previous
            // affine path. Shift its origin term to preserve the moving COM at t0.
            self.end_offset += t0 * (*velocity - next);
            *velocity = next;
        }
    }
    pub(super) fn begin(&mut self, h: f64, end_time: f64, oriented: bool) -> Result<(), XpbdError> {
        self.start_offset = self.end_offset;
        self.end_offset += h * self.linear;
        self.end_time = end_time;
        let initial_spin = if oriented {
            self.angular_velocity().length()
        } else {
            0.
        };
        match &mut self.spin {
            Spin::Principal(tensor) => tensor.advance(h, end_time),
            Spin::Isotropic {
                base,
                change,
                rotation,
                base_rotation,
                ..
            } => {
                let actual_angle = h * (*base + *change);
                let base_angle = h * *base;
                if !actual_angle.length_squared().is_finite()
                    || !base_angle.length_squared().is_finite()
                {
                    return Err(XpbdError::Numerical);
                }
                *rotation = (DQuat::from_scaled_axis(actual_angle) * *rotation).normalize();
                *base_rotation = (DQuat::from_scaled_axis(base_angle) * *base_rotation).normalize();
            }
        }
        if oriented {
            self.angular_path = h * initial_spin.max(self.angular_velocity().length());
            if self.angular_path >= std::f64::consts::PI {
                return Err(XpbdError::UnresolvedSweep);
            }
        }
        Ok(())
    }
    fn finish(&self) -> Result<(), XpbdError> {
        if self.end_offset.is_finite()
            && self.linear.is_finite()
            && self.angular_velocity().is_finite()
            && self.rotation_correction().is_finite()
        {
            Ok(())
        } else {
            Err(XpbdError::Numerical)
        }
    }
    pub fn can_rotate(&self) -> bool {
        matches!(&self.spin, Spin::Principal(t) if t.can_rotate())
    }
    pub fn angular_velocity(&self) -> DVec3 {
        match &self.spin {
            Spin::Principal(t) => t.spin(),
            Spin::Isotropic { base, change, .. } => *base + *change,
        }
    }
    fn rotation_correction(&self) -> DQuat {
        match &self.spin {
            Spin::Principal(t) => t.rotation_correction(),
            Spin::Isotropic {
                rotation,
                base_rotation,
                ..
            } if rotation != base_rotation => (*rotation * base_rotation.conjugate()).normalize(),
            Spin::Isotropic { .. } => DQuat::IDENTITY,
        }
    }
    pub fn directional_inverse_mass(
        &self,
        pair_inverse_mass: f64,
        arm: DVec3,
        direction: DVec3,
    ) -> Result<f64, XpbdError> {
        let axis = arm.cross(direction);
        let inverse = pair_inverse_mass
            + match &self.spin {
                Spin::Principal(t) => axis.dot(t.inverse_inertia(axis)),
                Spin::Isotropic {
                    inverse_inertia, ..
                } => inverse_inertia * axis.length_squared(),
            };
        if inverse.is_finite() && inverse > 0. {
            Ok(inverse)
        } else {
            Err(XpbdError::Numerical)
        }
    }
    pub fn friction(
        &self,
        arm: DVec3,
        normal: DVec3,
        tangent: DVec3,
        slip: f64,
        pair_inverse_mass: f64,
        cap: f64,
    ) -> Result<DVec3, XpbdError> {
        match &self.spin {
            Spin::Principal(t) => t.friction(arm, normal, tangent, pair_inverse_mass, cap),
            Spin::Isotropic { .. } => {
                let direction = tangent / slip;
                let inverse = self.directional_inverse_mass(pair_inverse_mass, arm, direction)?;
                Ok(-direction * cap.min(slip / inverse))
            }
        }
    }
    pub fn inverse_inertia(&self, torque: DVec3) -> DVec3 {
        match &self.spin {
            Spin::Principal(t) => t.inverse_inertia(torque),
            Spin::Isotropic {
                inverse_inertia, ..
            } => *inverse_inertia * torque,
        }
    }
    pub fn inverse_mass_at(
        &self,
        orientation: DQuat,
        translation: f64,
        arm: DVec3,
        direction: DVec3,
    ) -> Result<f64, XpbdError> {
        let Spin::Principal(t) = &self.spin else {
            return Err(XpbdError::InvalidBoundary);
        };
        let axis = arm.cross(direction);
        let w = translation + axis.dot(t.inverse_at(orientation, axis));
        if w.is_finite() && w > 0. {
            Ok(w)
        } else {
            Err(XpbdError::Numerical)
        }
    }
    pub fn orientations(&self) -> Option<(DQuat, DQuat)> {
        match &self.spin {
            Spin::Principal(t) => Some(t.orientations()),
            _ => None,
        }
    }
    pub fn project_oriented(
        &mut self,
        arm: DVec3,
        impulse: DVec3,
        h: f64,
    ) -> Result<(), XpbdError> {
        let Spin::Principal(t) = &mut self.spin else {
            return Err(XpbdError::InvalidBoundary);
        };
        if self.inverse_mass == 0. {
            return Ok(());
        }
        let delta = h * t.inverse_inertia(arm.cross(impulse));
        self.angular_path += delta.length();
        if self.angular_path >= std::f64::consts::PI {
            return Err(XpbdError::UnresolvedSweep);
        }
        self.project_normal(arm, impulse, h);
        if let Spin::Principal(t) = &mut self.spin {
            t.rotate_projection(delta)?;
        }
        Ok(())
    }
    pub fn project_skin_tangent(
        &mut self,
        arm: DVec3,
        impulse: DVec3,
        h: f64,
    ) -> Result<(), XpbdError> {
        let delta = h * self.inverse_inertia(arm.cross(impulse));
        if !delta.length_squared().is_finite() {
            return Err(XpbdError::Numerical);
        }
        self.project_normal(arm, impulse, h);
        match &mut self.spin {
            Spin::Principal(t) => t.rotate_projection(delta),
            Spin::Isotropic { rotation, .. } => {
                *rotation = (DQuat::from_scaled_axis(delta) * *rotation).normalize();
                Ok(())
            }
        }
    }
    pub fn project_normal(&mut self, start_arm: DVec3, impulse: DVec3, h: f64) {
        // The positional impulse contributes x_start × J to the discrete
        // orbital momentum. Its remaining velocity-level impulse acts at x_end.
        // Balance both during the solve. The intermediate spin is a consequence
        // of that splitting; only the reconciled spin advances orientation.
        self.apply_velocity_impulse(start_arm, impulse);
        self.end_offset += h * self.inverse_mass * impulse;
    }
    /// Project one simultaneous material manifold through its summed world
    /// linear and angular impulses, evaluating inertia at a common pose.
    pub fn project_material_wrench(
        &mut self,
        linear: DVec3,
        torque: DVec3,
        h: f64,
    ) -> Result<(), XpbdError> {
        self.received_impulse |= linear != DVec3::ZERO || torque != DVec3::ZERO;
        let delta = h * self.inverse_inertia(torque);
        self.angular_path += delta.length();
        if self.angular_path >= std::f64::consts::PI {
            return Err(XpbdError::UnresolvedSweep);
        }
        let Spin::Principal(t) = &mut self.spin else {
            return Err(XpbdError::InvalidBoundary);
        };
        self.linear += self.inverse_mass * linear;
        self.end_offset += h * self.inverse_mass * linear;
        t.add_torque_impulse(torque);
        t.rotate_projection(delta)
    }
    pub fn apply_velocity_impulse(&mut self, arm: DVec3, impulse: DVec3) {
        if self.inverse_mass == 0. {
            return;
        }
        self.received_impulse |= impulse != DVec3::ZERO;
        self.linear += self.inverse_mass * impulse;
        let torque = arm.cross(impulse);
        match &mut self.spin {
            Spin::Principal(t) => t.add_torque_impulse(torque),
            Spin::Isotropic {
                inverse_inertia,
                change,
                ..
            } => *change += *inverse_inertia * torque,
        }
    }
    fn motion(&self) -> ReceiverMotion {
        let angular = match &self.spin {
            Spin::Principal(t) => t.spin_correction(),
            Spin::Isotropic { change, .. } => *change,
        };
        let translation = match &self.forcing {
            Some(f) if self.received_impulse => {
                self.end_offset + self.end_time * (f.end_velocity - f.native_chord)
            }
            Some(_) => DVec3::ZERO,
            None => self.end_offset,
        };
        ReceiverMotion {
            translation_correction: translation.to_array(),
            rotation_correction: self.rotation_correction().to_array(),
            linear_velocity_correction: self.linear.to_array(),
            angular_velocity_correction: angular.to_array(),
        }
    }
}

/// Advance bodies and sphere receivers on a common integer substep clock. Every
/// contact observes the accumulated response of the same receiver. The supplied
/// boundary trajectory is the uncoupled motion over this interval. Return one
/// optional rigid correction per boundary and retain per-body reaction impulses.
/// A refused solve leaves every body's published state and reactions unchanged.
pub fn step_coupled(
    bodies: &mut [&mut SoftBody],
    dt: Duration,
    boundaries: &[Boundary],
    mobility: &[Option<SphereMobility>],
) -> Result<Vec<Option<ReceiverMotion>>, XpbdError> {
    solve(bodies, dt, boundaries, mobility, &[])
}

/// Advance finite centered spheres, cuboids and capsules with full principal
/// inertia. Oriented receivers require a principal motion even for equal moments.
/// A dynamic capsule retries an unresolved coupled sweep on a finer integer
/// clock, up to five doublings and at most 1,024 retry substeps. A finite
/// self-contact shell pressed by compliant prescribed jaws retries unresolved
/// surface contact on the same bounded clock.
/// For one rotating cuboid, up to four simultaneous frictionless points on a
/// free, initially unloaded, zero-gravity distance cable can collide between
/// sampled poses. Normal impulses and up to twelve rigid distance rows are
/// solved together at impact. Unsupported transient interactions refuse before
/// publishing any body state. Sustained contacts use ordinary XPBD projection.
pub fn step_coupled_rigid(
    bodies: &mut [&mut SoftBody],
    dt: Duration,
    boundaries: &[Boundary],
    mobility: &[Option<RigidMobility>],
) -> Result<Vec<Option<ReceiverMotion>>, XpbdError> {
    solve(bodies, dt, boundaries, mobility, &[])
}
/// Advance with optional normal viscous damping (N·s/m) on compliant prescribed
/// cuboids. Each entry names one distinct boundary and its positive damping.
/// The spring and dashpot act in parallel while compressed and exert no tension.
pub fn step_coupled_damped_pads(
    bodies: &mut [&mut SoftBody],
    dt: Duration,
    boundaries: &[Boundary],
    mobility: &[Option<RigidMobility>],
    damping: &[(usize, f64)],
) -> Result<Vec<Option<ReceiverMotion>>, XpbdError> {
    solve(bodies, dt, boundaries, mobility, damping)
}

fn solve<M: Copy + Into<RigidMobility>>(
    bodies: &mut [&mut SoftBody],
    dt: Duration,
    boundaries: &[Boundary],
    mobility: &[Option<M>],
    damping: &[(usize, f64)],
) -> Result<Vec<Option<ReceiverMotion>>, XpbdError> {
    if boundaries.len() != mobility.len() {
        return Err(XpbdError::InvalidBoundary);
    }
    let finite_material = bodies.iter().any(|b| b.uses_capsule_material_contact())
        && boundaries
            .iter()
            .any(|b| matches!(b, Boundary::DynamicCapsule { .. }));
    if finite_material && bodies.len() != 1 {
        return Err(XpbdError::InvalidConfig);
    }
    let count = bodies.first().map_or(1, |body| body.config.substeps);
    if bodies.iter().any(|body| body.config.substeps != count) {
        return Err(XpbdError::InvalidConfig);
    }
    let count = bodies
        .iter()
        .try_fold(count, |n, body| -> Result<u32, XpbdError> {
            Ok(n.max(body.structural_substeps(dt)?))
        })?;
    // A reattempt starts from committed particles and fresh rigid/reaction state.
    // Replay the same prescribed jaw trajectory if a coarse shell interaction
    // cannot satisfy both continuous separation and passive normal response.
    let compliant_shell = bodies.iter().any(|body| body.surface.is_some())
        && mobility.iter().any(|m| {
            matches!(m.map(Into::into), Some(RigidMobility::PrescribedCuboid {normal_compliance, ..}) if normal_compliance > 0.)
        });
    let capsule = boundaries
        .iter()
        .any(|b| matches!(b, Boundary::DynamicCapsule { .. }));
    let maximum = if capsule || compliant_shell {
        count
            .saturating_mul(if finite_material { 128 } else { 32 })
            .min(1024)
            .max(count)
    } else {
        count
    };
    let mut steps = count;
    let mut refinements = 0;
    let mut retry_material = false;
    loop {
        for body in bodies.iter_mut() {
            if let Some(surface) = &mut body.surface {
                surface.subdivide_on_budget = false;
                surface.retry_plane = refinements != 0
                    && boundaries
                        .iter()
                        .any(|b| matches!(b, Boundary::Plane { .. }));
            }
            if let Some(material) = &mut body.capsule_material {
                material.retry_manifold = retry_material;
            }
        }
        let result = solve_at_resolution(
            bodies,
            dt,
            boundaries,
            mobility,
            damping,
            u128::from(steps),
            None,
        );
        for body in bodies.iter_mut() {
            if let Some(surface) = &mut body.surface {
                surface.retry_plane = false;
            }
            if let Some(material) = &mut body.capsule_material {
                material.retry_manifold = false;
            }
        }
        if result.is_ok() {
            for body in bodies.iter_mut() {
                body.sweep_refinements = refinements;
                if let Some(surface) = &mut body.surface {
                    surface.local_intervals = None;
                }
            }
            return result;
        }
        let capsule_sweep = capsule && matches!(result, Err(XpbdError::UnresolvedSweep));
        let shell_contact = compliant_shell
            && (matches!(result, Err(XpbdError::UnresolvedSurface))
                || (matches!(result, Err(XpbdError::SurfaceBudget))
                    && bodies
                        .iter()
                        .any(|b| b.surface.as_ref().is_some_and(|s| s.subdivide_on_budget))));
        if !capsule_sweep && !shell_contact {
            return result;
        }
        if steps >= maximum {
            if shell_contact
                && count < maximum
                && bodies.len() == 1
                && boundaries
                    .iter()
                    .any(|b| matches!(b, Boundary::Plane { .. }))
                && boundaries.iter().zip(mobility).all(|(b, m)| {
                    matches!(
                        (b, m.map(Into::into)),
                        (Boundary::Plane { .. }, None)
                            | (
                                Boundary::Cuboid { .. },
                                Some(RigidMobility::PrescribedCuboid { .. })
                            )
                    )
                })
            {
                let body = &mut *bodies[0];
                if !body
                    .surface
                    .as_mut()
                    .unwrap()
                    .starts_clear(&body.particles, 2. * body.config.particle_radius)?
                {
                    return result;
                }
                return shell_retry::solve(
                    bodies, dt, boundaries, mobility, damping, count, maximum,
                );
            }
            if capsule_sweep && finite_material && !retry_material {
                // The ordinary coupled sweep exhausted its grid. Reconstruct
                // from the committed input with the same physical tolerances.
                steps = count;
                refinements = 0;
                retry_material = true;
                continue;
            }
            return result;
        }
        let next = steps.saturating_mul(2).min(maximum);
        if u128::from(next) > dt.as_nanos() {
            return result;
        }
        steps = next;
        refinements += 1;
    }
}

fn solve_at_resolution<M: Copy + Into<RigidMobility>>(
    bodies: &mut [&mut SoftBody],
    dt: Duration,
    boundaries: &[Boundary],
    mobility: &[Option<M>],
    damping: &[(usize, f64)],
    count: u128,
    mut retry: Option<&mut ShellRetry>,
) -> Result<Vec<Option<ReceiverMotion>>, XpbdError> {
    let ticks = dt.as_nanos();
    if ticks != 0 && ticks < count {
        return Err(XpbdError::InvalidStep);
    }
    // The original single-particle swept impulse needs no active-set solve.
    // Multiple points instead resolve their shared receiver at one event time.
    if bodies.iter().any(|b| b.uses_pad_surface_contact()) {
        if bodies.len() != 1 {
            return Err(XpbdError::InvalidConfig);
        }
        if boundaries
            .iter()
            .filter(|b| matches!(b, Boundary::Sphere { .. }))
            .count()
            > super::MAX_SURFACE_PADS
        {
            return Err(XpbdError::SurfaceBudget);
        }
        if boundaries.iter().zip(mobility).any(|(b, m)| match b {
            Boundary::Sphere { .. } => {
                !matches!(m.map(Into::into), Some(RigidMobility::Isotropic(_)))
            }
            Boundary::Cuboid { .. }
            | Boundary::Capsule { .. }
            | Boundary::DynamicCapsule { .. } => true,
            Boundary::StaticCuboid { .. }
            | Boundary::TranslatingCuboid { .. }
            | Boundary::Plane { .. } => false,
        }) {
            return Err(XpbdError::InvalidBoundary);
        }
    }
    let isolated_particle = matches!(
        boundaries,
        [Boundary::Cuboid { .. } | Boundary::DynamicCapsule { .. }]
    ) && matches!(bodies,[body] if body.particles.len()==1 && body.constraints.is_empty()
            && body.bending.is_empty() && !body.config.self_contact && body.config.friction==0.);
    let mut prepared = boundaries
        .iter()
        .zip(mobility)
        .map(|(&b, &m)| {
            let mut prepared = PreparedBoundary::new(b)?;
            if let Some(m) = m {
                if matches!(m.into(), RigidMobility::ForceAware { .. })
                    && !(matches!(b, Boundary::DynamicCapsule { .. })
                        && bodies.len() == 1
                        && bodies[0].uses_capsule_material_contact())
                {
                    return Err(XpbdError::InvalidBoundary);
                }
                if let RigidMobility::PrescribedCuboid {
                    normal_compliance, ..
                } = m.into()
                {
                    if !normal_compliance.is_finite() || normal_compliance < 0. {
                        return Err(XpbdError::InvalidBoundary);
                    }
                    prepared.normal_compliance = normal_compliance;
                }
                let angular_velocity = match b {
                    Boundary::Sphere {
                        angular_velocity, ..
                    } if !matches!(m.into(), RigidMobility::PrescribedCuboid { .. }) => {
                        angular_velocity
                    }
                    Boundary::Cuboid {
                        angular_velocity, ..
                    } if matches!(m.into(), RigidMobility::PrescribedCuboid { .. }) => {
                        angular_velocity
                    }
                    Boundary::Cuboid {
                        angular_velocity, ..
                    }
                    | Boundary::DynamicCapsule {
                        angular_velocity, ..
                    } if matches!(m.into(), RigidMobility::Principal(_)) => angular_velocity,
                    Boundary::DynamicCapsule {
                        angular_velocity, ..
                    } if matches!(m.into(), RigidMobility::ForceAware { .. })
                        && bodies.len() == 1
                        && bodies[0].uses_capsule_material_contact() =>
                    {
                        angular_velocity
                    }
                    _ => return Err(XpbdError::InvalidBoundary),
                };
                prepared.receiver = Some(Receiver::new(
                    m.into(),
                    DVec3::from(angular_velocity),
                    dt.as_secs_f64(),
                    isolated_particle,
                    prepared.velocity,
                )?);
            }
            if matches!(b, Boundary::Cuboid { .. } | Boundary::DynamicCapsule { .. })
                && prepared.receiver.is_none()
            {
                return Err(XpbdError::InvalidBoundary);
            }
            Ok(prepared)
        })
        .collect::<Result<Vec<_>, XpbdError>>()?;
    for &(index, coefficient) in damping {
        let pad = prepared.get_mut(index).ok_or(XpbdError::InvalidBoundary)?;
        let relaxation = pad.normal_compliance * coefficient;
        if !(relaxation.is_finite() && relaxation > 0.) || pad.normal_relaxation != 0. {
            return Err(XpbdError::InvalidBoundary);
        }
        pad.normal_relaxation = relaxation;
    }
    let empty = (0..boundaries.len())
        .map(|boundary| BoundaryImpulse {
            boundary,
            ..Default::default()
        })
        .collect::<Vec<_>>();
    let mut reactions = vec![empty; bodies.len()];
    for body in bodies.iter_mut() {
        body.begin_self_contact();
    }
    if ticks != 0 {
        for body in bodies.iter_mut() {
            body.scratch.copy_from_slice(&body.particles);
        }
        let iterations = bodies
            .iter()
            .map(|body| body.config.iterations)
            .max()
            .unwrap_or(0);
        let mut elapsed = 0;
        let mut limit = ticks;
        let mut events = 0;
        for i in 0..count {
            let substep = ticks / count + u128::from(i < ticks % count);
            let mut time = elapsed;
            let end = elapsed + substep;
            while time < end {
                let tail = (end - time).min(limit);
                let hit = if isolated_particle && events > 0 {
                    // After its first impact the point follows the refined clock.
                    // Resting contact must not repeatedly split at roundoff-sized gaps.
                    None
                } else {
                    super::impact_group::locate(bodies, &prepared, time, tail)?
                };
                if let Some(hit) = hit {
                    events += 1;
                    if events > 8 {
                        return Err(XpbdError::UnresolvedSweep);
                    }
                    substep_interval(
                        bodies,
                        &mut prepared,
                        &mut reactions,
                        time,
                        hit.head_ns,
                        iterations,
                        true,
                    )?;
                    time += hit.head_ns;
                    super::impact_group::resolve(bodies, &mut prepared[0], &mut reactions, &hit)?;
                    limit =
                        super::impact_group::remaining_step(bodies, &prepared[0], ticks - time)?;
                } else {
                    if let Some(local) = retry.as_deref_mut() {
                        local.advance(
                            bodies[0],
                            &mut prepared,
                            &mut reactions,
                            time..time + tail,
                            iterations,
                            0,
                        )?;
                    } else {
                        substep_interval(
                            bodies,
                            &mut prepared,
                            &mut reactions,
                            time,
                            tail,
                            iterations,
                            false,
                        )?;
                    }
                    time += tail;
                }
            }
            elapsed += substep;
        }
    }
    for (body, impulses) in bodies.iter_mut().zip(reactions) {
        if ticks != 0 {
            std::mem::swap(&mut body.particles, &mut body.scratch);
        }
        body.reactions = impulses;
        body.path_substeps = 0;
        body.publish_self_contact();
    }
    Ok(prepared
        .iter()
        .map(|b| {
            b.receiver
                .as_ref()
                .filter(|r| r.inverse_mass > 0.)
                .map(Receiver::motion)
        })
        .collect())
}

fn substep_interval(
    bodies: &mut [&mut SoftBody],
    prepared: &mut [PreparedBoundary],
    reactions: &mut [Vec<BoundaryImpulse>],
    elapsed: u128,
    substep: u128,
    iterations: u32,
    free_flight: bool,
) -> Result<(), XpbdError> {
    let h = substep as f64 * 1e-9;
    let end_time = (elapsed + substep) as f64 * 1e-9;
    for b in prepared.iter_mut() {
        let oriented = b.is_oriented();
        if let Some(r) = &mut b.receiver {
            r.begin_translation(&mut b.velocity, elapsed as f64 * 1e-9, end_time);
            r.begin(h, end_time, oriented)?;
        }
    }
    for body in bodies.iter_mut() {
        body.predict(h, prepared)?;
    }
    let contact_boundaries = if free_flight { &mut [] } else { &mut *prepared };
    // A shared receiver can move during the projection of another body.
    // Alternate bodies between sweeps, retaining each body's iteration budget.
    for i in 0..iterations {
        for (body, impulses) in bodies.iter_mut().zip(reactions.iter_mut()) {
            if i < body.config.iterations {
                body.project_iteration(elapsed, substep, contact_boundaries, impulses)?;
            }
        }
    }
    for (body, impulses) in bodies.iter_mut().zip(reactions.iter_mut()) {
        body.finish(h, end_time, contact_boundaries, impulses)?;
    }
    if prepared.len() >= 2
        && reactions.iter().flatten().any(|j| {
            prepared.get(j.boundary).is_some_and(|b| {
                matches!(
                    b.oriented_shape(),
                    Some(crate::oriented::OrientedShape::Capsule(_))
                )
            }) && (j.linear != [0.; 3] || j.angular != [0.; 3])
        })
    {
        crate::capsule_material::check_receiver_pair(
            prepared,
            crate::collision::Interval {
                h,
                t0: elapsed as f64 * 1e-9,
                t1: end_time,
                radius: 0.,
            },
        )?;
    }
    for b in prepared.iter_mut() {
        if let Some(r) = &mut b.receiver {
            r.finish()?;
        }
        if !free_flight && b.is_oriented() {
            for body in bodies.iter() {
                b.check_oriented_endpoint(&body.scratch, body.config.particle_radius)?;
            }
        }
    }
    Ok(())
}
