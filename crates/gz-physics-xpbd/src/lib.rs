// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Extended position-based dynamics (XPBD) for cables and rectangular cloth.
//! Units are SI. Constraint projection follows Macklin et al., XPBD (2016).
//! Each distance constraint carries its own tension/compression multiplier
//! across solver iterations and starts a fresh multiplier each substep.
pub mod articulated_impact;
#[cfg(feature = "one-way-checkpoint")]
mod one_way_checkpoint;
#[cfg(feature = "one-way-checkpoint")]
pub use one_way_checkpoint::{MAX_ONE_WAY_PARTICLES, OneWayConstruction, OneWayState};
#[cfg(feature = "prescribed-translation")]
mod translation_boundary;
#[cfg(feature = "prescribed-translation")]
pub use translation_boundary::TranslationBoundary;
mod boundary_path;
pub use boundary_path::{BoundaryPath, DEFAULT_PATH_TOLERANCE};
mod bending;
pub use bending::DihedralConstraint;
mod capsule;
mod capsule_material;
mod coulomb_manifold;
pub use capsule_material::{
    CapsuleContactSample, CapsuleFrictionLoad, CapsuleMaterialLoad, CapsuleRegion, MaterialFeature,
};
mod collision;
mod coupled;
mod cuboid;
mod energy;
mod static_box_faces;
pub use energy::MaterialEnergy;
mod impact_group;
mod material;
mod momentum;
mod oriented;
mod pad;
mod pad_loads;
mod pad_samples;
pub use pad_loads::ParticlePadImpulse;
pub use pad_samples::PadSample;
/// Maximum simultaneous finite spherical pads on a triangulated shell.
pub const MAX_SURFACE_PADS: usize = 4;
mod surface;
pub use surface::SelfContactLoad;
mod tensor;
pub use coupled::{
    ReceiverMotion, RigidMobility, SphereMobility, step_coupled, step_coupled_rigid,
};
pub use tensor::PrincipalRigidMotion;
/// Original name for finite sphere callers.
pub type PrincipalSphereMotion = PrincipalRigidMotion;
/// Original name for finite sphere callers.
pub type RigidSphereMobility = RigidMobility;
mod topology;
use collision::{Contacts, PreparedBoundary, SelfContact};
use glam::DVec3;
use std::{collections::HashSet, time::Duration};

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(
    feature = "one-way-checkpoint",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct Particle {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    /// In kg⁻¹. Zero prescribes the particle's given world velocity.
    pub inverse_mass: f64,
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(
    feature = "one-way-checkpoint",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct DistanceConstraint {
    pub a: usize,
    pub b: usize,
    pub rest_length: f64,
    /// In metres per newton. Zero enforces an inextensible link.
    pub compliance: f64,
}

#[derive(Clone, Copy, Debug)]
#[cfg_attr(
    feature = "one-way-checkpoint",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct SolverConfig {
    pub gravity: [f64; 3],
    pub substeps: u32,
    pub iterations: u32,
    pub particle_radius: f64,
    pub self_contact: bool,
    /// Coulomb coefficient against external boundaries.
    pub friction: f64,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            gravity: [0.0, 0.0, -9.81],
            substeps: 4,
            iterations: 8,
            particle_radius: 0.01,
            self_contact: false,
            friction: 0.3,
        }
    }
}

/// Geometry at the start of the requested step, with constant world-frame velocities.
#[derive(Clone, Copy, Debug)]
pub enum Boundary {
    /// One-way rounded cylinder. The axial segment has length twice half_length,
    /// in metres. Its world axis stays fixed as the center translates; signed
    /// angular speed about that axis supplies the surface's circumferential velocity.
    Capsule {
        center: [f64; 3],
        axis: [f64; 3],
        radius: f64,
        half_length: f64,
        velocity: [f64; 3],
        angular_speed: f64,
    },
    /// Free COM-centered capsule. Local +Z is its segment axis, and the
    /// accompanying principal motion supplies its start and end orientations.
    DynamicCapsule {
        center: [f64; 3],
        radius: f64,
        half_length: f64,
        velocity: [f64; 3],
        angular_velocity: [f64; 3],
    },
    /// Immobile one-way box. Orientation is a world unit quaternion [x, y, z, w].
    StaticCuboid {
        center: [f64; 3],
        half_extents: [f64; 3],
        orientation: [f64; 4],
    },
    /// COM-centered box. Its uncoupled orientation follows the shortest arc
    /// between the two world quaternions in the accompanying principal motion.
    Cuboid {
        center: [f64; 3],
        half_extents: [f64; 3],
        velocity: [f64; 3],
        angular_velocity: [f64; 3],
    },
    Sphere {
        center: [f64; 3],
        radius: f64,
        velocity: [f64; 3],
        angular_velocity: [f64; 3],
    },
    Plane {
        point: [f64; 3],
        normal: [f64; 3],
        velocity: [f64; 3],
    },
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[cfg_attr(
    feature = "one-way-checkpoint",
    derive(serde::Serialize, serde::Deserialize)
)]
pub struct BoundaryImpulse {
    pub boundary: usize,
    /// Total impulse **on the boundary** in N s, in world axes.
    pub linear: [f64; 3],
    /// Angular impulse on the boundary in N m s, about its fixed start-of-step
    /// center (sphere, capsule or cuboid) or reference point (plane). Point particles carry no spin.
    pub angular: [f64; 3],
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum XpbdError {
    #[error(
        "structural angular motion requires a smaller public step or a higher configured substep budget"
    )]
    UnresolvedStructure,
    #[error("finite capsule receivers also need simultaneous native rigid contact")]
    SimultaneousCapsules,
    #[error("rotating rigid boundary sweep needs a smaller soft substep to resolve contact")]
    UnresolvedSweep,
    #[error("swept surface self-contact exceeded its bounded work budget")]
    SurfaceBudget,
    #[error("swept surface self-contact has no resolved separating response")]
    UnresolvedSurface,
    #[error("material penetrates an articulated sphere at the proposed impact time")]
    PenetratingImpact,
    #[error(
        "continuous material approach requires an unstrained material in uniform free flight and a certified sphere trajectory"
    )]
    NonlinearApproach,
    #[error("material checkpoint has an incompatible construction, length or state")]
    MaterialCheckpoint,
    #[error("articulated material impact has no resolved passive response")]
    UnresolvedImpact,
    #[error("articulated material contact exceeds its geometric or simultaneous-row budget")]
    ImpactBudget,
    #[error("invalid particle mass, position or velocity")]
    InvalidParticle,
    #[error("invalid soft body topology, rest length or compliance")]
    InvalidConstraint,
    #[error("invalid solver configuration")]
    InvalidConfig,
    #[error("invalid boundary geometry or twist")]
    InvalidBoundary,
    #[error("a positive step requires at least one nanosecond per substep")]
    InvalidStep,
    #[error("soft-body step exceeds the representable numerical range")]
    Numerical,
}

#[derive(Clone, Copy)]
struct Distance {
    spec: DistanceConstraint,
    rest_direction: DVec3,
}

/// Sums for one committed public step of opt-in surface self-friction.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SelfContactImpulse {
    /// Sum of compressive magnitudes at material contacts, in N s.
    pub normal: f64,
    /// Sum of tangential impulse magnitudes, in N s.
    pub tangent: f64,
    /// Kinetic energy dissipated directly by tangential impulses, in joules.
    pub friction_work: f64,
}

#[derive(Clone, Copy)]
#[cfg_attr(
    feature = "one-way-checkpoint",
    derive(serde::Serialize, serde::Deserialize)
)]
enum MaterialTopology {
    Particles,
    Cable,
    Cloth(usize),
}

#[derive(Clone)]
pub struct SoftBody {
    #[cfg(feature = "one-way-checkpoint")]
    one_way_identity: std::sync::OnceLock<[u8; 32]>,
    topology: MaterialTopology,
    capsule_material: Option<capsule_material::CapsuleMaterial>,
    particles: Vec<Particle>,
    constraints: Vec<DistanceConstraint>,
    rest_directions: Vec<DVec3>,
    bending: Vec<DihedralConstraint>,
    bending_lambdas: Vec<f64>,
    config: SolverConfig,
    reactions: Vec<BoundaryImpulse>,
    self_impulse: SelfContactImpulse,
    sweep_refinements: u32,
    path_substeps: u32,
    momentum_rows: Option<momentum::MomentumRows>,
    // Reused transaction workspace. A numerical refusal never publishes a partial step.
    scratch: Vec<Particle>,
    predicted: Vec<DVec3>,
    lambdas: Vec<f64>,
    self_contact: Option<SelfContact>,
    surface: Option<surface::Surface>,
    pads: Option<pad::Pads>,
    contacts: Contacts,
    box_faces: static_box_faces::Faces,
}

impl SoftBody {
    pub fn new(
        particles: Vec<Particle>,
        constraints: Vec<DistanceConstraint>,
        config: SolverConfig,
    ) -> Result<Self, XpbdError> {
        if particles.is_empty()
            || particles.iter().any(|p| {
                !DVec3::from(p.position).is_finite()
                    || !DVec3::from(p.velocity).is_finite()
                    || !p.inverse_mass.is_finite()
                    || p.inverse_mass < 0.0
                    || p.inverse_mass > 0.0 && !p.inverse_mass.recip().is_finite()
            })
        {
            return Err(XpbdError::InvalidParticle);
        }
        if config.substeps == 0
            || config.iterations == 0
            || !DVec3::from(config.gravity).is_finite()
            || !config.particle_radius.is_finite()
            || config.particle_radius < 0.0
            || !config.friction.is_finite()
            || config.friction < 0.0
            || config.self_contact
                && (config.particle_radius == 0.0 || !(2.0 * config.particle_radius).is_finite())
        {
            return Err(XpbdError::InvalidConfig);
        }
        let mut edges = HashSet::with_capacity(constraints.len());
        let mut rest_directions = Vec::with_capacity(constraints.len());
        for &c in &constraints {
            if c.a >= particles.len()
                || c.b >= particles.len()
                || c.a == c.b
                || !c.rest_length.is_finite()
                || c.rest_length <= 0.0
                || !c.compliance.is_finite()
                || c.compliance < 0.0
                || !edges.insert((c.a.min(c.b), c.a.max(c.b)))
            {
                return Err(XpbdError::InvalidConstraint);
            }
            let delta = DVec3::from(particles[c.a].position) - DVec3::from(particles[c.b].position);
            if !delta.length().is_finite() {
                return Err(XpbdError::InvalidConstraint);
            }
            rest_directions.push(delta.try_normalize().unwrap_or(DVec3::X));
        }
        let self_contact = config
            .self_contact
            .then(|| SelfContact::new(edges, particles.len()));
        Ok(Self {
            #[cfg(feature = "one-way-checkpoint")]
            one_way_identity: Default::default(),
            topology: MaterialTopology::Particles,
            capsule_material: None,
            bending: Vec::new(),
            bending_lambdas: Vec::new(),
            scratch: particles.clone(),
            predicted: vec![DVec3::ZERO; particles.len()],
            lambdas: vec![0.0; constraints.len()],
            particles,
            constraints,
            config,
            rest_directions,
            self_contact,
            surface: None,
            pads: None,
            reactions: Vec::new(),
            self_impulse: SelfContactImpulse::default(),
            sweep_refinements: 0,
            path_substeps: 0,
            momentum_rows: None,
            contacts: Contacts::default(),
            box_faces: Default::default(),
        })
    }

    /// Use torque-free structural position impulses and enforce the tangent
    /// velocities of zero-compliance rows. Uniform gravity and external
    /// contacts are applied by their usual solver paths.
    pub fn with_angular_momentum_conservation(mut self) -> Self {
        #[cfg(feature = "one-way-checkpoint")]
        self.one_way_identity.take();
        self.momentum_rows = Some(momentum::MomentumRows::new(&self));
        self
    }
    /// Resolve capsule contact against frictionless consecutive cable edges or
    /// a rectangular cloth mesh with Coulomb skin friction. Prescribed guides report support impulses.
    /// A finite receiver in the shared coupled solve responds in translation and rotation.
    pub fn with_capsule_material_contact(mut self) -> Result<Self, XpbdError> {
        if self.config.particle_radius <= 0. || self.pads.is_some() {
            return Err(XpbdError::InvalidConfig);
        }
        let columns = match self.topology {
            MaterialTopology::Cable if self.config.friction == 0. => None,
            MaterialTopology::Cloth(columns) => Some(columns),
            MaterialTopology::Cable | MaterialTopology::Particles => {
                return Err(XpbdError::InvalidConfig);
            }
        };
        self.capsule_material = Some(capsule_material::CapsuleMaterial::new(
            self.particles.len(),
            columns,
            self.config.friction,
        )?);
        Ok(self)
    }
    pub fn uses_capsule_material_contact(&self) -> bool {
        self.capsule_material.is_some()
    }
    pub fn last_capsule_material_loads(&self) -> &[CapsuleMaterialLoad] {
        self.capsule_material
            .as_ref()
            .map_or(&[], |m| m.published.as_slice())
    }
    /// Tangential shell impulses, including all incident patch vertices carrying the skin couple.
    pub fn last_capsule_friction_loads(&self) -> &[CapsuleFrictionLoad] {
        self.capsule_material
            .as_ref()
            .map_or(&[], |m| m.friction.published.as_slice())
    }
    /// Retain individual compressive substep contacts for a friction-enabled capsule shell.
    pub fn with_capsule_contact_history(mut self) -> Result<Self, XpbdError> {
        self.capsule_material
            .as_mut()
            .ok_or(XpbdError::InvalidConfig)?
            .friction
            .enable_samples()?;
        Ok(self)
    }
    pub fn last_capsule_contact_samples(&self) -> &[CapsuleContactSample] {
        self.capsule_material
            .as_ref()
            .map_or(&[], |m| m.friction.published_samples.as_slice())
    }
    fn structural_substeps(&self, dt: Duration) -> Result<u32, XpbdError> {
        match &self.momentum_rows {
            Some(rows) if !dt.is_zero() => rows.substeps(self, dt),
            _ => Ok(self.config.substeps),
        }
    }
    pub fn particles(&self) -> &[Particle] {
        &self.particles
    }
    pub fn constraints(&self) -> &[DistanceConstraint] {
        &self.constraints
    }
    pub fn bending_constraints(&self) -> &[DihedralConstraint] {
        &self.bending
    }
    pub fn uses_pad_surface_contact(&self) -> bool {
        self.pads.is_some()
    }
    pub fn last_self_contact_impulse(&self) -> SelfContactImpulse {
        self.self_impulse
    }
    pub fn last_path_substeps(&self) -> u32 {
        self.path_substeps
    }
    fn begin_self_contact(&mut self) {
        if let Some(loads) = &mut self.contacts.pad_loads {
            loads.begin();
        }
        if let Some(m) = &mut self.capsule_material {
            m.begin_step();
        }
        if let Some(s) = &mut self.surface {
            s.impulse = SelfContactImpulse::default();
            s.begin_loads();
        }
    }
    fn publish_self_contact(&mut self) {
        if let Some(s) = &mut self.surface {
            s.publish_loads();
        }
        if let Some(loads) = &mut self.contacts.pad_loads {
            loads.publish();
        }
        if let Some(m) = &mut self.capsule_material {
            m.publish();
        }
        self.self_impulse = self.surface.as_ref().map(|s| s.impulse).unwrap_or_default();
    }
    /// Change the integration settings for subsequent public steps. Committed
    /// state, rest geometry and the previous step's observations are preserved.
    pub fn set_resolution(&mut self, substeps: u32, iterations: u32) -> Result<(), XpbdError> {
        if substeps == 0 || iterations == 0 {
            return Err(XpbdError::InvalidConfig);
        }
        #[cfg(feature = "one-way-checkpoint")]
        self.one_way_identity.take();
        self.config.substeps = substeps;
        self.config.iterations = iterations;
        Ok(())
    }
    /// Set the world-frame uniform acceleration for subsequent steps, in m/s².
    /// This also updates the gravitational field used by energy and sweep queries.
    pub fn set_gravity(&mut self, gravity: [f64; 3]) -> Result<(), XpbdError> {
        if !DVec3::from(gravity).is_finite() {
            return Err(XpbdError::InvalidConfig);
        }
        #[cfg(feature = "one-way-checkpoint")]
        self.one_way_identity.take();
        self.config.gravity = gravity;
        Ok(())
    }

    pub fn config(&self) -> SolverConfig {
        self.config
    }
    /// Refinement depth of the last committed coupled step. For locally split
    /// shell contact this is the deepest accepted bisection from its base grid.
    pub fn last_sweep_refinements(&self) -> u32 {
        self.sweep_refinements
    }
    /// Accepted leaf intervals when a shell used local subdivision on the last
    /// committed step. Uniform solves have no local subdivision count.
    pub fn last_local_shell_intervals(&self) -> Option<u32> {
        self.surface.as_ref().and_then(|s| s.local_intervals)
    }
    pub fn last_boundary_impulses(&self) -> &[BoundaryImpulse] {
        &self.reactions
    }

    /// Advance one integer-duration step. Remainder nanoseconds go to the earliest
    /// substeps. Reactions are reported in input boundary order, including zeros.
    pub fn step(&mut self, dt: Duration, boundaries: &[Boundary]) -> Result<(), XpbdError> {
        self.step_with_boundary_paths(dt, boundaries, &[], DEFAULT_PATH_TOLERANCE)
    }

    /// Advance prescribed spheres or fixed-axis capsules along bounded curves. The centerline
    /// stays within the supplied tolerance of each sweep. Reactions refer to the original
    /// boundary center at the start of the public interval, including when the curve changes direction.
    pub fn step_with_boundary_paths(
        &mut self,
        dt: Duration,
        boundaries: &[Boundary],
        paths: &[(usize, &dyn BoundaryPath)],
        tolerance: f64,
    ) -> Result<(), XpbdError> {
        if boundaries
            .iter()
            .any(|b| matches!(b, Boundary::Cuboid { .. } | Boundary::DynamicCapsule { .. }))
        {
            return Err(XpbdError::InvalidBoundary);
        }
        if self.pads.is_some()
            && boundaries
                .iter()
                .any(|b| matches!(b, Boundary::Sphere { .. } | Boundary::Capsule { .. }))
        {
            return Err(XpbdError::InvalidBoundary);
        }
        let ticks = dt.as_nanos();
        let structural = self.structural_substeps(dt)?;
        let schedule = if paths.is_empty() {
            None
        } else {
            Some(boundary_path::Schedule::build(
                dt, structural, boundaries, paths, tolerance,
            )?)
        };
        let count = schedule
            .as_ref()
            .map_or(u128::from(structural), |s| s.len() as u128);
        if ticks != 0 && ticks < count {
            return Err(XpbdError::InvalidStep);
        }
        let mut prepared = boundaries
            .iter()
            .copied()
            .map(PreparedBoundary::new)
            .collect::<Result<Vec<_>, _>>()?;
        let mut reactions = (0..boundaries.len())
            .map(|boundary| BoundaryImpulse {
                boundary,
                ..Default::default()
            })
            .collect::<Vec<_>>();
        self.begin_self_contact();
        if ticks != 0 {
            self.scratch.copy_from_slice(&self.particles);
            let mut elapsed = 0_u128;
            for i in 0..count {
                let substep = match &schedule {
                    Some(s) => s.apply(i as usize, &mut prepared) - elapsed,
                    None => ticks / count + u128::from(i < ticks % count),
                };
                let h = substep as f64 * 1e-9;
                self.predict(h)?;
                for _ in 0..self.config.iterations {
                    self.project_iteration(elapsed, substep, &mut prepared, &mut reactions)?;
                }
                self.finish(
                    h,
                    (elapsed + substep) as f64 * 1e-9,
                    &mut prepared,
                    &mut reactions,
                )?;
                elapsed += substep;
            }
            std::mem::swap(&mut self.particles, &mut self.scratch);
        }
        self.path_substeps = schedule.as_ref().map_or(0, |s| s.len() as u32);
        self.reactions = reactions;
        self.sweep_refinements = 0;
        if let Some(surface) = &mut self.surface {
            surface.local_intervals = None;
        }
        self.publish_self_contact();
        Ok(())
    }

    fn predict(&mut self, h: f64) -> Result<(), XpbdError> {
        for (x, p) in self.predicted.iter_mut().zip(&mut self.scratch) {
            let mut velocity = DVec3::from(p.velocity);
            if p.inverse_mass > 0.0 {
                velocity += h * DVec3::from(self.config.gravity);
            }
            *x = DVec3::from(p.position) + h * velocity;
            p.velocity = velocity.to_array();
            if !x.is_finite() {
                return Err(XpbdError::Numerical);
            }
        }
        if let Some(rows) = &mut self.momentum_rows {
            rows.prepare(
                &self.scratch,
                &self.constraints,
                &self.rest_directions,
                &self.bending,
            )?;
        }
        self.lambdas.fill(0.0);
        self.bending_lambdas.fill(0.0);
        self.contacts.clear(self.config.friction);
        self.box_faces.clear();
        if let Some(m) = &mut self.capsule_material {
            m.begin_substep();
        }
        if let Some(pads) = &mut self.pads {
            pads.clear();
        }
        Ok(())
    }

    fn project_iteration(
        &mut self,
        elapsed: u128,
        ticks: u128,
        boundaries: &mut [PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
    ) -> Result<(), XpbdError> {
        let h = ticks as f64 * 1e-9;
        let t0 = elapsed as f64 * 1e-9;
        let t1 = (elapsed + ticks) as f64 * 1e-9;
        for (i, ((&spec, &rest_direction), lambda)) in self
            .constraints
            .iter()
            .zip(&self.rest_directions)
            .zip(&mut self.lambdas)
            .enumerate()
        {
            Distance {
                spec,
                rest_direction,
            }
            .solve(
                &mut self.predicted,
                &mut self.scratch,
                lambda,
                h,
                self.momentum_rows.as_ref().map(|rows| rows.distance[i]),
            )?;
        }
        for (i, (hinge, lambda)) in self
            .bending
            .iter()
            .zip(&mut self.bending_lambdas)
            .enumerate()
        {
            hinge.solve(
                &mut self.predicted,
                &mut self.scratch,
                lambda,
                h,
                self.momentum_rows.as_ref().map(|rows| rows.bending[i]),
            )?;
        }
        if let Some(surface) = &mut self.surface {
            surface.project(
                &mut self.predicted,
                &mut self.scratch,
                2.0 * self.config.particle_radius,
                h,
                self.contacts.on_single_plane(boundaries),
            )?;
        } else if let Some(self_contact) = &mut self.self_contact {
            self_contact.project(
                &mut self.predicted,
                &mut self.scratch,
                2.0 * self.config.particle_radius,
                h,
            )?;
        }
        let interval = collision::Interval {
            t0,
            t1,
            h,
            radius: self.config.particle_radius,
        };
        collision::project_boundaries(
            &mut self.predicted,
            &mut self.scratch,
            boundaries,
            &mut self.contacts,
            reactions,
            interval,
            (self.pads.is_some(), self.capsule_material.is_some()),
        )?;
        if let MaterialTopology::Cloth(columns) = self.topology {
            self.box_faces.project(
                columns,
                &mut self.predicted,
                &mut self.scratch,
                boundaries,
                reactions,
                interval,
            )?;
        }
        if let Some(m) = &mut self.capsule_material {
            m.project(
                &mut self.predicted,
                &mut self.scratch,
                boundaries,
                reactions,
                interval,
            )?;
            m.stick(
                &mut self.predicted,
                &mut self.scratch,
                boundaries,
                reactions,
                interval,
            )?;
        }
        if let Some(pads) = &mut self.pads {
            pads.project(
                &mut self.predicted,
                &mut self.scratch,
                boundaries,
                reactions,
                interval,
                self.config.friction,
            )?;
        }
        Ok(())
    }

    fn finish(
        &mut self,
        h: f64,
        t1: f64,
        boundaries: &mut [PreparedBoundary],
        reactions: &mut [BoundaryImpulse],
    ) -> Result<(), XpbdError> {
        if let Some(pads) = &mut self.pads {
            pads.polish(
                &mut self.predicted,
                &mut self.scratch,
                boundaries,
                reactions,
                collision::Interval {
                    h,
                    t0: t1 - h,
                    t1,
                    radius: self.config.particle_radius,
                },
            )?;
        }
        // Static tangential position correction may move a normal witness on
        // another capsule. Recover its normal projection before the momentum
        // and friction solves. The final endpoint guard still checks every face.
        if let Some(m) = &mut self.capsule_material
            && m.retry_manifold
            && matches!(
                m.check_finite_endpoint(&self.predicted, boundaries, self.config.particle_radius),
                Err(XpbdError::UnresolvedSweep)
            )
        {
            m.project(
                &mut self.predicted,
                &mut self.scratch,
                boundaries,
                reactions,
                collision::Interval {
                    h,
                    t0: t1 - h,
                    t1,
                    radius: self.config.particle_radius,
                },
            )?;
        }
        if let Some(rows) = &mut self.momentum_rows {
            rows.finish(
                &self.predicted,
                &mut self.scratch,
                &self.constraints,
                &self.bending,
                self.config.iterations,
            )?;
        }
        collision::finish_contacts(
            &mut self.contacts,
            &self.predicted,
            &mut self.scratch,
            boundaries,
            reactions,
            self.config.friction,
            h,
        )?;
        self.box_faces.finish(
            &self.predicted,
            &mut self.scratch,
            boundaries,
            reactions,
            self.config,
            h,
        )?;
        if let Some(m) = &mut self.capsule_material {
            m.finish(
                &self.predicted,
                &mut self.scratch,
                boundaries,
                reactions,
                collision::Interval {
                    h,
                    t0: t1 - h,
                    t1,
                    radius: self.config.particle_radius,
                },
            )?;
        }
        if let Some(pads) = &mut self.pads {
            pads.finish(
                &self.predicted,
                &mut self.scratch,
                boundaries,
                reactions,
                self.config.friction,
                h,
                self.config.particle_radius,
            )?;
        }
        if let Some(surface) = &mut self.surface {
            surface.check_intersections(&self.predicted)?;
        }
        // An external boundary can move a vertex after the shell projection.
        // Refuse a conflicting substep before publishing either response.
        if !boundaries.is_empty()
            && let Some(surface) = &mut self.surface
        {
            let gap = 2. * self.config.particle_radius;
            if self.contacts.on_single_plane(boundaries) {
                surface.finish_supported(&mut self.predicted, &mut self.scratch, gap, |x, p| {
                    self.contacts.finish_plane_increment(
                        x,
                        p,
                        boundaries,
                        reactions,
                        self.config.friction,
                        collision::Interval {
                            t0: t1 - h,
                            t1,
                            h,
                            radius: self.config.particle_radius,
                        },
                    )
                })?;
            } else {
                surface.verify(&self.predicted, &self.scratch, gap)?;
            }
        }
        self.contacts.verify_pad_samples(
            &self.scratch,
            &self.predicted,
            boundaries,
            collision::Interval {
                h,
                t0: t1 - h,
                t1,
                radius: self.config.particle_radius,
            },
        )?;
        for (x, p) in self.predicted.iter().zip(&mut self.scratch) {
            if !x.is_finite() || !DVec3::from(p.velocity).is_finite() {
                return Err(XpbdError::Numerical);
            }
            p.position = x.to_array();
        }
        Ok(())
    }
}

/// Apply paired position and velocity corrections consistently, preserving
/// the target's existing velocity when a very small displacement rounds off.
fn correct(position: &mut DVec3, particle: &mut Particle, dx: DVec3, h: f64) {
    *position += dx;
    particle.velocity = (DVec3::from(particle.velocity) + dx / h).to_array();
}

impl Distance {
    fn solve(
        self,
        x: &mut [DVec3],
        particles: &mut [Particle],
        lambda: &mut f64,
        h: f64,
        reference: Option<DVec3>,
    ) -> Result<(), XpbdError> {
        let c = self.spec;
        let (wa, wb) = (particles[c.a].inverse_mass, particles[c.b].inverse_mass);
        if wa == 0.0 && wb == 0.0 {
            return Ok(());
        }
        let delta = x[c.a] - x[c.b];
        let len = delta.length();
        let alpha = c.compliance / (h * h);
        if !len.is_finite() {
            return Err(XpbdError::Numerical);
        }
        let n = if len > 0.0 {
            delta / len
        } else {
            self.rest_direction
        };
        let (n, inverse) = match reference {
            Some(prior) => {
                // Position projection changes discrete angular momentum by the
                // START lever cross the impulse. Use that lever as the direction.
                let overlap = n.dot(prior);
                (prior * overlap, (wa + wb) * overlap * overlap)
            }
            None => (n, wa + wb),
        };
        let denominator = inverse + alpha;
        if !denominator.is_finite() || denominator == 0. {
            return Err(XpbdError::Numerical);
        }
        let dl = (c.rest_length - len - alpha * *lambda) / denominator;
        *lambda += dl;
        correct(&mut x[c.a], &mut particles[c.a], n * (wa * dl), h);
        correct(&mut x[c.b], &mut particles[c.b], -n * (wb * dl), h);
        Ok(())
    }
}
