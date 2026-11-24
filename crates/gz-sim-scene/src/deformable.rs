// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! World-space XPBD bodies and explicitly selected rigid boundaries.
//! Read particle state after an advancing tick through [`bodies`].
//!
//! A centered, dynamic, jointed cuboid can opt into `coupling="articulated_wrench"`.
//! XPBD samples the link's native motion and accumulates the force impulse and
//! angular impulse from contacts with each material. The next advancing native
//! tick applies their sum at the link center of mass through its joint constraints.
//! `normal_compliance` sets per-particle normal contact compliance in m/N and defaults to zero.
mod boundary_path;
mod force;
pub use force::{MaterialForceError, set_material_force};
mod particle_visual;
mod resolution;
pub use resolution::{MaterialResolutionError, set_material_resolution};
#[cfg(feature = "distributed-physics")]
pub(crate) mod worker;
use super::*;
use gz_physics::{BodyState, DynamicBackend};
use gz_physics_multifidelity::SoftRigidInertia;
use gz_physics_xpbd::{
    Boundary, BoundaryImpulse, Particle, PrincipalRigidMotion, RigidMobility, SoftBody,
    SphereMobility, step_coupled_rigid,
};
use gz_sim_core::{StepFault, UpdateInfo, entity_by_id};

pub mod articulated_impact;
mod config;
#[cfg(test)]
mod forcing_tests;
#[cfg(feature = "distributed-physics")]
pub mod managed_impact;
pub(super) use config::{install, is_plugin};

#[derive(Clone, Copy)]
enum Surface {
    Plane {
        point: [f64; 3],
        normal: [f64; 3],
    },
    Rigid {
        link: u64,
        body: BodyId,
        shape: ReceiverShape,
        mobility: Option<ReceiverMass>,
        static_box_pose: Option<Transform>,
    },
}

#[derive(Clone, Copy)]
enum ReceiverShape {
    Capsule { radius: f64, half_length: f64 },
    Sphere(f64),
    Cuboid([f64; 3]),
}

#[derive(Clone, Copy)]
enum ReceiverMass {
    Isotropic(SphereMobility),
    Principal(SoftRigidInertia),
    ForceAware(SoftRigidInertia),
    ArticulatedWrench { normal_compliance: f64 },
}

impl ReceiverMass {
    fn from_native(mass: SoftRigidInertia) -> Self {
        match mass.isotropic_inverse_inertia() {
            Some(inverse_inertia) => Self::Isotropic(SphereMobility {
                inverse_mass: mass.inverse_mass,
                inverse_inertia,
            }),
            None => Self::Principal(mass),
        }
    }
    fn trajectory(
        self,
        start: BodyState,
        end: BodyState,
        acceleration: Option<[f64; 3]>,
        dt: Duration,
    ) -> RigidMobility {
        match self {
            Self::Isotropic(mass) => RigidMobility::Isotropic(mass),
            Self::ArticulatedWrench { normal_compliance } => RigidMobility::PrescribedCuboid {
                normal_compliance,
                start_orientation: start.orientation,
                end_orientation: end.orientation,
                start_angular_velocity: start.angular_velocity,
            },
            Self::Principal(mass) | Self::ForceAware(mass) => {
                let rotation = PrincipalRigidMotion {
                    inverse_mass: mass.inverse_mass,
                    inverse_principal_inertia: mass.inverse_principal_inertia,
                    principal_orientation: mass.principal_orientation,
                    start_orientation: start.orientation,
                    end_orientation: end.orientation,
                    start_angular_velocity: start.angular_velocity,
                };
                if matches!(self, Self::ForceAware(_))
                    && let Some(acceleration) = acceleration
                {
                    let v0 = DVec3::from(start.velocity);
                    let v1 = DVec3::from(end.velocity);
                    let budget = DVec3::from(acceleration) * dt.as_secs_f64();
                    let residual = v1 - v0 - budget;
                    // This admits only the force actually read from native at the
                    // barrier. An unexplained contact impulse keeps the full native chord.
                    if residual.length()
                        <= 256. * f64::EPSILON * (1. + v0.length() + v1.length() + budget.length())
                    {
                        return RigidMobility::ForceAware {
                            rotation,
                            start_velocity: start.velocity,
                            acceleration,
                        };
                    }
                }
                RigidMobility::Principal(rotation)
            }
        }
    }
}

#[derive(Resource)]
pub(crate) struct Deformables {
    #[cfg(feature = "distributed-physics")]
    pub(super) worker_threads: u8,
    #[cfg(feature = "distributed-physics")]
    pub(super) worker_processes: u8,
    #[cfg(feature = "distributed-physics")]
    pub(super) warm_budget: Option<std::time::Duration>,
    #[cfg(feature = "distributed-physics")]
    pub(super) packed_replies: bool,
    #[cfg(feature = "distributed-physics")]
    pub(super) worker_transport: gz_physics_distributed::moving::WorkerTransport,
    #[cfg(feature = "distributed-physics")]
    remote_material: bool,
    visuals: Vec<particle_visual::Binding>,
    automatic_impact: Option<articulated_impact::ScheduledImpact>,
    automatic_consumed: bool,
    impact_barriers: BTreeMap<Duration, articulated_impact::ScheduledImpact>,
    pending_accelerations: BTreeMap<String, [f64; 3]>,
    initial: BTreeMap<String, SoftBody>,
    current: BTreeMap<String, SoftBody>,
    surfaces: Vec<Surface>,
    starts: Vec<Option<BodyState>>,
    accelerations: Vec<Option<[f64; 3]>>,
    applied_forcing: Vec<CapsuleForcing>,
    boundaries: Vec<Boundary>,
    mobility: Vec<Option<RigidMobility>>,
    receivers: Vec<Option<(BodyId, BodyState)>>,
    active_slots: Vec<usize>,
    articulated_wrenches: bool,
    samples: BTreeMap<String, DeformableSample>,
}

/// Measured input and native predictor for an explicitly force-aware capsule.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CapsuleForcing {
    pub boundary: usize,
    pub acceleration: Option<[f64; 3]>,
    /// False if an unexplained native impulse or unavailable acceleration kept the native chord.
    pub used: bool,
    pub native_start_velocity: [f64; 3],
    pub native_end_velocity: [f64; 3],
    pub native_chord_velocity: [f64; 3],
}
/// Committed soft state. Reaction indices refer to the original SDF boundary
/// order. Removed surfaces contribute zero. Reset clears all prior reactions.
#[derive(Default)]
pub struct DeformableSample {
    movable_mass_kg: f64,
    supports: Vec<articulated_impact::ArticulatedSupportInterval>,
    impacts_this_step: Vec<std::sync::Arc<articulated_impact::ArticulatedMaterialImpact>>,
    impact_count: u64,
    latest_articulated_impact:
        Option<std::sync::Arc<articulated_impact::ArticulatedMaterialImpact>>,
    capsule_loads: Vec<gz_physics_xpbd::CapsuleMaterialLoad>,
    capsule_friction: Vec<gz_physics_xpbd::CapsuleFrictionLoad>,
    capsule_samples: Vec<gz_physics_xpbd::CapsuleContactSample>,
    pad_loads: Vec<gz_physics_xpbd::ParticlePadImpulse>,
    self_loads: Vec<gz_physics_xpbd::SelfContactLoad>,
    self_particle_impulses: Vec<gz_physics_xpbd::SelfContactParticleImpulse>,
    applied_forcing: Vec<CapsuleForcing>,
    particles: Vec<Particle>,
    reactions: Vec<BoundaryImpulse>,
    self_impulse: gz_physics_xpbd::SelfContactImpulse,
    sweep_refinements: u32,
    local_shell_intervals: Option<u32>,
    path_substeps: u32,
    time: Duration,
    epoch: u64,
}

impl DeformableSample {
    /// Full normal and tangential impulses on native links and material
    /// over this committed public step, in chronological order.
    pub fn support_this_step(&self) -> &[articulated_impact::ArticulatedSupportInterval] {
        &self.supports
    }

    /// Compressive support records, including tangential loads when enabled.
    pub fn normal_support_this_step(&self) -> &[articulated_impact::ArticulatedSupportInterval] {
        &self.supports
    }

    /// All accepted contacts in chronological order in this public physics step.
    pub fn impacts_this_step(
        &self,
    ) -> &[std::sync::Arc<articulated_impact::ArticulatedMaterialImpact>] {
        &self.impacts_this_step
    }
    /// Number of accepted contact events since the last epoch reset.
    pub fn articulated_impact_count(&self) -> u64 {
        self.impact_count
    }
    fn record_impact(
        &mut self,
        impact: std::sync::Arc<articulated_impact::ArticulatedMaterialImpact>,
    ) {
        self.impact_count += 1;
        self.impacts_this_step.push(impact.clone());
        self.latest_articulated_impact = Some(impact);
    }

    /// Latest actual impulsive articulated/material contact in the current epoch.
    pub fn latest_articulated_impact(
        &self,
    ) -> Option<&articulated_impact::ArticulatedMaterialImpact> {
        self.latest_articulated_impact.as_deref()
    }
    pub fn particles(&self) -> &[Particle] {
        &self.particles
    }
    /// World impulses on each compliant pad, indexed by original SDF boundary
    /// and construction particle. Populated for material with `pad_loads="true"`.
    pub fn last_pad_loads(&self) -> &[gz_physics_xpbd::ParticlePadImpulse] {
        &self.pad_loads
    }
    pub fn last_boundary_impulses(&self) -> &[BoundaryImpulse] {
        &self.reactions
    }
    /// Attributed support impulses, using original SDF boundary and construction particle indices.
    pub fn last_capsule_material_loads(&self) -> &[gz_physics_xpbd::CapsuleMaterialLoad] {
        &self.capsule_loads
    }
    /// Tangential skin impulses with their complete patch and original SDF boundary index.
    pub fn last_capsule_friction_loads(&self) -> &[gz_physics_xpbd::CapsuleFrictionLoad] {
        &self.capsule_friction
    }
    /// Pre/post native velocities and accepted constant-force motion at the last barrier.
    pub fn capsule_forcing(&self) -> &[CapsuleForcing] {
        &self.applied_forcing
    }
    /// Individual converged substep loads when enabled on a frictional capsule cloth.
    pub fn last_capsule_contact_samples(&self) -> &[gz_physics_xpbd::CapsuleContactSample] {
        &self.capsule_samples
    }

    /// Normal and friction impulses delivered to the actual material nodes.
    pub fn last_self_contact_particle_impulses(
        &self,
    ) -> &[gz_physics_xpbd::SelfContactParticleImpulse] {
        &self.self_particle_impulses
    }
    /// Barycentric normal loads on the geometric skins during the last advancing tick.
    pub fn last_self_contact_loads(&self) -> &[gz_physics_xpbd::SelfContactLoad] {
        &self.self_loads
    }
    pub fn last_self_contact_impulse(&self) -> gz_physics_xpbd::SelfContactImpulse {
        self.self_impulse
    }
    /// Number of resolved path intervals on this material during the last public step.
    pub fn last_path_substeps(&self) -> u32 {
        self.path_substeps
    }
    /// Number of finer contact clocks attempted before publishing this tick.
    pub fn last_sweep_refinements(&self) -> u32 {
        self.sweep_refinements
    }
    /// Exact number of accepted intervals for a locally subdivided shell step.
    pub fn last_local_shell_intervals(&self) -> Option<u32> {
        self.local_shell_intervals
    }
    pub fn sim_time(&self) -> Duration {
        self.time
    }
    pub fn epoch(&self) -> u64 {
        self.epoch
    }
}

pub(super) fn automatic_impact(world: &World) -> bool {
    world
        .get_resource::<Deformables>()
        .is_some_and(|soft| soft.automatic_impact.is_some())
}

pub(super) fn installed(world: &World) -> bool {
    world.contains_resource::<Deformables>()
}

/// Enumerate named soft bodies in lexical order, with particle positions in
/// world metres and velocities in world metres per second.
pub fn bodies(sim: &Simulation) -> impl Iterator<Item = (&str, &DeformableSample)> {
    sim.world()
        .get_resource::<Deformables>()
        .into_iter()
        .flat_map(|soft| {
            soft.samples
                .iter()
                .map(|(name, body)| (name.as_str(), body))
        })
}

/// Evaluate kinetic, gravitational and elastic energy of a named material in a
/// healthy loaded world. Values are in joules. The query does no work on other bodies.
pub fn material_energy(
    sim: &Simulation,
    name: &str,
) -> Option<Result<gz_physics_xpbd::MaterialEnergy, gz_physics_xpbd::XpbdError>> {
    if sim.is_faulted() {
        return None;
    }
    sim.world()
        .get_resource::<Deformables>()?
        .current
        .get(name)
        .map(SoftBody::mechanical_energy)
}

fn reset(info: Res<UpdateInfo>, mut soft: ResMut<Deformables>) {
    let soft = &mut *soft;
    soft.pending_accelerations.clear();
    soft.current.clone_from(&soft.initial);
    soft.automatic_consumed = false;
    soft.starts.fill(None);
    soft.accelerations.fill(None);
    soft.boundaries.clear();
    soft.publish(info.sim_time, info.epoch);
}

#[derive(Debug, thiserror::Error)]
enum CouplingError {
    #[error("deformable boundary no longer belongs to its loaded physics backend")]
    Binding,
    #[error("one-way capsule guide requires a fixed world axis and axial angular velocity")]
    TiltedCapsule,
    #[error("nonfinite deformable boundary trajectory or reaction")]
    Nonfinite,
    #[error(transparent)]
    Physics(#[from] PhysicsError),
    #[error(transparent)]
    Xpbd(#[from] gz_physics_xpbd::XpbdError),
}

fn capsule_axis(state: BodyState) -> Result<DVec3, CouplingError> {
    let axis = DQuat::from_array(state.orientation) * DVec3::Z;
    let spin = DVec3::from(state.angular_velocity);
    if !axis.is_finite()
        || !spin.is_finite()
        || axis.cross(spin).length() > 1e-12 * (1. + spin.length())
    {
        return Err(CouplingError::TiltedCapsule);
    }
    Ok(axis)
}

fn start_positions(world: &mut World) {
    if world.resource::<UpdateInfo>().dt.is_zero() {
        return;
    }
    let result = world.resource_scope(|world, mut soft: Mut<Deformables>| {
        if !initial::matches_physics_owner(world) {
            return Err(CouplingError::Binding);
        }
        let physics = world.resource::<PhysicsBackend>();
        let soft = &mut *soft;
        for ((surface, start), acceleration) in soft
            .surfaces
            .iter()
            .zip(&mut soft.starts)
            .zip(&mut soft.accelerations)
        {
            *acceleration = None;
            if let Surface::Rigid { link, body, .. } = *surface {
                // A removed model retires its surface. A loaded-world reset
                // reinstates the original public ID and construction handle.
                let Some(entity) = entity_by_id(world, link) else {
                    *start = None;
                    continue;
                };
                if world.get::<BodyHandle>(entity).map(BodyHandle::id) != Some(body) {
                    return Err(CouplingError::Binding);
                }
                *start = Some(physics.state(body).ok_or(CouplingError::Binding)?);
                if matches!(
                    surface,
                    Surface::Rigid {
                        mobility: Some(ReceiverMass::ForceAware(_)),
                        ..
                    }
                ) {
                    *acceleration = physics
                        .downcast_ref::<MultiFidelityBackend>()
                        .ok_or(CouplingError::Binding)?
                        .soft_linear_acceleration(body)?;
                }
            }
        }
        Ok(())
    });
    if let Err(error) = result {
        world.resource::<StepFault>().report(error);
    }
}

fn advance(
    info: Res<UpdateInfo>,
    mut soft: ResMut<Deformables>,
    mut physics: ResMut<PhysicsBackend>,
    fault: Res<StepFault>,
) {
    if !info.dt.is_zero() {
        #[cfg(feature = "distributed-physics")]
        if soft.remote_material {
            soft.publish(info.sim_time, info.epoch);
            return;
        }
        match soft.advance_with_forces(&mut physics, info.dt) {
            Ok(()) => soft.publish(info.sim_time, info.epoch),
            Err(error) => fault.report(error),
        }
    }
}

impl Deformables {
    fn publish(&mut self, time: Duration, epoch: u64) {
        for (name, body) in &self.current {
            let sample = self.samples.get_mut(name).expect("registered soft body");
            if time.is_zero() {
                sample.latest_articulated_impact = None;
                sample.impact_count = 0;
            }
            sample.impacts_this_step.clear();
            sample.supports.clear();
            sample.particles.clear();
            sample.particles.extend_from_slice(body.particles());
            sample.reactions.clear();
            sample.pad_loads.clear();
            sample.self_loads.clear();
            sample.self_particle_impulses.clear();
            sample.capsule_loads.clear();
            sample.capsule_friction.clear();
            sample.capsule_samples.clear();
            sample.applied_forcing.clear();
            sample.self_impulse = body.last_self_contact_impulse();
            sample.sweep_refinements = body.last_sweep_refinements();
            sample.local_shell_intervals = body.last_local_shell_intervals();
            sample.path_substeps = body.last_path_substeps();
            if !time.is_zero() {
                sample
                    .self_particle_impulses
                    .extend_from_slice(body.last_self_contact_particle_impulses());
                sample
                    .self_loads
                    .extend_from_slice(body.last_self_contact_loads());
                sample
                    .applied_forcing
                    .extend_from_slice(&self.applied_forcing);
                sample
                    .reactions
                    .extend((0..self.surfaces.len()).map(|boundary| BoundaryImpulse {
                        boundary,
                        ..Default::default()
                    }));
                sample
                    .capsule_loads
                    .extend(body.last_capsule_material_loads().iter().map(|v| {
                        gz_physics_xpbd::CapsuleMaterialLoad {
                            boundary: self.active_slots[v.boundary],
                            ..*v
                        }
                    }));
                sample.capsule_friction.extend(
                    body.last_capsule_friction_loads()
                        .iter()
                        .cloned()
                        .map(|mut v| {
                            v.boundary = self.active_slots[v.boundary];
                            v
                        }),
                );
                sample.capsule_samples.extend(
                    body.last_capsule_contact_samples()
                        .iter()
                        .copied()
                        .map(|mut c| {
                            c.boundary = self.active_slots[c.boundary];
                            c
                        }),
                );
                sample
                    .pad_loads
                    .extend(body.last_pad_loads().iter().map(|load| {
                        gz_physics_xpbd::ParticlePadImpulse {
                            boundary: self.active_slots[load.boundary],
                            ..*load
                        }
                    }));
                for value in body.last_boundary_impulses() {
                    let boundary = self.active_slots[value.boundary];
                    sample.reactions[boundary] = BoundaryImpulse { boundary, ..*value };
                }
            }
            sample.time = time;
            sample.epoch = epoch;
        }
    }

    fn advance(&mut self, physics: &mut PhysicsBackend, dt: Duration) -> Result<(), CouplingError> {
        self.boundaries.clear();
        self.receivers.clear();
        self.mobility.clear();
        self.active_slots.clear();
        self.applied_forcing.clear();
        let mut paths = Vec::new();
        for (slot, (surface, &start)) in self.surfaces.iter().zip(&self.starts).enumerate() {
            let (boundary, receiver, mobility) = match *surface {
                Surface::Plane { point, normal } => (
                    Boundary::Plane {
                        point,
                        normal,
                        velocity: [0.; 3],
                    },
                    None,
                    None,
                ),
                Surface::Rigid {
                    body,
                    shape,
                    mobility,
                    static_box_pose,
                    ..
                } => {
                    let Some(start) = start else {
                        continue;
                    };
                    let end = physics.state(body).ok_or(CouplingError::Binding)?;
                    let center = start.position;
                    if let Some(path) = boundary_path::from_native(physics, body, dt)? {
                        paths.push((self.boundaries.len(), path));
                    }
                    let velocity = (DVec3::from_array(end.position) - DVec3::from_array(center))
                        / dt.as_secs_f64();
                    if !velocity.is_finite() {
                        return Err(CouplingError::Nonfinite);
                    }
                    let path = mobility
                        .map(|mass| mass.trajectory(start, end, self.accelerations[slot], dt));
                    if matches!(mobility, Some(ReceiverMass::ForceAware(_))) {
                        self.applied_forcing.push(CapsuleForcing {
                            boundary: slot,
                            acceleration: self.accelerations[slot],
                            used: matches!(path, Some(RigidMobility::ForceAware { .. })),
                            native_start_velocity: start.velocity,
                            native_end_velocity: end.velocity,
                            native_chord_velocity: velocity.to_array(),
                        });
                    }
                    (
                        match shape {
                            ReceiverShape::Capsule {
                                radius,
                                half_length,
                            } if mobility.is_some() => Boundary::DynamicCapsule {
                                center,
                                radius,
                                half_length,
                                velocity: velocity.to_array(),
                                angular_velocity: end.angular_velocity,
                            },
                            ReceiverShape::Capsule {
                                radius,
                                half_length,
                            } => {
                                let axis = capsule_axis(start)?;
                                if axis.distance(capsule_axis(end)?) > 1e-12 {
                                    return Err(CouplingError::TiltedCapsule);
                                }
                                Boundary::Capsule {
                                    center,
                                    axis: axis.to_array(),
                                    radius,
                                    half_length,
                                    velocity: velocity.to_array(),
                                    angular_speed: axis.dot(DVec3::from(end.angular_velocity)),
                                }
                            }
                            ReceiverShape::Sphere(radius) => Boundary::Sphere {
                                center,
                                radius,
                                velocity: velocity.to_array(),
                                angular_velocity: end.angular_velocity,
                            },
                            ReceiverShape::Cuboid(half_extents)
                                if let Some(local) = static_box_pose =>
                            {
                                let rotation = DQuat::from_array(start.orientation);
                                Boundary::StaticCuboid {
                                    center: (DVec3::from_array(center)
                                        + rotation * DVec3::from_array(local.translation))
                                    .to_array(),
                                    half_extents,
                                    orientation: (rotation * DQuat::from_array(local.rotation))
                                        .to_array(),
                                }
                            }
                            ReceiverShape::Cuboid(half_extents) => Boundary::Cuboid {
                                center,
                                half_extents,
                                velocity: velocity.to_array(),
                                angular_velocity: end.angular_velocity,
                            },
                        },
                        mobility.map(|_| (body, end)),
                        path,
                    )
                }
            };
            self.boundaries.push(boundary);
            self.receivers.push(receiver);
            self.mobility.push(mobility);
            self.active_slots.push(slot);
        }
        if self.mobility.iter().all(Option::is_none) {
            let paths: Vec<_> = paths
                .iter()
                .map(|(i, p)| (*i, p as &dyn gz_physics_xpbd::BoundaryPath))
                .collect();
            for body in self.current.values_mut() {
                body.step_with_boundary_paths(
                    dt,
                    &self.boundaries,
                    &paths,
                    gz_physics_xpbd::DEFAULT_PATH_TOLERANCE,
                )?;
            }
            return Ok(());
        }
        let capsule_pair = self
            .surfaces
            .iter()
            .zip(&self.starts)
            .filter_map(|(s, start)| {
                if let (
                    Surface::Rigid {
                        body,
                        shape: ReceiverShape::Capsule { .. },
                        mobility: Some(_),
                        ..
                    },
                    Some(start),
                ) = (s, start)
                {
                    Some((*body, *start))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        // Preserve the committed soft configuration if a curved native pair
        // correction would introduce simultaneous rigid-rigid contact.
        let mut staged = (capsule_pair.len() == 2).then(|| self.current.clone());
        let mut bodies = staged
            .as_mut()
            .unwrap_or(&mut self.current)
            .values_mut()
            .collect::<Vec<_>>();
        let motions = step_coupled_rigid(&mut bodies, dt, &self.boundaries, &self.mobility)?;
        let mut updates = Vec::new();
        for (receiver, motion) in self.receivers.iter().zip(motions) {
            if let (Some((id, end)), Some(motion)) = (receiver, motion) {
                if motion.linear_velocity_correction == [0.; 3]
                    && motion.angular_velocity_correction == [0.; 3]
                    && motion.translation_correction == [0.; 3]
                    && motion.rotation_correction == [0., 0., 0., 1.]
                {
                    continue;
                }
                let add = |a, b| (DVec3::from_array(a) + DVec3::from_array(b)).to_array();
                updates.push((
                    *id,
                    BodyState {
                        position: add(end.position, motion.translation_correction),
                        orientation: (DQuat::from_array(motion.rotation_correction)
                            * DQuat::from_array(end.orientation))
                        .normalize()
                        .to_array(),
                        velocity: add(end.velocity, motion.linear_velocity_correction),
                        angular_velocity: add(
                            end.angular_velocity,
                            motion.angular_velocity_correction,
                        ),
                    },
                ));
            }
        }
        let backend = physics
            .downcast_mut::<MultiFidelityBackend>()
            .ok_or(CouplingError::Binding)?;
        if let [(aid, a0), (bid, b0)] = capsule_pair.as_slice() {
            if !updates.is_empty() {
                let at = |id| {
                    updates
                        .iter()
                        .find(|(i, _)| *i == id)
                        .map(|(_, s)| *s)
                        .or_else(|| backend.state(id))
                        .ok_or(CouplingError::Binding)
                };
                let both = [(*aid, *a0, at(*aid)?), (*bid, *b0, at(*bid)?)];
                backend.commit_soft_capsule_pair(both)?;
            }
            self.current = staged.expect("staged two-capsule world");
        } else if let [(id, state)] = updates.as_slice() {
            let shape = self.surfaces.iter().find_map(|surface| match surface {
                Surface::Rigid { body, shape, .. } if body == id => Some(shape),
                _ => None,
            });
            match shape {
                Some(ReceiverShape::Cuboid(_)) => {
                    backend.commit_soft_cuboid(*id, *state)?;
                }
                Some(ReceiverShape::Capsule { .. }) => {
                    backend.commit_soft_capsule(*id, *state)?;
                }
                _ => backend.commit_soft_spheres(&updates)?,
            }
        } else if !updates.is_empty() {
            backend.commit_soft_spheres(&updates)?;
        }
        if self.articulated_wrenches {
            self.queue_articulated_reactions(backend, dt)?;
        }
        Ok(())
    }

    /// Native has finished this tick. Apply the measured impulse over its next
    /// solve so joint constraints distribute the load through the articulation.
    fn queue_articulated_reactions(
        &self,
        backend: &mut MultiFidelityBackend,
        dt: Duration,
    ) -> Result<(), CouplingError> {
        for (boundary, &slot) in self.active_slots.iter().enumerate() {
            if !matches!(
                self.mobility[boundary],
                Some(RigidMobility::PrescribedCuboid { .. })
            ) {
                continue;
            }
            let (id, end) = self.receivers[boundary].ok_or(CouplingError::Binding)?;
            let start = self.starts[slot].ok_or(CouplingError::Binding)?;
            // XPBD records moments about the start center. Native expects
            // a world-space moment about the COM at the next solve's start.
            let offset = DVec3::from(start.position) - DVec3::from(end.position);
            let mut linear = DVec3::ZERO;
            let mut angular = DVec3::ZERO;
            for body in self.current.values() {
                let reaction = &body.last_boundary_impulses()[boundary];
                linear += DVec3::from(reaction.linear);
                angular += DVec3::from(reaction.angular);
            }
            if linear != DVec3::ZERO || angular != DVec3::ZERO {
                let force = linear / dt.as_secs_f64();
                let torque = (angular + offset.cross(linear)) / dt.as_secs_f64();
                backend.add_world_wrench(id, force.to_array(), torque.to_array())?;
            }
        }
        Ok(())
    }
}

#[cfg(feature = "distributed-physics")]
pub(crate) fn is_particle_visual(world: &World, id: u64) -> bool {
    world
        .get_resource::<Deformables>()
        .is_some_and(|s| s.visuals.iter().any(|v| v.visual == id))
}
