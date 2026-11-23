// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Atomic impulse at a geometric cable/cloth–articulated sphere contact time.
pub(crate) mod automatic;
mod recurrent;
mod support;
pub use support::{ArticulatedSupportInterval, SupportApplication};
#[cfg(feature = "distributed-physics")]
mod construction;
#[cfg(feature = "distributed-physics")]
mod rotor_construction;
use super::*;
use crate::{InputError, articulated_contact::loaded_link};
#[cfg(feature = "distributed-physics")]
pub use construction::{
    ArticulatedImpactBarrier, ArticulatedMaterialConstruction,
    AutomaticArticulatedMaterialConstruction, distributed_automatic_material_contact_tree_spec,
    distributed_material_contact_tree_spec,
};
use gz_physics_multifidelity::{ArticulatedContactSite, ArticulatedImpulseReport};
use gz_physics_xpbd::articulated_impact::{ImpactSphere, MaterialImpactReport, MaterialImpactTime};
#[cfg(feature = "distributed-physics")]
pub use rotor_construction::{RotorMaterialConstruction, distributed_rotor_material_contact_spec};
use std::{collections::BTreeSet, sync::Arc};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DetectedImpactTime {
    pub public_start: Duration,
    pub seconds_after_start: f64,
    pub rounding_distance: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ArticulatedMaterialImpact {
    pub time: Duration,
    pub detection: Option<DetectedImpactTime>,
    pub epoch: u64,
    /// Stable public link identifiers in the input order, indexed by material load.site.sphere.
    pub links: Vec<u64>,
    /// World witness in the local frame of its link, in metres, indexed in material load order.
    pub local_points: Vec<[f64; 3]>,
    /// World positions at the impulse, for particles that received a load.
    pub loaded_particles: Vec<(usize, [f64; 3])>,
    pub restitution: f64,
    pub material: MaterialImpactReport,
    pub native: ArticulatedImpulseReport,
}

#[derive(Debug, thiserror::Error)]
pub enum MaterialImpactError {
    #[error(transparent)]
    Input(#[from] InputError),
    #[error(transparent)]
    Physics(#[from] PhysicsError),
    #[error(transparent)]
    Material(#[from] gz_physics_xpbd::XpbdError),
    #[error(
        "a barrier impact requires one named finite-compliance material without other enrolled boundaries"
    )]
    Scene,
    #[error(
        "returning material requires a resolved isolated contact with finite separation speed and time budget"
    )]
    UnresolvedReturn,
    #[error(
        "gravity support requires converged unilateral normal load and isolated nonpenetrating features"
    )]
    UnresolvedSupport,
    #[error("authored articulated impact has no incoming contact at its scheduled time")]
    MissingIncoming,
}

/// Clone the original SDF material with its authored reset positions and velocities.
pub fn initial_material(sim: &Simulation, name: &str) -> Result<SoftBody, MaterialImpactError> {
    let soft = sim
        .world()
        .get_resource::<Deformables>()
        .ok_or(MaterialImpactError::Scene)?;
    let body = soft
        .initial
        .get(name)
        .filter(|body| body.supports_articulated_impact())
        .ok_or(MaterialImpactError::Scene)?;
    if soft.initial.len() != 1 || !soft.surfaces.is_empty() {
        return Err(MaterialImpactError::Scene);
    }
    Ok(body.clone())
}

/// Resolve a material impact at the current physical barrier. The caller supplies
/// an actual contact time. Every mesh primitive is checked against each selected
/// native sphere. A separated or separating manifold returns `None`; penetration
/// beyond geometric tolerance is a refusal. No elapsed time is consumed.
pub fn resolve(
    sim: &mut Simulation,
    material_name: &str,
    links: &[u64],
    restitution: f64,
) -> Result<Option<Arc<ArticulatedMaterialImpact>>, MaterialImpactError> {
    sim.validate_steps(0).map_err(InputError::from)?;
    let (time, epoch) = (sim.elapsed(), sim.epoch());
    resolve_world(
        sim.world_mut(),
        time,
        epoch,
        material_name,
        links,
        restitution,
        None,
    )
}

#[derive(Clone)]
pub(super) struct ScheduledImpact {
    pub returning_step: Option<Duration>,
    pub slip_transition_step: Option<Duration>,
    pub sustained: bool,
    pub material: String,
    pub links: Vec<u64>,
    pub restitution: f64,
}

/// Runs after native and deformable integration and before rigid state publication.
pub(super) fn at_barrier(world: &mut World) {
    let info = *world.resource::<UpdateInfo>();
    if info.dt.is_zero() || world.resource::<StepFault>().error().is_some() {
        return;
    }
    let Some(event) = world
        .resource::<Deformables>()
        .impact_barriers
        .get(&info.sim_time)
        .cloned()
    else {
        return;
    };
    let result = resolve_world(
        world,
        info.sim_time,
        info.epoch,
        &event.material,
        &event.links,
        event.restitution,
        None,
    );
    match result {
        Ok(Some(_)) => {}
        Ok(None) => world
            .resource::<StepFault>()
            .report(MaterialImpactError::MissingIncoming),
        Err(error) => world.resource::<StepFault>().report(error),
    }
}

fn resolve_world(
    world: &mut World,
    time: Duration,
    epoch: u64,
    material_name: &str,
    links: &[u64],
    restitution: f64,
    detected: Option<(&MaterialImpactTime, Duration)>,
) -> Result<Option<Arc<ArticulatedMaterialImpact>>, MaterialImpactError> {
    if !initial::matches_physics_owner(world) {
        return Err(InputError::InvalidPhysics.into());
    }
    if links.is_empty()
        || links.len() > gz_physics_multifidelity::MAX_ARTICULATED_CONTACT_SITES
        || links.iter().collect::<BTreeSet<_>>().len() != links.len()
        || !(0. ..=1.).contains(&restitution)
    {
        return Err(InputError::InvalidValue.into());
    }
    let bodies = links
        .iter()
        .map(|&id| loaded_link(world, id))
        .collect::<Result<Vec<_>, _>>()?;
    if !world.contains_resource::<Deformables>() {
        return Err(MaterialImpactError::Scene);
    }
    world.resource_scope(|world, mut soft: Mut<Deformables>| {
        if soft.current.len() != 1 || !soft.surfaces.is_empty() {
            return Err(MaterialImpactError::Scene);
        }
        let body = soft
            .current
            .get_mut(material_name)
            .ok_or(MaterialImpactError::Scene)?;
        let mut physics = world
            .get_resource_mut::<PhysicsBackend>()
            .ok_or(InputError::InvalidPhysics)?;
        let native = physics
            .downcast_mut::<MultiFidelityBackend>()
            .ok_or(InputError::InvalidPhysics)?;
        let args = ImpactContext {
            time,
            epoch,
            bodies: &bodies,
            links,
            restitution,
            detected,
        };
        let Some(impact) = apply_at_cut(native, body, &args)? else {
            return Ok(None);
        };
        if detected.is_some() {
            return Ok(Some(impact));
        }
        let soft = &mut *soft;
        let sample = soft
            .samples
            .get_mut(material_name)
            .expect("registered material sample");
        sample
            .particles
            .copy_from_slice(soft.current[material_name].particles());
        sample.record_impact(impact.clone());
        Ok(Some(impact))
    })
}

struct ImpactContext<'a> {
    time: Duration,
    epoch: u64,
    bodies: &'a [gz_physics::BodyId],
    links: &'a [u64],
    restitution: f64,
    detected: Option<(&'a MaterialImpactTime, Duration)>,
}
trait NativeMaterialContact {
    fn sphere(&self, id: gz_physics::BodyId) -> Result<([f64; 3], f64), PhysicsError>;
    fn body_state(&self, id: gz_physics::BodyId) -> Option<gz_physics::BodyState>;
    fn contacts(
        &mut self,
        sites: &[ArticulatedContactSite],
    ) -> Result<gz_physics_multifidelity::ArticulatedContactTransaction<'_>, PhysicsError>;
}
impl NativeMaterialContact for MultiFidelityBackend {
    fn sphere(&self, id: gz_physics::BodyId) -> Result<([f64; 3], f64), PhysicsError> {
        self.articulated_sphere_geometry(id)
    }
    fn body_state(&self, id: gz_physics::BodyId) -> Option<gz_physics::BodyState> {
        self.state(id)
    }
    fn contacts(
        &mut self,
        s: &[ArticulatedContactSite],
    ) -> Result<gz_physics_multifidelity::ArticulatedContactTransaction<'_>, PhysicsError> {
        self.prepare_articulated_contacts(s)
    }
}
impl NativeMaterialContact for gz_physics_multifidelity::ArticulatedNativeInterval<'_> {
    fn sphere(&self, id: gz_physics::BodyId) -> Result<([f64; 3], f64), PhysicsError> {
        self.sphere_geometry(id)
    }
    fn body_state(&self, id: gz_physics::BodyId) -> Option<gz_physics::BodyState> {
        self.state(id)
    }
    fn contacts(
        &mut self,
        s: &[ArticulatedContactSite],
    ) -> Result<gz_physics_multifidelity::ArticulatedContactTransaction<'_>, PhysicsError> {
        self.prepare(s)
    }
}
fn apply_at_cut(
    native: &mut impl NativeMaterialContact,
    body: &mut SoftBody,
    context: &ImpactContext<'_>,
) -> Result<Option<Arc<ArticulatedMaterialImpact>>, MaterialImpactError> {
    let ImpactContext {
        time,
        epoch,
        bodies,
        links,
        restitution,
        detected,
    } = *context;
    let mut geometry = Vec::with_capacity(bodies.len());
    let mut poses = Vec::with_capacity(bodies.len());
    for &id in bodies {
        let (center, radius) = native.sphere(id)?;
        geometry.push(ImpactSphere { center, radius });
        poses.push(native.body_state(id).ok_or(PhysicsError::MissingBody)?);
    }
    let material = if let Some((cut, _)) = detected {
        body.sphere_impact_at_detected_barrier(&geometry, cut)?
    } else {
        body.sphere_impact_at_barrier(&geometry)?
    };
    if material.sites().is_empty() {
        return Ok(None);
    }
    let sites: Vec<_> = material
        .sites()
        .iter()
        .map(|s| {
            let state = poses[s.sphere];
            let p = DVec3::from(s.point) - DVec3::from(state.position);
            ArticulatedContactSite {
                body: bodies[s.sphere],
                local_point: (DQuat::from_array(state.orientation).conjugate() * p).to_array(),
            }
        })
        .collect();
    let transaction = native.contacts(&sites)?;
    let basis = transaction.sample();
    let plan = material.prepare(
        &basis.velocities,
        |i, j| {
            basis
                .mobility(i, j)
                .expect("matching sampled material and native sites")
        },
        restitution,
    )?;
    let impulses = plan
        .report()
        .loads
        .iter()
        .map(|load| load.on_rigid)
        .collect::<Vec<_>>();
    if impulses.iter().all(|&j| j == [0.; 3]) {
        return Ok(None);
    }
    let rigid = transaction.commit(&impulses)?;
    let material = plan.commit();
    let loaded_particles = material
        .loads
        .iter()
        .flat_map(|load| load.on_particles.iter().map(|(id, _)| *id))
        .collect::<BTreeSet<_>>()
        .into_iter()
        .map(|id| (id, body.particles()[id].position))
        .collect();
    let impact = Arc::new(ArticulatedMaterialImpact {
        loaded_particles,
        time,
        detection: detected.map(|(c, public_start)| DetectedImpactTime {
            public_start,
            seconds_after_start: c.seconds(),
            rounding_distance: c.rounding_distance(),
        }),
        epoch,
        links: links.to_vec(),
        local_points: sites.iter().map(|s| s.local_point).collect(),
        restitution,
        material,
        native: rigid,
    });
    Ok(Some(impact))
}
