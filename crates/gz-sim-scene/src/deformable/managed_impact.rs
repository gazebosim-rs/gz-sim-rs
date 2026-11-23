// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! An ordinary Bevy world whose automatic native/XPBD contact has one managed publication.
use super::*;
use crate::{LoadError, PhysicsEngine, WorldLoadOptions, WorldRuntimeOptions};
use gz_physics_distributed::floating_tree::{
    AutomaticConstruction, AutomaticImpactReceipt, AutomaticMaterialPool, AutomaticMaterialSpec,
    AutomaticPool, AutomaticRotorMaterialSpec, ContactTreeSpec, RotorMaterialPool,
};
use gz_physics_multifidelity::{ArticulatedContactSample, ArticulatedImpulseReport};
use gz_physics_xpbd::{
    MaterialFeature,
    articulated_impact::{MaterialImpactLoad, MaterialImpactReport, MaterialImpactSite},
};
use std::{
    path::Path,
    sync::{Arc, Mutex},
};

pub(crate) enum MaterialOwner {
    Free(Box<AutomaticMaterialPool>),
    Rotor(Box<RotorMaterialPool>),
}
macro_rules! with_owner {
    ($owner:expr, |$p:ident| $body:expr) => {
        match $owner {
            MaterialOwner::Free($p) => $body,
            MaterialOwner::Rotor($p) => $body,
        }
    };
}
enum Construction {
    Free(AutomaticMaterialSpec),
    Rotor(Box<AutomaticRotorMaterialSpec>),
}
/// The physical joint, body, particle and contact observations share this clock and process lease.
#[derive(Resource)]
pub struct ManagedAutomaticMaterial(Mutex<MaterialOwner>);
impl ManagedAutomaticMaterial {
    pub fn owner(&self) -> u8 {
        with_owner!(&*self.0.lock().unwrap(), |p| p.owner())
    }
    pub fn worker_pid(&self, slot: u8) -> Option<u32> {
        with_owner!(&*self.0.lock().unwrap(), |p| p.worker_pid(slot))
    }
    pub fn generation(&self) -> u64 {
        with_owner!(&*self.0.lock().unwrap(), |p| p.generation())
    }
    pub fn interval(&self) -> (u64, u64) {
        let p = self.0.lock().unwrap();
        with_owner!(&*p, |p| (p.epoch(), p.tick()))
    }
}
type Check<T = ()> = Result<T, Box<dyn std::error::Error + Send + Sync>>;
fn load_error(world: &str, e: impl std::fmt::Display) -> LoadError {
    LoadError::Deformable {
        world: world.into(),
        reason: e.to_string(),
    }
}
fn construction(
    xml: &str,
    source: &str,
    world: &str,
) -> Result<(Construction, SoftBody), LoadError> {
    let s = match articulated_impact::distributed_automatic_material_contact_tree_spec(
        xml, source, world,
    ) {
        Ok(s) => s,
        Err(tree_reason) => {
            if let Ok(r) =
                articulated_impact::distributed_rotor_material_contact_spec(xml, source, world)
            {
                return Ok((
                    Construction::Rotor(Box::new(AutomaticRotorMaterialSpec {
                        pad: r.pad,
                        material_name: r.material_name,
                        step_ns: r.step.as_nanos() as u64,
                        restitution: r.restitution,
                    })),
                    r.material,
                ));
            }
            return Err(tree_reason);
        }
    };
    Ok((
        Construction::Free(AutomaticMaterialSpec {
            tree: ContactTreeSpec {
                tree: s.tree,
                limits: s.limits,
            },
            material_name: s.material_name,
            step_ns: s.step.as_nanos() as u64,
            links: s.links,
            restitution: s.restitution,
        }),
        s.material,
    ))
}
pub(crate) fn transient(
    xml: &str,
    source: &str,
    world: &str,
    exe: &Path,
) -> Result<MaterialOwner, LoadError> {
    match construction(xml, source, world)? {
        (Construction::Free(s), cold) => AutomaticMaterialPool::transient(exe, 2, s, cold)
            .map(|p| MaterialOwner::Free(Box::new(p))),
        (Construction::Rotor(s), cold) => RotorMaterialPool::transient(exe, 2, *s, cold)
            .map(|p| MaterialOwner::Rotor(Box::new(p))),
    }
    .map_err(|e| load_error(world, e))
}
pub(crate) fn install(
    sim: &mut Simulation,
    backend: PhysicsBackend,
    owner: MaterialOwner,
    world: &str,
) -> Result<(), LoadError> {
    if sim.step_duration().as_nanos() != u128::from(with_owner!(&owner, |p| p.spec().interval_ns()))
    {
        return Err(load_error(world, "managed interval differs from SDF step"));
    }
    sim.world_mut()
        .insert_resource(ManagedAutomaticMaterial(Mutex::new(owner)));
    gz_physics::install_with_step(sim, backend, step)?;
    sim.add_systems(Phase::Reset, reset.after(gz_physics::PhysicsReset));
    Ok(())
}
/// Move the committed native graph, including the actual last subinterval, to another real owner.
pub fn transfer(sim: &mut Simulation, epoch: u64, tick: u64, slot: u8) -> Result<(), String> {
    sim.committed_update_info().map_err(|e| e.to_string())?;
    let mut m = sim
        .world_mut()
        .get_resource_mut::<ManagedAutomaticMaterial>()
        .ok_or("world has no managed material owner")?;
    let p = m.0.get_mut().unwrap();
    with_owner!(p, |p| p
        .check_interval(epoch, tick)
        .and_then(|()| p.transfer(slot)))
    .map_err(|e| e.to_string())
}
/// Create a durable session or reopen the existing barrier against the supplied physical SDF.
/// Warm bodies, material, cumulative first-impact observation and public clock are restored together.
pub fn load_persistent(
    xml: &str,
    source: &str,
    world: &str,
    exe: &Path,
    directory: &Path,
    width: u8,
) -> Result<Simulation, LoadError> {
    open_persistent(xml, source, world, exe, directory, Some(width), None)
}
pub(crate) fn open_persistent(
    xml: &str,
    source: &str,
    world: &str,
    exe: &Path,
    directory: &Path,
    width: Option<u8>,
    step: Option<Duration>,
) -> Result<Simulation, LoadError> {
    fn open_as<S: AutomaticConstruction>(
        exe: &Path,
        directory: &Path,
        width: Option<u8>,
        step: Option<Duration>,
        spec: S,
        cold: SoftBody,
        file: &str,
    ) -> Check<AutomaticPool<S>> {
        if step.is_some_and(|v| v != Duration::from_nanos(spec.interval_ns())) {
            return Err("requested step differs from the managed material SDF".into());
        }
        if directory.join(file).try_exists()? {
            let p = AutomaticPool::resume(exe, directory, cold)?;
            if p.spec() != &spec || width.is_some_and(|w| w != p.width()) {
                return Err("saved material construction or worker count differs".into());
            }
            Ok(p)
        } else {
            Ok(AutomaticPool::create(
                exe,
                directory,
                width.unwrap_or(2),
                spec,
                cold,
            )?)
        }
    }
    let p = match construction(xml, source, world)? {
        (Construction::Free(s), cold) => open_as(
            exe,
            directory,
            width,
            step,
            s,
            cold,
            "barrier.gzautmaterial",
        )
        .map(|v| MaterialOwner::Free(Box::new(v))),
        (Construction::Rotor(s), cold) => open_as(
            exe,
            directory,
            width,
            step,
            *s,
            cold,
            "barrier.gzrotmaterial",
        )
        .map(|v| MaterialOwner::Rotor(Box::new(v))),
    }
    .map_err(|e| load_error(world, e))?;
    let (epoch, tick, length) = with_owner!(&p, |v| (v.epoch(), v.tick(), v.spec().interval_ns()));
    let nanos = tick
        .checked_mul(length)
        .ok_or_else(|| load_error(world, "material clock exhausted"))?;
    let time = Duration::from_nanos(nanos);
    let dt = if tick == 0 {
        Duration::ZERO
    } else {
        Duration::from_nanos(length)
    };
    let info = UpdateInfo {
        epoch,
        iteration: tick,
        sim_time: time,
        dt,
        start_time: time - dt,
        paused: tick == 0,
    };
    let mut sim = crate::load_world_with_automatic_owner(
        xml,
        source,
        world,
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: None,
            noise_seed: [0; 32],
            distributed_worker: Some(exe),
        },
        WorldRuntimeOptions::default(),
        Some((info, true)),
        Some(p),
    )?;
    sim.world_mut()
        .resource_scope(|w, mut m: Mut<ManagedAutomaticMaterial>| {
            with_owner!(m.0.get_mut().unwrap(), |p| publish(w, p, true))
        })
        .map_err(|e| load_error(world, e))?;
    // An initial SDF load has already installed every authored visual at its exact
    // construction transform. Only warm reopens need native pose propagation.
    if info.iteration == 0 {
        return Ok(sim);
    }
    let mut schedule = Schedule::default();
    schedule.add_systems(
        (
            crate::sync_bodies,
            crate::sync_models,
            crate::sync_local_poses,
            crate::sync_attached_poses,
            particle_visual::sync,
        )
            .chain(),
    );
    schedule.run(sim.world_mut());
    super::worker::sync_reopened(&mut sim);
    Ok(sim)
}
fn report(
    p: &AutomaticPool<impl AutomaticConstruction>,
    r: &AutomaticImpactReceipt,
) -> Check<Arc<articulated_impact::ArticulatedMaterialImpact>> {
    let c = &r.contact;
    let b = &c.basis;
    let n = c.loads.len();
    let blocks = (0..n)
        .flat_map(|i| (0..n).map(move |j| b.mobility(i, j).expect("checked durable square basis")))
        .collect();
    let before = ArticulatedContactSample::from_measurement(
        b.world_points.clone(),
        b.velocities.clone(),
        b.dofs,
        b.links,
        blocks,
    )?;
    let loads = c
        .loads
        .iter()
        .map(|v| {
            let sphere = p
                .spec()
                .receivers()
                .iter()
                .position(|id| *id == v.link)
                .ok_or("missing material receiver")?;
            let feature = match v.feature.as_slice() {
                [i] => MaterialFeature::Vertex(*i),
                [a, b] => MaterialFeature::Segment([*a, *b]),
                [a, b, c] => MaterialFeature::Triangle([*a, *b, *c]),
                _ => return Err("invalid material feature"),
            };
            Ok(MaterialImpactLoad {
                site: MaterialImpactSite {
                    sphere,
                    feature,
                    weights: v.weights,
                    point: v.point,
                    normal: v.normal,
                    clearance: v.clearance,
                },
                initial_normal_velocity: v.initial_normal_velocity,
                final_normal_velocity: v.final_normal_velocity,
                on_rigid: v.on_link,
                on_particles: v.on_particles.clone(),
                friction: None,
            })
        })
        .collect::<Result<Vec<_>, &str>>()?;
    let start = Duration::from_nanos(r.input_tick * p.spec().interval_ns());
    let loaded_particles = c
        .loads
        .iter()
        .flat_map(|l| l.on_particles.iter().map(|(i, _)| *i))
        .collect::<std::collections::BTreeSet<_>>()
        .into_iter()
        .map(|i| (i, r.head_particles[i].0))
        .collect();
    Ok(Arc::new(articulated_impact::ArticulatedMaterialImpact {
        loaded_particles,
        time: start + Duration::from_nanos(r.head_ns),
        detection: Some(articulated_impact::DetectedImpactTime {
            public_start: start,
            seconds_after_start: r.continuous_seconds,
            rounding_distance: r.rounding_distance,
        }),
        epoch: p.epoch(),
        links: p.spec().receivers().to_vec(),
        local_points: c.loads.iter().map(|l| l.local_point).collect(),
        restitution: c.restitution,
        material: MaterialImpactReport {
            loads,
            kinetic_energy_change: c.kinetic_energy_change,
            prescribed_support_work: c.prescribed_support_work,
            dissipated_energy: c.dissipated_energy,
        },
        native: ArticulatedImpulseReport {
            before,
            after_velocities: c.native.after_velocities.clone(),
            kinetic_energy_change: c.native.kinetic_energy_change,
        },
    }))
}
fn publish(
    world: &mut World,
    p: &AutomaticPool<impl AutomaticConstruction>,
    reopened: bool,
) -> Check {
    if let Some(image) = p.native_image() {
        let mut phys = world.resource_mut::<PhysicsBackend>();
        let be = phys
            .downcast_mut::<MultiFidelityBackend>()
            .ok_or("native material observer replaced")?;
        if (if p.spec().world_supported() {
            be.restore_timed_world_revolute(image)
        } else {
            be.restore_timed_revolute_tree(image)
        })? != p.last_native_duration()
        {
            return Err("native material duration changed during observation".into());
        }
    }
    let mut soft = world.resource_mut::<Deformables>();
    let name = p.spec().material();
    soft.current
        .get_mut(name)
        .ok_or("missing managed XPBD body")?
        .restore_articulated_material(p.material_image())?;
    soft.publish(
        Duration::from_nanos(p.tick() * p.spec().interval_ns()),
        p.epoch(),
    );
    if let Some(hit) = p.last_impact()
        && (reopened || hit.contact.tick == p.tick())
    {
        let r = report(p, hit)?;
        let sample = soft
            .samples
            .get_mut(name)
            .ok_or("missing managed material sample")?;
        if hit.contact.tick == p.tick() {
            sample.record_impact(r);
        } else {
            sample.impact_count = 1;
            sample.latest_articulated_impact = Some(r);
        }
        soft.automatic_consumed = true;
    }
    Ok(())
}
fn tick(world: &mut World, p: &mut AutomaticPool<impl AutomaticConstruction>) -> Check {
    let info = *world.resource::<UpdateInfo>();
    if info.dt.is_zero() {
        return Ok(());
    }
    if !initial::matches_physics_owner(world) {
        return Err("managed material physics owner replaced".into());
    }
    let soft = world.resource::<Deformables>();
    let local = soft
        .current
        .get(p.spec().material())
        .ok_or("managed material body was removed")?;
    if local.checkpoint_articulated_material()?.as_slice() != p.material_image() {
        return Err("local XPBD observer diverged before the managed interval".into());
    }
    let n = world
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .ok_or("missing native material observer")?;
    if let Some(image) = p.native_image() {
        if (if p.spec().world_supported() {
            n.checkpoint_timed_world_revolute(p.last_native_duration())
        } else {
            n.checkpoint_timed_revolute_tree(p.last_native_duration())
        })? != image
        {
            return Err(
                "local native observer diverged before the managed material interval".into(),
            );
        }
    } else {
        let ids = p
            .spec()
            .receivers()
            .iter()
            .map(|id| crate::articulated_contact::loaded_link(world, *id))
            .collect::<Result<Vec<_>, _>>()?;
        if p.spec().world_supported() {
            n.articulated_sweep(&ids, info.dt)?;
        } else {
            n.articulated_translation(&ids, info.dt)?;
        }
        for row in &p.observation().links {
            let id = crate::articulated_contact::loaded_link(world, row.id)?;
            let b = n.state(id).ok_or("missing native material body")?;
            if (b.position, b.orientation, b.velocity, b.angular_velocity)
                != (
                    row.position,
                    row.orientation,
                    row.velocity,
                    row.angular_velocity,
                )
            {
                return Err("cold native material observations differ".into());
            }
        }
    }
    p.advance_at(
        info.epoch,
        info.iteration
            .checked_sub(1)
            .ok_or("invalid material clock")?,
    )?;
    publish(world, p, false)
}
fn step(world: &mut World) {
    if let Err(e) = world.resource_scope(|w, mut m: Mut<ManagedAutomaticMaterial>| {
        with_owner!(m.0.get_mut().unwrap(), |p| tick(w, p))
    }) {
        world
            .resource::<StepFault>()
            .report(std::io::Error::other(e));
    }
}
fn reset(
    info: Res<UpdateInfo>,
    mut owner: ResMut<ManagedAutomaticMaterial>,
    fault: Res<StepFault>,
) {
    let p = owner.0.get_mut().unwrap();
    if with_owner!(&*p, |v| v.epoch()).checked_add(1) != Some(info.epoch) {
        fault.report(std::io::Error::other("stale managed material reset"));
        return;
    }
    if let Err(e) = with_owner!(p, |p| p.reset()) {
        fault.report(e);
    }
}
