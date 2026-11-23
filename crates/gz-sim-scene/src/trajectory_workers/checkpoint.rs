// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Immutable operator checkpoints for self-contained curved worlds and their ideal IMUs.
use super::TrajectoryProcessPool;
use crate::passive_checkpoint::Clock;
use crate::*;
use gz_physics_distributed::trajectory::{
    SCENE_MANIFEST_LIMIT, material, read_scene_cut, save_scene_cut,
};
use gz_sim_core::entity_by_id;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{path::Path, sync::Arc};
#[derive(Resource)]
struct Source {
    xml: Arc<str>,
    world: String,
    step: Option<Duration>,
    seed: [u8; 32],
    revision: u64,
    bodies: [u8; 32],
    identities: [u8; 32],
}
fn native(sim: &Simulation) -> &MultiFidelityBackend {
    sim.world()
        .resource::<PhysicsBackend>()
        .downcast_ref::<MultiFidelityBackend>()
        .expect("native owner")
}
fn identities(world: &World) -> [u8; 32] {
    let mut rows: Vec<_> = world
        .iter_entities()
        .filter_map(|e| {
            Some((
                e.get::<EntityId>()?.get(),
                e.get::<Name>().map(|n| n.0.as_str()).unwrap_or(""),
            ))
        })
        .collect();
    rows.sort_unstable();
    let mut h = Sha256::new();
    h.update(
        gz_sim_core::next_entity_id(world)
            .unwrap_or(u64::MAX)
            .to_le_bytes(),
    );
    for (id, name) in rows {
        h.update(id.to_le_bytes());
        h.update((name.len() as u64).to_le_bytes());
        h.update(name.as_bytes());
    }
    h.finalize().into()
}
pub(crate) fn bind(
    sim: &mut Simulation,
    xml: &str,
    world: &str,
    step: Option<Duration>,
    seed: [u8; 32],
) {
    let cold = native(sim)
        .trajectory_construction()
        .expect("validated when installing workers");
    let src = Source {
        xml: Arc::from(xml),
        world: world.into(),
        step,
        seed,
        revision: sim.schedule_revision(),
        bodies: Sha256::digest(cold).into(),
        identities: identities(sim.world()),
    };
    sim.world_mut().insert_resource(src);
}
#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("trajectory scene checkpoint: {0}")]
    Domain(&'static str),
    #[error(transparent)]
    Load(#[from] LoadError),
    #[error(transparent)]
    Archive(#[from] gz_physics_distributed::coordinator::RecoveryError),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Clock(#[from] gz_sim_core::StepError),
    #[error(transparent)]
    Physics(#[from] gz_physics::PhysicsError),
    #[error(transparent)]
    Input(#[from] InputError),
    #[error(transparent)]
    Material(#[from] gz_physics_distributed::articulation::ArticulationError),
}
type Result<T> = std::result::Result<T, Error>;
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Reading {
    time: Duration,
    epoch: u64,
    q: [f64; 4],
    a: [f64; 3],
    w: [f64; 3],
    wc: [f64; 3],
    ac: [f64; 3],
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sensor {
    id: u64,
    link: u64,
    rate_bits: u64,
    active: bool,
    reference: [f64; 4],
    previous: [f64; 3],
    schedule: SampleSchedule,
    sample: Option<Reading>,
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Cut {
    xml: String,
    world: String,
    step: Option<Duration>,
    seed: [u8; 32],
    clock: Clock,
    owner: usize,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    group_owners: Vec<usize>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    transports: Vec<super::WorkerTransport>,
    identities: [u8; 32],
    geometry: [u8; 32],
    sensors: Vec<Sensor>,
    queued: Vec<inputs::SavedInput>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    material: Vec<(String, gz_physics_xpbd::OneWayState)>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    material_image_v1: Option<Vec<String>>,
}
fn domain(xml: &str) -> Result<()> {
    if xml.len() > SCENE_MANIFEST_LIMIT {
        return Err(Error::Domain("SDF exceeds the scene manifest limit"));
    }
    let d = roxmltree::Document::parse(xml).map_err(|_| Error::Domain("invalid SDF"))?;
    for n in d.descendants().filter(|n| n.is_element()) {
        let name = n.tag_name().name();
        if n.tag_name().namespace().is_some()
            || !matches!(
                name,
                "sdf"
                    | "world"
                    | "physics"
                    | "max_step_size"
                    | "real_time_factor"
                    | "gravity"
                    | "model"
                    | "static"
                    | "allow_auto_disable"
                    | "pose"
                    | "link"
                    | "inertial"
                    | "mass"
                    | "inertia"
                    | "ixx"
                    | "iyy"
                    | "izz"
                    | "ixy"
                    | "ixz"
                    | "iyz"
                    | "collision"
                    | "visual"
                    | "geometry"
                    | "box"
                    | "size"
                    | "sphere"
                    | "radius"
                    | "capsule"
                    | "cylinder"
                    | "length"
                    | "material"
                    | "diffuse"
                    | "ambient"
                    | "specular"
                    | "emissive"
                    | "surface"
                    | "friction"
                    | "ode"
                    | "mu"
                    | "mu2"
                    | "plugin"
                    | "trajectory"
                    | "waypoint"
                    | "linear_velocity"
                    | "angular_velocity"
                    | "sensor"
                    | "always_on"
                    | "update_rate"
                    | "imu"
                    | "topic"
                    | "cable"
                    | "cloth"
                    | "particle"
                    | "particle_visual"
                    | "plane"
            )
        {
            return Err(Error::Domain(
                "checkpoint requires a self-contained rigid primitive world with ideal IMUs",
            ));
        }
        if name == "sensor" && n.attribute("type") != Some("imu") {
            return Err(Error::Domain(
                "only ideal IMUs have a saved sensor continuation",
            ));
        }
        if name == "plugin"
            && !matches!(
                (n.attribute("name"), n.attribute("filename")),
                (
                    Some("gz::sim::systems::PrescribedMotion"),
                    Some("gz-sim-prescribed-motion-system")
                ) | (
                    Some("gz::sim::systems::InitialVelocity"),
                    Some("gz-sim-initial-velocity-system")
                ) | (
                    Some("gz::sim::systems::Deformables"),
                    Some("gz-sim-deformables-system")
                )
            )
        {
            return Err(Error::Domain("plugin has no trajectory checkpoint state"));
        }
    }
    Ok(())
}
fn geometry(world: &World) -> [u8; 32] {
    let mut rows = Vec::new();
    for e in world.iter_entities() {
        let (Some(id), Some(p)) = (e.get::<EntityId>(), e.get::<WorldPose>()) else {
            continue;
        };
        let mut values = Vec::from(p.translation);
        values.extend(p.rotation);
        if let Some(v) = e.get::<Velocity>() {
            values.extend(v.linear);
            values.extend(v.angular);
        }
        rows.push((id.get(), values));
    }
    rows.sort_unstable_by_key(|r| r.0);
    let mut h = Sha256::new();
    for (id, v) in rows {
        h.update(id.to_le_bytes());
        for x in v {
            h.update(x.to_le_bytes());
        }
    }
    h.finalize().into()
}
/// Includes the committed clock, complete native graph, public identities, queued velocity/wrench
/// commands and the phase, derivative history and last observation of each ideal IMU.
pub fn save(sim: &Simulation, path: &Path) -> Result<()> {
    let src = sim
        .world()
        .get_resource::<Source>()
        .ok_or(Error::Domain("requires an ordinary curved worker world"))?;
    domain(&src.xml)?;
    sim.committed_update_info()?;
    if !initial::matches_physics_owner(sim.world())
        || !sim
            .world()
            .get_resource::<initial::InitialScene>()
            .is_some_and(|initial| initial.trajectory_metadata_matches(sim.world()))
        || sim.schedule_revision() != src.revision
        || identities(sim.world()) != src.identities
        || Sha256::digest(native(sim).trajectory_construction()?)[..] != src.bodies
    {
        return Err(Error::Domain(
            "schedule, population or native construction changed since loading",
        ));
    }
    let mut sensors = Vec::new();
    for e in sim.world().iter_entities() {
        let Some(imu) = e.get::<Imu>() else { continue };
        if imu.noise.is_some() {
            return Err(Error::Domain(
                "IMU noise history requires its own continuation",
            ));
        }
        let sample = e
            .get::<ImuSample>()
            .map(|s| -> Result<_> {
                let r = s
                    .reading
                    .as_ref()
                    .map_err(|_| Error::Domain("IMU has an invalid reading"))?;
                Ok(Reading {
                    time: s.time,
                    epoch: s.epoch,
                    q: r.orientation,
                    a: r.specific_force,
                    w: r.angular_velocity,
                    wc: r.angular_velocity_covariance,
                    ac: r.linear_acceleration_covariance,
                })
            })
            .transpose()?;
        sensors.push(Sensor {
            id: e
                .get::<EntityId>()
                .ok_or(Error::Domain("unbound IMU"))?
                .get(),
            link: imu.link,
            active: imu.active,
            rate_bits: imu.rate_hz.to_bits(),
            reference: imu.reference.to_array(),
            previous: imu.previous_velocity.to_array(),
            schedule: imu.schedule.clone(),
            sample,
        });
    }
    sensors.sort_unstable_by_key(|s| s.id);
    let mut queued = inputs::saved_inputs(sim.world());
    queued.sort_unstable_by_key(|i| i.0);
    let material = sim
        .world()
        .get_resource::<deformable::Deformables>()
        .map(|s| s.saved_material())
        .transpose()?
        .unwrap_or_default();
    let native_image = native(sim).checkpoint_trajectory()?;
    let (material_image_v1, state) = if material.is_empty() {
        (None, native_image)
    } else {
        let (names, rows) = material.into_iter().unzip();
        (Some(names), material::pack_scene_state(native_image, rows)?)
    };
    let c = Cut {
        material_image_v1,
        material: Vec::new(),
        xml: src.xml.to_string(),
        world: src.world.clone(),
        step: src.step,
        seed: src.seed,
        clock: Clock::from_simulation(sim),
        owner: sim.world().resource::<TrajectoryProcessPool>().owner(),
        transports: sim.world().resource::<TrajectoryProcessPool>().transports(),
        group_owners: {
            let owners = sim
                .world()
                .resource::<TrajectoryProcessPool>()
                .material_owners();
            if owners.len() > 1 { owners } else { Vec::new() }
        },
        identities: src.identities,
        geometry: geometry(sim.world()),
        sensors,
        queued,
    };
    save_scene_cut(path, &serde_json::to_vec(&c)?, &state)?;
    Ok(())
}
/// Rebuild the authored SDF and validate its complete publication before returning a runnable world.
pub fn open(path: &Path, executable: &Path) -> Result<Simulation> {
    open_checked(path, executable, None, Default::default())
}

/// Restore only if the operator supplied the same world, XML, clock override and noise seed.
/// Runtime options are validated by the normal SDF loader before starting its workers.
pub fn resume(
    xml: &str,
    world: &str,
    options: WorldLoadOptions<'_>,
    runtime: WorldRuntimeOptions<'_>,
    path: &Path,
) -> Result<Simulation> {
    let worker = options
        .distributed_worker
        .ok_or(Error::Domain("missing curve worker executable"))?;
    open_checked(path, worker, Some((xml, world, options)), runtime)
}

fn open_checked(
    path: &Path,
    executable: &Path,
    expected: Option<(&str, &str, WorldLoadOptions<'_>)>,
    runtime: WorldRuntimeOptions<'_>,
) -> Result<Simulation> {
    let (meta, image) = read_scene_cut(path)?;
    let c: Cut = serde_json::from_slice(&meta)?;
    if let Some((xml, world, options)) = expected
        && (xml != c.xml
            || world != c.world
            || options.step != c.step
            || options.noise_seed != c.seed
            || options.engine != PhysicsEngine::MultiFidelity)
    {
        return Err(Error::Domain(
            "checkpoint SDF, world or execution settings differ",
        ));
    }
    let (image, rows) = if let Some(names) = c.material_image_v1 {
        if !c.material.is_empty()
            || names.is_empty()
            || names.len() > 32
            || names.windows(2).any(|p| p[0] >= p[1])
        {
            return Err(Error::Domain(
                "conflicting or unordered durable material identities",
            ));
        }
        let (native, rows) = material::unpack_scene_state(&image)?;
        if rows.len() != names.len() {
            return Err(Error::Domain("durable material row count differs"));
        }
        (native, names.into_iter().zip(rows).collect())
    } else {
        (image, c.material)
    };
    domain(&c.xml)?;
    let info = c.clock.info();
    let mut sim = load_world_at_clock(
        &c.xml,
        "recovered-curved-world.sdf",
        &c.world,
        &sdformat::NoResources,
        WorldLoadOptions {
            engine: PhysicsEngine::MultiFidelity,
            step: c.step,
            noise_seed: c.seed,
            distributed_worker: Some(executable),
        },
        runtime,
        Some((info, c.clock.paused)),
    )?;
    if identities(sim.world()) != c.identities {
        return Err(Error::Domain("restored public identities differ"));
    }
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .downcast_mut::<MultiFidelityBackend>()
        .unwrap()
        .restore_trajectory(&image)?;
    let mut publish = Schedule::default();
    publish.add_systems(
        (
            sync_bodies,
            sync_models,
            sync_local_poses,
            sync_attached_poses,
        )
            .chain(),
    );
    publish.run(sim.world_mut());
    if let Some(mut s) = sim
        .world_mut()
        .get_resource_mut::<deformable::Deformables>()
    {
        s.restore_material(rows, info.sim_time, info.epoch)?;
        deformable::worker::sync_reopened(&mut sim);
    } else if !rows.is_empty() {
        return Err(Error::Domain("recovered SDF has no matching material"));
    }
    if geometry(sim.world()) != c.geometry {
        return Err(Error::Domain("restored Bevy publication differs"));
    }
    if sim
        .world()
        .iter_entities()
        .filter(|e| e.contains::<Imu>())
        .count()
        != c.sensors.len()
    {
        return Err(Error::Domain("restored IMU population differs"));
    }
    let mut previous_id = None;
    for s in c.sensors {
        if previous_id.is_some_and(|id| id >= s.id) {
            return Err(Error::Domain("invalid IMU identity order"));
        }
        previous_id = Some(s.id);
        let e = entity_by_id(sim.world(), s.id).ok_or(Error::Domain("missing sensor identity"))?;
        let mut imu = sim
            .world_mut()
            .get_mut::<Imu>(e)
            .ok_or(Error::Domain("missing restored IMU"))?;
        if imu.link != s.link
            || imu.noise.is_some()
            || !s.previous.into_iter().all(f64::is_finite)
            || !s.reference.into_iter().all(f64::is_finite)
        {
            return Err(Error::Domain("invalid IMU binding or motion"));
        }
        imu.rate_hz = f64::from_bits(s.rate_bits);
        imu.active = s.active;
        imu.reference = DQuat::from_array(s.reference);
        imu.previous_velocity = s.previous.into();
        imu.schedule = s.schedule;
        if let Some(r) = s.sample {
            if r.epoch != info.epoch || r.time > info.sim_time {
                return Err(Error::Domain("IMU sample is outside committed clock"));
            }
            sim.world_mut().entity_mut(e).insert(ImuSample {
                time: r.time,
                epoch: r.epoch,
                reading: Ok(ImuReading {
                    orientation: r.q,
                    specific_force: r.a,
                    angular_velocity: r.w,
                    angular_velocity_covariance: r.wc,
                    linear_acceleration_covariance: r.ac,
                }),
            });
        } else {
            sim.world_mut().entity_mut(e).remove::<ImuSample>();
        }
    }
    for (id, v, w) in c.queued {
        if let Some((l, a)) = v {
            queue_link_world_velocity(&mut sim, id, l, a)?;
        }
        if let Some((f, t)) = w {
            queue_link_world_wrench(&mut sim, id, f, t)?;
        }
    }
    let groups = sim
        .world()
        .resource::<TrajectoryProcessPool>()
        .material_owners()
        .len();
    if c.group_owners.is_empty() && groups == 1 {
        super::select(&mut sim, info.epoch, info.iteration, c.owner)
            .map_err(|_| Error::Domain("invalid recovered worker placement"))?;
    } else {
        if groups < 2 || groups != c.group_owners.len() || c.group_owners[0] != c.owner {
            return Err(Error::Domain("invalid partition placement manifest"));
        }
        for (group, slot) in c.group_owners.into_iter().enumerate() {
            super::select_material_group(&mut sim, info.epoch, info.iteration, group, slot)
                .map_err(|_| Error::Domain("invalid recovered material placement"))?;
        }
    }
    if !c.transports.is_empty() {
        if c.transports.len() != groups {
            return Err(Error::Domain("invalid worker transport manifest"));
        }
        for (group, route) in c.transports.into_iter().enumerate() {
            super::select_transport(&mut sim, info.epoch, info.iteration, group, route)
                .map_err(|_| Error::Domain("invalid recovered worker transport"))?;
        }
    }
    Ok(sim)
}
