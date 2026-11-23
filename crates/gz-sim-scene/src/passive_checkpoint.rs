// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Same-build, immutable recovery for self-contained passive SDF primitive pools.
//! The owner exposes supported scene operations. Consuming it yields a normal
//! Simulation for an application with its own ingress and scheduling resources.
mod held;
mod live;
mod population;
use super::*;
use gz_physics_distributed::{
    SceneBackend,
    coordinator::RecoveryError,
    physics::BodyWrench,
    pool::{PoolNativeRecovery, ScenePopulation},
};
pub use live::save_paused_runtime;
pub(crate) use live::{note_live_birth, note_live_replacement, note_live_retirement, same_pose};
pub use population::RestoredTemplates;
use population::{Changes, Population};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::path::Path;

const MAX_SDF_BYTES: usize = 1024 * 1024;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("passive scene checkpoint: {0}")]
    Domain(&'static str),
    #[error(transparent)]
    Load(#[from] LoadError),
    #[error(transparent)]
    Recovery(#[from] RecoveryError),
    #[error(transparent)]
    Edit(#[from] WorldEditError),
    #[error(transparent)]
    Remove(#[from] RemoveError),
    #[error(transparent)]
    Manifest(#[from] serde_json::Error),
    #[error(transparent)]
    Identity(#[from] IdentityError),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Domain {
    sdf_sha256: [u8; 32],
    world: String,
    workers: u8,
    refinement: Refinement,
    step_override: Option<Duration>,
    noise_seed: [u8; 32],
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    per_point_friction: bool,
    packed_handoffs: bool,
    contact_placement: bool,
    local_helper_limit: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    periodic_placement: Option<crate::PeriodicPlacement>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
enum Refinement {
    Fixed,
    Scene,
    Island,
}

impl Domain {
    fn new(
        xml: &str,
        world: &str,
        options: WorldLoadOptions<'_>,
        runtime: WorldRuntimeOptions<'_>,
    ) -> Result<Self, Error> {
        if xml.len() > MAX_SDF_BYTES {
            return Err(Error::Domain("SDF exceeds 1 MiB"));
        }
        if runtime.distributed_native_64 {
            return Err(Error::Domain(
                "passive checkpoint currently admits the standard native member limit",
            ));
        }
        if runtime.gpu_adapter.is_some() {
            return Err(Error::Domain(
                "passive checkpoint requires local CPU workers",
            ));
        }
        let (workers, refinement) = match options.engine {
            PhysicsEngine::DistributedPool(n @ 3..=8) => (n, Refinement::Fixed),
            PhysicsEngine::DistributedAdaptive(n @ 3..=8) => (n, Refinement::Scene),
            PhysicsEngine::DistributedIslands(n @ 3..=8) => (n, Refinement::Island),
            _ => {
                return Err(Error::Domain(
                    "requires a three- to eight-worker distributed pool",
                ));
            }
        };
        let doc = roxmltree::Document::parse(xml).map_err(|_| Error::Domain("invalid SDF XML"))?;
        if doc.root_element().tag_name().name() != "sdf"
            || doc
                .root_element()
                .children()
                .filter(|n| n.is_element())
                .count()
                != 1
            || !doc
                .root_element()
                .children()
                .any(|n| n.has_tag_name("world") && n.attribute("name") == Some(world))
        {
            return Err(Error::Domain("requires one self-contained SDF world"));
        }
        // Positive admission prevents ignored or future loader extensions from creating
        // unrepresented runtime state. The ordinary SDF loader validates semantics.
        for node in doc.descendants().filter(|n| n.is_element()) {
            if node.tag_name().namespace().is_some()
                || !matches!(
                    node.tag_name().name(),
                    "sdf"
                        | "world"
                        | "physics"
                        | "max_step_size"
                        | "real_time_factor"
                        | "gravity"
                        | "model"
                        | "static"
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
                        | "sphere"
                        | "capsule"
                        | "cylinder"
                        | "plane"
                        | "size"
                        | "normal"
                        | "length"
                        | "radius"
                        | "surface"
                        | "friction"
                        | "ode"
                        | "mu"
                        | "mu2"
                        | "material"
                        | "diffuse"
                        | "ambient"
                        | "specular"
                        | "emissive"
                )
            {
                return Err(Error::Domain(
                    "requires passive self-contained primitive geometry and materials",
                ));
            }
        }
        Ok(Self {
            sdf_sha256: Sha256::digest(xml.as_bytes()).into(),
            world: world.to_owned(),
            workers,
            refinement,
            step_override: options.step,
            noise_seed: options.noise_seed,
            per_point_friction: runtime.rapier_per_point_friction,
            packed_handoffs: runtime.distributed_packed_handoffs,
            contact_placement: runtime.distributed_contact_placement,
            local_helper_limit: runtime.distributed_local_helper_limit,
            periodic_placement: runtime.distributed_periodic_placement,
        })
    }
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Clock {
    start_time: Duration,
    sim_time: Duration,
    dt: Duration,
    iteration: u64,
    epoch: u64,
    update_paused: bool,
    pub(super) paused: bool,
}
impl Clock {
    pub(super) fn from_simulation(sim: &Simulation) -> Self {
        let info = *sim.world().resource::<UpdateInfo>();
        Self {
            start_time: info.start_time,
            sim_time: info.sim_time,
            dt: info.dt,
            iteration: info.iteration,
            epoch: info.epoch,
            update_paused: info.paused,
            paused: sim.is_paused(),
        }
    }
    pub(super) fn info(&self) -> UpdateInfo {
        UpdateInfo {
            start_time: self.start_time,
            sim_time: self.sim_time,
            dt: self.dt,
            iteration: self.iteration,
            epoch: self.epoch,
            paused: self.update_paused,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
enum Kind {
    World,
    Model,
    Link,
    Collision,
    Visual,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Node {
    id: u64,
    kind: Kind,
    name: String,
    local_name: String,
    parent: Option<u64>,
    body: Option<u64>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Bindings {
    next_public_id: u64,
    nodes: Vec<Node>,
}

fn hash_public_motion(
    hash: &mut Sha256,
    id: u64,
    pose: WorldPose,
    local: WorldPose,
    velocity: Option<Velocity>,
) {
    hash.update(id.to_le_bytes());
    for p in [pose, local] {
        for x in p.translation.into_iter().chain(p.rotation) {
            hash.update(x.to_le_bytes());
        }
    }
    hash.update([u8::from(velocity.is_some())]);
    if let Some(v) = velocity {
        for x in v.linear.into_iter().chain(v.angular) {
            hash.update(x.to_le_bytes());
        }
    }
}

fn public_state(sim: &Simulation) -> Result<(Bindings, [u8; 32]), Error> {
    let world = sim.world();
    let mut records = Vec::new();
    for e in world.iter_entities().filter(|e| e.contains::<EntityKind>()) {
        let kind = match e.get::<EntityKind>().expect("selected kind") {
            EntityKind::World => Kind::World,
            EntityKind::Model => Kind::Model,
            EntityKind::Link => Kind::Link,
            EntityKind::Collision => Kind::Collision,
            EntityKind::Visual => Kind::Visual,
            _ => return Err(Error::Domain("non-passive public entity")),
        };
        let id = e
            .get::<EntityId>()
            .ok_or(Error::Domain("public entity has no ID"))?
            .get();
        if gz_sim_core::entity_by_id(world, id) != Some(e.id()) {
            return Err(Error::Domain("inconsistent public ID index"));
        }
        let parent = e
            .get::<ChildOf>()
            .map(|p| {
                world
                    .get::<EntityId>(p.parent())
                    .map(EntityId::get)
                    .ok_or(Error::Domain("unidentified scene parent"))
            })
            .transpose()?;
        let node = Node {
            id,
            kind,
            parent,
            name: e
                .get::<Name>()
                .ok_or(Error::Domain("missing scene name"))?
                .0
                .clone(),
            local_name: e
                .get::<LocalName>()
                .ok_or(Error::Domain("missing local scene name"))?
                .0
                .clone(),
            body: e.get::<BodyHandle>().map(|b| b.id().raw()),
        };
        if (node.kind == Kind::Link) != node.body.is_some() {
            return Err(Error::Domain("missing or extra native body binding"));
        }
        let pose = *e
            .get::<WorldPose>()
            .ok_or(Error::Domain("missing public world pose"))?;
        let local = e
            .get::<LocalPose>()
            .ok_or(Error::Domain("missing public local pose"))?
            .0;
        let velocity = e.get::<Velocity>().copied();
        records.push((node, pose, local, velocity));
    }
    records.sort_unstable_by_key(|(n, ..)| n.id);
    let mut hash = Sha256::new();
    let mut nodes = Vec::with_capacity(records.len());
    for (node, pose, local, velocity) in records {
        hash_public_motion(&mut hash, node.id, pose, local, velocity);
        nodes.push(node);
    }
    Ok((
        Bindings {
            next_public_id: gz_sim_core::next_entity_id(world)?,
            nodes,
        },
        hash.finalize().into(),
    ))
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Manifest {
    domain: Domain,
    bindings: Bindings,
    public_pose_sha256: [u8; 32],
    clock: Clock,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    population: Option<Population>,
    #[serde(
        default,
        skip_serializing_if = "Vec::is_empty",
        deserialize_with = "held::decode"
    )]
    held_wrenches: Vec<BodyWrench>,
}
impl Manifest {
    fn seal(
        self,
        sim: &mut Simulation,
        path: &Path,
        version: ScenePopulation,
    ) -> Result<(), Error> {
        let bytes = serde_json::to_vec(&self)?;
        backend_mut(sim).save_population_checkpoint_with_wrenches(
            path,
            self.clock.iteration,
            &bytes,
            version,
            &self.held_wrenches,
        )?;
        Ok(())
    }
}

fn backend_mut(sim: &mut Simulation) -> Mut<'_, SceneBackend> {
    sim.world_mut()
        .resource_mut::<PhysicsBackend>()
        .map_unchanged(|b| {
            b.downcast_mut::<SceneBackend>()
                .expect("private initialized pool scene")
        })
}

/// A standard Bevy simulation restricted to the passive SDF state represented by
/// the durable checkpoint. `into_simulation` transfers it to an unrestricted owner.
pub struct PassivePoolScene {
    sim: Simulation,
    state: CheckpointState,
}

#[derive(Resource)]
struct CheckpointState {
    domain: Domain,
    bindings: Bindings,
    templates: RestoredTemplates,
    changes: Changes,
    initial_next_native: u64,
    epoch_start_native: u64,
}

impl PassivePoolScene {
    pub fn load(
        xml: &str,
        source: &str,
        world: &str,
        options: WorldLoadOptions<'_>,
        runtime: WorldRuntimeOptions<'_>,
    ) -> Result<Self, Error> {
        let domain = Domain::new(xml, world, options, runtime)?;
        let sim =
            load_world_with_runtime(xml, source, world, &sdformat::NoResources, options, runtime)?;
        Self::from_seed(sim, domain, xml, source)
    }

    fn from_seed(sim: Simulation, domain: Domain, xml: &str, source: &str) -> Result<Self, Error> {
        let (bindings, _) = public_state(&sim)?;
        let templates = RestoredTemplates::prepare(&sim, xml, source, &domain.world)?;
        let initial_next_native = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .expect("private initialized pool scene")
            .next_body_handle();
        Ok(Self {
            sim,
            state: CheckpointState {
                domain,
                bindings,
                templates,
                changes: Changes::default(),
                initial_next_native,
                epoch_start_native: initial_next_native,
            },
        })
    }

    /// Fence the former owner before resuming. No source paths from the archive
    /// are executed. A replacement receives its worker executable from the caller.
    pub fn resume(
        xml: &str,
        source: &str,
        world: &str,
        options: WorldLoadOptions<'_>,
        runtime: WorldRuntimeOptions<'_>,
        checkpoint: &Path,
    ) -> Result<Self, Error> {
        let domain = Domain::new(xml, world, options, runtime)?;
        let (version, metadata, native) = PoolNativeRecovery::open_scene_manifest(checkpoint)?;
        let manifest: Manifest = serde_json::from_slice(&metadata)?;
        if (version == ScenePopulation::Edited) != manifest.population.is_some() {
            return Err(Error::Domain(
                "scene version and population manifest differ",
            ));
        }
        if domain != manifest.domain {
            return Err(Error::Domain(
                "SDF, world or execution settings differ from checkpoint",
            ));
        }
        let tick = manifest.clock.iteration;
        if native.frame().tick != tick {
            return Err(Error::Domain("native and simulation clocks differ"));
        }
        let sim = load_world_at_clock(
            xml,
            source,
            world,
            &sdformat::NoResources,
            options,
            runtime,
            Some((manifest.clock.info(), manifest.clock.paused)),
        )?;
        let mut scene = Self::from_seed(sim, domain, xml, source)?;
        let newborn = if let Some(p) = manifest.population {
            scene.stage_population(p, manifest.bindings.next_public_id, tick, &native)?
        } else {
            Vec::new()
        };
        let (bindings, startup_pose) = public_state(&scene.sim)?;
        if bindings != manifest.bindings {
            return Err(Error::Domain(
                "loaded public hierarchy or native bindings differ",
            ));
        }
        let untouched_start = backend_mut(&mut scene.sim)
            .restore_population_checkpoint_with_wrenches(
                native,
                tick,
                version,
                manifest.held_wrenches,
            )?;
        if !untouched_start || startup_pose != manifest.public_pose_sha256 {
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
            publish.run(scene.sim.world_mut());
            // A same-tick birth has its original authored local transforms until the
            // next ordinary physics commit. Its native state was checked above.
            for pose in newborn {
                pose.publish(&mut scene.sim);
            }
        }
        scene.state.check_newborn_receipts(&scene.sim, tick)?;
        if public_state(&scene.sim)?.1 != manifest.public_pose_sha256 {
            return Err(Error::Domain("restored public poses or velocities differ"));
        }
        Ok(scene)
    }

    pub fn save(&mut self, path: &Path) -> Result<(), Error> {
        let (manifest, version) = self.state.manifest(&self.sim)?;
        manifest.seal(&mut self.sim, path, version)
    }

    pub fn simulation(&self) -> &Simulation {
        &self.sim
    }
    pub fn into_simulation(mut self) -> Simulation {
        self.sim.world_mut().insert_resource(self.state.templates);
        self.sim
    }
    pub fn set_paused(&mut self, paused: bool) {
        self.sim.set_paused(paused);
    }
    pub fn step_exact(&mut self, steps: u64) -> Result<(), StepError> {
        self.sim.step_exact(steps)
    }
    /// Advance the ordinary ECS schedule once with this barrier's recorded placement.
    /// Held distributed wrenches and worker-acknowledged impulses participate
    /// in admission. Deferred ECS edits have not yet published their body state.
    pub fn step_with_placement(
        &mut self,
        record: &gz_physics_distributed::pool::PlacementRecord,
    ) -> Result<(), gz_physics::PhysicsError> {
        crate::step_with_placement(&mut self.sim, record)
    }
    pub fn tick(&mut self) -> Result<UpdateInfo, StepError> {
        self.sim.tick()
    }
    pub fn reset_loaded(&mut self) -> Result<(), SceneResetError> {
        reset_loaded_world(&mut self.sim, |_| ())?;
        self.state.changes = Changes::default();
        self.state.epoch_start_native = backend_mut(&mut self.sim).next_body_handle();
        Ok(())
    }

    /// A worker-acknowledged linear impulse survives a checkpoint and is consumed once.
    pub fn queue_impulse(&mut self, link: u64, impulse: [f64; 3]) -> Result<(), InputError> {
        self.sim.validate_steps(1)?;
        let world = self.sim.world();
        let handle = gz_sim_core::entity_by_id(world, link)
            .and_then(|e| world.get::<BodyHandle>(e))
            .ok_or(InputError::NotFreeLink(link))?
            .id();
        backend_mut(&mut self.sim).queue_linear_impulse(handle, impulse)?;
        Ok(())
    }

    pub fn queue_velocity(
        &mut self,
        link: u64,
        linear: [f64; 3],
        angular: [f64; 3],
    ) -> Result<(), InputError> {
        queue_link_world_velocity(&mut self.sim, link, linear, angular)
    }

    pub fn queue_wrench(
        &mut self,
        link: u64,
        force: [f64; 3],
        torque: [f64; 3],
    ) -> Result<(), InputError> {
        queue_link_world_wrench(&mut self.sim, link, force, torque)
    }
}

impl CheckpointState {
    fn manifest(&self, sim: &Simulation) -> Result<(Manifest, ScenePopulation), Error> {
        if sim.is_faulted() || inputs::has_pending_input(sim.world()) {
            return Err(Error::Domain("fault or unconsumed public link input"));
        }
        let (bindings, public_pose_sha256) = public_state(sim)?;
        let population = self.manifest_population(sim)?;
        let version = if population.is_some() {
            ScenePopulation::Edited
        } else {
            ScenePopulation::Loaded
        };
        if population.is_none() && bindings != self.bindings {
            return Err(Error::Domain("public bindings or issued IDs have changed"));
        }
        let clock = Clock::from_simulation(sim);
        let tick = clock.iteration;
        if let Some(p) = &population {
            self.check_population(sim, p, bindings.next_public_id, tick)?;
            self.check_membership(&bindings, p)?;
        }
        let manifest = Manifest {
            population,
            domain: self.domain.clone(),
            bindings,
            public_pose_sha256,
            clock,
            held_wrenches: sim
                .world()
                .get_resource::<PhysicsBackend>()
                .and_then(|backend| backend.downcast_ref::<SceneBackend>())
                .ok_or(Error::Domain("missing passive pool backend"))?
                .held_scene_wrenches()
                .to_vec(),
        };
        Ok((manifest, version))
    }
}
