// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Concrete programmatic camera application; the ordinary Server stays headless.
use crate::{
    CameraOutputConfig, CameraOutputError, CameraWriterScope, Progress, ScheduledCameraOutput,
};
use bevy_ecs::{entity::Entity, hierarchy::ChildOf};
use gz_render_client::TrustedWorker;
use gz_render_scene::{SnapshotError, extract_camera};
use gz_sim_core::{EntityId, Simulation, UpdateInfo, entity_by_id};
use gz_sim_proto::prost::Name as ProtoName;
use gz_sim_scene::{CameraSensor, EntityKind, LocalName, Name, RenderingSensorsConfiguration};
use gz_sim_server::{Server, ServerOptions, SetupError, as_valid_topic};
use std::{
    collections::{BTreeMap, BTreeSet},
    num::NonZeroUsize,
    path::PathBuf,
};
use zenoh::{Session, Wait};

/// Caller-owned operator settings, borrowed only for initial/reset preparation.
/// No settings, Session or writer scope is retained for an absent camera family.
pub struct CameraServerOptions {
    pub worker_path: PathBuf,
    pub origin: [f64; 3],
    /// Explicit upstream configuration for the camera's separate direct Session.
    pub zenoh: zenoh::Config,
}

#[derive(Debug, thiserror::Error)]
pub enum CameraServerError {
    #[error(transparent)]
    Server(#[from] gz_sim_server::Error),
    #[error(transparent)]
    Camera(#[from] CameraOutputError),
    #[error(transparent)]
    Topic(#[from] SetupError),
}

/// Both observations come from the existing bounded output state machine.
#[derive(Debug)]
pub struct CameraStep {
    pub info: UpdateInfo,
    pub pumped: Option<Progress>,
    pub requested: Option<Progress>,
}

/// One bounded camera visit; a per-owner error does not skip its peer.
#[derive(Debug)]
pub struct CameraCapture {
    pub sensor_id: u64,
    pub result: Result<CameraFrameProgress, CameraServerError>,
}
#[derive(Debug)]
pub struct CameraFrameProgress {
    pub pumped: Option<Progress>,
    pub requested: Option<Progress>,
}

struct CameraNode {
    entity: Entity,
    id: u64,
    kind: EntityKind,
    local: String,
    scoped: String,
    parent: Option<Entity>,
}
struct CameraBinding {
    id: u64,
    nodes: Vec<CameraNode>,
}
impl CameraBinding {
    fn current(&self, sim: &Simulation) -> bool {
        let world = sim.world();
        self.nodes.iter().all(|node| {
            entity_by_id(world, node.id) == Some(node.entity)
                && world.get::<EntityKind>(node.entity) == Some(&node.kind)
                && world
                    .get::<LocalName>(node.entity)
                    .is_some_and(|n| n.0 == node.local)
                && world
                    .get::<Name>(node.entity)
                    .is_some_and(|n| n.0 == node.scoped)
                && world.get::<ChildOf>(node.entity).map(ChildOf::parent) == node.parent
        }) && self.nodes.last().is_some_and(|node| {
            world
                .get::<RenderingSensorsConfiguration>(node.entity)
                .is_some()
        })
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CameraKind {
    Rgb,
    Depth,
}
struct PreparedCamera {
    kind: CameraKind,
    binding: CameraBinding,
    config: CameraOutputConfig,
}
struct CameraRuntime {
    binding: CameraBinding,
    output: ScheduledCameraOutput,
    session: Session,
    owns_session: bool,
}
impl Drop for CameraRuntime {
    fn drop(&mut self) {
        self.output.retire();
        // Teardown may wait, including native final drops. This owner never
        // enters the Bevy schedule or Server's critical tick.
        if self.owns_session {
            let _ = self.session.close().wait();
        }
    }
}

struct PairOutput {
    binding: CameraBinding,
    output: ScheduledCameraOutput,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PairKind {
    Rgb,
    Depth,
    Mixed,
}

struct CameraPair {
    outputs: BTreeMap<u64, PairOutput>,
    session: Session,
    owns_session: bool,
}
impl Drop for PairOutput {
    fn drop(&mut self) {
        self.output.retire();
    }
}
impl Drop for CameraPair {
    fn drop(&mut self) {
        // Drop/retire every output before the one family-owned Session closes.
        self.outputs.clear();
        if self.owns_session {
            let _ = self.session.close().wait();
        }
    }
}

/// Programmatic single-camera application over the existing Server.
///
/// Preparation, update, pump, shutdown and Drop all belong outside the critical
/// step. A full update may have committed physics before returning a camera
/// error, including Busy. No error causes an implicit repeated acquisition.
/// Raw Server access retains its explicit headless/critical-step contracts.
/// Dynamic sensor registration and camera set_rate are absent.
pub struct CameraServer {
    // Drop camera IO before the world Server even on constructor/error unwind.
    runtime: Option<CameraRuntime>,
    pair: Option<CameraPair>,
    pair_ids: Option<[u64; 2]>,
    rgb_family_ids: Option<Vec<u64>>,
    pair_sensor_kinds: Option<[CameraKind; 2]>,
    // Intent outlives the live subset; reset must rebuild the same family.
    pair_kind: PairKind,
    server: Server,
    prepared_epoch: u64,
    shared_server_session: bool,
}
impl CameraServer {
    /// Prepare the selected camera family on the world's existing Zenoh session.
    ///
    /// This is the installed application's zero/single/RGB-pair/Depth-pair path.
    /// It validates the complete family before opening world IO, then shares
    /// that actual peer instead of guessing a dialable listener or opening an
    /// isolated outgoing peer. `camera_options.zenoh` remains the separate-peer
    /// configuration of the three existing constructors and is unused here.
    pub fn new_with_server_session(
        simulation: Simulation,
        world: &str,
        options: ServerOptions,
        server_zenoh: zenoh::Config,
        command_budget: NonZeroUsize,
        camera_options: &CameraServerOptions,
    ) -> Result<Self, CameraServerError> {
        options.validate()?;
        let paired = simulation.world().iter_entities().any(|entity| {
            entity.get::<EntityKind>() == Some(&EntityKind::World)
                && entity.contains::<RenderingSensorsConfiguration>()
        }) && simulation
            .world()
            .iter_entities()
            .filter(|entity| entity.contains::<CameraSensor>())
            .take(3)
            .count()
            == 2;
        let depth_pair = paired
            && simulation
                .world()
                .iter_entities()
                .filter_map(|entity| entity.get::<CameraSensor>())
                .all(|sensor| {
                    sensor
                        .0
                        .effective_type()
                        .is_some_and(|kind| kind.as_str() == "depth_camera")
                });
        let epoch = simulation.epoch();
        if paired {
            let (kind, prepared) = if depth_pair {
                (
                    PairKind::Depth,
                    prepare_depth_pair(&simulation, &options, camera_options)?,
                )
            } else {
                (
                    PairKind::Rgb,
                    prepare_pair(&simulation, &options, camera_options)?,
                )
            };
            let ids = [prepared[0].binding.id, prepared[1].binding.id];
            let kinds = [prepared[0].kind, prepared[1].kind];
            let server = if depth_pair {
                Server::with_external_depth_camera_outputs(
                    simulation,
                    world,
                    options,
                    server_zenoh,
                    command_budget,
                    ids,
                )
            } else {
                Server::with_external_camera_outputs(
                    simulation,
                    world,
                    options,
                    server_zenoh,
                    command_budget,
                    ids,
                )
            }?;
            let pair = install_pair_with_session(
                server.simulation(),
                prepared,
                server.zenoh_session(),
                false,
            )?;
            return Ok(Self {
                runtime: None,
                pair: Some(pair),
                rgb_family_ids: None,
                pair_ids: Some(ids),
                pair_sensor_kinds: Some(kinds),
                pair_kind: kind,
                server,
                prepared_epoch: epoch,
                shared_server_session: true,
            });
        }
        let prepared = prepare_camera(&simulation, &options, camera_options)?;
        let server = match prepared.as_ref() {
            Some(prepared) => Server::with_external_camera_output(
                simulation,
                world,
                options,
                server_zenoh,
                command_budget,
                prepared.binding.id,
            ),
            None => {
                Server::with_zenoh_config(simulation, world, options, server_zenoh, command_budget)
            }
        }?;
        let runtime = prepared
            .map(|prepared| {
                install_camera_with_session(
                    server.simulation(),
                    prepared,
                    server.zenoh_session(),
                    false,
                )
            })
            .transpose()?;
        Ok(Self {
            runtime,
            pair: None,
            rgb_family_ids: None,
            pair_ids: None,
            pair_sensor_kinds: None,
            pair_kind: PairKind::Rgb,
            server,
            prepared_epoch: epoch,
            shared_server_session: true,
        })
    }

    pub fn new(
        simulation: Simulation,
        world: &str,
        options: ServerOptions,
        server_zenoh: zenoh::Config,
        command_budget: NonZeroUsize,
        camera_options: &CameraServerOptions,
    ) -> Result<Self, CameraServerError> {
        options.validate()?;
        let prepared = prepare_camera(&simulation, &options, camera_options)?;
        let epoch = simulation.epoch();
        let runtime = prepared
            .map(|prepared| install_camera(&simulation, prepared, camera_options))
            .transpose()?;
        let server = match runtime.as_ref() {
            Some(runtime) => Server::with_external_camera_output(
                simulation,
                world,
                options,
                server_zenoh,
                command_budget,
                runtime.binding.id,
            ),
            None => {
                Server::with_zenoh_config(simulation, world, options, server_zenoh, command_budget)
            }
        }?;
        Ok(Self {
            runtime,
            pair: None,
            rgb_family_ids: None,
            pair_ids: None,
            pair_sensor_kinds: None,
            pair_kind: PairKind::Rgb,
            server,
            prepared_epoch: epoch,
            shared_server_session: false,
        })
    }

    /// Explicit pair admission; the legacy constructor retains zero/single semantics.
    pub fn new_pair(
        simulation: Simulation,
        world: &str,
        options: ServerOptions,
        server_zenoh: zenoh::Config,
        command_budget: NonZeroUsize,
        camera_options: &CameraServerOptions,
    ) -> Result<Self, CameraServerError> {
        options.validate()?;
        let prepared = prepare_pair(&simulation, &options, camera_options)?;
        let ids = [prepared[0].binding.id, prepared[1].binding.id];
        let epoch = simulation.epoch();
        let pair = install_pair(&simulation, prepared, camera_options)?;
        let server = Server::with_external_camera_outputs(
            simulation,
            world,
            options,
            server_zenoh,
            command_budget,
            ids,
        )?;
        Ok(Self {
            runtime: None,
            pair: Some(pair),
            rgb_family_ids: None,
            pair_ids: Some(ids),
            pair_sensor_kinds: Some([CameraKind::Rgb; 2]),
            pair_kind: PairKind::Rgb,
            server,
            prepared_epoch: epoch,
            shared_server_session: false,
        })
    }

    /// Exactly two homogeneous depth owners. The existing RGB-pair and scalar
    /// constructors retain their own strict admission and error boundaries.
    pub fn new_depth_pair(
        simulation: Simulation,
        world: &str,
        options: ServerOptions,
        server_zenoh: zenoh::Config,
        command_budget: NonZeroUsize,
        camera_options: &CameraServerOptions,
    ) -> Result<Self, CameraServerError> {
        options.validate()?;
        let prepared = prepare_depth_pair(&simulation, &options, camera_options)?;
        let ids = [prepared[0].binding.id, prepared[1].binding.id];
        let epoch = simulation.epoch();
        let pair = install_pair(&simulation, prepared, camera_options)?;
        let server = Server::with_external_depth_camera_outputs(
            simulation,
            world,
            options,
            server_zenoh,
            command_budget,
            ids,
        )?;
        Ok(Self {
            runtime: None,
            pair: Some(pair),
            rgb_family_ids: None,
            pair_ids: Some(ids),
            pair_sensor_kinds: Some([CameraKind::Depth; 2]),
            pair_kind: PairKind::Depth,
            server,
            prepared_epoch: epoch,
            shared_server_session: false,
        })
    }

    /// Exactly one RGB and one depth camera, on a separate caller-configured
    /// camera peer. The legacy constructors keep their homogeneous contracts.
    pub fn new_mixed_pair(
        simulation: Simulation,
        world: &str,
        options: ServerOptions,
        server_zenoh: zenoh::Config,
        command_budget: NonZeroUsize,
        camera_options: &CameraServerOptions,
    ) -> Result<Self, CameraServerError> {
        options.validate()?;
        let prepared = prepare_mixed_pair(&simulation, &options, camera_options)?;
        let ids = [prepared[0].binding.id, prepared[1].binding.id];
        let kinds = [prepared[0].kind, prepared[1].kind];
        let epoch = simulation.epoch();
        let pair = install_pair(&simulation, prepared, camera_options)?;
        let server = Server::with_external_mixed_camera_outputs(
            simulation,
            world,
            options,
            server_zenoh,
            command_budget,
            ids,
        )?;
        Ok(Self {
            runtime: None,
            pair: Some(pair),
            rgb_family_ids: None,
            pair_ids: Some(ids),
            pair_sensor_kinds: Some(kinds),
            pair_kind: PairKind::Mixed,
            server,
            prepared_epoch: epoch,
            shared_server_session: false,
        })
    }

    /// Explicit mixed pair on the world's existing peer. All camera preflight
    /// precedes world IO; retiring outputs never closes that shared session.
    pub fn new_mixed_pair_with_server_session(
        simulation: Simulation,
        world: &str,
        options: ServerOptions,
        server_zenoh: zenoh::Config,
        command_budget: NonZeroUsize,
        camera_options: &CameraServerOptions,
    ) -> Result<Self, CameraServerError> {
        options.validate()?;
        let prepared = prepare_mixed_pair(&simulation, &options, camera_options)?;
        let ids = [prepared[0].binding.id, prepared[1].binding.id];
        let kinds = [prepared[0].kind, prepared[1].kind];
        let epoch = simulation.epoch();
        let server = Server::with_external_mixed_camera_outputs(
            simulation,
            world,
            options,
            server_zenoh,
            command_budget,
            ids,
        )?;
        let pair = install_pair_with_session(
            server.simulation(),
            prepared,
            server.zenoh_session(),
            false,
        )?;
        Ok(Self {
            runtime: None,
            pair: Some(pair),
            rgb_family_ids: None,
            pair_ids: Some(ids),
            pair_sensor_kinds: Some(kinds),
            pair_kind: PairKind::Mixed,
            server,
            prepared_epoch: epoch,
            shared_server_session: true,
        })
    }

    /// Explicit three/four RGB owners on the world's existing peer. Legacy
    /// constructors retain their exact scalar/pair/depth/mixed admission.
    pub fn new_rgb_family_with_server_session(
        simulation: Simulation,
        world: &str,
        options: ServerOptions,
        server_zenoh: zenoh::Config,
        command_budget: NonZeroUsize,
        camera_options: &CameraServerOptions,
    ) -> Result<Self, CameraServerError> {
        options.validate()?;
        let prepared = prepare_rgb_family(&simulation, &options, camera_options)?;
        let ids = prepared.iter().map(|p| p.binding.id).collect::<Vec<_>>();
        let epoch = simulation.epoch();
        let server = Server::with_external_rgb_camera_family(
            simulation,
            world,
            options,
            server_zenoh,
            command_budget,
            ids.clone(),
        )?;
        let pair = install_pair_with_session(
            server.simulation(),
            prepared,
            server.zenoh_session(),
            false,
        )?;
        Ok(Self {
            runtime: None,
            pair: Some(pair),
            pair_ids: None,
            rgb_family_ids: Some(ids),
            pair_sensor_kinds: None,
            pair_kind: PairKind::Rgb,
            server,
            prepared_epoch: epoch,
            shared_server_session: true,
        })
    }

    fn require_scalar(&self) -> Result<(), CameraServerError> {
        if self.pair_ids.is_some() || self.rgb_family_ids.is_some() {
            return Err(CameraOutputError::PairMode.into());
        }
        Ok(())
    }

    /// Visit each owner once after whole-set validation, even if its peer is Busy.
    /// Output errors never imply that an already committed tick may be repeated.
    pub fn capture_all(&mut self) -> Result<Vec<CameraCapture>, CameraServerError> {
        self.require_running()?;
        self.require_prepared()?;
        if self.pair_ids.is_none() && self.rgb_family_ids.is_none() {
            if !self.current_camera()? {
                return Ok(Vec::new());
            }
            let id = self
                .runtime
                .as_ref()
                .expect("validated single camera")
                .binding
                .id;
            return Ok(vec![CameraCapture {
                sensor_id: id,
                result: self
                    .output_frame()
                    .map(|(pumped, requested)| CameraFrameProgress { pumped, requested }),
            }]);
        }
        self.current_pair()?;
        let sim = self.server.simulation();
        let Some(pair) = self.pair.as_mut() else {
            return Ok(Vec::new());
        };
        let mut rows = Vec::with_capacity(pair.outputs.len());
        for (&sensor_id, owner) in &mut pair.outputs {
            let result = (|| -> Result<CameraFrameProgress, CameraServerError> {
                let pumped = owner.output.pump(sim)?;
                if pumped == Progress::RetireRequired {
                    owner.output.retire();
                }
                let requested = owner.output.request_due(sim)?;
                if requested == Some(Progress::RetireRequired) {
                    owner.output.retire();
                }
                Ok(CameraFrameProgress {
                    pumped: Some(pumped),
                    requested,
                })
            })();
            rows.push(CameraCapture { sensor_id, result });
        }
        Ok(rows)
    }

    /// Nonowning observation only: does not start, pump or retire any worker.
    /// Lifetime retirement alone does not establish OS absence, reap or exit code.
    pub fn camera_worker_diagnostics(
        &self,
        id: u64,
    ) -> Option<(Option<u32>, gz_render_client::ClientLifetime)> {
        if self.pair_ids.is_some() || self.rgb_family_ids.is_some() {
            self.pair
                .as_ref()?
                .outputs
                .get(&id)?
                .output
                .client_diagnostics()
        } else {
            self.runtime
                .as_ref()
                .filter(|owner| owner.binding.id == id)?
                .output
                .client_diagnostics()
        }
    }

    pub fn pump_camera(&mut self, id: u64) -> Result<Option<Progress>, CameraServerError> {
        self.require_running()?;
        self.require_prepared()?;
        if self.pair_ids.is_some() || self.rgb_family_ids.is_some() {
            if !self.pair_ids.as_ref().is_some_and(|ids| ids.contains(&id))
                && !self
                    .rgb_family_ids
                    .as_ref()
                    .is_some_and(|ids| ids.contains(&id))
            {
                return Err(missing(id).into());
            }
            self.current_pair()?;
            let Some(owner) = self
                .pair
                .as_mut()
                .and_then(|pair| pair.outputs.get_mut(&id))
            else {
                return Ok(None);
            };
            let progress = owner.output.pump(self.server.simulation())?;
            if progress == Progress::RetireRequired {
                owner.output.retire();
            }
            Ok(Some(progress))
        } else {
            if self
                .runtime
                .as_ref()
                .is_some_and(|runtime| runtime.binding.id != id)
            {
                return Err(missing(id).into());
            }
            self.pump()
        }
    }

    pub fn camera_demand_for(
        &mut self,
        id: u64,
    ) -> Result<Option<(bool, bool)>, CameraServerError> {
        self.require_running()?;
        self.require_prepared()?;
        if self.pair_ids.is_some() || self.rgb_family_ids.is_some() {
            if !self.pair_ids.as_ref().is_some_and(|ids| ids.contains(&id))
                && !self
                    .rgb_family_ids
                    .as_ref()
                    .is_some_and(|ids| ids.contains(&id))
            {
                return Err(missing(id).into());
            }
            self.current_pair()?;
            self.pair
                .as_ref()
                .and_then(|pair| pair.outputs.get(&id))
                .map(|owner| owner.output.demand().map_err(CameraServerError::from))
                .transpose()
        } else {
            if self
                .runtime
                .as_ref()
                .is_some_and(|runtime| runtime.binding.id != id)
            {
                return Err(missing(id).into());
            }
            self.camera_demand()
        }
    }

    fn current_pair(&mut self) -> Result<(), CameraServerError> {
        let sim = self.server.simulation();
        let validation = (|| -> Result<Vec<u64>, CameraServerError> {
            if sim.is_faulted() {
                return Err(CameraOutputError::OwnerInvalid.into());
            }
            if self
                .pair
                .as_ref()
                .is_some_and(|pair| pair.session.is_closed())
            {
                return Err(CameraOutputError::SessionClosed.into());
            }
            let mut current = BTreeSet::new();
            for node in sim
                .world()
                .iter_entities()
                .filter(|node| node.contains::<CameraSensor>())
            {
                let id = node
                    .get::<EntityId>()
                    .ok_or(CameraOutputError::OwnerInvalid)?
                    .get();
                if entity_by_id(sim.world(), id) != Some(node.id())
                    || !current.insert(id)
                    || !self
                        .pair
                        .as_ref()
                        .is_some_and(|pair| pair.outputs.contains_key(&id))
                {
                    return Err(CameraOutputError::OwnerInvalid.into());
                }
            }
            let mut removed = Vec::new();
            if let Some(pair) = &self.pair {
                for (&id, owner) in &pair.outputs {
                    if entity_by_id(sim.world(), id).is_none()
                        && sim
                            .world()
                            .get_entity(owner.binding.nodes[0].entity)
                            .is_err()
                    {
                        removed.push(id);
                    } else if !owner.binding.current(sim) || !owner.output.binding_valid(sim) {
                        return Err(CameraOutputError::OwnerInvalid.into());
                    }
                }
            }
            Ok(removed)
        })();
        match validation {
            Ok(removed) => {
                if let Some(pair) = &mut self.pair {
                    for id in removed {
                        pair.outputs.remove(&id);
                    }
                    if pair.outputs.is_empty() {
                        self.pair = None;
                    }
                }
                Ok(())
            }
            Err(error) => {
                self.pair = None;
                Err(error)
            }
        }
    }

    pub fn server(&self) -> &Server {
        &self.server
    }
    pub fn server_mut(&mut self) -> &mut Server {
        &mut self.server
    }

    /// Whether this application currently owns configured camera IO, not whether
    /// an image subscriber or rendering worker exists.
    pub fn has_camera_output(&self) -> bool {
        self.runtime.is_some() || self.pair.is_some()
    }

    /// Commit reset preparation and rebuild only at its actual epoch boundary.
    /// A committed preparation error retires old output before returning it.
    pub fn prepare_io(&mut self, options: &CameraServerOptions) -> Result<(), CameraServerError> {
        self.require_running()?;
        let result = self.server.prepare_io();
        let epoch = self.server.simulation().epoch();
        if let Err(error) = result {
            if epoch != self.prepared_epoch || matches!(error, gz_sim_server::Error::WorldClosed) {
                self.runtime = None;
                self.pair = None;
            }
            return Err(error.into());
        }
        if epoch != self.prepared_epoch {
            self.runtime = None;
            self.pair = None;
            // Do not advance the epoch cache until preparation really succeeds.
            // The returned error is never converted into a repeated update here.
            if let Some(ids) = &self.rgb_family_ids {
                let prepared =
                    prepare_rgb_family(self.server.simulation(), self.server.options(), options)?;
                if prepared.iter().map(|p| p.binding.id).collect::<Vec<_>>() != *ids {
                    return Err(CameraOutputError::OwnerInvalid.into());
                }
                self.pair = Some(install_pair_with_session(
                    self.server.simulation(),
                    prepared,
                    self.server.zenoh_session(),
                    false,
                )?);
            } else if self.pair_ids.is_some() {
                let prepared = match self.pair_kind {
                    PairKind::Rgb => {
                        prepare_pair(self.server.simulation(), self.server.options(), options)?
                    }
                    PairKind::Depth => prepare_depth_pair(
                        self.server.simulation(),
                        self.server.options(),
                        options,
                    )?,
                    PairKind::Mixed => prepare_mixed_pair(
                        self.server.simulation(),
                        self.server.options(),
                        options,
                    )?,
                };
                if Some([prepared[0].binding.id, prepared[1].binding.id]) != self.pair_ids
                    || Some([prepared[0].kind, prepared[1].kind]) != self.pair_sensor_kinds
                {
                    return Err(CameraOutputError::OwnerInvalid.into());
                }
                self.pair = Some(if self.shared_server_session {
                    install_pair_with_session(
                        self.server.simulation(),
                        prepared,
                        self.server.zenoh_session(),
                        false,
                    )?
                } else {
                    install_pair(self.server.simulation(), prepared, options)?
                });
            } else {
                let prepared =
                    prepare_camera(self.server.simulation(), self.server.options(), options)?;
                self.runtime = prepared
                    .map(|prepared| {
                        if self.shared_server_session {
                            install_camera_with_session(
                                self.server.simulation(),
                                prepared,
                                self.server.zenoh_session(),
                                false,
                            )
                        } else {
                            install_camera(self.server.simulation(), prepared, options)
                        }
                    })
                    .transpose()?;
            }
            self.prepared_epoch = epoch;
        }
        Ok(())
    }

    /// Prepare and commit exactly one simulation tick, without a GPU/IPC step.
    /// Call capture outside critical stepping. Keeping these operations separate
    /// lets a caller report output backpressure without repeating committed physics.
    pub fn tick(&mut self, options: &CameraServerOptions) -> Result<UpdateInfo, CameraServerError> {
        self.prepare_io(options)?;
        match self.server.tick() {
            Ok(info) => Ok(info),
            Err(error) => {
                self.runtime = None;
                self.pair = None;
                Err(error.into())
            }
        }
    }

    /// Pump/admit the current frame outside critical stepping, without a tick.
    /// Busy leaves the already committed simulation state intact; it is an
    /// explicit output backpressure result, not permission to repeat a tick.
    pub fn capture(&mut self) -> Result<(Option<Progress>, Option<Progress>), CameraServerError> {
        self.require_scalar()?;
        self.require_running()?;
        self.require_prepared()?;
        if self.runtime.is_some() {
            self.output_frame()
        } else {
            Ok((None, None))
        }
    }

    pub fn update(
        &mut self,
        options: &CameraServerOptions,
    ) -> Result<CameraStep, CameraServerError> {
        self.require_scalar()?;
        let info = self.tick(options)?;
        let (pumped, requested) = self.capture()?;
        Ok(CameraStep {
            info,
            pumped,
            requested,
        })
    }

    fn output_frame(&mut self) -> Result<(Option<Progress>, Option<Progress>), CameraServerError> {
        if !self.current_camera()? {
            return Ok((None, None));
        }
        let sim = self.server.simulation();
        let runtime = self.runtime.as_mut().expect("checked active runtime");
        let pumped = runtime.output.pump(sim)?;
        if pumped == Progress::RetireRequired {
            runtime.output.retire();
        }
        let requested = runtime.output.request_due(sim)?;
        if requested == Some(Progress::RetireRequired) {
            runtime.output.retire();
        }
        Ok((Some(pumped), requested))
    }

    /// Advance one existing IPC transition without advancing physics or admitting
    /// another sample. Normal removal cancels output; replacement errors remain.
    /// Explicit Server shutdown retires camera IO before returning WorldClosed.
    pub fn pump(&mut self) -> Result<Option<Progress>, CameraServerError> {
        self.require_scalar()?;
        self.require_running()?;
        self.require_prepared()?;
        if !self.current_camera()? {
            return Ok(None);
        }
        let sim = self.server.simulation();
        let runtime = self.runtime.as_mut().expect("checked active runtime");
        let result = runtime.output.pump(sim)?;
        if result == Progress::RetireRequired {
            runtime.output.retire();
        }
        Ok(Some(result))
    }

    /// Observe image/info interest on this owner's actual declarations without
    /// admitting a sample, consuming cadence or starting a worker. These local
    /// routing snapshots are advisory, not remote receipt acknowledgements.
    /// Like pump, this may retire invalid/closed output and belongs outside tick.
    pub fn camera_demand(&mut self) -> Result<Option<(bool, bool)>, CameraServerError> {
        self.require_scalar()?;
        self.require_running()?;
        self.require_prepared()?;
        if !self.current_camera()? {
            return Ok(None);
        }
        Ok(Some(
            self.runtime
                .as_ref()
                .expect("checked active runtime")
                .output
                .demand()?,
        ))
    }

    fn current_camera(&mut self) -> Result<bool, CameraServerError> {
        let Some(runtime) = self.runtime.as_ref() else {
            return Ok(false);
        };
        let sim = self.server.simulation();
        if runtime.binding.current(sim) && runtime.output.binding_valid(sim) {
            return Ok(true);
        }
        let removed = entity_by_id(sim.world(), runtime.binding.id)
            .is_none_or(|entity| !sim.world().entity(entity).contains::<CameraSensor>());
        self.runtime = None;
        self.pair = None;
        if removed {
            Ok(false)
        } else {
            Err(CameraOutputError::OwnerInvalid.into())
        }
    }

    fn require_prepared(&mut self) -> Result<(), CameraServerError> {
        if self.server.io_preparation_required() {
            self.runtime = None;
            self.pair = None;
            return Err(gz_sim_server::Error::IoPreparationRequired.into());
        }
        Ok(())
    }

    fn require_running(&mut self) -> Result<(), CameraServerError> {
        if self.server.is_shutdown() {
            self.runtime = None;
            self.pair = None;
            return Err(gz_sim_server::Error::WorldClosed.into());
        }
        Ok(())
    }

    pub fn shutdown(&mut self) {
        self.runtime = None;
        self.pair = None;
        self.server.shutdown();
    }

    /// Retire camera IO and return the existing headless Server.
    pub fn into_server(self) -> Server {
        let Self {
            runtime,
            pair,
            server,
            ..
        } = self;
        drop(runtime);
        drop(pair);
        server
    }

    /// Retire camera IO before returning the existing preserved Scene API.
    pub fn into_simulation(self) -> Simulation {
        let Self {
            runtime,
            pair,
            server,
            ..
        } = self;
        drop(runtime);
        drop(pair);
        server.into_simulation()
    }
}

fn missing(id: u64) -> CameraOutputError {
    SnapshotError::MissingCamera(id).into()
}

fn prepare_camera(
    sim: &Simulation,
    options: &ServerOptions,
    settings: &CameraServerOptions,
) -> Result<Option<PreparedCamera>, CameraServerError> {
    let world = sim.world();
    let mut configured = world.iter_entities().filter(|entity| {
        entity.get::<EntityKind>() == Some(&EntityKind::World)
            && entity.contains::<RenderingSensorsConfiguration>()
    });
    let Some(root) = configured.next() else {
        return Ok(None);
    };
    if configured.next().is_some() {
        return Err(missing(0).into());
    }
    let mut cameras = world
        .iter_entities()
        .filter(|entity| entity.contains::<CameraSensor>());
    let Some(camera) = cameras.next() else {
        return Ok(None);
    };
    if cameras.next().is_some() {
        return Err(CameraOutputError::MultipleCameras.into());
    }
    prepare_selected_camera(sim, options, settings, camera.id(), root.id()).map(Some)
}

fn prepare_selected_camera(
    sim: &Simulation,
    options: &ServerOptions,
    settings: &CameraServerOptions,
    camera_entity: Entity,
    root: Entity,
) -> Result<PreparedCamera, CameraServerError> {
    let world = sim.world();
    let camera = world.entity(camera_entity);
    let id = camera.get::<EntityId>().ok_or_else(|| missing(0))?.get();
    let metadata = camera.get::<CameraSensor>().ok_or_else(|| missing(id))?;
    let mut nodes = Vec::new();
    let mut entity = camera.id();
    loop {
        if nodes.iter().any(|node: &CameraNode| node.entity == entity) {
            return Err(missing(id).into());
        }
        let kind = *world.get::<EntityKind>(entity).ok_or_else(|| missing(id))?;
        let local = world
            .get::<LocalName>(entity)
            .ok_or_else(|| missing(id))?
            .0
            .clone();
        let scoped = world
            .get::<Name>(entity)
            .ok_or_else(|| missing(id))?
            .0
            .clone();
        let node_id = world
            .get::<EntityId>(entity)
            .ok_or_else(|| missing(id))?
            .get();
        let parent = world.get::<ChildOf>(entity).map(ChildOf::parent);
        if local.is_empty() || local.contains("::") || entity_by_id(world, node_id) != Some(entity)
        {
            return Err(missing(id).into());
        }
        match (nodes.len(), kind) {
            (0, EntityKind::Sensor) | (1, EntityKind::Link) => {}
            (depth, EntityKind::Model) if depth >= 2 => {}
            (depth, EntityKind::World) if depth >= 3 && entity == root && parent.is_none() => {}
            _ => return Err(missing(id).into()),
        }
        nodes.push(CameraNode {
            entity,
            id: node_id,
            kind,
            local,
            scoped,
            parent,
        });
        if kind == EntityKind::World {
            break;
        }
        entity = parent.ok_or_else(|| missing(id))?;
    }
    let mut scope = String::new();
    let mut typed = String::new();
    for node in nodes.iter().rev() {
        if !scope.is_empty() {
            scope.push_str("::");
        }
        scope.push_str(&node.local);
        if scope != node.scoped {
            return Err(missing(id).into());
        }
        let prefix = match node.kind {
            EntityKind::World => "world",
            EntityKind::Model => "model",
            EntityKind::Link => "link",
            EntityKind::Sensor => "sensor",
            _ => unreachable!(),
        };
        typed.push_str(&format!("/{prefix}/{}", node.local));
    }
    let image = as_valid_topic(if metadata.0.topic.is_empty() {
        typed.push_str("/image");
        &typed
    } else {
        &metadata.0.topic
    });
    if image.is_empty() {
        return Err(SetupError::InvalidName.into());
    }
    let camera = metadata.0.camera().ok_or_else(|| missing(id))?;
    let mut info = as_valid_topic(&camera.camera_info_topic);
    if info.is_empty() {
        let mut parts: Vec<_> = image.split('/').collect();
        parts.pop();
        info = parts
            .into_iter()
            .filter(|part| !part.is_empty())
            .map(|part| format!("/{part}"))
            .collect();
        info.push_str("/camera_info");
    }
    let config = CameraOutputConfig {
        origin: settings.origin,
        frame_id: String::new(),
        image_key: options.topic_key(&image)?,
        info_key: options.topic_key(&info)?,
        worker_path: settings.worker_path.clone(),
        max_message_bytes: NonZeroUsize::new(options.max_message_bytes)
            .ok_or(SetupError::InvalidConfig)?,
    }
    .with_loaded_frame_id(sim, id)?;
    if config.frame_id.is_empty() {
        return Err(CameraOutputError::InvalidFrameId.into());
    }
    if config.image_key == config.info_key {
        return Err(CameraOutputError::DuplicateTopics.into());
    }
    TrustedWorker::from_operator_config(config.worker_path.clone())
        .map_err(CameraOutputError::Client)?;
    extract_camera(sim, id, 1, config.origin).map_err(CameraOutputError::Snapshot)?;
    let kind = if metadata
        .0
        .effective_type()
        .is_some_and(|value| value.as_str() == "depth_camera")
    {
        CameraKind::Depth
    } else {
        CameraKind::Rgb
    };
    Ok(PreparedCamera {
        kind,
        binding: CameraBinding { id, nodes },
        config,
    })
}

fn prepare_rgb_family(
    sim: &Simulation,
    options: &ServerOptions,
    settings: &CameraServerOptions,
) -> Result<Vec<PreparedCamera>, CameraServerError> {
    let roots: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|node| {
            node.get::<EntityKind>() == Some(&EntityKind::World)
                && node.contains::<RenderingSensorsConfiguration>()
        })
        .take(2)
        .map(|node| node.id())
        .collect();
    let cameras: Vec<_> = sim
        .world()
        .iter_entities()
        .filter(|node| node.contains::<CameraSensor>())
        .take(5)
        .map(|node| node.id())
        .collect();
    if roots.len() != 1 || !(3..=4).contains(&cameras.len()) {
        return Err(CameraOutputError::RgbFamilyCount.into());
    }
    let mut keys = BTreeSet::new();
    let mut ids = BTreeSet::new();
    let mut result = Vec::with_capacity(cameras.len());
    for entity in cameras {
        let prepared = prepare_selected_camera(sim, options, settings, entity, roots[0])?;
        if prepared.kind != CameraKind::Rgb {
            return Err(CameraOutputError::Snapshot(SnapshotError::InvalidCamera(
                prepared.binding.id,
            ))
            .into());
        }
        gz_sim_scene::SensorSchedule::new(
            sim.world()
                .get::<CameraSensor>(entity)
                .expect("selected camera")
                .0
                .update_rate,
        )
        .map_err(CameraOutputError::Schedule)?;
        if !ids.insert(prepared.binding.id) {
            return Err(CameraOutputError::OwnerInvalid.into());
        }
        for (key, message) in [
            (
                &prepared.config.image_key,
                gz_sim_proto::msgs::Image::full_name(),
            ),
            (
                &prepared.config.info_key,
                gz_sim_proto::msgs::CameraInfo::full_name(),
            ),
        ] {
            crate::publication_token(
                key,
                "00000000000000000000000000000001",
                "preflight",
                &message,
            )?;
            if !keys.insert(key.clone()) {
                return Err(CameraOutputError::DuplicateTopics.into());
            }
        }
        result.push(prepared);
    }
    result.sort_by_key(|camera| camera.binding.id);
    Ok(result)
}

fn prepare_pair(
    sim: &Simulation,
    options: &ServerOptions,
    settings: &CameraServerOptions,
) -> Result<Vec<PreparedCamera>, CameraServerError> {
    prepare_pair_kind(sim, options, settings, PairKind::Rgb)
}

fn prepare_depth_pair(
    sim: &Simulation,
    options: &ServerOptions,
    settings: &CameraServerOptions,
) -> Result<Vec<PreparedCamera>, CameraServerError> {
    prepare_pair_kind(sim, options, settings, PairKind::Depth)
}

fn prepare_mixed_pair(
    sim: &Simulation,
    options: &ServerOptions,
    settings: &CameraServerOptions,
) -> Result<Vec<PreparedCamera>, CameraServerError> {
    prepare_pair_kind(sim, options, settings, PairKind::Mixed)
}

fn prepare_pair_kind(
    sim: &Simulation,
    options: &ServerOptions,
    settings: &CameraServerOptions,
    kind: PairKind,
) -> Result<Vec<PreparedCamera>, CameraServerError> {
    let world = sim.world();
    let roots: Vec<_> = world
        .iter_entities()
        .filter(|node| {
            node.get::<EntityKind>() == Some(&EntityKind::World)
                && node.contains::<RenderingSensorsConfiguration>()
        })
        .take(2)
        .map(|node| node.id())
        .collect();
    let cameras: Vec<_> = world
        .iter_entities()
        .filter(|node| node.contains::<CameraSensor>())
        .take(3)
        .map(|node| node.id())
        .collect();
    if roots.len() != 1 || cameras.len() != 2 {
        return Err(if kind == PairKind::Mixed {
            CameraOutputError::MixedCameraCount
        } else {
            CameraOutputError::PairCameraCount
        }
        .into());
    }
    let mut keys = BTreeSet::new();
    let mut ids = BTreeSet::new();
    let mut result = Vec::with_capacity(2);
    for entity in cameras {
        let prepared = prepare_selected_camera(sim, options, settings, entity, roots[0])?;
        let id = prepared.binding.id;
        gz_sim_scene::SensorSchedule::new(
            world
                .get::<CameraSensor>(entity)
                .expect("selected camera")
                .0
                .update_rate,
        )
        .map_err(CameraOutputError::Schedule)?;
        if !ids.insert(id) {
            return Err(CameraOutputError::OwnerInvalid.into());
        }
        let admitted = match kind {
            PairKind::Rgb => prepared.kind == CameraKind::Rgb,
            PairKind::Depth => prepared.kind == CameraKind::Depth,
            PairKind::Mixed => true,
        };
        if !admitted {
            return Err(CameraOutputError::Snapshot(SnapshotError::InvalidCamera(id)).into());
        }
        for (key, message) in [
            (
                &prepared.config.image_key,
                gz_sim_proto::msgs::Image::full_name(),
            ),
            (
                &prepared.config.info_key,
                gz_sim_proto::msgs::CameraInfo::full_name(),
            ),
        ] {
            crate::publication_token(
                key,
                "00000000000000000000000000000001",
                "preflight",
                &message,
            )?;
        }
        if !keys.insert(prepared.config.image_key.clone())
            || !keys.insert(prepared.config.info_key.clone())
        {
            return Err(CameraOutputError::DuplicateTopics.into());
        }
        result.push(prepared);
    }
    if kind == PairKind::Mixed && result[0].kind == result[1].kind {
        return Err(CameraOutputError::MixedCameraKinds.into());
    }
    result.sort_by_key(|camera| camera.binding.id);
    Ok(result)
}

fn install_pair(
    sim: &Simulation,
    prepared: Vec<PreparedCamera>,
    settings: &CameraServerOptions,
) -> Result<CameraPair, CameraOutputError> {
    let session = zenoh::open(settings.zenoh.clone()).wait()?;
    install_pair_with_session(sim, prepared, session, true)
}

fn install_pair_with_session(
    sim: &Simulation,
    prepared: Vec<PreparedCamera>,
    session: Session,
    owns_session: bool,
) -> Result<CameraPair, CameraOutputError> {
    let mut pair = CameraPair {
        outputs: BTreeMap::new(),
        session,
        owns_session,
    };
    let scope =
        CameraWriterScope::new(NonZeroUsize::new(prepared.len()).expect("validated camera family"));
    for prepared in prepared {
        let id = prepared.binding.id;
        let output = ScheduledCameraOutput::prepare_selected(
            sim,
            &pair.session,
            &scope,
            id,
            prepared.config,
        )?;
        pair.outputs.insert(
            id,
            PairOutput {
                binding: prepared.binding,
                output,
            },
        );
    }
    Ok(pair)
}

fn install_camera(
    sim: &Simulation,
    prepared: PreparedCamera,
    settings: &CameraServerOptions,
) -> Result<CameraRuntime, CameraOutputError> {
    let session = zenoh::open(settings.zenoh.clone()).wait()?;
    install_camera_with_session(sim, prepared, session, true)
}

fn install_camera_with_session(
    sim: &Simulation,
    prepared: PreparedCamera,
    session: Session,
    owns_session: bool,
) -> Result<CameraRuntime, CameraOutputError> {
    let scope = CameraWriterScope::new(NonZeroUsize::new(1).expect("one camera"));
    let output = ScheduledCameraOutput::prepare(sim, &session, &scope, prepared.config)?
        .ok_or(CameraOutputError::OwnerInvalid)?;
    Ok(CameraRuntime {
        binding: prepared.binding,
        output,
        session,
        owns_session,
    })
}

#[cfg(test)]
#[path = "../tests/private/application.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/m111_rgb_family.rs"]
mod rgb_family_tests;
