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
use gz_sim_scene::{CameraSensor, EntityKind, LocalName, Name, RenderingSensorsConfiguration};
use gz_sim_server::{Server, ServerOptions, SetupError, as_valid_topic};
use std::{num::NonZeroUsize, path::PathBuf};
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
struct PreparedCamera {
    binding: CameraBinding,
    config: CameraOutputConfig,
}
struct CameraRuntime {
    binding: CameraBinding,
    output: ScheduledCameraOutput,
    session: Session,
}
impl Drop for CameraRuntime {
    fn drop(&mut self) {
        self.output.retire();
        // Teardown may wait, including native final drops. This owner never
        // enters the Bevy schedule or Server's critical tick.
        let _ = self.session.close().wait();
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
    server: Server,
    prepared_epoch: u64,
}
impl CameraServer {
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
            server,
            prepared_epoch: epoch,
        })
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
        self.runtime.is_some()
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
            }
            return Err(error.into());
        }
        if epoch != self.prepared_epoch {
            self.runtime = None;
            // Do not advance the epoch cache until preparation really succeeds.
            // The returned error is never converted into a repeated update here.
            let prepared =
                prepare_camera(self.server.simulation(), self.server.options(), options)?;
            self.runtime = prepared
                .map(|p| install_camera(self.server.simulation(), p, options))
                .transpose()?;
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
                Err(error.into())
            }
        }
    }

    /// Pump/admit the current frame outside critical stepping, without a tick.
    /// Busy leaves the already committed simulation state intact; it is an
    /// explicit output backpressure result, not permission to repeat a tick.
    pub fn capture(&mut self) -> Result<(Option<Progress>, Option<Progress>), CameraServerError> {
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
        if removed {
            Ok(false)
        } else {
            Err(CameraOutputError::OwnerInvalid.into())
        }
    }

    fn require_prepared(&mut self) -> Result<(), CameraServerError> {
        if self.server.io_preparation_required() {
            self.runtime = None;
            return Err(gz_sim_server::Error::IoPreparationRequired.into());
        }
        Ok(())
    }

    fn require_running(&mut self) -> Result<(), CameraServerError> {
        if self.server.is_shutdown() {
            self.runtime = None;
            return Err(gz_sim_server::Error::WorldClosed.into());
        }
        Ok(())
    }

    pub fn shutdown(&mut self) {
        self.runtime = None;
        self.server.shutdown();
    }

    /// Retire camera IO and return the existing headless Server.
    pub fn into_server(self) -> Server {
        let Self {
            runtime, server, ..
        } = self;
        drop(runtime);
        server
    }

    /// Retire camera IO before returning the existing preserved Scene API.
    pub fn into_simulation(self) -> Simulation {
        let Self {
            runtime, server, ..
        } = self;
        drop(runtime);
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
    let id = camera.get::<EntityId>().ok_or_else(|| missing(0))?.get();
    let metadata = camera.get::<CameraSensor>().expect("selected camera");
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
            (depth, EntityKind::World) if depth >= 3 && entity == root.id() && parent.is_none() => {
            }
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
    Ok(Some(PreparedCamera {
        binding: CameraBinding { id, nodes },
        config,
    }))
}

fn install_camera(
    sim: &Simulation,
    prepared: PreparedCamera,
    settings: &CameraServerOptions,
) -> Result<CameraRuntime, CameraOutputError> {
    let session = zenoh::open(settings.zenoh.clone()).wait()?;
    let scope = CameraWriterScope::new(NonZeroUsize::new(1).expect("one camera"));
    let output = ScheduledCameraOutput::prepare(sim, &session, &scope, prepared.config)?
        .ok_or(CameraOutputError::OwnerInvalid)?;
    Ok(CameraRuntime {
        binding: prepared.binding,
        output,
        session,
    })
}

#[cfg(test)]
#[path = "../tests/private/application.rs"]
mod tests;
