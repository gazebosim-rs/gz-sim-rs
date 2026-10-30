// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy::{
    asset::RenderAssetUsages,
    camera::RenderTarget,
    core_pipeline::tonemapping::Tonemapping,
    image::Image,
    pbr::{PreparedMaterial, RenderMaterialInstances},
    prelude::*,
    render::{
        RenderApp, RenderPlugin,
        erased_render_asset::ErasedRenderAssets,
        error_handler::{RenderErrorHandler, RenderErrorPolicy},
        mesh::RenderMesh,
        render_asset::RenderAssets,
        render_resource::{CachedPipelineState, PipelineCache, TextureFormat},
        renderer::{RenderAdapter, RenderAdapterInfo, RenderDevice, RenderInstance, RenderQueue},
        settings::RenderCreation,
        texture::GpuImage,
        view::{
            DebandDither,
            screenshot::{Screenshot, ScreenshotCaptured},
        },
    },
    shader::ShaderCacheError,
    window::ExitCondition,
};
use glam::DVec3;
use gz_render_frame::{ConversionError, WorldPose, to_render_pose};
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};

const WIDTH: u32 = 64;
const HEIGHT: u32 = 64;
const MAX_OBJECTS: usize = 128;
// Concurrent wgpu Vulkan instance creation crashes with an installed NVIDIA ICD
// even when that ICD has no usable device. Keep our own creation serialized.
static VULKAN_INIT: Mutex<()> = Mutex::new(());

/// A box in world coordinates. Full dimensions are in metres.
#[derive(Debug, Clone)]
pub struct BoxVisual {
    pub id: u64,
    pub pose: WorldPose,
    pub size: DVec3,
    pub color: [f32; 3],
}

/// A complete committed scene snapshot, with world-local IDs and simulation time.
#[derive(Debug, Clone)]
pub struct Snapshot {
    pub epoch: u64,
    pub time: Duration,
    pub origin: DVec3,
    pub visuals: Vec<BoxVisual>,
}

/// Captured RGBA8 sRGB pixels and the snapshot that produced them.
#[derive(Debug)]
pub struct Frame {
    pub epoch: u64,
    pub time: Duration,
    pub rgba: Vec<u8>,
}

#[derive(Debug, thiserror::Error)]
pub enum RenderError {
    #[error("Vulkan initialization lock is poisoned")]
    InitializationPoisoned,
    #[error("no compatible Vulkan adapter: {0}")]
    Adapter(#[from] wgpu::RequestAdapterError),
    #[error("could not create rendering device: {0}")]
    Device(#[from] wgpu::RequestDeviceError),
    #[error("invalid pose: {0}")]
    Pose(#[from] ConversionError),
    #[error("snapshot exceeds object limit or has duplicate or zero IDs")]
    InvalidIds,
    #[error("box dimensions and RGB values must be finite and in range")]
    InvalidVisual,
    #[error("a readback is already in flight")]
    Busy,
    #[error("the previous snapshot has not reached a renderer update")]
    SnapshotBusy,
    #[error("scene assets or pipelines are not yet ready")]
    NotReady,
    #[error("render device stopped: {0}")]
    RenderStopped(String),
    #[error("shader pipeline failed: {0}")]
    Shader(String),
    #[error("readback completed without pixel data")]
    ReadbackFailed,
    #[error("readback state is poisoned")]
    ReadbackPoisoned,
    #[error("no scene snapshot is installed")]
    NoSnapshot,
}

#[derive(Clone)]
enum StoppedReason {
    Device(String),
    Shader(String),
}
#[derive(Resource, Default)]
struct DeviceFailure(Option<StoppedReason>);

fn permanent_shader_error(error: &ShaderCacheError) -> Option<String> {
    match error {
        ShaderCacheError::ShaderNotLoaded(_) | ShaderCacheError::ShaderImportNotYetAvailable => {
            None
        }
        ShaderCacheError::ProcessShaderError(_) | ShaderCacheError::CreateShaderModule(_) => {
            Some(error.to_string())
        }
    }
}

struct VisualEntity {
    entity: Entity,
    material: Handle<StandardMaterial>,
}
struct Pending {
    epoch: u64,
    time: Duration,
    valid: bool,
    pixels: Arc<Mutex<Option<Option<Vec<u8>>>>>,
}

/// Whether an update submitted a new frame or is waiting for the previous one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpdateStatus {
    Advanced,
    Waiting,
}

/// Owns all renderer state. Drive it outside simulation stepping.
///
/// The target and camera are currently fixed at 64×64, looking from (0,0,5)
/// towards the render origin. Only boxes are supported. This is not a sensor API.
pub struct Renderer {
    app: App,
    target: Handle<Image>,
    mesh: Handle<Mesh>,
    visuals: BTreeMap<u64, VisualEntity>,
    stamp: Option<(u64, Duration)>,
    pending: Option<Pending>,
    ready_last_update: bool,
    ready: bool,
    adapter_info: wgpu::AdapterInfo,
    in_flight: Option<Arc<AtomicBool>>,
    snapshot_dirty: bool,
}

impl Renderer {
    /// Requests a Vulkan adapter and device, returning capability failures.
    pub fn new() -> Result<Self, RenderError> {
        let instance = {
            let _guard = VULKAN_INIT
                .lock()
                .map_err(|_| RenderError::InitializationPoisoned)?;
            wgpu::Instance::new(wgpu::InstanceDescriptor {
                backends: wgpu::Backends::VULKAN,
                ..wgpu::InstanceDescriptor::new_without_display_handle()
            })
        };
        let adapter =
            pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()))?;
        let info = adapter.get_info();
        let (device, queue) =
            pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default()))?;
        let render_creation = RenderCreation::manual(
            RenderDevice::from(device),
            RenderQueue::new(queue),
            RenderAdapterInfo::new(info.clone()),
            RenderAdapter::new(adapter),
            RenderInstance::new(instance),
        );
        let mut app = App::new();
        let mut plugins = DefaultPlugins
            .set(WindowPlugin {
                primary_window: None,
                exit_condition: ExitCondition::DontExit,
                ..default()
            })
            .disable::<bevy::log::LogPlugin>()
            .disable::<bevy::app::PanicHandlerPlugin>()
            .disable::<bevy::app::TerminalCtrlCHandlerPlugin>()
            .set(RenderPlugin {
                render_creation,
                synchronous_pipeline_compilation: true,
                ..default()
            });
        // The submission fence assumes render work finishes submission in update().
        if plugins.contains::<bevy::render::pipelined_rendering::PipelinedRenderingPlugin>() {
            plugins =
                plugins.disable::<bevy::render::pipelined_rendering::PipelinedRenderingPlugin>();
        }
        app.add_plugins(plugins);
        app.insert_resource(DeviceFailure::default());
        app.insert_resource(RenderErrorHandler(|error, main, _render| {
            main.resource_mut::<DeviceFailure>().0 =
                Some(StoppedReason::Device(error.description.clone()));
            RenderErrorPolicy::StopRendering
        }));
        app.finish();
        app.cleanup();
        let world = app.world_mut();
        let mut image =
            Image::new_target_texture(WIDTH, HEIGHT, TextureFormat::Rgba8UnormSrgb, None);
        image.asset_usage = RenderAssetUsages::RENDER_WORLD;
        let target = world.resource_mut::<Assets<Image>>().add(image);
        world.spawn((
            Camera3d::default(),
            Tonemapping::None,
            DebandDither::Disabled,
            RenderTarget::Image(target.clone().into()),
            Transform::from_xyz(0.0, 0.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
        ));
        let mesh = world
            .resource_mut::<Assets<Mesh>>()
            .add(Cuboid::new(1.0, 1.0, 1.0));
        Ok(Self {
            app,
            target,
            mesh,
            visuals: BTreeMap::new(),
            stamp: None,
            pending: None,
            ready_last_update: false,
            ready: false,
            adapter_info: info,
            in_flight: None,
            snapshot_dirty: false,
        })
    }

    /// Identity and type of the selected adapter, including whether it is CPU based.
    pub fn adapter_info(&self) -> &wgpu::AdapterInfo {
        &self.adapter_info
    }

    /// Atomically replaces the derived scene after validating the entire snapshot.
    pub fn set_snapshot(&mut self, snapshot: &Snapshot) -> Result<(), RenderError> {
        self.check_device()?;
        if snapshot.visuals.len() > MAX_OBJECTS {
            return Err(RenderError::InvalidIds);
        }
        let mut ids = BTreeSet::new();
        let mut converted = Vec::with_capacity(snapshot.visuals.len());
        for visual in &snapshot.visuals {
            if visual.id == 0 || !ids.insert(visual.id) {
                return Err(RenderError::InvalidIds);
            }
            let size = visual.size.as_vec3();
            if !visual.size.is_finite()
                || !size.is_finite()
                || size.min_element() <= 0.0
                || visual
                    .color
                    .iter()
                    .any(|c| !c.is_finite() || !(0.0..=1.0).contains(c))
            {
                return Err(RenderError::InvalidVisual);
            }
            let pose = to_render_pose(visual.pose, snapshot.origin)?;
            converted.push((
                visual,
                Transform {
                    translation: pose.translation,
                    rotation: pose.rotation,
                    scale: Vec3::new(size.x, size.z, size.y),
                },
            ));
        }
        if self.snapshot_dirty {
            return Err(RenderError::SnapshotBusy);
        }
        self.cancel_readback();
        let removed: Vec<_> = self
            .visuals
            .keys()
            .copied()
            .filter(|id| !ids.contains(id))
            .collect();
        for id in removed {
            let old = self.visuals.remove(&id).expect("key was just collected");
            self.app.world_mut().despawn(old.entity);
            self.app
                .world_mut()
                .resource_mut::<Assets<StandardMaterial>>()
                .remove(old.material.id());
        }
        for (visual, transform) in converted {
            let color = Color::srgb(visual.color[0], visual.color[1], visual.color[2]);
            if let Some(existing) = self.visuals.get(&visual.id) {
                self.app
                    .world_mut()
                    .entity_mut(existing.entity)
                    .insert(transform);
                if let Some(mut material) = self
                    .app
                    .world_mut()
                    .resource_mut::<Assets<StandardMaterial>>()
                    .get_mut(&existing.material)
                {
                    material.base_color = color;
                }
            } else {
                let material = self
                    .app
                    .world_mut()
                    .resource_mut::<Assets<StandardMaterial>>()
                    .add(StandardMaterial {
                        base_color: color,
                        unlit: true,
                        ..default()
                    });
                let entity = self
                    .app
                    .world_mut()
                    .spawn((
                        Mesh3d(self.mesh.clone()),
                        MeshMaterial3d(material.clone()),
                        transform,
                    ))
                    .id();
                self.visuals
                    .insert(visual.id, VisualEntity { entity, material });
            }
        }
        self.stamp = Some((snapshot.epoch, snapshot.time));
        self.snapshot_dirty = true;
        self.ready_last_update = false;
        self.ready = false;
        Ok(())
    }

    fn check_device(&self) -> Result<(), RenderError> {
        match &self.app.world().resource::<DeviceFailure>().0 {
            Some(StoppedReason::Device(error)) => Err(RenderError::RenderStopped(error.clone())),
            Some(StoppedReason::Shader(error)) => Err(RenderError::Shader(error.clone())),
            None => Ok(()),
        }
    }

    /// Runs one Bevy update. This can perform rendering work and must not run in a physics step.
    /// A reported device failure is terminal; drop this renderer and create another.
    pub fn update(&mut self) -> Result<UpdateStatus, RenderError> {
        self.check_device()?;
        if let Some(done) = &self.in_flight {
            if !done.load(Ordering::Acquire) {
                if let Err(error) = self
                    .app
                    .world()
                    .resource::<RenderDevice>()
                    .poll(wgpu::PollType::Poll)
                {
                    let error = error.to_string();
                    self.app.world_mut().resource_mut::<DeviceFailure>().0 =
                        Some(StoppedReason::Device(error.clone()));
                    self.cancel_readback();
                    self.ready = false;
                    return Err(RenderError::RenderStopped(error));
                }
                if !done.load(Ordering::Acquire) {
                    return Ok(UpdateStatus::Waiting);
                }
            }
            self.in_flight = None;
        }
        self.app.update();
        self.snapshot_dirty = false;
        let done = Arc::new(AtomicBool::new(false));
        let callback = done.clone();
        self.app
            .world()
            .resource::<RenderQueue>()
            .on_submitted_work_done(move || {
                callback.store(true, Ordering::Release);
            });
        self.in_flight = Some(done);
        if let Err(error) = self.check_device() {
            self.cancel_readback();
            self.ready = false;
            return Err(error);
        }
        let failure = self.app.get_sub_app(RenderApp).and_then(|render| {
            render
                .world()
                .get_resource::<PipelineCache>()
                .and_then(|cache| {
                    cache
                        .pipelines()
                        .find_map(|pipeline| match &pipeline.state {
                            CachedPipelineState::Err(error) => permanent_shader_error(error),
                            _ => None,
                        })
                })
        });
        if let Some(error) = failure {
            self.app.world_mut().resource_mut::<DeviceFailure>().0 =
                Some(StoppedReason::Shader(error.clone()));
            self.cancel_readback();
            self.ready = false;
            return Err(RenderError::Shader(error));
        }
        let ready = self.scene_assets_ready();
        self.ready = ready && self.ready_last_update;
        self.ready_last_update = ready;
        Ok(UpdateStatus::Advanced)
    }

    fn scene_assets_ready(&self) -> bool {
        let Some(render) = self.app.get_sub_app(RenderApp) else {
            return false;
        };
        let world = render.world();
        let Some(cache) = world.get_resource::<PipelineCache>() else {
            return false;
        };
        if cache.pipelines().next().is_none()
            || cache
                .pipelines()
                .any(|pipeline| !matches!(pipeline.state, CachedPipelineState::Ok(_)))
        {
            return false;
        }
        let Some(images) = world.get_resource::<RenderAssets<GpuImage>>() else {
            return false;
        };
        let Some(meshes) = world.get_resource::<RenderAssets<RenderMesh>>() else {
            return false;
        };
        let Some(materials) = world.get_resource::<ErasedRenderAssets<PreparedMaterial>>() else {
            return false;
        };
        let Some(instances) = world.get_resource::<RenderMaterialInstances>() else {
            return false;
        };
        images.get(self.target.id()).is_some()
            && self.visuals.values().all(|visual| {
                let Some(visibility) = self.app.world().get::<ViewVisibility>(visual.entity) else {
                    return false;
                };
                !visibility.get()
                    || (meshes.get(self.mesh.id()).is_some()
                        && materials.get(visual.material.id()).is_some()
                        && instances.instances.contains_key(
                            &bevy::render::sync_world::MainEntity::from(visual.entity),
                        ))
            })
    }

    /// Request one asynchronous readback. A cancelled request occupies the slot until it retires.
    pub fn request_readback(&mut self) -> Result<(), RenderError> {
        self.check_device()?;
        if self.pending.is_some() {
            return Err(RenderError::Busy);
        }
        let (epoch, time) = self.stamp.ok_or(RenderError::NoSnapshot)?;
        if !self.ready {
            return Err(RenderError::NotReady);
        }
        let pixels = Arc::new(Mutex::new(None));
        let copy = pixels.clone();
        self.app
            .world_mut()
            .spawn(Screenshot::image(self.target.clone()))
            .observe(move |event: On<ScreenshotCaptured>| {
                if let Ok(mut slot) = copy.lock() {
                    *slot = Some(event.image.data.clone());
                }
            });
        self.pending = Some(Pending {
            epoch,
            time,
            valid: true,
            pixels,
        });
        Ok(())
    }

    /// Whether a readback still occupies the bounded slot, including after cancellation.
    pub fn readback_in_flight(&self) -> bool {
        self.pending.is_some()
    }

    /// Suppresses delivery of an in-flight frame. The GPU operation may still complete.
    pub fn cancel_readback(&mut self) {
        if let Some(pending) = &mut self.pending {
            pending.valid = false;
        }
    }

    /// Take a completed frame; cancelled frames retire without being delivered.
    pub fn take_frame(&mut self) -> Result<Option<Frame>, RenderError> {
        self.check_device()?;
        let Some(pending) = self.pending.as_ref() else {
            return Ok(None);
        };
        let completed = pending
            .pixels
            .lock()
            .map_err(|_| RenderError::ReadbackPoisoned)?
            .take();
        let Some(pixels) = completed else {
            return Ok(None);
        };
        let pending = self
            .pending
            .take()
            .expect("pending readback was just checked");
        if !pending.valid {
            return Ok(None);
        }
        let rgba = pixels.ok_or(RenderError::ReadbackFailed)?;
        Ok(Some(Frame {
            epoch: pending.epoch,
            time: pending.time,
            rgba,
        }))
    }
}

#[cfg(test)]
#[path = "../tests/private/gpu.rs"]
mod tests;
