// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use bevy::{
    asset::RenderAssetUsages,
    camera::{CameraProjection, RenderTarget, SubCameraView},
    core_pipeline::tonemapping::Tonemapping,
    image::Image,
    pbr::{PreparedMaterial, RenderMaterialInstances},
    prelude::*,
    render::{
        RenderApp, RenderPlugin, RenderStartup,
        batching::gpu_preprocessing::{GpuPreprocessingMode, GpuPreprocessingSupport},
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
use gz_render_frame::{
    ConversionError, WorldPose,
    camera::{Calibration, to_render_camera_pose},
    to_render_pose,
};
use std::{
    collections::{BTreeMap, BTreeSet},
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    time::Duration,
};

#[path = "depth.rs"]
mod depth;
pub use depth::DepthFrame;

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

/// One explicit directional light, independent of SDF's relative intensity scale.
///
/// Rotation maps local Gazebo axes to world axes. Direction is the local
/// propagation vector; its magnitude has no effect. Color is linear RGB.
/// Illuminance is in lux. This does not establish native radiometric parity.
#[derive(Debug, Clone)]
pub struct DirectionalWorldLight {
    pub id: u64,
    pub rotation: glam::DQuat,
    pub direction: DVec3,
    pub color: [f64; 3],
    pub illuminance_lux: f64,
}

fn derive_light(
    light: &DirectionalWorldLight,
) -> Result<(DirectionalLight, Transform), RenderError> {
    // Reuse the authoritative orientation contract without introducing position.
    to_render_pose(
        WorldPose {
            translation: DVec3::ZERO,
            rotation: light.rotation,
        },
        DVec3::ZERO,
    )?;
    let lux = light.illuminance_lux as f32;
    if light.id == 0
        || !light.direction.is_finite()
        || light.direction == DVec3::ZERO
        || light
            .color
            .iter()
            .any(|c| !c.is_finite() || !(0.0..=1.0).contains(c))
        || !light.illuminance_lux.is_finite()
        || light.illuminance_lux < 0.0
        || !lux.is_finite()
        || (light.illuminance_lux > 0.0 && lux == 0.0)
    {
        return Err(RenderError::InvalidLight);
    }
    // Scale before normalizing so finite very large/small vectors remain valid.
    let local = (light.direction / light.direction.abs().max_element()).normalize();
    let world = light.rotation * local;
    let direction = to_render_pose(
        WorldPose {
            translation: world,
            rotation: glam::DQuat::IDENTITY,
        },
        DVec3::ZERO,
    )?
    .translation
    .normalize();
    Ok((
        DirectionalLight {
            color: Color::linear_rgb(
                light.color[0] as f32,
                light.color[1] as f32,
                light.color[2] as f32,
            ),
            illuminance: lux,
            shadow_maps_enabled: false,
            contact_shadows_enabled: false,
            ..default()
        },
        Transform::from_rotation(Quat::from_rotation_arc(Vec3::NEG_Z, direction)),
    ))
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
    #[error("directional light ID, direction, linear RGB or illuminance is invalid")]
    InvalidLight,
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
    #[error("this renderer supports only a 64 by 64 calibrated target")]
    InvalidCameraDimensions,
    #[error("metric depth requires every box strictly beyond the calibrated near plane")]
    DepthNearClip,
    #[error("no calibrated metric-depth snapshot is installed")]
    NoDepthSnapshot,
}

#[derive(Debug, Clone)]
struct CalibratedProjection(Calibration);

impl CameraProjection for CalibratedProjection {
    fn get_clip_from_view(&self) -> Mat4 {
        self.0.render_projection()
    }

    fn get_clip_from_view_for_sub(&self, sub: &SubCameraView) -> Mat4 {
        let full = sub.full_size.as_vec2();
        let size = sub.size.as_vec2();
        let scale = full / size;
        let offset = (full - 2.0 * sub.offset - size) / size;
        let crop = Mat4::from_cols(
            Vec4::new(scale.x, 0.0, 0.0, 0.0),
            Vec4::new(0.0, scale.y, 0.0, 0.0),
            Vec4::Z,
            Vec4::new(offset.x, -offset.y, 0.0, 1.0),
        );
        crop * self.get_clip_from_view()
    }

    fn update(&mut self, _width: f32, _height: f32) {
        // The renderer's target and calibrated dimensions are fixed at 64×64.
    }

    fn far(&self) -> f32 {
        self.0.config().far as f32
    }

    fn get_frustum_corners(&self, z_near: f32, z_far: f32) -> [bevy::math::Vec3A; 8] {
        let config = self.0.config();
        let corners = |z: f32| {
            let distance = f64::from(z.abs());
            let left = (-config.cx * distance / config.fx) as f32;
            let right = ((f64::from(config.width) - config.cx) * distance / config.fx) as f32;
            let bottom = (-(f64::from(config.height) - config.cy) * distance / config.fy) as f32;
            let top = (config.cy * distance / config.fy) as f32;
            [
                bevy::math::Vec3A::new(right, bottom, z),
                bevy::math::Vec3A::new(right, top, z),
                bevy::math::Vec3A::new(left, top, z),
                bevy::math::Vec3A::new(left, bottom, z),
            ]
        };
        let near = corners(z_near);
        let far = corners(z_far);
        [
            near[0], near[1], near[2], near[3], far[0], far[1], far[2], far[3],
        ]
    }
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
/// The target is fixed at 64×64. The default camera looks from (0,0,5) toward
/// the render origin; calibrated snapshots supply an explicit pose. Only boxes
/// are supported, with optional explicit directional illumination. This is not a sensor API.
pub struct Renderer {
    app: App,
    camera: Entity,
    target: Handle<Image>,
    mesh: Handle<Mesh>,
    visuals: BTreeMap<u64, VisualEntity>,
    light: Option<(u64, Entity)>,
    stamp: Option<(u64, Duration)>,
    pending: Option<Pending>,
    depth_pending: Option<depth::DepthPending>,
    depth_calibration: Option<Calibration>,
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
            // Scalar-math builds can give MeshUniform a smaller native size
            // than its shader layout. Bevy's CPU builder writes shader layout.
            .set(bevy::pbr::PbrPlugin {
                use_gpu_instance_buffer_builder: false,
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
        app.sub_app_mut(RenderApp).add_systems(
            RenderStartup,
            (|mut support: ResMut<GpuPreprocessingSupport>| {
                support.max_supported_mode = GpuPreprocessingMode::None;
            })
            .after(bevy::render::init_gpu_resource::<GpuPreprocessingSupport>),
        );
        app.cleanup();
        let world = app.world_mut();
        let mut image =
            Image::new_target_texture(WIDTH, HEIGHT, TextureFormat::Rgba8UnormSrgb, None);
        image.asset_usage = RenderAssetUsages::RENDER_WORLD;
        let target = world.resource_mut::<Assets<Image>>().add(image);
        let camera = world
            .spawn((
                Camera3d::default(),
                Tonemapping::None,
                DebandDither::Disabled,
                RenderTarget::Image(target.clone().into()),
                Transform::from_xyz(0.0, 0.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
            ))
            .id();
        let mesh = world
            .resource_mut::<Assets<Mesh>>()
            .add(Cuboid::new(1.0, 1.0, 1.0));
        Ok(Self {
            app,
            camera,
            target,
            mesh,
            visuals: BTreeMap::new(),
            light: None,
            stamp: None,
            pending: None,
            depth_pending: None,
            depth_calibration: None,
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
    /// Install a scene using the legacy fixed camera, even after a calibrated snapshot.
    pub fn set_snapshot(&mut self, snapshot: &Snapshot) -> Result<(), RenderError> {
        self.set_snapshot_with_light(snapshot, None)
    }

    /// Atomically installs one explicit directional light and a fixed-camera scene.
    /// Lux and linear light color are Bevy inputs, not SDF intensity conversion.
    pub fn set_directional_snapshot(
        &mut self,
        snapshot: &Snapshot,
        light: &DirectionalWorldLight,
    ) -> Result<(), RenderError> {
        self.set_snapshot_with_light(snapshot, Some(light))
    }

    fn set_snapshot_with_light(
        &mut self,
        snapshot: &Snapshot,
        light: Option<&DirectionalWorldLight>,
    ) -> Result<(), RenderError> {
        self.check_device()?;
        let derived_light = light.map(derive_light).transpose()?;
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
        if light.is_some_and(|light| ids.contains(&light.id)) {
            return Err(RenderError::InvalidLight);
        }
        if self.snapshot_dirty {
            return Err(RenderError::SnapshotBusy);
        }
        if self.depth_calibration.take().is_some() {
            self.app
                .world_mut()
                .entity_mut(self.camera)
                .insert((Msaa::default(), Camera3d::default()));
        }
        let was_lit = self.light.is_some();
        let lit = light.is_some();
        self.cancel_readback();
        match (light, derived_light) {
            (Some(light), Some(derived)) => {
                if let Some((id, entity)) = self.light {
                    if id == light.id {
                        self.app.world_mut().entity_mut(entity).insert(derived);
                    } else {
                        self.app.world_mut().despawn(entity);
                        self.light = Some((light.id, self.app.world_mut().spawn(derived).id()));
                    }
                } else {
                    self.light = Some((light.id, self.app.world_mut().spawn(derived).id()));
                }
                self.app
                    .world_mut()
                    .entity_mut(self.camera)
                    .insert(AmbientLight {
                        brightness: 0.0,
                        ..default()
                    });
            }
            _ => {
                if let Some((_, entity)) = self.light.take() {
                    self.app.world_mut().despawn(entity);
                    self.app
                        .world_mut()
                        .entity_mut(self.camera)
                        .remove::<AmbientLight>();
                }
            }
        }
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
                    if lit || was_lit {
                        material.unlit = !lit;
                        material.reflectance = if lit {
                            0.0
                        } else {
                            StandardMaterial::default().reflectance
                        };
                        material.perceptual_roughness = if lit {
                            1.0
                        } else {
                            StandardMaterial::default().perceptual_roughness
                        };
                    }
                }
            } else {
                let mut material = StandardMaterial {
                    base_color: color,
                    unlit: true,
                    ..default()
                };
                if lit {
                    material.unlit = false;
                    material.reflectance = 0.0;
                    material.perceptual_roughness = 1.0;
                }
                let material = self
                    .app
                    .world_mut()
                    .resource_mut::<Assets<StandardMaterial>>()
                    .add(material);
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
        self.app.world_mut().entity_mut(self.camera).insert((
            Transform::from_xyz(0.0, 0.0, 5.0).looking_at(Vec3::ZERO, Vec3::Y),
            Projection::Perspective(default()),
        ));
        self.snapshot_dirty = true;
        self.ready_last_update = false;
        self.ready = false;
        Ok(())
    }

    /// Install a calibrated 64×64 camera with this complete scene snapshot.
    /// The pose is a Gazebo camera world pose, rebased at the snapshot origin.
    pub fn set_calibrated_snapshot(
        &mut self,
        snapshot: &Snapshot,
        calibration: Calibration,
        pose: WorldPose,
    ) -> Result<(), RenderError> {
        self.set_calibrated_snapshot_with_light(snapshot, calibration, pose, None)
    }

    /// Installs a calibrated scene illuminated by one explicit lux-valued light.
    pub fn set_calibrated_directional_snapshot(
        &mut self,
        snapshot: &Snapshot,
        calibration: Calibration,
        pose: WorldPose,
        light: &DirectionalWorldLight,
    ) -> Result<(), RenderError> {
        self.set_calibrated_snapshot_with_light(snapshot, calibration, pose, Some(light))
    }

    fn set_calibrated_snapshot_with_light(
        &mut self,
        snapshot: &Snapshot,
        calibration: Calibration,
        pose: WorldPose,
        light: Option<&DirectionalWorldLight>,
    ) -> Result<(), RenderError> {
        self.check_device()?;
        let config = calibration.config();
        if config.width != WIDTH || config.height != HEIGHT {
            return Err(RenderError::InvalidCameraDimensions);
        }
        let camera = to_render_camera_pose(pose, snapshot.origin)?;
        self.set_snapshot_with_light(snapshot, light)?;
        self.app.world_mut().entity_mut(self.camera).insert((
            Transform::from_translation(camera.translation).with_rotation(camera.rotation),
            Projection::custom(CalibratedProjection(calibration)),
        ));
        Ok(())
    }

    /// Install a calibrated, unlit metric-depth scene. Every box must be wholly
    /// forward of the near plane; near/intersecting/behind geometry is refused.
    /// This does not implement native near-range -INF sensor semantics.
    pub fn set_calibrated_depth_snapshot(
        &mut self,
        snapshot: &Snapshot,
        calibration: Calibration,
        pose: WorldPose,
    ) -> Result<(), RenderError> {
        self.check_device()?;
        if calibration.config().width != WIDTH || calibration.config().height != HEIGHT {
            return Err(RenderError::InvalidCameraDimensions);
        }
        if snapshot.visuals.len() > MAX_OBJECTS {
            return Err(RenderError::InvalidIds);
        }
        depth::validate_near(snapshot, calibration, pose)?;
        self.set_calibrated_snapshot(snapshot, calibration, pose)?;
        self.app.world_mut().entity_mut(self.camera).insert((
            Msaa::Off,
            Camera3d {
                depth_texture_usages: (wgpu::TextureUsages::RENDER_ATTACHMENT
                    | wgpu::TextureUsages::COPY_SRC)
                    .into(),
                ..default()
            },
        ));
        self.depth_calibration = Some(calibration);
        Ok(())
    }

    /// Copy the actual stored Bevy depth attachment. Shares the single bounded
    /// readback slot with RGB; it does not run a render or block for the device.
    pub fn request_depth_readback(&mut self) -> Result<(), RenderError> {
        self.check_device()?;
        if self.readback_in_flight() {
            return Err(RenderError::Busy);
        }
        let stamp = self.stamp.ok_or(RenderError::NoSnapshot)?;
        let calibration = self.depth_calibration.ok_or(RenderError::NoDepthSnapshot)?;
        if !self.ready {
            return Err(RenderError::NotReady);
        }
        let entity = self
            .app
            .world()
            .get::<bevy::render::sync_world::RenderEntity>(self.camera)
            .ok_or(RenderError::NotReady)?
            .id();
        let texture = self
            .app
            .get_sub_app(RenderApp)
            .and_then(|render| {
                render
                    .world()
                    .get::<bevy::render::view::ViewDepthStencilTexture>(entity)
            })
            .ok_or(RenderError::NotReady)?
            .texture()
            .clone();
        let world = self.app.world();
        self.depth_pending = Some(depth::request(
            world.resource::<RenderDevice>(),
            world.resource::<RenderQueue>(),
            &texture,
            stamp,
            calibration,
        )?);
        // Extend the same submission fence across the requested depth copy.
        let done = Arc::new(AtomicBool::new(false));
        let callback = done.clone();
        world
            .resource::<RenderQueue>()
            .on_submitted_work_done(move || {
                callback.store(true, Ordering::Release);
            });
        self.in_flight = Some(done);
        Ok(())
    }

    /// Nonblocking delivery; cancelled copies retire without delivering a frame.
    pub fn take_depth_frame(&mut self) -> Result<Option<DepthFrame>, RenderError> {
        self.check_device()?;
        if self.depth_pending.is_none() {
            return Ok(None);
        }
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
        let completed = self
            .depth_pending
            .as_ref()
            .expect("pending was checked")
            .completed_bytes()?;
        let Some(bytes) = completed else {
            return Ok(None);
        };
        self.depth_pending
            .take()
            .expect("pending was checked")
            .finish(bytes)
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
        if self.readback_in_flight() {
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
        self.pending.is_some() || self.depth_pending.is_some()
    }

    /// Suppresses delivery of an in-flight frame. The GPU operation may still complete.
    pub fn cancel_readback(&mut self) {
        if let Some(depth) = &mut self.depth_pending {
            depth.pending.valid = false;
        }
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
