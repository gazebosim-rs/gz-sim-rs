// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_scene::{EntityKind, Name, VisualGeometry, VisualMaterial, VisualProperties, WorldPose};
use serde::Serialize;
use std::io;

pub(super) const MAX_BYTES: usize = 65536;

#[derive(Clone, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub(super) enum Shape {
    Box { size: [f64; 3] },
    Sphere { radius: f64 },
    Mesh { uri: String, scale: [f64; 3] },
}

#[derive(Clone, Serialize)]
pub(super) struct Visual {
    id: u64,
    name: String,
    translation: [f64; 3],
    rotation: [f64; 4],
    shape: Shape,
    color: [f64; 4],
}

#[derive(Clone, Serialize)]
pub(super) struct Scene {
    version: u8,
    kind: &'static str,
    world: String,
    world_id: u64,
    pub epoch: u64,
    pub seconds: u64,
    pub nanoseconds: u32,
    sequence: u64,
    pub paused: bool,
    visuals: Vec<Visual>,
}

impl Scene {
    pub(super) fn world_name(&self) -> &str {
        &self.world
    }
    pub(super) fn world_id(&self) -> u64 {
        self.world_id
    }
    pub(super) fn sequence(&self) -> u64 {
        self.sequence
    }
    pub(super) fn visual(&self, id: u64) -> Option<Visual> {
        self.visuals.iter().find(|visual| visual.id == id).cloned()
    }
    pub(super) fn visual_ids(&self) -> impl Iterator<Item = u64> + '_ {
        self.visuals.iter().map(|visual| visual.id)
    }
    pub(super) fn project_spawn(
        &mut self,
        source: &Visual,
        model: u64,
        pose: WorldPose,
    ) -> io::Result<()> {
        let id = model
            .checked_add(3)
            .ok_or_else(|| invalid("spawn visual identity exhausted"))?;
        let name = format!("{}::runtime_{model}::body::visual", self.world);
        if self.visuals.len() >= 128
            || id > u32::MAX as u64
            || self.visual(id).is_some()
            || name.len() > 256
        {
            return Err(invalid("spawn exceeds the bounded scene"));
        }
        let mut visual = source.clone();
        visual.id = id;
        visual.name = name;
        visual.translation = pose.translation;
        visual.rotation = pose.rotation;
        self.visuals.push(visual);
        self.visuals.sort_unstable_by_key(|visual| visual.id);
        if matches!(source.shape, Shape::Mesh { .. }) {
            self.version = 2;
        }
        self.check_projected()
    }
    pub(super) fn project_state(&mut self, id: u64, pose: WorldPose) -> io::Result<()> {
        let visual = self
            .visuals
            .iter_mut()
            .find(|visual| visual.id == id)
            .ok_or_else(|| invalid("state visual is not in the current scene"))?;
        visual.translation = pose.translation;
        visual.rotation = pose.rotation;
        self.check_projected()
    }
    pub(super) fn project_remove(&mut self, ids: &[u64]) -> io::Result<()> {
        self.visuals.retain(|visual| !ids.contains(&visual.id));
        self.version = if self
            .visuals
            .iter()
            .any(|visual| matches!(visual.shape, Shape::Mesh { .. }))
        {
            2
        } else {
            1
        };
        self.check_projected()
    }
    fn check_projected(&self) -> io::Result<()> {
        if self.visuals.iter().any(|visual| {
            !visual
                .translation
                .into_iter()
                .chain(visual.rotation)
                .all(representable)
                || (visual.rotation.into_iter().map(|v| v * v).sum::<f64>() - 1.0).abs() > 1e-6
        }) {
            return Err(invalid("projected visual pose is not representable"));
        }
        encode(self).map(|_| ())
    }
}

fn invalid(message: &'static str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

fn representable(value: f64) -> bool {
    value.is_finite() && (value as f32).is_finite()
}

pub(super) fn extract(sim: &Simulation, world_name: &str, sequence: u64) -> io::Result<Scene> {
    extract_with_assets(sim, world_name, sequence, None)
}

pub(super) fn extract_with_assets(
    sim: &Simulation,
    world_name: &str,
    sequence: u64,
    assets: Option<&super::assets::Assets>,
) -> io::Result<Scene> {
    if sim.is_faulted() || world_name.len() > 256 {
        return Err(invalid("gzweb snapshot needs a healthy named world"));
    }
    let world = sim.world();
    let mut world_id = None;
    let mut visuals = Vec::new();
    for entity in world.iter_entities() {
        let kind = entity.get::<EntityKind>();
        if kind == Some(&EntityKind::World) {
            if world_id.is_some() {
                return Err(invalid("gzweb MVP supports exactly one world"));
            }
            world_id = Some(
                entity
                    .get::<EntityId>()
                    .ok_or_else(|| invalid("gzweb world has no identity"))?
                    .get(),
            );
        }
        if matches!(kind, Some(EntityKind::Light | EntityKind::LightVisual)) {
            return Err(invalid("gzweb MVP does not lower SDF lights"));
        }
        if kind != Some(&EntityKind::Visual) {
            continue;
        }
        let id = entity
            .get::<EntityId>()
            .filter(|id| entity_by_id(world, id.get()) == Some(entity.id()))
            .ok_or_else(|| invalid("gzweb visual has no owned identity"))?
            .get();
        let name = &entity
            .get::<Name>()
            .ok_or_else(|| invalid("gzweb visual has no name"))?
            .0;
        let pose = entity
            .get::<WorldPose>()
            .ok_or_else(|| invalid("gzweb visual has no world pose"))?;
        let geometry = entity
            .get::<VisualGeometry>()
            .ok_or_else(|| invalid("gzweb visual has no geometry"))?;
        let color = if matches!(geometry.0.as_ref(), sdformat::Geometry::Mesh(_)) {
            if entity.contains::<VisualMaterial>() {
                return Err(invalid(
                    "M2 mesh visuals use embedded glTF materials; omit the SDF material override",
                ));
            }
            // Required wire field for compatibility; the mesh renderer uses its glTF material.
            [1.0; 4]
        } else {
            let material = &entity
                .get::<VisualMaterial>()
                .ok_or_else(|| invalid("gzweb visual has no material"))?
                .0;
            if material.script.is_some()
                || material.pbr.is_some()
                || !material.normal_map.is_empty()
            {
                return Err(invalid(
                    "gzweb MVP supports visible opaque diffuse primitives only",
                ));
            }
            material.diffuse.map(f64::from)
        };
        let properties = &entity
            .get::<VisualProperties>()
            .ok_or_else(|| invalid("gzweb visual has no properties"))?
            .0;
        if properties.transparency != 0.0 || properties.visibility_flags != u32::MAX {
            return Err(invalid(
                "gzweb MVP supports visible opaque diffuse primitives only",
            ));
        }
        let positive = |v: f64| representable(v) && v as f32 > 0.0;
        let shape = match geometry.0.as_ref() {
            sdformat::Geometry::Box { size } if size.iter().copied().all(positive) => {
                Shape::Box { size: *size }
            }
            sdformat::Geometry::Sphere { radius } if positive(*radius) => {
                Shape::Sphere { radius: *radius }
            }
            sdformat::Geometry::Mesh(mesh) if mesh.scale.iter().copied().all(positive) => {
                Shape::Mesh {
                    uri: assets
                        .ok_or_else(|| {
                            invalid("mesh visuals require admitted local glTF resources")
                        })?
                        .uri(id, mesh)?,
                    scale: mesh.scale,
                }
            }
            sdformat::Geometry::Mesh(_) => {
                return Err(invalid(
                    "gzweb mesh scale must be positive, finite and representable",
                ));
            }
            _ => {
                return Err(invalid(
                    "gzweb MVP supports finite box and sphere visuals only",
                ));
            }
        };
        let norm = pose.rotation.iter().map(|v| v * v).sum::<f64>();
        if name.len() > 256
            || !pose
                .translation
                .iter()
                .chain(pose.rotation.iter())
                .copied()
                .all(representable)
            || (norm - 1.0).abs() > 1e-6
            || !color
                .iter()
                .all(|v| v.is_finite() && (0.0..=1.0).contains(v))
            || color[3] != 1.0
        {
            return Err(invalid(
                "gzweb visual has invalid pose, name or opaque color",
            ));
        }
        if visuals.len() == 128 {
            return Err(invalid("gzweb snapshot exceeds 128 visuals"));
        }
        visuals.push(Visual {
            id,
            name: name.clone(),
            translation: pose.translation,
            rotation: pose.rotation,
            shape,
            color,
        });
    }
    visuals.sort_unstable_by_key(|visual| visual.id);
    let version = if visuals
        .iter()
        .any(|visual| matches!(visual.shape, Shape::Mesh { .. }))
    {
        2
    } else {
        1
    };
    Ok(Scene {
        version,
        kind: "scene",
        world: world_name.into(),
        world_id: world_id.ok_or_else(|| invalid("gzweb snapshot has no world identity"))?,
        epoch: sim.epoch(),
        seconds: sim.elapsed().as_secs(),
        nanoseconds: sim.elapsed().subsec_nanos(),
        sequence,
        paused: sim.is_paused(),
        visuals,
    })
}

pub(super) fn encode(scene: &Scene) -> io::Result<String> {
    let json = serde_json::to_string(scene).map_err(io::Error::other)?;
    if json.len() > MAX_BYTES {
        return Err(invalid("gzweb snapshot exceeds 64 KiB"));
    }
    Ok(json)
}
