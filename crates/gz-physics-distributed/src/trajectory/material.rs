// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Complete rigid-plus-one-way-material graphs solved by one leased process.
mod batch;
use super::*;
pub(crate) use batch::Executor;
use bincode::Options;
use glam::{DQuat, DVec3};
use gz_physics::{BodyId, BodyState, DynamicBackend};
use gz_physics_xpbd::{
    Boundary, BoundaryPath, DEFAULT_PATH_TOLERANCE, OneWayConstruction, OneWayState, SoftBody,
    TranslationBoundary,
};

#[derive(Clone, Copy, Serialize, Deserialize)]
pub enum Surface {
    Plane {
        point: [f64; 3],
        normal: [f64; 3],
    },
    Sphere {
        body: u64,
        radius: f64,
    },
    Capsule {
        body: u64,
        radius: f64,
        half_length: f64,
    },
}
impl Surface {
    fn native(self) -> Option<BodyId> {
        match self {
            Self::Plane { .. } => None,
            Self::Sphere { body, .. } | Self::Capsule { body, .. } => Some(BodyId::from_raw(body)),
        }
    }
}
#[derive(Serialize, Deserialize)]
pub struct MaterialRecipe {
    surfaces: Vec<Surface>,
    materials: Vec<(String, OneWayConstruction)>,
}
impl MaterialRecipe {
    pub fn new(
        surfaces: Vec<Surface>,
        bodies: impl IntoIterator<Item = (String, OneWayConstruction)>,
    ) -> Result<Self> {
        let materials: Vec<_> = bodies.into_iter().collect();
        let r = Self {
            surfaces,
            materials,
        };
        r.validate()?;
        Ok(r)
    }
    pub(super) fn surface_count(&self) -> usize {
        self.surfaces.len()
    }
    /// Assign the most expensive immutable material recipes first and keep original name
    /// ordering within each group. One cable or cloth remains indivisible.
    pub(super) fn partition(self, count: usize) -> Result<Vec<(Vec<usize>, Self)>> {
        self.validate()?;
        if !(2..=self.materials.len()).contains(&count) {
            return Err(invalid(
                "worker_processes exceeds the number of independent materials",
            ));
        }
        let weights = self
            .materials
            .iter()
            .map(|(_, seed)| Ok(batch::weight(&SoftBody::from_one_way_construction(seed)?)))
            .collect::<Result<Vec<_>>>()?;
        let (_, assignment) = batch::distribute(&weights, count);
        let mut bins: Vec<Vec<_>> = (0..count).map(|_| Vec::new()).collect();
        for (i, (group, (name, seed))) in assignment.into_iter().zip(self.materials).enumerate() {
            bins[group].push((i, (name, seed)));
        }
        Ok(bins
            .into_iter()
            .map(|rows| {
                let (indices, materials) = rows.into_iter().unzip();
                (
                    indices,
                    Self {
                        surfaces: self.surfaces.clone(),
                        materials,
                    },
                )
            })
            .collect())
    }
    pub(super) fn identities(&self) -> Result<Vec<Hash>> {
        self.materials
            .iter()
            .map(|(_, seed)| Ok(SoftBody::from_one_way_construction(seed)?.one_way_identity()?))
            .collect()
    }
    fn validate(&self) -> Result<()> {
        if self.surfaces.is_empty()
            || self.surfaces.len() > 64
            || self.materials.is_empty()
            || self.materials.len() > 32
            || self.materials.windows(2).any(|w| w[0].0 >= w[1].0)
        {
            return Err(invalid("invalid prescribed material population"));
        }
        Ok(())
    }
}
fn codec() -> impl Options {
    bincode::DefaultOptions::new()
        .with_fixint_encoding()
        .reject_trailing_bytes()
        .with_limit(MAX as u64)
}
fn encode<T: Serialize>(x: &T) -> Result<Vec<u8>> {
    let v = codec()
        .serialize(x)
        .map_err(|_| invalid("cannot encode coupled material cut"))?;
    if v.len() > MAX {
        return Err(invalid("material graph exceeds the managed state limit"));
    }
    Ok(v)
}
fn decode<T: serde::de::DeserializeOwned>(s: &[u8]) -> Result<T> {
    codec()
        .deserialize(s)
        .map_err(|_| invalid("invalid coupled material image"))
}
#[derive(Serialize, Deserialize)]
struct Construction {
    #[serde(with = "serde_bytes")]
    native: Vec<u8>,
    soft: MaterialRecipe,
}
pub(super) fn recipe(native: Vec<u8>, soft: MaterialRecipe) -> Result<Vec<u8>> {
    encode(&Construction { native, soft })
}
#[derive(Serialize, Deserialize)]
struct Cut {
    #[serde(with = "serde_bytes")]
    native: Vec<u8>,
    soft: Vec<OneWayState>,
}
/// Encode a durable complete native/material cut using the same bounded representation
/// as leased worker transfer. The enclosing manifest identifies the ordered materials.
pub fn pack_scene_state(native: Vec<u8>, soft: Vec<OneWayState>) -> Result<Vec<u8>> {
    if soft.is_empty() || soft.len() > 32 {
        return Err(invalid("invalid material sequence in scene state"));
    }
    encode(&Cut { native, soft })
}
/// Decode into private values for validation against a separately reconstructed SDF.
pub fn unpack_scene_state(image: &[u8]) -> Result<(Vec<u8>, Vec<OneWayState>)> {
    let cut: Cut = decode(image)?;
    if cut.soft.is_empty() || cut.soft.len() > 32 {
        return Err(invalid("invalid material sequence in scene image"));
    }
    Ok((cut.native, cut.soft))
}
pub(super) fn capture<'a>(
    native: &MultiFidelityBackend,
    bodies: impl IntoIterator<Item = &'a SoftBody>,
) -> Result<Vec<u8>> {
    let soft = bodies
        .into_iter()
        .map(SoftBody::one_way_state)
        .collect::<std::result::Result<_, _>>()?;
    encode(&Cut {
        native: native.checkpoint_trajectory()?,
        soft,
    })
}
pub(super) fn install(
    native: &mut MultiFidelityBackend,
    materials: &mut [&mut SoftBody],
    image: &[u8],
    surfaces: usize,
    allow_cold: bool,
) -> Result<()> {
    let c: Cut = decode(image)?;
    if c.soft.len() != materials.len() {
        return Err(invalid("coupled material count changed"));
    }
    for (b, state) in materials.iter().zip(&c.soft) {
        if state.reactions.len() != surfaces
            && !(allow_cold && state.reactions.is_empty() && state.path_substeps == 0)
        {
            return Err(invalid(
                "material reaction count differs from its construction",
            ));
        }
        b.validate_one_way_state(state)?;
    }
    native.install_trajectory(&c.native)?;
    for (b, state) in materials.iter_mut().zip(c.soft) {
        b.install_one_way_state(state)
            .expect("validated without mutating the material");
    }
    Ok(())
}

pub(super) struct World {
    native: MultiFidelityBackend,
    shapes: Vec<Surface>,
    soft: Vec<SoftBody>,
    batch: batch::Batch,
}
impl World {
    pub fn new(bytes: &[u8], threads: usize, executor: Executor) -> Result<Self> {
        let c: Construction = decode(bytes)?;
        c.soft.validate()?;
        let native = MultiFidelityBackend::from_trajectory_construction(&c.native)?;
        let soft: Vec<SoftBody> = c
            .soft
            .materials
            .iter()
            .map(|(_, s)| SoftBody::from_one_way_construction(s))
            .collect::<std::result::Result<_, _>>()?;
        if c.soft
            .surfaces
            .iter()
            .filter_map(|s| s.native())
            .any(|id| native.state(id).is_none())
        {
            return Err(invalid("unknown material boundary body"));
        }
        let batch = batch::Batch::new(&soft, threads, executor);
        Ok(Self {
            native,
            shapes: c.soft.surfaces,
            soft,
            batch,
        })
    }
    pub fn threads(&self) -> usize {
        self.batch.lanes()
    }
    pub fn snapshot(&self) -> Result<Vec<u8>> {
        capture(&self.native, &self.soft)
    }
    pub fn restore(&mut self, image: &[u8]) -> Result<()> {
        install(
            &mut self.native,
            &mut self.soft.iter_mut().collect::<Vec<_>>(),
            image,
            self.shapes.len(),
            true,
        )
    }
    pub fn step(&mut self, dt: Duration) -> Result<()> {
        let states: Vec<_> = self
            .shapes
            .iter()
            .map(|s| s.native().and_then(|id| self.native.state(id)))
            .collect();
        self.native.step(dt)?;
        let mut bounds = Vec::with_capacity(self.shapes.len());
        let mut paths = Vec::new();
        for (i, (&shape, start)) in self.shapes.iter().zip(states).enumerate() {
            if let Surface::Plane { point, normal } = shape {
                bounds.push(Boundary::Plane {
                    point,
                    normal,
                    velocity: [0.; 3],
                });
                continue;
            }
            let id = shape.native().unwrap();
            let start = start.ok_or(invalid("missing initial material boundary"))?;
            let end = self
                .native
                .state(id)
                .ok_or(invalid("missing final material boundary"))?;
            let center = start.position;
            let velocity =
                ((DVec3::from(end.position) - DVec3::from(center)) / dt.as_secs_f64()).to_array();
            let b = match shape {
                Surface::Sphere { radius, .. } => Boundary::Sphere {
                    radius,
                    center,
                    velocity,
                    angular_velocity: end.angular_velocity,
                },
                Surface::Capsule {
                    radius,
                    half_length,
                    ..
                } => {
                    let axis = capsule_axis(start)?;
                    let final_axis = capsule_axis(end)?;
                    if axis.distance(final_axis) > 1e-12 {
                        return Err(invalid("unsupported one-way capsule axis change"));
                    }
                    Boundary::Capsule {
                        radius,
                        half_length,
                        center,
                        velocity,
                        axis: axis.to_array(),
                        angular_speed: axis.dot(DVec3::from(end.angular_velocity)),
                    }
                }
                Surface::Plane { .. } => unreachable!(),
            };
            if let Some((curve, origin, clock)) = self.native.translation_path(id) {
                paths.push((
                    i,
                    TranslationBoundary::from_publication(curve, origin, clock, dt)
                        .ok_or(invalid("material and native clocks differ"))?,
                ));
            }
            bounds.push(b);
        }
        self.batch.step(&mut self.soft, dt, &bounds, &paths)
    }
}
fn capsule_axis(state: BodyState) -> Result<DVec3> {
    let axis = DQuat::from_array(state.orientation) * DVec3::Z;
    let spin = DVec3::from(state.angular_velocity);
    if !axis.is_finite()
        || !spin.is_finite()
        || axis.cross(spin).length() > 1e-12 * (1. + spin.length())
    {
        return Err(invalid("capsule requires fixed-axis spin"));
    }
    Ok(axis)
}
