// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Sparse, bounded reconstruction of the current template population.
use super::*;
use gz_physics_distributed::pool::PoolNativeRecovery;
use gz_sim_core::{advance_entity_id_floor, entity_by_id};
use std::collections::BTreeSet;

struct Source {
    name: String,
    native: BodyId,
    template: Arc<ModelTemplate>,
}

/// Original primitive templates in their restored World. Access remains valid
/// when a startup model has been retired from the current scene.
#[derive(Resource, Clone)]
pub struct RestoredTemplates(Arc<BTreeMap<u64, Source>>);
impl RestoredTemplates {
    pub(crate) fn prepare(
        sim: &Simulation,
        xml: &str,
        source: &str,
        world: &str,
    ) -> Result<Self, Error> {
        let parsed =
            sdformat::load_str(xml, source, &sdformat::NoResources).map_err(LoadError::from)?;
        let world_data = parsed
            .worlds
            .iter()
            .find(|w| w.name == world)
            .ok_or(Error::Domain("missing SDF world"))?;
        let mut templates = BTreeMap::new();
        for model in &world_data.models {
            let template = match prepare_model_template(sim, &parsed, model) {
                Ok(template) => template,
                Err(WorldEditError::UnsupportedModel) => continue,
                Err(e) => return Err(e.into()),
            };
            // The edited checkpoint's persistent template library supports visible
            // box/sphere clones and requires no resource lookup after retirement.
            if !matches!(
                model.links[0].visuals[0].geometry,
                Some(Geometry::Box { .. } | Geometry::Sphere { .. })
            ) {
                continue;
            }
            let visual = entity_by_id(sim.world(), template.source_visual_id())
                .ok_or(Error::Domain("missing template visual"))?;
            let link = sim
                .world()
                .get::<ChildOf>(visual)
                .ok_or(Error::Domain("template visual has no parent"))?
                .parent();
            let root = sim
                .world()
                .get::<ChildOf>(link)
                .ok_or(Error::Domain("template link has no parent"))?
                .parent();
            let id = sim
                .world()
                .get::<EntityId>(root)
                .ok_or(Error::Domain("missing template identity"))?
                .get();
            let native = sim
                .world()
                .get::<BodyHandle>(link)
                .ok_or(Error::Domain("missing template body"))?
                .id();
            templates.insert(
                id,
                Source {
                    name: model.scoped_name.clone(),
                    native,
                    template: Arc::new(template),
                },
            );
        }
        Ok(Self(Arc::new(templates)))
    }

    pub fn by_name(&self, scoped_name: &str) -> Option<Arc<ModelTemplate>> {
        self.0
            .values()
            .find(|s| s.name == scoped_name)
            .map(|s| Arc::clone(&s.template))
    }
    fn source(&self, id: u64) -> Result<&Source, Error> {
        self.0.get(&id).ok_or(Error::Domain(
            "model is outside the original box/sphere template library",
        ))
    }
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct CloneRecipe {
    model: u64,
    source_model: u64,
    source_visual: u64,
    birth_tick: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    newborn_public_sha256: Option<[u8; 32]>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    replaced_at_birth: bool,
    translation: [f64; 3],
    rotation: [f64; 4],
}
impl CloneRecipe {
    fn pose(&self) -> WorldPose {
        WorldPose {
            translation: self.translation,
            rotation: self.rotation,
        }
    }
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Population {
    next_native: u64,
    epoch_start_native: u64,
    retired: Vec<u64>,
    clones: Vec<CloneRecipe>,
}

#[derive(Default)]
pub(super) struct Changes {
    retired: BTreeSet<u64>,
    clones: BTreeMap<u64, CloneRecipe>,
}

pub(super) struct Newborn {
    entity: Entity,
    world: WorldPose,
    local: LocalPose,
    velocity: Option<Velocity>,
}
impl Newborn {
    pub(super) fn publish(self, sim: &mut Simulation) {
        let mut e = sim.world_mut().entity_mut(self.entity);
        e.insert((self.world, self.local));
        if let Some(v) = self.velocity {
            e.insert(v);
        }
    }
}

impl PassivePoolScene {
    /// Clone an original editable model's immutable template into a separated pose.
    /// The template remains available after retiring its original live instance.
    pub fn spawn(&mut self, source_model: u64, pose: WorldPose) -> Result<u64, Error> {
        let source = self.state.templates.source(source_model)?;
        let model = spawn_model_template(&mut self.sim, &source.template, pose)?;
        self.state.note_birth(
            model,
            source_model,
            self.sim.world().resource::<UpdateInfo>().iteration,
            pose,
        );
        Ok(model)
    }

    /// Retire a loaded template model or one of this owner's active clones.
    pub fn retire(&mut self, model: u64) -> Result<Vec<u64>, Error> {
        if !self.state.changes.clones.contains_key(&model) {
            self.state.templates.source(model)?;
        }
        let retired = remove_model(&mut self.sim, model)?;
        self.state.note_retirement(model);
        Ok(retired)
    }
}

impl CheckpointState {
    pub(super) fn note_birth(&mut self, model: u64, source_model: u64, tick: u64, pose: WorldPose) {
        let source = self.templates.0.get(&source_model).expect("known template");
        self.changes.clones.insert(
            model,
            CloneRecipe {
                model,
                source_model,
                source_visual: source.template.source_visual_id(),
                birth_tick: tick,
                newborn_public_sha256: None,
                replaced_at_birth: false,
                translation: pose.translation,
                rotation: pose.rotation,
            },
        );
    }
    pub(super) fn note_retirement(&mut self, model: u64) {
        if self.changes.clones.remove(&model).is_none()
            && self
                .bindings
                .nodes
                .iter()
                .any(|node| node.kind == Kind::Model && node.id == model)
        {
            self.changes.retired.insert(model);
        }
    }
    pub(super) fn note_replacement(&mut self, model: u64, tick: u64) {
        if let Some(recipe) = self.changes.clones.get_mut(&model)
            && recipe.birth_tick == tick
        {
            recipe.replaced_at_birth = true;
        }
    }
    pub(super) fn source_for_visual(&self, visual: u64) -> Option<u64> {
        self.templates
            .0
            .iter()
            .find(|(_, s)| s.template.source_visual_id() == visual)
            .map(|(id, _)| *id)
    }

    pub(super) fn clone_metadata_matches(&self, world: &World) -> bool {
        self.changes.clones.values().all(|c| {
            self.templates
                .0
                .get(&c.source_model)
                .is_some_and(|source| source.template.passive_metadata_matches(world, c.model))
        })
    }

    fn newborn_receipt(sim: &Simulation, root: u64) -> Result<[u8; 32], Error> {
        let world = sim.world();
        let mut hash = Sha256::new();
        for public in root..=root + 3 {
            let e = entity_by_id(world, public)
                .ok_or(Error::Domain("newborn public identity missing"))?;
            let e = world.entity(e);
            let pose = *e
                .get::<WorldPose>()
                .ok_or(Error::Domain("missing newborn world pose"))?;
            let local = e
                .get::<LocalPose>()
                .ok_or(Error::Domain("missing newborn local pose"))?
                .0;
            hash_public_motion(&mut hash, public, pose, local, e.get::<Velocity>().copied());
        }
        Ok(hash.finalize().into())
    }

    pub(super) fn check_newborn_receipts(&self, sim: &Simulation, tick: u64) -> Result<(), Error> {
        for c in self
            .changes
            .clones
            .values()
            .filter(|c| c.birth_tick == tick)
        {
            if let Some(expected) = c.newborn_public_sha256
                && Self::newborn_receipt(sim, c.model)? != expected
            {
                return Err(Error::Domain(
                    "newborn public transforms differ from its receipt",
                ));
            }
        }
        Ok(())
    }

    pub(super) fn manifest_population(
        &self,
        sim: &Simulation,
    ) -> Result<Option<Population>, Error> {
        let next_native = sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<SceneBackend>()
            .expect("private pool backend")
            .next_body_handle();
        if next_native == self.initial_next_native
            && self.changes.retired.is_empty()
            && self.changes.clones.is_empty()
        {
            return Ok(None);
        }
        let tick = sim.world().resource::<UpdateInfo>().iteration;
        let mut clones = Vec::with_capacity(self.changes.clones.len());
        for c in self.changes.clones.values() {
            let mut c = c.clone();
            c.newborn_public_sha256 = if c.birth_tick == tick {
                Some(Self::newborn_receipt(sim, c.model)?)
            } else {
                None
            };
            clones.push(c);
        }
        Ok(Some(Population {
            next_native,
            epoch_start_native: self.epoch_start_native,
            retired: self.changes.retired.iter().copied().collect(),
            clones,
        }))
    }

    /// Account for every live public node before exposing a durable barrier.
    pub(super) fn check_membership(
        &self,
        current: &Bindings,
        population: &Population,
    ) -> Result<(), Error> {
        let mut removed: BTreeSet<_> = population.retired.iter().copied().collect();
        let mut expected = Vec::with_capacity(current.nodes.len());
        for node in &self.bindings.nodes {
            if node.parent.is_some_and(|parent| removed.contains(&parent)) {
                removed.insert(node.id);
            }
            if !removed.contains(&node.id) {
                expected.push(node.clone());
            }
        }
        let world = self
            .bindings
            .nodes
            .iter()
            .find(|n| n.kind == Kind::World)
            .ok_or(Error::Domain("missing original world"))?;
        for c in &population.clones {
            let base = format!("{}::runtime_{}", world.name, c.model);
            let native = self.initial_next_native + (c.model - self.bindings.next_public_id) / 4;
            for (offset, kind, parent, suffix, body) in [
                (0, Kind::Model, world.id, "", None),
                (1, Kind::Link, c.model, "::body", Some(native)),
                (2, Kind::Collision, c.model + 1, "::body::collision", None),
                (3, Kind::Visual, c.model + 1, "::body::visual", None),
            ] {
                let name = format!("{base}{suffix}");
                let local_name = name
                    .rsplit("::")
                    .next()
                    .expect("nonempty clone name")
                    .to_owned();
                expected.push(Node {
                    id: c.model + offset,
                    kind,
                    name,
                    local_name,
                    parent: Some(parent),
                    body,
                });
            }
        }
        if expected != current.nodes {
            return Err(Error::Domain(
                "live public graph differs from the tracked births and retirements",
            ));
        }
        Ok(())
    }

    /// Verify all sparse ID arithmetic before starting any native worker.
    pub(super) fn check_population(
        &self,
        sim: &Simulation,
        p: &Population,
        next_public: u64,
        tick: u64,
    ) -> Result<(), Error> {
        let seed_public = self.bindings.next_public_id;
        let births = p
            .next_native
            .checked_sub(self.initial_next_native)
            .and_then(|n| n.checked_mul(4))
            .and_then(|n| n.checked_add(seed_public));
        if !(self.initial_next_native..=p.next_native).contains(&p.epoch_start_native)
            || (sim.epoch() == 0 && p.epoch_start_native != self.initial_next_native)
            || births != Some(next_public)
            || next_public > u64::from(u32::MAX) + 1
            || p.retired.windows(2).any(|x| x[0] >= x[1])
            || p.clones.windows(2).any(|x| x[0].model >= x[1].model)
            || p.retired.len() > self.templates.0.len()
            || p.clones.len()
                > usize::from(self.domain.workers)
                    * gz_physics_distributed::physics::MAX_PARTITION_BODIES
            || (p.retired.is_empty()
                && p.clones.is_empty()
                && p.next_native == self.initial_next_native)
        {
            return Err(Error::Domain(
                "invalid edited population allocator or canonical order",
            ));
        }
        for &id in &p.retired {
            self.templates.source(id)?;
        }
        for c in &p.clones {
            let source = self.templates.source(c.source_model)?;
            if c.model < seed_public
                || c.model >= next_public
                || !(c.model - seed_public).is_multiple_of(4)
                || self.initial_next_native + (c.model - seed_public) / 4 < p.epoch_start_native
                || (c.newborn_public_sha256.is_some() && c.birth_tick != tick)
                || c.birth_tick > tick
                || c.source_visual != source.template.source_visual_id()
                || !world_edit::representable(c.pose())
            {
                return Err(Error::Domain(
                    "invalid runtime model identity, source, pose or birth clock",
                ));
            }
        }
        Ok(())
    }
}

impl PassivePoolScene {
    pub(super) fn stage_population(
        &mut self,
        p: Population,
        next_public: u64,
        tick: u64,
        native: &PoolNativeRecovery,
    ) -> Result<Vec<Newborn>, Error> {
        self.state
            .check_population(&self.sim, &p, next_public, tick)?;
        let expected_high = if p.next_native > p.epoch_start_native {
            p.next_native.checked_sub(1)
        } else if !p.retired.is_empty() {
            self.state.initial_next_native.checked_sub(1)
        } else {
            None
        };
        if native.population_high_water() != expected_high {
            return Err(Error::Domain(
                "native allocator differs from current epoch birth/retirement history",
            ));
        }
        self.state.epoch_start_native = p.epoch_start_native;
        let mut newborn = Vec::new();
        for c in p.clones {
            let source = self.state.templates.source(c.source_model)?;
            let native_id =
                self.state.initial_next_native + (c.model - self.state.bindings.next_public_id) / 4;
            advance_entity_id_floor(self.sim.world_mut(), c.model)?;
            let id = world_spawn::restore_template_nodes(
                &mut self.sim,
                &source.template,
                c.pose(),
                source.native,
                BodyId::from_raw(native_id),
            )?;
            if id != c.model {
                return Err(Error::Domain("restored clone ID differs"));
            }
            if c.birth_tick == tick && !c.replaced_at_birth {
                let (_, body) = native
                    .frame()
                    .body(native_id)
                    .ok_or(Error::Domain("newborn missing native body"))?;
                if body.position.map(f64::to_bits) != c.translation.map(f64::to_bits)
                    || body.orientation.map(f64::to_bits) != c.rotation.map(f64::to_bits)
                    || body.linear != [0.; 3]
                    || body.angular != [0.; 3]
                {
                    return Err(Error::Domain(
                        "newborn native state differs from its birth receipt",
                    ));
                }
                // A paused tick can publish an existing clone without advancing the
                // clock; a later clone can be born at the same numerical tick. Retain
                // authored poses only for nodes whose individual receipt matches it.
                let authored = CheckpointState::newborn_receipt(&self.sim, c.model)?;
                if c.newborn_public_sha256
                    .is_none_or(|target| target == authored)
                {
                    for public in c.model..=c.model + 3 {
                        let e = entity_by_id(self.sim.world(), public)
                            .ok_or(Error::Domain("newborn public identity missing"))?;
                        let entity = self.sim.world().entity(e);
                        newborn.push(Newborn {
                            entity: e,
                            world: *entity.get::<WorldPose>().expect("published clone pose"),
                            local: *entity
                                .get::<LocalPose>()
                                .expect("published clone local pose"),
                            velocity: entity.get::<Velocity>().copied(),
                        });
                    }
                }
            }
            self.state.changes.clones.insert(c.model, c);
        }
        // Clone definitions reference the original SDF, including sources retired
        // in this epoch. Resolve all of them before removing startup model nodes.
        for id in p.retired {
            remove_model(&mut self.sim, id)?;
            self.state.changes.retired.insert(id);
        }
        advance_entity_id_floor(self.sim.world_mut(), next_public)?;
        backend_mut(&mut self.sim).advance_staged_body_floor(p.next_native)?;
        Ok(newborn)
    }
}
