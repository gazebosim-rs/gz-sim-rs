// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Explicit local world editing. Network input cannot introduce resources or SDF.
use super::{assets, snapshot};
use bevy_ecs::prelude::{ChildOf, Children};
use gz_sim_core::{EntityId, Simulation, next_entity_id};
use gz_sim_scene::{
    EntityKind, ModelState, ModelTemplate, Name, WorldPose, model_state, model_visual_pose,
    prepare_model_template,
};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeSet,
    io,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
        mpsc,
    },
};

fn invalid() -> io::Error {
    io::Error::new(
        io::ErrorKind::InvalidData,
        "lifecycle request is outside the admitted contract",
    )
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Wire {
    version: u8,
    kind: String,
    request_id: u64,
    world_id: u64,
    epoch: u64,
    action: String,
    template_id: Option<u64>,
    model_id: Option<u64>,
    translation: Option<[f64; 3]>,
    rotation: Option<[f64; 4]>,
    linear_velocity: Option<[f64; 3]>,
    angular_velocity: Option<[f64; 3]>,
}

#[cfg(test)]
#[path = "../../tests/private/web_lifecycle_contract.rs"]
mod contract_tests;

pub(super) enum Operation {
    Spawn { template_id: u64, pose: WorldPose },
    Remove(u64),
    GetState(u64),
    SetState(u64, ModelState),
}
pub(super) struct Request {
    pub request_id: u64,
    world_id: u64,
    epoch: u64,
    operation: Operation,
}
impl Request {
    pub(super) fn decode(text: &str, previous: u64) -> io::Result<Self> {
        // Deserialize the original text as well as checking exact variant keys:
        // the typed pass retains duplicate-field rejection that Value alone loses.
        let wire: Wire = serde_json::from_str(text).map_err(|_| invalid())?;
        let value: serde_json::Value = serde_json::from_str(text).map_err(|_| invalid())?;
        let object = value.as_object().ok_or_else(invalid)?;
        if wire.version != 1
            || wire.kind != "lifecycle"
            || wire.request_id <= previous
            || wire.world_id == 0
        {
            return Err(invalid());
        }
        let extra: &[&str] = match wire.action.as_str() {
            "spawn" => &["template_id", "translation", "rotation"],
            "remove" | "get_state" => &["model_id"],
            "set_state" => &[
                "model_id",
                "translation",
                "rotation",
                "linear_velocity",
                "angular_velocity",
            ],
            _ => return Err(invalid()),
        };
        let base = [
            "version",
            "kind",
            "request_id",
            "world_id",
            "epoch",
            "action",
        ];
        if object.len() != base.len() + extra.len()
            || object
                .keys()
                .any(|key| !base.contains(&key.as_str()) && !extra.contains(&key.as_str()))
        {
            return Err(invalid());
        }
        let operation = match wire.action.as_str() {
            "spawn" => Operation::Spawn {
                template_id: wire.template_id.ok_or_else(invalid)?,
                pose: WorldPose {
                    translation: wire.translation.ok_or_else(invalid)?,
                    rotation: wire.rotation.ok_or_else(invalid)?,
                },
            },
            "remove" => Operation::Remove(wire.model_id.ok_or_else(invalid)?),
            "get_state" => Operation::GetState(wire.model_id.ok_or_else(invalid)?),
            "set_state" => Operation::SetState(
                wire.model_id.ok_or_else(invalid)?,
                ModelState {
                    translation: wire.translation.ok_or_else(invalid)?,
                    rotation: wire.rotation.ok_or_else(invalid)?,
                    linear_velocity: wire.linear_velocity.ok_or_else(invalid)?,
                    angular_velocity: wire.angular_velocity.ok_or_else(invalid)?,
                },
            ),
            _ => unreachable!("validated action"),
        };
        if matches!(
            operation,
            Operation::Spawn { template_id: 0, .. }
                | Operation::Remove(0)
                | Operation::GetState(0)
                | Operation::SetState(0, _)
        ) {
            return Err(invalid());
        }
        Ok(Self {
            request_id: wire.request_id,
            world_id: wire.world_id,
            epoch: wire.epoch,
            operation,
        })
    }
    pub(super) fn recorded_world(&self) -> (u64, u64) {
        (self.world_id, self.epoch)
    }
    pub(super) fn recorded_request(&self) -> (&'static str, Option<u64>) {
        let model = match self.operation {
            Operation::Spawn { .. } => None,
            Operation::Remove(id) | Operation::GetState(id) | Operation::SetState(id, _) => {
                Some(id)
            }
        };
        (self.action(), model)
    }
    fn action(&self) -> &'static str {
        match self.operation {
            Operation::Spawn { .. } => "spawn",
            Operation::Remove(_) => "remove",
            Operation::GetState(_) => "get_state",
            Operation::SetState(..) => "set_state",
        }
    }
}

pub(super) struct Pending {
    pub trace: Option<String>,
    pub request: Request,
    pub alive: Arc<AtomicBool>,
    pub reply: mpsc::SyncSender<Committed>,
}

/// An inseparable reply frame. A slow socket must not pair an older ACK with a
/// newer Latest scene, or lose the exact sequence while the server keeps running.
pub(super) struct Committed {
    pub scene: Arc<String>,
    pub scan: Option<Arc<String>>,
    pub catalog: Arc<String>,
    pub ack: String,
}

// Keep the original native literal API. The alias prevents serde derive's
// implicit borrowed-&str lifetime inference; the field adapter owns decoding
// and returns only the validated constant, so no input borrow is retained.
type StaticKind = &'static str;

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct TemplateInfo {
    template_id: u64,
    label: String,
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Model {
    model_id: u64,
    name: String,
    visual_ids: Vec<u64>,
    can_remove: bool,
    can_set_state: bool,
}
#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Catalog {
    version: u8,
    #[serde(deserialize_with = "catalog_kind")]
    kind: StaticKind,
    world_id: u64,
    epoch: u64,
    scene_sequence: u64,
    templates: Vec<TemplateInfo>,
    models: Vec<Model>,
}
fn catalog_kind<'de, D: serde::Deserializer<'de>>(
    deserializer: D,
) -> Result<&'static str, D::Error> {
    let kind = String::deserialize(deserializer)?;
    if kind == "lifecycle_catalog" {
        Ok("lifecycle_catalog")
    } else {
        Err(serde::de::Error::custom(
            "expected recorded lifecycle catalog kind",
        ))
    }
}

impl Catalog {
    pub(super) fn decode(text: &str, scene: &snapshot::Scene) -> io::Result<Self> {
        if text.len() > snapshot::MAX_BYTES {
            return Err(invalid());
        }
        let catalog: Self = serde_json::from_str(text).map_err(|_| invalid())?;
        if catalog.version != 1
            || catalog.kind != "lifecycle_catalog"
            || catalog.world_id != scene.world_id()
            || catalog.epoch != scene.epoch
            || catalog.scene_sequence != scene.sequence()
            || catalog
                .models
                .iter()
                .any(|model| model.model_id > u32::MAX as u64)
        {
            return Err(invalid());
        }
        let mut templates = BTreeSet::new();
        if catalog.templates.iter().any(|template| {
            template.template_id == 0
                || template.template_id > u32::MAX as u64
                || template.label.len() > 256
                || !templates.insert(template.template_id)
        }) {
            return Err(invalid());
        }
        catalog.encode()?;
        let expected: BTreeSet<_> = scene.visual_ids().collect();
        let actual: BTreeSet<_> = catalog
            .models
            .iter()
            .flat_map(|model| model.visual_ids.iter().copied())
            .collect();
        if expected != actual {
            return Err(invalid());
        }
        Ok(catalog)
    }

    pub(super) fn definitions(&self) -> io::Result<Vec<(u64, String)>> {
        self.models
            .iter()
            .map(|model| {
                let ids: BTreeSet<_> = model.visual_ids.iter().copied().collect();
                serde_json::to_string(&("model", &model.name, ids))
                    .map(|value| (model.model_id, value))
                    .map_err(io::Error::other)
            })
            .collect()
    }

    pub(super) fn model_ids(&self) -> impl Iterator<Item = u64> + '_ {
        self.models.iter().map(|model| model.model_id)
    }

    pub(super) fn read_only(&mut self, scene: &snapshot::Scene) {
        self.epoch = scene.epoch;
        self.scene_sequence = scene.sequence();
        self.templates.clear();
        for model in &mut self.models {
            model.can_remove = false;
            model.can_set_state = false;
        }
    }

    pub(super) fn recording_key(&self) -> io::Result<String> {
        let mut catalog = self.clone();
        catalog.scene_sequence = 0;
        catalog.encode()
    }

    pub(super) fn replay_encode(&self) -> io::Result<String> {
        self.encode()
    }
    fn encode(&self) -> io::Result<String> {
        let mut owned = BTreeSet::new();
        let mut models = BTreeSet::new();
        if self.world_id == 0
            || self.world_id > u32::MAX as u64
            || self.models.len() > 128
            || self.templates.len() > 128
        {
            return Err(invalid());
        }
        for model in &self.models {
            if model.model_id == 0
                || model.model_id == self.world_id
                || !models.insert(model.model_id)
                || model.name.len() > 256
                || model.visual_ids.is_empty()
            {
                return Err(invalid());
            }
            for id in &model.visual_ids {
                if *id == 0 || *id > u32::MAX as u64 || !owned.insert(*id) {
                    return Err(invalid());
                }
            }
        }
        if owned.len() > 128
            || owned.contains(&self.world_id)
            || models.iter().any(|id| owned.contains(id))
        {
            return Err(invalid());
        }
        let encoded = serde_json::to_string(self).map_err(|_| invalid())?;
        if encoded.len() > snapshot::MAX_BYTES {
            return Err(invalid());
        }
        Ok(encoded)
    }
}

#[derive(Serialize)]
struct State {
    translation: [f64; 3],
    rotation: [f64; 4],
    linear_velocity: [f64; 3],
    angular_velocity: [f64; 3],
}
impl From<ModelState> for State {
    fn from(state: ModelState) -> Self {
        Self {
            translation: state.translation,
            rotation: state.rotation,
            linear_velocity: state.linear_velocity,
            angular_velocity: state.angular_velocity,
        }
    }
}
#[derive(Serialize)]
struct Ack {
    version: u8,
    kind: &'static str,
    request_id: u64,
    action: &'static str,
    world_id: u64,
    epoch: u64,
    scene_sequence: u64,
    seconds: u64,
    nanoseconds: u32,
    paused: bool,
    ok: bool,
    error: Option<&'static str>,
    model_id: Option<u64>,
    state: Option<State>,
}

struct Template {
    info: TemplateInfo,
    native: ModelTemplate,
    visual: snapshot::Visual,
}

pub(super) struct Lifecycle {
    templates: Vec<Template>,
    pub pending: Vec<Pending>,
    replies: Vec<(Pending, String)>,
}
impl Lifecycle {
    pub(super) fn new(
        sim: &Simulation,
        sdf: &sdformat::Sdf,
        world_name: &str,
        scene: &snapshot::Scene,
    ) -> io::Result<Self> {
        let backend = sim
            .world()
            .get_resource::<gz_physics::PhysicsBackend>()
            .ok_or_else(invalid)?;
        if backend.engine() != "rapier" {
            return Err(invalid());
        }
        let source = sdf
            .worlds
            .iter()
            .find(|world| world.name == world_name)
            .ok_or_else(invalid)?;
        let mut templates = Vec::new();
        for model in &source.models {
            let native = match prepare_model_template(sim, sdf, model) {
                Ok(template) => template,
                Err(gz_sim_scene::WorldEditError::UnsupportedModel) => continue,
                Err(_) => return Err(invalid()),
            };
            if templates.len() == 128 || model.name.len() > 256 {
                return Err(invalid());
            }
            let visual = scene
                .visual(native.source_visual_id())
                .ok_or_else(invalid)?;
            templates.push(Template {
                info: TemplateInfo {
                    template_id: templates.len() as u64 + 1,
                    label: model.name.clone(),
                },
                native,
                visual,
            });
        }
        let result = Self {
            templates,
            pending: Vec::new(),
            replies: Vec::new(),
        };
        result.catalog(sim, scene)?.encode()?;
        Ok(result)
    }

    fn catalog(&self, sim: &Simulation, scene: &snapshot::Scene) -> io::Result<Catalog> {
        let world = sim.world();
        let visible: BTreeSet<_> = scene.visual_ids().collect();
        let mut models = Vec::new();
        for root in world
            .iter_entities()
            .filter(|entity| entity.get::<EntityKind>() == Some(&EntityKind::Model))
        {
            let Some(parent) = root.get::<ChildOf>().map(ChildOf::parent) else {
                continue;
            };
            if world.get::<EntityId>(parent).map(EntityId::get) != Some(scene.world_id()) {
                continue;
            }
            let id = root.get::<EntityId>().ok_or_else(invalid)?.get();
            let mut pending = vec![root.id()];
            let mut visited = BTreeSet::new();
            let mut visuals = Vec::new();
            while let Some(entity) = pending.pop() {
                if !visited.insert(entity) {
                    return Err(invalid());
                }
                if world.get::<EntityKind>(entity) == Some(&EntityKind::Visual) {
                    let id = world.get::<EntityId>(entity).ok_or_else(invalid)?.get();
                    if !visible.contains(&id) {
                        return Err(invalid());
                    }
                    visuals.push(id);
                }
                if let Some(children) = world.get::<Children>(entity) {
                    pending.extend(children.iter());
                }
            }
            if visuals.is_empty() {
                continue;
            }
            visuals.sort_unstable();
            let admitted = model_state(sim, id).is_ok();
            models.push(Model {
                model_id: id,
                name: root.get::<Name>().ok_or_else(invalid)?.0.clone(),
                visual_ids: visuals,
                can_remove: admitted,
                can_set_state: admitted,
            });
        }
        models.sort_unstable_by_key(|model| model.model_id);
        Ok(Catalog {
            version: 1,
            kind: "lifecycle_catalog",
            world_id: scene.world_id(),
            epoch: sim.epoch(),
            scene_sequence: scene.sequence(),
            templates: self
                .templates
                .iter()
                .map(|template| template.info.clone())
                .collect(),
            models,
        })
    }

    pub(super) fn encode_catalog(
        &self,
        sim: &Simulation,
        scene: &snapshot::Scene,
    ) -> io::Result<String> {
        self.catalog(sim, scene)?.encode()
    }
    pub(super) fn recorded_outcomes(&self) -> io::Result<Vec<super::recording::Outcome>> {
        self.replies
            .iter()
            .map(|(pending, ack)| {
                let trace = pending.trace.clone().ok_or_else(|| {
                    io::Error::other("enabled recording lost lifecycle request provenance")
                })?;
                super::recording::Outcome::lifecycle(&pending.request, trace, ack.clone())
            })
            .collect()
    }

    pub(super) fn has_replies(&self) -> bool {
        !self.replies.is_empty()
    }
    pub(super) fn send_replies(
        &mut self,
        scene: Arc<String>,
        scan: Option<Arc<String>>,
        catalog: Arc<String>,
    ) {
        for (pending, ack) in self.replies.drain(..) {
            let _ = pending.reply.try_send(Committed {
                scene: scene.clone(),
                scan: scan.clone(),
                catalog: catalog.clone(),
                ack,
            });
        }
    }

    /// Called after remote reset preparation and the tick. Each pending command
    /// is bound to the next full Scene sequence, including rejected commands.
    pub(super) fn apply(
        &mut self,
        server: &mut gz_sim_server::Server,
        world_name: &str,
        sequence: u64,
        assets: &mut Option<assets::Assets>,
    ) -> io::Result<()> {
        if !self.pending.is_empty() {
            // One active socket owns one pending command. Retain this bound for
            // internal callers too: every ACK describes its own committed frame.
            let pending = self.pending.remove(0);
            if !pending.alive.load(Ordering::Acquire) {
                return Ok(());
            }
            let scene = snapshot::extract_with_assets(
                server.simulation(),
                world_name,
                sequence,
                assets.as_ref(),
            )?;
            if let Some(assets) = assets {
                assets.retain_aliases(&scene.visual_ids().collect());
            }
            let result = self.perform(server, &pending.request, scene.clone(), assets);
            if server.simulation().is_faulted() {
                return Err(io::Error::other(
                    "world lifecycle commit faulted the simulation",
                ));
            }
            let (ok, model_id, state, error) = match result {
                Ok((id, state)) => (true, Some(id), state.map(State::from), None),
                Err(_) => (
                    false,
                    None,
                    None,
                    Some("request refused by the current world or admitted lifecycle bounds"),
                ),
            };
            let sim = server.simulation();
            let ack = Ack {
                version: 1,
                kind: "lifecycle_ack",
                request_id: pending.request.request_id,
                action: pending.request.action(),
                world_id: scene.world_id(),
                epoch: sim.epoch(),
                scene_sequence: sequence,
                seconds: sim.elapsed().as_secs(),
                nanoseconds: sim.elapsed().subsec_nanos(),
                paused: sim.is_paused(),
                ok,
                error,
                model_id,
                state,
            };
            let encoded = serde_json::to_string(&ack).map_err(|_| invalid())?;
            if encoded.len() > snapshot::MAX_BYTES {
                return Err(invalid());
            }
            self.replies.push((pending, encoded));
        }
        Ok(())
    }

    fn perform(
        &self,
        server: &mut gz_sim_server::Server,
        request: &Request,
        mut scene: snapshot::Scene,
        assets: &mut Option<assets::Assets>,
    ) -> io::Result<(u64, Option<ModelState>)> {
        let sim = server.simulation();
        if request.world_id != scene.world_id() || request.epoch != sim.epoch() {
            return Err(invalid());
        }
        let mut catalog = self.catalog(sim, &scene)?;
        match &request.operation {
            Operation::Spawn { template_id, pose } => {
                let template = self
                    .templates
                    .iter()
                    .find(|template| template.info.template_id == *template_id)
                    .ok_or_else(invalid)?;
                let id = next_entity_id(sim.world()).map_err(|_| invalid())?;
                let visual_id = id.checked_add(3).ok_or_else(invalid)?;
                scene.project_spawn(
                    &template.visual,
                    id,
                    template.native.visual_world_pose(*pose),
                )?;
                catalog.models.push(Model {
                    model_id: id,
                    name: format!("{world}::runtime_{id}", world = scene.world_name()),
                    visual_ids: vec![visual_id],
                    can_remove: true,
                    can_set_state: true,
                });
                catalog.encode()?;
                let alias = if let Some(mesh) = template.native.visual_mesh() {
                    Some(assets.as_ref().ok_or_else(invalid)?.prepare_alias(
                        template.native.source_visual_id(),
                        visual_id,
                        mesh,
                    )?)
                } else {
                    None
                };
                let committed = server
                    .spawn_model_template(&template.native, *pose)
                    .map_err(|_| invalid())?;
                assert_eq!(committed, id, "exclusive projected model identity");
                if let Some(alias) = alias {
                    assets
                        .as_mut()
                        .expect("preflighted assets")
                        .commit_alias(alias);
                }
                Ok((committed, None))
            }
            Operation::Remove(id) => {
                let model = catalog
                    .models
                    .iter()
                    .find(|model| model.model_id == *id && model.can_remove)
                    .ok_or_else(invalid)?;
                scene.project_remove(&model.visual_ids)?;
                catalog.models.retain(|model| model.model_id != *id);
                catalog.encode()?;
                server.remove_editable_model(*id).map_err(|_| invalid())?;
                if let Some(assets) = assets {
                    assets.retain_aliases(&scene.visual_ids().collect());
                }
                Ok((*id, None))
            }
            Operation::GetState(id) => {
                catalog.encode()?;
                Ok((*id, Some(model_state(sim, *id).map_err(|_| invalid())?)))
            }
            Operation::SetState(id, state) => {
                let (visual, pose) = model_visual_pose(
                    sim,
                    *id,
                    WorldPose {
                        translation: state.translation,
                        rotation: state.rotation,
                    },
                )
                .map_err(|_| invalid())?;
                scene.project_state(visual, pose)?;
                catalog.encode()?;
                Ok((
                    *id,
                    Some(server.set_model_state(*id, *state).map_err(|_| invalid())?),
                ))
            }
        }
    }
}
