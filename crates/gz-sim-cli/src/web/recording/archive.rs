// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Complete admission before presentation; all admitted bytes remain immutable.
use super::{
    clock::{Clock, Stamp},
    lines::{self, Limits, Lines},
};
use crate::web::{lifecycle::Catalog, snapshot::Scene};
use bytes::Bytes;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::File,
    io::{self, Cursor, Read},
    ops::Range,
    path::Path,
    sync::Arc,
    time::Instant,
};

pub(super) const ASSET_FILE_BYTES: usize = 16 * 1024 * 1024;
pub(super) const ASSET_TOTAL_BYTES: usize = 64 * 1024 * 1024;

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Asset {
    pub route: String,
    pub bytes: usize,
    pub sha256: [u8; 32],
}

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Manifest {
    pub version: u8,
    pub complete: bool,
    pub frames: u32,
    pub stream_bytes: u64,
    pub stream_sha256: [u8; 32],
    pub asset_prefix: String,
    pub assets: Vec<Asset>,
}

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub(in crate::web) enum Action {
    Pause,
    Resume,
    Reset,
    Spawn,
    Remove,
    GetState,
    SetState,
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(in crate::web) struct Outcome {
    pub action: Action,
    #[serde(deserialize_with = "required_nullable")]
    pub model_id: Option<u64>,
    pub ok: bool,
    pub request: String,
    pub result: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct WorldReply {
    version: u8,
    kind: String,
    request_id: u64,
    ok: bool,
    epoch: u64,
    seconds: u64,
    nanoseconds: u32,
    paused: bool,
    #[serde(deserialize_with = "required_nullable")]
    error: Option<String>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct State {
    translation: [f64; 3],
    rotation: [f64; 4],
    linear_velocity: [f64; 3],
    angular_velocity: [f64; 3],
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct LifeReply {
    version: u8,
    kind: String,
    request_id: u64,
    action: String,
    world_id: u64,
    epoch: u64,
    scene_sequence: u64,
    seconds: u64,
    nanoseconds: u32,
    paused: bool,
    ok: bool,
    #[serde(deserialize_with = "required_nullable")]
    error: Option<String>,
    #[serde(deserialize_with = "required_nullable")]
    model_id: Option<u64>,
    #[serde(deserialize_with = "required_nullable")]
    state: Option<State>,
}
fn required_nullable<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: serde::Deserializer<'de>,
    T: Deserialize<'de>,
{
    Option::<T>::deserialize(deserializer)
}

impl Outcome {
    pub(in crate::web) fn lifecycle(
        request: &crate::web::lifecycle::Request,
        trace: String,
        result: String,
    ) -> io::Result<Self> {
        let reply: LifeReply = serde_json::from_str(&result).map_err(io::Error::other)?;
        let (action, requested) = request.recorded_request();
        let action = match action {
            "spawn" => Action::Spawn,
            "remove" => Action::Remove,
            "get_state" => Action::GetState,
            "set_state" => Action::SetState,
            _ => unreachable!(),
        };
        Ok(Self {
            action,
            model_id: if reply.ok { reply.model_id } else { requested },
            ok: reply.ok,
            request: trace,
            result,
        })
    }
    fn validate(&self, scene: &Scene) -> io::Result<()> {
        if matches!(self.action, Action::Pause | Action::Resume | Action::Reset) {
            let request = crate::web::Control::decode(&self.request, 0)?;
            let action = match request.action {
                crate::web::Action::Pause => Action::Pause,
                crate::web::Action::Resume => Action::Resume,
                crate::web::Action::Reset => Action::Reset,
            };
            let reply: WorldReply = serde_json::from_str(&self.result).map_err(io::Error::other)?;
            if action != self.action
                || reply.version != 1
                || reply.kind != "ack"
                || request.request_id != reply.request_id
                || reply.ok != self.ok
                || reply.epoch != scene.epoch
                || reply.seconds != scene.seconds
                || reply.nanoseconds != scene.nanoseconds
                || reply.paused != scene.paused
                || reply.ok == reply.error.is_some()
                || (reply.ok
                    && match action {
                        Action::Pause => !scene.paused,
                        Action::Resume => scene.paused,
                        Action::Reset => {
                            !scene.paused
                                || scene.epoch == 0
                                || scene.seconds != 0
                                || scene.nanoseconds != 0
                        }
                        _ => unreachable!(),
                    })
            {
                return Err(invalid(
                    "recorded world outcome disagrees with committed provenance",
                ));
            }
        } else {
            let request = crate::web::lifecycle::Request::decode(&self.request, 0)?;
            let (action, requested_id) = request.recorded_request();
            let expected = match self.action {
                Action::Spawn => "spawn",
                Action::Remove => "remove",
                Action::GetState => "get_state",
                Action::SetState => "set_state",
                _ => unreachable!(),
            };
            let reply: LifeReply = serde_json::from_str(&self.result).map_err(io::Error::other)?;
            if action != expected
                || reply.action != expected
                || reply.version != 1
                || reply.kind != "lifecycle_ack"
                || request.request_id != reply.request_id
                || reply.ok != self.ok
                || reply.world_id != scene.world_id()
                || reply.epoch != scene.epoch
                || reply.scene_sequence != scene.sequence()
                || reply.seconds != scene.seconds
                || reply.nanoseconds != scene.nanoseconds
                || reply.paused != scene.paused
                || reply.ok == reply.error.is_some()
                || (self.ok && request.recorded_world() != (scene.world_id(), scene.epoch))
                || (self.ok && reply.model_id != self.model_id)
                || (!self.ok
                    && (self.model_id != requested_id
                        || reply.model_id.is_some()
                        || reply.state.is_some()))
                || (self.ok && expected != "spawn" && reply.model_id != requested_id)
                || (self.ok
                    && matches!(expected, "get_state" | "set_state")
                    && reply.state.is_none())
                || (!matches!(expected, "get_state" | "set_state") && reply.state.is_some())
            {
                return Err(invalid(
                    "recorded lifecycle outcome disagrees with committed provenance",
                ));
            }
            if let Some(state) = reply.state
                && (!state
                    .translation
                    .into_iter()
                    .chain(state.rotation)
                    .all(|v| v.is_finite() && (v as f32).is_finite())
                    || !state
                        .linear_velocity
                        .into_iter()
                        .chain(state.angular_velocity)
                        .all(f64::is_finite)
                    || (state.rotation.into_iter().map(|v| v * v).sum::<f64>() - 1.0).abs() > 1e-12)
            {
                return Err(invalid("invalid recorded state response"));
            }
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub(super) enum Record {
    Header {
        version: u8,
    },
    Frame {
        ordinal: u32,
        scene: String,
        catalog: String,
        outcomes: Vec<Outcome>,
    },
    Footer {
        frames: u32,
    },
}

pub(super) struct Frame {
    pub scene: Scene,
    pub catalog: Catalog,
    pub outcomes: Vec<Outcome>,
}

pub(super) struct Archive {
    stream: Arc<[u8]>,
    index: Vec<Range<usize>>,
    pub stamps: Vec<Stamp>,
    pub asset_prefix: String,
    pub assets: BTreeMap<String, Bytes>,
}

fn invalid(message: &'static str) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidData, message)
}

pub(super) fn digest(bytes: &[u8]) -> [u8; 32] {
    Sha256::digest(bytes).into()
}

pub(super) fn asset_route(route: &str) -> bool {
    let Some(rest) = route.strip_prefix("/assets/") else {
        return false;
    };
    let Some((scope, name)) = rest.split_once('/') else {
        return false;
    };
    !scope.is_empty()
        && scope.bytes().all(|b| b.is_ascii_digit())
        && scope.parse::<u8>().is_ok_and(|scope| scope < 128)
        && !name.is_empty()
        && name.split('/').all(|part| {
            !part.is_empty()
                && part != "."
                && part != ".."
                && part
                    .bytes()
                    .all(|b| b.is_ascii_alphanumeric() || b"._-".contains(&b))
        })
}

fn file(root: &Path, name: &str, limit: u64) -> io::Result<Vec<u8>> {
    let path = root.join(name).canonicalize()?;
    if !path.starts_with(root) || !path.is_file() {
        return Err(invalid("recording file escapes archive or is not regular"));
    }
    let mut bytes = Vec::with_capacity(limit as usize + 1);
    File::open(path)?.take(limit + 1).read_to_end(&mut bytes)?;
    if bytes.len() as u64 > limit {
        return Err(invalid("recording file exceeds bound"));
    }
    Ok(bytes)
}

impl Archive {
    pub fn open(path: &Path) -> io::Result<Self> {
        let root = path.canonicalize()?;
        if !root.is_dir() {
            return Err(invalid("recording needs a local directory"));
        }
        let manifest: Manifest =
            serde_json::from_slice(&file(&root, "manifest.json", lines::LINE_BYTES as u64)?)
                .map_err(io::Error::other)?;
        if manifest.version != 1
            || !manifest.complete
            || !(1..=4096).contains(&manifest.frames)
            || manifest.stream_bytes > lines::FILE_BYTES
            || manifest.assets.len() > 128
        {
            return Err(invalid(
                "incomplete, unsupported or oversized recording manifest",
            ));
        }
        let prefix = url::Url::parse(&manifest.asset_prefix).map_err(io::Error::other)?;
        if prefix.scheme() != "http"
            || prefix.host_str() != Some("127.0.0.1")
            || prefix.port().is_none()
            || prefix.port() == Some(0)
            || prefix.path() != "/"
            || prefix.query().is_some()
            || prefix.fragment().is_some()
            || !prefix.username().is_empty()
            || prefix.password().is_some()
            || manifest.asset_prefix
                != format!("http://127.0.0.1:{}", prefix.port().expect("checked port"))
        {
            return Err(invalid("invalid recorded local asset prefix"));
        }
        let mut assets = BTreeMap::new();
        let mut total = 0usize;
        for asset in &manifest.assets {
            if !asset_route(&asset.route)
                || asset.bytes > ASSET_FILE_BYTES
                || assets.contains_key(&asset.route)
                || asset.bytes > ASSET_TOTAL_BYTES.saturating_sub(total)
            {
                return Err(invalid("recorded assets exceed admitted paths or bounds"));
            }
            let bytes = file(&root, &asset.route[1..], asset.bytes as u64)?;
            if bytes.len() != asset.bytes || digest(&bytes) != asset.sha256 {
                return Err(invalid("captured asset changed"));
            }
            total += bytes.len();
            assets.insert(asset.route.clone(), Bytes::from(bytes));
        }
        let stream = file(&root, "frames.jsonl", manifest.stream_bytes)?;
        if stream.len() as u64 != manifest.stream_bytes || digest(&stream) != manifest.stream_sha256
        {
            return Err(invalid("recorded stream changed"));
        }
        Self::admit(
            stream.into(),
            manifest.frames,
            manifest.asset_prefix,
            assets,
        )
    }

    pub(super) fn admit(
        stream: Arc<[u8]>,
        frames: u32,
        asset_prefix: String,
        assets: BTreeMap<String, Bytes>,
    ) -> io::Result<Self> {
        if !(1..=4096).contains(&frames) {
            return Err(invalid("recording needs 1–4096 frames"));
        }
        let mut lines = Lines::new(Cursor::new(stream.as_ref()), Limits::default())?;
        if !matches!(
            lines.next_record::<Record>()?,
            Some(Record::Header { version: 1 })
        ) {
            return Err(invalid("recording header missing"));
        }
        let mut index = Vec::new();
        let mut stamps = Vec::new();
        let mut world = None;
        let mut initial = BTreeSet::new();
        let mut seen = BTreeSet::new();
        let mut previous = BTreeSet::new();
        let mut identities = BTreeMap::new();
        let mut validated_documents = BTreeSet::new();
        for ordinal in 1..=frames {
            let start = lines.offset() as usize;
            let Some(Record::Frame {
                ordinal: actual,
                scene,
                catalog,
                outcomes,
            }) = lines.next_record()?
            else {
                return Err(invalid("recording frame missing"));
            };
            if actual != ordinal {
                return Err(invalid("recording ordinal is inconsistent"));
            }
            let frame = decode(&scene, &catalog, outcomes, &asset_prefix, &assets)?;
            for uri in frame.scene.mesh_uris() {
                if validated_documents.insert(uri.to_owned()) {
                    crate::web::assets::validate_recorded_document(uri, &asset_prefix, &assets)?;
                }
            }
            let identity = (frame.scene.world_id(), frame.scene.world_name().to_owned());
            if world.as_ref().is_some_and(|old| old != &identity) {
                return Err(invalid("recording changed world identity"));
            }
            world = Some(identity);
            for (id, definition) in frame
                .scene
                .definitions()?
                .into_iter()
                .chain(frame.catalog.definitions()?)
            {
                if identities.get(&id).is_some_and(|old| old != &definition) {
                    return Err(invalid(
                        "recording changed an existing identity definition or role",
                    ));
                }
                identities.insert(id, definition);
            }
            let ids: BTreeSet<_> = frame
                .catalog
                .model_ids()
                .chain(frame.scene.visual_ids())
                .collect();
            if ordinal == 1 {
                initial = ids.clone();
            }
            let changed_epoch = stamps
                .last()
                .is_some_and(|old: &Stamp| old.epoch != frame.scene.epoch);
            if changed_epoch && ids != initial {
                return Err(invalid("recorded reset did not restore startup identities"));
            }
            if ids.iter().any(|id| {
                seen.contains(id)
                    && !previous.contains(id)
                    && !(changed_epoch && initial.contains(id))
            }) {
                return Err(invalid("recording recycled retired identity"));
            }
            seen.extend(ids.iter().copied());
            previous = ids;
            stamps.push(Stamp {
                epoch: frame.scene.epoch,
                sequence: frame.scene.sequence(),
                seconds: frame.scene.seconds,
                nanoseconds: frame.scene.nanoseconds,
            });
            index.push(start..lines.offset() as usize);
        }
        if !matches!(lines.next_record::<Record>()?, Some(Record::Footer { frames: count }) if count == frames)
            || lines.next_record::<Record>()?.is_some()
        {
            return Err(invalid("recording footer missing or trailing records"));
        }
        Clock::new(stamps.clone(), Instant::now())?;
        Ok(Self {
            stream,
            index,
            stamps,
            asset_prefix,
            assets,
        })
    }

    pub fn frame(&self, ordinal: usize) -> io::Result<Frame> {
        let range = self
            .index
            .get(ordinal)
            .ok_or_else(|| invalid("playback ordinal out of range"))?
            .clone();
        let record: Record =
            serde_json::from_slice(&self.stream[range]).map_err(io::Error::other)?;
        let Record::Frame {
            scene,
            catalog,
            outcomes,
            ..
        } = record
        else {
            return Err(invalid("immutable frame index corrupt"));
        };
        decode(&scene, &catalog, outcomes, &self.asset_prefix, &self.assets)
    }
}

fn decode(
    scene: &str,
    catalog: &str,
    outcomes: Vec<Outcome>,
    prefix: &str,
    assets: &BTreeMap<String, Bytes>,
) -> io::Result<Frame> {
    let scene = Scene::decode(scene, |uri| {
        uri.strip_prefix(prefix)
            .is_some_and(|route| assets.contains_key(route))
    })?;
    let catalog = Catalog::decode(catalog, &scene)?;
    if outcomes.len() > 8 {
        return Err(invalid("too many recorded control outcomes"));
    }
    for outcome in &outcomes {
        if outcome.request.len() > 4096
            || outcome.result.len() > 4096
            || outcome.model_id == Some(0)
            || (matches!(
                outcome.action,
                Action::Pause | Action::Resume | Action::Reset
            ) && outcome.model_id.is_some())
            || (matches!(
                outcome.action,
                Action::Spawn | Action::Remove | Action::GetState | Action::SetState
            ) && outcome.ok
                && !outcome.model_id.is_some_and(|id| id <= u32::MAX as u64))
        {
            return Err(invalid("invalid recorded control outcome"));
        }
        outcome.validate(&scene)?;
        if outcome.ok
            && let Some(id) = outcome.model_id
        {
            let present = catalog.model_ids().any(|model| model == id);
            if (outcome.action == Action::Remove && present)
                || (matches!(
                    outcome.action,
                    Action::Spawn | Action::GetState | Action::SetState
                ) && !present)
            {
                return Err(invalid(
                    "recorded lifecycle result disagrees with committed catalog",
                ));
            }
        }
    }
    Ok(Frame {
        scene,
        catalog,
        outcomes,
    })
}

#[cfg(test)]
#[path = "archive_tests.rs"]
mod tests;

#[cfg(test)]
#[path = "archive_admission_tests.rs"]
mod admission_tests;
