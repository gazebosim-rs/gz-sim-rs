// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use crate::{Counters, Error, world_io};
use crate::{ServerOptions, SetupError};
use gz_sim_core::{EntityId, Simulation, entity_by_id};
use gz_sim_proto::{
    msgs::{Double, Empty},
    prost::{Message, Name},
};
use gz_sim_scene::{
    ImuPublication, LidarPublication, RateChange, lidar_rate, set_imu_rate, set_lidar_rate,
};
use std::{
    collections::BTreeMap,
    num::NonZeroUsize,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
};
use zenoh::{
    Wait,
    liveliness::LivelinessToken,
    query::{Query, Queryable},
};

struct ProviderSpec {
    name: String,
}

fn endpoint_groups<'a>(
    config: &ServerOptions,
    sensors: impl IntoIterator<Item = &'a ImuPublication>,
) -> Result<BTreeMap<String, Vec<ProviderSpec>>, SetupError> {
    if config.capacity == 0
        || config.capacity > i32::MAX as usize
        || config.max_message_bytes == 0
        || config.max_message_bytes > i64::MAX as usize
    {
        return Err(SetupError::InvalidConfig);
    }
    let mut groups: BTreeMap<String, Vec<ProviderSpec>> = BTreeMap::new();
    for sensor in sensors {
        let topic = format!("{}/set_rate", crate::options::as_valid_topic(&sensor.topic));
        let remapped = config.topic_remap(&topic).unwrap_or(&topic);
        let endpoint =
            crate::options::qualified_name(&config.partition, &config.namespace, remapped)?;
        groups.entry(endpoint).or_default().push(ProviderSpec {
            name: sensor.entity_name.clone(),
        });
    }
    Ok(groups)
}

/// Normalize the constructed raw rate sibling, then apply exactly one remap
/// and qualification. Native normalization removes forbidden token pairs.
pub(crate) fn lidar_endpoint(
    config: &ServerOptions,
    sensor: &LidarPublication,
) -> Result<String, SetupError> {
    config.topic_key(&crate::options::as_valid_topic(&format!(
        "{}/set_rate",
        sensor.topic
    )))
}

// Lidar control endpoints are never shared. The retained IMU endpoint groups
// deliberately keep their existing same-key multiple-provider semantics.
fn lidar_groups<'a>(
    config: &ServerOptions,
    sensors: impl IntoIterator<Item = &'a LidarPublication>,
    imus: &BTreeMap<String, Vec<ProviderSpec>>,
) -> Result<BTreeMap<String, &'a LidarPublication>, Error> {
    let mut groups = BTreeMap::new();
    let mut names = std::collections::BTreeSet::new();
    for sensor in sensors {
        let key = lidar_endpoint(config, sensor)?;
        if groups.len() >= 8
            || sensor.entity_name.is_empty()
            || !names.insert(sensor.entity_name.as_str())
            || imus.contains_key(&key)
            || groups.insert(key, sensor).is_some()
        {
            return Err(Error::LidarIdentity);
        }
    }
    Ok(groups)
}

enum Family {
    Imu,
    Lidar(LidarPublication),
}

pub(crate) struct Provider {
    name: String,
    family: Family,
    id: Option<u64>,
    input: mpsc::Receiver<Option<f64>>,
    active: Arc<AtomicBool>,
}
impl Drop for Provider {
    fn drop(&mut self) {
        self.active.store(false, Ordering::Release);
    }
}

struct DeclarationSpec {
    key: String,
    input: mpsc::SyncSender<Option<f64>>,
    active: Arc<AtomicBool>,
}

/// Plain data only: a failed command enqueue/drop cannot finalize a Query.
pub(super) struct RateSetup {
    providers: Vec<DeclarationSpec>,
    max_bytes: usize,
    ingress_drops: Arc<AtomicU64>,
}
impl RateSetup {
    pub(super) fn is_empty(&self) -> bool {
        self.providers.is_empty()
    }
}

/// CPU state. No Query, declaration, Session or transport-owned payload lives here.
pub(crate) struct Rates {
    providers: Vec<Provider>,
    next: usize,
}
impl Rates {
    pub(crate) fn prepare<'a>(
        config: &ServerOptions,
        sensors: impl IntoIterator<Item = &'a ImuPublication>,
        lidars: impl IntoIterator<Item = &'a LidarPublication>,
        ingress_drops: &Arc<AtomicU64>,
    ) -> Result<(Self, RateSetup), Error> {
        let mut providers = Vec::new();
        let mut declarations = Vec::new();
        let imu_groups = endpoint_groups(config, sensors)?;
        let lidar_groups = lidar_groups(config, lidars, &imu_groups)?;
        for (key, specs) in imu_groups {
            for spec in specs {
                let (input, receiver) = mpsc::sync_channel(config.capacity);
                let active = Arc::new(AtomicBool::new(true));
                providers.push(Provider {
                    name: spec.name,
                    family: Family::Imu,
                    id: None,
                    input: receiver,
                    active: Arc::clone(&active),
                });
                declarations.push(DeclarationSpec {
                    key: key.clone(),
                    input,
                    active,
                });
            }
        }
        for (key, metadata) in lidar_groups {
            let (input, receiver) = mpsc::sync_channel(config.capacity);
            let active = Arc::new(AtomicBool::new(true));
            providers.push(Provider {
                name: metadata.entity_name.clone(),
                family: Family::Lidar(metadata.clone()),
                id: None,
                input: receiver,
                active: Arc::clone(&active),
            });
            declarations.push(DeclarationSpec { key, input, active });
        }
        Ok((
            Self { providers, next: 0 },
            RateSetup {
                providers: declarations,
                max_bytes: config.max_message_bytes,
                ingress_drops: Arc::clone(ingress_drops),
            },
        ))
    }

    pub(crate) fn bind(&mut self, sim: &Simulation) -> Result<(), Error> {
        let mut imus = BTreeMap::new();
        let mut lidars = BTreeMap::new();
        let has_lidars = self
            .providers
            .iter()
            .any(|provider| matches!(&provider.family, Family::Lidar(_)));
        for entity in sim.world().iter_entities() {
            if entity.contains::<gz_sim_scene::PendingResetRemoval>() {
                continue;
            }
            if let Some(sensor) = entity.get::<ImuPublication>() {
                let id = entity.get::<EntityId>().ok_or(Error::ImuIdentity)?.get();
                if imus.insert(sensor.entity_name.as_str(), id).is_some() {
                    return Err(Error::ImuIdentity);
                }
            }
            if has_lidars && let Some(sensor) = entity.get::<LidarPublication>() {
                let id = entity.get::<EntityId>().ok_or(Error::LidarIdentity)?.get();
                if lidars.len() >= 8
                    || entity_by_id(sim.world(), id) != Some(entity.id())
                    || entity.get::<gz_sim_scene::EntityKind>()
                        != Some(&gz_sim_scene::EntityKind::Sensor)
                    || lidar_rate(sim, id).is_err()
                    || lidars
                        .insert(sensor.entity_name.as_str(), (id, sensor))
                        .is_some()
                {
                    return Err(Error::LidarIdentity);
                }
            }
        }
        // Complete the new-family preflight before assigning even the first ID.
        let mut ids = Vec::with_capacity(self.providers.len());
        for provider in &self.providers {
            ids.push(match &provider.family {
                Family::Imu => imus.get(provider.name.as_str()).copied(),
                Family::Lidar(expected) => match lidars.get(provider.name.as_str()) {
                    Some((id, actual)) if *actual == expected => Some(*id),
                    Some(_) => return Err(Error::LidarIdentity),
                    None => None,
                },
            });
        }
        for (provider, id) in self.providers.iter_mut().zip(ids) {
            provider.id = id;
        }
        self.retire(sim);
        Ok(())
    }

    pub(crate) fn retire(&mut self, sim: &Simulation) {
        self.providers.retain(|provider| {
            let Some((id, entity)) = provider
                .id
                .and_then(|id| entity_by_id(sim.world(), id).map(|entity| (id, entity)))
            else {
                return false;
            };
            match &provider.family {
                Family::Imu => sim
                    .world()
                    .get::<ImuPublication>(entity)
                    .is_some_and(|sensor| sensor.entity_name == provider.name),
                Family::Lidar(expected) => {
                    sim.world()
                        .get::<LidarPublication>(entity)
                        .is_some_and(|sensor| {
                            sensor == expected && sensor.entity_name == provider.name
                        })
                        && sim.world().get::<gz_sim_scene::EntityKind>(entity)
                            == Some(&gz_sim_scene::EntityKind::Sensor)
                        && lidar_rate(sim, id).is_ok()
                }
            }
        });
        if self.providers.is_empty() {
            self.providers = Vec::new();
            self.next = 0;
        }
    }

    /// Produce replacement plain setup. IO installation/acknowledgment belongs
    /// outside stepping; the caller must not advance before that phase completes.
    pub(crate) fn restart(
        config: &ServerOptions,
        sim: &Simulation,
        ingress_drops: &Arc<AtomicU64>,
    ) -> Result<(Self, RateSetup), Error> {
        let sensors: Vec<_> = match gz_sim_scene::initial_imu_publications(sim) {
            Ok(sensors) => sensors.cloned().collect(),
            Err(gz_sim_scene::SceneResetError::MissingInitialState) => sim
                .world()
                .iter_entities()
                .filter(|entity| !entity.contains::<gz_sim_scene::PendingResetRemoval>())
                .filter_map(|entity| entity.get::<ImuPublication>().cloned())
                .collect(),
            Err(error) => return Err(error.into()),
        };
        let lidars: Vec<_> = match gz_sim_scene::initial_lidar_publications(sim) {
            Ok(sensors) => sensors.cloned().collect(),
            Err(gz_sim_scene::SceneResetError::MissingInitialState) => sim
                .world()
                .iter_entities()
                .filter(|entity| !entity.contains::<gz_sim_scene::PendingResetRemoval>())
                .filter_map(|entity| entity.get::<LidarPublication>().cloned())
                .collect(),
            Err(error) => return Err(error.into()),
        };
        let (mut replacement, setup) = Self::prepare(config, &sensors, &lidars, ingress_drops)?;
        replacement.bind(sim)?;
        Ok((replacement, setup))
    }

    pub(crate) fn clear(&mut self) {
        self.providers = Vec::new();
        self.next = 0;
    }
    pub(crate) fn shutdown(&mut self) {
        self.clear();
    }
    pub(crate) fn is_empty(&self) -> bool {
        self.providers.is_empty()
    }
    pub(crate) fn into_active(self) -> Option<Self> {
        if self.is_empty() { None } else { Some(self) }
    }

    pub(crate) fn poll(
        &mut self,
        sim: &mut Simulation,
        budget: NonZeroUsize,
        counters: &mut Counters,
    ) -> Result<(), Error> {
        for _ in 0..budget.get() {
            let mut received = None;
            for _ in 0..self.providers.len() {
                self.next %= self.providers.len();
                let provider = &self.providers[self.next];
                self.next += 1;
                if !provider.active.load(Ordering::Acquire) {
                    continue;
                }
                match provider.input.try_recv() {
                    Ok(value) => {
                        received = Some((
                            provider.id,
                            value,
                            matches!(&provider.family, Family::Lidar(_)),
                        ));
                        break;
                    }
                    Err(mpsc::TryRecvError::Empty) => {}
                    Err(mpsc::TryRecvError::Disconnected) => {
                        return Err(Error::WorldNetwork("rate ingress closed".into()));
                    }
                }
            }
            let Some((id, value, lidar)) = received else {
                break;
            };
            if lidar {
                let change =
                    value.and_then(|value| id.and_then(|id| set_lidar_rate(sim, id, value).ok()));
                match change {
                    Some(RateChange::Applied) => counters.lidar_rates_applied += 1,
                    Some(RateChange::Ignored) => counters.lidar_rates_ignored += 1,
                    None => counters.lidar_rates_rejected += 1,
                }
            } else {
                let change =
                    value.and_then(|value| id.and_then(|id| set_imu_rate(sim, id, value).ok()));
                match change {
                    Some(RateChange::Applied) => counters.imu_rates_applied += 1,
                    Some(RateChange::Ignored) => counters.imu_rates_ignored += 1,
                    None => counters.imu_rates_rejected += 1,
                }
            }
        }
        Ok(())
    }
}

// Every callback capture is inside this Arc. Its final non-owner reference
// therefore accounts for queued requests AND the actual callback capture exit.
struct RateCallbackState {
    active: Arc<AtomicBool>,
    input: mpsc::SyncSender<Option<f64>>,
    ingress: mpsc::SyncSender<world_io::ControlWork>,
    ingress_drops: Arc<AtomicU64>,
    max_bytes: usize,
}

/// IO-only work. Field order ensures cancellation drops Query before the state
/// reference used to observe completion. CPU queues contain only Option<f64>.
pub(super) struct RateRequest {
    query: Option<Query>,
    state: Arc<RateCallbackState>,
}
impl RateRequest {
    pub(super) fn execute(self) -> Result<(), Error> {
        // The called function owns and finalizes Query before this Arc can drop.
        execute_query(self.query, &self.state)
    }
}

fn execute_query(query: Option<Query>, state: &RateCallbackState) -> Result<(), Error> {
    if !state.active.load(Ordering::Acquire) {
        return Ok(());
    }
    let value = query.as_ref().and_then(|query| {
        let bytes = query
            .payload()
            .map(|payload| payload.to_bytes())
            .unwrap_or_default();
        Double::decode(bytes.as_ref()).ok().map(|value| value.data)
    });
    if !state.active.load(Ordering::Acquire) {
        return Ok(());
    }
    match state.input.try_send(value) {
        Ok(()) => {
            if let Some(query) = &query {
                // Native Empty has no type attachment/envelope. The receipt is
                // queue handoff, not parse success or the later CPU outcome.
                // Native CreateMsg logs malformed input but still returns a
                // message, so the retained malformed receipt also stays Empty.
                // BLOCK may wait here; this function must run on the dedicated IO owner.
                query
                    .reply(query.key_expr().clone(), Vec::<u8>::new())
                    .wait()
                    .map_err(|error| Error::WorldNetwork(error.to_string()))?;
            }
        }
        Err(mpsc::TrySendError::Full(_)) => {
            state.ingress_drops.fetch_add(1, Ordering::Relaxed);
            if let Some(query) = &query {
                query
                    .reply_err("rate ingress full")
                    .wait()
                    .map_err(|error| Error::WorldNetwork(error.to_string()))?;
            }
        }
        Err(mpsc::TrySendError::Disconnected(_)) => {} // Receiver retired; no new mutation.
    }
    Ok(())
}

struct Declaration {
    queryable: Option<Queryable<()>>,
    presence: Option<LivelinessToken>,
    state: Arc<RateCallbackState>,
}
impl Drop for Declaration {
    fn drop(&mut self) {
        self.state.active.store(false, Ordering::Release);
    }
}

/// Owned and reconciled only by the application IO owner. Dropping a declaration
/// or a final Query may perform blocking protocol IO; neither belongs in tick.
pub(super) struct RateDeclarations {
    providers: Vec<Declaration>,
}
impl RateDeclarations {
    pub(super) fn new(
        session: &zenoh::Session,
        setup: RateSetup,
        ingress: mpsc::SyncSender<world_io::ControlWork>,
    ) -> Result<Self, Error> {
        if session.is_closed() {
            return Err(Error::WorldClosed);
        }
        let mut result = Self {
            providers: Vec::new(),
        };
        let process = session.zid().to_string();
        let types = format!("{}&{}", Double::full_name(), Empty::full_name());
        for provider in setup.providers {
            if !provider.active.load(Ordering::Acquire) {
                continue;
            }
            let state = Arc::new(RateCallbackState {
                active: provider.active,
                input: provider.input,
                ingress: ingress.clone(),
                ingress_drops: Arc::clone(&setup.ingress_drops),
                max_bytes: setup.max_bytes,
            });
            let mut declaration = Declaration {
                queryable: None,
                presence: None,
                state,
            };
            let node = uuid::Uuid::new_v4().to_string();
            let token = world_io::presence(&provider.key, &process, &node, "SS", &types)
                .map_err(|error| Error::WorldNetwork(error.to_string()))?;
            let callback = Arc::clone(&declaration.state);
            let queryable = session
                .declare_queryable(provider.key)
                .callback(move |query: Query| {
                    if !callback.active.load(Ordering::Acquire) {
                        return;
                    }
                    let mut query = Some(query);
                    if query
                        .as_ref()
                        .and_then(|query| query.payload())
                        .is_some_and(|payload| payload.len() > callback.max_bytes)
                    {
                        // Never retain an oversized payload in the bounded IO queue.
                        // Rejected finalization stays on this upstream callback,
                        // outside CPU execution; pressure qualification is separate.
                        drop(query.take());
                    }
                    let request = RateRequest {
                        query,
                        state: Arc::clone(&callback),
                    };
                    match callback
                        .ingress
                        .try_send(world_io::ControlWork::Rate(request))
                    {
                        Ok(()) => {}
                        Err(mpsc::TrySendError::Full(rejected)) => {
                            callback.ingress_drops.fetch_add(1, Ordering::Relaxed);
                            drop(rejected);
                        }
                        Err(mpsc::TrySendError::Disconnected(_)) => {}
                    }
                })
                .wait()
                .map_err(|error| Error::WorldNetwork(error.to_string()))?;
            declaration.queryable = Some(queryable);
            let presence = session
                .liveliness()
                .declare_token(token)
                .wait()
                .map_err(|error| Error::WorldNetwork(error.to_string()))?;
            declaration.presence = Some(presence);
            result.providers.push(declaration);
        }
        Ok(result)
    }

    /// Retire disabled endpoints and report whether all observed retirements
    /// have completed, including queued Query finalization and capture release.
    pub(super) fn reconcile(&mut self) -> bool {
        for provider in &mut self.providers {
            if !provider.state.active.load(Ordering::Acquire) {
                drop(provider.queryable.take());
                drop(provider.presence.take());
            }
        }
        self.providers.retain(|provider| {
            provider.state.active.load(Ordering::Acquire) || Arc::strong_count(&provider.state) != 1
        });
        self.providers
            .iter()
            .all(|provider| provider.state.active.load(Ordering::Acquire))
    }
    pub(super) fn is_empty(&self) -> bool {
        self.providers.is_empty()
    }
    pub(super) fn begin_shutdown(&self) {
        for provider in &self.providers {
            provider.state.active.store(false, Ordering::Release);
        }
    }
}
impl Drop for RateDeclarations {
    fn drop(&mut self) {
        self.begin_shutdown();
    }
}

#[cfg(test)]
#[path = "../tests/private/rates.rs"]
mod tests;

#[cfg(test)]
#[path = "../tests/private/lidar_rates.rs"]
mod lidar_tests;

#[cfg(test)]
#[path = "../tests/private/lidar_rate_boundaries.rs"]
mod lidar_boundary_tests;
