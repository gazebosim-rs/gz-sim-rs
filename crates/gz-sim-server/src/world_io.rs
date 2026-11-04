// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Direct Zenoh ownership for this world's clock, poses and control endpoints.
use gz_sim_proto::{
    msgs::{Boolean, Clock, PoseV},
    prost::{Message, Name},
};
use std::{
    env,
    error::Error,
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc,
    },
    thread,
    time::{Duration, Instant},
};
use zenoh::{
    Wait,
    liveliness::LivelinessToken,
    pubsub::Publisher,
    qos::CongestionControl,
    query::{Query, Queryable},
};

type Result<T> = std::result::Result<T, Box<dyn Error + Send + Sync>>;
pub(super) use crate::imu_io::ImuOutput;
pub(super) use crate::lidar_io::LidarOutput;

const TCP_PROFILE: [(&str, u64); 5] = [
    ("transport/link/tx/lease", 1000),
    ("transport/link/tx/keep_alive", 4),
    (
        "transport/link/tx/queue/congestion_control/block/wait_before_close",
        100_000,
    ),
    (
        "transport/link/tx/queue/congestion_control/drop/wait_before_drop",
        0,
    ),
    (
        "transport/link/tx/queue/congestion_control/drop/max_wait_before_drop_fragments",
        1000,
    ),
];

fn configure_tcp_profile(config: &mut zenoh::Config) -> Result<()> {
    let defaults = zenoh::Config::default();
    for (path, expected) in TCP_PROFILE {
        let value = config.get_json(path)?;
        // Native defaults favor graceful delivery over bounded simulator shutdown.
        if value.trim() == "null" || value == defaults.get_json(path)? {
            config.insert_json5(path, &expected.to_string())?;
        } else if value.trim().parse::<u64>() != Ok(expected) {
            return Err(format!("unsupported TCP setting {path}: expected {expected}").into());
        }
    }
    Ok(())
}

struct ControlResponse {
    bytes: Vec<u8>,
    failures: Arc<AtomicU64>,
    accounted: bool,
}

impl Drop for ControlResponse {
    fn drop(&mut self) {
        if !self.accounted {
            self.failures.fetch_add(1, Ordering::Relaxed);
        }
    }
}

pub(super) struct ControlRequest {
    pub(super) payload: Vec<u8>,
    response: mpsc::SyncSender<ControlResponse>,
    reply_failures: Arc<AtomicU64>,
}

impl ControlRequest {
    pub(super) fn reply(self, bytes: Vec<u8>) -> Result<()> {
        match self.response.try_send(ControlResponse {
            bytes,
            failures: self.reply_failures,
            accounted: false,
        }) {
            Ok(()) => Ok(()),
            Err(error) => {
                let (mut response, reason) = match error {
                    mpsc::TrySendError::Full(response) => (response, "control reply full"),
                    mpsc::TrySendError::Disconnected(response) => {
                        (response, "control reply closed")
                    }
                };
                // The CPU caller counts failed admission; only admitted replies
                // need the IO-side discard guard.
                response.accounted = true;
                Err(reason.into())
            }
        }
    }
}

struct PendingControl {
    query: Query,
    response: mpsc::Receiver<ControlResponse>,
}

pub(super) enum ControlWork {
    World(Query),
    Remove(usize, Query),
    InstallRemovals {
        keys: [String; 2],
        inputs: [mpsc::SyncSender<ControlRequest>; 2],
        receipt: mpsc::SyncSender<std::result::Result<(), String>>,
    },
    Rate(crate::rates::RateRequest),
    Stop,
}

struct RemovalDeclarations {
    _queries: Vec<Queryable<()>>,
    _presence: Vec<LivelinessToken>,
    inputs: [mpsc::SyncSender<ControlRequest>; 2],
}

fn queue_request(
    query: Query,
    commands: &mpsc::SyncSender<ControlRequest>,
    pending: &mut Vec<PendingControl>,
    failures: &Arc<AtomicU64>,
) -> std::result::Result<(), mpsc::TrySendError<ControlRequest>> {
    let (response, replies) = mpsc::sync_channel(1);
    let payload = query
        .payload()
        .map(|value| value.to_bytes().into_owned())
        .unwrap_or_default();
    commands.try_send(ControlRequest {
        payload,
        response,
        reply_failures: Arc::clone(failures),
    })?;
    pending.push(PendingControl {
        query,
        response: replies,
    });
    Ok(())
}

enum WorldWork {
    Clock(u64, Vec<u8>),
    Poses(u64, Vec<u8>),
    Imu(u64, crate::imu_io::ImuJob),
    Lidar(u64, crate::lidar_io::LidarJob),
    Odometry(u64, crate::diff_drive_io::Job),
    InstallDrive(crate::diff_drive_io::Setup, crate::diff_drive_io::Receipt),
    InstallJointControllers(
        crate::joint_controller_io::Setup,
        crate::joint_controller_io::Receipt,
    ),
    InstallLidar(
        String,
        mpsc::SyncSender<std::result::Result<Arc<LidarOutput>, String>>,
    ),
    InstallImu(
        String,
        mpsc::SyncSender<std::result::Result<Arc<ImuOutput>, String>>,
    ),
    RootClock(u64, Arc<Publisher<'static>>, Vec<u8>),
    InstallRates(
        crate::rates::RateSetup,
        mpsc::SyncSender<std::result::Result<(), String>>,
    ),
    InstallForces(
        crate::force_io::ForceSetup,
        mpsc::SyncSender<std::result::Result<(), String>>,
    ),
    Stop,
}

fn reply_controls(pending: &mut Vec<PendingControl>, session: &zenoh::Session, failed: &AtomicU64) {
    let mut index = 0;
    while index < pending.len() {
        let response = match pending[index].response.try_recv() {
            Ok(bytes) => Some(Some(bytes)),
            Err(mpsc::TryRecvError::Disconnected) => Some(None),
            Err(mpsc::TryRecvError::Empty) => None,
        };
        let Some(response) = response else {
            index += 1;
            continue;
        };
        let request = pending.swap_remove(index);
        if let Some(mut response) = response {
            let bytes = std::mem::take(&mut response.bytes);
            response.accounted = true;
            if session.is_closed()
                || request
                    .query
                    .reply(request.query.key_expr().clone(), bytes)
                    .wait()
                    .is_err()
            {
                failed.fetch_add(1, Ordering::Relaxed);
            }
        }
        // Query finalization can send a blocking protocol message. Keep it here.
        drop(request);
    }
}

pub(super) struct RootClock {
    pub(super) publisher: Arc<Publisher<'static>>,
    _presence: LivelinessToken,
}

pub(super) struct WorldIo {
    _presence: [LivelinessToken; 2],
    pub(super) clock: Arc<Publisher<'static>>,
    pub(super) poses: Arc<Publisher<'static>>,
    pub(super) controls: mpsc::Receiver<ControlRequest>,
    pub(super) rejected: Arc<AtomicU64>,
    pub(super) ingress_drops: Arc<AtomicU64>,
    pub(super) reply_failures: Arc<AtomicU64>,
    pub(super) publication_failures: Arc<[AtomicU64; 6]>,
    pub(super) epoch: Arc<AtomicU64>,
    pub(super) max_message_bytes: usize,
    capacity: usize,
    control_key: String,
    removal_keys: std::sync::OnceLock<[String; 2]>,
    pub(super) session: zenoh::Session,
    process: String,
    node: String,
    stop: Arc<AtomicBool>,
    incoming: crossbeam_channel::Sender<WorldWork>,
    control_worker: Option<thread::JoinHandle<()>>,
    publication_worker: Option<thread::JoinHandle<()>>,
    control_incoming: mpsc::SyncSender<ControlWork>,
}

impl WorldIo {
    pub(super) fn new(
        clock_key: String,
        control_key: String,
        poses_key: String,
        capacity: usize,
        max_bytes: usize,
    ) -> Result<Self> {
        let config = match env::var_os("ZENOH_CONFIG") {
            Some(path) => zenoh::Config::from_file(path)?,
            None => zenoh::Config::default(),
        };
        Self::new_with_config(
            clock_key,
            control_key,
            poses_key,
            capacity,
            max_bytes,
            config,
        )
    }

    pub(super) fn new_with_config(
        clock_key: String,
        control_key: String,
        poses_key: String,
        capacity: usize,
        max_bytes: usize,
        mut config: zenoh::Config,
    ) -> Result<Self> {
        if capacity == 0 || max_bytes == 0 {
            return Err("invalid world ingress bounds".into());
        }
        if clock_key == poses_key {
            return Err("world clock and pose publisher keys collide".into());
        }
        configure_tcp_profile(&mut config)?;
        let session = zenoh::open(config).wait()?;
        let process = uuid::Uuid::new_v4().to_string();
        let node = uuid::Uuid::new_v4().to_string();
        let clock = Arc::new(
            session
                .declare_publisher(clock_key.clone())
                .congestion_control(CongestionControl::Drop)
                .wait()?,
        );
        let poses = Arc::new(
            session
                .declare_publisher(poses_key.clone())
                .congestion_control(CongestionControl::Drop)
                .wait()?,
        );
        let rejected = Arc::new(AtomicU64::new(0));
        let ingress_drops = Arc::new(AtomicU64::new(0));
        let dropped = Arc::clone(&rejected);
        let ingress_lost = Arc::clone(&ingress_drops);
        let (control_incoming, queries) = mpsc::sync_channel(capacity);
        let tx = control_incoming.clone();
        let endpoint = session
            .declare_queryable(control_key.clone())
            .callback(move |query: Query| {
                if query
                    .payload()
                    .is_some_and(|payload| payload.len() > max_bytes)
                {
                    dropped.fetch_add(1, Ordering::Relaxed);
                    return;
                }
                match tx.try_send(ControlWork::World(query)) {
                    Ok(()) => {}
                    Err(mpsc::TrySendError::Full(_)) => {
                        dropped.fetch_add(1, Ordering::Relaxed);
                        ingress_lost.fetch_add(1, Ordering::Relaxed);
                    }
                    Err(mpsc::TrySendError::Disconnected(_)) => {
                        dropped.fetch_add(1, Ordering::Relaxed);
                    }
                }
            })
            .wait()?;
        let clock_presence = session
            .liveliness()
            .declare_token(presence(
                &clock_key,
                &process,
                &node,
                "MP",
                &Clock::full_name(),
            )?)
            .wait()?;
        let control_presence = session
            .liveliness()
            .declare_token(presence(
                &control_key,
                &process,
                &node,
                "SS",
                "gz.msgs.WorldControl&gz.msgs.Boolean",
            )?)
            .wait()?;
        let poses_presence = session
            .liveliness()
            .declare_token(presence(
                &poses_key,
                &process,
                &node,
                "MP",
                &PoseV::full_name(),
            )?)
            .wait()?;
        let (commands, controls) = mpsc::sync_channel(capacity);
        let stop = Arc::new(AtomicBool::new(false));
        let worker_stop = Arc::clone(&stop);
        let reply_failures = Arc::new(AtomicU64::new(0));
        let failed = Arc::clone(&reply_failures);
        let worker_rejected = Arc::clone(&rejected);
        let worker_ingress_lost = Arc::clone(&ingress_drops);
        let worker_session = session.clone();
        let removal_ingress = control_incoming.clone();
        let removal_process = process.clone();
        let removal_node = node.clone();
        let worker_clock = Arc::clone(&clock);
        let worker_poses = Arc::clone(&poses);
        let epoch = Arc::new(AtomicU64::new(0));
        let worker_epoch = Arc::clone(&epoch);
        let publication_failures: Arc<[AtomicU64; 6]> =
            Arc::new(std::array::from_fn(|_| AtomicU64::new(0)));
        let failures = Arc::clone(&publication_failures);
        let control_worker =
            thread::Builder::new()
                .name("world-controls".into())
                .spawn(move || {
                    let _declarations = (endpoint, control_presence);
                    let mut removals: Option<RemovalDeclarations> = None;
                    let mut pending = Vec::with_capacity(capacity);
                    while !worker_stop.load(Ordering::Acquire) {
                        reply_controls(&mut pending, &worker_session, &failed);
                        let incoming = if pending.is_empty() {
                            queries
                                .recv()
                                .map_err(|_| mpsc::RecvTimeoutError::Disconnected)
                        } else {
                            queries.recv_timeout(Duration::from_millis(1))
                        };
                        match incoming {
                            Ok(ControlWork::World(query)) if pending.len() < capacity => {
                                if let Err(error) =
                                    queue_request(query, &commands, &mut pending, &failed)
                                {
                                    worker_rejected.fetch_add(1, Ordering::Relaxed);
                                    if matches!(error, mpsc::TrySendError::Full(_)) {
                                        worker_ingress_lost.fetch_add(1, Ordering::Relaxed);
                                    }
                                }
                            }
                            Ok(ControlWork::World(_)) => {
                                worker_rejected.fetch_add(1, Ordering::Relaxed);
                                worker_ingress_lost.fetch_add(1, Ordering::Relaxed);
                            }
                            Ok(ControlWork::Rate(request)) => {
                                if request.execute().is_err() {
                                    failed.fetch_add(1, Ordering::Relaxed);
                                }
                            }
                            Ok(ControlWork::Remove(index, query)) => {
                                let input = removals
                                    .as_ref()
                                    .and_then(|declarations| declarations.inputs.get(index));
                                if pending.len() >= capacity {
                                    worker_ingress_lost.fetch_add(1, Ordering::Relaxed);
                                } else if let Some(input) = input
                                    && matches!(
                                        queue_request(query, input, &mut pending, &failed),
                                        Err(mpsc::TrySendError::Full(_))
                                    )
                                {
                                    worker_ingress_lost.fetch_add(1, Ordering::Relaxed);
                                }
                            }
                            Ok(ControlWork::InstallRemovals {
                                keys,
                                inputs,
                                receipt,
                            }) => {
                                let result = (|| -> Result<RemovalDeclarations> {
                                    if removals.is_some() {
                                        return Err("removal services already installed".into());
                                    }
                                    let mut queries = Vec::with_capacity(2);
                                    let mut presence_tokens = Vec::with_capacity(2);
                                    for (index, key) in keys.into_iter().enumerate() {
                                        let sender = removal_ingress.clone();
                                        let drops = Arc::clone(&worker_ingress_lost);
                                        queries.push(
                                            worker_session
                                                .declare_queryable(key.clone())
                                                .callback(move |query: Query| {
                                                    if query.payload().is_some_and(|payload| {
                                                        payload.len() > max_bytes
                                                    }) {
                                                        return;
                                                    }
                                                    if matches!(
                                                        sender.try_send(ControlWork::Remove(
                                                            index, query
                                                        )),
                                                        Err(mpsc::TrySendError::Full(_))
                                                    ) {
                                                        drops.fetch_add(1, Ordering::Relaxed);
                                                    }
                                                })
                                                .wait()?,
                                        );
                                        presence_tokens.push(
                                            worker_session
                                                .liveliness()
                                                .declare_token(presence(
                                                    &key,
                                                    &removal_process,
                                                    &removal_node,
                                                    "SS",
                                                    "gz.msgs.Entity&gz.msgs.Boolean",
                                                )?)
                                                .wait()?,
                                        );
                                    }
                                    Ok(RemovalDeclarations {
                                        _queries: queries,
                                        _presence: presence_tokens,
                                        inputs,
                                    })
                                })();
                                match result {
                                    Ok(declarations) => {
                                        if receipt.try_send(Ok(())).is_ok() {
                                            removals = Some(declarations);
                                        }
                                    }
                                    Err(error) => {
                                        let _ = receipt.try_send(Err(error.to_string()));
                                    }
                                }
                            }
                            Ok(ControlWork::Stop) | Err(mpsc::RecvTimeoutError::Disconnected) => {
                                break;
                            }
                            Err(mpsc::RecvTimeoutError::Timeout) => {}
                        }
                    }
                    // Both the pending and ingress Query values are destroyed on this IO thread.
                    drop(pending);
                    drop(queries);
                })?;
        let (incoming, publications) = crossbeam_channel::bounded(capacity);
        let publication_stop = Arc::clone(&stop);
        let publication_session = session.clone();
        let rate_ingress = control_incoming.clone();
        let publication_process = process.clone();
        let publication_node = node.clone();
        let publication_worker = match thread::Builder::new()
            .name("world-publications".into())
            .spawn(move || {
                let mut rates: Option<crate::rates::RateDeclarations> = None;
                let mut forces: Option<crate::force_io::ForceDeclarations> = None;
                let mut drive: Option<crate::diff_drive_io::Declarations> = None;
                let mut controllers: Option<crate::joint_controller_io::Declarations> = None;
                let mut controller_install: Option<(
                    crate::joint_controller_io::Setup,
                    crate::joint_controller_io::Receipt,
                )> = None;
                let mut drive_install: Option<(
                    crate::diff_drive_io::Setup,
                    crate::diff_drive_io::Receipt,
                )> = None;
                let mut imus: Option<crate::imu_io::ImuDeclarations> = None;
                let mut lidars: Option<crate::lidar_io::LidarDeclarations> = None;
                let mut lidar_retirement: Option<(
                    mpsc::SyncSender<crate::lidar_io::LidarId>,
                    mpsc::Receiver<crate::lidar_io::LidarId>,
                )> = None;
                let mut imu_retirement: Option<(
                    mpsc::SyncSender<crate::imu_io::ImuId>,
                    mpsc::Receiver<crate::imu_io::ImuId>,
                )> = None;
                let mut install: Option<(
                    crate::rates::RateSetup,
                    mpsc::SyncSender<std::result::Result<(), String>>,
                )> = None;
                let mut force_install: Option<(
                    crate::force_io::ForceSetup,
                    mpsc::SyncSender<std::result::Result<(), String>>,
                )> = None;
                while !publication_stop.load(Ordering::Acquire) {
                    if let Some((_, retired)) = &imu_retirement {
                        while retired.try_recv().is_ok() {}
                    }
                    if let Some(declarations) = &mut imus {
                        declarations.reconcile(publication_session.is_closed());
                        if declarations.is_empty() {
                            imus = None;
                            imu_retirement = None;
                        }
                    }
                    if let Some((_, retired)) = &lidar_retirement {
                        while retired.try_recv().is_ok() {}
                    }
                    if let Some(declarations) = &mut lidars {
                        declarations.reconcile(publication_session.is_closed());
                        if declarations.is_empty() {
                            lidars = None;
                            lidar_retirement = None;
                        }
                    }
                    if let Some(declarations) = &mut rates {
                        declarations.reconcile();
                        if declarations.is_empty() {
                            rates = None;
                        }
                    }
                    if let Some(declarations) = &mut forces {
                        declarations.reconcile();
                        if declarations.is_empty() {
                            forces = None;
                        }
                    }
                    if drive.as_mut().is_some_and(|declarations| {
                        declarations.reconcile(publication_session.is_closed())
                    }) {
                        drive = None;
                    }
                    if controllers.as_mut().is_some_and(|declarations| {
                        declarations.reconcile(publication_session.is_closed())
                    }) {
                        controllers = None;
                    }
                    if controllers.is_none()
                        && let Some((setup, receipt)) = controller_install.take()
                    {
                        match crate::joint_controller_io::Declarations::new(
                            &publication_session,
                            setup,
                            &publication_process,
                            &publication_node,
                        ) {
                            Ok((declarations, leases)) => {
                                let _ = receipt.try_send(Ok(leases));
                                controllers = Some(declarations);
                            }
                            Err(error) => {
                                let _ = receipt.try_send(Err(error.to_string()));
                            }
                        }
                    }
                    if drive.is_none()
                        && let Some((setup, receipt)) = drive_install.take()
                    {
                        match crate::diff_drive_io::Declarations::new(
                            &publication_session,
                            setup,
                            &publication_process,
                            &publication_node,
                        ) {
                            Ok((declarations, output)) => {
                                let _ = receipt.try_send(Ok(output));
                                drive = Some(declarations);
                            }
                            Err(error) => {
                                let _ = receipt.try_send(Err(error.to_string()));
                            }
                        }
                    }
                    if rates.is_none()
                        && let Some((setup, receipt)) = install.take()
                    {
                        match crate::rates::RateDeclarations::new(
                            &publication_session,
                            setup,
                            rate_ingress.clone(),
                        ) {
                            Ok(declarations) => {
                                if receipt.try_send(Ok(())).is_err() {
                                    declarations.begin_shutdown();
                                }
                                rates = Some(declarations);
                            }
                            Err(error) => {
                                let _ = receipt.try_send(Err(error.to_string()));
                            }
                        }
                    }
                    if forces.is_none()
                        && let Some((setup, receipt)) = force_install.take()
                    {
                        match crate::force_io::ForceDeclarations::new(&publication_session, setup) {
                            Ok(declarations) => {
                                if receipt.try_send(Ok(())).is_err() {
                                    declarations.begin_shutdown();
                                }
                                forces = Some(declarations);
                            }
                            Err(error) => {
                                let _ = receipt.try_send(Err(error.to_string()));
                            }
                        }
                    }
                    let incoming = if drive.is_some()
                        || controllers.is_some()
                        || controller_install.is_some()
                        || drive_install.is_some()
                        || install.is_some()
                        || rates.is_some()
                        || imus.is_some()
                        || lidars.is_some()
                        || forces.is_some()
                        || force_install.is_some()
                    {
                        publications.recv_timeout(Duration::from_millis(1))
                    } else {
                        publications
                            .recv()
                            .map_err(|_| crossbeam_channel::RecvTimeoutError::Disconnected)
                    };
                    match incoming {
                        Ok(WorldWork::Clock(generation, bytes)) => {
                            if generation == worker_epoch.load(Ordering::Acquire)
                                && worker_clock
                                    .put(bytes)
                                    .attachment(Clock::full_name())
                                    .wait()
                                    .is_err()
                            {
                                failures[0].fetch_add(1, Ordering::Relaxed);
                            }
                        }
                        Ok(WorldWork::Poses(generation, bytes)) => {
                            if generation == worker_epoch.load(Ordering::Acquire)
                                && worker_poses
                                    .put(bytes)
                                    .attachment(PoseV::full_name())
                                    .wait()
                                    .is_err()
                            {
                                failures[1].fetch_add(1, Ordering::Relaxed);
                            }
                        }
                        Ok(WorldWork::Imu(generation, job)) => {
                            if generation == worker_epoch.load(Ordering::Acquire)
                                && imus
                                    .as_ref()
                                    .is_some_and(|declarations| declarations.publish(job).is_err())
                            {
                                failures[2].fetch_add(1, Ordering::Relaxed);
                            }
                        }
                        Ok(WorldWork::InstallImu(key, receipt)) => {
                            let declarations =
                                imus.get_or_insert_with(crate::imu_io::ImuDeclarations::new);
                            let (retired, _) =
                                imu_retirement.get_or_insert_with(|| mpsc::sync_channel(capacity));
                            let result = declarations
                                .get_or_create(
                                    &publication_session,
                                    key,
                                    &publication_process,
                                    &publication_node,
                                    retired.clone(),
                                )
                                .map_err(|error| error.to_string());
                            let _ = receipt.try_send(result);
                        }
                        Ok(WorldWork::Lidar(generation, job)) => {
                            if generation == worker_epoch.load(Ordering::Acquire)
                                && lidars
                                    .as_ref()
                                    .is_some_and(|declarations| declarations.publish(job).is_err())
                            {
                                failures[4].fetch_add(1, Ordering::Relaxed);
                            }
                        }
                        Ok(WorldWork::InstallLidar(key, receipt)) => {
                            let declarations =
                                lidars.get_or_insert_with(crate::lidar_io::LidarDeclarations::new);
                            let (retired, _) = lidar_retirement
                                .get_or_insert_with(|| mpsc::sync_channel(capacity));
                            let result = declarations
                                .get_or_create(
                                    &publication_session,
                                    key,
                                    &publication_process,
                                    &publication_node,
                                    retired.clone(),
                                )
                                .map_err(|error| error.to_string());
                            let _ = receipt.try_send(result);
                        }
                        Ok(WorldWork::Odometry(generation, job)) => {
                            if generation == worker_epoch.load(Ordering::Acquire)
                                && drive
                                    .as_ref()
                                    .is_some_and(|declarations| declarations.publish(job).is_err())
                            {
                                failures[5].fetch_add(1, Ordering::Relaxed);
                            }
                        }
                        Ok(WorldWork::InstallDrive(setup, receipt)) => {
                            if drive_install.is_none() {
                                drive_install = Some((setup, receipt));
                            } else {
                                let _ =
                                    receipt.try_send(Err("drive setup is already pending".into()));
                            }
                        }
                        Ok(WorldWork::InstallJointControllers(setup, receipt)) => {
                            if controller_install.is_none() {
                                controller_install = Some((setup, receipt));
                            } else {
                                let _ = receipt.try_send(Err(
                                    "joint controller setup is already pending".into(),
                                ));
                            }
                        }
                        Ok(WorldWork::RootClock(generation, publisher, bytes)) => {
                            if generation == worker_epoch.load(Ordering::Acquire)
                                && publisher
                                    .put(bytes)
                                    .attachment(Clock::full_name())
                                    .wait()
                                    .is_err()
                            {
                                failures[3].fetch_add(1, Ordering::Relaxed);
                            }
                        }
                        Ok(WorldWork::InstallRates(setup, receipt)) => {
                            if install.is_none() {
                                install = Some((setup, receipt));
                            } else {
                                let _ =
                                    receipt.try_send(Err("rate setup is already pending".into()));
                            }
                        }
                        Ok(WorldWork::InstallForces(setup, receipt)) => {
                            if force_install.is_none() {
                                force_install = Some((setup, receipt));
                            } else {
                                let _ =
                                    receipt.try_send(Err("force setup is already pending".into()));
                            }
                        }
                        Ok(WorldWork::Stop)
                        | Err(crossbeam_channel::RecvTimeoutError::Disconnected) => break,
                        Err(crossbeam_channel::RecvTimeoutError::Timeout) => {}
                    }
                }
                if let Some(declarations) = &rates {
                    declarations.begin_shutdown();
                }
                if let Some(declarations) = &forces {
                    declarations.begin_shutdown();
                }
                if let Some(declarations) = &drive {
                    declarations.begin_shutdown();
                }
                if let Some(declarations) = &controllers {
                    declarations.begin_shutdown();
                }
                // Undeclaring an endpoint does not join an in-flight callback.
                // Keep its capture witness on this IO owner until it is released.
                loop {
                    let rates_done = rates.as_mut().is_none_or(|declarations| {
                        declarations.reconcile();
                        declarations.is_empty()
                    });
                    let forces_done = forces.as_mut().is_none_or(|declarations| {
                        declarations.reconcile();
                        declarations.is_empty()
                    });
                    let drive_done = drive
                        .as_mut()
                        .is_none_or(|declarations| declarations.reconcile(true));
                    let controllers_done = controllers
                        .as_mut()
                        .is_none_or(|declarations| declarations.reconcile(true));
                    if rates_done && forces_done && drive_done && controllers_done {
                        break;
                    }
                    thread::sleep(Duration::from_millis(1));
                }
            }) {
            Ok(worker) => worker,
            Err(error) => {
                stop.store(true, Ordering::Release);
                let _ = control_incoming.try_send(ControlWork::Stop);
                let _ = session.close().wait();
                let _ = control_worker.join();
                return Err(error.into());
            }
        };
        Ok(Self {
            _presence: [clock_presence, poses_presence],
            clock,
            poses,
            controls,
            rejected,
            ingress_drops,
            reply_failures,
            publication_failures,
            epoch,
            max_message_bytes: max_bytes,
            capacity,
            control_key,
            removal_keys: std::sync::OnceLock::new(),
            session,
            process,
            node,
            stop,
            incoming,
            control_worker: Some(control_worker),
            publication_worker: Some(publication_worker),
            control_incoming,
        })
    }

    pub(super) fn shutdown(&mut self) {
        self.stop.store(true, Ordering::Release);
        let _ = self.incoming.try_send(WorldWork::Stop);
        let _ = self.control_incoming.try_send(ControlWork::Stop);
        let _ = self.session.close().wait();
        if let Some(worker) = self.control_worker.take() {
            let _ = worker.join();
        }
        if let Some(worker) = self.publication_worker.take() {
            let _ = worker.join();
        }
    }

    pub(super) fn removal_inputs(
        &self,
        keys: [String; 2],
    ) -> std::result::Result<[mpsc::Receiver<ControlRequest>; 2], crate::Error> {
        if self.session.is_closed() {
            return Err(crate::Error::WorldClosed);
        }
        if keys[0] == keys[1] || keys.iter().any(|key| key == &self.control_key) {
            return Err(crate::Error::WorldNetwork(
                "world service keys collide".into(),
            ));
        }
        let retained_keys = keys.clone();
        let [(first, first_input), (second, second_input)] =
            std::array::from_fn(|_| mpsc::sync_channel(self.capacity));
        let (receipt, ready) = mpsc::sync_channel(1);
        self.control_incoming
            .try_send(ControlWork::InstallRemovals {
                keys,
                inputs: [first, second],
                receipt,
            })
            .map_err(|_| {
                crate::Error::WorldNetwork("world control setup queue unavailable".into())
            })?;
        let result = ready.recv_timeout(Duration::from_secs(5));
        if self.session.is_closed() {
            return Err(crate::Error::WorldClosed);
        }
        result
            .map_err(|error| crate::Error::WorldNetwork(error.to_string()))?
            .map_err(crate::Error::WorldNetwork)?;
        let _ = self.removal_keys.set(retained_keys);
        Ok([first_input, second_input])
    }

    pub(super) fn reply_boolean(
        &self,
        request: ControlRequest,
        accepted: bool,
    ) -> std::result::Result<(), crate::Error> {
        if self.session.is_closed() {
            return Err(crate::Error::WorldClosed);
        }
        let bytes = Boolean {
            data: accepted,
            ..Default::default()
        }
        .encode_to_vec();
        if bytes.len() > self.max_message_bytes {
            return Err(crate::Error::WorldNetwork(
                "Boolean exceeds message limit".into(),
            ));
        }
        request
            .reply(bytes)
            .map_err(|error| crate::Error::WorldNetwork(error.to_string()))
    }

    pub(super) fn imu_output(&self, key: String) -> Result<Arc<ImuOutput>> {
        if key == self.clock.key_expr().as_str() || key == self.poses.key_expr().as_str() {
            return Err("IMU key collides with world publication".into());
        }
        if self.session.is_closed() {
            return Err("world session is closed".into());
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        let (receipt, ready) = mpsc::sync_channel(1);
        self.incoming
            .send_deadline(WorldWork::InstallImu(key, receipt), deadline)
            .map_err(|error| format!("IMU setup admission failed: {error}"))?;
        ready
            .recv_timeout(deadline.saturating_duration_since(Instant::now()))?
            .map_err(Into::into)
    }

    pub(super) fn lidar_output(&self, key: String) -> Result<Arc<LidarOutput>> {
        if key == self.clock.key_expr().as_str() || key == self.poses.key_expr().as_str() {
            return Err("lidar key collides with world publication".into());
        }
        if self.session.is_closed() {
            return Err("world session is closed".into());
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        let (receipt, ready) = mpsc::sync_channel(1);
        self.incoming
            .send_deadline(WorldWork::InstallLidar(key, receipt), deadline)
            .map_err(|error| format!("lidar setup admission failed: {error}"))?;
        ready
            .recv_timeout(deadline.saturating_duration_since(Instant::now()))?
            .map_err(Into::into)
    }

    pub(super) fn joint_controller_inputs(
        &self,
        setup: crate::joint_controller_io::Setup,
    ) -> std::result::Result<Vec<Arc<crate::joint_controller_io::Lease>>, crate::Error> {
        if setup.endpoints.iter().any(|endpoint| {
            endpoint.key == self.clock.key_expr().as_str()
                || endpoint.key == self.poses.key_expr().as_str()
                || endpoint.key == self.control_key
                || self
                    .removal_keys
                    .get()
                    .is_some_and(|keys| keys.contains(&endpoint.key))
        }) {
            return Err(crate::Error::JointControllerIdentity);
        }
        if self.session.is_closed() {
            return Err(crate::Error::WorldClosed);
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        let (receipt, ready) = mpsc::sync_channel(1);
        self.incoming
            .send_deadline(WorldWork::InstallJointControllers(setup, receipt), deadline)
            .map_err(|error| {
                crate::Error::WorldNetwork(format!(
                    "joint controller setup admission failed: {error}"
                ))
            })?;
        ready
            .recv_timeout(deadline.saturating_duration_since(Instant::now()))
            .map_err(|error| {
                crate::Error::WorldNetwork(format!(
                    "joint controller setup receipt failed: {error}"
                ))
            })?
            .map_err(crate::Error::WorldNetwork)
    }

    // Initial construction can wait here. Reset needs a separate readiness phase.
    pub(super) fn diff_drive_output(
        &self,
        setup: crate::diff_drive_io::Setup,
    ) -> std::result::Result<Arc<crate::diff_drive_io::Output>, crate::Error> {
        if [&setup.command_key, &setup.odometry_key]
            .into_iter()
            .any(|key| {
                key == self.clock.key_expr().as_str()
                    || key == self.poses.key_expr().as_str()
                    || key == &self.control_key
            })
        {
            return Err(crate::Error::DiffDriveIdentity);
        }
        if self.session.is_closed() {
            return Err(crate::Error::WorldClosed);
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        let (receipt, ready) = mpsc::sync_channel(1);
        self.incoming
            .send_deadline(WorldWork::InstallDrive(setup, receipt), deadline)
            .map_err(|error| {
                crate::Error::WorldNetwork(format!("drive setup admission failed: {error}"))
            })?;
        ready
            .recv_timeout(deadline.saturating_duration_since(Instant::now()))
            .map_err(|error| {
                crate::Error::WorldNetwork(format!("drive setup receipt failed: {error}"))
            })?
            .map_err(crate::Error::WorldNetwork)
    }

    pub(super) fn install_rates(
        &self,
        setup: crate::rates::RateSetup,
    ) -> std::result::Result<(), crate::Error> {
        if self.session.is_closed() {
            return Err(crate::Error::WorldClosed);
        }
        if setup.is_empty() {
            return Ok(());
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        let (receipt, ready) = mpsc::sync_channel(1);
        let admission = self
            .incoming
            .send_deadline(WorldWork::InstallRates(setup, receipt), deadline);
        if self.session.is_closed() {
            return Err(crate::Error::WorldClosed);
        }
        admission.map_err(|error| {
            crate::Error::WorldNetwork(format!("rates setup admission failed: {error}"))
        })?;
        let result = ready.recv_timeout(deadline.saturating_duration_since(Instant::now()));
        if self.session.is_closed() {
            return Err(crate::Error::WorldClosed);
        }
        result
            .map_err(|error| crate::Error::WorldNetwork(error.to_string()))?
            .map_err(crate::Error::WorldNetwork)
    }

    pub(super) fn install_forces(
        &self,
        setup: crate::force_io::ForceSetup,
    ) -> std::result::Result<(), crate::Error> {
        if self.session.is_closed() {
            return Err(crate::Error::WorldClosed);
        }
        if setup.is_empty() {
            return Ok(());
        }
        let deadline = Instant::now() + Duration::from_secs(5);
        let (receipt, ready) = mpsc::sync_channel(1);
        let admission = self
            .incoming
            .send_deadline(WorldWork::InstallForces(setup, receipt), deadline);
        if self.session.is_closed() {
            return Err(crate::Error::WorldClosed);
        }
        admission.map_err(|error| {
            crate::Error::WorldNetwork(format!("forces setup admission failed: {error}"))
        })?;
        let result = ready.recv_timeout(deadline.saturating_duration_since(Instant::now()));
        if self.session.is_closed() {
            return Err(crate::Error::WorldClosed);
        }
        result
            .map_err(|error| crate::Error::WorldNetwork(error.to_string()))?
            .map_err(crate::Error::WorldNetwork)
    }

    // Called only while preparing a run, outside ticking and stepping.
    pub(super) fn prepare_root_clock(
        &self,
        key: String,
        timeout: Duration,
    ) -> Result<Option<RootClock>> {
        if self.session.is_closed() || timeout.is_zero() {
            return Err("root clock preparation unavailable".into());
        }
        let (partition, topic) = key
            .strip_prefix('@')
            .and_then(|value| value.split_once('@'))
            .ok_or("invalid qualified root clock key")?;
        let selector = format!(
            "@gz/{}/*/*/*/MP/%/%/%/{}/*/%/%",
            partition.replace('/', "%"),
            topic.replace('/', "%")
        );
        let replies = self
            .session
            .liveliness()
            .get(selector)
            .timeout(timeout)
            .wait()?;
        if let Ok(reply) = replies.recv() {
            if reply.result().is_ok() {
                return Ok(None);
            }
            return Err("root clock discovery failed".into());
        }
        let publisher = Arc::new(
            self.session
                .declare_publisher(key.clone())
                .congestion_control(CongestionControl::Drop)
                .wait()?,
        );
        let token = self
            .session
            .liveliness()
            .declare_token(presence(
                &key,
                &self.process,
                &self.node,
                "MP",
                &Clock::full_name(),
            )?)
            .wait()?;
        Ok(Some(RootClock {
            publisher,
            _presence: token,
        }))
    }

    pub(super) fn publish_root_clock(&self, output: &RootClock, bytes: Vec<u8>) -> Result<()> {
        if bytes.len() > self.max_message_bytes || self.session.is_closed() {
            return Err("invalid root clock publication".into());
        }
        self.incoming
            .try_send(WorldWork::RootClock(
                self.epoch.load(Ordering::Acquire),
                Arc::clone(&output.publisher),
                bytes,
            ))
            .map_err(|_| "world publication queue unavailable".into())
    }

    pub(super) fn publish_clock(&self, bytes: Vec<u8>) -> Result<()> {
        if bytes.len() > self.max_message_bytes || self.session.is_closed() {
            return Err("world clock exceeds message limit".into());
        }
        self.incoming
            .try_send(WorldWork::Clock(self.epoch.load(Ordering::Acquire), bytes))
            .map_err(|_| "world publication queue unavailable".into())
    }

    pub(super) fn publish_poses(&self, bytes: Vec<u8>) -> Result<()> {
        if bytes.len() > self.max_message_bytes || self.session.is_closed() {
            return Err("invalid world pose publication".into());
        }
        self.incoming
            .try_send(WorldWork::Poses(self.epoch.load(Ordering::Acquire), bytes))
            .map_err(|_| "world publication queue unavailable".into())
    }

    pub(super) fn publish_imu(&self, output: &Arc<ImuOutput>, bytes: Vec<u8>) -> Result<()> {
        if bytes.len() > self.max_message_bytes || self.session.is_closed() {
            return Err("invalid IMU publication".into());
        }
        self.incoming
            .try_send(WorldWork::Imu(
                self.epoch.load(Ordering::Acquire),
                crate::imu_io::ImuJob::new(output, bytes),
            ))
            .map_err(|_| "world publication queue unavailable".into())
    }

    pub(super) fn publish_lidar(&self, output: &Arc<LidarOutput>, bytes: Vec<u8>) -> Result<()> {
        if bytes.len() > self.max_message_bytes || self.session.is_closed() {
            return Err("invalid lidar publication".into());
        }
        self.incoming
            .try_send(WorldWork::Lidar(
                self.epoch.load(Ordering::Acquire),
                crate::lidar_io::LidarJob::new(output, bytes),
            ))
            .map_err(|_| "world publication queue unavailable".into())
    }

    pub(super) fn publish_odometry(
        &self,
        output: &Arc<crate::diff_drive_io::Output>,
        bytes: Vec<u8>,
    ) -> Result<()> {
        if bytes.len() > self.max_message_bytes || self.session.is_closed() {
            return Err("invalid odometry publication".into());
        }
        self.incoming
            .try_send(WorldWork::Odometry(
                self.epoch.load(Ordering::Acquire),
                crate::diff_drive_io::Job::new(output, bytes),
            ))
            .map_err(|_| "world publication queue unavailable".into())
    }

    pub(super) fn clock_interest(&self) -> Result<bool> {
        if self.session.is_closed() {
            return Err("world session is closed".into());
        }
        // Stable Zenoh1.8 resolves this from its local routing table, not a peer wait.
        Ok(self.clock.matching_status().wait()?.matching())
    }
}

impl Drop for WorldIo {
    fn drop(&mut self) {
        self.shutdown();
    }
}

pub(super) fn presence(
    key: &str,
    process: &str,
    node: &str,
    kind: &str,
    types: &str,
) -> Result<String> {
    let (partition, topic) = key
        .strip_prefix('@')
        .and_then(|key| key.split_once('@'))
        .ok_or("invalid qualified world key")?;
    // Native token parsing requires nonempty path segments. The first profile
    // uses an explicit nonempty partition; empty-partition discovery is unqualified.
    if partition.is_empty() || !topic.starts_with('/') {
        return Err("unsupported world key".into());
    }
    Ok(format!(
        "@gz/{}/{process}/{node}/{node}/{kind}/%/%/%/{}/{types}/%/%",
        partition.replace('/', "%"),
        topic.replace('/', "%")
    ))
}

#[cfg(test)]
#[path = "../tests/private/world_io.rs"]
mod tests;
