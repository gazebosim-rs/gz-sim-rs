// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_proto::{
    msgs::{Altimeter, Boolean, Clock, Header, Imu, PoseV, WorldControl, WorldReset},
    prost::Message,
};
use gz_sim_server::gpu_control::{
    LinkImpulse, LinkImpulseAck,
    placement::{EnvironmentPlacement, EnvironmentPlacementAck, Target},
};
use std::{
    error::Error,
    fs::File,
    io::Write,
    path::Path,
    process::{Child, Command, Stdio},
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
        mpsc,
    },
    time::{Duration, Instant},
};
use zenoh::{Wait, pubsub::Subscriber, sample::Sample};
#[path = "gpu_sensor_plot.rs"]
mod plot;
pub type Result<T> = std::result::Result<T, Box<dyn Error + Send + Sync>>;
const SCENE: &str = include_str!("../../../../examples/gpu_sensor_world.sdf");
const TOPICS: [&str; 6] = [
    "/gpu/rolling/imu",
    "/gpu/free/altimeter",
    "/gpu/supported/imu",
    "/gpu/supported/altimeter",
    "/world/default/pose/info",
    "/world/default/clock",
];
const TYPES: [&str; 6] = [
    "gz.msgs.IMU",
    "gz.msgs.Altimeter",
    "gz.msgs.IMU",
    "gz.msgs.Altimeter",
    "gz.msgs.Pose_V",
    "gz.msgs.Clock",
];
const PERIODS: [u64; 4] = [10, 20, 5, 8];
const TIMEOUT: Duration = Duration::from_secs(8);

struct Process(Child);
impl Drop for Process {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}
struct Wire {
    channel: usize,
    payload: Vec<u8>,
    label: Vec<u8>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Gpu,
    ScalarPlacement,
    NativeHandoff,
    NativeSupport,
}
struct Probe {
    mode: Mode,
    session: zenoh::Session,
    key_prefix: String,
    packets: mpsc::Receiver<Wire>,
    _subscriptions: Vec<Subscriber<()>>,
    overflow: Arc<AtomicUsize>,
    free_link: Option<u64>,
    plane_link: Option<u64>,
    supported_link: Option<u64>,
    clock_ms: u64,
    readings: [Vec<(u64, Vec<u8>)>; 4],
    sequences: [Option<u64>; 4],
    epoch: u64,
    csv: File,
}
fn stamp(h: &Header) -> Result<u64> {
    let t = h.stamp.as_ref().ok_or("missing stamp")?;
    if t.sec < 0 || !(0..1_000_000_000).contains(&t.nsec) || t.nsec % 1_000_000 != 0 {
        return Err("invalid millisecond sensor timestamp".into());
    }
    Ok(t.sec as u64 * 1000 + t.nsec as u64 / 1_000_000)
}
fn close(a: f64, b: f64, tolerance: f64) -> Result<()> {
    if (a - b).abs() > tolerance || !a.is_finite() {
        return Err(format!("observed {a}, expected {b} ± {tolerance}").into());
    }
    Ok(())
}
impl Probe {
    fn new(session: zenoh::Session, partition: &str, csv: File, mode: Mode) -> Result<Self> {
        let key_prefix = format!("@/{partition}@");
        let (tx, packets) = mpsc::sync_channel(1024);
        let overflow = Arc::new(AtomicUsize::new(0));
        let mut subscriptions = vec![];
        for (channel, topic) in TOPICS.iter().enumerate() {
            let tx = tx.clone();
            let overflow = overflow.clone();
            subscriptions.push(
                session
                    .declare_subscriber(format!("{key_prefix}{topic}"))
                    .callback(move |sample: Sample| {
                        let label = sample
                            .attachment()
                            .map(|s| s.to_bytes().into_owned())
                            .unwrap_or_default();
                        let payload = sample.payload().to_bytes().into_owned();
                        if tx
                            .try_send(Wire {
                                channel,
                                payload,
                                label,
                            })
                            .is_err()
                        {
                            overflow.fetch_add(1, Ordering::Relaxed);
                        }
                    })
                    .wait()?,
            );
        }
        Ok(Self {
            mode,
            session,
            key_prefix,
            packets,
            _subscriptions: subscriptions,
            overflow,
            free_link: None,
            plane_link: None,
            supported_link: None,
            clock_ms: 0,
            readings: Default::default(),
            sequences: [None; 4],
            epoch: 0,
            csv,
        })
    }
    fn rpc<M: Message + Default>(&self, topic: &str, payload: Vec<u8>) -> Result<M> {
        let key = format!("{}{topic}", self.key_prefix);
        let replies = self
            .session
            .get(&key)
            .payload(payload)
            .timeout(TIMEOUT)
            .wait()?;
        let sample = replies
            .recv_timeout(TIMEOUT)?
            .ok_or("missing service response")?
            .into_result()
            .map_err(|_| "service error")?;
        if sample.key_expr().as_str() != key {
            return Err("response key mismatch".into());
        }
        Ok(M::decode(sample.payload().to_bytes().as_ref())?)
    }
    fn control(&self, control: WorldControl) -> Result<()> {
        if !self
            .rpc::<Boolean>("/world/default/control", control.encode_to_vec())?
            .data
        {
            return Err("world control refused".into());
        }
        Ok(())
    }
    fn steps(&self, n: u32) -> Result<()> {
        self.control(WorldControl {
            multi_step: n,
            pause: true,
            ..Default::default()
        })
    }
    fn impulse(&self, request: &LinkImpulse, expected: bool, duplicate: bool) -> Result<()> {
        let ack =
            self.rpc::<LinkImpulseAck>("/world/default/gpu/link_impulse", request.encode_to_vec())?;
        if ack.accepted != expected
            || ack.duplicate != duplicate
            || expected && (Some(ack.epoch) != request.epoch || Some(ack.tick) != request.tick)
            || !expected && ack.reason.is_empty()
        {
            return Err(format!("unexpected impulse receipt for {request:?}: {ack:?}").into());
        }
        println!(
            "impulse link={} epoch={:?} tick={:?} accepted={} duplicate={} reason={:?}",
            request.link_id, request.epoch, request.tick, ack.accepted, ack.duplicate, ack.reason
        );
        Ok(())
    }
    fn wait(&mut self, ready: impl Fn(&Self) -> bool) -> Result<()> {
        let deadline = Instant::now() + TIMEOUT;
        while !ready(self) {
            let wire=self.packets.recv_timeout(deadline.saturating_duration_since(Instant::now())).map_err(|_|format!("timed out waiting for sample, epoch={}, latest clock={} ms, sensor counts={:?}, link IDs={:?}",self.epoch,self.clock_ms,self.readings.each_ref().map(Vec::len),[self.free_link,self.supported_link]))?;
            if wire.label != TYPES[wire.channel].as_bytes() {
                return Err(format!(
                    "unexpected Gazebo message attachment {:?}, expected {}",
                    String::from_utf8_lossy(&wire.label),
                    TYPES[wire.channel]
                )
                .into());
            }
            match wire.channel {
                4 => {
                    for pose in PoseV::decode(&wire.payload[..])?.pose {
                        match pose.name.as_str() {
                            "action_plane" => self.plane_link = Some(pose.id as u64),
                            "action_free" => self.free_link = Some(pose.id as u64),
                            "action_supported" => self.supported_link = Some(pose.id as u64),
                            _ => {}
                        }
                    }
                }
                5 => {
                    let t = Clock::decode(&wire.payload[..])?
                        .sim
                        .ok_or("clock missing sim")?;
                    self.clock_ms = t.sec as u64 * 1000 + t.nsec as u64 / 1_000_000;
                }
                i => self.sensor(i, &wire.payload)?,
            }
        }
        Ok(())
    }
    fn metadata(&mut self, i: usize, header: &Header) -> Result<(u64, u64)> {
        let tick = stamp(header)?;
        let next = if let Some((previous, _)) = self.readings[i].last() {
            (previous / PERIODS[i] + 1) * PERIODS[i]
        } else {
            1
        };
        if tick != next {
            return Err(format!("{} timestamp {tick}, expected {next}", TOPICS[i]).into());
        }
        let seq = header
            .data
            .iter()
            .find(|p| p.key == "seq")
            .and_then(|p| p.value.first())
            .ok_or("sensor header has no seq")?
            .parse::<u64>()?;
        let expected = self.sequences[i].map_or(0, |last| last + 1);
        if seq != expected {
            return Err(format!("{} sequence {seq}, expected {expected}", TOPICS[i]).into());
        }
        self.sequences[i] = Some(seq);
        if !header
            .data
            .iter()
            .any(|m| m.key == "frame_id" && m.value.len() == 1 && !m.value[0].is_empty())
        {
            return Err("missing frame identity".into());
        }
        Ok((tick, seq))
    }
    fn sensor(&mut self, i: usize, payload: &[u8]) -> Result<()> {
        let (tick, seq, values, normalized) = if i == 0 || i == 2 {
            let s = Imu::decode(payload)?;
            let (tick, seq) = self.metadata(i, s.header.as_ref().ok_or("IMU header")?)?;
            let a = s.linear_acceleration.as_ref().ok_or("IMU acceleration")?;
            let w = s.angular_velocity.as_ref().ok_or("IMU angular velocity")?;
            let magnitude = a.x.hypot(a.y);
            let expected = if i == 2 && tick == 5 { 100. } else { 0. };
            close(magnitude, expected, 4e-10)?;
            if i == 0 && self.mode == Mode::NativeSupport {
                let vertical = match tick {
                    1 | 10 => 1.2,
                    20 => 100.,
                    _ => 0.,
                };
                close(a.z, vertical, 2e-9)?;
                close(w.z, 0., 2e-12)?;
            } else {
                close(a.z, if i == 0 { 0. } else { 1.2 }, 5e-10)?;
                close(w.z, if i == 0 { 2.2 } else { -1.6 }, 2e-12)?;
            }
            (tick, seq, [a.x, a.y, a.z, w.z], s.encode_to_vec())
        } else {
            let s = Altimeter::decode(payload)?;
            let (tick, seq) = self.metadata(i, s.header.as_ref().ok_or("altimeter header")?)?;
            let t = tick as f64 * 0.001;
            if i == 1 {
                let offset = glam::DVec3::new(0.07, 0.02, 0.03);
                let (omega, r) = if self.mode == Mode::NativeHandoff {
                    (glam::DVec3::ZERO, offset)
                } else {
                    let omega = glam::DVec3::new(0.4, 1.8, 1.);
                    let angle = 2. * (omega.length() * 0.0005).atan() * tick as f64;
                    (
                        omega,
                        glam::DQuat::from_axis_angle(omega.normalize(), angle) * offset,
                    )
                };
                // Only steps 2–4 use the 16-substep native solver, with four
                // velocity integrations per substep. The excess displacement
                // over ballistic motion is g h² / (2 × 16 × 4) per public tick.
                let native_offset = if self.mode == Mode::NativeHandoff {
                    -1.2e-6 * tick.saturating_sub(1).min(3) as f64 / 128.
                } else {
                    0.
                };
                let vz = if tick >= 2 { 0.2 } else { 0. };
                close(
                    s.vertical_position,
                    -0.6 * t * t + vz * (t - 0.001) + r.z - 0.03 + native_offset,
                    3e-10,
                )?;
                close(s.vertical_velocity, -1.2 * t + vz + omega.cross(r).z, 3e-10)?;
            } else {
                close(s.vertical_position, 0., 1e-14)?;
                close(s.vertical_velocity, 0., 1e-14)?;
            }
            close(
                s.vertical_reference,
                if i == 1 { 0.85 } else { 0.28 },
                1e-14,
            )?;
            (
                tick,
                seq,
                [
                    s.vertical_position,
                    s.vertical_velocity,
                    s.vertical_reference,
                    0.,
                ],
                s.encode_to_vec(),
            )
        };
        writeln!(
            &mut self.csv,
            "{},{},{tick},{seq},{},{},{},{}",
            self.epoch, TOPICS[i], values[0], values[1], values[2], values[3]
        )?;
        if normalized != payload {
            return Err("sensor payload differs from its canonical prost encoding".into());
        }
        self.readings[i].push((tick, payload.to_vec()));
        Ok(())
    }
    fn place(&self, request: &EnvironmentPlacement, expected: Option<(u64, u64)>) -> Result<()> {
        let ack = self.rpc::<EnvironmentPlacementAck>(
            "/world/default/gpu/placement",
            request.encode_to_vec(),
        )?;
        if ack.accepted != expected.is_some()
            || expected.is_some_and(|(moved, cpu)| {
                ack.moved != moved
                    || ack.cpu != cpu
                    || ack.gpu + cpu != 3
                    || Some(ack.tick) != request.tick
                    || Some(ack.epoch) != request.epoch
            })
            || !ack.accepted && ack.reason.is_empty()
        {
            return Err(format!("unexpected placement receipt for {request:?}: {ack:?}").into());
        }
        println!(
            "placement epoch={} tick={} ids={:?} accepted={} moved={} cpu={} gpu={} reseed={} reason={:?}",
            ack.epoch,
            ack.tick,
            request.environments,
            ack.accepted,
            ack.moved,
            ack.cpu,
            ack.gpu,
            ack.reseeded_gpu_rows,
            ack.reason
        );
        Ok(())
    }
    fn episode(&mut self) -> Result<()> {
        let place = self.mode == Mode::ScalarPlacement;
        let route = |epoch, tick, ids: &[u64], target: Target| EnvironmentPlacement {
            epoch: Some(epoch),
            tick: Some(tick),
            environments: ids.to_vec(),
            target: target as i32,
        };
        if place {
            let original = route(self.epoch, 0, &[9, 31, 80], Target::Cpu);
            self.place(&original, Some((0, 3)))?;
            for (payload, reason) in [
                (vec![255], "malformed placement protobuf"),
                (
                    EnvironmentPlacement {
                        epoch: None,
                        ..original.clone()
                    }
                    .encode_to_vec(),
                    "missing epoch",
                ),
                (
                    EnvironmentPlacement {
                        tick: None,
                        ..original.clone()
                    }
                    .encode_to_vec(),
                    "missing tick",
                ),
                (
                    EnvironmentPlacement {
                        environments: vec![9; 257],
                        ..original.clone()
                    }
                    .encode_to_vec(),
                    "expected between 1 and 256 stable environment IDs",
                ),
            ] {
                let ack =
                    self.rpc::<EnvironmentPlacementAck>("/world/default/gpu/placement", payload)?;
                if ack.accepted
                    || ack.moved != 0
                    || ack.cpu != 3
                    || ack.gpu != 0
                    || ack.reason != reason
                    || ack.epoch != self.epoch
                    || ack.tick != 0
                {
                    return Err(format!("invalid placement should preserve the boundary and report {reason:?}, received {ack:?}").into());
                }
            }
            for refused in [
                route(self.epoch, 0, &[], Target::Cpu),
                route(self.epoch, 0, &[9, 9], Target::Gpu),
                route(self.epoch, 0, &[9, 777], Target::Gpu),
                route(self.epoch, 0, &[9], Target::Unspecified),
                route(self.epoch, 1, &[9], Target::Gpu),
            ] {
                self.place(&refused, None)?;
            }
            self.place(&route(self.epoch, 0, &[9], Target::Gpu), Some((1, 2)))?;
        }
        self.steps(1)?;
        self.wait(|p| p.clock_ms == 1 && p.readings.iter().all(|r| r.len() == 1))?;
        let first = LinkImpulse {
            epoch: Some(self.epoch),
            tick: Some(1),
            link_id: self.free_link.unwrap(),
            z: 0.17,
            ..Default::default()
        };
        let invalid = self.rpc::<LinkImpulseAck>("/world/default/gpu/link_impulse", vec![255])?;
        if invalid.accepted || !invalid.reason.contains("malformed") {
            return Err("malformed protobuf was accepted or lacked diagnostics".into());
        }
        for request in [
            LinkImpulse {
                tick: None,
                ..first.clone()
            },
            LinkImpulse {
                epoch: None,
                ..first.clone()
            },
            LinkImpulse {
                tick: Some(0),
                ..first.clone()
            },
        ] {
            self.impulse(&request, false, false)?;
        }
        self.impulse(&first, true, false)?;
        if self.mode == Mode::NativeHandoff {
            self.place(
                &route(self.epoch, 1, &[31], Target::NativeRapier),
                Some((1, 1)),
            )?;
            self.place(
                &route(self.epoch, 1, &[31], Target::NativeRapier),
                Some((0, 1)),
            )?;
            self.place(&route(self.epoch, 1, &[31], Target::Gpu), None)?;
        }
        if place {
            self.place(&route(self.epoch, 1, &[31], Target::Gpu), Some((1, 1)))?;
            self.place(&route(self.epoch, 1, &[31], Target::Gpu), Some((0, 1)))?;
        }
        self.impulse(&first, true, true)?;
        self.impulse(
            &LinkImpulse {
                z: 0.18,
                ..first.clone()
            },
            false,
            false,
        )?;
        self.impulse(
            &LinkImpulse {
                link_id: u64::MAX,
                ..first.clone()
            },
            false,
            false,
        )?;
        self.steps(3)?;
        self.wait(|p| p.clock_ms == 4)?;
        let support = LinkImpulse {
            epoch: Some(self.epoch),
            tick: Some(4),
            link_id: self.supported_link.unwrap(),
            y: 0.14,
            ..Default::default()
        };
        self.impulse(
            &LinkImpulse {
                z: 0.3,
                ..support.clone()
            },
            false,
            false,
        )?;
        self.impulse(&support, true, false)?;
        if self.mode == Mode::NativeHandoff {
            self.place(
                &route(self.epoch, 4, &[31], Target::GuardedGpu),
                Some((1, 0)),
            )?;
            self.place(&route(self.epoch, 4, &[31, 80], Target::NativeRapier), None)?;
        }
        if place {
            self.place(&route(self.epoch, 4, &[80], Target::Gpu), Some((1, 0)))?;
            self.place(&route(self.epoch, 4, &[9, 31], Target::Cpu), Some((2, 2)))?;
        }

        if self.mode == Mode::NativeSupport {
            self.steps(15)?;
            self.wait(|p| p.clock_ms == 19)?;
            let release = route(self.epoch, 19, &[9], Target::GuardedGpu);
            let lifted = LinkImpulse {
                epoch: Some(self.epoch),
                tick: Some(19),
                link_id: self.plane_link.ok_or("unresolved sphere-plane link")?,
                z: 0.11,
                ..Default::default()
            };
            self.place(&release, None)?;
            self.impulse(&lifted, true, false)?;
            self.impulse(&lifted, true, true)?;
            let locked = self.rpc::<EnvironmentPlacementAck>(
                "/world/default/gpu/placement",
                release.encode_to_vec(),
            )?;
            if locked.accepted
                || locked.cpu != 1
                || locked.gpu != 2
                || locked.moved != 0
                || !locked.reason.contains("contact history")
            {
                return Err(format!("unexpected loaded support receipt {locked:?}").into());
            }
            println!(
                "native support refused with pending external liftoff at 19 ms: {}",
                locked.reason
            );
            self.steps(1)?;
            self.wait(|p| p.clock_ms == 20)?;
            self.place(
                &route(self.epoch, 20, &[9], Target::GuardedGpu),
                Some((1, 0)),
            )?;
            self.steps(20)?;
        } else {
            self.steps(36)?;
        }
        self.wait(|p| {
            p.clock_ms == 40
                && p.readings
                    .iter()
                    .all(|r| r.last().is_some_and(|x| x.0 == 40))
        })?;
        self.impulse(&first, true, true)?;
        if place {
            self.place(&route(self.epoch, 40, &[80], Target::Cpu), Some((1, 3)))?;
        }
        Ok(())
    }
}
pub fn run(exe: &Path, adapter: &str, directory: &Path, mode: Mode) -> Result<()> {
    std::fs::create_dir_all(directory)?;
    let mut csv = File::create_new(directory.join("observations.csv"))?;
    writeln!(
        csv,
        "epoch,topic,tick_ms,publisher_sequence,x_or_altitude,y_or_vertical_velocity,z_or_vertical_reference,angular_z"
    )?;
    let config = zenoh::Config::from_json5(
        r#"{mode:"peer",listen:{endpoints:["tcp/127.0.0.1:0"]},scouting:{multicast:{enabled:false},gossip:{enabled:false}},transport:{shared_memory:{enabled:false}}}"#,
    )?;
    let session = zenoh::open(config).wait()?;
    let endpoints: Vec<_> = session
        .info()
        .locators()
        .wait()
        .into_iter()
        .map(|l| l.to_string())
        .collect();
    let scratch = tempfile::tempdir()?;
    let partition = format!(
        "gpu-sdf-{}-{}",
        std::process::id(),
        scratch.path().file_name().unwrap().to_string_lossy()
    );
    let server = scratch.path().join("zenoh.json");
    let sdf = scratch.path().join("world.sdf");
    let scene = if mode == Mode::NativeHandoff {
        SCENE
            .replace("id=\"31\"", "id=\"31\" native_handoff=\"true\"")
            .replace("angular=\"0.4 1.8 1\"", "angular=\"0 0 0\"")
    } else if mode == Mode::NativeSupport {
        SCENE
            .replace(
                "id=\"9\"",
                "id=\"9\" native_handoff=\"true\" execution=\"native\"",
            )
            .replace("0 1.78 0.22 0.12 0 0", "0 1.78 0.22 0 0 0")
            .replace(
                "0 2.1 0.65 0 0 0</pose>\n      <link name=\"body\"",
                "0 2.1 0.36 0 0 0</pose>\n      <link name=\"action_plane\"",
            )
            .replace("rolling::body", "rolling::action_plane")
            .replace(
                "linear=\"0 -0.72 -0.4\" angular=\"0 0 2.2\"",
                "linear=\"0 0 0\" angular=\"0 0 0\"",
            )
    } else if mode == Mode::ScalarPlacement {
        SCENE.replace(" profile=", " execution=\"cpu\" profile=")
    } else {
        SCENE.to_owned()
    };
    std::fs::write(&sdf, scene)?;
    std::fs::write(
        &server,
        serde_json::to_vec(
            &serde_json::json!({"mode":"client","listen":{"endpoints":[]},"connect":{"endpoints":endpoints,"timeout_ms":2000},"scouting":{"multicast":{"enabled":false},"gossip":{"enabled":false}},"transport":{"shared_memory":{"enabled":false}}}),
        )?,
    )?;
    let mut probe = Probe::new(session, &partition, csv, mode)?;
    let output = File::create_new(directory.join("server.log"))?;
    let mut child = Process(
        Command::new(exe)
            .arg(sdf)
            .args([
                "--physics-engine",
                "gpu-batch",
                "--gpu-adapter",
                adapter,
                "--server",
                "--iterations",
                "81",
            ])
            .env("ZENOH_CONFIG", server)
            .env("GZ_PARTITION", &partition)
            .stdout(output.try_clone()?)
            .stderr(output)
            .stdin(Stdio::null())
            .spawn()?,
    );
    probe.wait(|p| {
        p.free_link.is_some()
            && p.supported_link.is_some()
            && (mode != Mode::NativeSupport || p.plane_link.is_some())
    })?;
    println!(
        "resolved ordinary Gazebo links {:?}; starting paused",
        [probe.free_link, probe.supported_link]
    );
    probe.episode()?;
    // A pending command is intentionally discarded by reset before its step.
    probe.impulse(
        &LinkImpulse {
            epoch: Some(0),
            tick: Some(40),
            link_id: probe.free_link.unwrap(),
            z: 0.85,
            ..Default::default()
        },
        true,
        false,
    )?;
    let first = std::mem::take(&mut probe.readings);
    probe.control(WorldControl {
        pause: true,
        reset: Some(WorldReset {
            all: true,
            ..Default::default()
        }),
        ..Default::default()
    })?;
    probe.epoch = 1;
    probe.sequences = [None; 4];
    probe.wait(|p| p.clock_ms == 0)?;
    probe.impulse(
        &LinkImpulse {
            epoch: Some(0),
            tick: Some(40),
            link_id: probe.free_link.unwrap(),
            z: 0.85,
            ..Default::default()
        },
        false,
        false,
    )?;
    probe.episode()?;
    if first != probe.readings {
        return Err("sensor protobuf replay differs".into());
    }
    if probe.overflow.load(Ordering::Relaxed) != 0 {
        return Err("external sensor probe overflowed".into());
    }
    println!(
        "verified two epochs, sensor samples per epoch {:?}, identical sensor protobufs including native timestamps, frame IDs and publication sequences",
        first.each_ref().map(Vec::len)
    );
    probe.steps(1)?;
    let end = Instant::now() + TIMEOUT;
    loop {
        if let Some(status) = child.0.try_wait()? {
            if !status.success() {
                return Err(format!("server exited {status}").into());
            }
            break;
        }
        if Instant::now() > end {
            return Err("CLI failed to exit after its configured 81 exact steps".into());
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    plot::draw(directory, &first)?;
    Ok(())
}
