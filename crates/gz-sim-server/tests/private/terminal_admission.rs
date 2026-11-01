// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Baseline characterization and prepared terminal-admission regressions.
use super::*;
use gz_sim_proto::msgs::WorldReset;
use serde_json::{Value, json};

#[derive(Clone, Copy, Debug)]
enum Case {
    Unpaused,
    Paused,
    PendingResetImu,
    PendingResetWithoutImu,
    PreparedReset,
}
const CASES: [Case; 5] = [
    Case::Unpaused,
    Case::Paused,
    Case::PendingResetImu,
    Case::PendingResetWithoutImu,
    Case::PreparedReset,
];

#[derive(Clone, Copy, Debug)]
enum Api {
    Tick,
    Update,
}
impl Api {
    fn call(self, server: &mut Server) -> Result<UpdateInfo, Error> {
        match self {
            Self::Tick => server.tick(),
            Self::Update => server.update(),
        }
    }
}

fn control_key(server: &Server) -> String {
    crate::options::qualified_name(
        &server.partition,
        &server.namespace,
        "/world/default/control",
    )
    .unwrap()
}

fn accept_control(server: &mut Server, control: WorldControl) {
    let payload = control.encode_to_vec();
    let replies = server
        .world_io
        .session
        .get(control_key(server))
        .payload(payload.clone())
        .allowed_destination(Locality::SessionLocal)
        .timeout(Duration::from_secs(2))
        .wait()
        .unwrap();
    let request = server
        .world_io
        .controls
        .recv_timeout(Duration::from_secs(2))
        .unwrap();
    assert_eq!(request.payload, payload);
    server.respond(request);
    let reply = replies
        .recv_timeout(Duration::from_secs(2))
        .unwrap()
        .unwrap();
    assert!(
        Boolean::decode(reply.result().unwrap().payload().to_bytes().as_ref())
            .unwrap()
            .data
    );
}

struct Fixture {
    server: Server,
    step_witness: Arc<AtomicU64>,
    reset_witness: Arc<AtomicU64>,
    observers: Vec<SampleReceiver>,
    queued_control: Vec<u8>,
}
impl Fixture {
    fn closed(case: Case, explicit_shutdown: bool) -> Self {
        let mut xml = include_str!("../../../gz-sim-scene/tests/fixtures/fall.sdf").to_owned();
        if matches!(case, Case::PendingResetWithoutImu) {
            let start = xml.find("<sensor ").unwrap();
            let end = start + xml[start..].find("</sensor>").unwrap() + "</sensor>".len();
            xml.replace_range(start..end, "");
        }
        let mut sim = gz_sim_scene::load_world_with_engine(
            &xml,
            "terminal.sdf",
            "default",
            Duration::from_millis(1),
            &sdformat::NoResources,
            gz_sim_scene::PhysicsEngine::Tpe,
        )
        .unwrap();
        let body = sim
            .world()
            .iter_entities()
            .find(|e| {
                e.get::<gz_sim_scene::Name>()
                    .is_some_and(|n| n.0 == "default::ball::body")
            })
            .unwrap()
            .get::<gz_sim_core::EntityId>()
            .unwrap()
            .get();
        gz_sim_scene::queue_link_world_velocity(&mut sim, body, [1., 0., 0.], [0.; 3]).unwrap();
        let step_witness = Arc::new(AtomicU64::new(0));
        let reset_witness = Arc::new(AtomicU64::new(0));
        let steps = Arc::clone(&step_witness);
        sim.add_systems(gz_sim_core::Phase::Finalize, move || {
            steps.fetch_add(1, Ordering::Relaxed);
        });
        let resets = Arc::clone(&reset_witness);
        sim.add_systems(gz_sim_core::Phase::Reset, move || {
            resets.fetch_add(1, Ordering::Relaxed);
        });
        let mut server =
            isolated_server(sim, "default", config(), NonZeroUsize::new(8).unwrap()).unwrap();
        server.set_interest_policy(InterestPolicy::Always);
        server.set_pose_publication_period(Duration::ZERO);
        let mut observers = vec![
            samples(&server, "/world/default/clock", "gz.msgs.Clock"),
            samples(&server, "/world/default/pose/info", "gz.msgs.Pose_V"),
        ];
        if let Some(stream) = server.imus.first() {
            observers.push(samples(&server, &stream.metadata.topic, "gz.msgs.IMU"));
        }
        accept_control(
            &mut server,
            WorldControl {
                pause: false,
                ..Default::default()
            },
        );
        for _ in 0..2 {
            server.update().unwrap();
            for reader in &observers {
                reader.recv_timeout(Duration::from_secs(2)).unwrap();
            }
        }
        match case {
            Case::Unpaused => accept_control(
                &mut server,
                WorldControl {
                    pause: true,
                    multi_step: 3,
                    ..Default::default()
                },
            ),
            Case::Paused => accept_control(
                &mut server,
                WorldControl {
                    pause: true,
                    ..Default::default()
                },
            ),
            Case::PendingResetImu | Case::PendingResetWithoutImu | Case::PreparedReset => {
                accept_control(
                    &mut server,
                    WorldControl {
                        pause: true,
                        multi_step: 2,
                        reset: Some(WorldReset {
                            all: true,
                            ..Default::default()
                        }),
                        ..Default::default()
                    },
                );
                if matches!(case, Case::PreparedReset) {
                    server.prepare_io().unwrap();
                }
            }
        }
        assert_eq!(
            server.pending_steps(),
            match case {
                Case::Unpaused => 3,
                Case::Paused => 0,
                _ => 2,
            }
        );
        // Retain a genuine IO-admitted plain envelope in an inspectable bounded CPU
        // queue. No Query/native handle crosses that boundary. The terminal calls
        // must not consume this unread command or replace its pending-step budget.
        let queued_control = WorldControl {
            pause: false,
            multi_step: 9,
            ..Default::default()
        }
        .encode_to_vec();
        let replies = server
            .world_io
            .session
            .get(control_key(&server))
            .payload(queued_control.clone())
            .allowed_destination(Locality::SessionLocal)
            .timeout(Duration::from_secs(2))
            .wait()
            .unwrap();
        let request = server
            .world_io
            .controls
            .recv_timeout(Duration::from_secs(2))
            .unwrap();
        assert_eq!(request.payload, queued_control);
        let (send, input) = mpsc::sync_channel(1);
        server.world_io.controls = input;
        assert!(send.try_send(request).is_ok());
        drop(send);
        if explicit_shutdown {
            server.shutdown();
        } else {
            server.set_interest_policy(InterestPolicy::Subscribers);
            server.world_io.session.close().wait().unwrap();
        }
        assert!(replies.recv_timeout(Duration::from_secs(2)).is_err());
        assert!(server.world_io.session.is_closed());
        if explicit_shutdown {
            assert!(server.imus.is_empty());
            assert_eq!(server.imus.capacity(), 0);
        }
        Self {
            server,
            step_witness,
            reset_witness,
            observers,
            queued_control,
        }
    }

    fn snapshot(&self) -> Value {
        let server = &self.server;
        let info = server.simulation.world().resource::<UpdateInfo>();
        let mut entities = server.simulation.world().iter_entities().map(|e| {
            let value = json!({
                "ecs": format!("{:?}", e.id()),
                "id": e.get::<gz_sim_core::EntityId>().map(|id| id.get()),
                "name": e.get::<gz_sim_scene::Name>().map(|name| &name.0),
                "pose": e.get::<gz_sim_scene::WorldPose>().map(|p| json!({"translation":p.translation,"rotation":p.rotation})),
                "velocity": e.get::<gz_sim_scene::Velocity>().map(|v| json!({"linear":v.linear,"angular":v.angular})),
                "sample": e.get::<ImuSample>().map(|sample| json!({"time_ns":sample.time.as_nanos().to_string(),"epoch":sample.epoch,"reading":format!("{:?}",sample.reading)})),
            });
            (format!("{:?}", e.id()), value)
        }).collect::<Vec<_>>();
        entities.sort_by(|a, b| a.0.cmp(&b.0));
        json!({
            "elapsed_ns":server.simulation.elapsed().as_nanos().to_string(),
            "update_info":format!("{info:?}"),"epoch":info.epoch,"iteration":info.iteration,
            "paused":server.simulation.is_paused(),"entities":entities.into_iter().map(|(_,v)|v).collect::<Vec<_>>(),
            "pending_steps":server.pending_steps,"pending_reset":server.pending_reset,
            "prepared_reset":server.prepared_reset.map(|info|format!("{info:?}")),"reset_failed":server.reset_failed,
            "counters":format!("{:?}",server.counters()),
            "controls_accepted":server.counters().controls_accepted,"controls_rejected":server.counters().controls_rejected,
            "clock_failures":server.counters().clock_failures,"pose_failures":server.counters().pose_failures,
            "imu_streams":server.imus.iter().map(|stream|json!({"entity":format!("{:?}",stream.entity),"sequence":stream.sequence,"last_sample":format!("{:?}",stream.last_sample)})).collect::<Vec<_>>(),
            "step_witness":self.step_witness.load(Ordering::Relaxed),"reset_witness":self.reset_witness.load(Ordering::Relaxed),
            "session_closed":server.world_io.session.is_closed(),
        })
    }

    fn unread_control_survives(&self) {
        let request = self.server.world_io.controls.try_recv().unwrap();
        assert_eq!(request.payload, self.queued_control);
        assert!(matches!(
            self.server.world_io.controls.try_recv(),
            Err(mpsc::TryRecvError::Disconnected)
        ));
        for reader in &self.observers {
            assert!(reader.incoming.try_recv().is_err());
        }
    }
}

fn exercise(case: Case, api: Api) -> Value {
    let mut fixture = Fixture::closed(case, true);
    let before = fixture.snapshot();
    let mut observations = Vec::new();
    for attempt in 0..3 {
        let result = api.call(&mut fixture.server);
        assert!(
            result.is_err(),
            "closed application unexpectedly accepted a frame"
        );
        observations.push(json!({"attempt":attempt,"error":format!("{:?}",result.unwrap_err()),"after":fixture.snapshot()}));
    }
    fixture.unread_control_survives();
    json!({"case":format!("{case:?}"),"api":format!("{api:?}"),"before":before,"observations":observations})
}

#[test]
fn characterize_explicit_shutdown_frame_admission() {
    for case in CASES {
        for api in [Api::Tick, Api::Update] {
            println!("TERMINAL_BASELINE {}", exercise(case, api));
        }
    }
}

#[test]
fn direct_session_close_keeps_the_retained_advance_and_counter_semantics() {
    let mut fixture = Fixture::closed(Case::Unpaused, false);
    let before = fixture.snapshot();
    for _ in 0..3 {
        assert!(matches!(fixture.server.update(), Err(Error::WorldClosed)));
    }
    let after = fixture.snapshot();
    assert_eq!(
        fixture.server.simulation.elapsed(),
        Duration::from_millis(5)
    );
    assert_eq!(
        after["iteration"].as_u64().unwrap(),
        before["iteration"].as_u64().unwrap() + 3
    );
    assert_eq!(
        after["step_witness"].as_u64().unwrap(),
        before["step_witness"].as_u64().unwrap() + 3
    );
    assert_eq!(fixture.server.pending_steps, 0);
    assert_eq!(fixture.server.counters().clock_failures, 3);
    assert_eq!(fixture.server.counters().interest_failures, 6);
    assert_eq!(after["controls_accepted"], before["controls_accepted"]);
    fixture.unread_control_survives();
    println!(
        "DIRECT_SESSION_BASELINE {}",
        json!({"before":before,"after":after})
    );
    fixture.server.shutdown();
}

fn assert_terminal(case: Case) {
    let observations = [Api::Tick, Api::Update].map(|api| exercise(case, api));
    for result in &observations {
        println!("PREPARED_TERMINAL {}", result);
    }
    for result in observations {
        for observation in result["observations"].as_array().unwrap() {
            assert_eq!(observation["error"], "WorldClosed", "{result}");
            assert_eq!(
                observation["after"], result["before"],
                "closed application changed state: {result}"
            );
        }
    }
}

#[test]
fn explicit_shutdown_unpaused_frames_require_terminal_admission() {
    assert_terminal(Case::Unpaused);
}
#[test]
fn explicit_shutdown_paused_frames_require_terminal_admission() {
    assert_terminal(Case::Paused);
}
#[test]
fn explicit_shutdown_pending_imu_reset_requires_terminal_admission() {
    assert_terminal(Case::PendingResetImu);
}
#[test]
fn explicit_shutdown_pending_empty_reset_requires_terminal_admission() {
    assert_terminal(Case::PendingResetWithoutImu);
}
#[test]
fn explicit_shutdown_prepared_reset_requires_terminal_admission() {
    assert_terminal(Case::PreparedReset);
}
