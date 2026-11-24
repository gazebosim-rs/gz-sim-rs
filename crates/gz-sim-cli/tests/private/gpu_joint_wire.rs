// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use gz_sim_proto::msgs::{Axis, Boolean, Header, Joint, WorldControl};
fn time(t: i32) -> Time {
    Time {
        sec: 0,
        nsec: t * 1_000_000,
    }
}
fn state(t: i32) -> Model {
    Model {
        header: Some(Header {
            stamp: Some(time(t)),
            ..Default::default()
        }),
        joint: ["shoulder", "elbow"]
            .into_iter()
            .map(|name| Joint {
                name: name.into(),
                axis1: Some(Axis {
                    position: 0.5,
                    velocity: -1.,
                    ..Default::default()
                }),
                ..Default::default()
            })
            .collect(),
        ..Default::default()
    }
}
fn local() -> zenoh::Session {
    zenoh::open(zenoh::Config::from_json5(r#"{mode:"peer",listen:{endpoints:["tcp/127.0.0.1:0"]},scouting:{multicast:{enabled:false},gossip:{enabled:false}}}"#).unwrap()).wait().unwrap()
}
fn put<M: Message + Name>(s: &zenoh::Session, key: &str, m: M) {
    s.put(key, m.encode_to_vec())
        .attachment(M::full_name())
        .wait()
        .unwrap();
}
#[test]
fn rejects_bad_types_clocks_joints_and_missing_ticks_on_transport() {
    let s = local();
    let key = "arm/state";
    let sub = s.declare_subscriber(key).wait().unwrap();
    assert!(receive::<Model>(&sub, Instant::now()).is_err());
    s.put(key, []).wait().unwrap();
    assert!(Feedback::at(&sub, 1).is_err());
    put(&s, key, Double::default());
    assert!(Feedback::at(&sub, 1).is_err());
    s.put(key, vec![255])
        .attachment(Model::full_name())
        .wait()
        .unwrap();
    assert!(Feedback::at(&sub, 1).is_err());
    for bad in 0..11 {
        let mut m = state(1);
        match bad {
            0 => m.header = None,
            1 => m.header.as_mut().unwrap().stamp = None,
            2 => m.header.as_mut().unwrap().stamp.as_mut().unwrap().sec = -1,
            3 => m.header.as_mut().unwrap().stamp.as_mut().unwrap().nsec = 1_000_000_000,
            4 => m.header.as_mut().unwrap().stamp.as_mut().unwrap().nsec = 1,
            5 => m.header.as_mut().unwrap().stamp.as_mut().unwrap().sec = i64::MAX,
            6 => m.joint.truncate(1),
            7 => m.joint.swap(0, 1),
            8 => m.joint[0].axis1 = None,
            9 => m.joint[1].axis1.as_mut().unwrap().position = f64::NAN,
            _ => m.joint[1].axis1.as_mut().unwrap().velocity = f64::INFINITY,
        }
        put(&s, key, m);
        assert!(Feedback::at(&sub, 1).is_err());
    }
    put(&s, key, state(2));
    assert!(Feedback::at(&sub, 1).is_err());
    put(&s, key, state(0));
    put(&s, key, state(1));
    let f = Feedback::at(&sub, 1).unwrap();
    assert_eq!((f.tick, f.position, f.velocity), (1, [0.5; 2], [-1.; 2]));
    put(&s, key, Clock::default());
    assert!(wait_rewind(&sub, 100).is_err());
    put(
        &s,
        key,
        Clock {
            sim: Some(Time { sec: 0, nsec: 123 }),
            ..Default::default()
        },
    );
    assert!(wait_rewind(&sub, 100).is_err());
    put(&s, key, state(0));
    assert!(wait_rewind(&sub, 100).is_err());
    for t in [0, 1, 100, 0] {
        put(
            &s,
            key,
            Clock {
                sim: Some(time(t)),
                ..Default::default()
            },
        );
    }
    wait_rewind(&sub, 100).unwrap();
    let p = s.declare_publisher("arm/command").wait().unwrap();
    let c = s.declare_subscriber("arm/command").wait().unwrap();
    command(&p, -8.0).unwrap();
    assert_eq!(
        receive::<Double>(&c, Instant::now() + WAIT).unwrap().data,
        -8.
    );
    s.close().wait().unwrap();
    assert!(command(&p, 8.0).is_err());
    assert!(Feedback::at(&c, 1).is_err());
}
#[test]
fn world_control_reports_refusal_bad_payload_and_lost_service() {
    let s = local();
    let key = "arm/control";
    let server = s
        .declare_queryable(key)
        .callback(|q| {
            let control = WorldControl::decode(q.payload().unwrap().to_bytes().as_ref()).unwrap();
            match control.multi_step {
                0 => q
                    .reply(
                        q.key_expr().clone(),
                        Boolean {
                            data: false,
                            ..Default::default()
                        }
                        .encode_to_vec(),
                    )
                    .wait()
                    .unwrap(),
                1 => q.reply_err("service unavailable").wait().unwrap(),
                2 => q.reply(q.key_expr().clone(), vec![255]).wait().unwrap(),
                _ => q
                    .reply(
                        q.key_expr().clone(),
                        Boolean {
                            data: true,
                            ..Default::default()
                        }
                        .encode_to_vec(),
                    )
                    .wait()
                    .unwrap(),
            }
        })
        .wait()
        .unwrap();
    for n in 0..3 {
        assert!(super::super::advance(&s, key, n).is_err());
    }
    super::super::advance(&s, key, 4).unwrap();
    drop(server);
    assert!(super::super::advance(&s, key, 5).is_err());
    s.close().wait().unwrap();
    assert!(super::super::advance(&s, key, 5).is_err());
}
#[test]
fn unwritable_evidence_destination_returns_an_error() {
    let tmp = tempfile::tempdir().unwrap();
    assert!(save(tmp.path(), &serde_json::json!({})).is_err());
}

#[test]
fn startup_reset_qualifier_reports_each_failed_boundary_on_a_live_peer() {
    for failure in 0..8 {
        let session = std::sync::Arc::new(local());
        let side = session.clone();
        let prefix = "episode";
        let control_key = "episode/world/default/control";
        let clock = "episode/world/default/clock";
        let samples = session.declare_subscriber("episode/state").wait().unwrap();
        let motor = session.declare_publisher("episode/command").wait().unwrap();
        let index = std::sync::atomic::AtomicUsize::new(0);
        let _service = session
            .declare_queryable(control_key)
            .callback(move |q| {
                let call = index.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                if failure == call {
                    q.reply_err("boundary unavailable").wait().unwrap();
                    return;
                }
                if call == 1 {
                    let mut m = state(100);
                    m.joint[0].axis1.as_mut().unwrap().position =
                        if failure == 5 { 0. } else { -0.2 };
                    if failure == 4 {
                        m.header = None;
                    }
                    put(&side, "episode/state", m);
                }
                if call == 3 {
                    if failure == 6 {
                        put(&side, clock, Clock::default());
                    } else {
                        for t in [100, 0] {
                            put(
                                &side,
                                clock,
                                Clock {
                                    sim: Some(time(t)),
                                    ..Default::default()
                                },
                            )
                        }
                    }
                }
                q.reply(
                    q.key_expr().clone(),
                    Boolean {
                        data: true,
                        ..Default::default()
                    }
                    .encode_to_vec(),
                )
                .wait()
                .unwrap();
            })
            .wait()
            .unwrap();
        let result = reset_loaded(&session, prefix, &samples, &motor);
        assert_eq!(
            result.is_err(),
            failure != 7,
            "fixture {failure}: {result:?}"
        );
    }
}
