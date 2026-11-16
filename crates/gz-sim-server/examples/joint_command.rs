// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Finite canonical Double sender with legacy M9/M42 topics as defaults.
//! An optional trailing --topic selects a concrete owner route.
use gz_sim_proto::{
    msgs::Double,
    prost::{Message, Name},
};
use gz_sim_server::ServerOptions;
use std::{error::Error, time::Duration};
use zenoh::Wait;
fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    run(std::env::args().skip(1).collect())
}

struct Command {
    endpoint: String,
    topic: String,
    target: f64,
    seconds: f64,
    stop: bool,
}

// The original four positionals retain their order. A trailing --topic selects
// one concrete owner route; the process still requires the matching partition.
fn command_arguments(args: Vec<String>) -> Result<Command, Box<dyn Error + Send + Sync>> {
    if !matches!(args.len(), 4 | 6) {
        return Err("usage: joint_command tcp/host:port position|velocity|force target repeat_seconds [--topic TOPIC] (matching GZ_PARTITION required)".into());
    }
    if args[..4].iter().any(|value| value.starts_with("--")) {
        return Err("unknown or misplaced joint command option".into());
    }
    let selected = if args.len() == 6 {
        if args[4] != "--topic" || args[5].is_empty() || args[5].starts_with("--") {
            return Err("only one trailing --topic TOPIC is supported".into());
        }
        Some(args[5].as_str())
    } else {
        None
    };
    let (topic, stop) = match args[1].as_str() {
        "position" => ("/model/arm/joint/base_hinge/0/cmd_pos", false),
        "velocity" => ("/model/arm/joint/tip_hinge/cmd_vel", true),
        "force" => ("/model/rotor/joint/hinge/cmd_force", true),
        _ => {
            return Err(
                "mode must be position (radians), velocity (radians/second) or force (Nm)".into(),
            );
        }
    };
    let target: f64 = args[2].parse()?;
    let seconds: f64 = args[3].parse()?;
    if !target.is_finite() || !seconds.is_finite() || !(0.0..=3600.0).contains(&seconds) {
        return Err("finite target and repeat duration in0..=3600seconds required".into());
    }
    Ok(Command {
        endpoint: args[0].clone(),
        topic: selected.unwrap_or(topic).to_owned(),
        target,
        seconds,
        stop,
    })
}

fn command_key(
    options: &ServerOptions,
    topic: &str,
) -> Result<String, Box<dyn Error + Send + Sync>> {
    let key = options.topic_key(topic)?;
    let concrete = zenoh::key_expr::OwnedNonWildKeyExpr::try_from(key)?;
    Ok(concrete.as_str().to_owned())
}

fn run(args: Vec<String>) -> Result<(), Box<dyn Error + Send + Sync>> {
    let Command {
        endpoint,
        topic,
        target,
        seconds,
        stop,
    } = command_arguments(args)?;
    // Validate authoritative namespace/remapping and upstream concrete-key
    // admission before opening any session or declaring a publisher.
    let key = command_key(&ServerOptions::from_env()?, &topic)?;
    let mut config = zenoh::Config::default();
    config.insert_json5("scouting/multicast/enabled", "false")?;
    config
        .connect
        .endpoints
        .set(vec![endpoint.parse()?])
        .map_err(|_| "invalid Zenoh connection endpoint configuration")?;
    let session = zenoh::open(config).wait()?;
    let publisher = session.declare_publisher(key).wait()?;
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    while !publisher.matching_status().wait()?.matching() {
        if std::time::Instant::now() >= deadline {
            return Err("no joint command subscriber matched within five seconds".into());
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let packet = |data| {
        Double {
            data,
            ..Default::default()
        }
        .encode_to_vec()
    };
    let duration = Duration::from_secs_f64(seconds);
    let start = std::time::Instant::now();
    // A zero repeat interval still sends one command. Force then sends zero
    // effort, which coasts rather than braking the joint.
    loop {
        publisher
            .put(packet(target))
            .attachment(Double::full_name().into_bytes())
            .wait()?;
        if start.elapsed() >= duration {
            break;
        }
        std::thread::sleep(Duration::from_millis(50).min(duration.saturating_sub(start.elapsed())));
    }
    if stop {
        publisher
            .put(packet(0.0))
            .attachment(Double::full_name().into_bytes())
            .wait()?;
    }
    drop(publisher);
    session.close().wait()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn force_zero_repeat_delivers_canonical_target_then_zero_to_actual_subscriber() {
        let reservation = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let endpoint = format!("tcp/127.0.0.1:{}", reservation.local_addr().unwrap().port());
        drop(reservation);
        let mut config = zenoh::Config::default();
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        config
            .listen
            .endpoints
            .set(vec![endpoint.parse().unwrap()])
            .unwrap();
        let session = zenoh::open(config).wait().unwrap();
        let key = ServerOptions::from_env()
            .unwrap()
            .topic_key("/model/rotor/joint/hinge/cmd_force")
            .unwrap();
        let subscriber = session.declare_subscriber(key).wait().unwrap();
        run(vec![endpoint, "force".into(), "1.5".into(), "0".into()]).unwrap();
        for expected in [1.5_f64, 0.0] {
            let sample = subscriber
                .recv_timeout(Duration::from_secs(3))
                .unwrap()
                .unwrap();
            assert_eq!(
                sample.attachment().unwrap().to_bytes().as_ref(),
                Double::full_name().as_bytes()
            );
            let message = Double::decode(sample.payload().to_bytes().as_ref()).unwrap();
            assert_eq!(message.data.to_bits(), expected.to_bits());
            assert_eq!(message.header, None);
        }
        assert!(
            subscriber
                .recv_timeout(Duration::from_millis(50))
                .unwrap()
                .is_none()
        );
        drop(subscriber);
        session.close().wait().unwrap();
    }

    fn arguments(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| (*value).to_owned()).collect()
    }

    #[test]
    fn parser_preserves_default_modes_and_accepts_only_trailing_topic_selection() {
        let endpoint = "tcp/127.0.0.1:7449";
        for (mode, topic, stop) in [
            ("position", "/model/arm/joint/base_hinge/0/cmd_pos", false),
            ("velocity", "/model/arm/joint/tip_hinge/cmd_vel", true),
            ("force", "/model/rotor/joint/hinge/cmd_force", true),
        ] {
            let original = command_arguments(arguments(&[endpoint, mode, "-0", "0"])).unwrap();
            assert_eq!(original.endpoint, endpoint);
            assert_eq!(original.topic, topic);
            assert_eq!(original.target.to_bits(), (-0.0_f64).to_bits());
            assert_eq!(original.seconds, 0.0);
            assert_eq!(original.stop, stop);
            for seconds in ["0", "3600"] {
                let selected = command_arguments(arguments(&[
                    endpoint,
                    mode,
                    "-1.25",
                    seconds,
                    "--topic",
                    "/model/arm_7/joint/base_hinge/0/cmd_pos",
                ]))
                .unwrap();
                assert_eq!(selected.endpoint, endpoint);
                assert_eq!(selected.topic, "/model/arm_7/joint/base_hinge/0/cmd_pos");
                assert_eq!(selected.target, -1.25);
                assert_eq!(selected.seconds, seconds.parse::<f64>().unwrap());
                assert_eq!(selected.stop, stop);
            }
        }
    }

    #[test]
    fn malformed_options_modes_and_nonfinite_commands_are_refused_before_session_setup() {
        let endpoint = "tcp/127.0.0.1:7449";
        for values in [
            vec![],
            vec![endpoint, "position", "1"],
            vec![endpoint, "position", "1", "0", "--topic"],
            vec![endpoint, "position", "1", "0", "extra"],
            vec![endpoint, "position", "1", "0", "--unknown", "/selected"],
            vec![endpoint, "position", "1", "0", "--topic", ""],
            vec![endpoint, "position", "1", "0", "--topic", "--unknown"],
            vec![endpoint, "position", "1", "0", "--topic", "--topic"],
            vec![
                endpoint, "position", "1", "0", "--topic", "/a", "--topic", "/b",
            ],
            vec!["--topic", "/a", endpoint, "position", "1", "0"],
            vec![endpoint, "position", "--topic", "/a", "1", "0"],
            vec!["--unknown", "position", "1", "0"],
            vec![endpoint, "torque", "1", "0"],
            vec![endpoint, "force", "NaN", "0"],
            vec![endpoint, "velocity", "inf", "0"],
            vec![endpoint, "position", "-inf", "0"],
            vec![endpoint, "position", "1", "NaN"],
            vec![endpoint, "position", "1", "inf"],
            vec![endpoint, "position", "1", "-0.001"],
            vec![endpoint, "position", "1", "3600.001"],
        ] {
            assert!(command_arguments(arguments(&values)).is_err(), "{values:?}");
        }
    }

    #[test]
    fn selected_keys_keep_authoritative_single_hop_mapping_and_refuse_wildcards() {
        let mut options = ServerOptions {
            partition: "m65-joint-map".into(),
            namespace: "fleet".into(),
            ..Default::default()
        };
        options
            .add_topic_remap(
                "/model/arm_0/joint/base_hinge/0/cmd_pos",
                "/mapped/owner0/position",
            )
            .unwrap();
        options
            .add_topic_remap("/mapped/owner0/position", "/not-followed")
            .unwrap();
        let before = options.clone();
        assert_eq!(
            command_key(&options, "/model/arm_0/joint/base_hinge/0/cmd_pos").unwrap(),
            "@/m65-joint-map@/mapped/owner0/position"
        );
        assert_eq!(
            command_key(&options, "relative/input").unwrap(),
            "@/m65-joint-map@/fleet/relative/input"
        );
        assert_eq!(
            command_key(&options, "/model/arm_7/joint/tip_hinge/cmd_vel").unwrap(),
            "@/m65-joint-map@/model/arm_7/joint/tip_hinge/cmd_vel"
        );
        for topic in [
            "",
            "/",
            "/m62/*/cmd_pos",
            "/m62/**",
            "/m62/$*",
            "/a//b",
            "/a b",
            "/a?b",
            "/a#b",
            "/a@b",
        ] {
            assert!(command_key(&options, topic).is_err(), "{topic:?}");
        }
        assert_eq!(options, before);
        options.add_topic_remap("/selected", "/wild/*").unwrap();
        assert_eq!(
            options.topic_key("/selected").unwrap(),
            "@/m65-joint-map@/wild/*"
        );
        assert!(command_key(&options, "/selected").is_err());
    }

    #[test]
    fn selected_real_subscribers_receive_canonical_mode_targets_without_cross_owner_delivery() {
        let mut config = zenoh::Config::default();
        config
            .listen
            .endpoints
            .set(vec!["tcp/127.0.0.1:0".parse().unwrap()])
            .unwrap();
        config.connect.endpoints.set(vec![]).unwrap();
        config
            .insert_json5("scouting/multicast/enabled", "false")
            .unwrap();
        config
            .insert_json5("scouting/gossip/enabled", "false")
            .unwrap();
        let peer = zenoh::open(config).wait().unwrap();
        let endpoints = peer.info().locators().wait();
        assert_eq!(endpoints.len(), 1);
        let endpoint = endpoints[0].to_string();
        assert!(endpoint.starts_with("tcp/127.0.0.1:") && !endpoint.ends_with(":0"));
        let options = ServerOptions::from_env().unwrap();
        let family = uuid::Uuid::new_v4();
        for (mode, stop) in [("position", false), ("velocity", true), ("force", true)] {
            let topics = [
                format!("/m65/{family}/left/{mode}"),
                format!("/m65/{family}/right/{mode}"),
            ];
            let keys = topics
                .each_ref()
                .map(|topic| command_key(&options, topic).unwrap());
            assert_ne!(keys[0], keys[1]);
            let subscribers = keys
                .each_ref()
                .map(|key| peer.declare_subscriber(key.clone()).wait().unwrap());
            for (index, (subscriber, target)) in
                subscribers.iter().zip([0.75_f64, -1.5]).enumerate()
            {
                // Invoke the complete real helper, including matching, the
                // zero-duration first target, mode termination and close.
                run(vec![
                    endpoint.clone(),
                    mode.into(),
                    target.to_string(),
                    "0".into(),
                    "--topic".into(),
                    topics[index].clone(),
                ])
                .unwrap();
                let expected = if stop {
                    vec![target, 0.0]
                } else {
                    vec![target]
                };
                for data in expected {
                    let sample = subscriber
                        .recv_timeout(Duration::from_secs(3))
                        .unwrap()
                        .unwrap();
                    assert_eq!(sample.key_expr().as_str(), keys[index]);
                    assert_eq!(sample.encoding().to_string(), "zenoh/bytes");
                    assert_eq!(
                        sample.attachment().unwrap().to_bytes().as_ref(),
                        b"gz.msgs.Double"
                    );
                    let wire = Double {
                        data,
                        ..Default::default()
                    }
                    .encode_to_vec();
                    assert_eq!(sample.payload().to_bytes().as_ref(), wire);
                    let decoded = Double::decode(sample.payload().to_bytes().as_ref()).unwrap();
                    assert!(decoded.header.is_none());
                    assert_eq!(decoded.data.to_bits(), data.to_bits());
                }
                // Position retains its target: no synthetic zero follows it.
                // Velocity and force emitted exactly their target then zero.
                assert!(
                    subscriber
                        .recv_timeout(Duration::from_millis(50))
                        .unwrap()
                        .is_none()
                );
                for (other_index, other) in subscribers.iter().enumerate() {
                    if other_index != index {
                        assert!(
                            other
                                .recv_timeout(Duration::from_millis(50))
                                .unwrap()
                                .is_none()
                        );
                    }
                }
                assert!(!peer.is_closed());
            }
            drop(subscribers);
        }
        peer.close().wait().unwrap();
    }
}
