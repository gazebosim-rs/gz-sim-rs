// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Finite canonical Twist sender; the M8 topic remains the default.
use gz_sim_proto::{
    msgs::{Twist, Vector3d},
    prost::{Message, Name},
};
use gz_sim_server::ServerOptions;
use std::{error::Error, time::Duration};
use zenoh::Wait;
fn outgoing_config(endpoint: &str) -> Result<zenoh::Config, Box<dyn Error + Send + Sync>> {
    let mut config = zenoh::Config::default();
    config
        .listen
        .endpoints
        .set(vec![])
        .map_err(|_| "invalid Zenoh listener endpoint configuration")?;
    config.insert_json5("scouting/multicast/enabled", "false")?;
    config.insert_json5("scouting/gossip/enabled", "false")?;
    config
        .connect
        .endpoints
        .set(vec![endpoint.parse()?])
        .map_err(|_| "invalid Zenoh connection endpoint configuration")?;
    Ok(config)
}

struct Command {
    endpoint: String,
    linear: f64,
    angular: f64,
    seconds: f64,
    topic: String,
}

// One optional concrete topic; the original four positionals keep their order.
fn command_arguments(args: Vec<String>) -> Result<Command, Box<dyn Error + Send + Sync>> {
    let mut values = Vec::new();
    let mut topic = None;
    let mut arguments = args.into_iter();
    while let Some(argument) = arguments.next() {
        if argument == "--topic" {
            if topic.is_some() {
                return Err("--topic may be supplied only once".into());
            }
            let value = arguments.next().ok_or("--topic requires a concrete key")?;
            if value.starts_with("--") {
                return Err("--topic requires a key, not another option".into());
            }
            topic = Some(value);
        } else if argument.starts_with("--") {
            return Err("unknown command option".into());
        } else {
            values.push(argument);
        }
    }
    if values.len() != 4 {
        return Err("usage: drive_command [--topic KEY] tcp/host:port linear_m_per_s yaw_rad_per_s duration_seconds (GZ_PARTITION must match simulator)".into());
    }
    let linear: f64 = values[1].parse()?;
    let angular: f64 = values[2].parse()?;
    let seconds: f64 = values[3].parse()?;
    if ![linear, angular, seconds].into_iter().all(f64::is_finite)
        || !(0.0..=3600.0).contains(&seconds)
    {
        return Err("finite velocities and duration in 0..=3600 seconds required".into());
    }
    Ok(Command {
        endpoint: values.remove(0),
        linear,
        angular,
        seconds,
        topic: topic.unwrap_or_else(|| "/model/robot/cmd_vel".to_owned()),
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

fn packet(linear: f64, angular: f64) -> Vec<u8> {
    Twist {
        linear: Some(Vector3d {
            x: linear,
            ..Default::default()
        }),
        angular: Some(Vector3d {
            z: angular,
            ..Default::default()
        }),
        ..Default::default()
    }
    .encode_to_vec()
}

fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    let command = command_arguments(std::env::args().skip(1).collect())?;
    let key = command_key(&ServerOptions::from_env()?, &command.topic)?;
    let config = outgoing_config(&command.endpoint)?;
    let session = zenoh::open(config).wait()?;
    let publisher = session.declare_publisher(key).wait()?;
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    while !publisher.matching_status().wait()?.matching() {
        if std::time::Instant::now() >= deadline {
            return Err("no command subscriber matched within five seconds".into());
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let duration = Duration::from_secs_f64(command.seconds);
    let start = std::time::Instant::now();
    while start.elapsed() < duration {
        publisher
            .put(packet(command.linear, command.angular))
            .attachment(Twist::full_name().into_bytes())
            .wait()?;
        std::thread::sleep(Duration::from_millis(50).min(duration.saturating_sub(start.elapsed())));
    }
    publisher
        .put(packet(0.0, 0.0))
        .attachment(Twist::full_name().into_bytes())
        .wait()?;
    drop(publisher);
    session.close().wait()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::{Value, json};
    use std::time::Instant;

    fn field(config: &zenoh::Config, path: &str) -> Value {
        serde_json::from_str(&config.get_json(path).unwrap()).unwrap()
    }

    #[test]
    fn outgoing_config_has_only_the_requested_connection_and_no_discovery() {
        let endpoint = "tcp/127.0.0.1:7449";
        let config = outgoing_config(endpoint).unwrap();
        assert_eq!(field(&config, "listen/endpoints"), json!([]));
        assert_eq!(field(&config, "connect/endpoints"), json!([endpoint]));
        assert_eq!(field(&config, "scouting/multicast/enabled"), json!(false));
        assert_eq!(field(&config, "scouting/gossip/enabled"), json!(false));
        for invalid in ["", "not-an-endpoint"] {
            assert!(outgoing_config(invalid).is_err(), "{invalid:?}");
        }
    }

    #[test]
    fn real_outgoing_session_reaches_its_owned_peer_without_a_listener() {
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
        let locators = peer.info().locators().wait();
        assert_eq!(locators.len(), 1);
        let endpoint = locators[0].to_string();
        assert!(endpoint.starts_with("tcp/127.0.0.1:"));
        assert_ne!(endpoint, "tcp/127.0.0.1:0");
        let subscriber = peer
            .declare_subscriber("m12/outgoing-config")
            .wait()
            .unwrap();
        let outgoing = zenoh::open(outgoing_config(&endpoint).unwrap())
            .wait()
            .unwrap();
        assert!(outgoing.info().locators().wait().is_empty());
        let publisher = outgoing
            .declare_publisher("m12/outgoing-config")
            .wait()
            .unwrap();
        let deadline = Instant::now() + Duration::from_secs(5);
        while !publisher.matching_status().wait().unwrap().matching() {
            assert!(Instant::now() < deadline, "owned peer did not match");
            std::thread::sleep(Duration::from_millis(10));
        }
        publisher.put(b"outgoing-only").wait().unwrap();
        let sample = subscriber
            .recv_timeout(Duration::from_secs(5))
            .expect("owned peer receive failed")
            .expect("owned peer did not receive the outgoing publication");
        assert_eq!(sample.payload().to_bytes().as_ref(), b"outgoing-only");
        assert!(outgoing.info().locators().wait().is_empty());
        drop(publisher);
        drop(subscriber);
        outgoing.close().wait().unwrap();
        peer.close().wait().unwrap();
    }

    fn arguments(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| (*value).to_owned()).collect()
    }

    #[test]
    fn command_parser_keeps_default_positionals_and_selects_explicit_topics() {
        let values = ["tcp/127.0.0.1:7449", "0.4", "-0.5", "3"];
        let default = command_arguments(arguments(&values)).unwrap();
        assert_eq!(default.endpoint, values[0]);
        assert_eq!(
            [default.linear, default.angular, default.seconds],
            [0.4, -0.5, 3.0]
        );
        assert_eq!(default.topic, "/model/robot/cmd_vel");
        for selected in [
            [
                "--topic",
                "/m59/left/cmd_vel",
                values[0],
                values[1],
                values[2],
                values[3],
            ],
            [
                values[0],
                values[1],
                values[2],
                values[3],
                "--topic",
                "/m59/right/cmd_vel",
            ],
        ] {
            let command = command_arguments(arguments(&selected)).unwrap();
            assert_eq!(command.endpoint, values[0]);
            assert_eq!(
                [command.linear, command.angular, command.seconds],
                [0.4, -0.5, 3.0]
            );
            assert!(matches!(
                command.topic.as_str(),
                "/m59/left/cmd_vel" | "/m59/right/cmd_vel"
            ));
        }
        for seconds in ["0", "3600"] {
            assert!(command_arguments(arguments(&[values[0], "0", "0", seconds])).is_ok());
        }
    }

    #[test]
    fn malformed_options_and_nonfinite_or_unbounded_commands_are_refused() {
        for values in [
            vec![],
            vec!["tcp/127.0.0.1:7449", "0", "0"],
            vec!["--topic"],
            vec!["--topic", "--unknown", "tcp/127.0.0.1:7449", "0", "0", "1"],
            vec!["--topic", "--topic", "tcp/127.0.0.1:7449", "0", "0", "1"],
            vec![
                "--topic",
                "/a",
                "--topic",
                "/b",
                "tcp/127.0.0.1:7449",
                "0",
                "0",
                "1",
            ],
            vec!["--unknown", "tcp/127.0.0.1:7449", "0", "0", "1"],
            vec!["tcp/127.0.0.1:7449", "NaN", "0", "1"],
            vec!["tcp/127.0.0.1:7449", "0", "inf", "1"],
            vec!["tcp/127.0.0.1:7449", "0", "0", "NaN"],
            vec!["tcp/127.0.0.1:7449", "0", "0", "-1"],
            vec!["tcp/127.0.0.1:7449", "0", "0", "3600.1"],
            vec!["tcp/127.0.0.1:7449", "0", "0", "1", "extra"],
        ] {
            assert!(command_arguments(arguments(&values)).is_err(), "{values:?}");
        }
        let options = ServerOptions {
            partition: "m60-pure".into(),
            ..Default::default()
        };
        for topic in [
            "",
            "/",
            "/m59/*/cmd_vel",
            "/m59/**",
            "/m59/$*",
            "/a//b",
            "/a b",
            "/a?b",
            "/a#b",
        ] {
            assert!(command_key(&options, topic).is_err(), "{topic:?}");
        }
        let command = command_arguments(arguments(&[
            "tcp/127.0.0.1:7449",
            "0",
            "0",
            "1",
            "--topic",
            "",
        ]))
        .unwrap();
        assert!(command_key(&options, &command.topic).is_err());
    }

    #[test]
    fn explicit_routes_deliver_real_canonical_twists_and_zero_to_only_the_selected_owner() {
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
        let options = ServerOptions {
            partition: format!("m60-drive-{}", uuid::Uuid::new_v4()),
            ..Default::default()
        };
        let keys = ["/m59/left/cmd_vel", "/m59/right/cmd_vel"]
            .map(|topic| command_key(&options, topic).unwrap());
        assert_ne!(keys[0], keys[1]);
        assert_eq!(
            command_key(&options, "/model/robot/cmd_vel").unwrap(),
            options.topic_key("/model/robot/cmd_vel").unwrap()
        );
        let subscribers = keys
            .each_ref()
            .map(|key| peer.declare_subscriber(key.clone()).wait().unwrap());
        let outgoing = zenoh::open(outgoing_config(&endpoints[0].to_string()).unwrap())
            .wait()
            .unwrap();
        assert!(outgoing.info().locators().wait().is_empty());
        let publishers = keys
            .each_ref()
            .map(|key| outgoing.declare_publisher(key.clone()).wait().unwrap());
        let deadline = Instant::now() + Duration::from_secs(5);
        while !publishers
            .iter()
            .all(|publisher| publisher.matching_status().wait().unwrap().matching())
        {
            assert!(Instant::now() < deadline);
            std::thread::yield_now();
        }
        for (index, ((publisher, subscriber), (linear, angular))) in publishers
            .iter()
            .zip(subscribers.iter())
            .zip([(0.4, 0.0), (-0.25, 0.5)])
            .enumerate()
        {
            for (linear, angular) in [(linear, angular), (0.0, 0.0)] {
                let wire = packet(linear, angular);
                publisher
                    .put(wire.clone())
                    .attachment(Twist::full_name().into_bytes())
                    .wait()
                    .unwrap();
                let sample = subscriber
                    .recv_timeout(Duration::from_secs(5))
                    .unwrap()
                    .unwrap();
                assert_eq!(sample.key_expr().as_str(), keys[index]);
                assert_eq!(
                    sample.attachment().unwrap().to_bytes().as_ref(),
                    b"gz.msgs.Twist"
                );
                assert_eq!(sample.payload().to_bytes().as_ref(), wire);
                let value = Twist::decode(sample.payload().to_bytes().as_ref()).unwrap();
                assert!(value.header.is_none());
                let linear_value = value.linear.unwrap();
                let angular_value = value.angular.unwrap();
                assert!(linear_value.header.is_none() && angular_value.header.is_none());
                assert_eq!(
                    [linear_value.x, linear_value.y, linear_value.z],
                    [linear, 0.0, 0.0]
                );
                assert_eq!(
                    [angular_value.x, angular_value.y, angular_value.z],
                    [0.0, 0.0, angular]
                );
                assert!(subscribers[1 - index].try_recv().unwrap().is_none());
            }
        }
        drop(publishers);
        drop(subscribers);
        outgoing.close().wait().unwrap();
        peer.close().wait().unwrap();
    }
}
