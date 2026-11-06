// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Finite command sender for the M8 robot; uses upstream Zenoh and canonical Prost.
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

fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 4 {
        return Err("usage: drive_command tcp/host:port linear_m_per_s yaw_rad_per_s duration_seconds (GZ_PARTITION must match simulator)".into());
    }
    let linear: f64 = args[1].parse()?;
    let angular: f64 = args[2].parse()?;
    let seconds: f64 = args[3].parse()?;
    if ![linear, angular, seconds].into_iter().all(f64::is_finite)
        || !(0.0..=3600.0).contains(&seconds)
    {
        return Err("finite velocities and duration in 0..=3600 seconds required".into());
    }
    let key = ServerOptions::from_env()?.topic_key("/model/robot/cmd_vel")?;
    let config = outgoing_config(&args[0])?;
    let session = zenoh::open(config).wait()?;
    let publisher = session.declare_publisher(key).wait()?;
    let packet = |linear, angular| {
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
    };
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    while !publisher.matching_status().wait()?.matching() {
        if std::time::Instant::now() >= deadline {
            return Err("no command subscriber matched within five seconds".into());
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    let duration = Duration::from_secs_f64(seconds);
    let start = std::time::Instant::now();
    while start.elapsed() < duration {
        publisher
            .put(packet(linear, angular))
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
}
