// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded canonical commands for the M9 joints and M42 world hinge.
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

fn run(args: Vec<String>) -> Result<(), Box<dyn Error + Send + Sync>> {
    if args.len() != 4 {
        return Err("usage: joint_command tcp/host:port position|velocity|force target repeat_seconds (matching GZ_PARTITION required)".into());
    }
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
    let key = ServerOptions::from_env()?.topic_key(topic)?;
    let mut config = zenoh::Config::default();
    config.insert_json5("scouting/multicast/enabled", "false")?;
    config
        .connect
        .endpoints
        .set(vec![args[0].parse()?])
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
}
