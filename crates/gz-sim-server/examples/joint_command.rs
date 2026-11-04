// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded canonical commands for the two joints in the M9 example world.
use gz_sim_proto::{
    msgs::Double,
    prost::{Message, Name},
};
use gz_sim_server::ServerOptions;
use std::{error::Error, time::Duration};
use zenoh::Wait;
fn main() -> Result<(), Box<dyn Error + Send + Sync>> {
    let args: Vec<_> = std::env::args().skip(1).collect();
    if args.len() != 4 {
        return Err("usage: joint_command tcp/host:port position|velocity target repeat_seconds (matching GZ_PARTITION required)".into());
    }
    let (topic, stop) = match args[1].as_str() {
        "position" => ("/model/arm/joint/base_hinge/0/cmd_pos", false),
        "velocity" => ("/model/arm/joint/tip_hinge/cmd_vel", true),
        _ => return Err("mode must be position (radians) or velocity (radians/second)".into()),
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
    // A zero repeat interval still sends one position or velocity command.
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
