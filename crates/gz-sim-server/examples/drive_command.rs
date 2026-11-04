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
    let mut config = zenoh::Config::default();
    config.insert_json5("scouting/multicast/enabled", "false")?;
    config
        .connect
        .endpoints
        .set(vec![args[0].parse()?])
        .map_err(|_| "invalid Zenoh connection endpoint configuration")?;
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
