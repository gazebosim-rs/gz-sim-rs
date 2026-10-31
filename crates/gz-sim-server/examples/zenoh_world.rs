// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Bounded application diagnostic, not a performance or complete migration claim.
use gz_sim_scene::{PhysicsEngine, load_world_with_sdf_step};
use gz_sim_server::Server;
use std::{
    num::NonZeroUsize,
    path::Path,
    time::{Duration, Instant},
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<_> = std::env::args().collect();
    assert_eq!(args.len(), 2, "parent stop-file path required");
    let stop = Path::new(&args[1]);
    assert!(!stop.exists());
    let mut simulation = load_world_with_sdf_step(
        include_str!("../../gz-sim-scene/tests/fixtures/headless_imu.sdf"),
        "headless_imu.sdf",
        "default",
        &sdformat::NoResources,
        PhysicsEngine::Tpe,
    )?;
    simulation.set_paused(true);
    let config = gz_sim_server::ServerOptions {
        partition: std::env::var("GZ_PARTITION")?,
        ..Default::default()
    };
    let mut server = Server::new(simulation, "default", config, NonZeroUsize::new(8).unwrap())?;
    println!("WORLD_READY");
    let deadline = Instant::now() + Duration::from_secs(20);
    while !stop.exists() {
        assert!(
            Instant::now() < deadline,
            "parent did not finish native world controls"
        );
        server.prepare_io()?;
        server.tick()?;
        std::thread::sleep(Duration::from_millis(1));
    }
    assert!(server.counters().poses_queued > 0);
    assert_eq!(server.counters().pose_failures, 0);
    assert_eq!(server.counters().controls_accepted, 3);
    assert_eq!(server.counters().controls_rejected, 0);
    assert_eq!(server.counters().response_failures, 0);
    assert_eq!(server.simulation().elapsed(), Duration::ZERO);
    assert!(server.simulation().is_paused());
    server.shutdown();
    println!("WORLD_CONTROL_PASS");
    Ok(())
}
