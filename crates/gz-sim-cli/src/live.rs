// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use gz_sim_core::Simulation;
use gz_sim_scene::WorldPhysics;
use gz_sim_server::Server;
use signal_hook::{
    consts::{SIGINT, SIGTERM},
    flag,
};
use std::{
    error::Error,
    io,
    num::NonZeroUsize,
    path::PathBuf,
    sync::{
        Arc,
        atomic::{AtomicBool, Ordering},
    },
    thread,
    time::{Duration, Instant},
};

const POLL_INTERVAL: Duration = Duration::from_millis(10);

struct Termination {
    requested: Arc<AtomicBool>,
}

impl Termination {
    fn install() -> io::Result<Self> {
        let requested = Arc::new(AtomicBool::new(false));
        // The conditional exit runs before the flag is set, so only a second signal
        // forces exit. Actions stay registered through final output; a failure here
        // ends the process, so partial registration needs no rollback.
        for signal in [SIGINT, SIGTERM] {
            flag::register_conditional_shutdown(signal, 128 + signal, requested.clone())?;
            flag::register(signal, requested.clone())?;
        }
        Ok(Self { requested })
    }

    fn requested(&self) -> bool {
        self.requested.load(Ordering::Relaxed)
    }

    fn wait_until(&self, deadline: Instant) {
        while !self.requested() {
            let remaining = deadline.saturating_duration_since(Instant::now());
            if remaining.is_zero() {
                break;
            }
            thread::sleep(remaining.min(POLL_INTERVAL));
        }
    }
}

fn wall_period(step: Duration, factor: f64) -> io::Result<Duration> {
    if !factor.is_finite() || factor < 0.0 {
        return Err(io::Error::other(
            "real_time_factor must be finite and nonnegative",
        ));
    }
    if factor == 0.0 {
        return Ok(Duration::ZERO);
    }
    let seconds = step.as_secs_f64() / factor;
    Duration::try_from_secs_f64(seconds)
        .ok()
        .filter(|period| !period.is_zero())
        .ok_or_else(|| io::Error::other("unrepresentable real-time update period"))
}

// The ordinary variant owns no camera settings, session, callbacks or buffers.
enum Runtime {
    Headless(Box<Server>),
    #[cfg(feature = "rgb-camera")]
    Camera {
        application: Box<gz_camera_output::CameraServer>,
        options: Box<gz_camera_output::CameraServerOptions>,
        backpressured: bool,
    },
}

impl Runtime {
    fn new(
        simulation: Simulation,
        world: &str,
        worker: Option<PathBuf>,
    ) -> Result<Self, Box<dyn Error>> {
        let options = gz_sim_server::ServerOptions::from_env()?;
        let budget = NonZeroUsize::new(64).expect("nonzero command budget");
        #[cfg(feature = "rgb-camera")]
        if let Some(worker_path) = worker {
            let config = match std::env::var_os("ZENOH_CONFIG") {
                Some(path) => {
                    zenoh::Config::from_file(path).map_err(|error| -> Box<dyn Error> { error })?
                }
                None => zenoh::Config::default(),
            };
            // The output owns a separate outgoing peer. Retain upstream
            // connect/scouting settings without allocating a listener that could
            // consume the world Server's configured endpoint.
            let mut camera_config = config.clone();
            camera_config
                .insert_json5("listen/endpoints", "[]")
                .map_err(|error| -> Box<dyn Error> { error })?;
            let camera_options = gz_camera_output::CameraServerOptions {
                worker_path,
                origin: [0.0; 3],
                zenoh: camera_config,
            };
            let application = gz_camera_output::CameraServer::new(
                simulation,
                world,
                options,
                config,
                budget,
                &camera_options,
            )?;
            return Ok(if application.has_camera_output() {
                Self::Camera {
                    application: Box::new(application),
                    options: Box::new(camera_options),
                    backpressured: false,
                }
            } else {
                Self::Headless(Box::new(application.into_server()))
            });
        }
        #[cfg(not(feature = "rgb-camera"))]
        debug_assert!(worker.is_none(), "feature checked before loading");
        Ok(Self::Headless(Box::new(Server::new(
            simulation, world, options, budget,
        )?)))
    }

    fn server(&self) -> &Server {
        match self {
            Self::Headless(server) => server,
            #[cfg(feature = "rgb-camera")]
            Self::Camera { application, .. } => application.server(),
        }
    }

    fn server_mut(&mut self) -> &mut Server {
        match self {
            Self::Headless(server) => server,
            #[cfg(feature = "rgb-camera")]
            Self::Camera { application, .. } => application.server_mut(),
        }
    }

    fn tick(&mut self) -> Result<gz_sim_core::UpdateInfo, Box<dyn Error>> {
        match self {
            Self::Headless(server) => {
                server
                    .prepare_io()
                    .map_err(|error| super::with_fault(server.simulation(), error))?;
                server
                    .tick()
                    .map_err(|error| super::with_fault(server.simulation(), error))
            }
            #[cfg(feature = "rgb-camera")]
            Self::Camera {
                application,
                options,
                backpressured,
            } => {
                let info = application
                    .tick(options)
                    .map_err(|error| super::with_fault(application.server().simulation(), error))?;
                // Physics is already committed. Busy affects output only: never
                // retry tick or request the same acquisition in this iteration.
                match application.capture() {
                    Ok(_) => *backpressured = false,
                    Err(gz_camera_output::CameraServerError::Camera(
                        gz_camera_output::CameraOutputError::Busy,
                    )) => {
                        if !*backpressured {
                            eprintln!(
                                "camera output backpressure: bounded worker busy; simulation continues"
                            );
                        }
                        *backpressured = true;
                    }
                    Err(error) => {
                        return Err(super::with_fault(application.server().simulation(), error));
                    }
                }
                Ok(info)
            }
        }
    }

    fn into_simulation(self) -> Simulation {
        match self {
            Self::Headless(server) => (*server).into_simulation(),
            #[cfg(feature = "rgb-camera")]
            Self::Camera { application, .. } => (*application).into_simulation(),
        }
    }
}

pub(super) fn run(
    mut simulation: Simulation,
    world: &str,
    running: bool,
    iterations: u64,
    web_viewer: Option<u16>,
    camera_worker: Option<PathBuf>,
) -> Result<(Simulation, u64), Box<dyn Error>> {
    let factor = simulation
        .world()
        .get_resource::<WorldPhysics>()
        .and_then(WorldPhysics::selected)
        .map_or(1.0, |physics| physics.real_time_factor);
    let period = wall_period(simulation.step_duration(), factor)?;
    // Validate the wall-clock range before creating workers or advancing physics.
    Instant::now()
        .checked_add(period)
        .ok_or_else(|| io::Error::other("real-time update period exceeds wall-clock range"))?;
    let termination = Termination::install()?;
    simulation.set_paused(!running);
    let mut runtime = Runtime::new(simulation, world, camera_worker)?;
    runtime.server_mut().prepare_run(Duration::from_secs(5))?;
    let mut viewer = web_viewer
        .map(|port| super::web::Bridge::new(runtime.server().simulation(), world, port))
        .transpose()?;
    let mut completed = 0_u64;
    while !termination.requested() && (iterations == 0 || completed < iterations) {
        let started = Instant::now();
        let controls = viewer
            .as_mut()
            .map(|viewer| viewer.controls(runtime.server_mut()))
            .unwrap_or_default();
        runtime.server_mut().set_lidar_viewer_interest(
            viewer
                .as_ref()
                .is_some_and(|viewer| viewer.scan_interested()),
        );
        let info = runtime.tick()?;
        if let Some(viewer) = &mut viewer {
            viewer.publish(runtime.server().simulation(), world, controls)?;
        }
        if !info.dt.is_zero() {
            // Overflow would need 2^64 advancing ticks.
            completed += 1;
        }
        if iterations != 0 && completed == iterations {
            break;
        }
        let delay = if info.paused { POLL_INTERVAL } else { period };
        let deadline = started
            .checked_add(delay)
            .ok_or_else(|| io::Error::other("real-time update deadline overflow"))?;
        termination.wait_until(deadline);
    }
    Ok((runtime.into_simulation(), completed))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pacing_preserves_simulation_step_and_checks_invalid_wall_periods() {
        let step = Duration::from_millis(2);
        assert_eq!(wall_period(step, 1.0).unwrap(), step);
        assert_eq!(wall_period(step, 0.5).unwrap(), Duration::from_millis(4));
        assert_eq!(wall_period(step, 2.0).unwrap(), Duration::from_millis(1));
        assert_eq!(wall_period(step, 0.0).unwrap(), Duration::ZERO);
        assert_eq!(wall_period(step, -0.0).unwrap(), Duration::ZERO);
        for factor in [
            -1.0,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NAN,
            f64::MIN_POSITIVE,
            f64::MAX,
        ] {
            assert!(wall_period(step, factor).is_err(), "{factor}");
        }
    }
}
