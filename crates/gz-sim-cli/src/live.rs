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
        mixed_pair: bool,
        rgb_family: bool,
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
            // Camera publications use the world's existing upstream peer. A
            // listener-only configuration must not leave a second camera peer
            // isolated when discovery is disabled.
            let camera_options = gz_camera_output::CameraServerOptions {
                worker_path,
                origin: [0.0; 3],
                zenoh: config.clone(),
            };
            let constructor = if rgb_family {
                gz_camera_output::CameraServer::new_rgb_family_with_server_session
            } else if mixed_pair {
                gz_camera_output::CameraServer::new_mixed_pair_with_server_session
            } else {
                gz_camera_output::CameraServer::new_with_server_session
            };
            let application =
                constructor(simulation, world, options, config, budget, &camera_options)?;
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
        debug_assert!(
            worker.is_none() && !mixed_pair && !rgb_family,
            "feature checked before loading"
        );
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
                let captures = application
                    .capture_all()
                    .map_err(|error| super::with_fault(application.server().simulation(), error))?;
                let mut busy = false;
                let mut failure = None;
                // All owners were visited before these results are interpreted.
                for capture in captures {
                    match capture.result {
                        Ok(_) => {}
                        Err(gz_camera_output::CameraServerError::Camera(
                            gz_camera_output::CameraOutputError::Busy,
                        )) => busy = true,
                        Err(error) => {
                            failure.get_or_insert(error);
                        }
                    }
                }
                if let Some(error) = failure {
                    return Err(super::with_fault(application.server().simulation(), error));
                }
                if busy && !*backpressured {
                    eprintln!(
                        "camera output backpressure: bounded worker busy; simulation continues"
                    );
                }
                *backpressured = busy;
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

/// Playback uses the same termination owner and local viewer IO, without loading
/// SDF or constructing Simulation, Server, Zenoh, physics or sensor resources.
pub(super) fn replay(path: &std::path::Path, port: u16) -> Result<(), Box<dyn Error>> {
    let termination = Termination::install()?;
    super::web::replay(path, port, &termination.requested)?;
    Ok(())
}

pub(super) struct ViewerOptions {
    pub port: Option<u16>,
    pub lifecycle_sdf: Option<sdformat::Sdf>,
    pub recording_path: Option<PathBuf>,
}

pub(super) struct CameraOptions {
    pub worker: Option<PathBuf>,
    pub mixed_pair: bool,
    pub rgb_family: bool,
}

pub(super) fn run(
    mut simulation: Simulation,
    world: &str,
    running: bool,
    iterations: u64,
    camera: CameraOptions,
    viewer: ViewerOptions,
) -> Result<(Simulation, u64), Box<dyn Error>> {
    let ViewerOptions {
        port: web_viewer,
        lifecycle_sdf,
        recording_path,
    } = viewer;
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
    let mut runtime = Runtime::new(
        simulation,
        world,
        camera.worker,
        camera.mixed_pair,
        camera.rgb_family,
    )?;
    runtime.server_mut().prepare_run(Duration::from_secs(5))?;
    let mut viewer = web_viewer
        .map(|port| match lifecycle_sdf.as_ref() {
            Some(sdf) => match recording_path.as_deref() {
                Some(path) => super::web::Bridge::new_recording(
                    runtime.server().simulation(),
                    world,
                    port,
                    sdf,
                    path,
                ),
                None => super::web::Bridge::new_with_lifecycle(
                    runtime.server().simulation(),
                    world,
                    port,
                    sdf,
                ),
            },
            None => super::web::Bridge::new(runtime.server().simulation(), world, port),
        })
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
            viewer.apply_lifecycle(runtime.server_mut(), world)?;
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
    if let Some(viewer) = &mut viewer {
        viewer.finish_recording()?;
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

    #[cfg(feature = "rgb-camera")]
    #[test]
    fn explicit_rgb_family_runtime_dispatch_preserves_cardinality_and_worker_error_precedence() {
        use gz_camera_output::{CameraOutputError, CameraServerError};

        // Follow the retained exact-child test pattern, while changing only
        // that child's environment. Parallel tests keep their globals intact.
        const CHILD: &str = "GZ_M111_RUNTIME_FAMILY_PREFLIGHT_CHILD";
        const TEST: &str = "live::tests::explicit_rgb_family_runtime_dispatch_preserves_cardinality_and_worker_error_precedence";
        if let Some(selected) = std::env::var_os(CHILD) {
            assert_eq!(selected, TEST);
            assert!(std::env::var_os("ZENOH_CONFIG").is_none());
            assert_eq!(
                std::env::var("GZ_PARTITION").unwrap(),
                "m111-runtime-family-preflight"
            );
        } else {
            let output = std::process::Command::new("timeout")
                .args(["--signal=KILL", "20s"])
                .arg(std::env::current_exe().unwrap())
                .args(["--exact", TEST, "--nocapture"])
                .env(CHILD, TEST)
                .env("GZ_PARTITION", "m111-runtime-family-preflight")
                .env_remove("ZENOH_CONFIG")
                .env_remove("GZ_TRANSPORT_ZENOH_CONFIG_OVERRIDE")
                .stdin(std::process::Stdio::null())
                .output()
                .unwrap();
            let stdout = String::from_utf8(output.stdout).unwrap();
            let stderr = String::from_utf8_lossy(&output.stderr);
            assert!(
                output.status.success(),
                "{}: {stdout} {stderr}",
                output.status
            );
            assert!(
                stdout.contains(&format!("test {TEST} ... ok"))
                    && stdout
                        .contains("test result: ok. 1 passed; 0 failed; 0 ignored; 0 measured;")
            );
            return;
        }

        fn simulation(count: usize) -> Simulation {
            let fixture = include_str!("../../gz-sim-scene/tests/fixtures/camera-binding.sdf");
            let start = fixture.find("<model name=\"robot\">").unwrap();
            let end = start + fixture[start..].find("</model>").unwrap() + "</model>".len();
            let mut models = String::new();
            for index in 0..count {
                models += &fixture[start..end]
                    .replace("name=\"robot\"", &format!("name=\"robot_{index}\""))
                    .replace("/explicit-camera", &format!("/runtime_rgb_{index}/image"));
            }
            let xml = (fixture[..start].to_owned() + &models + &fixture[end..]).replace(
                "<world name=\"default\">",
                "<world name=\"default\"><plugin name='gz::sim::systems::Sensors' filename='gz-sim-sensors-system'/>",
            );
            gz_sim_scene::load_world_with_sdf_step(
                &xml,
                "runtime-family-preflight",
                "default",
                &sdformat::NoResources,
                gz_sim_scene::PhysicsEngine::Tpe,
            )
            .unwrap()
        }

        let construct = |count, family| {
            // An intentionally relative path is rejected by TrustedWorker's
            // pure preflight. Neither the worker nor a Zenoh peer can start.
            Runtime::new(
                simulation(count),
                "default",
                Some(PathBuf::from("relative-worker-must-not-launch")),
                false,
                family,
            )
            .err()
            .expect("preflight must refuse before creating runtime resources")
        };
        for count in [0, 1, 2, 5] {
            let error = construct(count, true);
            assert!(
                matches!(
                    error.downcast_ref::<CameraServerError>(),
                    Some(CameraServerError::Camera(CameraOutputError::RgbFamilyCount))
                ),
                "family count={count}: {error}"
            );
        }
        for count in [3, 4] {
            let error = construct(count, true);
            assert!(
                matches!(
                    error.downcast_ref::<CameraServerError>(),
                    Some(CameraServerError::Camera(CameraOutputError::Client(_)))
                ),
                "admitted family count={count}: {error}"
            );
            assert_eq!(error.to_string(), "renderer client: InvalidWorkerPath");
        }
        // The same two-camera input still chooses the mature pair route when
        // the new flag is false; a three-camera input is not silently widened.
        let pair = construct(2, false);
        assert!(matches!(
            pair.downcast_ref::<CameraServerError>(),
            Some(CameraServerError::Camera(CameraOutputError::Client(_)))
        ));
        assert_eq!(pair.to_string(), "renderer client: InvalidWorkerPath");
        let three = construct(3, false);
        assert!(matches!(
            three.downcast_ref::<CameraServerError>(),
            Some(CameraServerError::Camera(
                CameraOutputError::MultipleCameras
            ))
        ));
    }
}
