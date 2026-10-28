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

pub(super) fn run(
    mut simulation: Simulation,
    world: &str,
    running: bool,
    iterations: u64,
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
    let mut server = Server::new(
        simulation,
        world,
        gz_transport::Config::from_env()?,
        NonZeroUsize::new(64).expect("nonzero command budget"),
    )?;
    server.prepare_run(Duration::from_secs(5))?;
    let mut completed = 0_u64;
    while !termination.requested() && (iterations == 0 || completed < iterations) {
        let started = Instant::now();
        let info = server.tick()?;
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
    Ok((server.into_simulation(), completed))
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
