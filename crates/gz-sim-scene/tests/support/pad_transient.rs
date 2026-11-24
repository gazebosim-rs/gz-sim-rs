// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0

use super::{MASS as PARCEL_MASS, Rig, STIFF_PADS, overlaps};
use gz_physics::PhysicsBackend;

const PAD_MASS: f64 = 0.08;
const STIFFNESS: f64 = 30_000.0;

#[derive(Clone, Copy, Default)]
struct Motion {
    compression: f64,
    speed: f64,
}

struct Response {
    drive: f64,
    offset: f64,
    rates: [f64; 2],
    amplitudes: [f64; 2],
    damping: f64,
    detach: Option<(f64, Motion)>,
}

impl Response {
    fn new(start: Motion, drive: f64, duration: f64) -> Self {
        // Each point damps relative motion using the pair's reduced mass.
        // The centered, equally driven jaws leave the sphere stationary, so
        // the inertia in each jaw's equation of motion is the full pad mass.
        let row_mass = PAD_MASS * PARCEL_MASS / (PAD_MASS + PARCEL_MASS);
        let damping = 2.0 * 10.0 * (STIFFNESS * row_mass).sqrt();
        let discriminant = (damping * damping - 4.0 * PAD_MASS * STIFFNESS).sqrt();
        let fast = -(damping + discriminant) / (2.0 * PAD_MASS);
        let slow = STIFFNESS / (PAD_MASS * fast);
        let offset = drive / STIFFNESS;
        let displacement = start.compression - offset;
        let fast_amplitude = (start.speed - slow * displacement) / (fast - slow);
        let mut response = Self {
            drive,
            offset,
            rates: [slow, fast],
            amplitudes: [displacement - fast_amplitude, fast_amplitude],
            damping,
            detach: None,
        };
        // For the opening command the compressive force has one zero crossing.
        // After that instant the two faces are mechanically unloaded even
        // during the short interval before their geometric overlap disappears.
        if response.normal(duration) < 0.0 {
            let (mut left, mut right) = (0.0, duration);
            for _ in 0..52 {
                let mid = 0.5 * (left + right);
                if response.normal(mid) > 0.0 {
                    left = mid;
                } else {
                    right = mid;
                }
            }
            response.detach = Some((right, response.coupled(right)));
        }
        response
    }

    fn coupled(&self, t: f64) -> Motion {
        let slow = self.amplitudes[0] * (self.rates[0] * t).exp();
        let fast = self.amplitudes[1] * (self.rates[1] * t).exp();
        Motion {
            compression: self.offset + slow + fast,
            speed: self.rates[0] * slow + self.rates[1] * fast,
        }
    }

    fn normal(&self, t: f64) -> f64 {
        let state = self.coupled(t);
        STIFFNESS * state.compression + self.damping * state.speed
    }

    fn at(&self, t: f64) -> Motion {
        if let Some((release, at_release)) = self.detach
            && t > release
        {
            let free_time = t - release;
            let accel = self.drive / PAD_MASS;
            return Motion {
                compression: at_release.compression
                    + free_time * (at_release.speed + 0.5 * accel * free_time),
                speed: at_release.speed + accel * free_time,
            };
        }
        self.coupled(t)
    }
}

#[test]
fn pad_loading_unloading_and_separation_follow_unilateral_spring_damper() {
    let mut previous_error = [f64::INFINITY; 3];
    for step_us in [200, 100, 50, 25] {
        let mut rig = Rig::new(STIFF_PADS, step_us, 0.0);
        let dt = rig.sim.step_duration().as_secs_f64();
        let mut start = Motion::default();
        let mut error = [0.0_f64; 3];
        // Begin at first touch, reduce the squeeze while contact stays loaded,
        // then reverse the drive to test actual separation and free travel.
        for (drive, duration_us) in [(3.0, 120_000), (1.5, 8_000), (-3.0, 6_000)] {
            let duration = f64::from(duration_us) * 1e-6;
            let solution = Response::new(start, drive, duration);
            rig.drive([drive; 2]);
            let mut last = start;
            let mut phase_error = [0.0_f64; 3];
            let mut first_unloaded = None;
            let mut first_95pct = None;
            for i in 1..=duration_us / step_us {
                rig.sim.step_exact(1).unwrap();
                let t = f64::from(i) * dt;
                let expected = solution.at(t);
                // Newton's law integrated over the native public tick, so the
                // reference is an interval impulse rather than a point force.
                let impulse = drive * dt - PAD_MASS * (expected.speed - last.speed);
                let depth = overlaps(&rig);
                let backend = rig.sim.world().resource::<PhysicsBackend>();
                let parcel_vx = rig.state().velocity[0];
                let reaction = rig.read().normal;
                for side in 0..2 {
                    let inward = if side == 0 { 1.0 } else { -1.0 };
                    let pad_vx = backend.state(rig.pads[side]).unwrap().velocity[0];
                    let speed = inward * (pad_vx - parcel_vx);
                    phase_error[0] = phase_error[0].max((depth[side] - expected.compression).abs());
                    phase_error[1] = phase_error[1].max((speed - expected.speed).abs());
                    phase_error[2] = phase_error[2].max((reaction[side] - impulse).abs());
                }
                assert!(reaction.iter().all(|f| *f >= 0.0));
                if drive == 3.0 && depth.iter().all(|x| *x >= 0.95 * drive / STIFFNESS) {
                    first_95pct.get_or_insert(t);
                }
                if let Some((release, _)) = solution.detach {
                    if reaction == [0.0; 2] {
                        first_unloaded.get_or_insert(t);
                    }
                    if t >= release + 2.0 * dt {
                        assert_eq!(reaction, [0.0; 2], "pads pulling at {t} s");
                    }
                }
                last = expected;
            }
            if drive == 3.0 {
                let reached = first_95pct.expect("both pads should reach 95% compression");
                let target = 0.95 * drive / STIFFNESS;
                assert!(solution.coupled(reached + dt).compression >= target);
                assert!(solution.coupled(reached - 2.0 * dt).compression <= target);
                println!("pad_95pct step_us={step_us} time_s={reached:.9}");
            }
            if let Some((release, _)) = solution.detach {
                let measured = first_unloaded.expect("both pads must unload");
                assert!((measured - release).abs() < 2.0 * dt);
                assert!(overlaps(&rig).iter().all(|x| *x < 0.0));
                println!(
                    "pad_detach step_us={step_us} exact_s={release:.9} first_zero_s={measured:.9}"
                );
            }
            // An open jaw travels freely. Its integrated position error can
            // accumulate throughout the 6 ms interval following separation.
            let depth_tolerance = if drive < 0.0 { 5e-6 } else { 1e-7 };
            assert!(phase_error[0] < depth_tolerance);
            assert!(phase_error[1] < 5e-4);
            assert!(phase_error[2] / dt < 0.2);
            println!(
                "pad_response step_us={step_us} drive_n={drive} max_depth_error_m={:.9e} max_speed_error_m_s={:.9e} max_impulse_error_ns={:.9e} max_mean_force_error_n={:.9e}",
                phase_error[0],
                phase_error[1],
                phase_error[2],
                phase_error[2] / dt
            );
            for (largest, local) in error.iter_mut().zip(phase_error) {
                *largest = largest.max(local);
            }
            start = last;
        }
        for (current, previous) in error.into_iter().zip(previous_error) {
            assert!(
                current < previous,
                "refinement failed at {step_us} µs: {error:?}"
            );
        }
        previous_error = error;
    }
}

/// Normal reaction of each pad to a unit inward step, starting at first touch.
pub(super) fn centered_squeeze_force(time_s: f64) -> f64 {
    Response::new(Motion::default(), 1.0, time_s).normal(time_s)
}
