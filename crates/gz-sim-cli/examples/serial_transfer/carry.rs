// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::prelude::Resource;
use gz_sim_scene::grasp_guard::{GraspGuard, GraspResponse};
use gz_sim_scene::receiving_support::SupportReading;

#[derive(Resource)]
struct Travel(usize);
pub(super) fn configure(sim: &mut Simulation, trial: Trial, ms: u64) -> Result<()> {
    if matches!(trial, Trial::GuardedCarry | Trial::ObserveCarry) {
        if !(1..=4000).contains(&ms) {
            return Err("carry duration must be 1 to 4000 ms".into());
        }
        if sim.step_duration() != Duration::from_millis(1) {
            return Err("serial carry trial requires the authored 1 ms clock".into());
        }
        sim.world_mut()
            .insert_resource(Travel(usize::try_from(ms)?));
    } else if ms != 4000 {
        return Err("--travel-ms applies to guarded-carry and observe-carry".into());
    }
    Ok(())
}

#[derive(Debug, Default, PartialEq, Serialize)]
pub(super) struct Measurements {
    travel_ms: usize,
    commanded_peak_acceleration_m_s2: f64,
    observed_peak_carriage_acceleration_m_s2: f64,
    observed_peak_forward_acceleration_m_s2: f64,
    observed_peak_braking_m_s2: f64,
    peak_pad_slip_m_s: [f64; 2],
    first_alarm_ms: Option<usize>,
    repeated_alarm_ms: Option<usize>,
    extra_jaw_effort_n: f64,
    hold_on_blocked_return_m: Option<f64>,
    return_station: Option<&'static str>,
    lowering_ms: Option<usize>,
    first_recovery_support_ms: Option<usize>,
    recovery_airborne_ticks: usize,
    recovery_no_pad_ticks: usize,
    recovery_one_pad_ticks: usize,
    supported_open_ms: Option<usize>,
    sustained_at_open_ms: Option<u128>,
    load_at_open_n: Option<f64>,
    parcel_obstacle_impulse_ns: f64,
    support_upward_impulse_after_open_ns: f64,
}
struct Return {
    tick: usize,
    start: f64,
    station: usize,
    demand: Option<f64>,
}
pub(super) struct Links {
    pub(super) body: BodyId,
    pub(super) pads: [BodyId; 2],
    pub(super) stations: [BodyId; 3],
    pub(super) jaws: [u64; 2],
    pub(super) spindle: Part,
    pub(super) rail: Part,
}
pub(super) struct Supervisor {
    feedback: bool,
    guard: GraspGuard,
    route: Option<Return>,
    weight: f64,
    links: Links,
    last_velocity: Option<f64>,
    pub(super) report: Measurements,
}
const DISTANCE: f64 = 0.46;
const RETURN_STEPS: usize = 2000;
impl Supervisor {
    pub(super) fn new(sim: &Simulation, trial: Trial, weight: f64, links: Links) -> Result<Self> {
        let duration = sim
            .world()
            .get_resource::<Travel>()
            .ok_or("missing carry command duration")?
            .0;
        Ok(Self {
            feedback: trial == Trial::GuardedCarry,
            guard: GraspGuard::default(),
            route: None,
            weight,
            links,
            last_velocity: None,
            report: Measurements {
                travel_ms: duration,
                commanded_peak_acceleration_m_s2: (10. / 3_f64.sqrt()) * DISTANCE
                    / (duration as f64 * sim.step_duration().as_secs_f64()).powi(2),
                ..Default::default()
            },
        })
    }
    pub(super) fn recovering(&self) -> bool {
        self.route.is_some()
    }
    pub(super) fn remaining(&self, tick: usize, travel: f64, goal: f64) -> Option<f64> {
        if let Some(r) = &self.route {
            (self.report.lowering_ms.is_none() && self.report.hold_on_blocked_return_m.is_none())
                .then_some(r.station as f64 * DISTANCE - travel)
        } else {
            (MOVE_START..MOVE_END)
                .contains(&tick)
                .then_some((goal - travel).max(0.))
        }
    }
    pub(super) fn event(&self, tick: usize) -> bool {
        [
            self.report.first_alarm_ms,
            self.report.repeated_alarm_ms,
            self.report.lowering_ms,
            self.report.supported_open_ms,
        ]
        .contains(&Some(tick))
    }
    pub(super) fn command_rail(
        &self,
        sim: &mut Simulation,
        rail: Part,
        tick: usize,
        goal: f64,
        dt: f64,
    ) -> Result<()> {
        if let Some(r) = &self.route {
            if let Some(at) = self.report.hold_on_blocked_return_m {
                return drive_rail(sim, rail, at, at, 0, RETURN_STEPS, dt);
            }
            let dest = r.station as f64 * DISTANCE;
            drive_rail(sim, rail, r.start, dest, tick - r.tick, RETURN_STEPS, dt)
        } else {
            drive_rail(
                sim,
                rail,
                0.,
                goal,
                tick.saturating_sub(MOVE_START),
                self.report.travel_ms,
                dt,
            )
        }
    }
    pub(super) fn update(
        &mut self,
        sim: &mut Simulation,
        tick: usize,
        travel: f64,
        supports: [&SupportReading; 2],
        blocked: bool,
    ) -> Result<()> {
        if tick < MOVE_START {
            return Ok(());
        }
        let tactile =
            rigid_tactile::tactile(sim, self.links.body, self.links.pads, self.links.stations)?;
        self.report.parcel_obstacle_impulse_ns += tactile.station_ns[2];
        let dt = sim.step_duration();
        let velocity = sim
            .world()
            .get::<JointSample>(self.links.rail.entity)
            .ok_or("missing carry velocity")?
            .state
            .velocity;
        if (MOVE_START..=MOVE_END).contains(&tick) && !self.recovering() {
            if let Some(last) = self.last_velocity {
                let acceleration = (velocity - last) / dt.as_secs_f64();
                self.report.observed_peak_carriage_acceleration_m_s2 = self
                    .report
                    .observed_peak_carriage_acceleration_m_s2
                    .max(acceleration.abs());
                self.report.observed_peak_forward_acceleration_m_s2 = self
                    .report
                    .observed_peak_forward_acceleration_m_s2
                    .max(acceleration);
                self.report.observed_peak_braking_m_s2 =
                    self.report.observed_peak_braking_m_s2.max(-acceleration);
            }
            self.last_velocity = Some(velocity);
            for (peak, pad) in self.report.peak_pad_slip_m_s.iter_mut().zip(tactile.pads) {
                *peak = peak.max(pad.slip_speed_m_s);
            }
            if !blocked {
                let response = self.guard.observe(dt, tactile.pads);
                if response != GraspResponse::Hold && self.report.first_alarm_ms.is_none() {
                    self.report.first_alarm_ms = Some(tick);
                    if self.feedback {
                        // Add one measured preload per jaw, bounded by the existing 20 N open command.
                        let extra = (0.5
                            * (tactile.pads[0].normal_force_n + tactile.pads[1].normal_force_n))
                            .clamp(0., 20.);
                        JointForceCommand::set_batch(sim, self.links.jaws.map(|id| (id, extra)))?;
                        self.report.extra_jaw_effort_n = extra;
                    }
                }
                if response == GraspResponse::SetDown && self.report.repeated_alarm_ms.is_none() {
                    self.report.repeated_alarm_ms = Some(tick);
                    if self.feedback {
                        let station = usize::from(travel > DISTANCE / 2.);
                        self.report.return_station = Some(["pickup", "receiver"][station]);
                        self.route = Some(Return {
                            tick,
                            start: travel,
                            station,
                            demand: None,
                        });
                    }
                }
            }
        }
        let Some(r) = &mut self.route else {
            return Ok(());
        };
        if self.report.hold_on_blocked_return_m.is_some() {
            return Ok(());
        }
        if blocked && self.report.supported_open_ms.is_none() {
            let q = spindle_position(sim, self.links.spindle)?;
            set_joint_controller_command(sim, self.links.spindle.id, q)?;
            self.report.hold_on_blocked_return_m = Some(travel);
            return Ok(());
        }
        let target = r.station as f64 * DISTANCE;
        let support = supports[r.station];
        if self.report.first_recovery_support_ms.is_none() {
            if support.face_loaded {
                self.report.first_recovery_support_ms = Some(tick);
            } else {
                self.report.recovery_airborne_ticks += 1;
                let loaded = tactile
                    .pads
                    .iter()
                    .filter(|p| p.normal_force_n > 0.1)
                    .count();
                self.report.recovery_no_pad_ticks += usize::from(loaded == 0);
                self.report.recovery_one_pad_ticks += usize::from(loaded == 1);
            }
        }
        if self.report.lowering_ms.is_none()
            && tick >= r.tick + RETURN_STEPS
            && (travel - target).abs() < 0.003
            && velocity.abs() < 0.01
        {
            set_joint_controller_command(sim, self.links.spindle.id, -0.075)?;
            self.report.lowering_ms = Some(tick);
        }
        if let Some(open) = self.report.supported_open_ms {
            self.report.support_upward_impulse_after_open_ns += tactile.station_up_ns[r.station];
            if tick == open + 250 {
                set_joint_controller_command(sim, self.links.spindle.id, 1.6)?;
            }
        } else if self.report.lowering_ms.is_some() && (support.face_loaded || r.demand.is_some()) {
            let q = spindle_position(sim, self.links.spindle)?;
            let demand = r.demand.get_or_insert(q);
            *demand += (0.003 * (support.normal_force_n - self.weight)).clamp(-0.008, 0.008)
                * dt.as_secs_f64();
            set_joint_controller_command(sim, self.links.spindle.id, *demand)?;
            if support.ready {
                self.report.sustained_at_open_ms = Some(support.sustained.as_millis());
                self.report.load_at_open_n = support.mean_force_n;
                self.report.supported_open_ms = Some(tick);
                set_joint_controller_command(sim, self.links.spindle.id, q)?;
                JointForceCommand::set_batch(sim, self.links.jaws.map(|id| (id, -20.)))?;
            }
        }
        Ok(())
    }
}

fn spindle_position(sim: &Simulation, spindle: Part) -> Result<f64> {
    Ok(sim
        .world()
        .get::<JointSample>(spindle.entity)
        .ok_or("missing recovery spindle reading")?
        .state
        .position)
}

#[cfg(test)]
#[path = "carry/tests.rs"]
mod tests;
