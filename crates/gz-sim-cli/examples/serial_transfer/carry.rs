// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use bevy_ecs::prelude::Resource;
use gz_sim_scene::grasp_guard::{GraspGuard, GraspResponse};
#[cfg(test)]
use gz_sim_scene::receiving_support::SupportReading;

#[derive(Resource)]
struct Travel(usize);
#[cfg(test)]
pub(super) fn configure(sim: &mut Simulation, trial: Trial, ms: u64) -> Result<()> {
    configure_clock(sim, trial, ms, 1)
}
pub(super) fn configure_clock(
    sim: &mut Simulation,
    trial: Trial,
    ms: u64,
    control_ms: u64,
) -> Result<()> {
    if trial.carry() {
        if !(1..=4000).contains(&ms) {
            return Err("carry duration must be 1 to 4000 ms".into());
        }
        let cadence = cycle::Cadence::new(sim.step_duration(), control_ms)?;
        sim.world_mut().insert_resource(cadence);
        sim.world_mut()
            .insert_resource(Travel(usize::try_from(ms)?));
    } else if ms != 4000 || control_ms != 1 {
        return Err("--travel-ms and --control-ms require a monitored carry trial".into());
    }
    Ok(())
}

#[derive(Debug, Default, PartialEq, Serialize)]
pub(super) struct Measurements {
    travel_ms: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    cadence_us: Option<[u64; 2]>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    shelf_edge_lifts: Vec<EdgeLift>,
    #[serde(skip_serializing_if = "Option::is_none")]
    first_preload_reduction_ms: Option<usize>,
    #[serde(skip_serializing_if = "Option::is_none")]
    last_extra_preload_n: Option<f64>,
    commanded_peak_acceleration_m_s2: f64,
    observed_peak_carriage_acceleration_m_s2: f64,
    observed_peak_forward_acceleration_m_s2: f64,
    observed_peak_braking_m_s2: f64,
    peak_pad_slip_m_s: [f64; 2],
    pub(super) first_alarm_ms: Option<usize>,
    pub(super) repeated_alarm_ms: Option<usize>,
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
#[derive(Debug, PartialEq, Serialize)]
struct EdgeLift {
    first_contact_ms: usize,
    start_gaps_m: [f64; 2],
    initial_mean_upward_force_n: f64,
    initial_mean_total_normal_force_n: f64,
    resume_ms: Option<usize>,
    resume_gaps_m: Option<[f64; 2]>,
    clear_for_ms: usize,
    peak_spindle_position_rad: f64,
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
    stable_ms: usize,
    criteria: SupportCriteria,
    clear_ms: usize,
    control_dt: Duration,
    trial: Trial,
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
            stable_ms: 0,
            criteria: SupportCriteria::for_weight(weight, SUPPORT_DWELL),
            clear_ms: 0,
            control_dt: Duration::from_millis(
                sim.world().resource::<cycle::Cadence>().control_ms as u64,
            ),
            trial,
            guard: GraspGuard::default(),
            route: None,
            weight,
            links,
            last_velocity: None,
            report: Measurements {
                travel_ms: duration,
                cadence_us: {
                    let us = sim.step_duration().as_micros() as u64;
                    let period = sim.world().resource::<cycle::Cadence>().control_ms as u64 * 1000;
                    (us != 1000 || period != 1000).then_some([us, period])
                },
                commanded_peak_acceleration_m_s2: (10. / 3_f64.sqrt()) * DISTANCE
                    / (duration as f64 * 0.001).powi(2),
                ..Default::default()
            },
        })
    }
    pub(super) fn recovering(&self) -> bool {
        self.route.is_some()
    }
    pub(super) fn remaining(&self, tick: usize, travel: f64, goal: f64) -> Option<f64> {
        if let Some(r) = &self.route {
            (!self.raising()
                && self.report.lowering_ms.is_none()
                && self.report.hold_on_blocked_return_m.is_none())
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
            self.report
                .shelf_edge_lifts
                .last()
                .map(|e| e.first_contact_ms),
            self.report
                .shelf_edge_lifts
                .last()
                .and_then(|e| e.resume_ms),
        ]
        .contains(&Some(tick))
    }
    pub(super) fn verify_return(
        &mut self,
        sim: &mut Simulation,
        tick: usize,
        travel: f64,
        passage: &Passage,
    ) -> Result<()> {
        let Some(r) = self.route.as_mut().filter(|r| r.tick == tick) else {
            return Ok(());
        };
        for station in [r.station, 1 - r.station] {
            if passage.path_clear(sim, station as f64 * DISTANCE - travel)? {
                r.station = station;
                self.report.return_station = Some(["pickup", "receiver"][station]);
                return Ok(());
            }
        }
        let q = spindle_position(sim, self.links.spindle)?;
        set_joint_controller_command(sim, self.links.spindle.id, q)?;
        self.report.hold_on_blocked_return_m = Some(travel);
        self.report.return_station = None;
        Ok(())
    }
    fn raising(&self) -> bool {
        self.report
            .shelf_edge_lifts
            .last()
            .is_some_and(|r| r.resume_ms.is_none())
    }
    pub(super) fn check_shelf_edge(
        &mut self,
        sim: &mut Simulation,
        tick: usize,
        travel: f64,
        passage: &Passage,
        reading: &cycle::Reading,
    ) -> Result<()> {
        let lifting = self.raising();
        let Some(r) = &mut self.route else {
            return Ok(());
        };
        if self.trial != Trial::GuardedCarry
            || r.station != 1
            || self.report.lowering_ms.is_some()
            || self.report.hold_on_blocked_return_m.is_some()
        {
            return Ok(());
        }
        let impulse_ns = reading
            .tactile
            .as_ref()
            .expect("active carry reading")
            .station_ns[1];
        let both_pads = reading.both_pads_loaded;
        if !lifting
            && (impulse_ns <= 0.
                || reading.target.normal_force_n * self.control_dt.as_secs_f64()
                    >= self.criteria.minimum_alignment * impulse_ns
                || reading.target.face_loaded
                || !both_pads)
        {
            return Ok(());
        }
        let gaps = passage.receiver_gaps(sim, self.links.body)?;
        let q = spindle_position(sim, self.links.spindle)?;
        if !lifting {
            if gaps[1] >= 0. {
                return Ok(());
            }
            let max = sim
                .world()
                .get::<gz_sim_scene::JointDefinition>(self.links.spindle.entity)
                .and_then(|j| j.0.axis.as_ref())
                .map(|a| a.upper)
                .filter(|u| u.is_finite())
                .ok_or("missing finite spindle upper stop")?;
            set_joint_controller_command(sim, self.links.spindle.id, max)?;
            r.start = travel;
            self.clear_ms = 0;
            self.report.shelf_edge_lifts.push(EdgeLift {
                first_contact_ms: tick,
                start_gaps_m: gaps,
                initial_mean_upward_force_n: reading.target.normal_force_n,
                initial_mean_total_normal_force_n: impulse_ns / self.control_dt.as_secs_f64(),
                resume_ms: None,
                resume_gaps_m: None,
                clear_for_ms: 0,
                peak_spindle_position_rad: q,
            });
            return Ok(());
        }
        let log = self
            .report
            .shelf_edge_lifts
            .last_mut()
            .expect("active shelf clearance");
        log.peak_spindle_position_rad = log.peak_spindle_position_rad.max(q);
        // A lost grasp cannot become a successful set-down by resting on a different surface.
        if !both_pads {
            set_joint_controller_command(sim, self.links.spindle.id, q)?;
            self.report.hold_on_blocked_return_m = Some(travel);
            return Ok(());
        }
        self.clear_ms = if gaps[1] > 0. && impulse_ns == 0. {
            self.clear_ms + self.control_dt.as_millis() as usize
        } else {
            0
        };
        if self.clear_ms as u128 >= SUPPORT_DWELL.as_millis() {
            set_joint_controller_command(sim, self.links.spindle.id, q)?;
            log.resume_ms = Some(tick);
            log.resume_gaps_m = Some(gaps);
            log.clear_for_ms = self.clear_ms;
            r.start = travel;
            r.tick = tick;
        }
        Ok(())
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
            if let Some(at) = self
                .report
                .hold_on_blocked_return_m
                .or_else(|| self.raising().then_some(r.start))
            {
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
    pub(super) fn read_tactile(&self, sim: &Simulation) -> Result<rigid_tactile::Tactile> {
        rigid_tactile::tactile(sim, self.links.body, self.links.pads, self.links.stations)
    }
    #[cfg(test)]
    pub(super) fn update(
        &mut self,
        sim: &mut Simulation,
        tick: usize,
        travel: f64,
        supports: [&SupportReading; 2],
        blocked: bool,
    ) -> Result<()> {
        let t = self.read_tactile(sim)?;
        let reading = cycle::Reading {
            parcel_position: [0.; 3],
            carriage_y: travel,
            source: *supports[0],
            target: *supports[1],
            face_loaded_any: [supports[0].face_loaded, supports[1].face_loaded],
            stable_face: [false; 2],
            both_pads_loaded: false,
            pad: [0.; 2],
            tactile: Some(t),
        };
        self.update_with_reading(sim, tick, travel, &reading, blocked)
    }
    pub(super) fn update_with_reading(
        &mut self,
        sim: &mut Simulation,
        tick: usize,
        travel: f64,
        reading: &cycle::Reading,
        blocked: bool,
    ) -> Result<()> {
        if tick < MOVE_START {
            return Ok(());
        }
        let tactile = reading
            .tactile
            .as_ref()
            .ok_or("missing carry interval tactile observations")?;
        self.report.parcel_obstacle_impulse_ns += tactile.station_ns[2];
        let dt = self.control_dt;
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
                    if self.trial != Trial::ObserveCarry {
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
                    if self.trial != Trial::ObserveCarry {
                        let station = usize::from(travel > DISTANCE / 2.);
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
        let raising = self.raising();
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
        let support = [&reading.source, &reading.target][r.station];
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
        if !raising
            && self.report.lowering_ms.is_none()
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
            self.stable_ms = if reading.stable_face[r.station] {
                self.stable_ms + self.control_dt.as_millis() as usize
            } else {
                0
            };
            if self.trial == Trial::GuardedCarry
                && self.stable_ms as u128 >= SUPPORT_DWELL.as_millis()
                && support.sustained < SUPPORT_DWELL
                && !support.ready
            {
                // Return only the additional slip response to its nominal spring preload.
                // The normal 30 ms force gate still decides whether to open the fingers.
                let prior = self
                    .report
                    .last_extra_preload_n
                    .unwrap_or(self.report.extra_jaw_effort_n);
                let supported_fraction = (support.normal_force_n / self.weight).clamp(0., 1.);
                let decrease = self.report.extra_jaw_effort_n
                    * supported_fraction
                    * self.control_dt.as_secs_f64()
                    / SUPPORT_DWELL.as_secs_f64();
                let next = (prior - decrease).max(0.);
                if next < prior {
                    JointForceCommand::set_batch(sim, self.links.jaws.map(|id| (id, next)))?;
                    self.report.last_extra_preload_n = Some(next);
                    self.report.first_preload_reduction_ms.get_or_insert(tick);
                }
            }
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
