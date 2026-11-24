// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
pub(super) const NORMAL_LIFT: f64 = 2.35;
const LIFT_INCREMENT: f64 = 0.45;
const SAFE_LIFT: f64 = 2.80;
const BACKOFF: f64 = 0.10;
const RAIL_ASSEMBLY_MASS_KG: f64 = 2.46;
const BRAKE_DECAY_S: f64 = 0.004;
const BRAKE_LIMIT_N: f64 = 45.;
pub(super) const FORCE_TRIGGER: f64 = 0.15 * WEIGHT;
#[derive(PartialEq, Debug, Clone, Copy, Serialize)]
pub(super) enum Stage {
    Pickup,
    Transport,
    Backoff,
    Raising,
    Returning,
    Placing,
    Opening,
    Withdrawn,
    LostGrip,
    Unidentified,
}
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub(super) struct Event {
    pub tick: usize,
    pub stage: Stage,
    pub carriage_y_m: f64,
    #[serde(rename = "bar_force_n")]
    pub load_n: f64,
    pub spindle_rad: f64,
}
/// Signals needed by the route. Each observer obtains them from its own instruments.
#[derive(Clone, Copy)]
pub(super) struct Feedback {
    pub carried: bool,
    pub snag: bool,
    pub clear: bool,
    pub supported: bool,
    pub lost: bool,
    pub load_n: f64,
    pub spindle_rad: f64,
}
struct RailMotion {
    from: f64,
    to: f64,
    start: usize,
    duration: usize,
}
impl RailMotion {
    fn target(&self, tick: usize) -> (f64, f64) {
        let u = (tick.saturating_sub(self.start) as f64 / self.duration as f64).min(1.);
        let delta = self.to - self.from;
        (
            self.from + delta * u.powi(3) * (10. + u * (-15. + 6. * u)),
            delta * 30. * u.powi(2) * (1. - u).powi(2) / (self.duration as f64 * DT),
        )
    }
}
pub(super) struct Controller {
    pub stage: Stage,
    pub events: Vec<Event>,
    pub abort: bool,
    pub lift: f64,
    stage_start: usize,
    travel_ticks: usize,
    pub pickup_end_tick: usize,
    pub active_braking: bool,
    stopping_forward: bool,
    motion: RailMotion,
    dwell: usize,
}
impl Controller {
    pub fn new() -> Self {
        Self::with_travel_ticks(4000)
    }
    pub fn with_travel_ticks(travel_ticks: usize) -> Self {
        Self {
            stage: Stage::Pickup,
            events: vec![],
            abort: false,
            lift: NORMAL_LIFT,
            stage_start: 0,
            travel_ticks,
            pickup_end_tick: 5400,
            active_braking: false,
            stopping_forward: false,
            motion: RailMotion {
                from: 0.,
                to: 0.,
                start: 0,
                duration: 1,
            },
            dwell: 0,
        }
    }
    pub fn drive(&mut self, robot: &Robot, sim: &mut Simulation, tick: usize) -> Result<()> {
        if self.stage == Stage::Pickup {
            return robot.drive(sim, tick.min(5400), false, None, NORMAL_LIFT);
        }
        let entering = tick == self.stage_start + 1;
        if entering {
            if self.stage == Stage::Transport {
                set_joint_controller_command(sim, robot.wrist.id, 0.)?;
            }
            let target = match self.stage {
                Stage::Raising | Stage::Withdrawn => Some(self.lift),
                Stage::Placing => Some(if self.abort { -0.045 } else { 0.70 }),
                _ => None,
            };
            if let Some(q) = target {
                set_joint_controller_command(sim, robot.spindle.id, q)?;
            }
        }
        let rail = sim
            .world()
            .get::<JointSample>(robot.rail.entity)
            .ok_or("missing rail feedback")?
            .state;
        let (target, speed) = self.motion.target(tick);
        let mut force = 100. * (target - rail.position) + 28. * (speed - rail.velocity);
        // Once a contact or grasp-loss trip has latched, unload forward rail
        // momentum before following the retreat. This commands the real slider;
        // the brake never edits the measured velocity or any material state.
        self.stopping_forward &= rail.velocity > 0.;
        if self.stopping_forward {
            let brake = -(RAIL_ASSEMBLY_MASS_KG * rail.velocity / BRAKE_DECAY_S).min(BRAKE_LIMIT_N);
            force = force.min(brake);
        }
        let stroke = if matches!(self.stage, Stage::Opening | Stage::Withdrawn) {
            0.
        } else {
            0.034
        };
        let jaw = 300. * (stroke - 0.048);
        JointForceCommand::set_batch(
            sim,
            [
                (robot.rail.id, force.clamp(-80., 80.)),
                (robot.jaws[0].id, jaw),
                (robot.jaws[1].id, jaw),
            ],
        )?;
        Ok(())
    }
    fn enter(&mut self, stage: Stage, tick: usize, y: f64, s: &Feedback, to: f64, duration: usize) {
        self.events.push(Event {
            tick,
            stage,
            carriage_y_m: y,
            load_n: s.load_n,
            spindle_rad: s.spindle_rad,
        });
        self.stopping_forward = self.active_braking
            && matches!(
                stage,
                Stage::Backoff | Stage::LostGrip | Stage::Unidentified
            );
        self.stage = stage;
        self.stage_start = tick;
        self.dwell = 0;
        self.motion = RailMotion {
            from: y,
            to,
            start: tick,
            duration,
        };
    }
    pub fn observe(&mut self, tick: usize, y: f64, s: &Sample) -> Result<()> {
        self.apply(
            tick,
            y,
            &Feedback {
                carried: s.squeeze.iter().all(|&p| p >= 0.05) && s.source <= 0.001,
                snag: s.bar > FORCE_TRIGGER,
                clear: s.bar < 0.001,
                supported: (if self.abort { s.source } else { s.shelf }) > 0.9 * WEIGHT,
                lost: false,
                load_n: s.bar,
                spindle_rad: s.spindle,
            },
        )
    }
    pub fn apply(&mut self, tick: usize, y: f64, s: &Feedback) -> Result<()> {
        let rail_done =
            tick >= self.motion.start + self.motion.duration && (y - self.motion.to).abs() < 0.003;
        match self.stage {
            Stage::Pickup if tick >= self.pickup_end_tick => {
                if !s.carried {
                    return Err("cable pickup failed".into());
                }
                self.enter(Stage::Transport, tick, y, s, 0.46, self.travel_ticks);
            }
            Stage::Transport | Stage::Backoff | Stage::Raising | Stage::Returning if s.lost => {
                self.enter(Stage::LostGrip, tick, y, s, y, 1);
            }
            Stage::Transport if s.snag => {
                // Retreat from the measured event; obstacle dimensions and position
                // are unavailable to the controller.
                self.abort = self.lift + LIFT_INCREMENT > SAFE_LIFT + 1e-9;
                self.enter(Stage::Backoff, tick, y, s, (y - BACKOFF).max(0.), 1200);
            }
            Stage::Backoff if rail_done && s.clear => {
                if self.abort {
                    self.enter(Stage::Returning, tick, y, s, 0., 3500);
                } else {
                    self.lift += LIFT_INCREMENT;
                    self.enter(Stage::Raising, tick, y, s, y, 1);
                }
            }
            Stage::Raising
                if (s.spindle_rad - self.lift).abs() < 0.005
                    && tick - self.stage_start >= 1000
                    && s.clear =>
            {
                self.enter(Stage::Transport, tick, y, s, 0.46, self.travel_ticks);
            }
            Stage::Transport | Stage::Returning if rail_done => {
                self.enter(Stage::Placing, tick, y, s, self.motion.to, 1);
            }
            Stage::Placing => {
                if s.supported {
                    self.dwell += 1;
                } else {
                    self.dwell = 0;
                }
                if self.dwell == 30 {
                    self.enter(Stage::Opening, tick, y, s, self.motion.to, 1);
                }
            }
            Stage::Opening if tick - self.stage_start >= 375 => {
                self.enter(Stage::Withdrawn, tick, y, s, self.motion.to, 1)
            }
            _ => {}
        }
        Ok(())
    }
    pub fn limit_payload_momentum(&mut self, mass_kg: f64) -> usize {
        // The quintic rail profile peaks at 1.875 times its mean speed.
        // Bound commanded forward payload momentum to 4.8 mN s on the full move.
        let min_ticks = (1.875 * 0.46 * mass_kg / (0.0048 * DT)).ceil() as usize;
        self.travel_ticks = self.travel_ticks.max(min_ticks);
        self.travel_ticks
    }
    pub fn hold_unidentified(&mut self, tick: usize, y: f64, s: &Feedback) {
        self.enter(Stage::Unidentified, tick, y, s, y, 1);
    }
    pub fn done(&self, tick: usize) -> bool {
        matches!(
            self.stage,
            Stage::LostGrip | Stage::Withdrawn | Stage::Unidentified
        ) && tick - self.stage_start >= 2500
    }
}
