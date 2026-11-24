// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
pub(super) const NORMAL_LIFT: f64 = 2.35;
const LIFT_INCREMENT: f64 = 0.45;
const SAFE_LIFT: f64 = 2.80;
const BACKOFF: f64 = 0.10;
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
}
#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub(super) struct Event {
    pub tick: usize,
    pub stage: Stage,
    pub carriage_y_m: f64,
    pub bar_force_n: f64,
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
    motion: RailMotion,
    dwell: usize,
}
impl Controller {
    pub fn new() -> Self {
        Self {
            stage: Stage::Pickup,
            events: vec![],
            abort: false,
            lift: NORMAL_LIFT,
            stage_start: 0,
            motion: RailMotion {
                from: 0.,
                to: 0.,
                start: 0,
                duration: 1,
            },
            dwell: 0,
        }
    }
    pub fn drive(&self, robot: &Robot, sim: &mut Simulation, tick: usize) -> Result<()> {
        if self.stage == Stage::Pickup {
            return robot.drive(sim, tick, false, None, NORMAL_LIFT);
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
        let force = 100. * (target - rail.position) + 28. * (speed - rail.velocity);
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
    fn enter(&mut self, stage: Stage, tick: usize, y: f64, s: &Sample, to: f64, duration: usize) {
        self.events.push(Event {
            tick,
            stage,
            carriage_y_m: y,
            bar_force_n: s.bar,
            spindle_rad: s.spindle,
        });
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
        let rail_done =
            tick >= self.motion.start + self.motion.duration && (y - self.motion.to).abs() < 0.003;
        match self.stage {
            Stage::Pickup if tick >= 5400 => {
                if s.squeeze.iter().any(|&p| p < 0.05) || s.source > 0.001 {
                    return Err("cable pickup failed".into());
                }
                self.enter(Stage::Transport, tick, y, s, 0.46, 4000);
            }
            Stage::Transport if s.bar > FORCE_TRIGGER => {
                // Retreat from the measured event; obstacle dimensions and position
                // are unavailable to the controller.
                self.abort = self.lift + LIFT_INCREMENT > SAFE_LIFT + 1e-9;
                self.enter(Stage::Backoff, tick, y, s, (y - BACKOFF).max(0.), 1200);
            }
            Stage::Backoff if rail_done && s.bar < 0.001 => {
                if self.abort {
                    self.enter(Stage::Returning, tick, y, s, 0., 3500);
                } else {
                    self.lift += LIFT_INCREMENT;
                    self.enter(Stage::Raising, tick, y, s, y, 1);
                }
            }
            Stage::Raising
                if (s.spindle - self.lift).abs() < 0.005
                    && tick - self.stage_start >= 1000
                    && s.bar < 0.001 =>
            {
                self.enter(Stage::Transport, tick, y, s, 0.46, 4000);
            }
            Stage::Transport | Stage::Returning if rail_done => {
                self.enter(Stage::Placing, tick, y, s, self.motion.to, 1);
            }
            Stage::Placing => {
                let force = if self.abort { s.source } else { s.shelf };
                if force > 0.9 * WEIGHT {
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
    pub fn done(&self, tick: usize) -> bool {
        self.stage == Stage::Withdrawn && tick - self.stage_start >= 2500
    }
}
