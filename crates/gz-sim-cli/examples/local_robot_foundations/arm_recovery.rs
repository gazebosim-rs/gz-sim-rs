// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Transfer the existing two-pad tactile guard to the rotary arm.
use super::arm::{Controller, SCENE};
use super::envelope::{tactile, with_load};
use super::gripper::{body, ramp};
use super::*;
use gz_physics_multifidelity::MultiFidelityBackend;
use gz_sim_scene::grasp_guard::{GraspGuard, GraspResponse, SupportedRelease};
use serde::Serialize;

#[path = "mid_swing.rs"]
pub(super) mod mid_swing;

const TURN: f64 = std::f64::consts::FRAC_PI_2;
#[derive(Clone, Copy)]
struct Park {
    tick: usize,
    yaw: f64,
    height: f64,
    goal: f64,
}
impl Park {
    fn actuate(
        self,
        c: &Controller,
        r: &mut Recording,
        tick: usize,
        release: Option<usize>,
    ) -> Result<[f64; 4]> {
        let t = (tick - self.tick) as f64 * 0.001;
        let (turn, wy) = ramp(t, 0., 0.85);
        let (lower, vz) = ramp(t, 0.85, 0.65);
        let (open, vo) = release.map_or((0., 0.), |start| {
            ramp((tick - start) as f64 * 0.001, 0., 0.17)
        });
        let jaw = (0.074 * (1. - open), -0.074 * vo, 5.85 * (1. - open));
        c.actuate(
            &mut r.sim,
            [
                (
                    self.yaw + (self.goal - self.yaw) * turn,
                    (self.goal - self.yaw) * wy,
                    0.,
                ),
                (self.height * (1. - lower), -self.height * vz, 0.96 * 9.81),
                jaw,
                jaw,
            ],
            true,
        )
    }
}

#[derive(Debug, Serialize)]
struct Trial {
    mass_kg: f64,
    friction: f64,
    feedback: bool,
    alarm_tick: Option<usize>,
    independent_slip_tick: Option<usize>,
    set_down_tick: Option<usize>,
    release_tick: Option<usize>,
    outcome: &'static str,
    placement_error_m: f64,
    obstacle_impulse_ns: f64,
    floor_impulse_ns: f64,
    settled_source_ticks: usize,
    settled_target_ticks: usize,
    settled_jaw_ticks: usize,
    min_transfer_height_m: f64,
    max_jaw_command_n: f64,
    opposed_transfer_ticks: usize,
}
impl Trial {
    fn qualify(&self, label: &str) -> Result<()> {
        let expected = match label {
            "recovery" => "placed",
            "fixed" => "miss",
            "overload" => "set_down",
            _ => return Err(format!("unknown rotary grasp witness {label}").into()),
        };
        if self.outcome != expected || self.alarm_tick.is_none() || self.max_jaw_command_n > 9. {
            return Err(format!("rotary gripper {label} qualification failed: {self:?}").into());
        }
        Ok(())
    }
}

fn trial(mass: f64, mu: f64, feedback: bool, path: Option<&Path>) -> Result<Trial> {
    trial_with_tug(mass, mu, feedback, path, None)
}

fn trial_with_tug(
    mass: f64,
    mu: f64,
    feedback: bool,
    path: Option<&Path>,
    mut swing: Option<&mut mid_swing::Swing>,
) -> Result<Trial> {
    let xml = with_load(SCENE, mass, 0., mu);
    let mut r = Recording::new(&xml, "local_manipulation_arm.sdf", "default")?;
    let c = Controller::new(&r.sim)?;
    let links = [
        "arm::left",
        "arm::right",
        "source::body",
        "target::body",
        "floor::body",
        "obstacle::body",
        "parcel::box",
        "arm::palm",
    ]
    .iter()
    .map(|n| entity_id(&r.sim, &format!("default::{n}")))
    .collect::<Result<Vec<_>>>()?;
    let ids = links
        .iter()
        .map(|id| body(&r.sim, *id))
        .collect::<Result<Vec<_>>>()?;
    let mut outcome = Trial {
        mass_kg: mass,
        friction: mu,
        feedback,
        alarm_tick: None,
        independent_slip_tick: None,
        set_down_tick: None,
        release_tick: None,
        outcome: "miss",
        placement_error_m: 0.,
        obstacle_impulse_ns: 0.,
        floor_impulse_ns: 0.,
        settled_source_ticks: 0,
        settled_target_ticks: 0,
        settled_jaw_ticks: 0,
        min_transfer_height_m: f64::INFINITY,
        max_jaw_command_n: 0.,
        opposed_transfer_ticks: 0,
    };
    let mut guard = GraspGuard::default();
    let mut stop: Option<Park> = None;
    let mut release_gate = SupportedRelease::default();
    let mut initial_relative = 0.;
    if path.is_some() {
        r.capture()?;
    }
    for tick in 1..=6100 {
        let t = (tick - 1) as f64 * 0.001;
        let efforts = if let Some(park) = stop {
            park.actuate(&c, &mut r, tick, outcome.release_tick)?
        } else {
            let extra = if feedback && outcome.alarm_tick.is_some() {
                0.009
            } else {
                0.
            };
            c.command_with_grip(&mut r.sim, t, false, extra)?
        };
        outcome.max_jaw_command_n = outcome
            .max_jaw_command_n
            .max(efforts[2].abs())
            .max(efforts[3].abs());
        if let Some(s) = swing.as_deref_mut() {
            s.pull(&mut r.sim, links[6], tick)?;
        }
        r.advance()?;
        let sample = tactile(&r.sim, ids[6], [ids[0], ids[1]], [ids[2], ids[3], ids[4]])?;
        outcome.floor_impulse_ns += sample.station_ns[2];
        let backend = r
            .sim
            .world()
            .resource::<PhysicsBackend>()
            .downcast_ref::<MultiFidelityBackend>()
            .ok_or("missing local solver")?;
        for pair in backend.rigid_contact_observations() {
            if pair.a == ids[5] || pair.b == ids[5] {
                outcome.obstacle_impulse_ns += pair.normal_impulse;
                if (pair.a == ids[6] || pair.b == ids[6])
                    && let Some(s) = swing.as_deref_mut()
                {
                    s.parcel_obstacle_impulse_ns += pair.normal_impulse;
                }
            }
        }
        let p = state(&r.sim, links[6])?;
        let palm = state(&r.sim, links[7])?;
        let relative = p.position[2] - palm.position[2];
        if let Some(s) = swing.as_deref_mut() {
            s.observe(&r.sim, &links, tick, p, palm, &sample)?;
        }
        if tick == 700 {
            initial_relative = relative;
        }
        if tick > 700 && tick < 3320 && stop.is_none() {
            if initial_relative - relative > 0.00025 {
                outcome.independent_slip_tick.get_or_insert(tick);
            }
            let command = guard.observe(r.sim.step_duration(), sample.pads);
            if command != GraspResponse::Hold {
                outcome.alarm_tick.get_or_insert(tick);
            }
            if feedback && command == GraspResponse::SetDown {
                let yaw = palm.position[1].atan2(palm.position[0]);
                stop = Some(Park {
                    tick,
                    yaw,
                    height: palm.position[2] - 0.315,
                    goal: if yaw > TURN / 2. { TURN } else { 0. },
                });
                outcome.set_down_tick = Some(tick);
            }
        }
        if let Some(park) = stop {
            let support = sample.station_up_ns[usize::from(park.goal > 0.)];
            if tick > park.tick + 1500
                && release_gate.observe(r.sim.step_duration(), support, mass * 9.81)
            {
                outcome.release_tick.get_or_insert(tick);
            }
        } else if tick >= 3320 {
            outcome.release_tick.get_or_insert(tick);
        }
        if (1401..=2500).contains(&tick) {
            outcome.opposed_transfer_ticks +=
                usize::from(sample.pads.iter().all(|s| s.normal_force_n > 0.05));
            outcome.min_transfer_height_m = outcome.min_transfer_height_m.min(p.position[2]);
        }
        if tick > 5900 {
            outcome.settled_source_ticks += usize::from(sample.station_ns[0] > 0.);
            outcome.settled_target_ticks += usize::from(sample.station_ns[1] > 0.);
            outcome.settled_jaw_ticks +=
                usize::from(sample.pads.iter().any(|s| s.normal_force_n > 0.));
        }
        if let Some(s) = swing.as_deref_mut() {
            s.record_recovery(
                tick,
                stop,
                outcome.release_tick,
                &sample,
                r.sim.step_duration(),
            );
        }
        if path.is_some()
            && (tick % 50 == 0
                || swing
                    .as_ref()
                    .is_some_and(|s| s.capture_tick(tick, &outcome)))
        {
            r.capture()?;
            if let Some(s) = swing.as_deref_mut() {
                s.label_frame(tick, &outcome, r.frames.len() - 1);
            }
        }
    }
    let source = stop.is_some_and(|park| park.goal == 0.);
    let goal = if source {
        DVec3::new(0.32, 0., 0.17)
    } else {
        DVec3::new(0., 0.32, 0.17)
    };
    outcome.placement_error_m = DVec3::from(state(&r.sim, links[6])?.position).distance(goal);
    let stand = if source {
        outcome.settled_source_ticks
    } else {
        outcome.settled_target_ticks
    };
    let carried = stop.is_some()
        || (outcome.opposed_transfer_ticks >= 1078 && outcome.min_transfer_height_m > 0.28);
    if carried
        && stand >= 190
        && outcome.release_tick.is_some()
        && outcome.settled_jaw_ticks == 0
        && outcome.placement_error_m < 0.012
        && outcome.floor_impulse_ns == 0.
        && outcome.obstacle_impulse_ns == 0.
    {
        outcome.outcome = if stop.is_some() { "set_down" } else { "placed" };
    }
    if let Some(dir) = path {
        r.save(dir)?;
        write_json(&dir.join("trial.json"), &json!(outcome))?;
        let captures = swing.as_ref().map_or_else(
            || json!({"grip":14,"response":outcome.alarm_tick.unwrap_or(1200).div_ceil(50),"carry":38,"placed":122}),
            |s| json!(s.captures),
        );
        write_json(
            &dir.join("frames.json"),
            &json!({"kind":"authored-archive-v1","camera":"perspective","dt_ns":1,"captures":captures}),
        )?;
    }
    Ok(outcome)
}

pub(super) fn run(output: &Path) -> Result<()> {
    let mut cases = vec![];
    for mass in [0.2, 0.45, 0.55, 0.65, 0.85] {
        for mu in [0.35, 0.45, 0.7] {
            for enabled in [false, true] {
                let tag = if mass == 0.45 && mu == 0.35 {
                    Some(if enabled { "recovery" } else { "fixed" })
                } else if mass == 0.85 && mu == 0.35 && enabled {
                    Some("overload")
                } else {
                    None
                };
                let path = tag.map(|label| output.join(label));
                let result = trial(mass, mu, enabled, path.as_deref())?;
                if let Some(label) = tag {
                    result.qualify(label)?;
                }
                cases.push(result);
            }
        }
    }
    write_json(&output.join("matrix.json"), &json!(cases))
}

#[cfg(test)]
#[path = "arm_recovery_tests.rs"]
mod tests;
