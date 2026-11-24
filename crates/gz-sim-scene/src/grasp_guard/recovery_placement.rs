// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::SupportedRelease;
use std::time::Duration;

/// Phase of an emergency placement on a vertical force-driven lift.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlacementPhase {
    Lower,
    Hold,
    Open,
    Retreat,
    Placed,
    SupportLost,
    /// The receiver caught a parcel that the jaws had failed to guide down.
    UnguidedLanding,
}

/// Observations from the interval which has just committed. Positive Z is up.
#[derive(Debug, Clone, Copy)]
pub struct PlacementReading {
    pub pad_height_m: f64,
    pub pad_velocity_m_s: f64,
    pub parcel_velocity_m_s: f64,
    /// Signed upward normal impulse from the specifically selected receiver.
    pub receiver_impulse_ns: Option<f64>,
    pub obstacle_impulse_ns: f64,
    /// Minimum horizontal distance from the parcel COM projection to a deck edge.
    pub deck_margin_m: f64,
    /// Minimum measured separation between either inside jaw face and the parcel.
    pub jaw_clearance_m: f64,
    pub jaw_pressure_n: [f64; 2],
}

/// Next commands, applied to both jaws and the lift after the public tick.
#[derive(Debug, Clone, Copy)]
pub struct PlacementDrive {
    pub phase: PlacementPhase,
    pub jaw_force_n: f64,
    pub lift_force_n: f64,
}

/// Force feedback for the opposed-jaw vertical carry rig in Earth gravity.
/// Start once per failed regrasp. Speed feedback brakes the upward carriage
/// before descent. Checked receiver support qualifies opening and final placement.
pub struct RecoveryPlacement {
    mass: f64,
    carriage: f64,
    lower_m: f64,
    speed: f64,
    phase: PlacementPhase,
    contact_seen: bool,
    guided_for: Duration,
    unsupported_for: Duration,
    blocked: Duration,
    blind: Duration,
    support: SupportedRelease,
    hold_height: f64,
    retreat_from: f64,
    last: PlacementDrive,
}
impl RecoveryPlacement {
    /// Positive masses in kg, minimum pad height in metres and positive descent speed in m/s.
    pub fn new(parcel_kg: f64, carriage_kg: f64, lower_m: f64, descent_m_s: f64) -> Option<Self> {
        if ![parcel_kg, carriage_kg, descent_m_s]
            .into_iter()
            .all(|v| v.is_finite() && v > 0.)
            || !lower_m.is_finite()
            || !((parcel_kg + carriage_kg) * 9.81).is_finite()
        {
            return None;
        }
        Some(Self {
            mass: parcel_kg,
            carriage: carriage_kg,
            lower_m,
            speed: descent_m_s,
            phase: PlacementPhase::Lower,
            contact_seen: false,
            guided_for: Duration::ZERO,
            unsupported_for: Duration::ZERO,
            blocked: Duration::ZERO,
            blind: Duration::ZERO,
            support: SupportedRelease::default(),
            hold_height: lower_m,
            retreat_from: lower_m,
            last: PlacementDrive {
                phase: PlacementPhase::Lower,
                jaw_force_n: 4.2,
                lift_force_n: ((parcel_kg + carriage_kg) * 9.81).min(15.),
            },
        })
    }
    fn weight(&self) -> f64 {
        self.mass * 9.81
    }
    /// Feed committed observations. Lost or unsafe receiver support clears the
    /// confirmation window; missing support after release cancels completion.
    pub fn advance(&mut self, dt: Duration, o: PlacementReading) -> PlacementDrive {
        use PlacementPhase::*;
        if dt.is_zero() {
            return self.last;
        }
        if !o.pad_height_m.is_finite() || !o.pad_velocity_m_s.is_finite() {
            self.support = SupportedRelease::default();
            self.blocked = Duration::ZERO;
            self.guided_for = Duration::ZERO;
            return self.last;
        }
        {
            let weight = self.weight();
            let load = o
                .receiver_impulse_ns
                .filter(|j| j.is_finite())
                .map(|j| j / dt.as_secs_f64());
            let valid_margin = o.deck_margin_m > 0.0015;
            let quiet = o.parcel_velocity_m_s.abs() < 0.018;
            let jaws_clear =
                o.jaw_clearance_m > 0.0005 && o.jaw_pressure_n.iter().all(|f| f.abs() < 0.05);
            match self.phase {
                Lower => {
                    let loaded = load.is_some_and(|f| f > 0.1 * weight);
                    let arriving_without_grasp =
                        loaded && !self.contact_seen && self.guided_for < Duration::from_millis(5);
                    let guided = o
                        .jaw_pressure_n
                        .iter()
                        .all(|force| *force >= 0.1 && force.is_finite())
                        && (o.pad_velocity_m_s - o.parcel_velocity_m_s).abs() < 0.015;
                    // The impact sample can change velocity abruptly. Judge its
                    // approach using observations from strictly before this impact.
                    self.guided_for = if guided {
                        self.guided_for.saturating_add(dt)
                    } else {
                        Duration::ZERO
                    };
                    self.contact_seen |= loaded;
                    self.unsupported_for = if loaded || guided {
                        Duration::ZERO
                    } else {
                        self.unsupported_for.saturating_add(dt)
                    };
                    if self.unsupported_for >= Duration::from_millis(4) {
                        self.contact_seen = false;
                    }
                    let blocked = (o.pad_height_m <= self.lower_m + 0.0002
                        && o.pad_velocity_m_s.abs() < 0.003
                        && load.is_none_or(|f| f < 0.1 * weight))
                        || o.obstacle_impulse_ns > 0.8 * weight * dt.as_secs_f64()
                        || (load.is_some_and(|f| f > 0.8 * weight) && !valid_margin);
                    self.blocked = if blocked {
                        self.blocked.saturating_add(dt)
                    } else {
                        Duration::ZERO
                    };
                    if arriving_without_grasp {
                        self.hold_height = o.pad_height_m;
                        self.phase = UnguidedLanding;
                    } else if self.blocked >= Duration::from_millis(30) {
                        self.hold_height = o.pad_height_m;
                        self.phase = Hold;
                    } else if self.support.observe_checked(
                        dt,
                        o.receiver_impulse_ns,
                        weight,
                        valid_margin && quiet,
                    ) {
                        self.phase = Open;
                        self.support = SupportedRelease::default();
                    }
                }
                Open | Retreat | Placed => {
                    let loaded = load.is_some_and(|f| f >= 0.4 * weight) && valid_margin;
                    self.blind = if loaded {
                        Duration::ZERO
                    } else {
                        self.blind.saturating_add(dt)
                    };
                    if self.blind >= Duration::from_millis(4) {
                        self.phase = if self.phase == Open && !jaws_clear {
                            Hold
                        } else {
                            SupportLost
                        };
                        self.hold_height = o.pad_height_m;
                    } else if self.phase == Open && jaws_clear {
                        self.phase = Retreat;
                        self.retreat_from = o.pad_height_m;
                    } else if self.phase == Retreat
                        && self.support.observe_checked(
                            dt,
                            o.receiver_impulse_ns,
                            weight,
                            jaws_clear
                                && quiet
                                && valid_margin
                                && o.pad_height_m - self.retreat_from >= 0.010,
                        )
                    {
                        self.phase = Placed;
                        self.hold_height = o.pad_height_m;
                    }
                }
                Hold | SupportLost | UnguidedLanding => {}
            }
        }
        let load = (self.mass + self.carriage) * 9.81;
        let unloaded = self.carriage * 9.81;
        let lift = match self.phase {
            PlacementPhase::Lower | PlacementPhase::Open => {
                let speed = if self.contact_seen { 0.08 } else { self.speed };
                load + 30. * (-speed - o.pad_velocity_m_s)
            }
            PlacementPhase::Retreat => unloaded + 30. * (0.1 - o.pad_velocity_m_s),
            PlacementPhase::Placed
            | PlacementPhase::SupportLost
            | PlacementPhase::UnguidedLanding => {
                unloaded + 800. * (self.hold_height - o.pad_height_m) - 35. * o.pad_velocity_m_s
            }
            PlacementPhase::Hold => {
                load + 800. * (self.hold_height - o.pad_height_m) - 35. * o.pad_velocity_m_s
            }
        };
        self.last = PlacementDrive {
            phase: self.phase,
            jaw_force_n: match self.phase {
                Lower | Hold => 4.2,
                UnguidedLanding => 0.,
                Open | Retreat | Placed | SupportLost => -2.3,
            },
            lift_force_n: lift.clamp(-12., 15.),
        };
        self.last
    }
}

#[cfg(test)]
#[path = "../../tests/private/recovery_placement.rs"]
mod tests;
