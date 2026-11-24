// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! One planned transfer onto the left foot, followed by right-foot lift or placement.
use super::FootstepMotion as Motion;
pub const AXES: &[(&str, f64)] = &[
    ("left_hip_roll", 0.),
    ("right_hip_roll", 0.),
    ("left_ankle_roll", 0.),
    ("right_ankle_roll", 0.),
    ("left_hip_pitch", 0.),
    ("right_hip_pitch", 0.),
    ("left_knee_pitch", 0.),
    ("right_knee_pitch", 0.),
    ("left_ankle_pitch", 0.),
    ("right_ankle_pitch", 0.),
    ("spine_roll", 0.),
];
fn ramp(t: u64, start: u64, end: u64) -> f64 {
    let progress = (t.saturating_sub(start) as f64 / (end - start) as f64).min(1.);
    0.5 * (1. - (std::f64::consts::PI * progress).cos())
}
pub struct Target {
    pub balance: [f64; 2],
    pub swing_foot: usize,
    pub local_tick: u64,
    pub lifted: bool,
    hip_roll: [f64; 2],
    footprints: [f64; 2],
    swing_shorten: f64,
    crouch: f64,
    sagittal: [[f64; 3]; 2],
}
impl Target {
    /// Use the measured supporting foothold for the balance reference. The airborne leg
    /// tracks the authored world footprint within bounded kinematic corrections.
    pub(super) fn track_feet(&mut self, current: [glam::DVec2; 2], initial: [glam::DVec2; 2]) {
        use glam::DVec2;
        let swing = self.swing_foot;
        let stance = 1 - swing;
        let error = std::array::from_fn::<_, 2, _>(|i| {
            current[i] - initial[i] - DVec2::new(self.footprints[i], 0.)
        });
        let single = ramp(self.local_tick, 700, 2150) * (1. - ramp(self.local_tick, 5000, 6500));
        let measured = (error[stance] * single + 0.5 * (error[0] + error[1]) * (1. - single))
            .clamp_length_max(0.09);
        let pelvis = self.balance[0];
        self.balance[0] += 0.85 * measured.x;
        self.balance[1] += 0.85 * measured.y;
        let weight = ramp(self.local_tick, 2450, 3100) * (1. - ramp(self.local_tick, 4730, 5070));
        let adjust = (-error[swing] * 1.5 * weight)
            .clamp(DVec2::new(-0.060, -0.030), DVec2::new(0.060, 0.030));
        self.sagittal[swing] = leg(
            self.footprints[swing] - pelvis + adjust.x,
            self.swing_shorten,
        );
        self.hip_roll[swing] += (adjust.y / (0.748 - self.swing_shorten)).atan();
    }
    /// Keep planted-leg geometry referenced to the measured support centers. The
    /// swing leg reaches the authored mark while its foot is unloaded. On return
    /// to double support both leg references use the actual acquired footholds.
    pub(super) fn support_relative(
        &mut self,
        current: [glam::DVec2; 2],
        initial: [glam::DVec2; 2],
        lateral_limit: Option<f64>,
    ) {
        use glam::DVec2;
        let swing = self.swing_foot;
        let stance = 1 - swing;
        let offset: [DVec2; 2] =
            std::array::from_fn(|i| current[i] - initial[i] - DVec2::new(self.footprints[i], 0.));
        let single = ramp(self.local_tick, 700, 2150) * (1. - ramp(self.local_tick, 5000, 6500));
        let measured_support =
            offset[stance] * single + 0.5 * (offset[0] + offset[1]) * (1. - single);
        let mut support_error = measured_support.clamp_length_max(0.35);
        let mut swing_lane_offset = 0.;
        if let Some(limit) = lateral_limit {
            // In a shifted pair of footholds the original world lane can be beyond
            // one leg's lateral reach. Keep stance-relative width and move back
            // toward the authored lane by at most the requested distance per swing.
            support_error.y = measured_support.y;
            let stance_shift = offset[stance].y;
            swing_lane_offset = stance_shift - stance_shift.clamp(-limit, limit);
        }
        self.balance[0] += support_error.x;
        self.balance[1] += support_error.y;
        let airborne = ramp(self.local_tick, 2400, 3060) * (1. - ramp(self.local_tick, 4730, 4990));
        for (i, offset) in offset.into_iter().enumerate() {
            let acquisition = if i == swing { airborne } else { 0. };
            let mut planted = offset.clamp_length_max(0.35) * (1. - acquisition);
            if lateral_limit.is_some() {
                planted.y = offset.y * (1. - acquisition) + swing_lane_offset * acquisition;
            }
            let shorten = if i == swing {
                self.swing_shorten
            } else {
                self.crouch
            };
            let ankle_x = self.footprints[i] + planted.x - self.balance[0];
            self.sagittal[i] = leg(ankle_x, shorten);
            self.hip_roll[i] = ((planted.y - self.balance[1]) / (0.748 - shorten)).atan();
        }
    }
    /// Keep an early, load-bearing contact at the foothold that the robot actually reached.
    pub(super) fn acquire_support(&mut self, feet: [glam::DVec2; 2], weight: f64) {
        let center = 0.5 * (feet[0] + feet[1]);
        let blend = |a: f64, b: f64| a + weight * (b - a);
        self.balance[0] = blend(self.balance[0], center.x);
        self.balance[1] = blend(self.balance[1], center.y);
        for (i, foot) in feet.into_iter().enumerate() {
            let x = foot.x - center.x;
            let y = foot.y - center.y;
            let leg = leg(x, self.crouch);
            for (angle, target) in self.sagittal[i].iter_mut().zip(leg) {
                *angle = blend(*angle, target);
            }
            self.hip_roll[i] = blend(self.hip_roll[i], (y / (0.748 - self.crouch)).atan());
        }
        if weight > 0.5 {
            self.lifted = false;
        }
    }

    pub(super) fn heading_swing_weight(&self, side: usize) -> f64 {
        if side != self.swing_foot {
            return 0.;
        }
        ramp(self.local_tick, 2350, 3100) * (1. - ramp(self.local_tick, 4230, 4900))
    }

    pub(super) fn landing_error(&self, displacement: glam::DVec2) -> f64 {
        (self.footprints[self.swing_foot] - displacement.x).abs()
    }

    pub(super) fn lower_for_recovery(&mut self, risk: f64, loaded: bool) {
        let s = self.swing_foot;
        let shorten = self.swing_shorten - risk * (self.swing_shorten - self.crouch);
        self.sagittal[s] = leg(self.footprints[s] - self.balance[0], shorten);
        if loaded && risk > 0.3 {
            self.lifted = false;
            self.balance[1] *= 1. - risk;
            for angle in &mut self.hip_roll {
                *angle *= 1. - risk;
            }
        }
    }
    /// A level sole uses opposing hip and ankle roll. Project the pair as one coordinate
    /// into the intersection of both actual SDF limits before commanding either actuator.
    pub(super) fn limit_level_sole_roll(&mut self, limits: [[f64; 2]; 2]) {
        for (angle, [lower, upper]) in self.hip_roll.iter_mut().zip(limits) {
            *angle = angle.clamp(lower, upper);
        }
    }

    pub fn joint(&self, name: &str) -> Result<f64, gz_physics::PhysicsError> {
        if name == "spine_roll" {
            return Ok(0.);
        }
        let (prefix, kind) = name
            .split_once('_')
            .ok_or(gz_physics::PhysicsError::MissingJoint)?;
        let side = match prefix {
            "left" => 0,
            "right" => 1,
            _ => return Err(gz_physics::PhysicsError::MissingJoint),
        };
        match kind {
            "hip_roll" => Ok(self.hip_roll[side]),
            "ankle_roll" => Ok(-self.hip_roll[side]),
            "hip_pitch" => Ok(self.sagittal[side][0]),
            "knee_pitch" => Ok(self.sagittal[side][1]),
            "ankle_pitch" => Ok(self.sagittal[side][2]),
            _ => Err(gz_physics::PhysicsError::MissingJoint),
        }
    }
}
/// Sagittal hip, knee and ankle targets from the two authored joint-center spans.
/// Keeping the three pitch angles summed to zero gives a level sole reference.
fn leg(x: f64, shorten: f64) -> [f64; 3] {
    let (thigh, shank) = (0.377, 0.371);
    let z = thigh + shank - shorten;
    let cos =
        ((z * z + x * x - thigh * thigh - shank * shank) / (2. * thigh * shank)).clamp(-1., 1.);
    let knee = cos.acos();
    let hip = (-x).atan2(z) - (shank * knee.sin()).atan2(thigh + shank * knee.cos());
    [hip, knee, -hip - knee]
}
pub fn target(
    mode: Motion,
    t: u64,
    lateral: f64,
    length: f64,
    count: u16,
) -> Result<Target, gz_physics::PhysicsError> {
    let checked = super::FootstepPlan {
        motion: mode,
        lateral_m: lateral,
        length_m: length,
        count,
        placement_feedback: false,
        recovery_landing: false,
        support_relative: false,
        lateral_correction_limit_m: None,
        heading_gain: None,
    };
    if !checked.valid() {
        return Err(gz_physics::PhysicsError::InvalidValue);
    }
    if mode == Motion::Walk {
        return Ok(walk(t, lateral, length, count));
    }
    let shift = ramp(t, 600, 2200) * (1. - ramp(t, 5100, 6600));
    let crouch = 0.012 * ramp(t, 200, 600);
    let lift = if mode == Motion::WeightShift {
        0.
    } else {
        0.052 * ramp(t, 2350, 3200) * (1. - ramp(t, 4030, 4880))
    };
    let foot_x = if mode == Motion::Step {
        length * ramp(t, 3100, 4000)
    } else {
        0.
    };
    let pelvis_x = 0.5 * foot_x * ramp(t, 5000, 6200);
    let hip_roll = -(lateral * shift / (0.748 - crouch)).atan();
    Ok(Target {
        balance: [pelvis_x, lateral * shift],
        swing_foot: 1,
        local_tick: t,
        lifted: mode != Motion::WeightShift && (2440..4720).contains(&t),
        hip_roll: [hip_roll; 2],
        footprints: [0., foot_x],
        swing_shorten: crouch + lift,
        crouch,
        sagittal: [
            leg(-pelvis_x, crouch),
            leg(foot_x - pelvis_x, crouch + lift),
        ],
    })
}

/// Alternate a planned slow transfer with a level swing, then stop in double support.
fn walk(t: u64, lateral: f64, length: f64, count: u16) -> Target {
    const CYCLE: u64 = 7000;
    let step = (t / CYCLE).min(u64::from(count) - 1);
    let phase = t - step * CYCLE;
    let stance = (step % 2) as usize;
    let swing = 1 - stance;
    let sign = if stance == 0 { 1. } else { -1. };
    let transfer = ramp(phase, 600, 2200);
    let recenter = ramp(phase, 5100, 6600);
    let roll_offset = sign * lateral * transfer * (1. - recenter);
    let crouch = 0.012 * ramp(t, 200, 600);
    let clearance = 0.052 * ramp(phase, 2350, 3200) * (1. - ramp(phase, 4030, 4880));
    let x_stance = step as f64 * length;
    let x_previous = step.saturating_sub(1) as f64 * length;
    let x_next = (step + 1) as f64 * length;
    let x_swing = x_previous + (x_next - x_previous) * ramp(phase, 3100, 4000);
    let x_start = 0.5 * (x_stance + x_previous);
    let x_end = 0.5 * (x_stance + x_next);
    let pelvis =
        x_start + (x_stance - x_start) * transfer + (x_end - x_stance) * ramp(phase, 5000, 6200);
    let mut sagittal = [[0.; 3]; 2];
    sagittal[stance] = leg(x_stance - pelvis, crouch);
    sagittal[swing] = leg(x_swing - pelvis, crouch + clearance);
    let mut footprints = [0.; 2];
    footprints[stance] = x_stance;
    footprints[swing] = x_swing;
    Target {
        footprints,
        swing_shorten: crouch + clearance,
        crouch,
        balance: [pelvis, roll_offset],
        swing_foot: swing,
        local_tick: phase,
        lifted: (2440..4720).contains(&phase),
        hip_roll: [-(roll_offset / (0.748 - crouch)).atan(); 2],
        sagittal,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn full_swing_respects_the_measured_stance_width_and_bounded_lateral_correction() {
        use glam::DVec2;
        let origin = [DVec2::new(0.025, 0.103), DVec2::new(0.025, -0.103)];
        let shifted = origin.map(|p| p + DVec2::new(0., 1.));
        let lateral = |feet, limit| {
            let mut goal = target(Motion::Walk, 3600, 0.100, 0.115, 64).unwrap();
            goal.support_relative(feet, origin, Some(limit));
            let height = 0.748 - goal.swing_shorten;
            let angle = goal.hip_roll[goal.swing_foot];
            (goal.balance[1], goal.balance[1] + height * angle.tan())
        };
        let (base_com, base_foot) = lateral(origin, 0.);
        let (com, foot) = lateral(shifted, 0.);
        assert!((com - base_com - 1.).abs() < 1e-12);
        assert!((foot - base_foot - 1.).abs() < 1e-12);
        let (_, corrected) = lateral(shifted, 0.035);
        assert!((foot - corrected - 0.035).abs() < 1e-12);
    }

    #[test]
    fn long_walk_witness_projects_opposing_roll_axes_at_the_narrower_ankle_stop() {
        let mut target = target(Motion::Walk, 282718, 0.100, 0.115, 64).unwrap();
        let feet = [
            glam::DVec2::new(4.498060782455264, 0.416428487671531),
            glam::DVec2::new(4.597433105276595, 0.10671161411515472),
        ];
        let origin = [
            glam::DVec2::new(0.025, 0.103),
            glam::DVec2::new(0.025, -0.103),
        ];
        target.support_relative(feet, origin, None);
        let raw = target.joint("right_ankle_roll").unwrap();
        let left = target.joint("left_ankle_roll").unwrap();
        assert!((raw - 0.4001240997091232).abs() < 1.0e-12);
        target.limit_level_sole_roll([[-0.4, 0.4]; 2]);
        assert_eq!(target.joint("right_ankle_roll").unwrap(), 0.4);
        assert_eq!(target.joint("right_hip_roll").unwrap(), -0.4);
        assert_eq!(target.joint("left_ankle_roll").unwrap(), left);
        println!(
            "step=41 tick=282718 raw_right_ankle_roll={raw:.15} authored_stop=0.4 hip=-0.4 ankle=0.4"
        );
    }
}
