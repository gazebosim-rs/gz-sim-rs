// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A short, indivisible run of 1 ms feedback intervals with independently recorded actuator outputs.
use super::*;
use gz_physics::posture::{PostureLaw, PostureMode, PostureTargets};
pub const MAX_POSTURE_INTERVALS: usize = 16;
pub(super) const PROGRAM_LIMIT: usize = 64 * 1024;
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RobotPostureInterval {
    pub targets: PostureTargets,
    /// Individually addressed world forces at link centers of mass, held for this interval.
    pub forces: Vec<(u64, [f64; 3])>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RobotPostureProgram {
    pub law: PostureLaw,
    pub intervals: Vec<RobotPostureInterval>,
}
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RobotPostureTorque {
    pub joint_id: u64,
    pub target_rad: f64,
    pub effort_nm: f64,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RobotPostureFeedback {
    /// Input tick of the 1 ms interval.
    pub tick: u64,
    /// Actual COM read before the interval when balance feedback is active.
    pub center_of_mass: Option<([f64; 3], [f64; 3])>,
    pub torques: Vec<RobotPostureTorque>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RobotPostureReceipt {
    pub input_digest: [u8; 32],
    pub start: u64,
    pub end: u64,
    pub intervals: Vec<RobotPostureFeedback>,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RobotPostureRecord {
    pub epoch: u64,
    pub program: RobotPostureProgram,
    pub receipt: RobotPostureReceipt,
}
impl RobotPostureProgram {
    pub(super) fn check(&self, s: &GroundedRobotSpec) -> Result<()> {
        let law = &self.law;
        if !(1..=MAX_POSTURE_INTERVALS).contains(&self.intervals.len())
            || law.axes.is_empty()
            || law.axes.len() > s.joints.len()
            || law.link_order.len() != s.links.len()
            || law.link_order.iter().copied().collect::<BTreeSet<_>>()
                != s.links.iter().map(|l| l.id).collect()
            || !law.balance.reference.iter().all(|v| v.is_finite())
            || !law
                .balance
                .gains
                .iter()
                .chain(&law.balance.max_correction)
                .all(|v| v.is_finite() && *v >= 0.)
        {
            return Err(invalid(
                "invalid robot posture topology, feedback or batch size",
            ));
        }
        let mut joints = BTreeSet::new();
        for a in &law.axes {
            if !joints.insert(a.joint_id)
                || !s.joints.iter().any(|j| j.joint_id == a.joint_id)
                || !a.limits.iter().all(|v| v.is_finite())
                || !(a.limits[0]..=a.limits[1]).contains(&a.reference_rad)
                || !a.stiffness.is_finite()
                || a.stiffness <= 0.
                || !a.damping.is_finite()
                || a.damping < 0.
                || !a.max_effort_nm.is_finite()
                || a.max_effort_nm <= 0.
                || s.limits
                    .iter()
                    .find(|l| l.joint_id == a.joint_id)
                    .is_some_and(|l| a.limits[0] < l.lower || a.limits[1] > l.upper)
            {
                return Err(invalid(
                    "invalid robot actuator, reference, gains, stops or torque cap",
                ));
            }
        }
        for interval in &self.intervals {
            let targets = &interval.targets;
            if !targets.balance_offset.iter().all(|v| v.is_finite())
                || targets.axes.len() > law.axes.len()
                || interval.forces.len() > s.links.len()
            {
                return Err(invalid("invalid robot interval feedback or input count"));
            }
            let mut seen = BTreeSet::new();
            for t in &targets.axes {
                let a = law
                    .axes
                    .iter()
                    .find(|a| a.joint_id == t.joint_id)
                    .ok_or(invalid("unknown robot posture actuator"))?;
                if !seen.insert(t.joint_id)
                    || !(a.limits[0]..=a.limits[1]).contains(&t.target_rad)
                    || !t.feedforward_nm.is_finite()
                    || !t.direct_effort_nm.is_none_or(f64::is_finite)
                {
                    return Err(invalid("invalid or repeated robot setpoint"));
                }
            }
            for (id, f) in &interval.forces {
                if s.links.binary_search_by_key(id, |l| l.id).is_err()
                    || !seen.insert(*id)
                    || !f.iter().all(|v| v.is_finite())
                {
                    return Err(invalid("invalid or repeated robot interval force"));
                }
            }
        }
        Ok(())
    }
    pub(super) fn encode(&self) -> Result<Vec<u8>> {
        Ok(codec(PROGRAM_LIMIT)
            .serialize(self)
            .map_err(std::io::Error::other)?)
    }
}
impl RobotPostureReceipt {
    pub(super) fn check(
        &self,
        p: &RobotPostureProgram,
        digest: [u8; 32],
        start: u64,
    ) -> Result<()> {
        if self.input_digest != digest
            || self.start != start
            || Some(self.end) != start.checked_add(p.intervals.len() as u64)
            || self.intervals.len() != p.intervals.len()
        {
            return Err(invalid("robot feedback digest or interval differs"));
        }
        for (i, (f, input)) in self.intervals.iter().zip(&p.intervals).enumerate() {
            let passive = input.targets.mode == PostureMode::Passive;
            if f.tick != start + i as u64
                || f.torques.len() != if passive { 0 } else { p.law.axes.len() }
                || f.center_of_mass.is_some() != (input.targets.mode == PostureMode::Balance)
                || f.center_of_mass
                    .is_some_and(|(p, v)| !p.iter().chain(&v).all(|x| x.is_finite()))
                || f.torques.iter().zip(&p.law.axes).any(|(t, a)| {
                    t.joint_id != a.joint_id
                        || !(a.limits[0]..=a.limits[1]).contains(&t.target_rad)
                        || !t.effort_nm.is_finite()
                        || t.effort_nm.abs() > a.max_effort_nm
                })
            {
                return Err(invalid(
                    "invalid robot feedback sample or actuator readback",
                ));
            }
        }
        Ok(())
    }
}
