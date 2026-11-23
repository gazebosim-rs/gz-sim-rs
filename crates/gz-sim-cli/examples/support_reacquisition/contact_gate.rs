// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! A prospective contact window for the fixture's finite, horizontal lift.
use super::*;
#[derive(Default)]
pub(super) struct Gate {
    armed: bool,
    remaining: u64,
    boundary: u64,
    /// Relative outer-step numbers, with 1 the first step after installation.
    pub(super) triggers: Vec<u64>,
}
impl Gate {
    pub(super) fn triggered_on_current_boundary(&self) -> bool {
        self.triggers.last().copied() == Some(self.boundary)
    }
    pub(super) fn advance(
        &mut self,
        foot: BodyState,
        pad: BodyState,
        length: u64,
        dt: f64,
    ) -> bool {
        self.boundary += 1;
        if self.remaining > 0 {
            self.remaining -= 1;
            return true;
        }
        let q = DQuat::from_array(foot.orientation);
        let dz = sole_gap(foot.into(), pad.into());
        // Require a genuine flight before arming again. Submillimetre resting
        // contact chatter stays on the coarse clock.
        self.armed |= dz > 0.001;
        if !self.armed {
            return false;
        }
        let v = DVec3::from(foot.velocity) - DVec3::from(pad.velocity);
        let w = DVec3::from(foot.angular_velocity);
        let a = DVec3::from(foot.position) - DVec3::from(pad.position);
        let half = DVec3::from(FOOT_HALF);
        let radius = half.length();
        let angle = w.length() * dt;
        let gravity_drop = 0.5 * 9.81 * dt * dt;
        if dz > (-v.z * dt).max(0.) + radius * angle + gravity_drop {
            return false;
        }
        for (axis, deck_half) in [(DVec3::X, PAD_HALF[0]), (DVec3::Y, PAD_HALF[1])] {
            let extent = (q.conjugate() * axis).abs().dot(half);
            if a.dot(axis).abs() > extent + deck_half + v.dot(axis).abs() * dt + radius * angle {
                return false;
            }
        }
        // The four underside corners bound a nearly level box over this deck.
        // Constant-spin chord remainder is bounded by r*theta^2/2. This
        // fixture has no free-flight force other than gravity.
        for x in [-half.x, half.x] {
            for y in [-half.y, half.y] {
                let r = q * DVec3::new(x, y, -half.z);
                let gap = a.z + r.z - PAD_HALF[2];
                let delta = (v + w.cross(r)).z * dt - gravity_drop - radius * angle * angle * 0.5;
                if delta < 0. && gap + delta <= 0. {
                    self.armed = false;
                    self.remaining = length - 1;
                    self.triggers.push(self.boundary);
                    return true;
                }
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn body(x: f64, z: f64, vz: f64) -> BodyState {
        BodyState {
            position: [x, 0., z],
            velocity: [0., 0., vz],
            orientation: [0., 0., 0., 1.],
            angular_velocity: [0.; 3],
        }
    }
    #[test]
    fn stationary_receding_and_laterally_separated_poses_leave_gate_idle() {
        let pad = body(0., -0.06, 0.);
        let mut g = Gate::default();
        assert!(!g.advance(body(0., 0.062, -1.), pad, 2, 0.0005));
        for f in [
            body(0., 0.0604, 1.),
            body(2., 0.0604, -1.),
            body(0., 0.0604, 0.),
        ] {
            assert!(!g.advance(f, pad, 2, 0.0005));
        }
        assert!(g.advance(body(0., 0.0604, -1.), pad, 2, 0.0005));
        assert!(g.advance(body(0., 0.06, 0.), pad, 2, 0.0005));
        assert!(!g.advance(body(0., 0.0604, -1.), pad, 2, 0.0005));
        assert!(!g.advance(body(0., 0.062, 1.), pad, 2, 0.0005));
        assert!(g.advance(body(0., 0.0604, -1.), pad, 2, 0.0005));
        assert_eq!(g.triggers, [5, 9]);
    }
    #[test]
    fn angular_motion_catches_a_descending_corner_with_stationary_center() {
        let mut g = Gate::default();
        let pad = body(0., -0.06, 0.);
        assert!(!g.advance(body(0., 0.064, 0.), pad, 1, 0.0005));
        let mut f = body(0., 0.0603, 0.);
        f.angular_velocity = [0., 4., 0.];
        assert!(g.advance(f, pad, 1, 0.0005));
    }
}
