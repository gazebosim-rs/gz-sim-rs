// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use super::*;
use glam::DMat3;

pub(super) struct BalanceFrame {
    props: [BodyProperties; 6],
    loaded_origin: [DVec3; 6],
    clear_origin: [DVec3; 6],
    wrist: DVec3,
    gravity: DVec3,
}

struct BodyProperties {
    mass: f64,
    center: DVec3,
    inertia: DMat3,
}

impl BodyProperties {
    fn com(&self, state: &State) -> DVec3 {
        DVec3::from(state.xyz) + DQuat::from_array(state.q) * self.center
    }

    fn spin(&self, state: &State) -> DVec3 {
        let q = DQuat::from_array(state.q);
        q * (self.inertia * (q.conjugate() * DVec3::from(state.w)))
    }
}

impl BalanceFrame {
    pub(super) fn new(sdf: &sdformat::Sdf, loaded: &[State], clear: &[State]) -> Self {
        let world = &sdf.worlds[0];
        let model = |name| world.models.iter().find(|m| m.name == name).unwrap();
        let arm = model("gripper");
        let props: [_; 6] = std::array::from_fn(|i| {
            let (owner, name) = if i == 5 {
                (model("parcel"), "box")
            } else {
                (arm, LINKS[i].0)
            };
            let inertial = &owner
                .links
                .iter()
                .find(|l| l.name == name)
                .unwrap()
                .inertial;
            let [xx, yy, zz, xy, xz, yz] = inertial.inertia;
            let matrix = DMat3::from_cols_array(&[xx, xy, xz, xy, yy, yz, xz, yz, zz]);
            let axes = DMat3::from_quat(DQuat::from_array(inertial.pose.transform.rotation));
            BodyProperties {
                mass: inertial.mass,
                center: DVec3::from(inertial.pose.transform.translation),
                inertia: axes * matrix * axes.transpose(),
            }
        });
        let pivot = arm.joints.iter().find(|j| j.name == "wrist").unwrap();
        Self {
            loaded_origin: std::array::from_fn(|i| props[i].com(&loaded[i])),
            clear_origin: std::array::from_fn(|i| props[i].com(&clear[i])),
            props,
            wrist: DVec3::from(loaded[2].xyz)
                + DQuat::from_array(loaded[2].q) * DVec3::from(pivot.pose.transform.translation),
            gravity: DVec3::from(world.gravity),
        }
    }

    /// Momentum in a nonrotating frame falling from the initial wrist pivot.
    /// Gravity cancels in this frame. Aligning each control center at t=0 also
    /// removes the deliberate lateral offset that makes the parcel miss.
    pub(super) fn difference(
        &self,
        i: usize,
        hit: &State,
        clear: &State,
        t: f64,
    ) -> (DVec3, DVec3) {
        let p = &self.props[i];
        let hit_com = p.com(hit);
        let displacement =
            (hit_com - self.loaded_origin[i]) - (p.com(clear) - self.clear_origin[i]);
        let delta_p = p.mass * (DVec3::from(hit.v) - DVec3::from(clear.v));
        let radius = hit_com - self.wrist - 0.5 * self.gravity * t * t;
        let control_p = p.mass * (DVec3::from(clear.v) - self.gravity * t);
        (
            delta_p,
            radius.cross(delta_p) + displacement.cross(control_p) + p.spin(hit) - p.spin(clear),
        )
    }
}
