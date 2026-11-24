// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
use glam::{DQuat, DVec3, DVec4};
use gz_physics_gpu::FreeBoxState;
pub const I: DVec3 = DVec3::new(0.00001450666666666667, 0.00002602666666666667, 0.0000192);
pub const OFFSET: DVec3 = DVec3::new(0.005, 0., -0.002);
pub const MASS: f64 = 0.08;
pub const TOP: f64 = 0.014;
#[derive(Clone, Copy)]
pub struct Body {
    pub p: DVec3,
    pub v: DVec3,
    pub q: DQuat,
    pub w: DVec3,
}
impl Body {
    pub fn from_gpu(s: FreeBoxState) -> Self {
        let q = DQuat::from_array(s.orientation);
        Self {
            p: s.mass_center.into(),
            v: s.center_velocity.into(),
            q,
            w: (q.inverse() * DVec3::from(s.momentum)) / I,
        }
    }
    pub fn free(self, dt: f64) -> Self {
        let deriv = |v: DVec4, w: DVec3| {
            let q = DQuat::from_array(v.to_array());
            let wq = DQuat::from_xyzw(w.x, w.y, w.z, 0.);
            (
                DVec4::from_array((q * wq).to_array()) * 0.5,
                -w.cross(I * w) / I,
            )
        };
        let q = DVec4::from_array(self.q.to_array());
        let w = self.w;
        let (a, b) = deriv(q, w);
        let (c, d) = deriv(q + a * (dt / 2.), w + b * (dt / 2.));
        let (e, f) = deriv(q + c * (dt / 2.), w + d * (dt / 2.));
        let (g, h) = deriv(q + e * dt, w + f * dt);
        Self {
            p: self.p + self.v * dt - DVec3::Z * (4.905 * dt * dt),
            v: self.v - DVec3::Z * (9.81 * dt),
            q: DQuat::from_array((q + (a + 2. * c + 2. * e + g) * (dt / 6.)).to_array())
                .normalize(),
            w: w + (b + 2. * d + 2. * f + h) * (dt / 6.),
        }
    }
    pub fn lowest(self) -> (DVec3, f64) {
        let mut bottom = (DVec3::ZERO, f64::INFINITY);
        for x in [-0.024, 0.024] {
            for y in [-0.012, 0.012] {
                for z in [-0.020, 0.020] {
                    let arm = self.q * (DVec3::new(x, y, z) - OFFSET);
                    let gap = self.p.z + arm.z - TOP;
                    if gap < bottom.1 {
                        bottom = (arm, gap);
                    }
                }
            }
        }
        bottom
    }
}
