// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Spatial momentum and attitude after an actually separated cuboid leaves the fingers.
use super::*;
use crate::spatial::{cross, rotate};
use glam::{DMat3, DQuat, DVec3};

#[derive(Clone, Copy, Debug)]
pub struct BoxFlightMaterial {
    /// Principal moments at the mass center, in the collision box's local axes.
    pub inertia: [f64; 3],
    pub half_depth: f64,
    pub initial_orientation: [f64; 4],
    pub transverse_origin: f64,
}
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxLinkState {
    pub position: [f64; 3],
    pub orientation: [f64; 4],
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}
#[derive(Clone, Copy, Debug, PartialEq, Pod, Zeroable)]
#[repr(C)]
pub struct FreeBoxState {
    pub mass_center: [f64; 3],
    pub center_velocity: [f64; 3],
    /// Local-to-world unit quaternion (x, y, z, w).
    pub orientation: [f64; 4],
    /// Conserved world angular momentum about the moving mass center.
    pub momentum: [f64; 3],
}
#[derive(Clone, Copy, Pod, Zeroable)]
#[repr(C)]
struct Lane {
    carry: CarryGripperLane,
    body: FreeBoxState,
    inertia: [f64; 3],
    swept: [[f64; 2]; 3],
}
const WORDS: usize = WIDTH + 22;
const _: () = assert!(size_of::<Lane>() == WORDS * 8);

pub(super) struct BoxFlight {
    device: ResidentBatch<Lane>,
    material: BoxFlightMaterial,
    active: bool,
    published: bool,
}
impl BoxFlightMaterial {
    fn accepts(self, l: &CarryGripperLane) -> bool {
        let q = DQuat::from_array(self.initial_orientation);
        let m = DVec3::from(self.inertia);
        let trace = m.element_sum();
        if !q.is_finite() || (q.length_squared() - 1.).abs() > 1e-12 {
            return false;
        }
        let y = q * DVec3::Y;
        let x = q * DVec3::X;
        let [sin, cos] = [l.box_heading.sin(), l.box_heading.cos()];
        trace.is_finite()
            && l.box_half != [0.; 2]
            && self.half_depth > 0.
            && self.half_depth.is_finite()
            && self.transverse_origin.is_finite()
            && y.x.hypot(y.z) < 1e-12
            && (x.x - cos).abs() < 1e-12
            && (x.z - sin).abs() < 1e-12
            && self
                .inertia
                .iter()
                .all(|&i| i > 0. && i.recip().is_normal() && 2. * i <= trace)
            && (m.y - l.angular_inertia).abs() <= 64. * f64::EPSILON * trace
            && [l.box_half[0], self.half_depth, l.box_half[1]]
                .into_iter()
                .zip(self.inertia)
                .zip(self.local_center(l).to_array())
                .all(|((half, i), offset)| {
                    trace * 0.5 - i + l.hand.ball_mass * offset * offset
                        <= l.hand.ball_mass * half * half * (1. + 64. * f64::EPSILON)
                })
    }
    fn local_center(self, l: &CarryGripperLane) -> DVec3 {
        DQuat::from_array(self.initial_orientation).inverse()
            * DVec3::new(l.com_offset[0], 0., l.com_offset[1])
    }
    fn start(self, l: &CarryGripperLane, impulse: DVec3, point: DVec3) -> FreeBoxState {
        let h = &l.hand;
        let q = DQuat::from_rotation_y(-h.angle) * DQuat::from_array(self.initial_orientation);
        let center = self.local_center(l);
        let offset = q * center;
        let omega = DVec3::new(0., -h.spin, 0.);
        let angular = q * (DVec3::from(self.inertia) * (q.inverse() * omega))
            + (q * (point - center)).cross(impulse);
        FreeBoxState {
            mass_center: (DVec3::new(h.ball[0], self.transverse_origin, h.ball[1]) + offset)
                .to_array(),
            center_velocity: (DVec3::new(h.velocity[0], 0., h.velocity[1])
                + omega.cross(offset)
                + impulse / h.ball_mass)
                .to_array(),
            orientation: q.to_array(),
            momentum: angular.to_array(),
        }
    }
}
impl FreeBoxState {
    /// Geometry-origin pose and twist. The center of mass can follow a different trajectory.
    pub fn link_state(
        self,
        inertia: [f64; 3],
        local_center: [f64; 3],
    ) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let q = DQuat::from_array(self.orientation);
        let omega = q * ((q.inverse() * DVec3::from(self.momentum)) / DVec3::from(inertia));
        let offset = q * DVec3::from(local_center);
        (
            (DVec3::from(self.mass_center) - offset).to_array(),
            (DVec3::from(self.center_velocity) - omega.cross(offset)).to_array(),
            omega.to_array(),
        )
    }
}
fn clear_sweep(m: BoxFlightMaterial, start: &FreeBoxState, l: &Lane) -> bool {
    let h = l.carry.hand;
    let q = DQuat::from_array(start.orientation);
    let offset = m.local_center(&l.carry);
    let origin = DVec3::from(start.mass_center) - q * offset;
    let half = DVec3::new(l.carry.box_half[0], m.half_depth, l.carry.box_half[1]);
    let rot = DMat3::from_quat(q);
    let extent = rot.x_axis.abs() * half.x + rot.y_axis.abs() * half.y + rot.z_axis.abs() * half.z;
    let dt = STEP.as_secs_f64();
    // Every point travels at most |L|/I_min * radius through angular motion.
    // Include ballistic displacement and the measured swept pad bounds over all four substeps.
    let angular = dt * DVec3::from(start.momentum).length() / DVec3::from(m.inertia).min_element();
    let padding = (half + offset.abs()).length() * angular.min(2.);
    let velocity = DVec3::from(start.center_velocity) * dt;
    let lo = origin - extent - DVec3::splat(padding) + velocity.min(DVec3::ZERO)
        - DVec3::Z * (0.5 * h.gravity * dt * dt);
    let hi = origin + extent + DVec3::splat(padding) + velocity.max(DVec3::ZERO);
    // Pads and shelf are conservatively extended in Y. Stop before any
    // possible contact. General spatial collision resolution is a separate profile.
    let separate = |x: [f64; 2], z: [f64; 2], r: [f64; 2]| {
        hi.x < x[0] - r[0] || lo.x > x[1] + r[0] || hi.z < z[0] - r[1] || lo.z > z[1] + r[1]
    };
    lo.z > h.ground
        && separate(
            [l.carry.shelf_center[0]; 2],
            [l.carry.shelf_center[1]; 2],
            l.carry.shelf_half,
        )
        && (0..2).all(|i| separate(l.swept[i + 1], l.swept[0], h.half_size[i]))
}
impl CarryGripperBatch {
    /// Prepare an optional spatial release kernel for a single cuboid without changing its initial dynamics.
    pub fn enable_box_flight(&mut self, material: BoxFlightMaterial) -> Result<(), GpuError> {
        let [l] = self.lanes() else {
            return Err(GpuError::InvalidBoxRelease);
        };
        if self.tick() != 0 || self.flight.is_some() || !material.accepts(l) {
            return Err(GpuError::InvalidBoxRelease);
        }
        let initial = Lane {
            carry: *l,
            body: material.start(l, DVec3::ZERO, DVec3::ZERO),
            inertia: material.inertia,
            swept: l.hand.position.map(|x| [x; 2]),
        };
        let gpu = self.planar.gpu.companion::<Lane>(1)?;
        let mut device = ResidentBatch::on_device(gpu, vec![initial], module, None)?;
        let total = self.planar.device_info().allocated_buffer_bytes
            + device.device_info().allocated_buffer_bytes;
        device.gpu.info.allocated_buffer_bytes = total;
        self.planar.gpu.info.allocated_buffer_bytes = total;
        self.flight = Some(BoxFlight {
            device,
            material,
            active: false,
            published: false,
        });
        Ok(())
    }
    /// At a free boundary apply a world impulse (N·s) at a box-local material point (m).
    pub fn release_box(&mut self, impulse: [f64; 3], point: [f64; 3]) -> Result<(), GpuError> {
        let f = self.flight.as_mut().ok_or(GpuError::InvalidBoxRelease)?;
        let l = self.planar.committed[0];
        let p = DVec3::from(point);
        let j = DVec3::from(impulse);
        let half = DVec3::new(l.box_half[0], f.material.half_depth, l.box_half[1]);
        if f.active || !p.is_finite() || !j.is_finite() || !p.abs().cmple(half).all() {
            return Err(GpuError::InvalidBoxRelease);
        }
        let start = f.material.start(&l, j, p);
        let record = Lane {
            carry: l,
            body: start,
            inertia: f.material.inertia,
            swept: l.hand.position.map(|x| [x; 2]),
        };
        if l.hand.normal_impulse != [0.; 3]
            || l.shelf_normal != 0.
            || !clear_sweep(f.material, &start, &record)
        {
            return Err(GpuError::SpatialContactBoundary);
        }
        f.device.committed[0] = record;
        f.device.tick = self.planar.tick;
        f.device.upload_needed = true;
        f.active = true;
        Ok(())
    }
    /// Active state layout and total explicit buffers allocated by both kernels on the device.
    pub fn device_info(&self) -> &crate::GpuInfo {
        match self.flight.as_ref().filter(|f| f.published) {
            Some(f) => f.device.device_info(),
            None => self.planar.device_info(),
        }
    }
    /// Six-degree-of-freedom body at the latest accepted public tick.
    pub fn free_box(&self) -> Option<FreeBoxState> {
        self.flight
            .as_ref()
            .filter(|f| f.published)
            .map(|f| f.device.lanes()[0].body)
    }
    /// Full geometric link pose and twist at the latest accepted public tick.
    pub fn free_box_link(&self) -> Option<BoxLinkState> {
        let f = self.flight.as_ref().filter(|f| f.published)?;
        let state = f.device.lanes()[0].body;
        let (p, v, w) = state.link_state(
            f.material.inertia,
            f.material.local_center(&self.planar.lanes()[0]).to_array(),
        );
        Some(BoxLinkState {
            position: p,
            orientation: state.orientation,
            linear: v,
            angular: w,
        })
    }
    pub fn step(&mut self) -> Result<crate::StepMetrics, GpuError> {
        let Some(f) = self.flight.as_mut().filter(|f| f.active) else {
            return self.planar.step();
        };
        let old = self.planar.committed[0];
        let prev = f.device.committed[0].body;
        if self.planar.upload_needed {
            f.device.committed[0].carry.track_target = old.track_target;
            f.device.committed[0].carry.hand.target = old.hand.target;
            f.device.upload_needed = true;
        }
        let material = f.material;
        let metrics = f.device.advance_checked(1, |rows| {
            if clear_sweep(material, &prev, &rows[0]) {
                Ok(())
            } else {
                Err(GpuError::SpatialContactBoundary)
            }
        })?;
        let l = &mut f.device.committed[0];
        let (p, v, w) = l
            .body
            .link_state(material.inertia, material.local_center(&l.carry).to_array());
        // The canonical spatial attitude is exposed by free_box. The planar API
        // continues reporting the geometric X/Z components and projected spin.
        self.planar.committed[0] = l.carry;
        let h = &mut self.planar.committed[0].hand;
        h.ball = [p[0], p[2]];
        h.velocity = [v[0], v[2]];
        h.spin = -w[1];
        let x = DQuat::from_array(l.body.orientation) * DVec3::X;
        h.angle = x.z.atan2(x.x) - l.carry.box_heading;
        f.published = true;
        self.planar.tick = f.device.tick;
        self.planar.upload_needed = false;
        Ok(metrics)
    }
    pub fn reset(&mut self) {
        if let Some(f) = &mut self.flight {
            f.active = false;
            f.published = false;
            f.device.reset();
        }
        self.planar.reset();
    }
}
fn rotated<T: Scalar>(q: [T; 4], v: [T; 3]) -> [T; 3] {
    let a = [q[0], q[1], q[2]];
    let b = cross(a, v).map(|x| x + x);
    let c = cross(a, b);
    std::array::from_fn(|i| v[i] + q[3] * b[i] + c[i])
}
fn module() -> naga::Module {
    kernel::build(&constants(), |v: [_; WORDS], c| {
        let mut base = std::array::from_fn(|i| v[i]);
        // Payload contact is absent; these words are unused by the unloaded hand.
        for x in &mut base[13..19] {
            *x = c[1];
        }
        let (hand, swept) = integrate::<false, false, true, _>(base, c);
        let mut p: [_; 3] = std::array::from_fn(|i| v[WIDTH + i]);
        let mut vel: [_; 3] = std::array::from_fn(|i| v[WIDTH + 3 + i]);
        let mut q: [_; 4] = std::array::from_fn(|i| v[WIDTH + 6 + i]);
        let l: [_; 3] = std::array::from_fn(|i| v[WIDTH + 10 + i]);
        let omega = |q: [_; 4]| {
            let local = rotated([c[1] - q[0], c[1] - q[1], c[1] - q[2], q[3]], l);
            rotated(q, std::array::from_fn(|i| local[i] / v[WIDTH + 13 + i]))
        };
        for _ in 0..4 {
            let mid = rotate(q, omega(q), c[0] * c[3], c[3]);
            q = rotate(q, omega(mid), c[0], c[3]);
            for i in 0..3 {
                p[i] = p[i] + vel[i] * c[0];
            }
            p[2] = p[2] - c[3] * c[0] * c[0] * v[21];
            vel[2] = vel[2] - c[0] * v[21];
        }
        std::array::from_fn::<_, 49, _>(|i| match i {
            0..33 => hand[i],
            33..36 => (WIDTH + i - 33, p[i - 33]),
            36..39 => (WIDTH + i - 33, vel[i - 36]),
            39..43 => (WIDTH + i - 33, q[i - 39]),
            _ => (WIDTH + 16 + i - 43, swept[(i - 43) / 2][(i - 43) % 2]),
        })
    })
}
