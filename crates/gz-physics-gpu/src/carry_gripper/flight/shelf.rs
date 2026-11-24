// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Isolated frictionless normal impact between one cuboid corner and a finite shelf top.
use super::*;
mod edge;
use crate::arithmetic::Condition;
pub use edge::ShelfWitness;
use edge::Solver;
/// Vertices may share a single restitution solve only when both their surface
/// distances and signed arrival times are indistinguishable at this scale.
const TIE_SECONDS: f64 = 1e-10;
use crate::spatial::dot;
const DT: f64 = 0.00025;
const EPS: f64 = 1e-11;
#[derive(Clone, Copy, Debug)]
/// Transverse dimensions of the authored horizontal receiving surface and its
/// Newton normal restitution coefficient. The contact pair must have zero friction.
pub struct SpatialShelf {
    pub center_y: f64,
    pub half_depth: f64,
    pub restitution: f64,
}
#[derive(Clone, Copy, Debug, PartialEq)]
/// Positive impulses act on the parcel. Components and witness coordinates use world axes.
pub struct ShelfImpact {
    pub tick: u64,
    /// Seconds since the beginning of the indicated one-millisecond public tick.
    pub offset_s: f64,
    /// World impulse-weighted witness position for the accepted manifold.
    pub point_m: [f64; 3],
    pub linear_impulse: [f64; 3],
    /// Device change in angular momentum about the center of mass (N m s).
    pub angular_impulse: [f64; 3],
}
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct EventLane {
    free: Lane,
    time: f64,
    arm: [f64; 3],
    restitution: f64,
}
pub(super) struct Shelf {
    surface: SpatialShelf,
    device: Solver,
    pub(super) witnesses: [ShelfWitness; 2],
    pub(super) count: usize,
    pub(super) receipt: Option<ShelfImpact>,
}
#[derive(Clone, Copy)]
struct Event {
    time: f64,
    arm: DVec3,
    state: FreeBoxState,
    second: Option<DVec3>,
}
#[derive(Clone, Copy)]
struct Shape {
    half: DVec3,
    offset: DVec3,
    inertia: DVec3,
    top: f64,
    g: f64,
}
impl Shape {
    fn new(l: &Lane, material: BoxFlightMaterial) -> Self {
        Self {
            half: DVec3::new(
                l.carry.box_half[0],
                material.half_depth,
                l.carry.box_half[1],
            ),
            offset: material.local_center(&l.carry),
            inertia: material.inertia.into(),
            top: l.carry.shelf_center[1] + l.carry.shelf_half[1],
            g: l.carry.hand.gravity,
        }
    }
    fn advance(self, mut s: FreeBoxState, dt: f64) -> FreeBoxState {
        let (p, v, q) = advance(
            s.mass_center,
            s.center_velocity,
            s.orientation,
            s.momentum,
            self.inertia.to_array(),
            self.g,
            dt,
            0.,
            0.5,
        );
        s.mass_center = p;
        s.center_velocity = v;
        s.orientation = q;
        s
    }
    fn height(self, s: FreeBoxState, arm: DVec3) -> f64 {
        s.mass_center[2] + (DQuat::from_array(s.orientation) * arm).z - self.top
    }
    fn velocity(self, s: FreeBoxState, arm: DVec3) -> f64 {
        let q = DQuat::from_array(s.orientation);
        let w = q * ((q.inverse() * DVec3::from(s.momentum)) / self.inertia);
        s.center_velocity[2] + w.cross(q * arm).z
    }
    /// Upper bound on each corner's acceleration, including angular acceleration.
    fn acceleration(self, s: FreeBoxState) -> f64 {
        let max_spin = DVec3::from(s.momentum).length() / self.inertia.min_element();
        self.g + 2. * (self.half + self.offset.abs()).length() * max_spin.powi(2)
    }
    fn corners(self) -> impl Iterator<Item = DVec3> {
        (0..8).map(move |i| {
            DVec3::from_array(std::array::from_fn(|j| {
                if i & (1 << j) == 0 {
                    -self.half[j]
                } else {
                    self.half[j]
                }
            })) - self.offset
        })
    }
    /// Certify every corner in this interval. Near a crossing the acceleration
    /// bound must prove monotonicity; a change of velocity sign near contact is ambiguous.
    fn interval(
        self,
        start: FreeBoxState,
        dt: f64,
        edges: bool,
    ) -> Result<(FreeBoxState, Option<Event>), GpuError> {
        let end = self.advance(start, dt);
        let a = self.acceleration(start);
        let mut event: Option<Event> = None;
        for arm in self.corners() {
            let h0 = self.height(start, arm);
            let h1 = self.height(end, arm);
            if h0.min(h1) > EPS + a * dt * dt / 8. {
                continue;
            }
            let speed = self.velocity(start, arm);
            if h0 >= -EPS && speed > a * dt {
                continue;
            }
            if h0 < -EPS || speed >= -a * dt {
                return Err(GpuError::SpatialContactBoundary);
            }
            if h1 > 0. {
                continue;
            }
            if event.is_some() && !edges {
                return Err(GpuError::SpatialContactBoundary);
            }
            let (mut lo, mut hi) = (0., dt);
            for _ in 0..32 {
                let mid = (lo + hi) * 0.5;
                if self.height(self.advance(start, mid), arm) > 0. {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            if let Some(other) = &mut event {
                if other.second.is_some() || (hi - other.time).abs() > TIE_SECONDS {
                    return Err(GpuError::SpatialContactBoundary);
                }
                other.second = Some(arm);
                other.time = other.time.min(hi);
                other.state = self.advance(start, other.time);
            } else {
                event = Some(Event {
                    time: hi,
                    arm,
                    state: self.advance(start, hi),
                    second: None,
                });
            }
        }
        if edges && let Some(hit) = &mut event {
            // Include a neighbor whose crossing falls just beyond the substep boundary.
            let mut pair = [DVec3::ZERO; 2];
            let mut count = 0;
            for arm in self.corners() {
                let gap = self.height(hit.state, arm).abs();
                let approach = -self.velocity(hit.state, arm);
                if gap <= EPS && approach > 0. && gap <= approach * TIE_SECONDS {
                    if count == 2 {
                        return Err(GpuError::SpatialContactBoundary);
                    }
                    pair[count] = arm;
                    count += 1;
                }
            }
            if count == 0 || (hit.second.is_some() && count != 2) {
                return Err(GpuError::SpatialContactBoundary);
            }
            if count == 2 {
                // Only two adjacent vertices form an admitted edge.
                if (0..3).filter(|&i| pair[0][i] != pair[1][i]).count() != 1 {
                    return Err(GpuError::SpatialContactBoundary);
                }
                hit.arm = pair[0];
                hit.second = Some(pair[1]);
            }
        }
        Ok((end, event))
    }
    fn classify(
        self,
        start: FreeBoxState,
        l: &Lane,
        surface: SpatialShelf,
    ) -> Result<bool, GpuError> {
        let q = DQuat::from_array(start.orientation);
        let origin = DVec3::from(start.mass_center) - q * self.offset;
        let rot = DMat3::from_quat(q);
        let extent = rot.x_axis.abs() * self.half.x
            + rot.y_axis.abs() * self.half.y
            + rot.z_axis.abs() * self.half.z;
        // The entire body stays inside this swept box even if it rebounds.
        // A static frictionless Newton impact with e <= 1 cannot add kinetic energy.
        let v = DVec3::from(start.center_velocity);
        let angular = q.inverse() * DVec3::from(start.momentum);
        let mass = l.carry.hand.ball_mass;
        let speed = (v.length_squared() + angular.dot(angular / self.inertia) / mass).sqrt()
            + self.g * 4. * DT;
        let radius = (self.half + self.offset.abs()).length();
        let travel = 4. * DT * speed * (1. + radius * (mass / self.inertia.min_element()).sqrt());
        let lo = origin - extent - DVec3::splat(travel);
        let hi = origin + extent + DVec3::splat(travel);
        if lo.z > self.top
            || hi.x < l.carry.shelf_center[0] - l.carry.shelf_half[0]
            || lo.x > l.carry.shelf_center[0] + l.carry.shelf_half[0]
            || hi.y < surface.center_y - surface.half_depth
            || lo.y > surface.center_y + surface.half_depth
        {
            return Ok(false);
        }
        if lo.x <= l.carry.shelf_center[0] - l.carry.shelf_half[0]
            || hi.x >= l.carry.shelf_center[0] + l.carry.shelf_half[0]
            || lo.y <= surface.center_y - surface.half_depth
            || hi.y >= surface.center_y + surface.half_depth
        {
            return Err(GpuError::SpatialContactBoundary);
        }
        Ok(true)
    }
}
impl Shelf {
    pub(super) fn new(f: &BoxFlight, surface: SpatialShelf, edges: bool) -> Result<Self, GpuError> {
        let l = f.device.committed[0];
        if !surface.center_y.is_finite()
            || !surface.half_depth.is_finite()
            || surface.half_depth <= 0.
            || !(0. ..=1.).contains(&surface.restitution)
            || l.carry.shelf_friction != 0.
        {
            return Err(GpuError::InvalidBoxRelease);
        }
        let initial = EventLane {
            free: l,
            time: 0.,
            arm: [0.; 3],
            restitution: surface.restitution,
        };
        let device = Solver::new(f, initial, edges)?;
        Ok(Self {
            device,
            surface,
            receipt: None,
            witnesses: Default::default(),
            count: 0,
        })
    }
    pub(super) fn allocated(&self) -> u64 {
        self.device.allocated()
    }
    pub(super) fn reset(&mut self) {
        self.receipt = None;
        self.count = 0;
        self.device.reset();
    }
    pub(super) fn step(
        &mut self,
        free: &mut ResidentBatch<Lane>,
        material: BoxFlightMaterial,
    ) -> Result<crate::StepMetrics, GpuError> {
        let start = free.committed[0];
        let shape = Shape::new(&start, material);
        let mut next = start.body;
        let mut hit = None;
        if shape.classify(next, &start, self.surface)? {
            for i in 0..4 {
                let (end, event) =
                    shape.interval(next, DT, matches!(self.device, Solver::Edge(_)))?;
                if let Some(mut event) = event {
                    if self.receipt.is_some() {
                        return Err(GpuError::SpatialContactBoundary);
                    }
                    event.time += i as f64 * DT;
                    hit = Some(event);
                    break;
                }
                next = end;
            }
        }
        let Some(event) = hit else {
            return free.advance_checked(1, |rows| {
                validate_others(material, &start.body, &rows[0], false)
            });
        };
        if let Some(second) = event.second
            && !edge::well_conditioned(
                event.state,
                [event.arm, second],
                shape.inertia,
                start.carry.hand.ball_mass,
            )
        {
            return Err(GpuError::SpatialContactBoundary);
        }
        let input = EventLane {
            free: start,
            time: event.time,
            arm: event.arm.to_array(),
            restitution: self.surface.restitution,
        };
        let (metrics, out) = self.device.step(free.tick, input, event.second, |out| {
            validate_others(material, &start.body, &out, true)?;
            // Check the remaining fraction and all following substeps for secondary contacts.
            let [first, second] = out.carry.point_impulse[3];
            if first < 0. || second < 0. || first + second <= 0. {
                return Err(GpuError::SpatialContactBoundary);
            }
            let q = DQuat::from_array(event.state.orientation);
            let mut post = event.state;
            post.center_velocity[2] += (first + second) / start.carry.hand.ball_mass;
            let torque = (q * event.arm).cross(DVec3::Z * first)
                + (q * event.second.unwrap_or(DVec3::ZERO)).cross(DVec3::Z * second);
            post.momentum = (DVec3::from(post.momentum) + torque).to_array();
            let mut t = event.time;
            for i in 1..=4 {
                let end = i as f64 * DT;
                if end <= t {
                    continue;
                }
                let (advanced, other) = shape.interval(post, end - t, false)?;
                if other.is_some() {
                    return Err(GpuError::SpatialContactBoundary);
                }
                post = advanced;
                t = end;
            }
            Ok(())
        })?;
        let j = DVec3::Z * out.carry.shelf_normal;
        let mut weighted = DVec3::ZERO;
        let q = DQuat::from_array(event.state.orientation);
        self.count = if event.second.is_some() { 2 } else { 1 };
        for (i, arm) in std::iter::once(event.arm).chain(event.second).enumerate() {
            let p = DVec3::from(event.state.mass_center) + q * arm;
            let impulse = out.carry.point_impulse[3][i];
            self.witnesses[i] = ShelfWitness {
                point_m: p.to_array(),
                normal_impulse: impulse,
            };
            weighted += p * (impulse / j.z);
        }
        self.receipt = Some(ShelfImpact {
            tick: free.tick + 1,
            offset_s: event.time,
            point_m: weighted.to_array(),
            linear_impulse: j.to_array(),
            angular_impulse: (DVec3::from(out.body.momentum) - DVec3::from(start.body.momentum))
                .to_array(),
        });
        free.committed[0] = out;
        free.tick += 1;
        free.upload_needed = true;
        Ok(metrics)
    }
}
fn validate_others(
    material: BoxFlightMaterial,
    start: &FreeBoxState,
    out: &Lane,
    impact: bool,
) -> Result<(), GpuError> {
    if clear_environment(material, start, out, true, impact) {
        Ok(())
    } else {
        Err(GpuError::SpatialContactBoundary)
    }
}
fn module<const N: usize, const EDGE: bool>() -> naga::Module {
    kernel::build(&constants(), |v: [_; N], c| {
        let (hand, swept) = unloaded_hand(std::array::from_fn(|i| v[i]), c);
        let mut p = std::array::from_fn(|i| v[WIDTH + i]);
        let mut vel = std::array::from_fn(|i| v[WIDTH + 3 + i]);
        let mut q = std::array::from_fn(|i| v[WIDTH + 6 + i]);
        let mut l = std::array::from_fn(|i| v[WIDTH + 10 + i]);
        let inertia = std::array::from_fn(|i| v[WIDTH + 13 + i]);
        let tau = v[WORDS];
        let arm = std::array::from_fn(|i| v[WORDS + 1 + i]);
        let mut t = c[1];
        for _ in 0..4 {
            let dt = (tau - t).maximum(c[1]).minimum(c[0]);
            (p, vel, q) = advance(p, vel, q, l, inertia, v[21], dt, c[1], c[3]);
            t = t + c[0];
        }
        let r = rotated(q, arm);
        let w = angular(q, l, inertia, c[1]);
        let row = [r[1], c[1] - r[0], c[1]];
        let effective = c[2] / v[20] + dot(row, angular(q, row, inertia, c[1]));
        let factor = c[1] - (c[2] + v[WORDS + 4]);
        let desired = factor * (vel[2] + cross(w, r)[2]);
        let single = desired / effective;
        let (first, second, other) = if EDGE {
            let second_arm = rotated(q, std::array::from_fn(|i| v[WORDS + 5 + i]));
            let other = [second_arm[1], c[1] - second_arm[0], c[1]];
            let delta = angular(q, other, inertia, c[1]);
            let coupling = c[2] / v[20] + dot(row, delta);
            let other_weight = c[2] / v[20] + dot(other, delta);
            let rhs = factor * (vel[2] + cross(w, second_arm)[2]);
            let [j, k] = edge::pair_impulses(
                [effective, other_weight],
                coupling,
                [desired, rhs],
                c[1],
                c[2],
            );
            let paired = v[WORDS + 8].greater(c[1]);
            (paired.select(j, single), paired.select(k, c[1]), other)
        } else {
            (single, c[1], [c[1]; 3])
        };
        let impulse = first + second;
        vel[2] = vel[2] + impulse / v[20];
        l = std::array::from_fn(|i| l[i] + row[i] * first + other[i] * second);
        t = c[0];
        for _ in 0..4 {
            let dt = (t - tau).maximum(c[1]).minimum(c[0]);
            (p, vel, q) = advance(p, vel, q, l, inertia, v[21], dt, c[1], c[3]);
            t = t + c[0];
        }
        std::array::from_fn::<_, 52, _>(|i| match i {
            0..33 => match hand[i].0 {
                67 | 69 => (hand[i].0, impulse),
                82 => (82, first),
                83 => (83, second),
                _ => hand[i],
            },
            33..36 => (WIDTH + i - 33, p[i - 33]),
            36..39 => (WIDTH + i - 33, vel[i - 36]),
            39..43 => (WIDTH + i - 33, q[i - 39]),
            43..46 => (WIDTH + i - 33, l[i - 43]),
            _ => (WIDTH + 16 + i - 46, swept[(i - 46) / 2][(i - 46) % 2]),
        })
    })
}

#[cfg(test)]
#[path = "../../../tests/private/shelf_corner.rs"]
mod tests;
