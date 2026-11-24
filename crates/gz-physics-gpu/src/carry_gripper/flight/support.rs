// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Six-degree-of-freedom box contact on the interior of a finite, static shelf top.
use super::*;
use crate::arithmetic::Condition;
use crate::device::Dispatch;
use crate::spatial::dot;
const SUBSTEPS: usize = 8;
const SWEEPS: usize = 8;
type V<T> = [T; 3];

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Pod, Zeroable)]
pub struct SpatialShelfLoads {
    /// Signed world impulses on material vertices in N s. Corner index bits select
    /// positive local X, Y and Z, respectively. Each vertex has a unilateral normal.
    pub world_impulses: [[f64; 3]; 8],
    /// Total world angular impulse about the moving mass center, in N m s.
    pub angular_impulse: [f64; 3],
    /// Work by the normal and tangential impulses, using each substep's midpoint twist (J).
    pub contact_work: [f64; 2],
    /// Largest positive change in kinetic energy caused by one contact solve (J).
    pub max_added_energy: f64,
}
#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub(super) struct ContactLane {
    lane: Lane,
    half: [f64; 3],
    center: [f64; 3],
    loads: SpatialShelfLoads,
    current: [[f64; 3]; 8],
    incoming_twist_momentum: [f64; 6],
}
const LOAD: usize = WORDS + 6;
const TORQUE: usize = LOAD + 24;
const WORK: usize = TORQUE + 3;
const CURRENT: usize = WORK + 3;
const BEFORE: usize = CURRENT + 24;
const N: usize = BEFORE + 6;
const _: () = assert!(size_of::<ContactLane>() == 8 * N);

pub(super) struct Support {
    pub(super) device: ResidentBatch<ContactLane>,
    passes: Vec<Dispatch>,
    surface: SpatialShelf,
    pub(super) latest: Option<SpatialShelfLoads>,
}
impl Support {
    pub(super) fn new(f: &BoxFlight, surface: SpatialShelf) -> Result<Self, GpuError> {
        if surface.restitution != 0.
            || !surface.half_depth.is_finite()
            || surface.half_depth <= 0.
            || !surface.center_y.is_finite()
        {
            return Err(GpuError::InvalidBoxRelease);
        }
        let input = ContactLane::input(f.device.lanes()[0], f.material);
        let gpu = f.device.gpu.companion::<ContactLane>(1)?;
        let device = ResidentBatch::on_device(gpu, vec![input], hand, None)?;
        let pass = |module| {
            device.gpu.dispatch(
                module,
                0,
                device.device_info().state_bytes,
                size_of::<ContactLane>() as u64,
                1,
                None,
            )
        };
        let predict = pass(predict)?;
        let constraints = pass(solve)?;
        let finish = pass(finish)?;
        let mut passes = vec![device.dispatch.clone()];
        for _ in 0..SUBSTEPS {
            passes.push(predict.clone());
            passes.push(constraints.clone());
            passes.push(finish.clone());
        }
        Ok(Self {
            device,
            passes,
            surface,
            latest: None,
        })
    }
    pub(super) fn step(
        &mut self,
        free: &mut ResidentBatch<Lane>,
        mat: BoxFlightMaterial,
    ) -> Result<crate::StepMetrics, GpuError> {
        let start = free.committed[0];
        if !shelf::Shape::new(&start, mat).classify(start.body, &start, self.surface)? {
            let met = free.advance_checked(1, |rows| {
                shelf::validate_others(mat, &start.body, &rows[0], false)
            })?;
            self.latest = None;
            return Ok(met);
        }
        self.device.committed[0] = ContactLane::input(start, mat);
        self.device.tick = free.tick;
        self.device.upload_needed = true;
        let metrics = self.device.advance_sequence(1, Some(&self.passes), |r| {
            // The swept bounds assume a passive shelf. Refuse a non-dissipative
            // numerical solve before publishing its body or advancing the clock.
            if r[0].loads.max_added_energy > 1e-12 {
                return Err(GpuError::SpatialContactBoundary);
            }
            shelf::validate_others(mat, &start.body, &r[0].lane, true)
        })?;
        let output = self.device.committed[0];
        let mut lane = output.lane;
        let mut ends = [0.; 2];
        let mut tangent = 0.;
        for (i, j) in output.loads.world_impulses.iter().enumerate() {
            ends[i & 1] += j[2];
            tangent += j[0];
        }
        lane.carry.shelf_up = ends.iter().sum();
        lane.carry.shelf_normal = lane.carry.shelf_up;
        lane.carry.shelf_tangent = tangent;
        lane.carry.point_impulse[3] = ends;
        free.committed[0] = lane;
        free.tick = self.device.tick;
        free.upload_needed = true;
        self.latest = Some(output.loads);
        Ok(metrics)
    }
}
impl ContactLane {
    fn input(lane: Lane, m: BoxFlightMaterial) -> Self {
        Self {
            half: [lane.carry.box_half[0], m.half_depth, lane.carry.box_half[1]],
            center: m.local_center(&lane.carry).to_array(),
            lane,
            loads: SpatialShelfLoads::default(),
            current: [[0.; 3]; 8],
            incoming_twist_momentum: [0.; 6],
        }
    }
}
fn hand() -> naga::Module {
    kernel::build(&constants(), |v: [_; N], c| {
        let (h, s) = unloaded_hand(std::array::from_fn(|i| v[i]), c);
        std::array::from_fn::<_, 39, _>(|i| {
            if i < 33 {
                h[i]
            } else {
                (WIDTH + 16 + i - 33, s[(i - 33) / 2][(i - 33) % 2])
            }
        })
    })
}
fn at<T: Copy, const K: usize>(v: &[T], i: usize) -> [T; K] {
    std::array::from_fn(|j| v[i + j])
}
fn predict() -> naga::Module {
    kernel::build(&[0.001 / SUBSTEPS as f64, 0., 0.5], |v: [_; N], c| {
        let (p, vel, q) = advance(
            at(&v, WIDTH),
            at(&v, WIDTH + 3),
            at(&v, WIDTH + 6),
            at(&v, WIDTH + 10),
            at(&v, WIDTH + 13),
            v[21],
            c[0],
            c[1],
            c[2],
        );
        std::array::from_fn::<_, 40, _>(|i| match i {
            0..3 => (WIDTH + i, p[i]),
            3..6 => (WIDTH + i, vel[i - 3]),
            6..10 => (WIDTH + i, q[i - 6]),
            10..34 => (CURRENT + i - 10, c[1]),
            34..37 => (BEFORE + i - 34, vel[i - 34]),
            _ => (BEFORE + i - 34, v[WIDTH + 10 + i - 37]),
        })
    })
}
fn arm<T: Scalar>(v: &[T; N], index: usize, zero: T) -> V<T> {
    let local = std::array::from_fn(|axis| {
        let h = v[WORDS + axis];
        (if index & (1 << axis) == 0 {
            zero - h
        } else {
            h
        }) - v[WORDS + 3 + axis]
    });
    rotated(at(v, WIDTH + 6), local)
}
fn add<T: Scalar>(a: V<T>, b: V<T>) -> V<T> {
    std::array::from_fn(|i| a[i] + b[i])
}
fn scale<T: Scalar>(a: V<T>, f: T) -> V<T> {
    a.map(|v| v * f)
}
fn solve() -> naga::Module {
    kernel::build_fixed_loop(
        &[0., 1., 1e-7, 1e-30, 0.5, 1e-10],
        (SWEEPS / 2) as u32,
        |mut v: [_; N], c| {
            for (slot, value) in sweep::<false, _>(&v, c) {
                v[slot] = value;
            }
            sweep::<true, _>(&v, c)
        },
    )
}
fn sweep<const REVERSE: bool, T: Scalar>(v: &[T; N], c: &[T]) -> [(usize, T); 30] {
    let [zero, one, skin, tiny, half] = [c[0], c[1], c[2], c[3], c[4]];
    let q = at(v, WIDTH + 6);
    let inertia = at(v, WIDTH + 13);
    let inv_m = one / v[20];
    let top = v[63] + v[65];
    let mut velocity = at(v, WIDTH + 3);
    let mut momentum = at(v, WIDTH + 10);
    let mut loads: [[_; 3]; 8] = std::array::from_fn(|i| at(v, CURRENT + 3 * i));
    // The two endpoints of an edge across the transverse axis see the same incoming
    // velocity. A simultaneous, relaxed pair update preserves reflection symmetry.
    // Other edges use Gauss–Seidel updates, with alternating traversal.
    for k in 0..4 {
        let edge = if REVERSE { 3 - k } else { k };
        let first = (edge & 1) + ((edge & 2) << 1);
        let pair = [first, first + 2];
        let arms = pair.map(|i| arm(v, i, zero));
        let active = arms.map(|r| (v[WIDTH + 2] + r[2] - top).at_most(skin));
        let omega = angular(q, momentum, inertia, zero);
        let axis = [zero, zero, one];
        let tau = arms.map(|r| cross(r, axis));
        let dw = tau.map(|a| angular(q, a, inertia, zero));
        let k = [inv_m + dot(tau[0], dw[0]), inv_m + dot(tau[1], dw[1])];
        let coupling = inv_m + dot(tau[0], dw[1]);
        let old = pair.map(|i| loads[i][2]);
        let speeds = arms.map(|r| velocity[2] + cross(omega, r)[2]);
        let rhs = std::array::from_fn(|j| k[j] * old[j] + coupling * old[1 - j] - speeds[j]);
        let paired = active[0].and(active[1]);
        let regular = (k[0] * k[1] - coupling * coupling).greater(tiny + k[0] * k[1] * c[5]);
        let block = shelf::edge::pair_impulses(k, coupling, rhs, zero, one);
        for j in 0..2 {
            let solo = (old[j] - speeds[j] / k[j]).maximum(zero);
            let fallback = paired.select(half * (solo + old[j]), solo);
            let normal = active[j].select(paired.and(regular).select(block[j], fallback), zero);
            let change = normal - old[j];
            loads[pair[j]][2] = normal;
            velocity[2] = velocity[2] + change * inv_m;
            momentum = add(momentum, scale(tau[j], change));
        }
        let post_normal = angular(q, momentum, inertia, zero);
        let relax = paired.select(half, one);
        let mut impulses = [[zero; 3]; 2];
        for j in 0..2 {
            let i = pair[j];
            let r = arms[j];
            let point = add(velocity, cross(post_normal, r));
            let axes = [[one, zero, zero], [zero, one, zero]].map(|a| cross(r, a));
            let dw = axes.map(|a| angular(q, a, inertia, zero));
            let kx = inv_m + dot(axes[0], dw[0]);
            let ky = inv_m + dot(axes[1], dw[1]);
            let kxy = dot(axes[0], dw[1]);
            // Rotation-invariant gradient step using the largest tangent inverse
            // effective mass. At slip its fixed point lies opposite the relative motion.
            let twice = one + one;
            let rate =
                twice / (kx + ky + ((kx - ky) * (kx - ky) + (twice * kxy) * (twice * kxy)).root());
            let trial = [loads[i][0] - rate * point[0], loads[i][1] - rate * point[1]];
            let norm = (trial[0] * trial[0] + trial[1] * trial[1]).root();
            let factor = (v[66] * loads[i][2] / norm.maximum(tiny)).minimum(one);
            // Apply a convex combination of complete feasible impulses, or reproject
            // if this normal pass unloaded a point and shrank its friction disk.
            let mut next = std::array::from_fn::<_, 2, _>(|axis| {
                loads[i][axis] + relax * (trial[axis] * factor - loads[i][axis])
            });
            let n = (next[0] * next[0] + next[1] * next[1]).root();
            let cap = (v[66] * loads[i][2] / n.maximum(tiny)).minimum(one);
            for axis in 0..2 {
                next[axis] = next[axis] * cap;
                impulses[j][axis] = next[axis] - loads[i][axis];
                loads[i][axis] = next[axis];
            }
        }
        velocity = add(velocity, scale(add(impulses[0], impulses[1]), inv_m));
        momentum = add(
            momentum,
            add(cross(arms[0], impulses[0]), cross(arms[1], impulses[1])),
        );
    }
    std::array::from_fn::<_, 30, _>(|i| match i {
        0..3 => (WIDTH + 3 + i, velocity[i]),
        3..6 => (WIDTH + 10 + i - 3, momentum[i - 3]),
        _ => (CURRENT + i - 6, loads[(i - 6) / 3][(i - 6) % 3]),
    })
}
fn finish() -> naga::Module {
    kernel::build(&[0., 0.5], |v: [_; N], c| {
        let z = c[0];
        let top = v[63] + v[65];
        let mut correction = z;
        let mut torque = at(&v, TORQUE);
        let mut work = [z; 2];
        let velocity = scale(add(at(&v, BEFORE), at(&v, WIDTH + 3)), c[1]);
        let l = scale(add(at(&v, BEFORE + 3), at(&v, WIDTH + 10)), c[1]);
        let omega = angular(at(&v, WIDTH + 6), l, at(&v, WIDTH + 13), z);
        for i in 0..8 {
            let r = arm(&v, i, z);
            correction = correction.maximum(top - v[WIDTH + 2] - r[2]);
            let j = at(&v, CURRENT + 3 * i);
            torque = add(torque, cross(r, j));
            let speed = add(velocity, cross(omega, r));
            work[0] = work[0] + j[2] * speed[2];
            work[1] = work[1] + j[0] * speed[0] + j[1] * speed[1];
        }
        std::array::from_fn::<_, 31, _>(|i| match i {
            0..24 => (LOAD + i, v[LOAD + i] + v[CURRENT + i]),
            24..27 => (TORQUE + i - 24, torque[i - 24]),
            27..29 => (WORK + i - 27, v[WORK + i - 27] + work[i - 27]),
            29 => (WORK + 2, v[WORK + 2].maximum(work[0] + work[1])),
            _ => (WIDTH + 2, v[WIDTH + 2] + correction),
        })
    })
}
