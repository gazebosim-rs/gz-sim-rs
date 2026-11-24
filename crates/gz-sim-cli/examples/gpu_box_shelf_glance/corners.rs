// Copyright (C) 2026 Open Source Robotics Foundation
// SPDX-License-Identifier: Apache-2.0
//! Resolve both ends of the glancing edge using Rapier's actual active material points.
use super::*;
use rapier3d_f64::{geometry::ContactData, utils::OrthonormalBasis};

const MATCHED: &str = include_str!(
    "../../../../docs/measurements/local-manipulation/gpu-shelf-glance-mirrored-states.json"
);
#[derive(Deserialize)]
struct Case {
    gpu: Pair,
    rapier: Pair,
    gpu_contact: DeviceContact,
}
#[derive(Deserialize)]
struct DeviceContact {
    vertices: [[f64; 3]; 8],
    angular: [f64; 3],
}
#[derive(Default)]
pub(super) struct Witnesses {
    impulses: [Vector; 8],
    torques: [Vector; 8],
    first_loaded: [Option<usize>; 8],
    peak_cone_excess: f64,
    max_witness_offset: f64,
    arriving: usize,
    pub(super) incoming: (Rotation, Vector, Vector),
    first_state: Option<(Rotation, Vector, Vector)>,
    first_burst: Vector,
    burst_steps: usize,
}
impl Witnesses {
    pub(super) fn record(
        &mut self,
        step: usize,
        local: Vector,
        arm: Vector,
        normal: Vector,
        sign: f64,
        data: &ContactData,
    ) {
        // Rapier writes the accumulated tangential impulse in the solver's fixed
        // basis on body one. The box can be either member of the contact pair.
        let axis = -normal;
        let t = axis.orthonormal_vector();
        let shear = data.tangent_impulse;
        let impulse = -sign * (axis * data.impulse + t * shear.x + axis.cross(t) * shear.y);
        let i = usize::from(local.x > 0.)
            + 2 * usize::from(local.y > 0.)
            + 4 * usize::from(local.z > 0.);
        let nearest = Vector::new(
            0.024 * local.x.signum(),
            0.012 * local.y.signum(),
            0.020 * local.z.signum(),
        );
        self.max_witness_offset = self.max_witness_offset.max(local.distance(nearest));
        self.peak_cone_excess = self
            .peak_cone_excess
            .max(impulse.x.hypot(impulse.y) - 0.45 * impulse.z);
        self.impulses[i] += impulse;
        self.torques[i] += arm.cross(impulse);
        if data.impulse > 1e-14 {
            self.first_loaded[i].get_or_insert(step);
            if i == self.arriving {
                self.first_state.get_or_insert(self.incoming);
            }
        }
        if i == self.arriving
            && self.first_loaded[i].is_some_and(|first| step - first < self.burst_steps)
        {
            self.first_burst += impulse;
        }
    }
}
fn plastic(
    (q, velocity, omega): (Rotation, Vector, Vector),
    held: usize,
    arriving: usize,
) -> Value {
    let arm = |i: usize| {
        q * (Vector::new(
            if i & 1 != 0 { 0.024 } else { -0.024 },
            if i & 2 != 0 { 0.012 } else { -0.012 },
            -0.020,
        ) - OFFSET)
    };
    let inv_i = |torque: Vector| {
        q * ((q.inverse() * torque)
            / Vector::new(0.00001450666666666667, 0.00002602666666667, 0.0000192))
    };
    let r = arm(arriving);
    let local_velocity = velocity + omega.cross(r);
    let response = |impulse: Vector| impulse / MASS + inv_i(r.cross(impulse)).cross(r);
    let [x, y, z] = [Vector::X, Vector::Y, Vector::Z].map(response);
    // Eliminate the three impulse components for a trial slip multiplier.
    // The last row enforces zero separating speed at the plastic impact.
    let impulse = |slip: f64| {
        let x = x + slip * Vector::X;
        let y = y + slip * Vector::Y;
        let rhs = -local_velocity;
        Vector::new(
            rhs.dot(y.cross(z)),
            x.dot(rhs.cross(z)),
            x.dot(y.cross(rhs)),
        ) / x.dot(y.cross(z))
    };
    let excess = |j: Vector| j.x.hypot(j.y) - 0.45 * j.z;
    let mut lo = 0.;
    let mut hi = 1.;
    assert!(excess(impulse(lo)) > 0.);
    while excess(impulse(hi)) > 0. {
        hi *= 2.;
    }
    for _ in 0..80 {
        let m = 0.5 * (lo + hi);
        if excess(impulse(m)) > 0. {
            lo = m;
        } else {
            hi = m;
        }
    }
    let j = impulse(hi);
    let a = r.cross(j);
    let v = velocity + j / MASS;
    let w = omega + inv_i(a);
    let after = v + w.cross(r);
    let old_after = v + w.cross(arm(held));
    let work = j.dot((local_velocity + after) * 0.5);
    assert!(
        work < 0.
            && j.z > 0.
            && after.z.abs() < 1e-12
            && excess(j).abs() < 1e-12
            && old_after.z > 0.
    );
    json!({"linear_ns":j.to_array(),"angular_nms":a.to_array(),"incoming_point_m_s":local_velocity.to_array(),"outgoing_point_m_s":after.to_array(),"departing_old_corner_normal_m_s":old_after.z,"work_j":work,"outgoing_yaw_rad_s":w.z})
}
fn vector(s: &Value) -> Vector {
    Vector::from_array(serde_json::from_value(s.clone()).unwrap())
}
fn momentum(s: &State) -> Vector {
    let q = Rotation::from_array(s.orientation);
    q * (MOMENTS * (q.inverse() * Vector::from_array(s.omega)))
}
fn signed_balance(a: &Pair) -> (Vector, Vector) {
    let x = &a.incoming;
    let y = &a.continuous_outgoing;
    (
        MASS * (Vector::from_array(y.center_velocity) - Vector::from_array(x.center_velocity)
            + DT * 9.81 * Vector::Z),
        momentum(y) - momentum(x),
    )
}
pub(super) fn run() -> Result<(), Box<dyn std::error::Error>> {
    let cases: BTreeMap<String, Case> = serde_json::from_str(MATCHED)?;
    let mut out = BTreeMap::new();
    for (name, case) in cases {
        let mut records = BTreeMap::new();
        let (gpu_j, gpu_a) = signed_balance(&case.gpu);
        let device_sum: Vector = case
            .gpu_contact
            .vertices
            .iter()
            .copied()
            .map(Vector::from_array)
            .sum();
        assert!(
            device_sum.distance(gpu_j) < 1e-11
                && Vector::from_array(case.gpu_contact.angular).distance(gpu_a) < 1e-11
        );
        let (held, arriving) = if name == "oblique_plus" {
            (0, 1)
        } else {
            (2, 3)
        };
        for (label, pair) in [("gpu", &case.gpu), ("rapier", &case.rapier)] {
            let mut trials = Vec::new();
            for steps in [
                32_usize, 128, 512, 1024, 2048, 4096, 8192, 16384, 32768, 65536,
            ] {
                let mut w = Witnesses {
                    arriving,
                    burst_steps: steps.div_ceil(1000),
                    ..Default::default()
                };
                let native = solve(&pair.incoming, steps, false, Some(&mut w));
                let linear: Vector = w.impulses.iter().copied().sum();
                let angular: Vector = w.torques.iter().copied().sum();
                assert!(w.first_loaded[held].is_some() && w.first_loaded[arriving].is_some());
                let linear_error = linear.distance(vector(&native["linear_impulse_ns"]));
                let angular_error = angular.distance(vector(&native["angular_impulse_nms"]));
                assert!(linear_error < 2e-12 && angular_error < 2e-8);
                assert!(w.peak_cone_excess < 1e-16 && w.max_witness_offset < 1e-12);
                trials.push(json!({"substeps":steps,"already_supported_corner":held,"arriving_corner":arriving,"corner_impulses_ns":w.impulses.map(|v|v.to_array()),"corner_angular_impulses_nms":w.torques.map(|v|v.to_array()),"first_loaded_substeps":w.first_loaded,"linear_residual_ns":linear_error,"angular_residual_nms":angular_error,"max_shear_cone_excess_ns":w.peak_cone_excess,"max_local_witness_offset_m":w.max_witness_offset,"first_microsecond_ns":w.first_burst.to_array(),"plastic_point":plastic(w.first_state.unwrap(),held,arriving),"native":native}));
            }
            records.insert(label, trials);
        }
        out.insert(name,json!({"gpu_linear_impulse_ns":gpu_j.to_array(),"gpu_angular_impulse_nms":gpu_a.to_array(),"gpu_corner_impulses_ns":case.gpu_contact.vertices,"controls":records}));
    }
    println!("{}", serde_json::to_string_pretty(&out)?);
    Ok(())
}
